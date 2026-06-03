#include "circle/perception/camera_info_loader.hpp"

#include <fstream>
#include <regex>

namespace circle::perception {

namespace {

bool parseMatrixData(const std::string& yaml, const std::string& key,
                     float out[9]) {
  const std::string needle = key + ":";
  const auto pos = yaml.find(needle);
  if (pos == std::string::npos) {
    return false;
  }
  const auto data_pos = yaml.find("data:", pos);
  if (data_pos == std::string::npos) {
    return false;
  }

  std::regex num_re(R"([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)");
  int i = 0;

  const auto bracket = yaml.find('[', data_pos);
  if (bracket != std::string::npos) {
    const auto end = yaml.find(']', bracket);
    if (end == std::string::npos) {
      return false;
    }
    const std::string slice = yaml.substr(bracket + 1, end - bracket - 1);
    auto begin = std::sregex_iterator(slice.begin(), slice.end(), num_re);
    const auto end_it = std::sregex_iterator();
    for (; begin != end_it && i < 9; ++begin, ++i) {
      out[i] = std::stof((*begin).str());
    }
    return i >= 4;
  }

  // OpenCV camera_info YAML list form:
  //   data:
  //   - fx
  //   - 0
  //   ...
  size_t line = yaml.find('\n', data_pos);
  while (line != std::string::npos && i < 9) {
    const size_t next = yaml.find('\n', line + 1);
    const std::string row =
        yaml.substr(line + 1, next == std::string::npos ? std::string::npos
                                                      : next - line - 1);
    line = next;
    const auto dash = row.find('-');
    if (dash == std::string::npos) {
      if (i >= 4) {
        break;
      }
      continue;
    }
    const std::string tail = row.substr(dash + 1);
    auto begin = std::sregex_iterator(tail.begin(), tail.end(), num_re);
    if (begin == std::sregex_iterator()) {
      if (i >= 4) {
        break;
      }
      continue;
    }
    out[i++] = std::stof((*begin).str());
  }
  return i >= 4;
}

int parseIntField(const std::string& yaml, const std::string& key) {
  const std::regex re(key + R"(\s*:\s*([0-9]+))");
  std::smatch m;
  if (std::regex_search(yaml, m, re) && m.size() > 1) {
    return std::stoi(m[1].str());
  }
  return 0;
}

}  // namespace

bool loadCameraIntrinsicsFromYaml(const std::string& path,
                                  circle::types::CameraIntrinsics& out,
                                  int* out_width,
                                  int* out_height) {
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  const std::string yaml((std::istreambuf_iterator<char>(in)),
                         std::istreambuf_iterator<char>());

  float k[9]{};
  if (!parseMatrixData(yaml, "camera_matrix", k)) {
    return false;
  }
  out.fx = k[0];
  out.fy = k[4];
  out.cx = k[2];
  out.cy = k[5];

  if (out_width) {
    *out_width = parseIntField(yaml, "image_width");
  }
  if (out_height) {
    *out_height = parseIntField(yaml, "image_height");
  }
  return out.fx > 0.0F && out.fy > 0.0F;
}

}  // namespace circle::perception
