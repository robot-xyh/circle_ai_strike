#include <atomic>
#include <algorithm>
#include <chrono>
#include <cctype>
#include <csignal>
#include <cstdio>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <sys/socket.h>

#include "circle/bf/logger.hpp"
#include "circle/debug/debug_preview_reader.hpp"
#include "circle/debug/debug_video_server.hpp"
#include "circle/debug/preview_overlay.hpp"
#include "circle/debug/webrtc_h264_sender.hpp"
#include "circle/debug_common/strike_param_tune.hpp"
#include "circle/debug_common/strike_png_param_tune.hpp"
#include "circle/debug_common/telemetry_schema.hpp"
#include "circle/ipc/param_block_shm.hpp"
#include "circle/ipc/shm_contract.hpp"
#include "circle/ipc/strike_params_snapshot_shm.hpp"
#include "circle/ipc/strike_telemetry_shm.hpp"
#include "circle/strike/strike_params_yaml.hpp"
#include "circle/strike_png/strike_png_params_yaml.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
#include <yaml-cpp/yaml.h>
#endif

namespace {

std::atomic<bool> g_running{true};

void onSignal(int) { g_running.store(false); }

struct DebugImageSaveConfig {
  bool enabled{false};
  std::string dir{"logs_ws/debug_frames"};
  int jpeg_quality{85};
  uint32_t max_files{2000};
  bool save_annotated{true};
  bool save_raw{false};
  double interval_s{0.5};
  bool on_interval{false};
  bool on_detection{false};
  bool on_override{false};
  bool on_raw0{false};
};

/** Saves preview SHM frames to disk as JPEG.
 *  bf_flight publishes raw pixels + overlay metadata; bf_debugd draws overlay for
 *  streaming. save_raw writes the pre-overlay frame; save_annotated the drawn one.
 *  Trigger conditions come from strike telemetry SHM. */
class DebugImageSaver {
 public:
  explicit DebugImageSaver(DebugImageSaveConfig cfg) : cfg_(std::move(cfg)) {
    namespace fs = std::filesystem;
    std::error_code ec;
    if (cfg_.save_annotated) {
      annotated_dir_ = fs::path(cfg_.dir) / "annotated";
      fs::create_directories(annotated_dir_, ec);
    }
    if (cfg_.save_raw) {
      raw_dir_ = fs::path(cfg_.dir) / "raw";
      fs::create_directories(raw_dir_, ec);
    }
    jpeg_params_ = {cv::IMWRITE_JPEG_QUALITY, std::clamp(cfg_.jpeg_quality, 1, 100)};
    interval_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(std::max(0.0, cfg_.interval_s)));
  }

  bool active() const {
    return (cfg_.save_annotated || cfg_.save_raw) &&
           (cfg_.on_interval || cfg_.on_detection || cfg_.on_override ||
            cfg_.on_raw0);
  }

  bool wantsRaw() const { return cfg_.save_raw; }

  bool shouldSave(const circle::ipc::StrikeTelemetrySample& tel,
                  std::chrono::steady_clock::time_point now,
                  std::string* tag) {
    const bool det_valid = tel.detection_valid != 0;
    std::string reason;
    auto add = [&](const char* t) {
      if (!reason.empty()) {
        reason += '-';
      }
      reason += t;
    };
    if (cfg_.on_detection && det_valid) {
      add("det");
    }
    if (cfg_.on_raw0 && !det_valid) {
      add("raw0");
    }
    if (cfg_.on_override && tel.msp_override_active != 0) {
      add("ovr");
    }
    bool eligible = !reason.empty();
    if (cfg_.on_interval) {
      eligible = true;
      add("int");
    }
    if (!eligible) {
      return false;
    }
    if (!first_ && (now - last_save_) < interval_) {
      return false;
    }
    last_save_ = now;
    first_ = false;
    if (tag != nullptr) {
      *tag = reason.empty() ? "x" : reason;
    }
    return true;
  }

  void save(uint64_t frame_seq, int64_t stamp_ns, const cv::Mat& raw_bgr,
            const cv::Mat& annotated_bgr, const std::string& tag) {
    char base[160];
    std::snprintf(base, sizeof(base), "f%08llu_%lld_%s",
                  static_cast<unsigned long long>(frame_seq),
                  static_cast<long long>(stamp_ns), tag.c_str());
    const std::string filename = std::string(base) + ".jpg";
    if (cfg_.save_raw && !raw_bgr.empty()) {
      writeOne(raw_dir_ / filename, raw_bgr, saved_raw_);
    }
    if (cfg_.save_annotated && !annotated_bgr.empty()) {
      writeOne(annotated_dir_ / filename, annotated_bgr, saved_annotated_);
    }
  }

 private:
  void writeOne(const std::filesystem::path& path, const cv::Mat& img,
                std::deque<std::filesystem::path>& ring) {
    if (!cv::imwrite(path.string(), img, jpeg_params_)) {
      return;
    }
    ring.push_back(path);
    while (cfg_.max_files > 0 && ring.size() > cfg_.max_files) {
      std::error_code ec;
      std::filesystem::remove(ring.front(), ec);
      ring.pop_front();
    }
  }

  DebugImageSaveConfig cfg_;
  std::filesystem::path annotated_dir_;
  std::filesystem::path raw_dir_;
  std::vector<int> jpeg_params_;
  std::deque<std::filesystem::path> saved_annotated_;
  std::deque<std::filesystem::path> saved_raw_;
  std::chrono::steady_clock::time_point last_save_{};
  std::chrono::steady_clock::duration interval_{};
  bool first_{true};
};

std::unique_ptr<DebugImageSaver> makeDebugImageSaver(
    const DebugImageSaveConfig& cfg) {
  if (!cfg.enabled) {
    return nullptr;
  }
  auto saver = std::make_unique<DebugImageSaver>(cfg);
  if (!saver->active()) {
    circle::bf::logWarn("bf_debugd: bf_debug.image_save.enabled but no content or "
                        "trigger selected; nothing will be saved");
    return nullptr;
  }
  circle::bf::logInfo("bf_debugd: bf_debug.image_save enabled dir=", cfg.dir,
                      " annotated=", cfg.save_annotated,
                      " raw=", cfg.save_raw,
                      " interval_s=", cfg.interval_s, " triggers=[",
                      cfg.on_interval ? "int " : "",
                      cfg.on_detection ? "det " : "",
                      cfg.on_override ? "ovr " : "",
                      cfg.on_raw0 ? "raw0" : "", "]");
  return saver;
}

struct BfDebugConfig {
  int http_port{8080};
  std::string preview_shm{circle::ipc::kDebugPreviewShmName};
  std::string plots_html{"/opt/circle/share/circle/debug/web/plots.html"};
  std::string strike_flight_config;
  /** "target_strike" (default) or "target_strike_png": selects which param
   *  schema bf_debugd uses for the YAML fallback snapshot and live save. */
  std::string tune_mode{"target_strike"};
  /** "mjpeg" (default) or "h264_webrtc" when GStreamer WebRTC is built in. */
  std::string video_transport{"mjpeg"};
  std::string webrtc_encoder{"auto"};
  int webrtc_bitrate_kbps{600};
  bool webrtc_bitrate_scale_with_fps{true};
  double webrtc_fps{15.0};
  bool encode_when_client_open{true};
  int mjpeg_quality{70};
  std::string log_level{"info"};
  std::string log_color{"auto"};
  DebugImageSaveConfig image_save{};
};

std::string bfConfigPackageRoot(const std::string& config_path) {
  namespace fs = std::filesystem;
  std::error_code ec;
  const fs::path abs = fs::absolute(config_path, ec);
  fs::path parent = abs.parent_path();
  if (parent.filename() == "config") {
    return parent.parent_path().string();
  }
  return parent.string();
}

void resolveConfigRelativePath(const std::string& package_root,
                               std::string& value) {
  if (value.empty()) {
    return;
  }
  namespace fs = std::filesystem;
  const fs::path p(value);
  if (p.is_absolute()) {
    return;
  }
  std::error_code ec;
  value = fs::absolute(fs::path(package_root) / p, ec).string();
}

BfDebugConfig loadConfig(const std::string& path) {
  BfDebugConfig cfg;
#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
  try {
    YAML::Node root = YAML::LoadFile(path);
    const YAML::Node node = root["bf_debug"] ? root["bf_debug"] : root;
    if (node["http_port"]) {
      cfg.http_port = node["http_port"].as<int>();
    }
    if (node["preview_shm"]) {
      cfg.preview_shm = node["preview_shm"].as<std::string>();
    }
    if (node["plots_html"]) {
      cfg.plots_html = node["plots_html"].as<std::string>();
    }
    if (node["tune_mode"]) {
      cfg.tune_mode = node["tune_mode"].as<std::string>();
    }
    if (node["strike_flight_config"]) {
      cfg.strike_flight_config = node["strike_flight_config"].as<std::string>();
    }
    if (node["video_transport"]) {
      cfg.video_transport = node["video_transport"].as<std::string>();
    }
    if (node["mjpeg_quality"]) {
      cfg.mjpeg_quality = std::clamp(node["mjpeg_quality"].as<int>(), 30, 95);
    }
    if (node["webrtc_encoder"]) {
      cfg.webrtc_encoder = node["webrtc_encoder"].as<std::string>();
    }
    if (node["webrtc_bitrate_kbps"]) {
      cfg.webrtc_bitrate_kbps = node["webrtc_bitrate_kbps"].as<int>();
    }
    if (node["webrtc_fps"]) {
      cfg.webrtc_fps = node["webrtc_fps"].as<double>();
    }
    if (node["webrtc_bitrate_scale_with_fps"]) {
      cfg.webrtc_bitrate_scale_with_fps =
          node["webrtc_bitrate_scale_with_fps"].as<bool>();
    }
    if (node["encode_when_client_open"]) {
      cfg.encode_when_client_open = node["encode_when_client_open"].as<bool>();
    }
    if (node["log_level"]) {
      cfg.log_level = node["log_level"].as<std::string>();
    }
    if (node["log_color"]) {
      cfg.log_color = node["log_color"].as<std::string>();
    }
    const YAML::Node img =
        node["image_save"] ? node["image_save"] : YAML::Node();
    if (img["enabled"]) {
      cfg.image_save.enabled = img["enabled"].as<bool>();
    }
    if (img["dir"]) {
      cfg.image_save.dir = img["dir"].as<std::string>();
    }
    if (img["jpeg_quality"]) {
      cfg.image_save.jpeg_quality = img["jpeg_quality"].as<int>();
    }
    if (img["max_files"]) {
      cfg.image_save.max_files = img["max_files"].as<uint32_t>();
    }
    if (img["save_annotated"]) {
      cfg.image_save.save_annotated = img["save_annotated"].as<bool>();
    }
    if (img["save_raw"]) {
      cfg.image_save.save_raw = img["save_raw"].as<bool>();
    }
    if (img["interval_s"]) {
      cfg.image_save.interval_s = img["interval_s"].as<double>();
    }
    if (img["on_interval"]) {
      cfg.image_save.on_interval = img["on_interval"].as<bool>();
    }
    if (img["on_detection"]) {
      cfg.image_save.on_detection = img["on_detection"].as<bool>();
    }
    if (img["on_override"]) {
      cfg.image_save.on_override = img["on_override"].as<bool>();
    }
    if (img["on_raw0"]) {
      cfg.image_save.on_raw0 = img["on_raw0"].as<bool>();
    }
    const std::string package_root = bfConfigPackageRoot(path);
    resolveConfigRelativePath(package_root, cfg.plots_html);
    resolveConfigRelativePath(package_root, cfg.strike_flight_config);
    resolveConfigRelativePath(package_root, cfg.image_save.dir);
  } catch (...) {
  }
#else
  (void)path;
#endif
  return cfg;
}

std::string makeBfParamsJson(const BfDebugConfig& cfg,
                             circle::ipc::StrikeParamsSnapshotReader& reader,
                             std::mutex& reader_mu) {
  {
    // The HTTP server dispatches each request on its own detached thread, so
    // the snapshot reader is shared across threads: guard open()/read() with a
    // mutex. bf_debugd opens this reader once at startup, but bf_flight only
    // creates the snapshot SHM after its (slow) rknn/MSP init. If bf_debugd
    // won that race the reader stayed closed forever and the tuning page kept
    // serving on-disk YAML values, making every live edit look ineffective.
    // Reopen lazily until bf_flight has published its live params snapshot.
    std::lock_guard<std::mutex> lk(reader_mu);
    std::string snapshot;
    if (reader.readCurrentJson(snapshot) ||
        (reader.open(circle::ipc::kStrikeParamsSnapshotShmName) &&
         reader.readCurrentJson(snapshot))) {
      return snapshot;
    }
  }
  const bool png_mode = cfg.tune_mode == "target_strike_png";
#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
  if (!cfg.strike_flight_config.empty()) {
    try {
      if (png_mode) {
        const auto params = circle::strike_png::loadStrikePngParamsFromYaml(
            cfg.strike_flight_config);
        return circle::debug_common::strikePngParamsJson(params);
      }
      const auto params =
          circle::strike::loadStrikeParamsFromYaml(cfg.strike_flight_config);
      return circle::debug_common::strikeCoreParamsJson(params);
    } catch (const std::exception& e) {
      return std::string(R"({"ok":false,"error":"flight config load failed: )") +
             e.what() + "\"}";
    }
  }
#endif
  if (png_mode) {
    return circle::debug_common::strikePngParamsJson(
        circle::strike_png::StrikePngNodeParams{});
  }
  return circle::debug_common::strikeCoreParamsJson(circle::strike::StrikeParams{});
}

std::string readFile(const std::string& path) {
  std::ifstream in(path);
  if (!in) {
    return {};
  }
  return std::string((std::istreambuf_iterator<char>(in)),
                     std::istreambuf_iterator<char>());
}

bool httpSendAll(int fd, const char* data, int len) {
  int sent = 0;
  while (sent < len) {
    const int n = ::send(fd, data + sent, len - sent, MSG_NOSIGNAL);
    if (n <= 0) {
      return false;
    }
    sent += n;
  }
  return true;
}

void httpSendSimple(int fd, int status, const char* phrase,
                    const std::string& body) {
  std::string hdr = "HTTP/1.1 " + std::to_string(status) + " " + phrase + "\r\n"
                    "Content-Type: application/json\r\n"
                    "Connection: close\r\n"
                    "Content-Length: " +
                    std::to_string(body.size()) + "\r\n\r\n";
  (void)httpSendAll(fd, hdr.c_str(), static_cast<int>(hdr.size()));
  (void)httpSendAll(fd, body.c_str(), static_cast<int>(body.size()));
}

std::string timestampSuffixLocal() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf{};
#if defined(_WIN32)
  localtime_s(&tm_buf, &t);
#else
  localtime_r(&t, &tm_buf);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
  return oss.str();
}

std::string makeTimestampedSavePath(const std::string& base_path) {
  namespace fs = std::filesystem;
  const fs::path base(base_path);
  const std::string ext = base.extension().string().empty() ? ".yaml" : base.extension().string();
  return (base.parent_path() / (base.stem().string() + "_" + timestampSuffixLocal() + ext))
      .string();
}

std::string queryParamValue(const std::string& query, const std::string& key) {
  const std::string needle = key + "=";
  size_t pos = query.find(needle);
  if (pos == std::string::npos) {
    if (query.rfind(key + "=", 0) == 0) {
      pos = 0;
    } else {
      return {};
    }
  }
  pos += needle.size();
  const size_t end = query.find('&', pos);
  return (end == std::string::npos) ? query.substr(pos) : query.substr(pos, end - pos);
}

bool isSafeSavedParamsFilename(const std::string& name) {
  if (name.empty() || name.size() > 200) {
    return false;
  }
  if (name.find("..") != std::string::npos || name.find('/') != std::string::npos ||
      name.find('\\') != std::string::npos) {
    return false;
  }
  namespace fs = std::filesystem;
  if (fs::path(name).extension() != ".yaml") {
    return false;
  }
  for (unsigned char c : name) {
    if (!(std::isalnum(c) || c == '_' || c == '-' || c == '.')) {
      return false;
    }
  }
  return true;
}

bool sendAttachmentResponse(int fd, const std::string& filename,
                            const std::string& body) {
  std::string hdr = "HTTP/1.1 200 OK\r\n"
                    "Content-Type: application/x-yaml\r\n"
                    "Content-Disposition: attachment; filename=\"" +
                    filename +
                    "\"\r\n"
                    "Connection: close\r\n"
                    "Content-Length: " +
                    std::to_string(body.size()) + "\r\n\r\n";
  return httpSendAll(fd, hdr.c_str(), static_cast<int>(hdr.size())) &&
         httpSendAll(fd, body.c_str(), static_cast<int>(body.size()));
}

std::pair<int, std::string> saveBfStrikeParams(
    const BfDebugConfig& cfg,
    circle::ipc::StrikeParamsSnapshotReader& reader,
    std::mutex& reader_mu) {
#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
  if (cfg.strike_flight_config.empty()) {
    return {400, R"({"ok":false,"error":"strike_flight_config not set"})"};
  }
  std::string snapshot;
  {
    std::lock_guard<std::mutex> lk(reader_mu);
    if (reader.currentSeq() == 0) {
      reader.open(circle::ipc::kStrikeParamsSnapshotShmName);
    }
    if (!reader.readCurrentJson(snapshot)) {
      return {503,
              R"({"ok":false,"error":"live params snapshot unavailable"})"};
    }
  }
  std::string err;
  const std::string output_path = makeTimestampedSavePath(cfg.strike_flight_config);
  if (cfg.tune_mode == "target_strike_png") {
    circle::strike_png::StrikePngNodeParams live;
    if (!circle::debug_common::strikePngParamsFromTunableJson(
            cfg.strike_flight_config, snapshot, live, &err)) {
      return {500, std::string(R"({"ok":false,"error":")") + err + "\"}"};
    }
    if (!circle::debug_common::saveStrikePngTunableParamsToYaml(
            cfg.strike_flight_config, output_path, live, &err)) {
      return {500, std::string(R"({"ok":false,"error":")") + err + "\"}"};
    }
  } else {
    circle::strike::StrikeParams live;
    if (!circle::debug_common::strikeParamsFromTunableJson(
            cfg.strike_flight_config, snapshot, live, &err)) {
      return {500, std::string(R"({"ok":false,"error":")") + err + "\"}"};
    }
    if (!circle::debug_common::saveStrikeTunableParamsToYaml(
            cfg.strike_flight_config, output_path, live, &err)) {
      return {500, std::string(R"({"ok":false,"error":")") + err + "\"}"};
    }
  }
  const std::filesystem::path out_file(output_path);
  circle::bf::logInfo("bf_debugd: saved live strike params to ", output_path);
  return {200, std::string(R"({"ok":true,"path":")") + output_path +
                     R"(","filename":")" + out_file.filename().string() + "\"}"};
#else
  (void)cfg;
  (void)reader;
  (void)reader_mu;
  return {501, R"({"ok":false,"error":"yaml support not built"})"};
#endif
}

void serveSavedParamsDownload(int fd, const std::string& query,
                              const std::string& save_dir) {
  namespace fs = std::filesystem;
  const std::string filename = queryParamValue(query, "file");
  if (!isSafeSavedParamsFilename(filename)) {
    httpSendSimple(fd, 400, "Bad Request",
                   R"({"ok":false,"error":"invalid file name"})");
    return;
  }

  std::error_code ec;
  const fs::path save_root = fs::weakly_canonical(fs::path(save_dir), ec);
  if (ec || save_root.empty()) {
    httpSendSimple(fd, 500, "Internal Server Error",
                   R"({"ok":false,"error":"save dir unavailable"})");
    return;
  }

  const fs::path candidate = save_root / filename;
  const fs::path canonical = fs::weakly_canonical(candidate, ec);
  if (ec || canonical.empty() || !fs::is_regular_file(canonical, ec)) {
    httpSendSimple(fd, 404, "Not Found",
                   R"({"ok":false,"error":"file not found"})");
    return;
  }

  const std::string root_prefix = save_root.string();
  const std::string file_prefix = canonical.string();
  if (file_prefix.size() < root_prefix.size() ||
      file_prefix.compare(0, root_prefix.size(), root_prefix) != 0) {
    httpSendSimple(fd, 403, "Forbidden",
                   R"({"ok":false,"error":"path escape blocked"})");
    return;
  }

  const std::string body = readFile(canonical.string());
  if (body.empty()) {
    httpSendSimple(fd, 404, "Not Found",
                   R"({"ok":false,"error":"file not found"})");
    return;
  }
  (void)sendAttachmentResponse(fd, filename, body);
}

uint32_t sampledHash(const std::vector<uint8_t>& data) {
  uint32_t h = 2166136261u;
  if (data.empty()) {
    return h;
  }
  const size_t step = std::max<size_t>(1, data.size() / 4096);
  for (size_t i = 0; i < data.size(); i += step) {
    h ^= data[i];
    h *= 16777619u;
  }
  return h;
}

int effectiveWebRtcBitrateKbps(int nominal_kbps, double fps,
                               bool scale_with_fps) {
  if (!scale_with_fps) {
    return std::clamp(nominal_kbps, 120, 50000);
  }
  // Match tracking_debug_node: mpph264enc CBR at low fps needs lower bps to
  // avoid oversized NAL/RTP bursts on Wi-Fi.
  constexpr double kRefFps = 30.0;
  const double clamped_fps = fps > 0.1 ? fps : kRefFps;
  const double scale = std::min(clamped_fps, kRefFps) / kRefFps;
  const int out =
      static_cast<int>(std::lround(static_cast<double>(nominal_kbps) * scale));
  return std::clamp(out, 120, 50000);
}

std::string makeVideoStatusJson(bool telemetry_open, bool preview_open,
                                const std::string& transport,
                                uint64_t preview_seq,
                                uint64_t published_preview_frames,
                                int64_t preview_stamp_ns,
                                int mjpeg_clients,
                                int preview_clients,
                                double webrtc_fps,
                                int nominal_bitrate_kbps,
                                int effective_bitrate_kbps,
                                const std::string& encoder_name
#if defined(BF_DEBUGD_HAS_WEBRTC) && BF_DEBUGD_HAS_WEBRTC
                                ,
                                const circle::debug::WebRtcH264Sender* webrtc
#endif
) {
  const bool h264 = (transport == "h264_webrtc");
#if defined(BF_DEBUGD_HAS_WEBRTC) && BF_DEBUGD_HAS_WEBRTC
  const bool h264_stack_ready = h264 && webrtc && webrtc->isRunning();
  const bool webrtc_signaling_ready = h264_stack_ready;
  const bool webrtc_frame_ready = h264_stack_ready && webrtc->hasFrameSize();
  const bool webrtc_media_ready =
      webrtc_signaling_ready && webrtc->hasMediaPipeline();
#else
  const bool h264_stack_ready = false;
  const bool webrtc_signaling_ready = false;
  const bool webrtc_frame_ready = false;
  const bool webrtc_media_ready = false;
#endif
  const bool ok = preview_open && (!h264 || h264_stack_ready);

  std::ostringstream oss;
  oss << R"({"ok":)" << (ok ? "true" : "false")
      << R"(,"mode":"bf_debugd","telemetry_open":)"
      << (telemetry_open ? "true" : "false")
      << R"(,"preview_open":)" << (preview_open ? "true" : "false")
      << R"(,"transport":")" << transport << '"'
      << R"(,"preview_browser":")" << (h264 ? "webrtc" : "mjpeg") << '"'
      << R"(,"h264_mode":)" << (h264 ? "true" : "false")
      << R"(,"preview_seq":)" << preview_seq
      << R"(,"published_preview_frames":)" << published_preview_frames
      << R"(,"preview_stamp_ns":)" << preview_stamp_ns
      << R"(,"mjpeg_clients":)" << mjpeg_clients
      << R"(,"active_clients":)" << preview_clients
      << R"(,"preview_clients":)" << preview_clients;
  if (preview_open) {
    oss << R"(,"mjpeg_preview_path":"/api/video/mjpeg")";
  }
  if (h264) {
    oss << R"(,"webrtc_offer_path":"/api/webrtc/offer")";
  }
#if defined(BF_DEBUGD_HAS_WEBRTC) && BF_DEBUGD_HAS_WEBRTC
  oss << R"(,"encoder":")" << encoder_name << '"'
      << R"(,"h264_stack_ready":)" << (h264_stack_ready ? "true" : "false")
      << R"(,"encoder_ready":)" << (h264_stack_ready ? "true" : "false")
      << R"(,"webrtc_signaling_ready":)"
      << (webrtc_signaling_ready ? "true" : "false")
      << R"(,"webrtc_frame_ready":)" << (webrtc_frame_ready ? "true" : "false")
      << R"(,"webrtc_media_ready":)" << (webrtc_media_ready ? "true" : "false")
      << R"(,"fps":)" << webrtc_fps
      << R"(,"bitrate_kbps":)" << nominal_bitrate_kbps
      << R"(,"webrtc_encoder_bitrate_kbps":)" << effective_bitrate_kbps;
  if (webrtc) {
    const auto st = webrtc->stats();
    oss << R"(,"webrtc_frames_accepted":)" << st.frames_accepted
        << R"(,"webrtc_frames_dropped":)" << st.frames_dropped
        << R"(,"webrtc_frames_pushed":)" << st.frames_pushed
        << R"(,"webrtc_h264_buffers":)" << st.h264_buffers
        << R"(,"webrtc_rtp_packets":)" << st.rtp_packets
        << R"(,"webrtc_frame_width":)" << st.frame_width
        << R"(,"webrtc_frame_height":)" << st.frame_height;
  }
#else
  oss << R"(,"encoder_ready":false,"webrtc_media_ready":false)";
#endif
  oss << R"(,"status":")";
  if (!preview_open) {
    oss << "preview SHM not open";
  } else if (h264 && !h264_stack_ready) {
    oss << "H.264/WebRTC stack failed to start (check GStreamer mpph264enc)";
  } else if (h264 && !webrtc_frame_ready) {
    oss << "H.264/WebRTC waiting for preview frames from SHM";
  } else if (h264 && !webrtc_media_ready) {
    oss << "H.264/WebRTC waiting for browser SDP (/api/webrtc/offer)";
  } else if (h264) {
    oss << "H.264/WebRTC streaming";
  } else {
    oss << "MJPEG preview (/api/video/mjpeg)";
  }
  oss << "\"}";
  return oss.str();
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path = "/etc/circle/strike_bf_debug.yaml";
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
      config_path = argv[++i];
    }
  }

  const BfDebugConfig cfg = loadConfig(config_path);
  circle::bf::configureLogger(cfg.log_level, cfg.log_color);
  std::unique_ptr<DebugImageSaver> image_saver =
      makeDebugImageSaver(cfg.image_save);

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  circle::ipc::StrikeTelemetryReader telemetry;
  auto fetchSeriesJson = [&telemetry]() -> std::string {
    if (!telemetry.isOpen()) {
      (void)telemetry.open(circle::ipc::kStrikeTelemetryShmName);
    }
    return telemetry.seriesJson();
  };
  const bool telemetry_open = telemetry.open(circle::ipc::kStrikeTelemetryShmName);
  circle::ipc::ParamBlockWriter param_writer;
  param_writer.open(circle::ipc::kParamBlockShmName);
  circle::ipc::StrikeParamsSnapshotReader params_snapshot_reader;
  params_snapshot_reader.open(circle::ipc::kStrikeParamsSnapshotShmName);
  std::mutex params_snapshot_mu;
  circle::debug::DebugVideoServer server;
  circle::debug::DebugPreviewReader preview_reader;
  std::atomic<bool> preview_open_state{preview_reader.open(cfg.preview_shm)};

#if defined(BF_DEBUGD_HAS_WEBRTC) && BF_DEBUGD_HAS_WEBRTC
  circle::debug::WebRtcH264Sender::Config wcfg;
  wcfg.encoder_element =
      circle::debug::WebRtcH264Sender::GstElementFromEncoderBackend(
          cfg.webrtc_encoder);
  wcfg.fps = std::clamp(cfg.webrtc_fps, 5.0, 30.0);
  const double webrtc_fps = wcfg.fps;
  const int effective_bitrate = effectiveWebRtcBitrateKbps(
      cfg.webrtc_bitrate_kbps, wcfg.fps, cfg.webrtc_bitrate_scale_with_fps);
  wcfg.bitrate_kbps = effective_bitrate;
  circle::debug::WebRtcH264Sender webrtc(wcfg);
  const std::string encoder_name = wcfg.encoder_element;
  if (!wcfg.encoder_element.empty() && !webrtc.start()) {
    circle::bf::logError("bf_debugd: WebRTC start failed: ",
                         webrtc.lastError(), " (encoder=",
                         wcfg.encoder_element, ")");
  } else if (wcfg.encoder_element.empty()) {
    circle::bf::logWarn("bf_debugd: no H.264 GStreamer encoder for "
                        "webrtc_encoder=",
                        cfg.webrtc_encoder);
  }

  server.setWebrtcOfferHandler(
      [&](const std::string& body, const std::string& ip) {
        return webrtc.handleBrowserOffer(body, ip);
      });
#else
  const double webrtc_fps = std::clamp(cfg.webrtc_fps, 5.0, 30.0);
  const int effective_bitrate = effectiveWebRtcBitrateKbps(
      cfg.webrtc_bitrate_kbps, webrtc_fps, cfg.webrtc_bitrate_scale_with_fps);
  const std::string encoder_name;
#endif

  const std::string video_transport =
#if defined(BF_DEBUGD_HAS_WEBRTC) && BF_DEBUGD_HAS_WEBRTC
      (cfg.video_transport == "h264_webrtc") ? "h264_webrtc" : "mjpeg";
#else
      "mjpeg";
#endif

  std::atomic<uint64_t> last_preview_seq{0};
  std::atomic<uint64_t> published_preview_frames{0};
  std::atomic<int64_t> last_preview_stamp_ns{0};

  server.setHandlers(
      [&]() { return readFile(cfg.plots_html); },
      [&](const std::string&) { return fetchSeriesJson(); },
      [&]() {
        return makeBfParamsJson(cfg, params_snapshot_reader,
                                params_snapshot_mu);
      },
      [&](const std::string& body) -> std::pair<int, std::string> {
        uint64_t seq_before = 0;
        {
          std::lock_guard<std::mutex> lk(params_snapshot_mu);
          if (params_snapshot_reader.currentSeq() == 0) {
            params_snapshot_reader.open(circle::ipc::kStrikeParamsSnapshotShmName);
          }
          seq_before = params_snapshot_reader.currentSeq();
        }
        if (!param_writer.writeJson(body)) {
          return {500,
                  R"({"ok":false,"error":"param_block write failed"})"};
        }
        // bf_flight applies param_block on its control loop (~5 ms). The browser
        // used to call refreshParams() immediately after POST and read stale SHM.
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
        while (std::chrono::steady_clock::now() < deadline) {
          {
            std::lock_guard<std::mutex> lk(params_snapshot_mu);
            if (params_snapshot_reader.currentSeq() == 0) {
              params_snapshot_reader.open(circle::ipc::kStrikeParamsSnapshotShmName);
            }
            if (params_snapshot_reader.currentSeq() > seq_before) {
              std::string snapshot;
              if (params_snapshot_reader.readCurrentJson(snapshot)) {
                return {200, std::move(snapshot)};
              }
              break;
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return {200, R"({"ok":true,"pending":true})"};
      },
      [&]() {
        return makeVideoStatusJson(telemetry_open, preview_open_state.load(),
                                   video_transport,
                                   last_preview_seq.load(),
                                   published_preview_frames.load(),
                                   last_preview_stamp_ns.load(),
                                   server.activeMjpegClients(),
                                   server.totalPreviewClients(),
                                   webrtc_fps, cfg.webrtc_bitrate_kbps,
                                   effective_bitrate, encoder_name
#if defined(BF_DEBUGD_HAS_WEBRTC) && BF_DEBUGD_HAS_WEBRTC
                                   ,
                                   &webrtc
#endif
        );
      },
      true);
  const std::string params_save_dir =
      cfg.strike_flight_config.empty()
          ? std::string{}
          : std::filesystem::path(cfg.strike_flight_config).parent_path().string();
  server.setSaveParamsHandler([&]() {
    return saveBfStrikeParams(cfg, params_snapshot_reader, params_snapshot_mu);
  });
  server.setParamsDownloadHandler([&](int fd, const std::string& query) {
    serveSavedParamsDownload(fd, query, params_save_dir);
  });

  if (!server.start(cfg.http_port, false)) {
    circle::bf::logError("bf_debugd: failed to start HTTP on port ",
                         cfg.http_port);
    return 1;
  }

  if (readFile(cfg.plots_html).empty()) {
    circle::bf::logWarn("bf_debugd: plots_html not found or empty: ",
                        cfg.plots_html,
                        " (8080 will show blank page; set bf_debug.plots_html "
                        "in strike_bf_debug.yaml)");
  }

  std::ostringstream started_msg;
  started_msg << "bf_debugd started port=" << cfg.http_port
              << " preview_shm=" << cfg.preview_shm
              << " mjpeg_quality=" << cfg.mjpeg_quality
#if defined(BF_DEBUGD_HAS_WEBRTC) && BF_DEBUGD_HAS_WEBRTC
              << " webrtc_encoder=" << encoder_name
              << " webrtc_bitrate_kbps=" << effective_bitrate
              << " (nominal=" << cfg.webrtc_bitrate_kbps
              << " scale_fps="
              << (cfg.webrtc_bitrate_scale_with_fps ? "yes" : "no") << ")"
              << " webrtc_fps=" << webrtc_fps
              << " transport=" << video_transport
#endif
              << " (build: ice-filter-v2)";
  circle::bf::logInfo(started_msg.str());

  auto last_log = std::chrono::steady_clock::now();
  auto last_seq_change = std::chrono::steady_clock::now();
  uint64_t read_ok = 0;
  uint64_t read_empty = 0;
  uint64_t publish_ok = 0;
  uint64_t jpeg_fail = 0;
  uint64_t image_save_ok = 0;
  uint64_t last_seq = 0;
  size_t last_jpeg_bytes = 0;
  uint32_t last_hash = 0;
  uint32_t last_w = 0;
  uint32_t last_h = 0;

  while (g_running.load()) {
    circle::debug::DebugPreviewFrameWithOverlay frame;
    if (!preview_reader.isOpen()) {
      preview_open_state.store(preview_reader.open(cfg.preview_shm));
    }
    if (preview_reader.readLatestWithOverlay(frame) && !frame.data.empty()) {
      ++read_ok;
      preview_open_state.store(true);
      const int h = static_cast<int>(frame.height);
      const int w = static_cast<int>(frame.width);
      const size_t step = frame.stride > 0 ? frame.stride
                                           : static_cast<size_t>(w) * 3u;
      cv::Mat bgr(h, w, CV_8UC3);
      if (frame.encoding == "rgb8") {
        cv::Mat rgb(h, w, CV_8UC3, frame.data.data(), step);
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      } else {
        cv::Mat src(h, w, CV_8UC3, frame.data.data(), step);
        src.copyTo(bgr);
      }
      cv::Mat raw_bgr;
      if (image_saver && image_saver->wantsRaw()) {
        raw_bgr = bgr.clone();
      }
      // Draw overlay if present
      if (frame.overlay_valid) {
        circle::debug::PreviewOverlayContext ctx =
            circle::debug::toPreviewOverlayContext(frame.overlay);
        circle::debug::drawPreviewOverlay(bgr, ctx);
      }
      std::vector<uint8_t> jpeg;
      const bool need_jpeg =
          video_transport == "mjpeg" || server.activeMjpegClients() > 0;
      if (need_jpeg) {
        const std::vector<int> jpeg_params{
            cv::IMWRITE_JPEG_QUALITY, cfg.mjpeg_quality,
            cv::IMWRITE_JPEG_OPTIMIZE, 0};
        if (cv::imencode(".jpg", bgr, jpeg, jpeg_params) && !jpeg.empty()) {
          server.publishMjpegFrame(jpeg.data(), jpeg.size());
          last_jpeg_bytes = jpeg.size();
        } else {
          ++jpeg_fail;
        }
      }
      last_preview_seq.store(frame.seq);
      last_preview_stamp_ns.store(frame.stamp_ns);
      published_preview_frames.fetch_add(1);
      ++publish_ok;
      last_seq = frame.seq;
      last_seq_change = std::chrono::steady_clock::now();
      last_hash = sampledHash(frame.data);
      last_w = frame.width;
      last_h = frame.height;
      if (image_saver) {
        circle::ipc::StrikeTelemetrySample tel{};
        if (telemetry.readLatest(tel)) {
          std::string save_tag;
          const auto now = std::chrono::steady_clock::now();
          if (image_saver->shouldSave(tel, now, &save_tag)) {
            image_saver->save(frame.seq, frame.stamp_ns, raw_bgr, bgr, save_tag);
            ++image_save_ok;
          }
        }
      }
#if defined(BF_DEBUGD_HAS_WEBRTC) && BF_DEBUGD_HAS_WEBRTC
      if (video_transport == "h264_webrtc" && webrtc.isRunning()) {
        const bool allow_out =
            !cfg.encode_when_client_open ||
            server.totalPreviewClients() > 0 || webrtc.hasMediaPipeline();
        if (allow_out) {
          static uint64_t last_webrtc_frame_seq = 0;
          if (frame.seq != last_webrtc_frame_seq) {
            webrtc.pushBgrFrame(bgr);
            last_webrtc_frame_seq = frame.seq;
          }
        }
      }
#endif
    } else {
      ++read_empty;
      circle::ipc::StrikeTelemetrySample sample;
      (void)telemetry.readLatest(sample);
    }
    const auto now = std::chrono::steady_clock::now();
    if (now - last_log >= std::chrono::seconds(5)) {
      const auto seq_stale_for =
          std::chrono::duration_cast<std::chrono::seconds>(now - last_seq_change)
              .count();
      if (publish_ok > 0 && seq_stale_for >= 3) {
        circle::bf::logWarn(
            "bf_debugd/video: preview SHM seq unchanged for ", seq_stale_for,
            "s (is bf_flight running?)");
      }
      // Periodic video stats disabled (noisy).
      /*
      circle::bf::logInfo(
          "bf_debugd/video: preview_open=",
          (preview_open_state.load() ? "yes" : "no"), " read_ok=", read_ok,
          " read_empty=", read_empty, " publish_ok=", publish_ok,
          " jpeg_fail=", jpeg_fail, " last_seq=", last_seq, " size=", last_w,
          'x', last_h, " hash=0x", std::hex, last_hash, std::dec,
          " jpeg_bytes=", last_jpeg_bytes,
          " mjpeg_clients=", server.activeMjpegClients(),
          " preview_clients=", server.totalPreviewClients());
      */
      last_log = now;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(33));
  }

  server.stop();
#if defined(BF_DEBUGD_HAS_WEBRTC) && BF_DEBUGD_HAS_WEBRTC
  webrtc.stop();
#endif
  return 0;
}
