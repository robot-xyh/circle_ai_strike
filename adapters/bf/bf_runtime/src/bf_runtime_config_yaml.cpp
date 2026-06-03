#include "circle/bf/runtime/bf_control_host.hpp"

#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML

#include <filesystem>

#include <yaml-cpp/yaml.h>

#include "circle/bf/bf_flight_config_yaml.hpp"
#include "circle/perception/camera_config_loader.hpp"
#include "circle/perception/camera_info_loader.hpp"

namespace circle::bf::runtime {

namespace {

/** YAML relative paths resolve against the circle_pilot package root
 *  (config/*.yaml -> parent dir). Mirrors bf_flight/bf_debugd. */
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

/** Parse one `bf_flight:` block onto cfg. Only keys present are written, so
 *  this can be called twice: first on a shared base file, then on an inline
 *  override map (per-mode tweaks win). */
void parseBfRuntimeNode(const YAML::Node& bf, BfRuntimeConfig& cfg) {
  if (!bf || !bf.IsMap()) {
    return;
  }
  {
    if (bf["msp"]) {
      circle::bf::loadBfFlightMspFromYaml(bf["msp"], cfg.msp);
    }
    if (bf["camera"]) {
      cfg.camera_yaml = bf["camera"].as<std::string>();
    }
    if (bf["camera_info"]) {
      cfg.camera_info_yaml = bf["camera_info"].as<std::string>();
    }
    if (bf["camera_device"]) {
      cfg.camera.device = bf["camera_device"].as<std::string>();
    }
    if (bf["camera_width"]) {
      cfg.camera.width = bf["camera_width"].as<uint32_t>();
    }
    if (bf["camera_height"]) {
      cfg.camera.height = bf["camera_height"].as<uint32_t>();
    }
    if (bf["camera_fps"]) {
      cfg.camera.fps = bf["camera_fps"].as<uint32_t>();
    }
    if (bf["model_path"]) {
      cfg.model_path = bf["model_path"].as<std::string>();
    }
    if (bf["detection_file"]) {
      cfg.detection_file = bf["detection_file"].as<std::string>();
    }
    if (bf["log_level"]) {
      cfg.log_level = bf["log_level"].as<std::string>();
    }
    if (bf["log_color"]) {
      cfg.log_color = bf["log_color"].as<std::string>();
    }
    if (bf["max_infer_fps"]) {
      cfg.max_infer_fps = bf["max_infer_fps"].as<uint32_t>();
    }
    if (bf["zero_copy_pipeline_enabled"]) {
      cfg.zero_copy_pipeline_enabled = bf["zero_copy_pipeline_enabled"].as<bool>();
    }
    if (bf["zero_copy_slot_count"]) {
      cfg.zero_copy_slot_count = bf["zero_copy_slot_count"].as<int>();
    }
    if (bf["infer_worker_count"]) {
      cfg.infer_worker_count = bf["infer_worker_count"].as<int>();
    }
    if (bf["rknn_core_masks"]) {
      cfg.rknn_core_masks = bf["rknn_core_masks"].as<std::vector<int>>();
    }
    if (bf["conf_threshold"]) {
      cfg.conf_threshold = bf["conf_threshold"].as<float>();
    }
    if (bf["detection_coast_s"]) {
      cfg.detection_coast_s = bf["detection_coast_s"].as<double>();
    }
    if (bf["engage_require_fresh_detection"]) {
      cfg.engage_require_fresh_detection =
          bf["engage_require_fresh_detection"].as<bool>();
    }
    if (bf["engage_detection_fresh_timeout_s"]) {
      cfg.engage_detection_fresh_timeout_s =
          bf["engage_detection_fresh_timeout_s"].as<double>();
    }
    if (bf["watchdog_enabled"]) {
      cfg.watchdog_enabled = bf["watchdog_enabled"].as<bool>();
    }
    if (bf["watchdog_state_timeout_s"]) {
      cfg.watchdog_state_timeout_s = bf["watchdog_state_timeout_s"].as<double>();
    }
    if (bf["watchdog_overrun_warn_s"]) {
      cfg.watchdog_overrun_warn_s = bf["watchdog_overrun_warn_s"].as<double>();
    }
    if (bf["throttle_handover_s"]) {
      cfg.throttle_handover_s = bf["throttle_handover_s"].as<double>();
    }
    if (bf["control_loop_hz"]) {
      cfg.control_loop_hz = bf["control_loop_hz"].as<double>();
    }
    if (bf["iou_threshold"]) {
      cfg.iou_threshold = bf["iou_threshold"].as<float>();
    }
    if (bf["max_det"]) {
      cfg.max_det = bf["max_det"].as<int>();
    }
    if (bf["preview_shm_enabled"]) {
      cfg.preview_shm_enabled = bf["preview_shm_enabled"].as<bool>();
    }
    if (bf["preview_max_fps"]) {
      cfg.preview_max_fps = bf["preview_max_fps"].as<uint32_t>();
    }
    if (bf["rc"]) {
      circle::bf::loadBfFlightRcFromYaml(bf["rc"], cfg.rc);
    }
    if (bf["pipeline_out_width"]) {
      cfg.camera.output_width = bf["pipeline_out_width"].as<uint32_t>();
    }
    if (bf["pipeline_out_height"]) {
      cfg.camera.output_height = bf["pipeline_out_height"].as<uint32_t>();
    }
  }
}

}  // namespace

void loadBfRuntimeConfigFromYaml(const std::string& path, BfRuntimeConfig& cfg) {
  const std::string package_root = bfConfigPackageRoot(path);
  try {
    YAML::Node root = YAML::LoadFile(path);

    // Optional indirection: a controller config may point at a shared runtime
    // file via `bf_flight_config: <path>` (or `bf_flight: <path>` scalar) so
    // multiple control modes reuse one `bf_flight:` block. The referenced file
    // is parsed first as the base; an inline `bf_flight:` map (if any) is then
    // applied on top as per-mode overrides.
    std::string ref;
    if (root["bf_flight_config"]) {
      ref = root["bf_flight_config"].as<std::string>();
    } else if (root["bf_flight"] && root["bf_flight"].IsScalar()) {
      ref = root["bf_flight"].as<std::string>();
    }
    bool parsed_any = false;
    if (!ref.empty()) {
      std::string ref_abs = ref;
      resolveConfigRelativePath(package_root, ref_abs);
      try {
        YAML::Node ref_root = YAML::LoadFile(ref_abs);
        const YAML::Node ref_bf =
            ref_root["bf_flight"] ? ref_root["bf_flight"] : ref_root;
        parseBfRuntimeNode(ref_bf, cfg);
        parsed_any = true;
      } catch (...) {
      }
    }
    if (root["bf_flight"] && root["bf_flight"].IsMap()) {
      parseBfRuntimeNode(root["bf_flight"], cfg);
      parsed_any = true;
    }
    if (!parsed_any) {
      parseBfRuntimeNode(root, cfg);
    }
  } catch (...) {
  }
  resolveConfigRelativePath(package_root, cfg.camera_yaml);
  resolveConfigRelativePath(package_root, cfg.camera_info_yaml);
  resolveConfigRelativePath(package_root, cfg.model_path);
  resolveConfigRelativePath(package_root, cfg.detection_file);

  if (!cfg.camera_yaml.empty()) {
    circle::perception::loadMppCameraConfigFromYaml(cfg.camera_yaml, cfg.camera);
  } else {
    cfg.camera.device = cfg.camera.device.empty() ? "/dev/video0" : cfg.camera.device;
    if (cfg.camera.width == 0) {
      cfg.camera.width = 1280;
    }
    if (cfg.camera.height == 0) {
      cfg.camera.height = 1024;
    }
    if (cfg.camera.fps == 0) {
      cfg.camera.fps = 60;
    }
    if (cfg.camera.output_width == 0) {
      cfg.camera.output_width = 640;
    }
    if (cfg.camera.output_height == 0) {
      cfg.camera.output_height = 512;
    }
  }
  if (!cfg.camera_info_yaml.empty()) {
    circle::perception::loadCameraIntrinsicsFromYaml(cfg.camera_info_yaml,
                                                     cfg.intrinsics);
  }
}

}  // namespace circle::bf::runtime

#endif  // CIRCLE_STRIKE_HAS_YAML
