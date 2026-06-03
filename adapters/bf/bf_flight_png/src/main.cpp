#include <atomic>
#include <csignal>
#include <memory>
#include <string>

#include "circle/bf/logger.hpp"
#include "circle/bf/msp_client.hpp"
#include "circle/bf/runtime/bf_control_host.hpp"
#include "circle/strike_png/strike_png_node_params.hpp"
#include "png_controller_adapter.hpp"

#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
#include "circle/strike_png/strike_png_params_yaml.hpp"
#endif

namespace {

std::atomic<bool> g_running{true};

void onSignal(int) { g_running.store(false); }

}  // namespace

int main(int argc, char** argv) {
  std::string config_path = "/etc/circle/strike_png_bf_flight.yaml";
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
      config_path = argv[++i];
    }
  }

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  circle::strike_png::StrikePngNodeParams png{};
  circle::bf::runtime::BfRuntimeConfig runtime;
#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
  png = circle::strike_png::loadStrikePngParamsFromYaml(config_path);
  circle::bf::runtime::loadBfRuntimeConfigFromYaml(config_path, runtime);
#endif
  runtime.mode_tag = "target_strike_png";
  runtime.dry_run = png.dry_run;
  runtime.require_armed_to_command = png.require_armed_to_command;
  runtime.filter.min_score = png.min_score;
  runtime.filter.target_class_name = png.target_class_name;

  circle::bf::configureLogger(runtime.log_level, runtime.log_color);

  circle::bf::png::PngControllerAdapter adapter(png);
  auto msp = std::make_shared<circle::bf::MspClient>();
  circle::bf::runtime::BfControlHost host(std::move(runtime), msp, adapter);
  return host.run(g_running);
}
