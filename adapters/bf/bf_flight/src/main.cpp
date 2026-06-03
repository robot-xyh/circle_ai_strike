#include <atomic>
#include <csignal>
#include <memory>
#include <string>

#include "circle/bf/logger.hpp"
#include "circle/bf/msp_client.hpp"
#include "circle/bf/runtime/bf_control_host.hpp"
#include "circle/strike/strike_params.hpp"
#include "strike_controller_adapter.hpp"

#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
#include "circle/strike/strike_params_yaml.hpp"
#endif

namespace {

std::atomic<bool> g_running{true};

void onSignal(int) { g_running.store(false); }

}  // namespace

int main(int argc, char** argv) {
  std::string config_path = "/etc/circle/strike_bf_flight.yaml";
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
      config_path = argv[++i];
    }
  }

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  circle::strike::StrikeParams strike{};
  circle::bf::runtime::BfRuntimeConfig runtime;
#if defined(CIRCLE_STRIKE_HAS_YAML) && CIRCLE_STRIKE_HAS_YAML
  strike = circle::strike::loadStrikeParamsFromYaml(config_path);
  circle::bf::runtime::loadBfRuntimeConfigFromYaml(config_path, runtime);
#endif
  runtime.mode_tag = "target_strike";
  runtime.filter = strike.filter;
  runtime.dry_run = strike.dry_run;
  runtime.require_armed_to_command = strike.require_armed_to_command;

  circle::bf::configureLogger(runtime.log_level, runtime.log_color);

  circle::bf::flight::StrikeControllerAdapter adapter(strike);
  auto msp = std::make_shared<circle::bf::MspClient>();
  circle::bf::runtime::BfControlHost host(std::move(runtime), msp, adapter);
  return host.run(g_running);
}
