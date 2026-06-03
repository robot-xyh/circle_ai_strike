#pragma once

#include <string>
#include <variant>
#include <vector>

namespace circle::fc_iface {

using ParamValue = std::variant<bool, int64_t, double, std::string>;

struct TunableParamDesc {
  std::string name;
  double min_value{0.0};
  double max_value{1.0};
  double step{0.01};
  bool is_bool{false};
};

class IParamStore {
 public:
  virtual ~IParamStore() = default;

  virtual bool get(const std::string& name, ParamValue& out) const = 0;
  virtual bool set(const std::string& name, const ParamValue& value) = 0;
  virtual std::vector<TunableParamDesc> listTunable() const = 0;
};

}  // namespace circle::fc_iface
