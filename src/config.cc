#include "config.h"
#include "baldr/rapidjson_utils.h"

#include <valhalla/property_tree/ptree.hpp>

#include <filesystem>

namespace {
struct config_singleton_t {
protected:
  boost::property_tree::ptree config_;

  config_singleton_t() = delete;

  config_singleton_t(const std::string& config_inline) {
    if (config_inline.empty()) {
      throw std::runtime_error("Config singleton was not initialized before usage");
    }

    auto inline_config = std::stringstream(config_inline);
    rapidjson::read_json(inline_config, config_);
  }

public:
  config_singleton_t(config_singleton_t const&) = delete;
  void operator=(const config_singleton_t&) = delete;
  friend const boost::property_tree::ptree&
  valhalla::config(const std::string& config_inline);
};
} // namespace

namespace valhalla {

const boost::property_tree::ptree& config(const std::string& config_inline) {
  static config_singleton_t instance(config_inline);
  return instance.config_;
}

}; // namespace valhalla
