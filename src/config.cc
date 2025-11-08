#include "config.h"
#include "baldr/rapidjson_utils.h"

#include <valhalla/property_tree/ptree.hpp>

#include <filesystem>

namespace {
struct config_singleton_t {
protected:
  valhalla::property_tree config_;

  config_singleton_t() = delete;

  config_singleton_t(const std::string& config_file_or_inline) {
    if (config_file_or_inline.empty()) {
      throw std::runtime_error("Config singleton was not initialized before usage");
    }

#if !defined __EMSCRIPTEN__
    try {
      if (std::filesystem::is_regular_file(config_file_or_inline)) {
        rapidjson::read_json(config_file_or_inline, config_);
      } else {
        auto inline_config = std::stringstream(config_file_or_inline);
        rapidjson::read_json(inline_config, config_);
      }
    } catch (const std::filesystem::filesystem_error& e) {
      if (e.code() == std::errc::filename_too_long) {
        auto inline_config = std::stringstream(config_file_or_inline);
        rapidjson::read_json(inline_config, config_);
      } else {
        throw e;
      }
    }
#else
    auto inline_config = std::stringstream(config_file_or_inline);
    rapidjson::read_json(inline_config, config_);
#endif
  }

public:
  config_singleton_t(config_singleton_t const&) = delete;
  void operator=(const config_singleton_t&) = delete;
  friend const valhalla::property_tree& valhalla::config(const std::string& config_inline);
};
} // namespace

namespace valhalla {

const property_tree& config(const std::string& config_file_or_inline) {
  static config_singleton_t instance(config_file_or_inline);
  return instance.config_;
}

}; // namespace valhalla
