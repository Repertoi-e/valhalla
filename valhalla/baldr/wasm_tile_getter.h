#pragma once

#ifdef __EMSCRIPTEN__

#include <valhalla/baldr/tilegetter.h>

#include <emscripten/val.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace valhalla {
namespace baldr {

/**
 * Tile getter that delegates IO to JavaScript when running under Emscripten.
 */
class wasm_tile_getter_t final : public tile_getter_t {
public:
  wasm_tile_getter_t() = default;
  ~wasm_tile_getter_t() override = default;

  GET_response_t
  get(const std::string& url, const uint64_t offset = 0, const uint64_t size = 0) override;

  HEAD_response_t head(const std::string&, header_mask_t) override {
    return {};
  }

  bool gzipped() const override {
    return false;
  }
};

} // namespace baldr
} // namespace valhalla

#endif
