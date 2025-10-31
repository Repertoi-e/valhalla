#pragma once

#ifdef __EMSCRIPTEN__

#include "arche/arche.h"

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include <emscripten/val.h>

#include <valhalla/baldr/tilegetter.h>

namespace valhalla {
namespace baldr {

/**
 * Tile getter that delegates IO to JavaScript when running under Emscripten.
 */
class wasm_tile_getter_t final : public tile_getter_t {
public:
  wasm_tile_getter_t() = default;
  ~wasm_tile_getter_t() override = default;

  GET_response_t get(const std::string& url,
                     const uint64_t offset = 0,
                     const uint64_t size = 0) override {
    GET_response_t response;

    auto module = emscripten::val::global("Module");
    if (module.isNull() || module.isUndefined()) {
        return response;
    }
    auto fn = module["fetchGraphTile"];
    if (fn.isNull() || fn.isUndefined()) {
        return response;
    }

    auto tile = module.call<emscripten::val>("fetchGraphTile", url);

    response.data_ = (char*) (tile["data"]["byteOffset"].as<size_t>());
    response.size_ = tile["size"].as<uint64_t>();
    response.status_ = tile_getter_t::status_code_t::SUCCESS;
    response.http_code_ = 200;

    return response;
  }

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
