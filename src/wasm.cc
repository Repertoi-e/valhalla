#include <valhalla/property_tree/ptree.hpp>

#include "arche/arche.h"
#undef ref
#undef search        // boost shih
#undef always_inline // boost shih

#include "baldr/rapidjson_utils.h"
#include "baldr/tilegetter.h"
#include "baldr/wasm_tile_getter.h"
#include "config.h"
#include "midgard/logging.h"
#include "odin/util.h"
#include "tyr/actor.h"
#include "valhalla/exceptions.h"

#include <emscripten/bind.h>
#include <emscripten/emscripten.h>

namespace valhalla {

using baldr::tile_getter_t;
using baldr::wasm_tile_getter_t;

odin::locales_singleton_t load_narrative_locals() { return {}; }

std::shared_ptr<valhalla::odin::NarrativeDictionary> load_narrative_locals_for(const std::string& locale_string)
{
    auto module = emscripten::val::global("Module");
    if (module.isNull() || module.isUndefined()) return nullptr;
    auto fn = module["fetchLocale"];
    if (fn.isNull() || fn.isUndefined()) return nullptr;

    auto json_val = module.call<emscripten::val>("fetchLocale", locale_string).await();
    std::string json = json_val.as<std::string>();

    boost::property_tree::ptree narrative_pt;
    std::stringstream ss;
    ss << json;
    rapidjson::read_json(ss, narrative_pt);
    auto narrative_dictionary = std::make_shared<valhalla::odin::NarrativeDictionary>(locale_string, narrative_pt);
    return narrative_dictionary;
}

tile_getter_t::GET_response_t
wasm_tile_getter_t::get(const std::string& url, const uint64_t offset, const uint64_t size) {
    tile_getter_t::GET_response_t response;

    auto module = emscripten::val::global("Module");
    if (module.isNull() || module.isUndefined()) return response;
    auto fn = module["fetchGraphTile"];
    if (fn.isNull() || fn.isUndefined()) return response;
    
    auto tile = module.call<emscripten::val>("fetchGraphTile", url).await();

    response.data_ = (char*)(tile["data"]["byteOffset"].as<size_t>());
    response.size_ = tile["size"].as<uint64_t>();

    // printf("C/ Tile fetched: %s, size: %zu\n", url.c_str(), response.size_);
    uint8_t checksum = 0;
    for (size_t i = 0; i < response.size_; ++i) {
    checksum ^= response.data_[i];
    }
    // printf("C/ Tile checksum: %02x\n", checksum);

    response.status_ = tile_getter_t::status_code_t::SUCCESS;
    response.http_code_ = 200;

    return response;
}
} // namespace valhalla

extern "C" {
void* malloc(size_t size) {
  return (void*)malloc<byte>({.Count = (s64)size});
}

void* calloc(size_t num, size_t size) {
  void* block = malloc(num * size);
  memset0((byte*)block, num * size);
  return block;
}

void* realloc(void* block, size_t newSize) {
  if (!block) {
    return malloc(newSize);
  }
  return (void*)realloc((byte*)block, {.NewCount = (s64)newSize});
}

// No need to define this global function if the library was built without a
// namespace
void free(void* block) {
  free((byte*)block);
}

void* emscripten_builtin_memalign(size_t alignment, size_t size) {
  return (void*)malloc<byte>({.Count = (s64)size, .Alignment = (u32)alignment});
}

void emscripten_builtin_free(void* ptr) {
  free(ptr);
}

void* emscripten_builtin_malloc(size_t size) {
  return malloc(size);
}

void* __libc_malloc(size_t size) {
  return malloc(size);
}
void __libc_free(void* ptr) {
  free(ptr);
}

int posix_memalign(void** memptr, size_t alignment, size_t size) {
  if (alignment & (alignment - 1)) {
    return EINVAL;
  }
  *memptr = emscripten_builtin_memalign(alignment, size);
  return 0;
}
}

namespace emscripten {

val js_malloc(size_t size) {
  return val(typed_memory_view(size, (byte*)malloc<byte>({.Count = (s64)size})));
}

void js_free(val buffer) {
  free((byte*)buffer.as<uintptr_t>());
}

template <class Allocator = std::allocator<bool>>
class_<std::vector<bool, Allocator>> register_vector_bool(const char* name) {
  typedef std::vector<bool, Allocator> VecType;
#if __cplusplus >= 201703L
  register_optional<bool>();
#endif

  void (VecType::*push_back)(const bool&) = &VecType::push_back;
  void (VecType::*resize)(const size_t, bool) = &VecType::resize;
  size_t (VecType::*size)() const = &VecType::size;
  return class_<std::vector<bool>>(name)
      .template constructor<>()
      .function("push_back", push_back, allow_raw_pointers())
      .function("resize", resize, allow_raw_pointers())
      .function("size", size)
      .function("get", &internal::VectorAccess<VecType>::get, allow_raw_pointers())
      .function("set", &internal::VectorAccess<VecType>::set, allow_raw_pointers());
}
} // namespace emscripten

namespace valhalla {
std::string serialize_error(const valhalla_exception_t& exception, Api& request);
bool Options_Action_Enum_Parse(const std::string& action, Options::Action* a);

tyr::actor_t* actor = nullptr;

void init_actor(std::string valhalla_config_json) {
  actor = new valhalla::tyr::actor_t(config(std::string(valhalla_config_json)));
}
} // namespace valhalla


extern "C" EMSCRIPTEN_KEEPALIVE const char* do_request(const char* action_js, const char* request_js) {
  using namespace valhalla;
  // RequestArena.Used = 0; // Reset the arena for this request
  // if (!RequestArena.Block) {
  //   RequestArena.AutomaticBlockSize = 64_MiB;
  // }

  context newContext = Context;
  // newContext.Alloc = {arena_allocator, &RequestArena};
  // newContext.LogAllAllocations = true;

  std::string result;
  Api request;

  try {
    Options::Action action;

    if (!Options_Action_Enum_Parse(action_js, &action)) {
      throw valhalla_exception_t{400};
    }

    switch (action) {
      case Options::route:
        result = actor->route(request_js, nullptr, &request);
        break;
      case Options::locate:
        result = actor->locate(request_js, nullptr, &request);
        break;
      case Options::sources_to_targets:
        result = actor->matrix(request_js, nullptr, &request);
        break;
      case Options::optimized_route:
        result = actor->optimized_route(request_js, nullptr, &request);
        break;
      case Options::isochrone:
        result = actor->isochrone(request_js, nullptr, &request);
        break;
      case Options::trace_route:
        result = actor->trace_route(request_js, nullptr, &request);
        break;
      case Options::trace_attributes:
        result = actor->trace_attributes(request_js, nullptr, &request);
        break;
      case Options::height:
        result = actor->height(request_js, nullptr, &request);
        break;
      case Options::transit_available:
        result = actor->transit_available(request_js, nullptr, &request);
        break;
      case Options::expansion:
        result = actor->expansion(request_js, nullptr, &request);
        break;
      case Options::status:
        result = actor->status(request_js, nullptr, &request);
        break;
      default:
        throw valhalla_exception_t{400};
    }
  } // request processing error specific error condition
  catch (const valhalla_exception_t& ve) {
    result = serialize_error(ve, request);
  } catch (const std::exception& e) {
    result = serialize_error({599, std::string(e.what())}, request);
  } catch (...) { result = serialize_error({599, std::string("Unknown exception thrown")}, request); }

  for (auto& stat : request.info().statistics()) {
    // print("STAT {}: {}\n", stat.key().c_str(), stat.value());
  }

  return strdup(result.c_str()); // @Leak
}

EMSCRIPTEN_BINDINGS(valhalla_bindings) {
  using namespace emscripten;
  using namespace valhalla;

  function("init_actor", &init_actor);

  function("_malloc", &js_malloc);
  function("_free", &js_free);
}
