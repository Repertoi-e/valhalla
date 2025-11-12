#include "baldr/rapidjson_utils.h"
#include "baldr/tilegetter.h"
#include "baldr/wasm_tile_getter.h"
#include "config.h"
#include "midgard/logging.h"
#include "odin/util.h"
#include "tyr/actor.h"
#include "valhalla/exceptions.h"

#include <valhalla/property_tree/ptree.hpp>

#include <emscripten/bind.h>
#include <emscripten/emscripten.h>

using namespace valhalla;
using namespace emscripten;

static val g_fetch_locale = val::undefined();
static val g_fetch_tile = val::undefined();
static val g_list_tiles = val::undefined();
static val g_timezone_resolver = val::undefined();
static val g_default_timezone_name = val::undefined();

namespace valhalla {
std::string serialize_error(const valhalla_exception_t& exception, Api& request);
bool Options_Action_Enum_Parse(const std::string& action, Options::Action* a);
} // namespace valhalla

namespace date {

std::string detect_default_timezone_name() {
  if (g_default_timezone_name.isNull() || g_default_timezone_name.isUndefined()) {
    throw valhalla_exception_t{500, "Default timezone name function not initialized"};
  }
  return g_default_timezone_name().as<std::string>();
}

val invoke_timezone_provider(const std::string& tz_name, double epoch_seconds, bool is_local) {
  if (g_timezone_resolver.isNull() || g_timezone_resolver.isUndefined()) {
    throw valhalla_exception_t{500, "Timezone resolver function not initialized"};
  }
  return g_timezone_resolver(val(tz_name), val(epoch_seconds), val(is_local));
}
} // namespace date

using baldr::tile_getter_t;
using baldr::wasm_tile_getter_t;

namespace valhalla{
std::unordered_set<baldr::GraphId> fetch_wasm_tile_manifest() {
  std::unordered_set<baldr::GraphId> tiles;
  try {
    if (g_list_tiles.isNull() || g_list_tiles.isUndefined()) {
      throw valhalla_exception_t{500, "Tile manifest function not initialized"};
    }
    auto js_paths = g_list_tiles();
    const auto length = js_paths["length"].as<unsigned>();
    tiles.reserve(length);
    for (unsigned i = 0; i < length; ++i) {
      auto path_val = js_paths[i];
      if (path_val.isNull() || path_val.isUndefined()) {
        continue;
      }
      try {
        auto path = path_val.as<std::string>();
        tiles.emplace(valhalla::baldr::GraphTile::GetTileId(path));
      } catch (...) {
        // Ignore manifest entries that we cannot parse
      }
    }
  } catch (...) {
    // Surface an empty set if interop fails; callers handle absence gracefully
  }
  return tiles;
}

odin::locales_singleton_t load_narrative_locals() {
  return {};
}

std::shared_ptr<valhalla::odin::NarrativeDictionary>
load_narrative_locales_for(const std::string& locale_string) {
  if (g_fetch_locale.isNull() || g_fetch_locale.isUndefined()) {
    throw valhalla_exception_t{500, "Locale fetch function not initialized"};
  }

  auto json_val = g_fetch_locale(locale_string);
  std::string json = json_val.as<std::string>();

  property_tree narrative_pt;
  std::stringstream ss;
  ss << json;
  rapidjson::read_json(ss, narrative_pt);
  auto narrative_dictionary =
      std::make_shared<valhalla::odin::NarrativeDictionary>(locale_string, narrative_pt);
  return narrative_dictionary;
}
} // namespace valhalla

tile_getter_t::GET_response_t
wasm_tile_getter_t::get(const std::string& url, const uint64_t offset, const uint64_t size) {
  tile_getter_t::GET_response_t response;

  if (g_fetch_tile.isNull() || g_fetch_tile.isUndefined()) {
    throw valhalla_exception_t{500, "Tile fetch function not initialized"};
  }
  auto tile = g_fetch_tile(url);

  response.data_ = (char*)(tile["data"]["byteOffset"].as<size_t>());
  response.size_ = tile["size"].as<uint64_t>();

  // printf("C/ Tile fetched: %s, size: %zu\n", url.c_str(), response.size_);
  /*uint8_t checksum = 0;
  for (size_t i = 0; i < response.size_; ++i) {
    checksum ^= response.data_[i];
  }
  printf("C/ Tile checksum: %02x\n", checksum);*/

  response.status_ = tile_getter_t::status_code_t::SUCCESS;
  response.http_code_ = 200;

  return response;
}

extern "C" {
/*void* malloc(size_t size) {
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
}*/
}

namespace emscripten {
val js_malloc(size_t size) {
  // return val(typed_memory_view(size, (byte*)malloc<byte>({.Count = (s64)size})));
  return val(typed_memory_view(size, (char*)malloc(size)));
}

void js_free(val buffer) {
  free((char*)buffer.as<uintptr_t>());
}
} // namespace emscripten

void these_are_the_js_callbacks(val fetch_locale,
                                val fetch_tile,
                                val list_tiles,
                                val timezone_resolver,
                                val default_timezone_name) {
  g_fetch_locale = fetch_locale;
  g_fetch_tile = fetch_tile;
  g_list_tiles = list_tiles;
  g_timezone_resolver = timezone_resolver;
  g_default_timezone_name = default_timezone_name;
}

tyr::actor_t* actor = nullptr;

void init(std::string valhalla_config_json) {
  actor = new tyr::actor_t(config(valhalla_config_json));
}

void prefetch_locales_for(std::string locale_string) {
  // Just load and it gets cached
  auto _ = odin::get_locales_ensure_narrative_dictionary(std::string{locale_string});
}

std::string do_request(std::string action_js, std::string request_js) {
  // RequestArena.Used = 0; // Reset the arena for this request
  // if (!RequestArena.Block) {
  //   RequestArena.AutomaticBlockSize = 64_MiB;
  // }

  // context newContext = Context;
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
  catch (const valhalla_exception_t& e) {
    // result = serialize_error(e, request);
    result = "exception";
  } catch (const std::exception& e) {
    result = serialize_error({599, std::string(e.what())}, request);
  } catch (...) { result = serialize_error({599, std::string("Unknown exception thrown")}, request); }

  for (auto& stat : request.info().statistics()) {
    // print("STAT {}: {}\n", stat.key().c_str(), stat.value());
  }

  return result; // strdup(result.c_str()); // @Leak
}

EMSCRIPTEN_BINDINGS(valhalla_bindings) {
  function("these_are_the_js_callbacks", &these_are_the_js_callbacks);
  function("init", &init);
  function("prefetch_locales_for", &prefetch_locales_for);
  function("do_request", &do_request);

  function("_malloc", &js_malloc);
  function("_free", &js_free);
}

/*
#include "test.h"
#include <gtest/gtest.h>
#include <date/tz.h>
#include <chrono>


// These tests are intended to run only under the Emscripten/JS timezone shim.
// They validate that Temporal integration (or Intl fallback) produces sane results.
// If Temporal polyfill is present it should expose transitions and ambiguity on DST folds.

using namespace date;
using namespace std::chrono;

namespace {

bool running_in_wasm_stub() {
#if defined(__EMSCRIPTEN__)
  return true;
#else
  return false;
#endif
}

// Helper to get sys_info for specific epoch seconds in a zone.
static sys_info sys_at(const time_zone* tz, std::int64_t epoch_sec) {
  return tz->get_info(sys_seconds{seconds{epoch_sec}});
}

// Attempt to detect a DST transition near a provided epoch range by scanning for offset changes.
static bool find_transition(const time_zone* tz, std::int64_t start, std::int64_t end, std::int64_t&
when, int& before_off, int& after_off) { int prev = (int)sys_at(tz, start).offset.count(); for
(std::int64_t t = start + 3600; t <= end; t += 3600) { // hourly step int cur = (int)sys_at(tz,
t).offset.count(); if (cur != prev) { when = t; before_off = prev; after_off = cur; return true;
    }
    prev = cur;
  }
  return false;
}

} // namespace

TEST(WasmTimezone, BasicOffsetUTC) {
  if (!running_in_wasm_stub()) GTEST_SKIP() << "Not running under Emscripten stub";
  auto* tz = locate_zone("Etc/UTC");
  auto info = sys_at(tz, 0);
  EXPECT_EQ(info.offset.count(), 0) << "UTC offset should be 0";
  EXPECT_TRUE(info.begin <= info.end);
  EXPECT_FALSE(info.abbrev.empty());
}

TEST(WasmTimezone, NonUTCZoneReturnsOffset) {
  if (!running_in_wasm_stub()) GTEST_SKIP();
  auto* tz = locate_zone("America/New_York");
  auto now = std::chrono::system_clock::now();
  auto info = tz->get_info(std::chrono::time_point_cast<seconds>(now));
  // Offset should be a whole number of minutes; multiple of 60 seconds.
  EXPECT_EQ(info.offset.count() % 60, 0) << "Offset should be minute-aligned";
  EXPECT_FALSE(info.abbrev.empty());
}

TEST(WasmTimezone, TransitionDetectionTemporalOrFallback) {
  if (!running_in_wasm_stub()) GTEST_SKIP();
  auto* tz = locate_zone("America/New_York");
  // Choose a range around a typical DST change (March) for recent years.
  // We try current year March 1 through April 15.
  auto ymd = year_month_day{floor<days>(system_clock::now())};
  int yr = (int)ymd.year();
  // Build epoch seconds for target span using civil to sys conversion.
  auto start = sys_days{year{yr}/March/1}.time_since_epoch().count();
  auto end   = sys_days{year{yr}/April/15}.time_since_epoch().count();
  std::int64_t when; int before_off; int after_off;
  bool found = find_transition(tz, start, end, when, before_off, after_off);
  // If Temporal is present we expect at least one transition; Intl fallback might miss it (then we
allow not found).
  // We can't directly check Temporal presence from C++; heuristic: begin != -inf and end != +inf near
'when' if found. if (found) { auto info_before = sys_at(tz, when - 3600); auto info_after  =
sys_at(tz, when + 3600); EXPECT_NE(info_before.offset.count(), info_after.offset.count());
  }
}

TEST(WasmTimezone, AmbiguousFoldHeuristic) {
  if (!running_in_wasm_stub()) GTEST_SKIP();
  auto* tz = locate_zone("America/New_York");
  // Fall transition: around first Sunday in November.
  auto ymd = year_month_day{floor<days>(system_clock::now())};
  int yr = (int)ymd.year();
  // Search Nov 1-15 for offset decrease.
  auto start = sys_days{year{yr}/November/1}.time_since_epoch().count();
  auto end   = sys_days{year{yr}/November/15}.time_since_epoch().count();
  std::int64_t when; int before_off; int after_off;
  bool found = find_transition(tz, start, end, when, before_off, after_off);
  if (!found) GTEST_SKIP() << "No fold transition detected (Intl fallback)";
  // Construct a local_seconds in the hour after transition to probe ambiguity.
  // We can't directly query local_info via public API unless we create local_seconds; emulate by
calling get_info on local_seconds path indirectly.
  // Here we just assert sys_info offset difference suffices.
  EXPECT_GT(before_off, after_off) << "Fall back should reduce offset";
}

// If Temporal implemented: ambiguous/nonexistent local times should be distinguishable. Without
direct local_seconds API exposure here we rely on future extension.

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
*/