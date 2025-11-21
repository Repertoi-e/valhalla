#ifndef VALHALLA_SIF_DYNAMICCOST_CONST_H_
#define VALHALLA_SIF_DYNAMICCOST_CONST_H_

#include <valhalla/baldr/accessrestriction.h>
#include <valhalla/baldr/datetime.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/graphtileptr.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/baldr/rapidjson_fwd.h>
#include <valhalla/baldr/time_info.h>
#include <valhalla/baldr/timedomain.h>
#include <valhalla/baldr/transitdeparture.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/sif/costconstants.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/midgard/util.h>

#include <array>
#include <cstdint>
#include <memory>
#include <unordered_map>

// macros aren't great but writing these out for every option is an abomination worse than this macro

/**
 * this macro takes a ranged_default_t and uses it to make sure user provided values (in json or in
 * pbf) are in range before setting it on the costing options
 *
 * @param costing_options  pointer to protobuf costing options object
 * @param range            ranged_default_t object which will check any provided values are in range
 * @param json             rapidjson value object which should contain user provided costing options
 * @param json_key         the json key to use to pull a user provided value out of the jsonn
 * @param option_name      the name of the option will be set on the costing options object
 */

#define JSON_PBF_RANGED_DEFAULT(costing_options, range, json, json_key, option_name)                 \
  {                                                                                                  \
    costing_options->set_##option_name(                                                              \
        range(rapidjson::get<decltype(range.def)>(json, json_key,                                    \
                                                  costing_options->has_##option_name##_case()        \
                                                      ? costing_options->option_name()               \
                                                      : range.def)));                                \
  }

/**
 * same as above, but for costing options without pbf's awful oneof
 *
 * @param costing_options  pointer to protobuf costing options object
 * @param range            ranged_default_t object which will check any provided values are in range
 * @param json             rapidjson value object which should contain user provided costing options
 * @param json_key         the json key to use to pull a user provided value out of the jsonn
 * @param option_name      the name of the option will be set on the costing options object
 */

#define JSON_PBF_RANGED_DEFAULT_V2(costing_options, range, json, json_key, option_name)              \
  {                                                                                                  \
    costing_options->set_##option_name(                                                              \
        range(rapidjson::get<decltype(range.def)>(json, json_key,                                    \
                                                  costing_options->option_name()                     \
                                                      ? costing_options->option_name()               \
                                                      : range.def)));                                \
  }

/**
 * this macro takes a default value and uses it when no user provided values exist (in json or in pbf)
 * to set the option on the costing options object
 *
 * @param costing_options  pointer to protobuf costing options object
 * @param def              the default value which is used when neither json nor pbf is provided
 * @param json             rapidjson value object which should contain user provided costing options
 * @param json_key         the json key to use to pull a user provided value out of the json
 * @param option_name      the name of the option will be set on the costing options object
 */

#define JSON_PBF_DEFAULT(costing_options, def, json, json_key, option_name)                          \
  {                                                                                                  \
    costing_options->set_##option_name(                                                              \
        rapidjson::get<std::remove_cv<std::remove_reference<decltype(def)>::type>::                  \
                           type>(json, json_key,                                                     \
                                 costing_options->has_##option_name##_case()                         \
                                     ? costing_options->option_name()                                \
                                     : def));                                                        \
  }

/**
 * same as above, but for costing options without pbf's awful oneof
 *
 * @param costing_options  pointer to protobuf costing options object
 * @param def              the default value which is used when neither json nor pbf is provided
 * @param json             rapidjson value object which should contain user provided costing options
 * @param json_key         the json key to use to pull a user provided value out of the json
 * @param option_name      the name of the option will be set on the costing options object
 */

#define JSON_PBF_DEFAULT_V2(costing_options, def, json, json_key, option_name)                       \
  {                                                                                                  \
    costing_options->set_##option_name(                                                              \
        rapidjson::get<std::remove_cv<                                                               \
            std::remove_reference<decltype(def)>::type>::type>(json, json_key,                       \
                                                               costing_options->option_name()        \
                                                                   ? costing_options->option_name()  \
                                                                   : def));                          \
  }

namespace valhalla {
namespace sif {

struct cost_edge_t {
  double start{0.};
  double end{1.};
  double factor{1.};
};

struct custom_cost_t {
  std::vector<cost_edge_t> ranges;
  double avg_factor{1.};

  // once ranges are filled up, sort and compute average
  // returns the minimum factor
  double sort_and_find_smallest();
};

// Holds a range plus a default value for that range
template <class T> struct ranged_default_t {
  T min, def, max;

  // Returns the value snapped to the default if outside of the range
  T operator()(const T& value) const {
    if (value < min || value > max) {
      return def;
    }
    return value;
  }
};

class DynamicCost;

const std::unordered_map<Costing::Type, std::vector<Costing::Type>> kCostingTypeMapping{
    {Costing::none_, {Costing::none_}},
    {Costing::bicycle, {Costing::bicycle}},
    {Costing::bus, {Costing::bus}},
    {Costing::motor_scooter, {Costing::motor_scooter}},
    {Costing::multimodal, {Costing::multimodal, Costing::transit, Costing::pedestrian}},
    {Costing::pedestrian, {Costing::pedestrian}},
    {Costing::transit, {Costing::transit, Costing::pedestrian}},
    {Costing::truck, {Costing::truck}},
    {Costing::motorcycle, {Costing::motorcycle}},
    {Costing::taxi, {Costing::taxi}},
    {Costing::auto_, {Costing::auto_}},
    {Costing::bikeshare, {Costing::bikeshare, Costing::pedestrian, Costing::bicycle}},
};

const sif::Cost kNoCost(0.0f, 0.0f);

// Default unit size (seconds) for cost sorting.
constexpr uint32_t kDefaultUnitSize = 1;

// Maximum penalty allowed. Cannot be too high because sometimes one cannot avoid a particular
// attribute or condition to complete a route.
constexpr float kMaxPenalty = 12.0f * midgard::kSecPerHour; // 12 hours

// Maximum ferry penalty (when use_ferry == 0 or use_rail_ferry == 0). Can't make this too large
// since a ferry is sometimes required to complete a route.
constexpr float kMaxFerryPenalty = 6.0f * midgard::kSecPerHour; // 6 hours

// Default uturn costs
constexpr float kTCUnfavorablePencilPointUturn = 15.f;
constexpr float kTCUnfavorableUturn = 600.f;
constexpr float kTCNameInconsistentUturn = 10.f;

// Maximum highway avoidance bias (modulates the highway factors based on road class)
constexpr float kMaxHighwayBiasFactor = 8.0f;

/**
 * Mask values used in the allowed function by loki::reach to control how conservative
 * the decision should be. By default allowed methods will not disallow start/end/simple
 * restrictions/shortcuts and closures are determined by the costing configuration
 */
constexpr uint16_t kDisallowNone = 0x0;
constexpr uint16_t kDisallowStartRestriction = 0x1;
constexpr uint16_t kDisallowEndRestriction = 0x2;
constexpr uint16_t kDisallowSimpleRestriction = 0x4;
constexpr uint16_t kDisallowClosure = 0x8;
constexpr uint16_t kDisallowShortcut = 0x10;

constexpr std::array<float, 253> populate_speedfactor() {
  std::array<float, 253> speedfactor{};
  speedfactor[0] = midgard::kSecPerHour; // TODO - what to make speed=0?
  for (uint32_t s = 1; s <= baldr::kMaxSpeedKph; s++) {
    speedfactor[s] = (midgard::kSecPerHour * 0.001f) / static_cast<float>(s);
  }

  return speedfactor;
}
constexpr std::array<float, 253> kSpeedFactor = populate_speedfactor();

constexpr std::array<float, 16> populate_densityfactor() {
  std::array<float, 16> densityfactor{};
  // Set density factors - used to penalize edges in dense, urban areas
  for (uint32_t d = 0; d < 16; d++) {
    densityfactor[d] = 0.85f + (d * 0.025f);
  }

  return densityfactor;
}
constexpr std::array<float, 16> kDensityFactor = populate_densityfactor(); // Density factor

constexpr std::array<float, 16> kTransDensityFactor = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.1f,
                                                       1.2f, 1.3f, 1.4f, 1.6f, 1.9f, 2.2f,
                                                       2.5f, 2.8f, 3.1f, 3.5f};

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_DYNAMICCOST_CONST_H_