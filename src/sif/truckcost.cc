#include "sif/truckcost.h"
#include "baldr/accessrestriction.h"
#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/nodeinfo.h"
#include "baldr/rapidjson_utils.h"
#include "proto_conversions.h"
#include "sif/dynamiccost.h"
#include "sif/osrm_car_duration.h"

#ifdef INLINE_TEST_TODO_FIX
#include "test.h"
#include "worker.h"

#include <random>
#endif

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

// Default options/values
namespace truckcost_internal {

// Base transition costs
// Note: all roads of class "Service, other" are already penalized with low_class_penalty, so for
// generic service roads these penalties will add up
constexpr float kDefaultServicePenalty = 0.0f; // Seconds

// Other options
constexpr float kDefaultLowClassPenalty = 30.0f; // Seconds
constexpr float kDefaultUseTolls = 0.5f;         // Factor between 0 and 1
constexpr float kDefaultUseTracks = 0.f;         // Avoid tracks by default. Factor between 0 and 1
constexpr float kDefaultUseLivingStreets =
    0.f;                                    // Avoid living streets by default. Factor between 0 and 1
constexpr float kDefaultUseHighways = 0.5f; // Factor between 0 and 1

// Default turn costs
constexpr float kTCStraight = 0.5f;
constexpr float kTCSlight = 0.75f;
constexpr float kTCFavorable = 1.0f;
constexpr float kTCFavorableSharp = 1.5f;
constexpr float kTCCrossing = 2.0f;
constexpr float kTCUnfavorable = 2.5f;
constexpr float kTCUnfavorableSharp = 3.5f;
constexpr float kTCReverse = 9.5f;
constexpr float kTCRamp = 1.5f;
constexpr float kTCRoundabout = 0.5f;

// Default truck attributes
constexpr float kDefaultTruckWeight = 21.77f;  // Metric Tons (48,000 lbs)
constexpr float kDefaultTruckAxleLoad = 9.07f; // Metric Tons (20,000 lbs)
constexpr float kDefaultTruckHeight = 4.11f;   // Meters (13 feet 6 inches)
constexpr float kDefaultTruckWidth = 2.6f;     // Meters (102.36 inches)
constexpr float kDefaultTruckLength = 21.64f;  // Meters (71 feet)
constexpr uint32_t kDefaultAxleCount = 5;      // 5 axles for above truck config

// Turn costs based on side of street driving
constexpr float kRightSideTurnCosts[] = {kTCStraight,       kTCSlight,  kTCFavorable,
                                         kTCFavorableSharp, kTCReverse, kTCUnfavorableSharp,
                                         kTCUnfavorable,    kTCSlight};
constexpr float kLeftSideTurnCosts[] = {kTCStraight,         kTCSlight,  kTCUnfavorable,
                                        kTCUnfavorableSharp, kTCReverse, kTCFavorableSharp,
                                        kTCFavorable,        kTCSlight};

// How much to favor truck routes.
constexpr float kTruckRouteFactor = 0.85f;
constexpr float kDefaultUseTruckRoute = 0.0f;
constexpr float kMinNonTruckRouteFactor = 1.0f;

constexpr float kHighwayFactor[] = {
    1.0f, // Motorway
    0.5f, // Trunk
    0.0f, // Primary
    0.0f, // Secondary
    0.0f, // Tertiary
    0.0f, // Unclassified
    0.0f, // Residential
    0.0f  // Service, other
};

constexpr float kSurfaceFactor[] = {
    0.0f, // kPavedSmooth
    0.0f, // kPaved
    0.0f, // kPaveRough
    0.1f, // kCompacted
    0.2f, // kDirt
    0.5f, // kGravel
    1.0f  // kPath
};

// Valid ranges and defaults
constexpr ranged_default_t<float> kLowClassPenaltyRange{0.f, kDefaultLowClassPenalty, kMaxPenalty};
constexpr ranged_default_t<float> kTruckWeightRange{0.f, kDefaultTruckWeight, 100.0f};
constexpr ranged_default_t<float> kTruckAxleLoadRange{0.f, kDefaultTruckAxleLoad, 40.0f};
constexpr ranged_default_t<float> kTruckHeightRange{0.f, kDefaultTruckHeight, 10.0f};
constexpr ranged_default_t<float> kTruckWidthRange{0.f, kDefaultTruckWidth, 10.0f};
constexpr ranged_default_t<float> kTruckLengthRange{0.f, kDefaultTruckLength, 50.0f};
constexpr ranged_default_t<float> kUseTollsRange{0.f, kDefaultUseTolls, 1.0f};
constexpr ranged_default_t<uint32_t> kAxleCountRange{2, kDefaultAxleCount, 20};
constexpr ranged_default_t<float> kUseHighwaysRange{0.f, kDefaultUseHighways, 1.0f};
constexpr ranged_default_t<float> kTopSpeedRange{10.f, kMaxAssumedTruckSpeed, kMaxSpeedKph};
constexpr ranged_default_t<float> kHGVNoAccessRange{0.f, kMaxPenalty, kMaxPenalty};
constexpr ranged_default_t<float> kUseTruckRouteRange{0.f, kDefaultUseTruckRoute, 1.0f};

BaseCostingOptionsConfig GetBaseCostOptsConfig() {
  BaseCostingOptionsConfig cfg{};
  // override defaults
  cfg.service_penalty_.def = kDefaultServicePenalty;
  cfg.use_tracks_.def = kDefaultUseTracks;
  cfg.use_living_streets_.def = kDefaultUseLivingStreets;
  return cfg;
}

const BaseCostingOptionsConfig kBaseCostOptsConfig = GetBaseCostOptsConfig();

} // namespace truckcost_internal

// Constructor
TruckCost::TruckCost(DynamicCost* parent, const Costing& costing) {
  using namespace truckcost_internal;

  const auto& costing_options = costing.options();
  // Get the base costs
  parent->get_base_costs(costing);

  low_class_penalty_ = costing_options.low_class_penalty();
  non_truck_route_factor_ =
      costing_options.use_truck_route() < 0.5f
          ? kMinNonTruckRouteFactor + 2.f * costing_options.use_truck_route()
          : ((kMinNonTruckRouteFactor - 5.f) + 12.f * costing_options.use_truck_route());

  // Get the vehicle attributes
  hazmat_ = costing_options.hazmat();
  weight_ = costing_options.weight();
  axle_load_ = costing_options.axle_load();
  height_ = costing_options.height();
  width_ = costing_options.width();
  length_ = costing_options.length();
  axle_count_ = costing_options.axle_count();

  // Create speed cost table
  // Preference to use highways. Is a value from 0 to 1
  // Factor for highway use - use a non-linear factor with values at 0.5 being neutral (factor
  // of 0). Values between 0.5 and 1 slowly decrease to a maximum of -0.125 (to slightly prefer
  // highways) while values between 0.5 to 0 slowly increase to a maximum of kMaxHighwayBiasFactor
  // to avoid/penalize highways.
  float use_highways = costing_options.use_highways();
  if (use_highways >= 0.5f) {
    float f = (0.5f - use_highways);
    highway_factor_ = f * f * f;
  } else {
    float f = 1.0f - (use_highways * 2.0f);
    highway_factor_ = kMaxHighwayBiasFactor * (f * f);
  }

  // Preference to use toll roads (separate from toll booth penalty). Sets a toll
  // factor. A toll factor of 0 would indicate no adjustment to weighting for toll roads.
  // use_tolls = 1 would reduce weighting slightly (a negative delta) while
  // use_tolls = 0 would penalize (positive delta to weighting factor).
  float use_tolls = costing_options.use_tolls();
  toll_factor_ = use_tolls < 0.5f ? (2.0f - 4 * use_tolls) : // ranges from 2 to 0
                     (0.5f - use_tolls) * 0.03f;             // ranges from 0 to -0.015

  // determine what to do with hgv=no edges
  bool no_hgv_access_penalty_active = !(costing_options.hgv_no_access_penalty() == kMaxPenalty);
  no_hgv_access_penalty_ = no_hgv_access_penalty_active * costing_options.hgv_no_access_penalty();
  // set the access mask to both car & truck if that penalty is active
  parent->access_mask_ = no_hgv_access_penalty_active ? (kAutoAccess | kTruckAccess) : kTruckAccess;
}

// Destructor
TruckCost::~TruckCost() {
}

// Auto costing will allow hierarchy transitions by default.
bool TruckCost::AllowTransitions() const {
  return true;
}

bool TruckCost::ModeSpecificAllowed(const baldr::AccessRestriction& restriction) const {
  switch (restriction.type()) {
    case AccessType::kHazmat:
      if (hazmat_ && !restriction.value()) {
        return false;
      }
      break;
    case AccessType::kMaxAxleLoad:
      if (axle_load_ > static_cast<float>(restriction.value() * 0.01)) {
        return false;
      }
      break;
    case AccessType::kMaxAxles:
      if (axle_count_ > static_cast<uint8_t>(restriction.value())) {
        return false;
      }
      break;
    case AccessType::kMaxHeight:
      if (height_ > static_cast<float>(restriction.value() * 0.01)) {
        return false;
      }
      break;
    case AccessType::kMaxLength:
      if (length_ > static_cast<float>(restriction.value() * 0.01)) {
        return false;
      }
      break;
    case AccessType::kMaxWeight:
      if (weight_ > static_cast<float>(restriction.value() * 0.01)) {
        return false;
      }
      break;
    case AccessType::kMaxWidth:
      if (width_ > static_cast<float>(restriction.value() * 0.01)) {
        return false;
      }
      break;
    default:
      return true;
  };
  return true;
}

// Check if access is allowed on the specified edge.
bool TruckCost::Allowed(const DynamicCost* parent,
                        const baldr::DirectedEdge* edge,
                        const bool is_dest,
                        const EdgeLabel& pred,
                        const graph_tile_ptr& tile,
                        const baldr::GraphId& edgeid,
                        const uint64_t current_time,
                        const uint32_t tz_index,
                        uint8_t& restriction_idx,
                        uint8_t& destonly_access_restr_mask) const {
  // Check access, U-turn, and simple turn restriction.
  if (!parent->IsAccessible(edge) ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      ((pred.restrictions() & (1 << edge->localedgeidx())) && (!parent->ignore_turn_restrictions_)) ||
      edge->surface() == Surface::kImpassable || parent->IsUserAvoidEdge(edgeid) ||
      (!parent->allow_destination_only_ && !pred.destonly() && edge->destonly_hgv()) ||
      (pred.closure_pruning() && parent->IsClosed(edge, tile)) ||
      (parent->exclude_unpaved_ && !pred.unpaved() && edge->unpaved()) ||
      parent->CheckExclusions(edge, pred)) {
    return false;
  }

  return parent->EvaluateRestrictions(parent->access_mask_, edge, is_dest, tile, edgeid, current_time,
                                      tz_index, restriction_idx, destonly_access_restr_mask);
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool TruckCost::AllowedReverse(const DynamicCost* parent,
                               const baldr::DirectedEdge* edge,
                               const EdgeLabel& pred,
                               const baldr::DirectedEdge* opp_edge,
                               const graph_tile_ptr& tile,
                               const baldr::GraphId& opp_edgeid,
                               const uint64_t current_time,
                               const uint32_t tz_index,
                               uint8_t& restriction_idx,
                               uint8_t& destonly_access_restr_mask) const {
  // Check access, U-turn, and simple turn restriction.
  if (!parent->IsAccessible(opp_edge) ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      ((opp_edge->restrictions() & (1 << pred.opp_local_idx())) &&
       !parent->ignore_turn_restrictions_) ||
      opp_edge->surface() == Surface::kImpassable || parent->IsUserAvoidEdge(opp_edgeid) ||
      (!parent->allow_destination_only_ && !pred.destonly() && opp_edge->destonly_hgv()) ||
      (pred.closure_pruning() && parent->IsClosed(opp_edge, tile)) ||
      (parent->exclude_unpaved_ && !pred.unpaved() && opp_edge->unpaved()) ||
      parent->CheckExclusions(opp_edge, pred)) {
    return false;
  }

  return parent->EvaluateRestrictions(parent->access_mask_, opp_edge, false, tile, opp_edgeid,
                                      current_time, tz_index, restriction_idx,
                                      destonly_access_restr_mask);
}

// Get the cost to traverse the edge in seconds
Cost TruckCost::EdgeCost(const DynamicCost* parent,
                         const baldr::DirectedEdge* edge,
                         const baldr::GraphId& edgeid,
                         const graph_tile_ptr& tile,
                         const baldr::TimeInfo& time_info,
                         uint8_t& flow_sources) const {
  using namespace truckcost_internal;
  auto edge_speed = parent->fixed_speed_ == baldr::kDisableFixedSpeed
                        ? tile->GetSpeed(edge, parent->flow_mask_, time_info.second_of_week, true,
                                         &flow_sources, time_info.seconds_from_now)
                        : parent->fixed_speed_;

  auto final_speed =
      std::min(edge_speed, edge->truck_speed() ? std::min(edge->truck_speed(), parent->top_speed_)
                                               : parent->top_speed_);

  float sec = edge->length() * kSpeedFactor[final_speed];

  if (parent->shortest_) {
    return Cost(edge->length(), sec);
  }

  float factor = 1.f;
  switch (edge->use()) {
    case Use::kFerry:
      factor = parent->ferry_factor_;
      break;
    case Use::kRailFerry:
      factor = parent->rail_ferry_factor_;
      break;
    default:
      factor = kDensityFactor[edge->density()] +
               highway_factor_ * kHighwayFactor[static_cast<uint32_t>(edge->classification())] +
               kSurfaceFactor[static_cast<uint32_t>(edge->surface())] +
               parent->SpeedPenalty(edge, tile, time_info, flow_sources, edge_speed);
      break;
  }

  if (edge->truck_route() > 0) {
    factor *= kTruckRouteFactor;
  } else {
    factor *= non_truck_route_factor_;
  }

  if (edge->toll()) {
    factor += toll_factor_;
  }

  if (edge->use() == Use::kTrack) {
    factor *= parent->track_factor_;
  } else if (edge->use() == Use::kLivingStreet) {
    factor *= parent->living_street_factor_;
  } else if (edge->use() == Use::kServiceRoad) {
    factor *= parent->service_factor_;
  }

  if (parent->IsClosed(edge, tile)) {
    // Add a penalty for traversing a closed edge
    factor *= parent->closure_factor_;
  }
  factor *= EdgeFactor(edgeid);

  return {sec * factor, sec};
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost TruckCost::TransitionCost(const DynamicCost* parent,
                               const baldr::DirectedEdge* edge,
                               const baldr::NodeInfo* node,
                               const EdgeLabel& pred,
                               const graph_tile_ptr& /*tile*/,
                               const std::function<LimitedGraphReader()>& /*reader_getter*/) const {
  using namespace truckcost_internal;

  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  uint32_t idx = pred.opp_local_idx();
  Cost c = parent->base_transition_cost(node, edge, &pred, idx);
  c.secs += OSRMCarTurnDuration(edge, node, idx);

  // Penalty to transition onto low class roads.
  if (edge->classification() == baldr::RoadClass::kResidential ||
      edge->classification() == baldr::RoadClass::kServiceOther) {
    c.cost += low_class_penalty_;
  }

  // Penalty if the request wants to avoid hgv=no edges instead of disallowing
  c.cost +=
      no_hgv_access_penalty_ * (pred.has_hgv_access() && !(edge->forwardaccess() & kTruckAccess));

  const auto stopimpact = edge->stopimpact(idx);
  const auto turntype = edge->turntype(idx);
  // Transition time = turncost * stopimpact * densityfactor
  if (stopimpact > 0 && !parent->shortest_) {
    float turn_cost;
    if (edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
      turn_cost = kTCCrossing;
    } else {
      turn_cost = (node->drive_on_right()) ? kRightSideTurnCosts[static_cast<uint32_t>(turntype)]
                                           : kLeftSideTurnCosts[static_cast<uint32_t>(turntype)];
    }

    if ((edge->use() != Use::kRamp && pred.use() == Use::kRamp) ||
        (edge->use() == Use::kRamp && pred.use() != Use::kRamp)) {
      turn_cost += kTCRamp;
      if (edge->roundabout())
        turn_cost += kTCRoundabout;
    }

    float seconds = turn_cost;

    bool has_left =
        (turntype == baldr::Turn::Type::kLeft || turntype == baldr::Turn::Type::kSharpLeft);
    bool has_right =
        (turntype == baldr::Turn::Type::kRight || turntype == baldr::Turn::Type::kSharpRight);
    bool has_reverse = turntype == baldr::Turn::Type::kReverse;
    bool is_turn = has_left || has_right || has_reverse;
    // Separate time and penalty when traffic is present. With traffic, edge speeds account for
    // much of the intersection transition time (TODO - evaluate different elapsed time settings).
    // Still want to add a penalty so routes avoid high cost intersections.
    if (is_turn) {
      seconds *= stopimpact;
    }

    parent->AddUturnPenalty(idx, node, edge, has_reverse, has_left, has_right, true,
                            pred.internal_turn(), seconds);

    // Apply density factor and stop impact penalty if there isn't traffic on this edge or you're not
    // using traffic
    if (!pred.has_measured_speed()) {
      if (!is_turn)
        seconds *= stopimpact;
      seconds *= kTransDensityFactor[node->density()];
    }
    c.cost += seconds;
  }
  return c;
}

// Returns the cost to make the transition from the predecessor edge
// when using a reverse search (from destination towards the origin).
// pred is the opposing current edge in the reverse tree
// edge is the opposing predecessor in the reverse tree
Cost TruckCost::TransitionCostReverse(const DynamicCost* parent,
                                      const uint32_t idx,
                                      const baldr::NodeInfo* node,
                                      const baldr::DirectedEdge* pred,
                                      const baldr::DirectedEdge* edge,
                                      const graph_tile_ptr& /*tile*/,
                                      const GraphId& /*pred_id*/,
                                      const std::function<LimitedGraphReader()>& /*reader_getter*/,
                                      const bool has_measured_speed,
                                      const InternalTurn internal_turn) const {
  using namespace truckcost_internal;

  // TODO: do we want to update the cost if we have flow or speed from traffic.

  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  Cost c = parent->base_transition_cost(node, edge, pred, idx);
  c.secs += OSRMCarTurnDuration(edge, node, pred->opp_local_idx());

  // Penalty to transition onto low class roads.
  if (edge->classification() == baldr::RoadClass::kResidential ||
      edge->classification() == baldr::RoadClass::kServiceOther) {
    c.cost += low_class_penalty_;
  }

  // Penalty if the request wants to avoid hgv=no edges instead of disallowing
  c.cost += no_hgv_access_penalty_ *
            ((pred->forwardaccess() & kTruckAccess) && !(edge->forwardaccess() & kTruckAccess));

  const auto stopimpact = edge->stopimpact(idx);
  const auto turntype = edge->turntype(idx);
  // Transition time = turncost * stopimpact * densityfactor
  if (stopimpact > 0 && !parent->shortest_) {
    float turn_cost;
    if (edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
      turn_cost = kTCCrossing;
    } else {
      turn_cost = (node->drive_on_right()) ? kRightSideTurnCosts[static_cast<uint32_t>(turntype)]
                                           : kLeftSideTurnCosts[static_cast<uint32_t>(turntype)];
    }

    if ((edge->use() != Use::kRamp && pred->use() == Use::kRamp) ||
        (edge->use() == Use::kRamp && pred->use() != Use::kRamp)) {
      turn_cost += kTCRamp;
      if (edge->roundabout())
        turn_cost += kTCRoundabout;
    }

    float seconds = turn_cost;
    bool has_left =
        (turntype == baldr::Turn::Type::kLeft || turntype == baldr::Turn::Type::kSharpLeft);
    bool has_right =
        (turntype == baldr::Turn::Type::kRight || turntype == baldr::Turn::Type::kSharpRight);
    bool has_reverse = turntype == baldr::Turn::Type::kReverse;

    bool is_turn = has_left || has_right || has_reverse;
    // Separate time and penalty when traffic is present. With traffic, edge speeds account for
    // much of the intersection transition time (TODO - evaluate different elapsed time settings).
    // Still want to add a penalty so routes avoid high cost intersections.
    if (is_turn) {
      seconds *= stopimpact;
    }

    parent->AddUturnPenalty(idx, node, edge, has_reverse, has_left, has_right, true, internal_turn,
                            seconds);

    // Apply density factor and stop impact penalty if there isn't traffic on this edge or you're not
    // using traffic
    if (!has_measured_speed) {
      if (!is_turn)
        seconds *= stopimpact;
      seconds *= kTransDensityFactor[node->density()];
    }
    c.cost += seconds;
  }
  return c;
}

void ParseTruckCostOptions(const rapidjson::Document& doc,
                           const std::string& costing_options_key,
                           Costing* c) {
  using namespace truckcost_internal;

  c->set_type(Costing::truck);
  c->set_name(Costing_Enum_Name(c->type()));
  auto* co = c->mutable_options();

  rapidjson::Value dummy;
  const auto& json = rapidjson::get_child(doc, costing_options_key.c_str(), dummy);

  ParseBaseCostOptions(json, c, kBaseCostOptsConfig);
  JSON_PBF_RANGED_DEFAULT(co, kLowClassPenaltyRange, json, "/low_class_penalty", low_class_penalty);
  JSON_PBF_DEFAULT_V2(co, false, json, "/hazmat", hazmat);
  JSON_PBF_RANGED_DEFAULT(co, kTruckWeightRange, json, "/weight", weight);
  JSON_PBF_RANGED_DEFAULT(co, kTruckAxleLoadRange, json, "/axle_load", axle_load);
  JSON_PBF_RANGED_DEFAULT(co, kTruckHeightRange, json, "/height", height);
  JSON_PBF_RANGED_DEFAULT(co, kTruckWidthRange, json, "/width", width);
  JSON_PBF_RANGED_DEFAULT(co, kTruckLengthRange, json, "/length", length);
  JSON_PBF_RANGED_DEFAULT(co, kUseTollsRange, json, "/use_tolls", use_tolls);
  JSON_PBF_RANGED_DEFAULT(co, kUseHighwaysRange, json, "/use_highways", use_highways);
  JSON_PBF_RANGED_DEFAULT_V2(co, kAxleCountRange, json, "/axle_count", axle_count);
  JSON_PBF_RANGED_DEFAULT(co, kTopSpeedRange, json, "/top_speed", top_speed);
  JSON_PBF_RANGED_DEFAULT(co, kHGVNoAccessRange, json, "/hgv_no_access_penalty",
                          hgv_no_access_penalty);
  JSON_PBF_RANGED_DEFAULT_V2(co, kUseTruckRouteRange, json, "/use_truck_route", use_truck_route);
}

} // namespace sif
} // namespace valhalla

/**********************************************************************************************/

#ifdef INLINE_TEST_TODO_FIX

using namespace valhalla;
using namespace sif;

namespace {

class TestTruckCost : public TruckCost {
public:
  TestTruckCost(const Costing& costing_options) : TruckCost(costing_options){};

  using TruckCost::alley_penalty_;
  using TruckCost::country_crossing_cost_;
  using TruckCost::destination_only_penalty_;
  using TruckCost::ferry_transition_cost_;
  using TruckCost::gate_cost_;
  using TruckCost::maneuver_penalty_;
  using TruckCost::parent->service_factor_;
  using TruckCost::service_penalty_;
  using TruckCost::toll_booth_cost_;
};

TestTruckCost* make_truckcost_from_json(const std::string& property, float testVal) {
  std::stringstream ss;
  ss << R"({"costing": "truck", "costing_options":{"truck":{")" << property << R"(":)" << testVal
     << "}}}";
  Api request;
  ParseApi(ss.str(), valhalla::Options::route, request);
  return new TestTruckCost(request.options().costings().find((int)Costing::truck)->second);
}

std::uniform_real_distribution<float>*
make_distributor_from_range(const ranged_default_t<float>& range) {
  float rangeLength = range.max - range.min;
  return new std::uniform_real_distribution<float>(range.min - rangeLength, range.max + rangeLength);
}

TEST(TruckCost, testTruckCostParams) {
  using namespace truckcost_internal;

  constexpr unsigned testIterations = 250;
  constexpr unsigned seed = 0;
  std::mt19937 generator(seed);
  std::shared_ptr<std::uniform_real_distribution<float>> distributor;
  std::shared_ptr<TestTruckCost> ctorTester;

  using namespace truckcost_internal;
  const auto& defaults = kBaseCostOptsConfig;

  // maneuver_penalty_
  distributor.reset(make_distributor_from_range(defaults.maneuver_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("maneuver_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->maneuver_penalty_,
                test::IsBetween(defaults.maneuver_penalty_.min, defaults.maneuver_penalty_.max));
  }

  // alley_penalty_
  distributor.reset(make_distributor_from_range(defaults.alley_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("alley_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->alley_penalty_,
                test::IsBetween(defaults.alley_penalty_.min, defaults.alley_penalty_.max));
  }

  // destination_only_penalty_
  distributor.reset(make_distributor_from_range(defaults.dest_only_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("destination_only_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->destination_only_penalty_,
                test::IsBetween(defaults.dest_only_penalty_.min, defaults.dest_only_penalty_.max));
  }

  // gate_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(defaults.gate_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("gate_cost", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->gate_cost_.secs,
                test::IsBetween(defaults.gate_cost_.min, defaults.gate_cost_.max));
  }

  // gate_penalty_ (Cost.cost)
  distributor.reset(make_distributor_from_range(defaults.gate_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("gate_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->gate_cost_.cost,
                test::IsBetween(defaults.gate_penalty_.min, defaults.gate_penalty_.max));
  }

  // tollbooth_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(defaults.toll_booth_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("toll_booth_cost", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->toll_booth_cost_.secs,
                test::IsBetween(defaults.toll_booth_cost_.min, defaults.toll_booth_cost_.max));
  }

  // tollbooth_penalty_ (Cost.cost)
  distributor.reset(make_distributor_from_range(defaults.toll_booth_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("toll_booth_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->toll_booth_cost_.cost,
                test::IsBetween(defaults.toll_booth_penalty_.min,
                                defaults.toll_booth_penalty_.max + defaults.toll_booth_cost_.def));
  }

  // country_crossing_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(defaults.country_crossing_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("country_crossing_cost", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->country_crossing_cost_.secs,
                test::IsBetween(defaults.country_crossing_cost_.min,
                                defaults.country_crossing_cost_.max));
  }

  // country_crossing_penalty_ (Cost.cost)
  distributor.reset(make_distributor_from_range(defaults.country_crossing_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("country_crossing_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->country_crossing_cost_.cost,
                test::IsBetween(defaults.country_crossing_penalty_.min,
                                defaults.country_crossing_penalty_.max +
                                    defaults.country_crossing_cost_.def));
  }

  // ferry_transition_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(defaults.ferry_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("ferry_cost", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->ferry_transition_cost_.secs,
                test::IsBetween(defaults.ferry_cost_.min, defaults.ferry_cost_.max));
  }

  // low_class_penalty_
  distributor.reset(make_distributor_from_range(kLowClassPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("low_class_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->low_class_penalty_,
                test::IsBetween(kLowClassPenaltyRange.min, kLowClassPenaltyRange.max));
  }

  // service_penalty_
  distributor.reset(make_distributor_from_range(defaults.service_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("service_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->service_penalty_,
                test::IsBetween(defaults.service_penalty_.min, defaults.service_penalty_.max));
  }

  // parent->service_factor_
  distributor.reset(make_distributor_from_range(defaults.parent->service_factor_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("service_factor", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->parent->service_factor_,
                test::IsBetween(defaults.parent->service_factor_.min,
                                defaults.parent->service_factor_.max));
  }

  // weight_
  distributor.reset(make_distributor_from_range(kTruckWeightRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("weight", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->weight_, test::IsBetween(kTruckWeightRange.min, kTruckWeightRange.max));
  }

  // axle_load_
  distributor.reset(make_distributor_from_range(kTruckAxleLoadRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("axle_load", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->axle_load_,
                test::IsBetween(kTruckAxleLoadRange.min, kTruckAxleLoadRange.max));
  }

  // height_
  distributor.reset(make_distributor_from_range(kTruckHeightRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("height", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->height_, test::IsBetween(kTruckHeightRange.min, kTruckHeightRange.max));
  }

  // width_
  distributor.reset(make_distributor_from_range(kTruckWidthRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("width", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->width_, test::IsBetween(kTruckWidthRange.min, kTruckWidthRange.max));
  }

  // length_
  distributor.reset(make_distributor_from_range(kTruckLengthRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("length", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->length_, test::IsBetween(kTruckLengthRange.min, kTruckLengthRange.max));
  }
}
} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#endif
