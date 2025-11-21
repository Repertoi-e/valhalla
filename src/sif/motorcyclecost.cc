#include "sif/motorcyclecost.h"
#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/nodeinfo.h"
#include "baldr/rapidjson_utils.h"
#include "proto_conversions.h"
#include "sif/costconstants.h"
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
namespace motorcyclecost_internal {

// Other options
constexpr float kDefaultUseHighways = 0.5f; // Factor between 0 and 1
constexpr float kDefaultUseTolls = 0.5f;    // Factor between 0 and 1
constexpr float kDefaultUseTrails = 0.0f;   // Factor between 0 and 1

constexpr Surface kMinimumMotorcycleSurface = Surface::kImpassable;

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

// Turn costs based on side of street driving
constexpr float kRightSideTurnCosts[] = {kTCStraight,       kTCSlight,  kTCFavorable,
                                         kTCFavorableSharp, kTCReverse, kTCUnfavorableSharp,
                                         kTCUnfavorable,    kTCSlight};
constexpr float kLeftSideTurnCosts[] = {kTCStraight,         kTCSlight,  kTCUnfavorable,
                                        kTCUnfavorableSharp, kTCReverse, kTCFavorableSharp,
                                        kTCFavorable,        kTCSlight};

// Valid ranges and defaults
constexpr ranged_default_t<float> kUseHighwaysRange{0, kDefaultUseHighways, 1.0f};
constexpr ranged_default_t<float> kUseTollsRange{0, kDefaultUseTolls, 1.0f};
constexpr ranged_default_t<float> kUseTrailsRange{0, kDefaultUseTrails, 1.0f};
constexpr ranged_default_t<uint32_t> kMotorcycleSpeedRange{10, baldr::kMaxAssumedSpeed,
                                                           baldr::kMaxSpeedKph};

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

constexpr float kMaxTrailBiasFactor = 8.0f;

constexpr float kSurfaceFactor[] = {
    0.0f, // kPavedSmooth
    0.0f, // kPaved
    0.0f, // kPaveRough
    0.1f, // kCompacted
    0.2f, // kDirt
    0.5f, // kGravel
    1.0f  // kPath
};

BaseCostingOptionsConfig GetBaseCostOptsConfig() {
  BaseCostingOptionsConfig cfg{};
  // override defaults
  cfg.disable_rail_ferry_ = true;
  return cfg;
}

const BaseCostingOptionsConfig kBaseCostOptsConfig = GetBaseCostOptsConfig();

} // namespace motorcyclecost_internal

// Constructor
MotorcycleCost::MotorcycleCost(DynamicCost* parent, const Costing& costing) {
  using namespace motorcyclecost_internal;

  const auto& costing_options = costing.options();

  // Get the base costs
  parent->get_base_costs(costing);

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

  // Set toll factor based on preference to use tolls (value from 0 to 1).
  // Toll factor of 0 would indicate no adjustment to weighting for toll roads.
  // use_tolls = 1 would reduce weighting slightly (a negative delta) while
  // use_tolls = 0 would penalize (positive delta to weighting factor).
  float use_tolls = costing_options.use_tolls();
  toll_factor_ = use_tolls < 0.5f ? (2.0f - 4 * use_tolls) : // ranges from 2 to 0
                     (0.5f - use_tolls) * 0.03f;             // ranges from 0 to -0.015

  // Set the surface factor based on the use trails value - this is a
  // preference to use trails/tracks/bad surface types (a value from 0 to 1).
  float use_trails = costing_options.use_trails();

  // Factor for trail use - use a non-linear factor with values at 0.5 being neutral (factor
  // of 0). Values between 0.5 and 1 slowly decrease to a maximum of -0.125 (to slightly prefer
  // trails) while values between 0.5 to 0 slowly increase to a maximum of the surfact_factor_
  // to avoid/penalize trails.
  // modulates surface factor based on use_trails
  if (use_trails >= 0.5f) {
    float f = (0.5f - use_trails);
    surface_factor_ = f * f * f;
  } else {
    float f = 1.0f - use_trails * 2.0f;
    surface_factor_ = static_cast<uint32_t>(kMaxTrailBiasFactor * (f * f));
  }
}

// Destructor
MotorcycleCost::~MotorcycleCost() {
}

// Check if access is allowed on the specified edge.
bool MotorcycleCost::Allowed(const DynamicCost* parent,
                             const baldr::DirectedEdge* edge,
                             const bool is_dest,
                             const EdgeLabel& pred,
                             const graph_tile_ptr& tile,
                             const baldr::GraphId& edgeid,
                             const uint64_t current_time,
                             const uint32_t tz_index,
                             uint8_t& restriction_idx,
                             uint8_t& destonly_access_restr_mask) const {
  using namespace motorcyclecost_internal;
  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes.
  if (!parent->IsAccessible(edge) ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      ((pred.restrictions() & (1 << edge->localedgeidx())) && !parent->ignore_turn_restrictions_) ||
      (edge->surface() > kMinimumMotorcycleSurface) || parent->IsUserAvoidEdge(edgeid) ||
      (!parent->allow_destination_only_ && !pred.destonly() && edge->destonly()) ||
      (pred.closure_pruning() && parent->IsClosed(edge, tile)) ||
      parent->CheckExclusions(edge, pred)) {
    return false;
  }

  return parent->EvaluateRestrictions(parent->access_mask_, edge, is_dest, tile, edgeid, current_time,
                                      tz_index, restriction_idx, destonly_access_restr_mask);
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool MotorcycleCost::AllowedReverse(const DynamicCost* parent,
                                    const baldr::DirectedEdge* edge,
                                    const EdgeLabel& pred,
                                    const baldr::DirectedEdge* opp_edge,
                                    const graph_tile_ptr& tile,
                                    const baldr::GraphId& opp_edgeid,
                                    const uint64_t current_time,
                                    const uint32_t tz_index,
                                    uint8_t& restriction_idx,
                                    uint8_t& destonly_access_restr_mask) const {
  using namespace motorcyclecost_internal;
  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes.
  if (!parent->IsAccessible(opp_edge) ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      ((opp_edge->restrictions() & (1 << pred.opp_local_idx())) &&
       !parent->ignore_turn_restrictions_) ||
      (opp_edge->surface() > kMinimumMotorcycleSurface) || parent->IsUserAvoidEdge(opp_edgeid) ||
      (!parent->allow_destination_only_ && !pred.destonly() && opp_edge->destonly()) ||
      (pred.closure_pruning() && parent->IsClosed(opp_edge, tile)) ||
      parent->CheckExclusions(opp_edge, pred)) {
    return false;
  }

  return parent->EvaluateRestrictions(parent->access_mask_, opp_edge, false, tile, opp_edgeid,
                                      current_time, tz_index, restriction_idx,
                                      destonly_access_restr_mask);
}

Cost MotorcycleCost::EdgeCost(const DynamicCost* parent,
                              const baldr::DirectedEdge* edge,
                              const baldr::GraphId& edgeid,
                              const graph_tile_ptr& tile,
                              const baldr::TimeInfo& time_info,
                              uint8_t& flow_sources) const {
  using namespace motorcyclecost_internal;
  auto edge_speed = parent->fixed_speed_ == baldr::kDisableFixedSpeed
                        ? tile->GetSpeed(edge, parent->flow_mask_, time_info.second_of_week, false,
                                         &flow_sources, time_info.seconds_from_now)
                        : parent->fixed_speed_;

  auto final_speed = std::min(edge_speed, parent->top_speed_);

  float sec = (edge->length() * kSpeedFactor[final_speed]);

  if (parent->shortest_) {
    return Cost(edge->length(), sec);
  }

  // Special case for travel on a ferry
  if (edge->use() == Use::kFerry) {
    // Use the edge speed (should be the speed of the ferry)
    return {sec * parent->ferry_factor_, sec};
  }

  float factor = kDensityFactor[edge->density()] +
                 highway_factor_ * kHighwayFactor[static_cast<uint32_t>(edge->classification())] +
                 surface_factor_ * kSurfaceFactor[static_cast<uint32_t>(edge->surface())];
  factor += parent->SpeedPenalty(edge, tile, time_info, flow_sources, edge_speed);
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

  factor *= parent->EdgeFactor(edgeid);

  return {sec * factor, sec};
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost MotorcycleCost::TransitionCost(
    const DynamicCost* parent,
    const baldr::DirectedEdge* edge,
    const baldr::NodeInfo* node,
    const EdgeLabel& pred,
    const graph_tile_ptr& /*tile*/,
    const std::function<LimitedGraphReader()>& /*reader_getter*/) const {
  using namespace motorcyclecost_internal;
  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  uint32_t idx = pred.opp_local_idx();
  Cost c = parent->base_transition_cost(node, edge, &pred, idx);
  c.secs += OSRMCarTurnDuration(edge, node, idx);

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

    parent->AddUturnPenalty(idx, node, edge, has_reverse, has_left, has_right, false,
                            InternalTurn::kNoTurn, seconds);

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
Cost MotorcycleCost::TransitionCostReverse(
    const DynamicCost* parent,
    const uint32_t idx,
    const baldr::NodeInfo* node,
    const baldr::DirectedEdge* pred,
    const baldr::DirectedEdge* edge,
    const graph_tile_ptr& /*tile*/,
    const GraphId& /*pred_id*/,
    const std::function<LimitedGraphReader()>& /*reader_getter*/,
    const bool has_measured_speed,
    const InternalTurn /*internal_turn*/) const {
  using namespace motorcyclecost_internal;

  // Motorcycles should be able to make uturns on short internal edges; therefore, InternalTurn
  // is ignored for now.
  // TODO: do we want to update the cost if we have flow or speed from traffic.

  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  Cost c = parent->base_transition_cost(node, edge, pred, idx);
  c.secs += OSRMCarTurnDuration(edge, node, pred->opp_local_idx());

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

    parent->AddUturnPenalty(idx, node, edge, has_reverse, has_left, has_right, false,
                            InternalTurn::kNoTurn, seconds);

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

void ParseMotorcycleCostOptions(const rapidjson::Document& doc,
                                const std::string& costing_options_key,
                                Costing* c) {
  using namespace motorcyclecost_internal;
  c->set_type(Costing::motorcycle);
  c->set_name(Costing_Enum_Name(c->type()));
  auto* co = c->mutable_options();

  rapidjson::Value dummy;
  const auto& json = rapidjson::get_child(doc, costing_options_key.c_str(), dummy);

  ParseBaseCostOptions(json, c, kBaseCostOptsConfig);
  JSON_PBF_RANGED_DEFAULT(co, kUseHighwaysRange, json, "/use_highways", use_highways);
  JSON_PBF_RANGED_DEFAULT(co, kUseTollsRange, json, "/use_tolls", use_tolls);
  JSON_PBF_RANGED_DEFAULT(co, kUseTrailsRange, json, "/use_trails", use_trails);
  JSON_PBF_RANGED_DEFAULT(co, kMotorcycleSpeedRange, json, "/top_speed", top_speed);
}

} // namespace sif
} // namespace valhalla

/**********************************************************************************************/

#ifdef INLINE_TEST_TODO_FIX

using namespace valhalla;
using namespace sif;

namespace {

class TestMotorcycleCost : public MotorcycleCost {
public:
  TestMotorcycleCost(const Costing& costing_options) : MotorcycleCost(costing_options){};

  using MotorcycleCost::alley_penalty_;
  using MotorcycleCost::country_crossing_cost_;
  using MotorcycleCost::destination_only_penalty_;
  using MotorcycleCost::ferry_transition_cost_;
  using MotorcycleCost::gate_cost_;
  using MotorcycleCost::maneuver_penalty_;
  using MotorcycleCost::parent->service_factor_;
  using MotorcycleCost::service_penalty_;
  using MotorcycleCost::toll_booth_cost_;
};

TestMotorcycleCost* make_motorcyclecost_from_json(const std::string& property, float testVal) {
  std::stringstream ss;
  ss << R"({"costing": "motorcycle", "costing_options":{"motorcycle":{")" << property << R"(":)"
     << testVal << "}}}";
  Api request;
  ParseApi(ss.str(), valhalla::Options::route, request);
  return new TestMotorcycleCost(request.options().costings().find((int)Costing::motorcycle)->second);
}

template <typename T>
std::uniform_real_distribution<T>* make_distributor_from_range(const ranged_default_t<T>& range) {
  T rangeLength = range.max - range.min;
  return new std::uniform_real_distribution<T>(range.min - rangeLength, range.max + rangeLength);
}

TEST(MotorcycleCost, testMotorcycleCostParams) {
  using namespace motorcyclecost_internal;
  constexpr unsigned testIterations = 250;
  constexpr unsigned seed = 0;
  std::mt19937 generator(seed);
  std::shared_ptr<std::uniform_real_distribution<float>> distributor;
  std::shared_ptr<TestMotorcycleCost> ctorTester;

  const auto& defaults = kBaseCostOptsConfig;

  // maneuver_penalty_
  distributor.reset(make_distributor_from_range(defaults.maneuver_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorcyclecost_from_json("maneuver_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->maneuver_penalty_,
                test::IsBetween(defaults.maneuver_penalty_.min, defaults.maneuver_penalty_.max));
  }

  // alley_penalty_
  distributor.reset(make_distributor_from_range(defaults.alley_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorcyclecost_from_json("alley_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->alley_penalty_,
                test::IsBetween(defaults.alley_penalty_.min, defaults.alley_penalty_.max));
  }

  // destination_only_penalty_
  distributor.reset(make_distributor_from_range(defaults.dest_only_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_motorcyclecost_from_json("destination_only_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->destination_only_penalty_,
                test::IsBetween(defaults.dest_only_penalty_.min, defaults.dest_only_penalty_.max));
  }

  // gate_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(defaults.gate_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorcyclecost_from_json("gate_cost", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->gate_cost_.secs,
                test::IsBetween(defaults.gate_cost_.min, defaults.gate_cost_.max));
  }

  // gate_penalty_ (Cost.cost)
  distributor.reset(make_distributor_from_range(defaults.gate_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorcyclecost_from_json("gate_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->gate_cost_.cost,
                test::IsBetween(defaults.gate_penalty_.min, defaults.gate_penalty_.max));
  }

  // toll_booth_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(defaults.toll_booth_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorcyclecost_from_json("toll_booth_cost", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->toll_booth_cost_.secs,
                test::IsBetween(defaults.toll_booth_cost_.min, defaults.toll_booth_cost_.max));
  }

  // tollbooth_penalty_ (Cost.cost)
  distributor.reset(make_distributor_from_range(defaults.toll_booth_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorcyclecost_from_json("toll_booth_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->toll_booth_cost_.cost,
                test::IsBetween(defaults.toll_booth_penalty_.min,
                                defaults.toll_booth_penalty_.max + defaults.toll_booth_cost_.def));
  }

  // country_crossing_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(defaults.country_crossing_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_motorcyclecost_from_json("country_crossing_cost", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->country_crossing_cost_.secs,
                test::IsBetween(defaults.country_crossing_cost_.min,
                                defaults.country_crossing_cost_.max));
  }

  // country_crossing_penalty_ (Cost.cost)
  distributor.reset(make_distributor_from_range(defaults.country_crossing_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_motorcyclecost_from_json("country_crossing_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->country_crossing_cost_.cost,
                test::IsBetween(defaults.country_crossing_penalty_.min,
                                defaults.country_crossing_penalty_.max +
                                    defaults.country_crossing_cost_.def));
  }

  // ferry_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(defaults.ferry_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorcyclecost_from_json("ferry_cost", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->ferry_transition_cost_.secs,
                test::IsBetween(defaults.ferry_cost_.min, defaults.ferry_cost_.max));
  }

  // service_penalty_
  distributor.reset(make_distributor_from_range(defaults.service_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorcyclecost_from_json("service_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->service_penalty_,
                test::IsBetween(defaults.service_penalty_.min, defaults.service_penalty_.max));
  }

  // parent->service_factor_
  distributor.reset(make_distributor_from_range(defaults.parent->service_factor_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorcyclecost_from_json("service_factor", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->parent->service_factor_,
                test::IsBetween(defaults.parent->service_factor_.min,
                                defaults.parent->service_factor_.max));
  }

  /*
   // use_ferry
   distributor.reset(make_distributor_from_range(defaults.use_ferry_));
   for (unsigned i = 0; i < testIterations; ++i) {
     ctorTester.reset(make_motorcyclecost_from_json("use_ferry", (*distributor)(generator)));
EXPECT_THAT(ctorTester->use_ferry , test::IsBetween(defaults.use_ferry_.min,
defaults.use_ferry_.max));
   }

    // use_highways
    distributor.reset(make_distributor_from_range(kUseHighwaysRange));
    for (unsigned i = 0; i < testIterations; ++i) {
      ctorTester.reset(make_motorcyclecost_from_json("use_highways", (*distributor)(generator)));
EXPECT_THAT(ctorTester->use_highways , test::IsBetween(kUseHighwaysRange.min, kUseHighwaysRange.max));
    }

     // use_trails
     distributor.reset(make_distributor_from_range(kUseTrailsRange));
     for (unsigned i = 0; i < testIterations; ++i) {
       ctorTester.reset(make_motorcyclecost_from_json("use_trails", (*distributor)(generator)));
EXPECT_THAT(ctorTester->use_trails , test::IsBetween(kUseTrailsRange.min, kUseTrailsRange.max));
     }

   // use_tolls
   distributor.reset(make_distributor_from_range(kUseTollsRange));
   for (unsigned i = 0; i < testIterations; ++i) {
     ctorTester.reset(make_motorcyclecost_from_json("use_tolls", (*distributor)(generator)));
EXPECT_THAT(ctorTester->use_tolls , test::IsBetween(kUseTollsRange.min, kUseTollsRange.max));
   }
   **/
}
} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#endif
