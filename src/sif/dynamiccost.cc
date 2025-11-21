#include "sif/dynamiccost.h"
#include "baldr/graphconstants.h"
#include "baldr/rapidjson_utils.h"
#include "exceptions.h"
#include "proto_conversions.h"
#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/dynamiccost.h"
#include "sif/hierarchylimits.h"
#include "sif/motorcyclecost.h"
#include "sif/motorscootercost.h"
#include "sif/nocost.h"
#include "sif/pedestriancost.h"
#include "sif/transitcost.h"
#include "sif/truckcost.h"

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {

constexpr double kMinCustomFactor = std::numeric_limits<double>::epsilon();

uint8_t SpeedMask_Parse(const std::optional<const rapidjson::Value*>& speed_types) {
  static const std::unordered_map<std::string, uint8_t> types{
      {"freeflow", kFreeFlowMask},
      {"constrained", kConstrainedFlowMask},
      {"predicted", kPredictedFlowMask},
      {"current", kCurrentFlowMask},
  };

  if (!speed_types)
    return kDefaultFlowMask;

  bool had_value = false;
  uint8_t mask = 0;
  if ((*speed_types)->IsArray()) {
    had_value = true;
    for (const auto& speed_type : (*speed_types)->GetArray()) {
      if (speed_type.IsString()) {
        auto i = types.find(speed_type.GetString());
        if (i != types.cend()) {
          mask |= i->second;
        }
      }
    }
  }

  return had_value ? mask : kDefaultFlowMask;
}

} // namespace

namespace valhalla {
namespace sif {

// default options/parameters
namespace {

// max penalty to apply when use tracks
constexpr float kMaxTrackPenalty = 300.f; // 5 min

// min and max factors to apply when use tracks
constexpr float kMinTrackFactor = 0.8f;
constexpr float kMaxTrackFactor = 4.f;

// max penalty to apply when use living streets
constexpr float kMaxLivingStreetPenalty = 500.f;

// min and max factors to apply when use living streets
constexpr float kMinLivingStreetFactor = 0.8f;
constexpr float kMaxLivingStreetFactor = 3.f;

// min factor to apply when use lit
constexpr float kMinLitFactor = 1.f;

constexpr float kMinFactor = 0.1f;
constexpr float kMaxFactor = 100000.0f;

// Base transition costs
constexpr float kDefaultDestinationOnlyPenalty = 600.0f; // Seconds
constexpr float kDefaultManeuverPenalty = 5.0f;          // Seconds
constexpr float kDefaultAlleyPenalty = 5.0f;             // Seconds
constexpr float kDefaultGateCost = 30.0f;                // Seconds
constexpr float kDefaultGatePenalty = 300.0f;            // Seconds
constexpr float kDefaultPrivateAccessPenalty = 450.0f;   // Seconds
constexpr float kDefaultTollBoothCost = 15.0f;           // Seconds
constexpr float kDefaultTollBoothPenalty = 0.0f;         // Seconds
constexpr float kDefaultFerryCost = 300.0f;              // Seconds
constexpr float kDefaultRailFerryCost = 300.0f;          // Seconds
constexpr float kDefaultCountryCrossingCost = 600.0f;    // Seconds
constexpr float kDefaultCountryCrossingPenalty = 0.0f;   // Seconds
constexpr float kDefaultServicePenalty = 15.0f;          // Seconds

// Other options
constexpr float kDefaultUseFerry = 0.5f;         // Default preference of using a ferry 0-1
constexpr float kDefaultUseRailFerry = 0.4f;     // Default preference of using a rail ferry 0-1
constexpr float kDefaultUseTracks = 0.5f;        // Default preference of using tracks 0-1
constexpr float kDefaultUseLivingStreets = 0.1f; // Default preference of using living streets 0-1
constexpr float kDefaultUseLit = 0.f;            // Default preference of using lit ways 0-1

// How much to avoid generic service roads.
constexpr float kDefaultServiceFactor = 1.0f;

// Default penalty factor for avoiding closures (increases the cost of an edge as if its being
// traversed at kMinSpeedKph)
constexpr float kDefaultClosureFactor = 9.0f;
// Default range of closure factor to use for closed edges. Min is set to 1.0, which means do not
// penalize closed edges. The max is set to 10.0 in order to limit how much expansion occurs from the
// non-closure end
constexpr ranged_default_t<float> kClosureFactorRange{1.0f, kDefaultClosureFactor, 10.0f};

constexpr ranged_default_t<uint32_t> kFixedSpeedRange{0, baldr::kDisableFixedSpeed,
                                                      baldr::kMaxSpeedKph};
} // namespace

/**
 * When all ranges for a given edge are added, sort by range start
 * and return the smallest factor found for this edge.
 */
double custom_cost_t::sort_and_find_smallest() {
  if (ranges.empty())
    return 1.;

  std::sort(ranges.begin(), ranges.end(),
            [](const cost_edge_t& a, const cost_edge_t& b) { return a.start < b.start; });

  // keep track of how much of the edge is
  // not covered by ranges
  double uncovered = 1.;
  double avg = 0.;
  double min_factor = 1.;
  for (const auto& range : ranges) {
    uncovered -= range.end - range.start;
    avg += (range.end - range.start) * range.factor;
    min_factor = std::min(min_factor, range.factor);
  }
  avg += uncovered * 1.;
  avg_factor = std::max(avg, kMinCustomFactor);
  return std::max(min_factor, kMinCustomFactor);
}

/*
 * Assign default values for costing options in constructor. In case of different
 * default values they should be overridden in "<type>cost.cc" file.
 */
BaseCostingOptionsConfig::BaseCostingOptionsConfig()
    : dest_only_penalty_{0.f, kDefaultDestinationOnlyPenalty, kMaxPenalty},
      maneuver_penalty_{0.f, kDefaultManeuverPenalty, kMaxPenalty},
      alley_penalty_{0.f, kDefaultAlleyPenalty, kMaxPenalty},
      gate_cost_{0.f, kDefaultGateCost, kMaxPenalty},
      gate_penalty_{0.f, kDefaultGatePenalty, kMaxPenalty},
      private_access_penalty_{0.f, kDefaultPrivateAccessPenalty, kMaxPenalty},
      country_crossing_cost_{0.f, kDefaultCountryCrossingCost, kMaxPenalty},
      country_crossing_penalty_{0.f, kDefaultCountryCrossingPenalty, kMaxPenalty},
      toll_booth_cost_{0.f, kDefaultTollBoothCost, kMaxPenalty},
      toll_booth_penalty_{0.f, kDefaultTollBoothPenalty, kMaxPenalty},
      ferry_cost_{0.f, kDefaultFerryCost, kMaxPenalty}, use_ferry_{0.f, kDefaultUseFerry, 1.f},
      rail_ferry_cost_{0.f, kDefaultRailFerryCost, kMaxPenalty},
      use_rail_ferry_{0.f, kDefaultUseRailFerry, 1.f},
      service_penalty_{0.f, kDefaultServicePenalty, kMaxPenalty},
      service_factor_{kMinFactor, kDefaultServiceFactor, kMaxFactor},
      use_tracks_{0.f, kDefaultUseTracks, 1.f},
      use_living_streets_{0.f, kDefaultUseLivingStreets, 1.f}, use_lit_{0.f, kDefaultUseLit, 1.f},
      closure_factor_{kClosureFactorRange}, exclude_unpaved_(false), exclude_bridges_(false),
      exclude_tunnels_(false), exclude_tolls_(false), exclude_highways_(false),
      exclude_ferries_(false), has_excludes_(false), exclude_cash_only_tolls_(false) {
}

DynamicCost::DynamicCost(const Costing& costing,
                         const TravelMode mode,
                         uint32_t access_mask,
                         bool penalize_uturns)
    : costing_type_(costing.type()), pass_(0), allow_transit_connections_(false),
      allow_destination_only_(true), allow_conditional_destination_(false), travel_mode_(mode),
      access_mask_(access_mask), closure_factor_(kDefaultClosureFactor), flow_mask_(kDefaultFlowMask),
      shortest_(costing.options().shortest()),
      ignore_restrictions_(costing.options().ignore_restrictions()),
      ignore_non_vehicular_restrictions_(costing.options().ignore_non_vehicular_restrictions()),
      ignore_turn_restrictions_(costing.options().ignore_restrictions() ||
                                costing.options().ignore_non_vehicular_restrictions()),
      ignore_oneways_(costing.options().ignore_oneways()),
      ignore_access_(costing.options().ignore_access()),
      ignore_closures_(costing.options().ignore_closures()),
      ignore_construction_(costing.options().ignore_construction()),
      top_speed_(costing.options().top_speed()), fixed_speed_(costing.options().fixed_speed()),
      filter_closures_(ignore_closures_ ? false : costing.filter_closures()),
      penalize_uturns_(penalize_uturns), min_linear_cost_factor_(1.) {

  // set user supplied hierarchy limits if present, fill the other
  // required levels up with sentinel values (clamping to config supplied limits/defaults is handled
  // by thor worker)
  for (const auto& level : TileHierarchy::levels()) {
    const auto& res = costing.options().hierarchy_limits().find(level.level);
    if (res == costing.options().hierarchy_limits().end()) {
      HierarchyLimits hl;
      hl.set_expand_within_dist(kMaxDistance);
      hl.set_max_up_transitions(kUnlimitedTransitions);
      hl.set_up_transition_count(0);
      hierarchy_limits_.push_back(hl);
    } else {
      hierarchy_limits_.push_back(res->second);
      // for internal use only
      hierarchy_limits_.back().set_up_transition_count(0);
    }
  }

  // Add avoid edges to internal set
  for (auto& edge : costing.options().exclude_edges()) {
    user_exclude_edges_.insert({GraphId(edge.id()), edge.percent_along()});
  }

  switch (costing_type_) {
    case Costing::auto_:
    case Costing::bus:
    case Costing::taxi:
      auto_cost_ = AutoCost(this, costing);
      break;
    case Costing::bicycle:
      bicycle_cost_ = BicycleCost(this, costing);
      break;
    case Costing::motorcycle:
      motorcycle_cost_ = MotorcycleCost(this, costing);
      break;
    case Costing::motor_scooter:
      motor_scooter_cost_ = MotorScooterCost(this, costing);
      break;
    case Costing::pedestrian:
    case Costing::bikeshare:
      pedestrian_cost_ = PedestrianCost(this, costing);
      break;
    case Costing::transit:
      transit_cost_ = TransitCost(this, costing);
      break;
    case Costing::truck:
      truck_cost_ = TruckCost(this, costing);
      break;
    case Costing::none_:
    case Costing::multimodal:
      no_cost_ = NoCost();
  }

  // add linear feature factors
  for (auto& e : costing.options().cost_factor_edges()) {
    // short-circuit the ones with factor 0 by putting them on the exclude pile
    if (e.factor() == 0.) {
      user_exclude_edges_.insert({static_cast<GraphId>(e.id()), e.start()});
      break;
    }
    auto& cost_edge = linear_cost_edges_[static_cast<GraphId>(e.id())];
    cost_edge.ranges.push_back({e.start(), e.end(), e.factor()});
  }

  // once all cost factors are filled, sort by range, precompute overall average
  // and store the overall minimum factor so it won't mess with the A* heuristic
  for (auto& [edge, cost_factors] : linear_cost_edges_) {
    min_linear_cost_factor_ =
        std::min(min_linear_cost_factor_, cost_factors.sort_and_find_smallest());
  }
}

DynamicCost::~DynamicCost() {
}

// limits). Defaults to false. Costing methods that wish to allow multiple
// passes with relaxed hierarchy transitions must override this method.
bool DynamicCost::AllowMultiPass() const {
  switch (costing_type_) {
    case Costing::auto_:
    case Costing::taxi:
    case Costing::bus:
    case Costing::motorcycle:
    case Costing::motor_scooter:
    case Costing::pedestrian:
    case Costing::bikeshare:
    case Costing::truck:
      return true;
    default:
      return false;
  }
}

// We provide a convenience method for those algorithms which dont have time components or aren't
// using them for the current route. Here we just call out to the derived classes costing function
// with a time that tells the function that we aren't using time. This avoids having to worry about
// default parameters and inheritance (which are a bad mix)
Cost DynamicCost::EdgeCost(const baldr::DirectedEdge* edge,
                           const baldr::GraphId& edgeid,
                           const graph_tile_ptr& tile) const {
  uint8_t flow_sources;
  return EdgeCost(edge, edgeid, tile, TimeInfo::invalid(), flow_sources);
}

void DynamicCost::AddUturnPenalty(const uint32_t idx,
                                  const baldr::NodeInfo* node,
                                  const baldr::DirectedEdge* edge,
                                  const bool has_reverse,
                                  const bool has_left,
                                  const bool has_right,
                                  const bool penalize_internal_uturns,
                                  const InternalTurn internal_turn,
                                  float& seconds) const {
  if (node->drive_on_right()) {
    // Did we make a uturn on a short, internal edge or did we make a uturn at a node.
    if (has_reverse ||
        (penalize_internal_uturns && internal_turn == InternalTurn::kLeftTurn && has_left))
      seconds += kTCUnfavorableUturn;
    // Did we make a pencil point uturn?
    else if (edge->turntype(idx) == baldr::Turn::Type::kSharpLeft && edge->edge_to_right(idx) &&
             !edge->edge_to_left(idx) && edge->named() && edge->name_consistency(idx))
      seconds *= kTCUnfavorablePencilPointUturn;
  } else {
    // Did we make a uturn on a short, internal edge or did we make a uturn at a node.
    if (has_reverse ||
        (penalize_internal_uturns && internal_turn == InternalTurn::kRightTurn && has_right))
      seconds += kTCUnfavorableUturn;
    // Did we make a pencil point uturn?
    else if (edge->turntype(idx) == baldr::Turn::Type::kSharpRight && !edge->edge_to_right(idx) &&
             edge->edge_to_left(idx) && edge->named() && edge->name_consistency(idx))
      seconds *= kTCUnfavorablePencilPointUturn;
  }
}

// Returns the transfer cost between 2 transit stops.
Cost DynamicCost::TransferCost() const {
  if (costing_type_ == Costing::transit) {
    return transit_cost_.TransferCost();
  }
  return {0.0f, 0.0f};
}

// Returns the default transfer cost between 2 transit stops.
Cost DynamicCost::DefaultTransferCost() const {
  if (costing_type_ == Costing::transit) {
    return transit_cost_.DefaultTransferCost();
  }
  return {0.0f, 0.0f};
}

// Get the general unit size that can be considered as equal for sorting
// purposes. Defaults to 1 (second).
uint32_t DynamicCost::UnitSize() const {
  if (costing_type_ == Costing::transit) {
    return transit_cost_.UnitSize();
  }
  return kDefaultUnitSize;
}

// Set to allow use of transit connections.
void DynamicCost::SetAllowTransitConnections(const bool allow) {
  allow_transit_connections_ = allow;
}

// Sets the flag indicating whether destination only edges are allowed.
void DynamicCost::set_allow_destination_only(const bool allow) {
  allow_destination_only_ = allow;
}

// Sets the flag indicating whether edges with valid restriction conditional=destination are allowed.
void DynamicCost::set_allow_conditional_destination(const bool allow) {
  allow_conditional_destination_ = allow;
}

// Returns the maximum transfer distance between stops that you are willing
// to travel for this mode.  It is the max distance you are willing to
// travel between transfers.
uint32_t DynamicCost::GetMaxTransferDistanceMM() {
  if (costing_type_ == Costing::pedestrian || costing_type_ == Costing::bikeshare) {
    return pedestrian_cost_.GetMaxTransferDistanceMM();
  }
  return 0;
}

// This method overrides the factor for this mode.  The lower the value
// the more the mode is favored.
float DynamicCost::GetModeFactor() {
  if (costing_type_ == Costing::pedestrian || costing_type_ == Costing::bikeshare) {
    return pedestrian_cost_.GetModeFactor();
  } else if (costing_type_ == Costing::transit) {
    return transit_cost_.GetModeFactor();
  }
  return 1.0f;
}

bool DynamicCost::Allowed(const baldr::DirectedEdge* edge,
                          const bool is_dest,
                          const EdgeLabel& pred,
                          const baldr::graph_tile_ptr& tile,
                          const baldr::GraphId& edgeid,
                          const uint64_t current_time,
                          const uint32_t tz_index,
                          uint8_t& restriction_idx,
                          uint8_t& destonly_access_restr_mask) const {
  switch (costing_type_) {
    case Costing::auto_:
    case Costing::bus:
    case Costing::taxi:
      return auto_cost_.Allowed(this, edge, is_dest, pred, tile, edgeid, current_time, tz_index,
                                restriction_idx, destonly_access_restr_mask);
    case Costing::bicycle:
      return bicycle_cost_.Allowed(this, edge, is_dest, pred, tile, edgeid, current_time, tz_index,
                                   restriction_idx, destonly_access_restr_mask);
    case Costing::motorcycle:
      return motorcycle_cost_.Allowed(this, edge, is_dest, pred, tile, edgeid, current_time, tz_index,
                                      restriction_idx, destonly_access_restr_mask);
    case Costing::motor_scooter:
      return motor_scooter_cost_.Allowed(this, edge, is_dest, pred, tile, edgeid, current_time,
                                         tz_index, restriction_idx, destonly_access_restr_mask);
    case Costing::pedestrian:
    case Costing::bikeshare:
      return pedestrian_cost_.Allowed(this, edge, is_dest, pred, tile, edgeid, current_time, tz_index,
                                      restriction_idx, destonly_access_restr_mask);
    case Costing::transit:
      return transit_cost_.Allowed(this, edge, is_dest, pred, tile, edgeid, current_time, tz_index,
                                   restriction_idx, destonly_access_restr_mask);
    case Costing::truck:
      return truck_cost_.Allowed(this, edge, is_dest, pred, tile, edgeid, current_time, tz_index,
                                 restriction_idx, destonly_access_restr_mask);
    case Costing::none_:
    case Costing::multimodal:
      return no_cost_.Allowed(this, edge, is_dest, pred, tile, edgeid, current_time, tz_index,
                              restriction_idx, destonly_access_restr_mask);
  }

  throw std::runtime_error("Allowed not implemented for costing type");
}

bool DynamicCost::AllowedReverse(const baldr::DirectedEdge* edge,
                                 const EdgeLabel& pred,
                                 const baldr::DirectedEdge* opp_edge,
                                 const baldr::graph_tile_ptr& tile,
                                 const baldr::GraphId& edgeid,
                                 const uint64_t current_time,
                                 const uint32_t tz_index,
                                 uint8_t& restriction_idx,
                                 uint8_t& destonly_access_restr_mask) const {
  switch (costing_type_) {
    case Costing::auto_:
    case Costing::bus:
    case Costing::taxi:
      return auto_cost_.AllowedReverse(this, edge, pred, opp_edge, tile, edgeid, current_time,
                                       tz_index, restriction_idx, destonly_access_restr_mask);
    case Costing::bicycle:
      return bicycle_cost_.AllowedReverse(this, edge, pred, opp_edge, tile, edgeid, current_time,
                                          tz_index, restriction_idx, destonly_access_restr_mask);
    case Costing::motorcycle:
      return motorcycle_cost_.AllowedReverse(this, edge, pred, opp_edge, tile, edgeid, current_time,
                                             tz_index, restriction_idx, destonly_access_restr_mask);
    case Costing::motor_scooter:
      return motor_scooter_cost_.AllowedReverse(this, edge, pred, opp_edge, tile, edgeid,
                                                current_time, tz_index, restriction_idx,
                                                destonly_access_restr_mask);
    case Costing::pedestrian:
    case Costing::bikeshare:
      return pedestrian_cost_.AllowedReverse(this, edge, pred, opp_edge, tile, edgeid, current_time,
                                             tz_index, restriction_idx, destonly_access_restr_mask);
    case Costing::transit:
      return transit_cost_.AllowedReverse(this, edge, pred, opp_edge, tile, edgeid, current_time,
                                          tz_index, restriction_idx, destonly_access_restr_mask);
    case Costing::truck:
      return truck_cost_.AllowedReverse(this, edge, pred, opp_edge, tile, edgeid, current_time,
                                        tz_index, restriction_idx, destonly_access_restr_mask);
    case Costing::none_:
    case Costing::multimodal:
      return no_cost_.AllowedReverse(this, edge, pred, opp_edge, tile, edgeid, current_time, tz_index,
                                     restriction_idx, destonly_access_restr_mask);
  }

  throw std::runtime_error("AllowedReverse not implemented for costing type");
}

Cost DynamicCost::EdgeCost(const baldr::DirectedEdge* edge,
                           const baldr::TransitDeparture* departure,
                           const uint32_t curr_time) const {
  switch (costing_type_) {
    case Costing::transit:
      return transit_cost_.EdgeCost(this, edge, departure, curr_time);
    default:
      throw std::runtime_error("Costing does not implement transit edge costs");
  }
  throw std::runtime_error("EdgeCost not implemented for costing type");
}

Cost DynamicCost::EdgeCost(const baldr::DirectedEdge* edge,
                           const baldr::graph_tile_ptr& tile,
                           const baldr::TimeInfo& time_info,
                           uint8_t& flow_sources) const {
  switch (costing_type_) {
    case Costing::auto_:
    case Costing::bus:
    case Costing::taxi:
      return auto_cost_.EdgeCost(this, edge, tile, time_info, flow_sources);
    case Costing::bicycle:
      return bicycle_cost_.EdgeCost(this, edge, tile, time_info, flow_sources);
    case Costing::motorcycle:
      return motorcycle_cost_.EdgeCost(this, edge, tile, time_info, flow_sources);
    case Costing::motor_scooter:
      return motor_scooter_cost_.EdgeCost(this, edge, tile, time_info, flow_sources);
    case Costing::pedestrian:
    case Costing::bikeshare:
      return pedestrian_cost_.EdgeCost(this, edge, tile, time_info, flow_sources);
    case Costing::transit:
      throw std::runtime_error("Transit costing does not implement non-transit edge costs");
    case Costing::truck:
      return truck_cost_.EdgeCost(this, edge, tile, time_info, flow_sources);
    case Costing::none_:
    case Costing::multimodal:
      return no_cost_.EdgeCost(this, edge, tile, time_info, flow_sources);
  }
  throw std::runtime_error("EdgeCost not implemented for costing type");
}

Cost DynamicCost::TransitionCost(
    const baldr::DirectedEdge* edge,
    const baldr::NodeInfo* node,
    const EdgeLabel& pred,
    const baldr::graph_tile_ptr& tile,
    const std::function<baldr::LimitedGraphReader()>& reader_getter) const {

  switch (costing_type_) {
    case Costing::auto_:
    case Costing::bus:
    case Costing::taxi:
      return auto_cost_.TransitionCost(this, edge, node, pred, tile, reader_getter);
    case Costing::bicycle:
      return bicycle_cost_.TransitionCost(this, edge, node, pred, tile, reader_getter);
    case Costing::motorcycle:
      return motorcycle_cost_.TransitionCost(this, edge, node, pred, tile, reader_getter);
    case Costing::motor_scooter:
      return motor_scooter_cost_.TransitionCost(this, edge, node, pred, tile, reader_getter);
    case Costing::pedestrian:
    case Costing::bikeshare:
      return pedestrian_cost_.TransitionCost(this, edge, node, pred, tile, reader_getter);
    case Costing::transit:
      return transit_cost_.TransitionCost(this, edge, node, pred, tile, reader_getter);
    case Costing::truck:
      return truck_cost_.TransitionCost(this, edge, node, pred, tile, reader_getter);
    case Costing::none_:
    case Costing::multimodal:
      return {};
  }
  throw std::runtime_error("TransitionCost not implemented for costing type");
}

Cost DynamicCost::TransitionCostReverse(
    const uint32_t idx,
    const baldr::NodeInfo* node,
    const baldr::DirectedEdge* opp_edge,
    const baldr::DirectedEdge* opp_pred_edge,
    const baldr::graph_tile_ptr& tile,
    const baldr::GraphId& pred_id,
    const std::function<baldr::LimitedGraphReader()>& reader_getter,
    const bool has_measured_speed,
    const InternalTurn internal_turn) const {
  switch (costing_type_) {
    case Costing::auto_:
    case Costing::bus:
    case Costing::taxi:
      return auto_cost_.TransitionCostReverse(this, idx, node, opp_edge, opp_pred_edge, tile, pred_id,
                                              reader_getter, has_measured_speed, internal_turn);
    case Costing::bicycle:
      return bicycle_cost_.TransitionCostReverse(this, idx, node, opp_edge, opp_pred_edge, tile,
                                                 pred_id, reader_getter, has_measured_speed,
                                                 internal_turn);
    case Costing::motorcycle:
      return motorcycle_cost_.TransitionCostReverse(this, idx, node, opp_edge, opp_pred_edge, tile,
                                                    pred_id, reader_getter, has_measured_speed,
                                                    internal_turn);
    case Costing::motor_scooter:
      return motor_scooter_cost_.TransitionCostReverse(this, idx, node, opp_edge, opp_pred_edge, tile,
                                                       pred_id, reader_getter, has_measured_speed,
                                                       internal_turn);
    case Costing::pedestrian:
    case Costing::bikeshare:
      return pedestrian_cost_.TransitionCostReverse(this, idx, node, opp_edge, opp_pred_edge, tile,
                                                    pred_id, reader_getter, has_measured_speed,
                                                    internal_turn);
    case Costing::truck:
      return truck_cost_.TransitionCostReverse(this, idx, node, opp_edge, opp_pred_edge, tile,
                                               pred_id, reader_getter, has_measured_speed,
                                               internal_turn);
    case Costing::transit:
    case Costing::none_:
    case Costing::multimodal:
      break;
  }

  return {0.0f, 0.0f};
}

float DynamicCost::AStarCostFactor() const {
  switch (costing_type_) {
    case Costing::auto_:
    case Costing::bus:
    case Costing::taxi:
    case Costing::truck:
    case Costing::motorcycle:
    case Costing::motor_scooter:
      return kSpeedFactor[top_speed_] * min_linear_cost_factor_;
    case Costing::bicycle:
      // Assume max speed of 2 * the average speed set for costing
      return kSpeedFactor[static_cast<uint32_t>(2 * bicycle_cost_.speed_)];
    case Costing::pedestrian:
    case Costing::bikeshare:
      // On first pass use the walking speed plus a small factor to account for
      // favoring walkways, on the second pass use the the maximum ferry speed.
      if (pass_ == 0) {

        // Determine factor based on all of the factor options
        float factor = 1.f;
        if (pedestrian_cost_.walkway_factor_ < 1.f) {
          factor *= pedestrian_cost_.walkway_factor_;
        }
        if (pedestrian_cost_.sidewalk_factor_ < 1.f) {
          factor *= pedestrian_cost_.sidewalk_factor_;
        }
        if (pedestrian_cost_.alley_factor_ < 1.f) {
          factor *= pedestrian_cost_.alley_factor_;
        }
        if (pedestrian_cost_.driveway_factor_ < 1.f) {
          factor *= pedestrian_cost_.driveway_factor_;
        }
        if (track_factor_ < 1.f) {
          factor *= track_factor_;
        }
        if (living_street_factor_ < 1.f) {
          factor *= living_street_factor_;
        }
        if (service_factor_ < 1.f) {
          factor *= service_factor_;
        }
        return (pedestrian_cost_.speedfactor_ * factor);
      } else {
        return (midgard::kSecPerHour * 0.001f) / static_cast<float>(baldr::kMaxFerrySpeedKph);
      }
    case Costing::transit:
      return 0.0f;
    case Costing::none_:
    case Costing::multimodal:
      return 1.f;
  }

  throw std::runtime_error("AStarCostFactor not implemented for costing type");
}

// Gets the hierarchy limits.
std::vector<HierarchyLimits>& DynamicCost::GetHierarchyLimits() {
  return hierarchy_limits_;
}

// Sets hierarchy limits.
void DynamicCost::SetHierarchyLimits(const std::vector<HierarchyLimits>& hierarchy_limits) {
  hierarchy_limits_ = hierarchy_limits;
}

// Relax hierarchy limits.
void DynamicCost::RelaxHierarchyLimits(const bool using_bidirectional) {
  // since bidirectional A* does about half the expansion we can do half the relaxation here
  const float relax_factor = using_bidirectional ? 8.f : 16.f;
  const float expansion_within_factor = using_bidirectional ? 2.0f : 4.0f;

  for (auto& hierarchy : hierarchy_limits_) {
    sif::RelaxHierarchyLimits(hierarchy, relax_factor, expansion_within_factor);
  }
}

// Set the current travel mode.
void DynamicCost::set_travel_mode(const TravelMode mode) {
  travel_mode_ = mode;
}

// Get the current travel mode.
TravelMode DynamicCost::travel_mode() const {
  return travel_mode_;
}

// Get the current travel type.
uint8_t DynamicCost::travel_type() const {
  switch (costing_type_) {
    case Costing::auto_:
    case Costing::taxi:
      return static_cast<uint8_t>(VehicleType::kCar);
    case Costing::bus:
      return static_cast<uint8_t>(VehicleType::kBus);
    case Costing::motorcycle:
      return static_cast<uint8_t>(VehicleType::kMotorcycle);
    case Costing::motor_scooter:
      return static_cast<uint8_t>(VehicleType::kMotorScooter);
    case Costing::truck:
      return static_cast<uint8_t>(VehicleType::kTruck);
    case Costing::bicycle:
      return static_cast<uint8_t>(bicycle_cost_.type_);
    case Costing::pedestrian:
    case Costing::bikeshare:
      return static_cast<uint8_t>(pedestrian_cost_.type_);
    default:
      return 0;
  }
}

// Get the wheelchair required flag.
bool DynamicCost::wheelchair() const {
  if (costing_type_ == Costing::transit) {
    return transit_cost_.wheelchair();
  }
  return false;
}

// Get the bicycle required flag.
bool DynamicCost::bicycle() const {
  if (costing_type_ == Costing::transit) {
    return transit_cost_.bicycle();
  }
  return false;
}

// Add to the exclude list.
void DynamicCost::AddToExcludeList(const graph_tile_ptr& tile) {
  if (costing_type_ == Costing::transit) {
    transit_cost_.AddToExcludeList(tile);
  }
}

// Checks if we should exclude or not.
bool DynamicCost::IsExcluded(const graph_tile_ptr& tile, const baldr::DirectedEdge* edge) {
  if (costing_type_ == Costing::transit) {
    return transit_cost_.IsExcluded(tile, edge);
  }
  return false;
}

// Checks if we should exclude or not.
bool DynamicCost::IsExcluded(const graph_tile_ptr& tile, const baldr::NodeInfo* node) {
  if (costing_type_ == Costing::transit) {
    return transit_cost_.IsExcluded(tile, node);
  }
  return false;
}

// Adds a list of edges (GraphIds) to the user specified avoid list.
void DynamicCost::AddUserAvoidEdges(const std::vector<AvoidEdge>& exclude_edges) {
  for (auto edge : exclude_edges) {
    user_exclude_edges_.insert({edge.id, edge.percent_along});
  }
}

Cost DynamicCost::BSSCost() const {
  if (costing_type_ == Costing::bicycle) {
    return bicycle_cost_.BSSCost();
  } else if (costing_type_ == Costing::pedestrian || costing_type_ == Costing::bikeshare) {
    return pedestrian_cost_.BSSCost();
  }
  return kNoCost;
}

void DynamicCost::set_use_tracks(float use_tracks) {
  // Calculate penalty value based on use preference. Return value
  // in range [kMaxTrackPenalty; 0], if use < 0.5; or
  // 0, if use > 0.5.
  track_penalty_ = use_tracks < 0.5f ? (kMaxTrackPenalty * (1.f - 2.f * use_tracks)) : 0.f;
  // Calculate factor value based on use preference. Return value
  // in range [kMaxTrackFactor; 1], if use < 0.5; or
  // in range [1; kMinTrackFactor], if use > 0.5
  track_factor_ = use_tracks < 0.5f
                      ? (kMaxTrackFactor - 2.f * use_tracks * (kMaxTrackFactor - 1.f))
                      : (kMinTrackFactor + 2.f * (1.f - use_tracks) * (1.f - kMinTrackFactor));
}

void DynamicCost::set_use_living_streets(float use_living_streets) {
  // Calculate penalty value based on use preference. Return value
  // in range [kMaxLivingStreetPenalty; 0], if use < 0.5; or
  // 0, if use > 0.5.
  living_street_penalty_ =
      use_living_streets < 0.5f ? (kMaxLivingStreetPenalty * (1.f - 2.f * use_living_streets)) : 0;

  // Calculate factor value based on use preference. Return value
  // in range [kMaxLivingStreetFactor; 1], if use < 0.5; or
  // in range [1; kMinLivingStreetFactor], if use > 0.5.
  // Thus living_street_factor_ is inversely proportional to use_living_streets.
  living_street_factor_ =
      use_living_streets < 0.5f
          ? (kMaxLivingStreetFactor - 2.f * use_living_streets * (kMaxLivingStreetFactor - 1.f))
          : (kMinLivingStreetFactor +
             2.f * (1.f - use_living_streets) * (1.f - kMinLivingStreetFactor));
}

void DynamicCost::set_use_lit(float use_lit) {
  unlit_factor_ =
      use_lit < 0.5f ? kMinLitFactor + 2.f * use_lit : ((kMinLitFactor - 5.f) + 12.f * use_lit);
}

void ParseBaseCostOptions(const rapidjson::Value& json,
                          Costing* c,
                          const BaseCostingOptionsConfig& cfg) {
  auto* co = c->mutable_options();

  // ignore bogus input
  if (co->has_flow_mask_case() && co->flow_mask() > kDefaultFlowMask)
    co->clear_flow_mask();

  // defer to json or defaults if no pbf is present
  auto speed_types = rapidjson::get_child_optional(json, "/speed_types");
  if (speed_types || !co->has_flow_mask_case())
    co->set_flow_mask(SpeedMask_Parse(speed_types));

  // named costing
  auto name = rapidjson::get_optional<std::string>(json, "/name");
  if (name) {
    c->set_name(*name);
  }

  // various traversability flags
  JSON_PBF_DEFAULT_V2(co, false, json, "/ignore_restrictions", ignore_restrictions);
  JSON_PBF_DEFAULT_V2(co, false, json, "/ignore_oneways", ignore_oneways);
  JSON_PBF_DEFAULT_V2(co, false, json, "/ignore_access", ignore_access);
  JSON_PBF_DEFAULT_V2(co, false, json, "/ignore_closures", ignore_closures);
  JSON_PBF_DEFAULT_V2(co, false, json, "/ignore_construction", ignore_construction);
  JSON_PBF_DEFAULT_V2(co, false, json, "/ignore_non_vehicular_restrictions",
                      ignore_non_vehicular_restrictions);

  // shortest
  JSON_PBF_DEFAULT_V2(co, false, json, "/shortest", shortest);

  // disable hierarchy pruning
  co->set_disable_hierarchy_pruning(
      rapidjson::get<bool>(json, "/disable_hierarchy_pruning", co->disable_hierarchy_pruning()));

  // hierarchy limits
  for (const auto& level : TileHierarchy::levels()) {
    std::string hierarchy_limits_path = "/hierarchy_limits/" + std::to_string(level.level);

    unsigned int max_up_transitions =
        rapidjson::get<decltype(max_up_transitions)>(json,
                                                     std::string(hierarchy_limits_path +
                                                                 "/max_up_transitions")
                                                         .c_str(),
                                                     kUnlimitedTransitions);
    float expand_within_distance =
        rapidjson::get<decltype(expand_within_distance)>(json,
                                                         std::string(hierarchy_limits_path +
                                                                     "/expand_within_distance")
                                                             .c_str(),
                                                         kMaxDistance);

    // don't set anything on the protobuf if the user sent nothing
    if (max_up_transitions == kUnlimitedTransitions && expand_within_distance == kMaxDistance)
      continue;

    // set on protobuf
    HierarchyLimits hierarchylimits;
    hierarchylimits.set_max_up_transitions(max_up_transitions);
    hierarchylimits.set_expand_within_dist(expand_within_distance);

    co->mutable_hierarchy_limits()->insert({level.level, hierarchylimits});
  }

  // destination only penalty
  JSON_PBF_RANGED_DEFAULT(co, cfg.dest_only_penalty_, json, "/destination_only_penalty",
                          destination_only_penalty);

  // maneuver_penalty
  JSON_PBF_RANGED_DEFAULT(co, cfg.maneuver_penalty_, json, "/maneuver_penalty", maneuver_penalty);

  // alley_penalty
  JSON_PBF_RANGED_DEFAULT(co, cfg.alley_penalty_, json, "/alley_penalty", alley_penalty);

  // gate_cost
  JSON_PBF_RANGED_DEFAULT(co, cfg.gate_cost_, json, "/gate_cost", gate_cost);

  // gate_penalty
  JSON_PBF_RANGED_DEFAULT(co, cfg.gate_penalty_, json, "/gate_penalty", gate_penalty);

  // private_access_penalty
  JSON_PBF_RANGED_DEFAULT(co, cfg.private_access_penalty_, json, "/private_access_penalty",
                          private_access_penalty);

  // country_crossing_cost
  JSON_PBF_RANGED_DEFAULT(co, cfg.country_crossing_cost_, json, "/country_crossing_cost",
                          country_crossing_cost);

  // country_crossing_penalty
  JSON_PBF_RANGED_DEFAULT(co, cfg.country_crossing_penalty_, json, "/country_crossing_penalty",
                          country_crossing_penalty);

  if (!cfg.disable_toll_booth_) {
    // toll_booth_cost
    JSON_PBF_RANGED_DEFAULT(co, cfg.toll_booth_cost_, json, "/toll_booth_cost", toll_booth_cost);

    // toll_booth_penalty
    JSON_PBF_RANGED_DEFAULT(co, cfg.toll_booth_penalty_, json, "/toll_booth_penalty",
                            toll_booth_penalty);
  }

  if (!cfg.disable_ferry_) {
    // ferry_cost
    JSON_PBF_RANGED_DEFAULT(co, cfg.ferry_cost_, json, "/ferry_cost", ferry_cost);

    // use_ferry
    JSON_PBF_RANGED_DEFAULT(co, cfg.use_ferry_, json, "/use_ferry", use_ferry);
  }

  if (!cfg.disable_rail_ferry_) {
    // rail_ferry_cost
    JSON_PBF_RANGED_DEFAULT(co, cfg.rail_ferry_cost_, json, "/rail_ferry_cost", rail_ferry_cost);

    // use_rail_ferry
    JSON_PBF_RANGED_DEFAULT(co, cfg.use_rail_ferry_, json, "/use_rail_ferry", use_rail_ferry);
  }

  JSON_PBF_DEFAULT_V2(co, cfg.exclude_unpaved_, json, "/exclude_unpaved", exclude_unpaved);

  JSON_PBF_DEFAULT_V2(co, cfg.exclude_bridges_, json, "/exclude_bridges", exclude_bridges);
  JSON_PBF_DEFAULT_V2(co, cfg.exclude_tunnels_, json, "/exclude_tunnels", exclude_tunnels);
  JSON_PBF_DEFAULT_V2(co, cfg.exclude_tolls_, json, "/exclude_tolls", exclude_tolls);
  JSON_PBF_DEFAULT_V2(co, cfg.exclude_highways_, json, "/exclude_highways", exclude_highways);
  JSON_PBF_DEFAULT_V2(co, cfg.exclude_ferries_, json, "/exclude_ferries", exclude_ferries);

  JSON_PBF_DEFAULT_V2(co, cfg.exclude_cash_only_tolls_, json, "/exclude_cash_only_tolls",
                      exclude_cash_only_tolls);

  // service_penalty
  JSON_PBF_RANGED_DEFAULT(co, cfg.service_penalty_, json, "/service_penalty", service_penalty);

  // service_factor
  JSON_PBF_RANGED_DEFAULT(co, cfg.service_factor_, json, "/service_factor", service_factor);

  // use_tracks
  JSON_PBF_RANGED_DEFAULT(co, cfg.use_tracks_, json, "/use_tracks", use_tracks);

  // use_living_streets
  JSON_PBF_RANGED_DEFAULT(co, cfg.use_living_streets_, json, "/use_living_streets",
                          use_living_streets);

  // use_lit
  JSON_PBF_RANGED_DEFAULT_V2(co, cfg.use_lit_, json, "/use_lit", use_lit);

  // closure_factor
  JSON_PBF_RANGED_DEFAULT(co, cfg.closure_factor_, json, "/closure_factor", closure_factor);

  // HOT/HOV
  JSON_PBF_DEFAULT_V2(co, cfg.include_hot_, json, "/include_hot", include_hot);
  JSON_PBF_DEFAULT_V2(co, cfg.include_hov2_, json, "/include_hov2", include_hov2);
  JSON_PBF_DEFAULT_V2(co, cfg.include_hov3_, json, "/include_hov3", include_hov3);

  JSON_PBF_RANGED_DEFAULT_V2(co, kFixedSpeedRange, json, "/fixed_speed", fixed_speed);
}

void ParseCosting(const rapidjson::Document& doc,
                  const std::string& costing_options_key,
                  Options& options) {
  // get the needed costing options in there
  for (const auto& costing_type : kCostingTypeMapping.at(options.costing_type())) {
    // Create the costing options key
    const auto& costing_str = valhalla::Costing_Enum_Name(costing_type);
    if (costing_str.empty())
      continue;
    const auto key = costing_options_key + "/" + costing_str;
    // Parse the costing options
    auto& costing = (options.mutable_costings())[costing_type];
    ParseCosting(doc, key, &costing, costing_type);
  }
}

void ParseCosting(const rapidjson::Document& doc,
                  const std::string& key,
                  Costing* costing,
                  Costing::Type costing_type) {
  // if the costing wasnt specified we have to find it nested in the json object
  if (costing_type == Costing::Type_ARRAYSIZE) {
    // it has to have a costing object, it has to be an object, it has to have a member
    // named costing and the value of that member has to be a string type
    auto json = rapidjson::get_child_optional(doc, key.c_str());
    decltype((*json)->MemberBegin()) costing_itr;
    if (!json || !(*json)->IsObject() ||
        (costing_itr = (*json)->FindMember("costing")) == (*json)->MemberEnd() ||
        !costing_itr->value.IsString()) {
      throw valhalla_exception_t{127};
    }
    // then we can try to parse the string and if its invalid we barf
    std::string costing_str = costing_itr->value.GetString();
    if (!Costing_Enum_Parse(costing_str, &costing_type)) {
      throw valhalla_exception_t{125, "'" + costing_str + "'"};
    }
  }
  // finally we can parse the costing
  switch (costing_type) {
    case Costing::auto_: {
      sif::ParseAutoCostOptions(doc, key, costing);
      break;
    }
    case Costing::bicycle: {
      sif::ParseBicycleCostOptions(doc, key, costing);
      break;
    }
    case Costing::bus: {
      sif::ParseBusCostOptions(doc, key, costing);
      break;
    }
    case Costing::taxi: {
      sif::ParseTaxiCostOptions(doc, key, costing);
      break;
    }
    case Costing::motor_scooter: {
      sif::ParseMotorScooterCostOptions(doc, key, costing);
      break;
    }
    case Costing::multimodal: {
      costing->set_type(Costing::multimodal); // Nothing to parse for this one
      break;
    }
    case Costing::pedestrian: {
      sif::ParsePedestrianCostOptions(doc, key, costing);
      break;
    }
    case Costing::bikeshare: {
      costing->set_type(Costing::bikeshare); // Nothing to parse for this one
      break;
    }
    case Costing::transit: {
      sif::ParseTransitCostOptions(doc, key, costing);
      break;
    }
    case Costing::truck: {
      sif::ParseTruckCostOptions(doc, key, costing);
      break;
    }
    case Costing::motorcycle: {
      sif::ParseMotorcycleCostOptions(doc, key, costing);
      break;
    }
    case Costing::none_: {
      sif::ParseNoCostOptions(doc, key, costing);
      break;
    }
    default: {
      throw std::logic_error("Unknown costing");
    }
  }
  costing->set_type(costing_type);
}

} // namespace sif
} // namespace valhalla
