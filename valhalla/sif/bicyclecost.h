#ifndef VALHALLA_SIF_BICYCLECOST_H_
#define VALHALLA_SIF_BICYCLECOST_H_

#include "dynamiccost_const.h"

#include <valhalla/baldr/rapidjson_fwd.h>
#include <valhalla/proto/options.pb.h>

namespace valhalla {
namespace sif {

/**
 * Derived class providing dynamic edge costing for bicycle routes.
 */
class BicycleCost {
public:
  BicycleCost() {}
  
  /**
   * Construct bicycle costing. Pass in cost type and costing_options using protocol buffer(pbf).
   * @param  costing specified costing type.
   * @param  costing_options pbf with request costing_options.
   */
  BicycleCost(DynamicCost* parent, const Costing& costing_options);

  ~BicycleCost() {
  }

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters such as conditional restrictions and
   * conditional access that can depend on time and travel mode.
   * @param  edge                        Pointer to a directed edge.
   * @param  is_dest                     Is a directed edge the destination?
   * @param  pred                        Predecessor edge information.
   * @param  tile                        Current tile.
   * @param  edgeid                      GraphId of the directed edge.
   * @param  current_time                Current time (seconds since epoch). A value of 0
   *                                     indicates the route is not time dependent.
   * @param  tz_index                    timezone index for the node
   * @param  destonly_access_restr_mask  Mask containing access restriction types that had a
   * local traffic exemption at the start of the expansion. This mask will be mutated by eliminating
   * flags for locally exempt access restriction types that no longer exist on the passed edge
   *
   * @return Returns true if access is allowed, false if not.
   */
  bool Allowed(const DynamicCost* parent,
               const baldr::DirectedEdge* edge,
               const bool is_dest,
               const EdgeLabel& pred,
               const baldr::graph_tile_ptr& tile,
               const baldr::GraphId& edgeid,
               const uint64_t current_time,
               const uint32_t tz_index,
               uint8_t& restriction_idx,
               uint8_t& destonly_access_restr_mask) const;

  /**
   * Checks if access is allowed for an edge on the reverse path
   * (from destination towards origin). Both opposing edges (current and
   * predecessor) are provided. The access check is generally based on mode
   * of travel and the access modes allowed on the edge. However, it can be
   * extended to exclude access based on other parameters such as conditional
   * restrictions and conditional access that can depend on time and travel
   * mode.
   * @param  edge                        Pointer to a directed edge.
   * @param  pred                        Predecessor edge information.
   * @param  opp_edge                    Pointer to the opposing directed edge.
   * @param  tile                        Current tile.
   * @param  edgeid                      GraphId of the opposing edge.
   * @param  current_time                Current time (seconds since epoch). A value of 0
   *                                     indicates the route is not time dependent.
   * @param  tz_index                    timezone index for the node
   * @param  destonly_access_restr_mask  Mask containing access restriction types that had a
   * local traffic exemption at the start of the expansion. This mask will be mutated by eliminating
   * flags for locally exempt access restriction types that no longer exist on the passed edge
   * @return  Returns true if access is allowed, false if not.
   */
  bool AllowedReverse(const DynamicCost* parent,
                      const baldr::DirectedEdge* edge,
                      const EdgeLabel& pred,
                      const baldr::DirectedEdge* opp_edge,
                      const baldr::graph_tile_ptr& tile,
                      const baldr::GraphId& opp_edgeid,
                      const uint64_t current_time,
                      const uint32_t tz_index,
                      uint8_t& restriction_idx,
                      uint8_t& destonly_access_restr_mask) const;

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param   edge       Pointer to a directed edge.
   * @param   tile       Current tile.
   * @param   time_info  Time info about edge passing.
   * @return  Returns the cost and time (seconds)
   */
  Cost EdgeCost(const DynamicCost* parent,
                const baldr::DirectedEdge* edge,
                const baldr::graph_tile_ptr&,
                const baldr::TimeInfo&,
                uint8_t&) const;

  /**
   * Returns the cost to make the transition from the predecessor edge.
   * Defaults to 0. Costing models that wish to include edge transition
   * costs (i.e., intersection/turn costs) must override this method.
   * @param  edge          Directed edge (the to edge)
   * @param  node          Node (intersection) where transition occurs.
   * @param  pred          Predecessor edge information.
   * @param  tile          Pointer to the graph tile containing the to edge.
   * @param  reader_getter Functor that facilitates access to a limited version of the graph reader
   * @return Returns the cost and time (seconds)
   */
  Cost TransitionCost(const DynamicCost* parent,
                      const baldr::DirectedEdge* edge,
                      const baldr::NodeInfo* node,
                      const EdgeLabel& pred,
                      const baldr::graph_tile_ptr& tile,
                      const std::function<baldr::LimitedGraphReader()>& reader_getter) const;

  /**
   * Returns the cost to make the transition from the predecessor edge
   * when using a reverse search (from destination towards the origin).
   * @param  idx                Directed edge local index
   * @param  node               Node (intersection) where transition occurs.
   * @param  pred               the opposing current edge in the reverse tree.
   * @param  edge               the opposing predecessor in the reverse tree
   * @param  tile               Graphtile that contains the node and the opp_edge
   * @param  edge_id            Graph ID of opp_pred_edge to get its tile if needed
   * @param  reader_getter      Functor that facilitates access to a limited version of the graph
   * reader
   * @param  has_measured_speed Do we have any of the measured speed types set?
   * @param  internal_turn      Did we make an turn on a short internal edge.
   * @return  Returns the cost and time (seconds)
   */
  Cost TransitionCostReverse(const DynamicCost* parent,
                             const uint32_t idx,
                             const baldr::NodeInfo* node,
                             const baldr::DirectedEdge* pred,
                             const baldr::DirectedEdge* edge,
                             const baldr::graph_tile_ptr& tile,
                             const baldr::GraphId& pred_id,
                             const std::function<baldr::LimitedGraphReader()>& reader_getter,
                             const bool /*has_measured_speed*/,
                             const InternalTurn /*internal_turn*/) const;

  Cost BSSCost() const;

  // Hidden in source file so we don't need it to be protected
  // We expose it within the source file for testing purposes

  float use_roads_;           // Preference of using roads between 0 and 1
  float avoid_roads_;         // Inverse of use roads
  float road_factor_;         // Road factor based on use_roads_
  float sidepath_factor_;     // Factor to use when use_sidepath is set on an edge
  float livingstreet_factor_; // Factor to use for living streets
  float track_factor_;        // Factor to use tracks
  float avoid_bad_surfaces_;  // Preference of avoiding bad surfaces for the bike type

  // Average speed (kph) on smooth, flat roads.
  float speed_;

  // Bicycle type
  BicycleType type_;

  // Minimal surface type that will be penalized for costing
  baldr::Surface minimal_surface_penalized_;
  baldr::Surface worst_allowed_surface_;

  // Cycle lane accommodation factors
  float cyclelane_factor_[8];
  float path_cyclelane_factor_[4];

  // Surface speed factors (based on road surface type).
  const float* surface_speed_factor_;

  // Road speed penalty factor. Penalties apply above a threshold (based on the use_roads factor)
  float speedpenalty_[baldr::kMaxSpeedKph + 1];
  uint32_t speed_penalty_threshold_;

  // Elevation/grade penalty (weighting applied based on the edge's weighted
  // grade (relative value from 0-15)
  float grade_penalty[16];
};

/**
 * Parses the bicycle cost options from json and stores values in pbf.
 * @param doc The json request represented as a DOM tree.
 * @param costing_options_key A string representing the location in the DOM tree where the costing
 *                            options are stored.
 * @param pbf_costing         A mutable protocol buffer where the parsed json values will be stored.
 */
void ParseBicycleCostOptions(const rapidjson::Document& doc,
                             const std::string& costing_options_key,
                             Costing* pbf_costing);

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_BICYCLECOST_H_
