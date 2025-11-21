#ifndef VALHALLA_SIF_TRUCKCOST_H_
#define VALHALLA_SIF_TRUCKCOST_H_

#include "dynamiccost_const.h"

#include <valhalla/baldr/rapidjson_fwd.h>
#include <valhalla/proto/options.pb.h>

namespace valhalla {
namespace sif {

/**
 * Derived class providing dynamic edge costing for truck routes.
 */
class TruckCost {
public: 
  TruckCost() {}

  /**
   * Construct truck costing. Pass in cost type and costing_options using protocol buffer(pbf).
   * @param  costing specified costing type.
   * @param  costing_options pbf with request costing_options.
   */
  TruckCost(DynamicCost* parent, const Costing& costing_options);

  ~TruckCost();

  /**
   * Does the costing allow hierarchy transitions. Truck costing will allow
   * transitions by default.
   * @return  Returns true if the costing model allows hierarchy transitions).
   */
  bool AllowTransitions() const;

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
   *
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
   * Callback for Allowed doing mode  specific restriction checks
   */
  bool ModeSpecificAllowed(const baldr::AccessRestriction& restriction) const;

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param  edge      Pointer to a directed edge.
   * @param  tile      Current tile.
   * @param  time_info Time info about edge passing.
   * @return  Returns the cost and time (seconds)
   */
  Cost EdgeCost(const DynamicCost* parent,
                const baldr::DirectedEdge* edge,
                const baldr::GraphId& edgeid,
                const baldr::graph_tile_ptr& tile,
                const baldr::TimeInfo& time_info,
                uint8_t& flow_sources) const;

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
                             const bool has_measured_speed,
                             const InternalTurn internal_turn) const;

  /**
   * Get the cost factor for A* heuristics. This factor is multiplied
   * with the distance to the destination to produce an estimate of the
   * minimum cost to the destination. The A* heuristic must underestimate the
   * cost to the destination. So a time based estimate based on speed should
   * assume the maximum speed is used to the destination such that the time
   * estimate is less than the least possible time along roads.
   */
  float AStarCostFactor() const;

public:
  float toll_factor_;       // Factor applied when road has a toll
  float low_class_penalty_; // Penalty (seconds) to go to residential or service road

  // Vehicle attributes (used for special restrictions and costing)
  bool hazmat_;                  // Carrying hazardous materials
  float weight_;                 // Vehicle weight in metric tons
  float axle_load_;              // Axle load weight in metric tons
  float height_;                 // Vehicle height in meters
  float width_;                  // Vehicle width in meters
  float length_;                 // Vehicle length in meters
  float highway_factor_;         // Factor applied when road is a motorway or trunk
  float non_truck_route_factor_; // Factor applied when road is not part of a designated truck route
  uint8_t axle_count_;           // Vehicle axle count

  // determine if we should allow hgv=no edges and penalize them instead
  float no_hgv_access_penalty_;
};

/**
 * Parses the truck cost options from json and stores values in pbf.
 * @param doc The json request represented as a DOM tree.
 * @param costing_options_key A string representing the location in the DOM tree where the costing
 *                            options are stored.
 * @param pbf_costing A mutable protocol buffer where the parsed json values will be stored.
 */
void ParseTruckCostOptions(const rapidjson::Document& doc,
                           const std::string& costing_options_key,
                           Costing* pbf_costing);

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_TRUCKCOST_H_
