#ifndef VALHALLA_SIF_TRANSITCOST_H_
#define VALHALLA_SIF_TRANSITCOST_H_

#include "dynamiccost_const.h"

#include <valhalla/baldr/rapidjson_fwd.h>
#include <valhalla/proto/options.pb.h>

namespace valhalla {
namespace sif {

/**
 * Derived class providing dynamic edge costing for transit parts
 * of multi-modal routes.
 */
class TransitCost {
public:
  TransitCost() {}

  /**
   * Construct transit costing. Pass in cost type and costing_options using protocol buffer(pbf).
   * @param  costing specified costing type.
   * @param  costing_options pbf with request costing_options.
   */
  TransitCost(const DynamicCost* parent, const Costing& costing_options);

  ~TransitCost();

  /**
   * Get the wheelchair required flag.
   * @return  Returns true if wheelchair is required.
   */
  bool wheelchair() const;

  /**
   * Get the bicycle required flag.
   * @return  Returns true if bicycle is required.
   */
  bool bicycle() const;

  /**
   * This method overrides the factor for this mode.  The higher the value
   * the more the mode is favored.
   */
  float GetModeFactor();

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters such as conditional restrictions and
   * conditional access that can depend on time and travel mode.
   * @param  edge           Pointer to a directed edge.
   * @param  is_dest        Is a directed edge the destination?
   * @param  pred           Predecessor edge information.
   * @param  tile           Current tile.
   * @param  edgeid         GraphId of the directed edge.
   * @param  current_time   Current time (seconds since epoch).
   * @param  tz_index       timezone index for the node
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
               uint8_t& /*destonly_access_restr_mask*/) const;

  /**
   * Checks if access is allowed for an edge on the reverse path
   * (from destination towards origin). Both opposing edges (current and
   * predecessor) are provided. The access check is generally based on mode
   * of travel and the access modes allowed on the edge. However, it can be
   * extended to exclude access based on other parameters such as conditional
   * restrictions and conditional access that can depend on time and travel
   * mode.
   * @param  edge           Pointer to a directed edge.
   * @param  pred           Predecessor edge information.
   * @param  opp_edge       Pointer to the opposing directed edge.
   * @param  tile           Current tile.
   * @param  edgeid         GraphId of the opposing edge.
   * @param  current_time   Current time (seconds since epoch).
   * @param  tz_index       timezone index for the node
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
                      uint8_t& /*destonly_access_restr_mask*/) const;

  /**
   * Get the cost to traverse the specified directed edge using a transit
   * departure (schedule based edge traversal). Cost includes
   * the time (seconds) to traverse the edge.
   * @param   edge      Pointer to a directed edge.
   * @param   departure Transit departure record.
   * @param   curr_time Current local time (seconds from midnight).
   * @return  Returns the cost and time (seconds)
   */
  Cost EdgeCost(const DynamicCost* parent,
                const baldr::DirectedEdge* edge,
                const baldr::TransitDeparture* departure,
                const uint32_t curr_time) const;

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
   * Returns the transfer cost between 2 transit stops.
   * @return  Returns the transfer cost and time (seconds).
   */
  Cost TransferCost() const;

  /**
   * Returns the default transfer cost between 2 transit lines.
   * @return  Returns the transfer cost and time (seconds).
   */
  Cost DefaultTransferCost() const;

  /**
   * Get the cost factor for A* heuristics. This factor is multiplied
   * with the distance to the destination to produce an estimate of the
   * minimum cost to the destination. The A* heuristic must underestimate the
   * cost to the destination. So a time based estimate based on speed should
   * assume the maximum speed is used to the destination such that the time
   * estimate is less than the least possible time along roads.
   */
  float AStarCostFactor() const;

  /**
   * Override unit size since walking costs are higher range of vales
   */
  uint32_t UnitSize() const;

  /**This method adds to the exclude list based on the
   * user-provided exclude and include lists.
   */
  void AddToExcludeList(const baldr::graph_tile_ptr& tile);

  /**
   * Checks if we should exclude or not.
   * @return  Returns true if we should exclude, false if not.
   */
  bool IsExcluded(const baldr::graph_tile_ptr& tile, const baldr::DirectedEdge* edge);

  /**
   * Checks if we should exclude or not.
   * @return  Returns true if we should exclude, false if not.
   */
  bool IsExcluded(const baldr::graph_tile_ptr& tile, const baldr::NodeInfo* node);

public:
  // Are wheelchair or bicycle required
  bool wheelchair_;
  bool bicycle_;

  // This is the factor for this mode.  The higher the value the more the
  // mode is favored.
  float mode_factor_;

  // A measure of willingness to ride on buses or rail. Ranges from 0-1 with
  // 0 being not willing at all and 1 being totally comfortable with taking
  // this transportation. These factors determine how much rail or buses are
  // preferred over each other (if at all).
  float use_bus_;
  float use_rail_;
  float bus_factor_;
  float rail_factor_;

  // A measure of willingness to make transfers. Ranges from 0-1 with
  // 0 being not willing at all and 1 being totally comfortable.
  float use_transfers_;
  float transfer_factor_;

  float transfer_cost_;    // Transfer cost
  float transfer_penalty_; // Transfer penalty

  // TODO - compute transit tile level based on tile specification?
  float transit_tile_level = 3;

  // stops exclude list
  std::unordered_set<std::string> stop_exclude_onestops_;

  // stops include list
  std::unordered_set<std::string> stop_include_onestops_;

  // operator exclude list
  std::unordered_set<std::string> operator_exclude_onestops_;

  // operator include list
  std::unordered_set<std::string> operator_include_onestops_;

  // route excluded list
  std::unordered_set<std::string> route_exclude_onestops_;

  // route include list
  std::unordered_set<std::string> route_include_onestops_;

  // Set of routes to exclude (by GraphId)
  std::unordered_set<baldr::GraphId> exclude_routes_;

  // Set of stops to exclude (by GraphId)
  std::unordered_set<baldr::GraphId> exclude_stops_;
};

/**
 * Parses the transit cost options from json and stores values in pbf.
 * @param doc The json request represented as a DOM tree.
 * @param costing_options_key A string representing the location in the DOM tree where the costing
 *                            options are stored.
 * @param pbf_costing A mutable protocol buffer where the parsed json values will be stored.
 */
void ParseTransitCostOptions(const rapidjson::Document& doc,
                             const std::string& costing_options_key,
                             Costing* pbf_costing);

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_TRANSITCOST_H_
