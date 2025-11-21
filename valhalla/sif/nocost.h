#ifndef VALHALLA_SIF_NOCOST_H_
#define VALHALLA_SIF_NOCOST_H_

#include "dynamiccost_const.h"

#include <valhalla/baldr/rapidjson_fwd.h>
#include <valhalla/proto/options.pb.h>

namespace valhalla {
namespace sif {

/**
 * Derived class providing dynamic edge costing for "direct" no-cost routes.
 *
 * Intended for use-cases where we dont care about mode of travel, this costing allows all edges.
 */
class NoCost {
public:
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
   * @param  current_time   Current time (seconds since epoch). A value of 0
   *                        indicates the route is not time dependent.
   * @param  tz_index       timezone index for the node
   * @return Returns true if access is allowed, false if not.
   */
  bool Allowed(const DynamicCost* parent,
               const baldr::DirectedEdge* edge,
               const bool,
               const EdgeLabel&,
               const baldr::graph_tile_ptr&,
               const baldr::GraphId&,
               const uint64_t,
               const uint32_t,
               uint8_t&,
               uint8_t&) const {
    return !edge->is_shortcut();
  }

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
   * @param  current_time   Current time (seconds since epoch). A value of 0
   *                        indicates the route is not time dependent.
   * @param  tz_index       timezone index for the node
   * @return  Returns true if access is allowed, false if not.
   */
  bool AllowedReverse(const DynamicCost* parent,
                      const baldr::DirectedEdge*,
                      const EdgeLabel&,
                      const baldr::DirectedEdge* opp_edge,
                      const baldr::graph_tile_ptr&,
                      const baldr::GraphId&,
                      const uint64_t,
                      const uint32_t,
                      uint8_t&,
                      uint8_t&) const {
    return !opp_edge->is_shortcut();
  }

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param   edge      Pointer to a directed edge.
   * @param   tile      Graph tile.
   * @param   time_info Time info about edge passing.
   * @return  Returns the cost and time (seconds)
   */
  Cost EdgeCost(const DynamicCost* parent,
                const baldr::DirectedEdge* edge,
                const baldr::GraphId& edgeid,
                const baldr::graph_tile_ptr&,
                const baldr::TimeInfo&,
                uint8_t&) const {
    return {static_cast<float>(edge->length()), static_cast<float>(edge->length())};
  }
};

/**
 * Parses the cost options from json and stores values in pbf.
 * @param doc The json request represented as a DOM tree.
 * @param costing_options_key A string representing the location in the DOM tree where the costing
 *                            options are stored.
 * @param pbf_costing A mutable protocol buffer where the parsed json values will be stored.
 */
void ParseNoCostOptions(const rapidjson::Document& doc,
                        const std::string& costing_options_key,
                        Costing* pbf_costing);

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_NOCOST_H_
