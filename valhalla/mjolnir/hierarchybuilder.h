#ifndef VALHALLA_MJOLNIR_HIERARCHYBUILDER_H
#define VALHALLA_MJOLNIR_HIERARCHYBUILDER_H

#include <valhalla/property_tree/ptree_fwd.hpp>

#include <string>

namespace valhalla {
namespace mjolnir {

/**
 * Class used to divide the road network graph into hierarchy levels.
 */
class HierarchyBuilder {
public:
  /**
   * Build the set of hierarchies based on the TileHierarchy configuration
   * and the current local hierarchy.
   * @param  pt              Configuration property tree
   * @param  new_to_old_bin  Filename of the new to old node association file
   * @param  old_to_new_bin  Filename for the old to new node association file.
   */
  static void Build(const property_tree& pt,
                    const std::string& new_to_old_bin,
                    const std::string& old_to_new_bin);
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_HIERARCHYBUILDER_H
