#ifndef VALHALLA_MJOLNIR_SHORTCUTBUILDER_H
#define VALHALLA_MJOLNIR_SHORTCUTBUILDER_H

#include <valhalla/property_tree/ptree_fwd.hpp>

namespace valhalla {
namespace mjolnir {

/**
 * Class used to build shortcut edges.
 */
class ShortcutBuilder {
public:
  /**
   * Build the shortcut edges.
   */
  static void Build(const property_tree& pt);
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_SHORTCUTBUILDER_H
