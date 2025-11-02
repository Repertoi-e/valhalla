#pragma once

#include <valhalla/property_tree/ptree_fwd.hpp>

#include <string>
#include <vector>

namespace valhalla {
namespace mjolnir {

bool BuildAdminFromPBF(const property_tree& pt,
                       const std::vector<std::string>& input_files);
}
} // namespace valhalla
