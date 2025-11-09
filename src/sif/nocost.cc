#include "sif/nocost.h"
#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "proto_conversions.h"
#include "sif/costconstants.h"
#include "sif/dynamiccost.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

void ParseNoCostOptions(const rapidjson::Document&, const std::string&, Costing* c) {
  // this is probably not needed but its part of the contract for costing..
  c->set_type(Costing::none_);
  c->set_name(Costing_Enum_Name(c->type()));
}

} // namespace sif
} // namespace valhalla
