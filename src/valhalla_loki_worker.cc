#include "baldr/rapidjson_utils.h"
#include "loki/worker.h"

#include <valhalla/property_tree/ptree.hpp>

#include <iostream>

int main(int argc, char** argv) {

  if (argc < 2) {
    std::cerr << "Usage: " << std::string(argv[0]) << " conf/valhalla.json" << std::endl;
    return 1;
  }

  // config file
  std::string config_file(argv[1]);
  property_tree config;
  rapidjson::read_json(config_file, config);

  // run the service worker
  valhalla::loki::run_service(config);

  return 0;
}
