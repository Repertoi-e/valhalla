#pragma once

#include <cstddef>

namespace valhalla {
namespace baldr {

// A holder struct for memory owned by the GraphTile.
class GraphMemory {
protected:
  GraphMemory() = default;

public:
  virtual ~GraphMemory() = default;

  GraphMemory(char* data_ptr, size_t data_size) : data(data_ptr), size(data_size) {}

  GraphMemory(const GraphMemory&) = delete;
  GraphMemory& operator=(const GraphMemory&) = delete;

  char* data;
  size_t size;
};

} // namespace baldr
} // namespace valhalla
