#ifndef VALHALLA_BALDR_DIRECTEDEDGE_SOA_H_
#define VALHALLA_BALDR_DIRECTEDEDGE_SOA_H_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace valhalla {
namespace baldr {

class DirectedEdge;

struct DirectedEdgeWordLaneStats {
  std::size_t total_bytes = 0;
  std::size_t original_bytes = 0;
  std::size_t group_bytes[7] = {};
};

struct DirectedEdgeWordLaneBlob {
  std::vector<uint8_t> payload;
  DirectedEdgeWordLaneStats stats;
};

class DirectedEdgeWordLanes {
public:
  static constexpr uint32_t kMagic = 0x4445534f; // 'DESO'
  static constexpr uint16_t kVersion = 2;

  static DirectedEdgeWordLaneBlob Encode(const std::vector<DirectedEdge>& edges);

  static bool Decode(const uint8_t* data, std::size_t size, std::vector<DirectedEdge>& edges_out);
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_DIRECTEDEDGE_SOA_H_
