#include "baldr/directededge_soa.h"

#include "baldr/directededge.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <vector>

namespace valhalla {
namespace baldr {
namespace {

enum class FieldId : uint16_t {
  kEndnode = 0,
  kRestrictions,
  kOppIndex,
  kForward,
  kLeavesTile,
  kCtryCrossing,
  kEdgeinfoOffset,
  kAccessRestriction,
  kStartRestriction,
  kEndRestriction,
  kComplexRestriction,
  kDestOnly,
  kNotThru,
  kSpeed,
  kFreeFlowSpeed,
  kConstrainedFlowSpeed,
  kTruckSpeed,
  kNameConsistency,
  kUse,
  kLanecount,
  kDensity,
  kClassification,
  kSurface,
  kToll,
  kRoundabout,
  kTruckRoute,
  kHasPredictedSpeed,
  kForwardAccess,
  kReverseAccess,
  kMaxUpSlopeBits,
  kMaxDownSlopeBits,
  kSacScale,
  kCycleLane,
  kBikeNetwork,
  kUseSidepath,
  kDismount,
  kSidewalkLeft,
  kSidewalkRight,
  kShoulder,
  kLaneConn,
  kTurnlanes,
  kSign,
  kInternal,
  kTunnel,
  kBridge,
  kTrafficSignal,
  kSpare1,
  kDeadend,
  kBssConnection,
  kStopSign,
  kYieldSign,
  kHovType,
  kIndoor,
  kLit,
  kDestOnlyHgv,
  kSpare4,
  kTurntypeBits,
  kEdgeToLeftBits,
  kLength,
  kWeightedGradeBits,
  kCurvature,
  kStopImpactBits,
  kEdgeToRightBits,
  kStopOrLine,
  kLocalEdgeIdx,
  kOppLocalIdx,
  kShortcut,
  kSuperseded,
  kIsShortcut,
  kSpeedType,
  kNamed,
  kLink,
};

struct FieldDescriptor {
  FieldId id;
  uint8_t word;
  uint8_t offset;
  uint8_t bits;
};

constexpr FieldDescriptor kFieldDescriptors[] = {
    {FieldId::kEndnode, 0, 0, 46},         {FieldId::kRestrictions, 0, 46, 8},
    {FieldId::kOppIndex, 0, 54, 7},        {FieldId::kForward, 0, 61, 1},
    {FieldId::kLeavesTile, 0, 62, 1},      {FieldId::kCtryCrossing, 0, 63, 1},
    {FieldId::kEdgeinfoOffset, 1, 0, 25},  {FieldId::kAccessRestriction, 1, 25, 12},
    {FieldId::kStartRestriction, 1, 37, 12},{FieldId::kEndRestriction, 1, 49, 12},
    {FieldId::kComplexRestriction, 1, 61, 1},{FieldId::kDestOnly, 1, 62, 1},
    {FieldId::kNotThru, 1, 63, 1},         {FieldId::kSpeed, 2, 0, 8},
    {FieldId::kFreeFlowSpeed, 2, 8, 8},    {FieldId::kConstrainedFlowSpeed, 2, 16, 8},
    {FieldId::kTruckSpeed, 2, 24, 8},      {FieldId::kNameConsistency, 2, 32, 8},
    {FieldId::kUse, 2, 40, 6},             {FieldId::kLanecount, 2, 46, 4},
    {FieldId::kDensity, 2, 50, 4},         {FieldId::kClassification, 2, 54, 3},
    {FieldId::kSurface, 2, 57, 3},         {FieldId::kToll, 2, 60, 1},
    {FieldId::kRoundabout, 2, 61, 1},      {FieldId::kTruckRoute, 2, 62, 1},
    {FieldId::kHasPredictedSpeed, 2, 63, 1},{FieldId::kForwardAccess, 3, 0, 12},
    {FieldId::kReverseAccess, 3, 12, 12},  {FieldId::kMaxUpSlopeBits, 3, 24, 5},
    {FieldId::kMaxDownSlopeBits, 3, 29, 5},{FieldId::kSacScale, 3, 34, 3},
    {FieldId::kCycleLane, 3, 37, 2},       {FieldId::kBikeNetwork, 3, 39, 1},
    {FieldId::kUseSidepath, 3, 40, 1},     {FieldId::kDismount, 3, 41, 1},
    {FieldId::kSidewalkLeft, 3, 42, 1},    {FieldId::kSidewalkRight, 3, 43, 1},
    {FieldId::kShoulder, 3, 44, 1},        {FieldId::kLaneConn, 3, 45, 1},
    {FieldId::kTurnlanes, 3, 46, 1},       {FieldId::kSign, 3, 47, 1},
    {FieldId::kInternal, 3, 48, 1},        {FieldId::kTunnel, 3, 49, 1},
    {FieldId::kBridge, 3, 50, 1},          {FieldId::kTrafficSignal, 3, 51, 1},
    {FieldId::kSpare1, 3, 52, 1},          {FieldId::kDeadend, 3, 53, 1},
    {FieldId::kBssConnection, 3, 54, 1},   {FieldId::kStopSign, 3, 55, 1},
    {FieldId::kYieldSign, 3, 56, 1},       {FieldId::kHovType, 3, 57, 1},
    {FieldId::kIndoor, 3, 58, 1},          {FieldId::kLit, 3, 59, 1},
    {FieldId::kDestOnlyHgv, 3, 60, 1},     {FieldId::kSpare4, 3, 61, 3},
    {FieldId::kTurntypeBits, 4, 0, 24},    {FieldId::kEdgeToLeftBits, 4, 24, 8},
    {FieldId::kLength, 4, 32, 24},         {FieldId::kWeightedGradeBits, 4, 56, 4},
    {FieldId::kCurvature, 4, 60, 4},       {FieldId::kStopImpactBits, 5, 0, 24},
    {FieldId::kEdgeToRightBits, 5, 24, 8}, {FieldId::kStopOrLine, 5, 0, 32},
    {FieldId::kLocalEdgeIdx, 5, 32, 7},    {FieldId::kOppLocalIdx, 5, 39, 7},
    {FieldId::kShortcut, 5, 46, 7},        {FieldId::kSuperseded, 5, 53, 7},
    {FieldId::kIsShortcut, 5, 60, 1},      {FieldId::kSpeedType, 5, 61, 1},
    {FieldId::kNamed, 5, 62, 1},           {FieldId::kLink, 5, 63, 1},
};

constexpr bool ValidateFieldDescriptors() {
  for (std::size_t i = 0; i < std::size(kFieldDescriptors); ++i) {
    if (kFieldDescriptors[i].id != static_cast<FieldId>(i)) {
      return false;
    }
  }
  return true;
}

static_assert(ValidateFieldDescriptors(),
              "DirectedEdge field descriptor table order mismatch");

constexpr std::size_t kWordCount = 6;

const FieldDescriptor& GetDescriptor(FieldId id) {
  const auto idx = static_cast<std::size_t>(id);
  if (idx >= std::size(kFieldDescriptors)) {
    throw std::out_of_range("DirectedEdge field descriptor index out of range");
  }
  return kFieldDescriptors[idx];
}

constexpr uint64_t MaskForBits(uint32_t bits) {
  return bits == 64 ? std::numeric_limits<uint64_t>::max()
                    : (bits == 0 ? 0ULL : ((1ULL << bits) - 1ULL));
}

uint64_t ExtractField(const uint64_t* words, const FieldDescriptor& desc) {
  const uint64_t mask = MaskForBits(desc.bits);
  return desc.bits == 64 ? words[desc.word]
                         : ((words[desc.word] >> desc.offset) & mask);
}

void SetField(uint64_t* words, const FieldDescriptor& desc, uint64_t value) {
  if (desc.bits == 64) {
    words[desc.word] = value;
    return;
  }
  const uint64_t mask = MaskForBits(desc.bits) << desc.offset;
  words[desc.word] = (words[desc.word] & ~mask) | ((value << desc.offset) & mask);
}

constexpr std::size_t BytesForBits(uint32_t bits) {
  return static_cast<std::size_t>((bits + 7u) / 8u);
}

void AppendLE(std::vector<uint8_t>& out, uint64_t value, std::size_t bytes) {
  for (std::size_t i = 0; i < bytes; ++i) {
    out.push_back(static_cast<uint8_t>((value >> (i * 8)) & 0xff));
  }
}

template <typename T> void AppendLE(std::vector<uint8_t>& out, T value) {
  AppendLE(out, static_cast<uint64_t>(value), sizeof(T));
}

uint64_t ReadLE(const uint8_t* data, std::size_t bytes) {
  uint64_t value = 0;
  for (std::size_t i = 0; i < bytes; ++i) {
    value |= static_cast<uint64_t>(data[i]) << (i * 8);
  }
  return value;
}

uint64_t ReadLE(const uint8_t*& data, std::size_t& remaining, std::size_t bytes) {
  if (remaining < bytes) {
    throw std::runtime_error("DirectedEdgeWordLanes decode buffer underflow");
  }
  const uint64_t value = ReadLE(data, bytes);
  data += bytes;
  remaining -= bytes;
  return value;
}

void AppendBE(std::vector<uint8_t>& out, uint64_t value, std::size_t bytes) {
  for (std::size_t i = 0; i < bytes; ++i) {
    const std::size_t shift = (bytes - 1 - i) * 8;
    out.push_back(static_cast<uint8_t>((value >> shift) & 0xff));
  }
}

uint64_t ReadBE(const uint8_t*& data, std::size_t& remaining, std::size_t bytes) {
  if (remaining < bytes) {
    throw std::runtime_error("DirectedEdgeWordLanes decode buffer underflow");
  }
  uint64_t value = 0;
  for (std::size_t i = 0; i < bytes; ++i) {
    value = (value << 8) | static_cast<uint64_t>(data[i]);
  }
  data += bytes;
  remaining -= bytes;
  return value;
}

void AppendVarint(std::vector<uint8_t>& out, uint64_t value) {
  while (value >= 0x80) {
    out.push_back(static_cast<uint8_t>((value & 0x7f) | 0x80));
    value >>= 7;
  }
  out.push_back(static_cast<uint8_t>(value));
}

bool ReadVarint(const uint8_t*& data, std::size_t& remaining, uint64_t& value_out) {
  uint64_t value = 0;
  uint32_t shift = 0;
  while (true) {
    if (remaining == 0 || shift >= 64) {
      return false;
    }
    const uint8_t byte = *data++;
    --remaining;
    value |= static_cast<uint64_t>(byte & 0x7fu) << shift;
    if ((byte & 0x80u) == 0) {
      break;
    }
    shift += 7;
  }
  value_out = value;
  return true;
}

inline uint64_t ZigZagEncode(int64_t value) {
  return (static_cast<uint64_t>(value) << 1) ^ static_cast<uint64_t>(value >> 63);
}

inline int64_t ZigZagDecode(uint64_t value) {
  return static_cast<int64_t>((value >> 1) ^ -static_cast<int64_t>(value & 1u));
}

class BitWriter {
public:
  void Write(uint64_t value, uint8_t bits) {
    if (bits == 0) {
      return;
    }
    for (uint8_t i = 0; i < bits; ++i) {
      const uint8_t bit = static_cast<uint8_t>((value >> i) & 0x1);
      current_ |= static_cast<uint8_t>(bit << bitpos_);
      if (++bitpos_ == 8) {
        data_.push_back(current_);
        current_ = 0;
        bitpos_ = 0;
      }
    }
  }

  void Flush() {
    if (bitpos_ > 0) {
      data_.push_back(current_);
      current_ = 0;
      bitpos_ = 0;
    }
  }

  const std::vector<uint8_t>& data() const {
    return data_;
  }

  std::vector<uint8_t>& data() {
    return data_;
  }

private:
  std::vector<uint8_t> data_;
  uint8_t current_ = 0;
  uint8_t bitpos_ = 0;
};

class BitReader {
public:
  BitReader(const uint8_t* data, std::size_t size) : data_(data), size_(size) {
  }

  uint64_t Read(uint8_t bits) {
    uint64_t value = 0;
    for (uint8_t i = 0; i < bits; ++i) {
      if (bitpos_ == 8) {
        Advance();
      }
      if (!has_byte_) {
        throw std::runtime_error("DirectedEdgeWordLanes bit reader underflow");
      }
      const uint8_t bit = static_cast<uint8_t>((current_ >> bitpos_) & 0x1);
      value |= static_cast<uint64_t>(bit) << i;
      ++bitpos_;
    }
    return value;
  }

  bool Empty() const {
    return consumed_ >= size_ && bitpos_ == 0;
  }

private:
  void Advance() {
    if (consumed_ >= size_) {
      has_byte_ = false;
      return;
    }
    current_ = data_[consumed_++];
    bitpos_ = 0;
    has_byte_ = true;
  }

  const uint8_t* data_ = nullptr;
  std::size_t size_ = 0;
  std::size_t consumed_ = 0;
  uint8_t current_ = 0;
  uint8_t bitpos_ = 8;
  bool has_byte_ = false;
};

using EdgeWords = std::array<uint64_t, kWordCount>;

void ExtractWords(const std::vector<DirectedEdge>& edges, std::vector<EdgeWords>& out) {
  out.resize(edges.size());
  for (std::size_t i = 0; i < edges.size(); ++i) {
    std::memcpy(out[i].data(), &edges[i], sizeof(DirectedEdge));
  }
}

DirectedEdge BuildEdgeFromWords(const EdgeWords& words) {
  DirectedEdge edge;
  std::memcpy(&edge, words.data(), sizeof(DirectedEdge));
  return edge;
}

bool FieldHasAnyNonZero(const std::vector<EdgeWords>& words, const FieldDescriptor& desc) {
  for (const auto& edge_words : words) {
    if (ExtractField(edge_words.data(), desc) != 0) {
      return true;
    }
  }
  return false;
}

uint32_t CountNonZero(const std::vector<EdgeWords>& words, const FieldDescriptor& desc) {
  uint32_t count = 0;
  for (const auto& edge_words : words) {
    if (ExtractField(edge_words.data(), desc) != 0) {
      ++count;
    }
  }
  return count;
}

bool ShouldUseSparseBoolEncoding(std::size_t edge_count, uint32_t non_zero_count) {
  if (edge_count == 0 || non_zero_count == 0) {
    return false;
  }
  // Favor sparse list encoding when fewer than ~15% of values are set.
  return static_cast<uint64_t>(non_zero_count) * 100ull <=
         static_cast<uint64_t>(edge_count) * 15ull;
}

constexpr std::array<FieldId, 8> kGroup0Fields{
  FieldId::kSpeed,                FieldId::kFreeFlowSpeed,
  FieldId::kConstrainedFlowSpeed, FieldId::kTruckSpeed,
  FieldId::kNameConsistency,      FieldId::kOppIndex,
  FieldId::kEdgeToLeftBits,       FieldId::kEdgeToRightBits};

constexpr std::array<FieldId, 3> kGroup1Fields{
    FieldId::kEndnode, FieldId::kEdgeinfoOffset, FieldId::kLength};

constexpr std::array<FieldId, 2> kGroup2Fields{
    FieldId::kForwardAccess, FieldId::kReverseAccess};

constexpr std::array<FieldId, 7> kGroup4Fields{
    FieldId::kRestrictions,      FieldId::kAccessRestriction, FieldId::kStartRestriction,
    FieldId::kEndRestriction,    FieldId::kComplexRestriction, FieldId::kDestOnly,
    FieldId::kNotThru};

constexpr std::array<FieldId, 32> kGroup5Fields{
    FieldId::kToll,            FieldId::kRoundabout,    FieldId::kTruckRoute,
    FieldId::kHasPredictedSpeed, FieldId::kCycleLane,   FieldId::kBikeNetwork,
    FieldId::kUseSidepath,     FieldId::kDismount,      FieldId::kSidewalkLeft,
    FieldId::kSidewalkRight,   FieldId::kShoulder,      FieldId::kLaneConn,
    FieldId::kTurnlanes,       FieldId::kSign,          FieldId::kInternal,
    FieldId::kTunnel,          FieldId::kBridge,        FieldId::kTrafficSignal,
    FieldId::kSpare1,          FieldId::kDeadend,       FieldId::kBssConnection,
    FieldId::kStopSign,        FieldId::kYieldSign,     FieldId::kHovType,
    FieldId::kIndoor,          FieldId::kLit,           FieldId::kDestOnlyHgv,
    FieldId::kSpare4,          FieldId::kIsShortcut,    FieldId::kNamed,
    FieldId::kLink,            FieldId::kSpeedType};

constexpr std::array<FieldId, 19> kGroup6Fields{
    FieldId::kTurntypeBits,   FieldId::kStopOrLine,   FieldId::kUse,
    FieldId::kLanecount,      FieldId::kDensity,      FieldId::kClassification,
    FieldId::kSurface,        FieldId::kCurvature,    FieldId::kWeightedGradeBits,
    FieldId::kMaxUpSlopeBits, FieldId::kMaxDownSlopeBits,
    FieldId::kSacScale,       FieldId::kLocalEdgeIdx, FieldId::kOppLocalIdx,
    FieldId::kShortcut,       FieldId::kSuperseded,   FieldId::kForward,
    FieldId::kLeavesTile,     FieldId::kCtryCrossing};

constexpr std::size_t kGroupCount = 7;

enum class GroupId : uint16_t {
  kGroup0 = 0,
  kGroup1 = 1,
  kGroup2 = 2,
  kGroup3 = 3,
  kGroup4 = 4,
  kGroup5 = 5,
  kGroup6 = 6,
};

struct GroupRecord {
  GroupId id;
  std::vector<uint8_t> payload;
};

enum class Group5Encoding : uint8_t {
  kBitpack = 0,
  kSparseList = 1,
};

std::vector<uint8_t> EncodeGroup0(const std::vector<EdgeWords>& words) {
  std::vector<uint8_t> data;
  std::vector<FieldId> active_fields;
  active_fields.reserve(kGroup0Fields.size());
  for (const auto field_id : kGroup0Fields) {
    if (FieldHasAnyNonZero(words, GetDescriptor(field_id))) {
      active_fields.push_back(field_id);
    }
  }

  AppendLE<uint16_t>(data, static_cast<uint16_t>(active_fields.size()));
  for (const auto field_id : active_fields) {
    const auto& desc = GetDescriptor(field_id);
    AppendLE<uint16_t>(data, static_cast<uint16_t>(field_id));
    for (const auto& edge_words : words) {
      const auto value = ExtractField(edge_words.data(), desc);
      data.push_back(static_cast<uint8_t>(value & 0xff));
    }
  }
  return data;
}

std::vector<uint8_t> EncodeGroup1(const std::vector<EdgeWords>& words) {
  std::vector<uint8_t> data;
  AppendLE<uint16_t>(data, static_cast<uint16_t>(kGroup1Fields.size()));
  for (const auto field_id : kGroup1Fields) {
    const auto& desc = GetDescriptor(field_id);
    AppendLE<uint16_t>(data, static_cast<uint16_t>(field_id));
    const auto bytes = BytesForBits(desc.bits);
    AppendLE<uint8_t>(data, static_cast<uint8_t>(bytes));
    if (words.empty()) {
      continue;
    }

    uint64_t previous = ExtractField(words.front().data(), desc);
    AppendBE(data, previous, bytes);

    for (std::size_t i = 1; i < words.size(); ++i) {
      const uint64_t value = ExtractField(words[i].data(), desc);
      const int64_t delta = static_cast<int64_t>(value) - static_cast<int64_t>(previous);
      AppendVarint(data, ZigZagEncode(delta));
      previous = value;
    }
  }
  return data;
}

std::vector<uint8_t> EncodeGroup2(const std::vector<EdgeWords>& words) {
  std::vector<uint8_t> data;
  AppendLE<uint16_t>(data, static_cast<uint16_t>(kGroup2Fields.size()));
  for (const auto field_id : kGroup2Fields) {
    const auto& desc = GetDescriptor(field_id);
    AppendLE<uint16_t>(data, static_cast<uint16_t>(field_id));
    const auto bytes = BytesForBits(desc.bits);
    for (const auto& edge_words : words) {
      const auto value = ExtractField(edge_words.data(), desc);
      AppendLE(data, value, bytes);
    }
  }
  return data;
}

std::vector<uint8_t> EncodeGroup4(const std::vector<EdgeWords>& words) {
  std::vector<uint8_t> data;
  AppendLE<uint16_t>(data, static_cast<uint16_t>(kGroup4Fields.size()));
  const std::size_t edge_count = words.size();
  const std::size_t mask_bytes = (edge_count + 7) / 8;
  for (const auto field_id : kGroup4Fields) {
    const auto& desc = GetDescriptor(field_id);
    AppendLE<uint16_t>(data, static_cast<uint16_t>(field_id));
    const auto value_bytes = BytesForBits(desc.bits);
    AppendLE<uint8_t>(data, static_cast<uint8_t>(value_bytes));

    std::vector<uint8_t> mask(mask_bytes, 0);
    std::vector<uint8_t> values;
    values.reserve(edge_count * value_bytes / 20 + 16);

    uint32_t non_zero = 0;
    for (std::size_t i = 0; i < edge_count; ++i) {
      const auto value = ExtractField(words[i].data(), desc);
      if (value == 0) {
        continue;
      }
      mask[i / 8] |= static_cast<uint8_t>(1u << (i % 8));
      ++non_zero;
      for (std::size_t b = 0; b < value_bytes; ++b) {
        values.push_back(static_cast<uint8_t>((value >> (b * 8)) & 0xff));
      }
    }

    AppendLE<uint32_t>(data, static_cast<uint32_t>(mask.size()));
    data.insert(data.end(), mask.begin(), mask.end());
    AppendLE<uint32_t>(data, non_zero);
    data.insert(data.end(), values.begin(), values.end());
  }
  return data;
}

std::vector<uint8_t> EncodeBitpackField(const std::vector<EdgeWords>& words,
                                        const FieldDescriptor& desc) {
  BitWriter writer;
  for (const auto& edge_words : words) {
    const auto value = ExtractField(edge_words.data(), desc);
    writer.Write(value, desc.bits);
  }
  writer.Flush();

  std::vector<uint8_t> payload;
  AppendLE<uint8_t>(payload, desc.bits);
  AppendLE<uint32_t>(payload, static_cast<uint32_t>(writer.data().size()));
  payload.insert(payload.end(), writer.data().begin(), writer.data().end());
  return payload;
}

std::vector<uint8_t> EncodeSparseBoolField(const std::vector<EdgeWords>& words,
                                           const FieldDescriptor& desc,
                                           uint32_t non_zero) {
  std::vector<uint8_t> payload;
  AppendLE<uint32_t>(payload, non_zero);
  if (non_zero == 0) {
    return payload;
  }

  uint32_t last_index = 0;
  bool first = true;
  for (std::size_t i = 0; i < words.size(); ++i) {
    if (ExtractField(words[i].data(), desc) == 0) {
      continue;
    }
    const uint32_t index = static_cast<uint32_t>(i);
    if (first) {
      AppendVarint(payload, index);
      first = false;
    } else {
      AppendVarint(payload, index - last_index);
    }
    last_index = index;
  }
  return payload;
}

std::vector<uint8_t> EncodeGroup5(const std::vector<EdgeWords>& words) {
  std::vector<uint8_t> data;
  struct Plan {
    FieldId id;
    const FieldDescriptor* desc;
    Group5Encoding encoding;
    uint32_t non_zero;
  };

  std::vector<Plan> plans;
  plans.reserve(kGroup5Fields.size());
  const std::size_t edge_count = words.size();
  for (const auto field_id : kGroup5Fields) {
    const auto& desc = GetDescriptor(field_id);
    const uint32_t non_zero = CountNonZero(words, desc);
    if (non_zero == 0) {
      continue;
    }
    Group5Encoding encoding = Group5Encoding::kBitpack;
    if (desc.bits == 1 && ShouldUseSparseBoolEncoding(edge_count, non_zero)) {
      encoding = Group5Encoding::kSparseList;
    }
    plans.push_back(Plan{field_id, &desc, encoding, non_zero});
  }

  AppendLE<uint16_t>(data, static_cast<uint16_t>(plans.size()));
  for (const auto& plan : plans) {
    AppendLE<uint16_t>(data, static_cast<uint16_t>(plan.id));
    AppendLE<uint8_t>(data, static_cast<uint8_t>(plan.encoding));
    std::vector<uint8_t> payload;
    switch (plan.encoding) {
    case Group5Encoding::kBitpack:
      payload = EncodeBitpackField(words, *plan.desc);
      break;
    case Group5Encoding::kSparseList:
      payload = EncodeSparseBoolField(words, *plan.desc, plan.non_zero);
      break;
    }
    AppendLE<uint32_t>(data, static_cast<uint32_t>(payload.size()));
    data.insert(data.end(), payload.begin(), payload.end());
  }
  return data;
}

std::vector<uint8_t> EncodeGroup6(const std::vector<EdgeWords>& words) {
  std::vector<uint8_t> data;
  AppendLE<uint16_t>(data, static_cast<uint16_t>(kGroup6Fields.size()));
  for (const auto field_id : kGroup6Fields) {
    const auto& desc = GetDescriptor(field_id);
    const auto value_bytes = BytesForBits(desc.bits);
    AppendLE<uint16_t>(data, static_cast<uint16_t>(field_id));
    AppendLE<uint8_t>(data, static_cast<uint8_t>(value_bytes));
    for (const auto& edge_words : words) {
      const auto value = ExtractField(edge_words.data(), desc);
      AppendLE(data, value, value_bytes);
    }
  }
  return data;
}

GroupRecord MakeGroupRecord(GroupId id, std::vector<uint8_t>&& payload) {
  GroupRecord record;
  record.id = id;
  record.payload = std::move(payload);
  return record;
}

void DecodeGroup5Bitpack(const uint8_t*& data,
                         std::size_t& remaining,
                         const FieldDescriptor& desc,
                         std::vector<EdgeWords>& words) {
  const auto bits = static_cast<uint8_t>(ReadLE(data, remaining, sizeof(uint8_t)));
  if (bits != desc.bits) {
    throw std::runtime_error("DirectedEdgeWordLanes bitpack width mismatch");
  }
  const auto byte_count = static_cast<uint32_t>(ReadLE(data, remaining, sizeof(uint32_t)));
  if (remaining < byte_count) {
    throw std::runtime_error("DirectedEdgeWordLanes bitpack payload overflow");
  }
  BitReader reader(data, byte_count);
  data += byte_count;
  remaining -= byte_count;
  if (bits == 0) {
    return;
  }
  for (auto& edge_words : words) {
    const auto value = reader.Read(bits);
    SetField(edge_words.data(), desc, value);
  }
}

void DecodeGroup5SparseList(const uint8_t*& data,
                            std::size_t& remaining,
                            const FieldDescriptor& desc,
                            std::vector<EdgeWords>& words) {
  if (desc.bits != 1) {
    throw std::runtime_error("DirectedEdgeWordLanes sparse bool width mismatch");
  }
  const auto non_zero = static_cast<uint32_t>(ReadLE(data, remaining, sizeof(uint32_t)));
  uint32_t last_index = 0;
  for (uint32_t i = 0; i < non_zero; ++i) {
    uint64_t delta = 0;
    if (!ReadVarint(data, remaining, delta)) {
      throw std::runtime_error("DirectedEdgeWordLanes sparse bool underflow");
    }
    if (delta > std::numeric_limits<uint32_t>::max()) {
      throw std::runtime_error("DirectedEdgeWordLanes sparse bool index overflow");
    }
    uint32_t index = static_cast<uint32_t>(delta);
    if (i > 0) {
      index += last_index;
    }
    if (index >= words.size()) {
      throw std::runtime_error("DirectedEdgeWordLanes sparse bool index out of range");
    }
    SetField(words[index].data(), desc, 1);
    last_index = index;
  }
}

void DecodeGroup0(const uint8_t*& data,
                  std::size_t& remaining,
                  std::vector<EdgeWords>& words) {
  const auto field_count = static_cast<uint16_t>(ReadLE(data, remaining, sizeof(uint16_t)));
  for (uint16_t f = 0; f < field_count; ++f) {
    const auto field_id = static_cast<FieldId>(ReadLE(data, remaining, sizeof(uint16_t)));
    const auto& desc = GetDescriptor(field_id);
    for (auto& edge_words : words) {
      const auto value = ReadLE(data, remaining, 1);
      SetField(edge_words.data(), desc, value);
    }
  }
}

void DecodeGroup1(const uint8_t*& data,
                  std::size_t& remaining,
                  std::vector<EdgeWords>& words) {
  const auto field_count = static_cast<uint16_t>(ReadLE(data, remaining, sizeof(uint16_t)));
  const std::size_t edge_count = words.size();
  for (uint16_t f = 0; f < field_count; ++f) {
    const auto field_id = static_cast<FieldId>(ReadLE(data, remaining, sizeof(uint16_t)));
    const auto bytes = static_cast<uint8_t>(ReadLE(data, remaining, sizeof(uint8_t)));
    const auto& desc = GetDescriptor(field_id);
    if (edge_count == 0) {
      continue;
    }
    uint64_t previous = ReadBE(data, remaining, bytes);
    SetField(words[0].data(), desc, previous);
    for (std::size_t i = 1; i < edge_count; ++i) {
      uint64_t encoded_delta = 0;
      if (!ReadVarint(data, remaining, encoded_delta)) {
        throw std::runtime_error("DirectedEdgeWordLanes delta decode underflow");
      }
      const int64_t delta = ZigZagDecode(encoded_delta);
      const int64_t current = static_cast<int64_t>(previous) + delta;
      if (current < 0) {
        throw std::runtime_error("DirectedEdgeWordLanes delta decode produced negative value");
      }
      const uint64_t value = static_cast<uint64_t>(current);
      SetField(words[i].data(), desc, value);
      previous = value;
    }
  }
}

void DecodeGroup2(const uint8_t*& data,
                  std::size_t& remaining,
                  std::vector<EdgeWords>& words) {
  const auto field_count = static_cast<uint16_t>(ReadLE(data, remaining, sizeof(uint16_t)));
  for (uint16_t f = 0; f < field_count; ++f) {
    const auto field_id = static_cast<FieldId>(ReadLE(data, remaining, sizeof(uint16_t)));
    const auto& desc = GetDescriptor(field_id);
    const auto bytes = BytesForBits(desc.bits);
    for (auto& edge_words : words) {
      const auto value = ReadLE(data, remaining, bytes);
      SetField(edge_words.data(), desc, value);
    }
  }
}

void DecodeGroup4(const uint8_t*& data,
                  std::size_t& remaining,
                  std::vector<EdgeWords>& words) {
  const auto field_count = static_cast<uint16_t>(ReadLE(data, remaining, sizeof(uint16_t)));
  const std::size_t edge_count = words.size();
  for (uint16_t f = 0; f < field_count; ++f) {
    const auto field_id = static_cast<FieldId>(ReadLE(data, remaining, sizeof(uint16_t)));
    const auto value_bytes = static_cast<uint8_t>(ReadLE(data, remaining, sizeof(uint8_t)));
    const auto mask_size = static_cast<uint32_t>(ReadLE(data, remaining, sizeof(uint32_t)));
    if (remaining < mask_size) {
      throw std::runtime_error("DirectedEdgeWordLanes decode mask overflow");
    }
    const uint8_t* mask = data;
    data += mask_size;
    remaining -= mask_size;
    const auto non_zero = static_cast<uint32_t>(ReadLE(data, remaining, sizeof(uint32_t)));
    const auto& desc = GetDescriptor(field_id);
    uint32_t decoded_non_zero = 0;
    for (std::size_t i = 0; i < edge_count; ++i) {
      uint64_t value = 0;
      const bool has_value = (mask[i / 8] >> (i % 8)) & 0x1;
      if (has_value) {
        if (remaining < value_bytes) {
          throw std::runtime_error("DirectedEdgeWordLanes decode sparse value overflow");
        }
        for (std::size_t b = 0; b < value_bytes; ++b) {
          value |= static_cast<uint64_t>(data[b]) << (b * 8);
        }
        data += value_bytes;
        remaining -= value_bytes;
        ++decoded_non_zero;
      }
      SetField(words[i].data(), desc, value);
    }
    if (decoded_non_zero != non_zero) {
      throw std::runtime_error("DirectedEdgeWordLanes sparse count mismatch");
    }
  }
}

void DecodeGroup5(const uint8_t*& data,
                  std::size_t& remaining,
                  std::vector<EdgeWords>& words) {
  const auto field_count = static_cast<uint16_t>(ReadLE(data, remaining, sizeof(uint16_t)));
  for (uint16_t f = 0; f < field_count; ++f) {
    const auto field_id = static_cast<FieldId>(ReadLE(data, remaining, sizeof(uint16_t)));
    const auto encoding = static_cast<Group5Encoding>(ReadLE(data, remaining, sizeof(uint8_t)));
    const auto payload_size = static_cast<uint32_t>(ReadLE(data, remaining, sizeof(uint32_t)));
    if (remaining < payload_size) {
      throw std::runtime_error("DirectedEdgeWordLanes group5 payload overflow");
    }
    const uint8_t* payload = data;
    std::size_t payload_remaining = payload_size;
    data += payload_size;
    remaining -= payload_size;
    const auto& desc = GetDescriptor(field_id);
    switch (encoding) {
    case Group5Encoding::kBitpack:
      DecodeGroup5Bitpack(payload, payload_remaining, desc, words);
      break;
    case Group5Encoding::kSparseList:
      DecodeGroup5SparseList(payload, payload_remaining, desc, words);
      break;
    default:
      throw std::runtime_error("DirectedEdgeWordLanes unknown group5 encoding");
    }
    if (payload_remaining != 0) {
      throw std::runtime_error("DirectedEdgeWordLanes group5 payload under-consumed");
    }
  }
}

void DecodeGroup6(const uint8_t*& data,
                  std::size_t& remaining,
                  std::vector<EdgeWords>& words) {
  const auto field_count = static_cast<uint16_t>(ReadLE(data, remaining, sizeof(uint16_t)));
  for (uint16_t f = 0; f < field_count; ++f) {
    const auto field_id = static_cast<FieldId>(ReadLE(data, remaining, sizeof(uint16_t)));
    const auto value_bytes = static_cast<uint8_t>(ReadLE(data, remaining, sizeof(uint8_t)));
    const auto& desc = GetDescriptor(field_id);
    for (auto& edge_words : words) {
      const auto value = ReadLE(data, remaining, value_bytes);
      SetField(edge_words.data(), desc, value);
    }
  }
}

} // namespace

DirectedEdgeWordLaneBlob DirectedEdgeWordLanes::Encode(const std::vector<DirectedEdge>& edges) {
  DirectedEdgeWordLaneBlob result;
  result.stats.original_bytes = edges.size() * sizeof(DirectedEdge);

#ifndef NDEBUG
  std::array<bool, static_cast<std::size_t>(FieldId::kLink) + 1> field_seen{};
  auto mark_fields = [&field_seen](const auto& container) {
    for (auto field_id : container) {
      field_seen[static_cast<std::size_t>(field_id)] = true;
    }
  };
  mark_fields(kGroup0Fields);
  mark_fields(kGroup1Fields);
  mark_fields(kGroup2Fields);
  mark_fields(kGroup4Fields);
  mark_fields(kGroup5Fields);
  mark_fields(kGroup6Fields);
  field_seen[static_cast<std::size_t>(FieldId::kStopImpactBits)] = true; // Encoded via kStopOrLine.
  bool all_fields_covered = true;
  for (bool covered : field_seen) {
    if (!covered) {
      all_fields_covered = false;
      break;
    }
  }
  assert(all_fields_covered && "DirectedEdgeWordLanes field coverage incomplete");
#endif

  std::vector<EdgeWords> words;
  ExtractWords(edges, words);

  std::vector<GroupRecord> groups;
  groups.reserve(6);
  groups.emplace_back(MakeGroupRecord(GroupId::kGroup0, EncodeGroup0(words)));
  groups.emplace_back(MakeGroupRecord(GroupId::kGroup1, EncodeGroup1(words)));
  groups.emplace_back(MakeGroupRecord(GroupId::kGroup2, EncodeGroup2(words)));
  groups.emplace_back(MakeGroupRecord(GroupId::kGroup4, EncodeGroup4(words)));
  groups.emplace_back(MakeGroupRecord(GroupId::kGroup5, EncodeGroup5(words)));
  groups.emplace_back(MakeGroupRecord(GroupId::kGroup6, EncodeGroup6(words)));

  std::vector<uint8_t>& buffer = result.payload;
  buffer.reserve(edges.size() * 16);
  AppendLE<uint32_t>(buffer, kMagic);
  AppendLE<uint16_t>(buffer, kVersion);
  AppendLE<uint32_t>(buffer, static_cast<uint32_t>(edges.size()));
  AppendLE<uint16_t>(buffer, static_cast<uint16_t>(groups.size()));

  for (const auto& group : groups) {
    AppendLE<uint16_t>(buffer, static_cast<uint16_t>(group.id));
    AppendLE<uint32_t>(buffer, static_cast<uint32_t>(group.payload.size()));
    buffer.insert(buffer.end(), group.payload.begin(), group.payload.end());
    const auto idx = static_cast<std::size_t>(group.id);
    if (idx < kGroupCount) {
      result.stats.group_bytes[idx] = group.payload.size();
    }
  }

  result.stats.total_bytes = buffer.size();
  return result;
}

bool DirectedEdgeWordLanes::Decode(const uint8_t* data,
                                   std::size_t size,
                                   std::vector<DirectedEdge>& edges_out) {
  if (size < sizeof(uint32_t) + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint16_t)) {
    return false;
  }
  std::size_t remaining = size;
  const uint32_t magic = static_cast<uint32_t>(ReadLE(data, remaining, sizeof(uint32_t)));
  if (magic != kMagic) {
    return false;
  }
  const uint16_t version = static_cast<uint16_t>(ReadLE(data, remaining, sizeof(uint16_t)));
  if (version != kVersion) {
    return false;
  }
  const auto edge_count = static_cast<uint32_t>(ReadLE(data, remaining, sizeof(uint32_t)));
  const auto group_count = static_cast<uint16_t>(ReadLE(data, remaining, sizeof(uint16_t)));

  std::vector<EdgeWords> words(edge_count);
  for (auto& w : words) {
    w.fill(0);
  }

  for (uint16_t g = 0; g < group_count; ++g) {
    if (remaining < sizeof(uint16_t) + sizeof(uint32_t)) {
      return false;
    }
    const auto group_id = static_cast<GroupId>(ReadLE(data, remaining, sizeof(uint16_t)));
    const auto payload_size = static_cast<uint32_t>(ReadLE(data, remaining, sizeof(uint32_t)));
    if (remaining < payload_size) {
      return false;
    }
    const uint8_t* group_data = data;
    std::size_t group_remaining = payload_size;
    data += payload_size;
    remaining -= payload_size;
    switch (group_id) {
    case GroupId::kGroup0:
      DecodeGroup0(group_data, group_remaining, words);
      break;
    case GroupId::kGroup1:
      DecodeGroup1(group_data, group_remaining, words);
      break;
    case GroupId::kGroup2:
      DecodeGroup2(group_data, group_remaining, words);
      break;
    case GroupId::kGroup4:
      DecodeGroup4(group_data, group_remaining, words);
      break;
    case GroupId::kGroup5:
      DecodeGroup5(group_data, group_remaining, words);
      break;
    case GroupId::kGroup6:
      DecodeGroup6(group_data, group_remaining, words);
      break;
    case GroupId::kGroup3:
      break;
    default:
      return false;
    }
    if (group_remaining != 0) {
      return false;
    }
  }

  if (remaining != 0) {
    bool all_zero = true;
    for (std::size_t i = 0; i < remaining; ++i) {
      if (data[i] != 0) {
        all_zero = false;
        break;
      }
    }
    if (!all_zero) {
      return false;
    }
    remaining = 0;
  }

  edges_out.resize(edge_count);
  for (std::size_t i = 0; i < edge_count; ++i) {
    edges_out[i] = BuildEdgeFromWords(words[i]);
  }
  return true;
}

} // namespace baldr
} // namespace valhalla
