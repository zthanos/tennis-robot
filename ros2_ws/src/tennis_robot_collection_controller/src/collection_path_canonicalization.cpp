#include "tennis_robot_collection_controller/collection_path_canonicalization.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>

#include <openssl/sha.h>

namespace tennis_robot_collection_controller
{
namespace
{

void append_u32_be(std::vector<std::uint8_t> & output, std::uint32_t value)
{
  for (int shift = 24; shift >= 0; shift -= 8) {
    output.push_back(static_cast<std::uint8_t>((value >> shift) & 0xffU));
  }
}

void append_f64_be(std::vector<std::uint8_t> & output, double value)
{
  if (!std::isfinite(value)) {
    throw CanonicalizationError("path contains non-finite float64");
  }
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "float64 bit width required");
  std::memcpy(&bits, &value, sizeof(bits));
  for (int shift = 56; shift >= 0; shift -= 8) {
    output.push_back(static_cast<std::uint8_t>((bits >> shift) & 0xffU));
  }
}

bool is_valid_utf8(const std::string & text)
{
  for (std::size_t index = 0; index < text.size();) {
    const auto lead = static_cast<unsigned char>(text[index]);
    if (lead <= 0x7f) {
      ++index;
      continue;
    }
    std::size_t continuation_count = 0;
    std::uint32_t code_point = 0;
    if ((lead & 0xe0) == 0xc0) { continuation_count = 1; code_point = lead & 0x1f; }
    else if ((lead & 0xf0) == 0xe0) { continuation_count = 2; code_point = lead & 0x0f; }
    else if ((lead & 0xf8) == 0xf0) { continuation_count = 3; code_point = lead & 0x07; }
    else { return false; }
    if (index + continuation_count >= text.size()) { return false; }
    for (std::size_t offset = 1; offset <= continuation_count; ++offset) {
      const auto next = static_cast<unsigned char>(text[index + offset]);
      if ((next & 0xc0) != 0x80) { return false; }
      code_point = (code_point << 6) | (next & 0x3f);
    }
    const auto minimum = continuation_count == 1 ? 0x80U : continuation_count == 2 ? 0x800U : 0x10000U;
    if (code_point < minimum || code_point > 0x10ffffU || (code_point >= 0xd800U && code_point <= 0xdfffU)) {
      return false;
    }
    index += continuation_count + 1;
  }
  return true;
}

}  // namespace

std::vector<std::uint8_t> canonicalize_collection_path_v1(const nav_msgs::msg::Path & path)
{
  if (!is_valid_utf8(path.header.frame_id)) {
    throw CanonicalizationError("path frame_id is not valid UTF-8");
  }
  if (path.header.frame_id.size() > std::numeric_limits<std::uint32_t>::max() ||
      path.poses.size() > std::numeric_limits<std::uint32_t>::max()) {
    throw CanonicalizationError("path field exceeds canonicalization length limit");
  }
  std::vector<std::uint8_t> output;
  output.reserve(8 + path.header.frame_id.size() + path.poses.size() * 56);
  append_u32_be(output, static_cast<std::uint32_t>(path.header.frame_id.size()));
  output.insert(output.end(), path.header.frame_id.begin(), path.header.frame_id.end());
  append_u32_be(output, static_cast<std::uint32_t>(path.poses.size()));
  for (const auto & pose_stamped : path.poses) {
    const auto & position = pose_stamped.pose.position;
    const auto & orientation = pose_stamped.pose.orientation;
    append_f64_be(output, position.x);
    append_f64_be(output, position.y);
    append_f64_be(output, position.z);
    append_f64_be(output, orientation.x);
    append_f64_be(output, orientation.y);
    append_f64_be(output, orientation.z);
    append_f64_be(output, orientation.w);
  }
  return output;
}

std::string collection_path_sha256_v1(const nav_msgs::msg::Path & path)
{
  const auto bytes = canonicalize_collection_path_v1(path);
  std::array<unsigned char, SHA256_DIGEST_LENGTH> digest{};
  SHA256(bytes.data(), bytes.size(), digest.data());
  static constexpr char hex[] = "0123456789abcdef";
  std::string output;
  output.reserve(digest.size() * 2);
  for (const auto value : digest) {
    output.push_back(hex[value >> 4]);
    output.push_back(hex[value & 0x0f]);
  }
  return output;
}

}  // namespace tennis_robot_collection_controller
