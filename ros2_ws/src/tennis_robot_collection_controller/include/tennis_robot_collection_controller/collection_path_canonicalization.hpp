#ifndef TENNIS_ROBOT_COLLECTION_CONTROLLER__COLLECTION_PATH_CANONICALIZATION_HPP_
#define TENNIS_ROBOT_COLLECTION_CONTROLLER__COLLECTION_PATH_CANONICALIZATION_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav_msgs/msg/path.hpp"

namespace tennis_robot_collection_controller
{

class CanonicalizationError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// Exact CollectionPathCanonicalizationV1 byte stream; header stamp excluded.
std::vector<std::uint8_t> canonicalize_collection_path_v1(
  const nav_msgs::msg::Path & path);

/// Lowercase hexadecimal SHA-256 of canonicalize_collection_path_v1(path).
std::string collection_path_sha256_v1(const nav_msgs::msg::Path & path);

}  // namespace tennis_robot_collection_controller

#endif  // TENNIS_ROBOT_COLLECTION_CONTROLLER__COLLECTION_PATH_CANONICALIZATION_HPP_
