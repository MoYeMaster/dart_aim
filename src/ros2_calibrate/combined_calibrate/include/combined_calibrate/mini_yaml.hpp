// Tiny YAML reader for the flat files this package writes and for ROS
// camera_info files.  It understands the three shapes that matter:
//
//   key: 1.0
//   key: [1.0, 2.0, 3.0]
//   key:
//     rows: 3
//     data: [1.0, 2.0, 3.0]
//
// and also block sequences ("- 1.0").  Anything else is ignored on purpose:
// this is not a general YAML implementation.
#pragma once

#include <string>
#include <vector>

namespace combined_calibrate {

class MiniYaml {
 public:
  static bool loadFile(const std::string &path, MiniYaml &out, std::string *error = nullptr);

  MiniYaml() = default;
  explicit MiniYaml(std::vector<std::string> lines) : lines_(std::move(lines)) {}

  bool has(const std::string &key) const;
  bool getDouble(const std::string &key, double &value) const;
  bool getInt(const std::string &key, int &value) const;
  bool getString(const std::string &key, std::string &value) const;
  /// All numbers belonging to `key`, inline or in a nested block.
  std::vector<double> getNumberList(const std::string &key) const;
  /// String entries of a flow or block sequence.
  std::vector<std::string> getStringList(const std::string &key) const;

 private:
  size_t findKeyIndex(const std::string &key) const;
  std::vector<std::string> lines_;
};

}  // namespace combined_calibrate
