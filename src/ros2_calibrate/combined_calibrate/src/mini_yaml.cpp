#include "combined_calibrate/mini_yaml.hpp"

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>

namespace combined_calibrate {
namespace {

std::string trim(const std::string &text) {
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  const auto last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

bool isIdentStart(char c) { return std::isalpha(static_cast<unsigned char>(c)) || c == '_'; }

bool isIdentChar(char c) { return std::isalnum(static_cast<unsigned char>(c)) || c == '_'; }

/// "  rows: 3" -> true, "numpy-bracket" or "- 1.0" -> false.
bool isKeyLine(const std::string &line) {
  const std::string text = trim(line);
  if (text.empty() || !isIdentStart(text[0])) return false;
  size_t i = 0;
  while (i < text.size() && isIdentChar(text[i])) ++i;
  while (i < text.size() && (text[i] == ' ' || text[i] == '\t')) ++i;
  return i < text.size() && text[i] == ':';
}

std::string keyName(const std::string &line) {
  const std::string text = trim(line);
  size_t i = 0;
  while (i < text.size() && isIdentChar(text[i])) ++i;
  return text.substr(0, i);
}

void appendNumbers(const std::string &fragment, std::vector<double> &out) {
  const char *cursor = fragment.c_str();
  while (*cursor != '\0') {
    while (*cursor != '\0' && !std::isdigit(static_cast<unsigned char>(*cursor)) && *cursor != '-' &&
           *cursor != '+' && *cursor != '.') {
      ++cursor;
    }
    if (*cursor == '\0') break;
    char *end = nullptr;
    const double value = std::strtod(cursor, &end);
    if (end == cursor) {
      ++cursor;
      continue;
    }
    out.push_back(value);
    cursor = end;
  }
}

/// "data:" on the same line as the key (flow map or inline list).
size_t inlineDataPos(const std::string &rest) {
  size_t pos = rest.find("data:");
  if (pos == std::string::npos) pos = rest.find("value:");
  return pos;
}

}  // namespace

bool MiniYaml::loadFile(const std::string &path, MiniYaml &out, std::string *error) {
  std::ifstream file(path);
  if (!file) {
    if (error) *error = "cannot open " + path;
    return false;
  }
  out.lines_.clear();
  std::string line;
  while (std::getline(file, line)) out.lines_.push_back(line);
  if (error) error->clear();
  return true;
}

size_t MiniYaml::findKeyIndex(const std::string &key) const {
  for (size_t i = 0; i < lines_.size(); ++i) {
    const std::string text = trim(lines_[i]);
    if (text.size() < key.size() + 1) continue;
    if (text.compare(0, key.size(), key) != 0) continue;
    const size_t after = text.find_first_not_of(" \t", key.size());
    if (after == std::string::npos || text[after] != ':') continue;
    return i;
  }
  return std::string::npos;
}

bool MiniYaml::has(const std::string &key) const { return findKeyIndex(key) != std::string::npos; }

bool MiniYaml::getDouble(const std::string &key, double &value) const {
  const size_t index = findKeyIndex(key);
  if (index == std::string::npos) return false;
  const std::string text = trim(lines_[index]);
  std::vector<double> numbers;
  appendNumbers(text.substr(text.find(':') + 1), numbers);
  if (numbers.empty()) return false;
  value = numbers.front();
  return true;
}

bool MiniYaml::getInt(const std::string &key, int &value) const {
  double number = 0.0;
  if (!getDouble(key, number)) return false;
  value = static_cast<int>(std::lround(number));
  return true;
}

bool MiniYaml::getString(const std::string &key, std::string &value) const {
  const size_t index = findKeyIndex(key);
  if (index == std::string::npos) return false;
  const std::string text = trim(lines_[index]);
  std::string rest = trim(text.substr(text.find(':') + 1));
  if (rest.empty() || rest[0] == '#') return false;
  const size_t comment = rest.find(" #");
  if (comment != std::string::npos) rest = trim(rest.substr(0, comment));
  if (rest.size() >= 2 && (rest.front() == '"' || rest.front() == '\'')) {
    const size_t end = rest.find(rest.front(), 1);
    if (end != std::string::npos) rest = rest.substr(1, end - 1);
  }
  if (rest.empty()) return false;
  value = rest;
  return true;
}

std::vector<double> MiniYaml::getNumberList(const std::string &key) const {
  std::vector<double> numbers;
  const size_t index = findKeyIndex(key);
  if (index == std::string::npos) return numbers;

  const std::string text = trim(lines_[index]);
  const std::string rest = trim(text.substr(text.find(':') + 1));
  const size_t data_pos = inlineDataPos(rest);

  enum class Mode { kUnknown, kData, kMap };
  Mode mode = Mode::kUnknown;

  if (data_pos != std::string::npos) {
    appendNumbers(rest.substr(data_pos + 5), numbers);
    mode = Mode::kData;
  } else if (!rest.empty() && rest.find(':') == std::string::npos) {
    appendNumbers(rest, numbers);
    if (!numbers.empty()) mode = Mode::kData;
  } else if (!rest.empty()) {
    mode = Mode::kMap;
  }

  for (size_t i = index + 1; i < lines_.size(); ++i) {
    const std::string line = trim(lines_[i]);
    if (line.empty() || line[0] == '#') continue;
    if (isKeyLine(line)) {
      const std::string name = keyName(line);
      if (name == "data" || name == "value" || name == "values") {
        appendNumbers(line.substr(line.find(':') + 1), numbers);
        mode = Mode::kData;
        continue;
      }
      if (name == "rows" || name == "cols" || name == "dt" || name == "size" || name == "type" ||
          name == "depth") {
        if (mode == Mode::kUnknown) mode = Mode::kMap;
        continue;
      }
      break;
    }
    if (mode == Mode::kUnknown) mode = Mode::kData;
    if (mode == Mode::kData) appendNumbers(line, numbers);
  }
  return numbers;
}

std::vector<std::string> MiniYaml::getStringList(const std::string &key) const {
  std::vector<std::string> entries;
  const size_t index = findKeyIndex(key);
  if (index == std::string::npos) return entries;

  std::string blob = lines_[index].substr(lines_[index].find(':') + 1);
  for (size_t i = index + 1; i < lines_.size(); ++i) {
    const std::string text = trim(lines_[i]);
    if (text.empty() || text[0] == '#') continue;
    if (isKeyLine(text)) break;
    blob += " " + text;
  }

  std::string token;
  auto flush = [&entries, &token]() {
    std::string cleaned = trim(token);
    while (!cleaned.empty() && (cleaned.front() == '-' || cleaned.front() == '[' || cleaned.front() == ',')) {
      cleaned = trim(cleaned.substr(1));
    }
    while (!cleaned.empty() && (cleaned.back() == ']' || cleaned.back() == ',')) {
      cleaned = trim(cleaned.substr(0, cleaned.size() - 1));
    }
    if (cleaned.size() >= 2 && (cleaned.front() == '"' || cleaned.front() == '\'')) {
      const size_t end = cleaned.find(cleaned.front(), 1);
      if (end != std::string::npos) cleaned = cleaned.substr(1, end - 1);
    }
    if (!cleaned.empty()) entries.push_back(cleaned);
    token.clear();
  };

  for (char c : blob) {
    if (c == ',' || c == '[' || c == ']') {
      flush();
    } else {
      token.push_back(c);
    }
  }
  flush();
  return entries;
}

}  // namespace combined_calibrate
