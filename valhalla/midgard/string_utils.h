#pragma once

#include <algorithm>
#include <cctype>
#include <string>
#include <string_view>
#include <vector>

namespace valhalla {
namespace midgard {
namespace string_utils {

enum class SplitMode { KeepEmpty, SkipEmpty };

inline std::string to_lower_copy(std::string_view input) {
  std::string result(input.begin(), input.end());
  std::transform(result.begin(), result.end(), result.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return result;
}

inline void to_lower_in_place(std::string& value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
}

inline void to_upper_in_place(std::string& value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
}

inline void trim_in_place(std::string& value) {
  auto is_space = [](unsigned char c) { return std::isspace(c) != 0; };
  value.erase(value.begin(), std::find_if_not(value.begin(), value.end(), is_space));
  value.erase(std::find_if_not(value.rbegin(), value.rend(), is_space).base(), value.end());
}

inline std::string trim_copy(std::string_view value) {
  std::string result(value.begin(), value.end());
  trim_in_place(result);
  return result;
}

inline void trim_chars_in_place(std::string& value, std::string_view chars) {
  if (value.empty() || chars.empty()) {
    return;
  }
  auto is_trim_char = [chars](unsigned char c) {
    return chars.find(static_cast<char>(c)) != std::string_view::npos;
  };
  value.erase(value.begin(), std::find_if_not(value.begin(), value.end(), is_trim_char));
  value.erase(std::find_if_not(value.rbegin(), value.rend(), is_trim_char).base(), value.end());
}

inline void replace_all(std::string& target, std::string_view from, std::string_view to) {
  if (from.empty()) {
    return;
  }
  std::size_t pos = 0;
  while ((pos = target.find(from, pos)) != std::string::npos) {
    target.replace(pos, from.length(), to);
    pos += to.length();
  }
}

inline void remove_chars_in_place(std::string& value, std::string_view chars) {
  if (value.empty() || chars.empty()) {
    return;
  }
  value.erase(std::remove_if(value.begin(), value.end(),
                             [chars](unsigned char c) {
                               return chars.find(static_cast<char>(c)) != std::string_view::npos;
                             }),
              value.end());
}

inline std::vector<std::string>
split(std::string_view input, std::string_view delimiters, SplitMode mode = SplitMode::KeepEmpty) {
  std::vector<std::string> result;
  if (delimiters.empty()) {
    result.emplace_back(input);
    return result;
  }

  std::size_t start = 0;
  while (start <= input.size()) {
    const auto pos = input.find_first_of(delimiters, start);
    const auto token = input.substr(start, pos - start);
    if (!(mode == SplitMode::SkipEmpty && token.empty())) {
      result.emplace_back(token);
    }
    if (pos == std::string_view::npos) {
      break;
    }
    start = pos + 1;
  }
  return result;
}

inline std::vector<std::string>
split(std::string_view input, char delimiter, SplitMode mode = SplitMode::KeepEmpty) {
  const char delim[] = {delimiter, '\0'};
  return split(input, std::string_view(delim, 1), mode);
}

inline bool iequals(std::string_view lhs, std::string_view rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (std::tolower(static_cast<unsigned char>(lhs[i])) !=
        std::tolower(static_cast<unsigned char>(rhs[i]))) {
      return false;
    }
  }
  return true;
}

inline bool iends_with(std::string_view value, std::string_view suffix) {
  if (suffix.size() > value.size()) {
    return false;
  }
  const auto offset = value.size() - suffix.size();
  for (std::size_t i = 0; i < suffix.size(); ++i) {
    if (std::tolower(static_cast<unsigned char>(value[offset + i])) !=
        std::tolower(static_cast<unsigned char>(suffix[i]))) {
      return false;
    }
  }
  return true;
}

inline std::string join(const std::vector<std::string>& parts, std::string_view delimiter) {
  if (parts.empty()) {
    return {};
  }
  std::size_t total_size = (parts.size() - 1) * delimiter.size();
  for (const auto& part : parts) {
    total_size += part.size();
  }

  std::string result;
  result.reserve(total_size);
  bool first = true;
  for (const auto& part : parts) {
    if (!first) {
      result.append(delimiter.data(), delimiter.size());
    }
    first = false;
    result.append(part);
  }
  return result;
}

} // namespace string_utils
} // namespace midgard
} // namespace valhalla
