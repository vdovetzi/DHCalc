#pragma once

#include <algorithm>
#include <cstdint>
#include <string>
#include <utf8.h>

namespace dhcalc {
namespace utilities {

// ── String utilities ────────────────────────────────────────────────────────

inline std::string trim_copy(std::string_view text) {
  const auto start = text.find_first_not_of(" \t\r\n");
  if (start == std::string_view::npos) {
    return {};
  }
  const auto end = text.find_last_not_of(" \t\r\n");
  return std::string(text.substr(start, end - start + 1));
}

inline std::string lowercase_copy(std::string_view text) {
  std::string result(text);
  std::transform(
      result.begin(), result.end(), result.begin(),
      [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return result;
}

inline std::string normalize_greek(const std::string &header) {
  std::string result;

  auto it = header.begin();
  auto end = header.end();

  while (it != end) {
    uint32_t cp = utf8::next(it, end);

    switch (cp) {
    case 0x03B8: // θ (theta)
      result += "theta";
      break;
    case 0x03B1: // α (alpha)
      result += "alpha";
      break;
    default:
      // Convert code point back to UTF-8
      utf8::append(cp, std::back_inserter(result));
      break;
    }
  }

  return result;
}

inline std::string normalize_header(const std::string &raw) {
  return lowercase_copy(normalize_greek(trim_copy(raw)));
}

} // namespace utilities
} // namespace dhcalc