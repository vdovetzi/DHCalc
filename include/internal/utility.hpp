#pragma once

#include <utf8.h>

#include <symengine/visitor.h>

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

// ── SymEngine utilities
// ────────────────────────────────────────────────────────

inline SymEngine::RCP<const SymEngine::Basic>
clean_expression(const SymEngine::RCP<const SymEngine::Basic> &expr,
                 double zero_threshold = 1e-10, int precision = 6) {
  SymEngine::map_basic_basic subs_map;

  std::function<void(const SymEngine::RCP<const SymEngine::Basic> &)> collect =
      [&](const SymEngine::RCP<const SymEngine::Basic> &node) {
        if (is_a<SymEngine::RealDouble>(*node)) {
          double val = down_cast<const SymEngine::RealDouble &>(*node).i;

          if (std::abs(val) < zero_threshold) {
            subs_map[node] = SymEngine::integer(0);
          } else {
            int d = precision - 1 -
                    static_cast<int>(std::floor(std::log10(std::abs(val))));
            double factor = std::pow(10.0, d);
            double rounded = std::round(val * factor) / factor;
            if (rounded != val) {
              subs_map[node] = SymEngine::real_double(rounded);
            }
          }
        } else {
          for (const auto &arg : node->get_args()) {
            collect(arg);
          }
        }
      };

  collect(expr);

  if (subs_map.empty())
    return expr;
  return expr->subs(subs_map);
}

} // namespace utilities
} // namespace dhcalc