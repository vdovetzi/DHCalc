#include "dhcalc/dh_table_parser.hpp"
#include "internal/utility.hpp"

#include <exception>
#include <rapidcsv.h>
#include <utf8.h>

#include <cctype>
#include <fstream>
#include <istream>
#include <numbers>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace dhcalc {
namespace {

constexpr double kDegreesToRadians = std::numbers::pi_v<double> / 180.0;

// ── Column resolution ───────────────────────────────────────────────────────

struct ResolvedColumns {
  std::string theta_col; // original column name in the document
  bool theta_in_degrees{};
  std::string alpha_col;
  bool alpha_in_degrees{};
  std::string r_col;
  std::string d_col;
  std::optional<std::string> joint_col;
};

ResolvedColumns
resolve_columns(const std::vector<std::string> &raw_column_names) {
  std::unordered_map<std::string, std::string> lookup;

  for (const auto &raw : raw_column_names) {
    std::string key = utilities::normalize_header(raw);
    if (key.empty()) {
      throw ParseError("Header contains an empty column name.");
    }

    if (key == "theta" || key == "alpha") {
      key += "_deg";
    }

    if (!lookup.emplace(key, raw).second) {
      throw ParseError("Duplicate column '" + key + "' in header row.");
    }
  }

  // ── Theta ───────────────────────────────────────────────────────────
  const bool has_theta_deg = lookup.contains("theta_deg");
  const bool has_theta_rad = lookup.contains("theta_rad");

  if (has_theta_deg == has_theta_rad) {
    throw ParseError("Provide exactly one of 'θ_deg'/'theta_deg' or "
                     "'θ_rad'/'theta_rad' in the header.");
  }

  // ── Alpha ───────────────────────────────────────────────────────────
  const bool has_alpha_deg = lookup.contains("alpha_deg");
  const bool has_alpha_rad = lookup.contains("alpha_rad");

  if (has_alpha_deg == has_alpha_rad) {
    throw ParseError("Provide exactly one of 'α_deg'/'alpha_deg' or "
                     "'α_rad'/'alpha_rad' in the header.");
  }

  // ── Required columns ────────────────────────────────────────────────
  auto require = [&](const std::string &key) -> std::string {
    const auto it = lookup.find(key);
    if (it == lookup.end()) {
      throw ParseError("Required column '" + key +
                       "' is missing from the header.");
    }
    return it->second;
  };

  ResolvedColumns cols;
  cols.theta_col = has_theta_deg ? require("theta_deg") : require("theta_rad");
  cols.theta_in_degrees = has_theta_deg;
  cols.alpha_col = has_alpha_deg ? require("alpha_deg") : require("alpha_rad");
  cols.alpha_in_degrees = has_alpha_deg;
  cols.r_col = require("r");
  cols.d_col = require("d");

  const auto jit = lookup.find("joint");
  if (jit != lookup.end()) {
    cols.joint_col = jit->second;
  }

  return cols;
}

// ── Strict numeric parsing ──────────────────────────────────────────────────
// rapidcsv's built-in GetCell<double> may silently accept trailing garbage
// ("10xyz" → 10.0).  We parse as string first and validate the entire token.

double parse_double_strict(const std::string &raw, std::size_t row,
                           const std::string &column) {
  const std::string value = utilities::trim_copy(raw);
  if (value.empty()) {
    throw ParseError("Row " + std::to_string(row + 1) + ", column '" + column +
                     "': value is empty.");
  }

  try {
    std::size_t parsed_characters = 0;
    const double result = std::stod(value, &parsed_characters);
    if (parsed_characters != value.size()) {
      throw ParseError("Row " + std::to_string(row + 1) + ", column '" +
                       column + "': '" + value + "' is not a valid number.");
    }
    return result;
  } catch (const ParseError &) {
    throw;
  } catch (const std::exception &) {
    throw ParseError("Row " + std::to_string(row + 1) + ", column '" + column +
                     "': '" + value + "' is not a valid number.");
  }
}

double get_numeric_cell(const rapidcsv::Document &doc,
                        const std::string &column, std::size_t row) {
  std::string raw;
  try {
    raw = doc.GetCell<std::string>(column, row);
  } catch (const std::exception &) {
    throw ParseError("Row " + std::to_string(row + 1) + ", column '" + column +
                     "': missing value.");
  }
  return parse_double_strict(raw, row, column);
}

} // namespace

namespace detail {

std::vector<DHFrameParameters> parse_dh_table_impl(std::istream &input) {
  rapidcsv::Document doc(
      input, rapidcsv::LabelParams(0, -1), // first row = column headers
      rapidcsv::SeparatorParams('\t'),     // tab-separated
      rapidcsv::ConverterParams(),         // defaults
      rapidcsv::LineReaderParams(true, '#',
                                 true) // skip # comments and blank lines
  );

  ResolvedColumns cols;

  try {
    const auto column_names = doc.GetColumnNames();
    cols = resolve_columns(column_names);
  } catch (const std::exception &) {
    throw ParseError("DH table is empty or missing a header row.");
  }

  const std::size_t row_count = doc.GetRowCount();

  std::vector<DHFrameParameters> joints;
  joints.reserve(row_count);

  for (std::size_t i = 0; i < row_count; ++i) {
    DHFrameParameters joint;

    // ── Optional joint name ─────────────────────────────────────────
    if (cols.joint_col.has_value()) {
      try {
        joint.name =
            utilities::trim_copy(doc.GetCell<std::string>(*cols.joint_col, i));
      } catch (...) {
        joint.name.clear();
      }
    }

    // ── Required numeric parameters ─────────────────────────────────
    joint.theta_radians = get_numeric_cell(doc, cols.theta_col, i);
    joint.alpha_radians = get_numeric_cell(doc, cols.alpha_col, i);
    joint.r = get_numeric_cell(doc, cols.r_col, i);
    joint.d = get_numeric_cell(doc, cols.d_col, i);

    // ── Unit conversion ─────────────────────────────────────────────
    if (cols.theta_in_degrees) {
      joint.theta_radians *= kDegreesToRadians;
    }
    if (cols.alpha_in_degrees) {
      joint.alpha_radians *= kDegreesToRadians;
    }

    joints.push_back(std::move(joint));
  }

  return joints;
}

} // namespace detail

// ── Public API ──────────────────────────────────────────────────────────────

std::vector<DHFrameParameters>
parse_dh_table(const std::filesystem::path &path) {
  std::ifstream input(path);
  if (!input.is_open()) {
    throw ParseError("Unable to open DH table file: " + path.string());
  }

  return detail::parse_dh_table_impl(input);
}

} // namespace dhcalc
