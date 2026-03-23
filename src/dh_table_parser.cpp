#include "dhcalc/dh_table_parser.hpp"
#include "internal/utility.hpp"
#include "symengine/parser/parser.h"

#include <rapidcsv.h>
#include <utf8.h>

#include <optional>

namespace dhcalc {
namespace {

// ── Column resolution ───────────────────────────────────────────────────────

struct ResolvedColumns {
  std::string theta_col;
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
  const std::size_t column_count = doc.GetColumnCount();

  SymEngine::Parser parser;

  std::vector<DHFrameParameters> joints;
  joints.reserve(row_count);

  for (std::size_t i = 0; i < row_count; ++i) {
    try {
      std::vector<std::string> row_cells = doc.GetRow<std::string>(i);
      if (row_cells.size() != column_count) {
        throw ParseError("Row " + std::to_string(i + 2) + ": expected " +
                         std::to_string(column_count) + " columns but found " +
                         std::to_string(row_cells.size()));
      }
    } catch (const ParseError &) {
      throw;
    } catch (const std::exception &e) {
      throw ParseError("Row " + std::to_string(i + 2) +
                       ": error reading row - " + std::string(e.what()));
    }
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

    auto get_cell = [&](const std::string &col) -> std::string {
      try {
        return doc.GetCell<std::string>(col, i);
      } catch (const std::exception &) {
        throw ParseError("Row " + std::to_string(i + 1) + ", column '" + col +
                         "': missing value.");
      }
    };

    std::string theta_str = utilities::trim_copy(get_cell(cols.theta_col));
    std::string alpha_str = utilities::trim_copy(get_cell(cols.alpha_col));
    std::string r_str = utilities::trim_copy(get_cell(cols.r_col));
    std::string d_str = utilities::trim_copy(get_cell(cols.d_col));

    if (theta_str.empty() || alpha_str.empty() || r_str.empty() ||
        d_str.empty()) {
      throw ParseError("Row " + std::to_string(i + 2) +
                       ": empty value in required column.");
    }

    try {
      joint.theta = parser.parse(theta_str);
      joint.alpha = parser.parse(alpha_str);
      joint.r = parser.parse(r_str);
      joint.d = parser.parse(d_str);
    } catch (const SymEngine::ParseError &e) {
      throw ParseError("Row " + std::to_string(i + 2) +
                       ": invalid expression '" + std::string(e.what()) + "'");
    } catch (const SymEngine::SymEngineException &e) {
      throw ParseError("Row " + std::to_string(i + 2) +
                       ": failed to parse expression - " +
                       std::string(e.what()));
    } catch (const std::exception &e) {
      throw ParseError("Row " + std::to_string(i + 2) +
                       ": unexpected error parsing expression - " +
                       std::string(e.what()));
    }

    // ── Unit conversion ─────────────────────────────────────────────
    static const auto deg_rad_factor =
        SymEngine::Expression(SymEngine::real_double(std::numbers::pi)) / 180.0;

    if (cols.theta_in_degrees) {
      joint.theta = joint.theta * deg_rad_factor;
    }
    if (cols.alpha_in_degrees) {
      joint.alpha = joint.alpha * deg_rad_factor;
    }

    joints.emplace_back(std::move(joint));
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
