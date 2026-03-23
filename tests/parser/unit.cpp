#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <sstream>
#include <string>

#include "dhcalc/dh_table_parser.hpp"
#include "internal/dh_table_parser_impl.hpp"

static constexpr double kDeg2Rad = std::numbers::pi_v<double> / 180.0;
static constexpr double kEps = 1e-9;

// Helper to evaluate a SymEngine::Expression to double
static double eval_expr(const SymEngine::Expression &expr) {
  auto evaled = expand(expr);
  if (is_a<SymEngine::RealDouble>(*evaled.get_basic())) {
    return down_cast<const SymEngine::RealDouble &>(*evaled.get_basic()).i;
  }
  // For more complex expressions, try to evaluate numerically
  return eval_double(evaled);
}

// ─── Basic parsing ──────────────────────────────────────────────────────────

TEST(ParserBasic, SingleJointDegrees) {
  std::istringstream input(
      "joint\ttheta_deg\talpha_deg\tr\td\nJ1\t90\t0\t1.0\t0.0\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_EQ(joints[0].name, "J1");
  EXPECT_NEAR(eval_expr(joints[0].theta), 90.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), 0.0, kEps);
  EXPECT_NEAR(eval_expr(joints[0].r), 1.0, kEps);
  EXPECT_NEAR(eval_expr(joints[0].d), 0.0, kEps);
}

TEST(ParserBasic, SingleJointRadians) {
  std::istringstream input(
      "theta_rad\talpha_rad\tr\td\n1.5708\t0.0\t2.0\t0.5\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), 1.5708, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), 0.0, kEps);
  EXPECT_NEAR(eval_expr(joints[0].r), 2.0, kEps);
  EXPECT_NEAR(eval_expr(joints[0].d), 0.5, kEps);
}

TEST(ParserBasic, MultipleJoints) {
  std::istringstream input("joint\ttheta_deg\talpha_deg\tr\td\n"
                           "base\t0\t-90\t0\t0.5\n"
                           "link\t45\t0\t1.0\t0\n"
                           "tip\t-30\t90\t0.5\t0.2\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 3u);
  EXPECT_EQ(joints[0].name, "base");
  EXPECT_EQ(joints[1].name, "link");
  EXPECT_EQ(joints[2].name, "tip");
  EXPECT_NEAR(eval_expr(joints[1].theta), 45.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), -90.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[2].d), 0.2, kEps);
}

// ─── Symbolic expressions ───────────────────────────────────────────────────

TEST(ParserSymbolic, PiConstant) {
  std::istringstream input("theta_rad\talpha_rad\tr\td\npi\tpi/2\t1\t0\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), std::numbers::pi, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), std::numbers::pi / 2, kEps);
}

TEST(ParserSymbolic, ArithmeticExpressions) {
  std::istringstream input(
      "theta_deg\talpha_deg\tr\td\n90+45\t180-90\t2*1.5\t10/2\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), 135.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), 90.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].r), 3.0, kEps);
  EXPECT_NEAR(eval_expr(joints[0].d), 5.0, kEps);
}

TEST(ParserSymbolic, TrigonometricFunctions) {
  std::istringstream input(
      "theta_rad\talpha_rad\tr\td\nsin(pi/2)\tcos(0)\t1\t0\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), 1.0, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), 1.0, kEps);
}

TEST(ParserSymbolic, ComplexExpression) {
  std::istringstream input(
      "theta_rad\talpha_rad\tr\td\n2*pi/3\tsqrt(2)/2\t3^2\tlog(exp(1))\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), 2 * std::numbers::pi / 3, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), std::sqrt(2.0) / 2, kEps);
  EXPECT_NEAR(eval_expr(joints[0].r), 9.0, kEps);
  EXPECT_NEAR(eval_expr(joints[0].d), 1.0, kEps);
}

TEST(ParserSymbolic, NegativeExpressions) {
  std::istringstream input(
      "theta_deg\talpha_deg\tr\td\n-45\t-(90)\t-1*2\t-5\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), -45.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), -90.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].r), -2.0, kEps);
  EXPECT_NEAR(eval_expr(joints[0].d), -5.0, kEps);
}

TEST(ParserSymbolic, ParenthesesPrecedence) {
  std::istringstream input(
      "theta_deg\talpha_deg\tr\td\n(90+45)*2\t90/2+45\t(2+3)*4\t10/(2+3)\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), 270.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), 90.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].r), 20.0, kEps);
  EXPECT_NEAR(eval_expr(joints[0].d), 2.0, kEps);
}

// ─── Unicode Greek headers ──────────────────────────────────────────────────

TEST(ParserGreek, ThetaAlphaUnicode) {
  // θ = 0xCE 0xB8, α = 0xCE 0xB1
  std::istringstream input("joint\t\xCE\xB8_deg\t\xCE\xB1_deg\tr\td\n"
                           "J1\t45\t-90\t1.0\t0.0\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), 45.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), -90.0 * kDeg2Rad, kEps);
}

TEST(ParserGreek, ThetaRadUnicode) {
  std::istringstream input("\xCE\xB8_rad\t\xCE\xB1_rad\tr\td\n"
                           "3.14159\t1.5708\t0.0\t1.0\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), 3.14159, 1e-5);
  EXPECT_NEAR(eval_expr(joints[0].alpha), 1.5708, 1e-4);
}

TEST(ParserGreek, MixedUnicodeExpressions) {
  std::istringstream input("\xCE\xB8_rad\t\xCE\xB1_rad\tr\td\n"
                           "pi/4\t2*pi/3\tsqrt(2)\texp(0)\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), std::numbers::pi / 4, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), 2 * std::numbers::pi / 3, kEps);
}

// ─── Column order independence ──────────────────────────────────────────────

TEST(ParserColumnOrder, Reversed) {
  std::istringstream input(
      "d\tr\talpha_deg\ttheta_deg\tjoint\n0.5\t2.0\t90\t45\tRev\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_EQ(joints[0].name, "Rev");
  EXPECT_NEAR(eval_expr(joints[0].theta), 45.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), 90.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].r), 2.0, kEps);
  EXPECT_NEAR(eval_expr(joints[0].d), 0.5, kEps);
}

TEST(ParserColumnOrder, NoJointColumn) {
  std::istringstream input("theta_deg\talpha_deg\tr\td\n90\t0\t1\t0\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_TRUE(joints[0].name.empty());
}

TEST(ParserColumnOrder, Scrambled) {
  std::istringstream input("r\tjoint\td\ttheta_deg\talpha_deg\n"
                           "1.5\tJ2\t0.75\t30\t-45\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_EQ(joints[0].name, "J2");
  EXPECT_NEAR(eval_expr(joints[0].theta), 30.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), -45.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].r), 1.5, kEps);
  EXPECT_NEAR(eval_expr(joints[0].d), 0.75, kEps);
}

// ─── Comments and blank lines ───────────────────────────────────────────────

TEST(ParserComments, SkipsCommentsAndBlanks) {
  std::istringstream input("# This is a comment\n"
                           "\n"
                           "joint\ttheta_deg\talpha_deg\tr\td\n"
                           "# Another comment\n"
                           "\n"
                           "J1\t0\t0\t1\t0\n"
                           "\n"
                           "# End\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_EQ(joints[0].name, "J1");
}

TEST(ParserComments, InlineCommentsNotSupported) {
  // rapidcsv with '#' as comment char treats entire line as comment
  std::istringstream input("theta_deg\talpha_deg\tr\td\n"
                           "45\t90\t1\t0  # this joint is special\n");
  // The comment character makes the whole line a comment
  EXPECT_THROW(auto joints = dhcalc::detail::parse_dh_table_impl(input),
               dhcalc::ParseError);
}

// ─── Zero joints (header only) ──────────────────────────────────────────────

TEST(ParserZeroJoints, HeaderOnly) {
  std::istringstream input("joint\ttheta_deg\talpha_deg\tr\td\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  EXPECT_TRUE(joints.empty());
}

TEST(ParserZeroJoints, HeaderWithComments) {
  std::istringstream input("# Header only table\n"
                           "theta_rad\talpha_rad\tr\td\n"

                           "# No data rows\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  EXPECT_TRUE(joints.empty());
}

// ─── UTF-8 BOM handling ────────────────────────────────────────────────────

TEST(ParserBOM, Utf8BomStripped) {
  std::istringstream input(
      "\xEF\xBB\xBFjoint\ttheta_deg\talpha_deg\tr\td\nJ1\t0\t0\t1\t0\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_EQ(joints[0].name, "J1");
}

// ─── Negative and decimal values ────────────────────────────────────────────

TEST(ParserValues, NegativeAndDecimal) {
  std::istringstream input("theta_deg\talpha_deg\tr\td\n"
                           "-45.5\t-90.0\t0.123\t-0.456\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), -45.5 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), -90.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].r), 0.123, kEps);
  EXPECT_NEAR(eval_expr(joints[0].d), -0.456, kEps);
}

TEST(ParserValues, ScientificNotation) {
  std::istringstream input("theta_rad\talpha_rad\tr\td\n"
                           "1e-3\t2.5e2\t1.0e1\t-3e-4\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), 1e-3, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), 2.5e2, kEps);
  EXPECT_NEAR(eval_expr(joints[0].r), 10.0, kEps);
  EXPECT_NEAR(eval_expr(joints[0].d), -3e-4, kEps);
}

TEST(ParserValues, VerySmallNumbers) {
  std::istringstream input("theta_rad\talpha_rad\tr\td\n"
                           "1e-15\t-9.87654e-10\t0.0\t1.23e-8\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), 1e-15, 1e-20);
  EXPECT_NEAR(eval_expr(joints[0].alpha), -9.87654e-10, 1e-15);
  EXPECT_NEAR(eval_expr(joints[0].r), 0.0, kEps);
  EXPECT_NEAR(eval_expr(joints[0].d), 1.23e-8, 1e-13);
}

// ─── Mixed numeric and symbolic ─────────────────────────────────────────────

TEST(ParserMixed, NumericAndSymbolic) {
  std::istringstream input("joint\ttheta_deg\talpha_deg\tr\td\n"
                           "J1\t90\t0\tpi\t1.5\n"
                           "J2\t45+45\t-90\t2*sqrt(2)\t0\n"
                           "J3\t0\tpi*180/pi\t1\tsin(pi/6)*2\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 3u);

  EXPECT_NEAR(eval_expr(joints[0].theta), 90.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].r), std::numbers::pi, kEps);
  EXPECT_NEAR(eval_expr(joints[0].d), 1.5, kEps);

  EXPECT_NEAR(eval_expr(joints[1].theta), 90.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[1].alpha), -90.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[1].r), 2 * std::sqrt(2.0), kEps);

  EXPECT_NEAR(eval_expr(joints[2].alpha), 180.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[2].d), 1.0, kEps);
}

// ─── Whitespace handling ────────────────────────────────────────────────────

TEST(ParserWhitespace, ExtraSpacesInCells) {
  std::istringstream input("theta_deg\talpha_deg\tr\td\n"
                           "  45  \t  90  \t  1.5  \t  0.5  \n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), 45.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), 90.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].r), 1.5, kEps);
  EXPECT_NEAR(eval_expr(joints[0].d), 0.5, kEps);
}

TEST(ParserWhitespace, SpacesInExpressions) {
  std::istringstream input("theta_deg\talpha_deg\tr\td\n"
                           "45 + 45\t90 * 2 - 90\t2 * 1.5\t10 / 2\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), 90.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), 90.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].r), 3.0, kEps);
  EXPECT_NEAR(eval_expr(joints[0].d), 5.0, kEps);
}

// ─── Degree/radian conversion ───────────────────────────────────────────────

TEST(ParserConversion, DegreesAutoConvertedToRadians) {
  std::istringstream input("theta_deg\talpha_deg\tr\td\n"
                           "180\t90\t1\t0\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), std::numbers::pi, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), std::numbers::pi / 2, kEps);
}

TEST(ParserConversion, RadiansNotConverted) {
  std::istringstream input("theta_rad\talpha_rad\tr\td\n"
                           "3.14159\t1.5708\t1\t0\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), 3.14159, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), 1.5708, kEps);
}

TEST(ParserConversion, SymbolicDegreesConverted) {
  std::istringstream input("theta_deg\talpha_deg\tr\td\n"
                           "90+45\t180\t1\t0\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), 135.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), 180.0 * kDeg2Rad, kEps);
}

// ─── Stanford arm from file ─────────────────────────────────────────────────
// TODO: make normal test 
TEST(ParserFile, StanfordArmFile) {
  const auto path = std::filesystem::path(PROJECT_SOURCE_DIR) / "examples" /
                    "stanford_arm.dh.tsv";
  if (!std::filesystem::exists(path)) {
    GTEST_SKIP() << "Example file not found: " << path;
  }

  auto joints = dhcalc::parse_dh_table(path);
  ASSERT_EQ(joints.size(), 3u);

  EXPECT_EQ(joints[0].name, "1");
  EXPECT_NEAR(eval_expr(joints[0].alpha), 90.0 * kDeg2Rad, kEps);
  // EXPECT_NEAR(eval_expr(joints[0].d), 0.412, kEps);

  EXPECT_EQ(joints[2].name, "3");
  // Verify joint 3 has expected structure (prismatic joint)
  EXPECT_NO_THROW(eval_expr(joints[2].d));
}

TEST(ParserFile, EmptyTableFile) {
  const auto path =
      std::filesystem::path(PROJECT_SOURCE_DIR) / "examples" / "empty.dh.tsv";
  if (!std::filesystem::exists(path)) {
    GTEST_SKIP() << "Empty table file not found: " << path;
  }

  auto joints = dhcalc::parse_dh_table(path);
  EXPECT_TRUE(joints.empty());
}

// ─── Error cases ────────────────────────────────────────────────────────────

TEST(ParserErrors, EmptyInput) {
  std::istringstream input("");
  EXPECT_THROW(dhcalc::detail::parse_dh_table_impl(input), dhcalc::ParseError);
}

TEST(ParserErrors, OnlyComments) {
  std::istringstream input("# just a comment\n# another\n");
  EXPECT_THROW(dhcalc::detail::parse_dh_table_impl(input), dhcalc::ParseError);
}

TEST(ParserErrors, MissingRColumn) {
  std::istringstream input("theta_deg\talpha_deg\td\n0\t0\t0\n");
  EXPECT_THROW(dhcalc::detail::parse_dh_table_impl(input), dhcalc::ParseError);
}

TEST(ParserErrors, MissingDColumn) {
  std::istringstream input("theta_deg\talpha_deg\tr\n0\t0\t0\n");
  EXPECT_THROW(dhcalc::detail::parse_dh_table_impl(input), dhcalc::ParseError);
}

TEST(ParserErrors, BothThetaDegAndRad) {
  std::istringstream input(
      "theta_deg\ttheta_rad\talpha_deg\tr\td\n0\t0\t0\t0\t0\n");
  EXPECT_THROW(dhcalc::detail::parse_dh_table_impl(input), dhcalc::ParseError);
}

TEST(ParserErrors, BothAlphaDegAndRad) {
  std::istringstream input(
      "theta_deg\talpha_deg\talpha_rad\tr\td\n0\t0\t0\t0\t0\n");
  EXPECT_THROW(dhcalc::detail::parse_dh_table_impl(input), dhcalc::ParseError);
}

TEST(ParserErrors, NoThetaColumn) {
  std::istringstream input("alpha_deg\tr\td\n0\t0\t0\n");
  EXPECT_THROW(dhcalc::detail::parse_dh_table_impl(input), dhcalc::ParseError);
}

TEST(ParserErrors, NoAlphaColumn) {
  std::istringstream input("theta_deg\tr\td\n0\t0\t0\n");
  EXPECT_THROW(dhcalc::detail::parse_dh_table_impl(input), dhcalc::ParseError);
}

TEST(ParserErrors, ColumnCountMismatch) {
  std::istringstream input("theta_deg\talpha_deg\tr\td\n0\t0\t0\n");
  EXPECT_THROW(dhcalc::detail::parse_dh_table_impl(input), dhcalc::ParseError);
}

TEST(ParserErrors, EmptyNumericField) {
  std::istringstream input("theta_deg\talpha_deg\tr\td\n\t0\t0\t0\n");
  EXPECT_THROW(dhcalc::detail::parse_dh_table_impl(input), dhcalc::ParseError);
}

TEST(ParserErrors, UnclosedParenthesis) {
  std::istringstream input("theta_deg\talpha_deg\tr\td\n(10+5\t0\t0\t0\n");
  EXPECT_THROW(dhcalc::detail::parse_dh_table_impl(input), dhcalc::ParseError);
}

TEST(ParserErrors, DivisionByZero) {
  // SymEngine parser should handle this, but the expression might not evaluate
  std::istringstream input("theta_deg\talpha_deg\tr\td\n10/0\t0\t0\t0\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);
  // Parser succeeds, but evaluation would fail/return inf
  ASSERT_EQ(joints.size(), 1u);
}

TEST(ParserErrors, DuplicateColumn) {
  std::istringstream input("theta_deg\talpha_deg\tr\tr\td\n0\t0\t0\t0\t0\n");
  EXPECT_THROW(dhcalc::detail::parse_dh_table_impl(input), dhcalc::ParseError);
}

TEST(ParserErrors, FileNotFound) {
  EXPECT_THROW(dhcalc::parse_dh_table("/nonexistent/path/to/file.tsv"),
               dhcalc::ParseError);
}

TEST(ParserErrors, TooManyColumns) {
  std::istringstream input("theta_deg\talpha_deg\tr\td\n0\t0\t0\t0\t999\n");
  EXPECT_THROW(dhcalc::detail::parse_dh_table_impl(input), dhcalc::ParseError);
}

TEST(ParserErrors, EmptyColumnName) {
  std::istringstream input("theta_deg\t\tr\td\n0\t0\t0\t0\n");
  EXPECT_THROW(dhcalc::detail::parse_dh_table_impl(input), dhcalc::ParseError);
}

TEST(ParserErrors, MismatchedDataTypes) {
  // All cells with mismatched tabs
  std::istringstream input("theta_deg\talpha_deg\tr\td\n"
                           "45\t90\n"
                           "30\t60\t1\t0\n");
  EXPECT_THROW(dhcalc::detail::parse_dh_table_impl(input), dhcalc::ParseError);
}

// ─── Advanced symbolic cases ────────────────────────────────────────────────

TEST(ParserAdvanced, NestedFunctions) {
  std::istringstream input(
      "theta_rad\talpha_rad\tr\td\n"
      "sin(cos(pi/4))\tlog(exp(2))\tsqrt(abs(-16))\tfloor(3.7)\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta),
              std::sin(std::cos(std::numbers::pi / 4)), kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), 2.0, kEps);
  EXPECT_NEAR(eval_expr(joints[0].r), 4.0, kEps);
  EXPECT_NEAR(eval_expr(joints[0].d), 3.0, kEps);
}

TEST(ParserAdvanced, PowersAndRoots) {
  std::istringstream input("theta_rad\talpha_rad\tr\td\n"
                           "2^3\t3^2\t16^(1/2)\t27^(1/3)\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), 8.0, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), 9.0, kEps);
  EXPECT_NEAR(eval_expr(joints[0].r), 4.0, kEps);
  EXPECT_NEAR(eval_expr(joints[0].d), 3.0, kEps);
}

TEST(ParserAdvanced, ComplexArithmetic) {
  std::istringstream input(
      "theta_deg\talpha_deg\tr\td\n"
      "((90*2-45)/3+15)*2\t360/4-45\t(1+2)*(4-1)\t10-5*2+3\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), 120.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), 45.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].r), 9.0, kEps);
  EXPECT_NEAR(eval_expr(joints[0].d), 3.0, kEps);
}

TEST(ParserAdvanced, MixedConstants) {
  std::istringstream input("theta_rad\talpha_rad\tr\td\n"
                           "pi+1\te-1\tpi*e\tsqrt(2)*sqrt(3)\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), std::numbers::pi + 1, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), std::numbers::e - 1, kEps);
  EXPECT_NEAR(eval_expr(joints[0].r), std::numbers::pi * std::numbers::e, kEps);
  EXPECT_NEAR(eval_expr(joints[0].d), std::sqrt(6.0), kEps);
}

// ─── Edge cases with joint names ────────────────────────────────────────────

TEST(ParserJointNames, SpecialCharacters) {
  std::istringstream input("joint\ttheta_deg\talpha_deg\tr\td\n"
                           "joint_1\t0\t0\t1\t0\n"
                           "joint-2\t45\t90\t1\t0\n"
                           "joint.3\t90\t-90\t1\t0\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 3u);
  EXPECT_EQ(joints[0].name, "joint_1");
  EXPECT_EQ(joints[1].name, "joint-2");
  EXPECT_EQ(joints[2].name, "joint.3");
}

TEST(ParserJointNames, NumericNames) {
  std::istringstream input("joint\ttheta_deg\talpha_deg\tr\td\n"
                           "1\t0\t0\t1\t0\n"
                           "2\t45\t90\t1\t0\n"
                           "123\t90\t-90\t1\t0\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 3u);
  EXPECT_EQ(joints[0].name, "1");
  EXPECT_EQ(joints[1].name, "2");
  EXPECT_EQ(joints[2].name, "123");
}

TEST(ParserJointNames, WhitespaceInNames) {
  std::istringstream input("joint\ttheta_deg\talpha_deg\tr\td\n"
                           "  base  \t0\t0\t1\t0\n"
                           "link 1\t45\t90\t1\t0\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 2u);
  EXPECT_EQ(joints[0].name, "base");   // trimmed
  EXPECT_EQ(joints[1].name, "link 1"); // internal space preserved
}

TEST(ParserJointNames, EmptyName) {
  std::istringstream input("joint\ttheta_deg\talpha_deg\tr\td\n"
                           "\t0\t0\t1\t0\n"
                           "J2\t45\t90\t1\t0\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 2u);
  EXPECT_TRUE(joints[0].name.empty());
  EXPECT_EQ(joints[1].name, "J2");
}

// ─── Large tables ───────────────────────────────────────────────────────────

TEST(ParserLarge, TenJoints) {
  std::ostringstream oss;
  oss << "joint\ttheta_deg\talpha_deg\tr\td\n";
  for (int i = 0; i < 10; ++i) {
    oss << "J" << i << "\t" << (i * 10) << "\t" << (i * 5) << "\t" << (i * 0.1)
        << "\t" << (i * 0.05) << "\n";
  }

  std::istringstream input(oss.str());
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 10u);
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(joints[i].name, "J" + std::to_string(i));
    EXPECT_NEAR(eval_expr(joints[i].theta), (i * 10.0) * kDeg2Rad, kEps);
    EXPECT_NEAR(eval_expr(joints[i].alpha), (i * 5.0) * kDeg2Rad, kEps);
    EXPECT_NEAR(eval_expr(joints[i].r), i * 0.1, kEps);
    EXPECT_NEAR(eval_expr(joints[i].d), i * 0.05, kEps);
  }
}

TEST(ParserLarge, MixedSymbolicAndNumeric) {
  std::istringstream input("joint\ttheta_deg\talpha_deg\tr\td\n"
                           "J1\t90\t0\t1\t0\n"
                           "J2\t0\t-90\tsqrt(2)\t0.5\n"
                           "J3\t45+45\tpi*180/pi\t2*1.5\tsin(pi/6)*4\n"
                           "J4\t-30\t60\texp(0)\tlog(e)\n"
                           "J5\t0\t0\tpi/2\t2^3/4\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 5u);

  EXPECT_NEAR(eval_expr(joints[1].r), std::sqrt(2.0), kEps);
  EXPECT_NEAR(eval_expr(joints[2].theta), 90.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[2].d), 2.0, kEps);
  EXPECT_NEAR(eval_expr(joints[3].r), 1.0, kEps);
  EXPECT_NEAR(eval_expr(joints[4].r), std::numbers::pi / 2, kEps);
  EXPECT_NEAR(eval_expr(joints[4].d), 2.0, kEps);
}

// ─── Case sensitivity ───────────────────────────────────────────────────────

TEST(ParserCase, MixedCaseHeaders) {
  std::istringstream input("THETA_DEG\tALPHA_DEG\tR\tD\n90\t0\t1\t0\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), 90.0 * kDeg2Rad, kEps);
}

TEST(ParserCase, MixedCaseGreek) {
  std::istringstream input("Theta_Deg\tAlpha_Deg\tr\td\n45\t90\t1\t0\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(eval_expr(joints[0].theta), 45.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(eval_expr(joints[0].alpha), 90.0 * kDeg2Rad, kEps);
}
