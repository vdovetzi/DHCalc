#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <sstream>
#include <string>

#include "dhcalc/dh_table_parser.hpp"
#include "internal/dh_table_parser_impl.hpp"

static constexpr double kDeg2Rad = std::numbers::pi_v<double> / 180.0;
static constexpr double kEps = 1e-12;

// ─── Basic parsing ──────────────────────────────────────────────────────────

TEST(ParserBasic, SingleJointDegrees) {
  std::istringstream input(
      "joint\ttheta_deg\talpha_deg\tr\td\nJ1\t90\t0\t1.0\t0.0\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_EQ(joints[0].name, "J1");
  EXPECT_NEAR(joints[0].theta_radians, 90.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(joints[0].alpha_radians, 0.0, kEps);
  EXPECT_NEAR(joints[0].r, 1.0, kEps);
  EXPECT_NEAR(joints[0].d, 0.0, kEps);
}

TEST(ParserBasic, SingleJointRadians) {
  std::istringstream input(
      "theta_rad\talpha_rad\tr\td\n1.5708\t0.0\t2.0\t0.5\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(joints[0].theta_radians, 1.5708, kEps);
  EXPECT_NEAR(joints[0].r, 2.0, kEps);
  EXPECT_NEAR(joints[0].d, 0.5, kEps);
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
  EXPECT_NEAR(joints[1].theta_radians, 45.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(joints[0].alpha_radians, -90.0 * kDeg2Rad, kEps);
}

// ─── Unicode Greek headers ──────────────────────────────────────────────────

TEST(ParserGreek, ThetaAlphaUnicode) {
  // θ = 0xCE 0xB8, α = 0xCE 0xB1
  std::istringstream input("joint\t\xCE\xB8_deg\t\xCE\xB1_deg\tr\td\n"
                           "J1\t45\t-90\t1.0\t0.0\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(joints[0].theta_radians, 45.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(joints[0].alpha_radians, -90.0 * kDeg2Rad, kEps);
}

TEST(ParserGreek, ThetaRadUnicode) {
  std::istringstream input("\xCE\xB8_rad\t\xCE\xB1_rad\tr\td\n"
                           "3.14159\t1.5708\t0.0\t1.0\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(joints[0].theta_radians, 3.14159, 1e-5);
  EXPECT_NEAR(joints[0].alpha_radians, 1.5708, 1e-4);
}

// ─── Column order independence ──────────────────────────────────────────────

TEST(ParserColumnOrder, Reversed) {
  std::istringstream input(
      "d\tr\talpha_deg\ttheta_deg\tjoint\n0.5\t2.0\t90\t45\tRev\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_EQ(joints[0].name, "Rev");
  EXPECT_NEAR(joints[0].theta_radians, 45.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(joints[0].alpha_radians, 90.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(joints[0].r, 2.0, kEps);
  EXPECT_NEAR(joints[0].d, 0.5, kEps);
}

TEST(ParserColumnOrder, NoJointColumn) {
  std::istringstream input("theta_deg\talpha_deg\tr\td\n90\t0\t1\t0\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_TRUE(joints[0].name.empty());
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

// ─── Zero joints (header only) ─────────────────────────────────────────────

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
  EXPECT_NEAR(joints[0].theta_radians, -45.5 * kDeg2Rad, kEps);
  EXPECT_NEAR(joints[0].alpha_radians, -90.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(joints[0].r, 0.123, kEps);
  EXPECT_NEAR(joints[0].d, -0.456, kEps);
}

TEST(ParserValues, ScientificNotation) {
  std::istringstream input("theta_rad\talpha_rad\tr\td\n"
                           "1e-3\t2.5e2\t1.0e1\t-3e-4\n");
  auto joints = dhcalc::detail::parse_dh_table_impl(input);

  ASSERT_EQ(joints.size(), 1u);
  EXPECT_NEAR(joints[0].theta_radians, 1e-3, kEps);
  EXPECT_NEAR(joints[0].alpha_radians, 2.5e2, kEps);
  EXPECT_NEAR(joints[0].r, 10.0, kEps);
  EXPECT_NEAR(joints[0].d, -3e-4, kEps);
}

// ─── Stanford arm from file ─────────────────────────────────────────────────

TEST(ParserFile, StanfordArmFile) {
  const auto path = std::filesystem::path(PROJECT_SOURCE_DIR) / "examples" /
                    "stanford_arm.dh.tsv";
  if (!std::filesystem::exists(path)) {
    GTEST_SKIP() << "Example file not found: " << path;
  }

  auto joints = dhcalc::parse_dh_table(path);
  ASSERT_EQ(joints.size(), 6u);

  EXPECT_EQ(joints[0].name, "1");
  EXPECT_NEAR(joints[0].alpha_radians, -90.0 * kDeg2Rad, kEps);
  EXPECT_NEAR(joints[0].d, 0.412, kEps);

  EXPECT_EQ(joints[2].name, "3");
  EXPECT_NEAR(joints[2].d, 0.5, kEps);
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

TEST(ParserErrors, InvalidNumber) {
  std::istringstream input("theta_deg\talpha_deg\tr\td\nabc\t0\t0\t0\n");
  EXPECT_THROW(dhcalc::detail::parse_dh_table_impl(input), dhcalc::ParseError);
}

TEST(ParserErrors, EmptyNumericField) {
  std::istringstream input("theta_deg\talpha_deg\tr\td\n\t0\t0\t0\n");
  EXPECT_THROW(dhcalc::detail::parse_dh_table_impl(input), dhcalc::ParseError);
}

TEST(ParserErrors, TrailingGarbageInNumber) {
  std::istringstream input("theta_deg\talpha_deg\tr\td\n10xyz\t0\t0\t0\n");
  EXPECT_THROW(dhcalc::detail::parse_dh_table_impl(input), dhcalc::ParseError);
}

TEST(ParserErrors, DuplicateColumn) {
  std::istringstream input("theta_deg\talpha_deg\tr\tr\td\n0\t0\t0\t0\t0\n");
  EXPECT_THROW(dhcalc::detail::parse_dh_table_impl(input), dhcalc::ParseError);
}

TEST(ParserErrors, FileNotFound) {
  EXPECT_THROW(dhcalc::parse_dh_table("/nonexistent/path/to/file.tsv"),
               dhcalc::ParseError);
}
