#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "dhcalc/dh_table_parser.hpp"
#include "dhcalc/kinematics.hpp"

namespace fs = std::filesystem;

static constexpr double kEps = 1e-10;

namespace {

// Helper to evaluate SymEngine::Expression to double
static double eval_expr(const SymEngine::Expression &expr) {
  auto evaled = expand(expr);
  if (is_a<SymEngine::RealDouble>(*evaled.get_basic())) {
    return down_cast<const SymEngine::RealDouble &>(*evaled.get_basic()).i;
  }
  // For more complex expressions, try to evaluate numerically
  return eval_double(evaled);
}

// Helper to get matrix element as double
double get_matrix_elem(const SymEngine::DenseMatrix &M, size_t i, size_t j) {
  return eval_expr(SymEngine::Expression(M.get(i, j)));
}

// Create a unique temporary directory under /tmp/
fs::path create_temp_dir() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<uint64_t> dist;

  for (int attempt = 0; attempt < 100; ++attempt) {
    std::ostringstream name;
    name << "dhcalc_stress_" << std::hex << dist(gen);
    fs::path dir = fs::temp_directory_path() / name.str();

    if (fs::create_directories(dir)) {
      return dir;
    }
  }

  throw std::runtime_error(
      "Failed to create temporary directory after 100 attempts.");
}

// Generate a TSV file with the given number of joints and random DH parameters.
fs::path generate_tsv(const fs::path &dir, std::size_t joint_count,
                      unsigned seed) {
  std::mt19937 gen(seed);
  std::uniform_real_distribution<double> angle_dist(-180.0, 180.0);
  std::uniform_real_distribution<double> length_dist(-5.0, 5.0);

  std::ostringstream filename;
  filename << "stress_" << joint_count << "_joints_seed_" << seed << ".dh.tsv";
  fs::path path = dir / filename.str();

  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error("Failed to create stress test file: " +
                             path.string());
  }

  out << "joint\t\xCE\xB8_deg\t\xCE\xB1_deg\tr\td\n";

  for (std::size_t i = 0; i < joint_count; ++i) {
    out << "J" << i << '\t' << angle_dist(gen) << '\t' << angle_dist(gen)
        << '\t' << length_dist(gen) << '\t' << length_dist(gen) << '\n';
  }

  return path;
}

} // namespace

class StressTestFixture : public ::testing::Test {
protected:
  fs::path temp_dir_;

  void SetUp() override {
    temp_dir_ = create_temp_dir();
    std::cout << "[StressTest] Temp dir: " << temp_dir_ << std::endl;
  }

  void TearDown() override {
    std::error_code ec;
    fs::remove_all(temp_dir_, ec);
  }
};

// ─── Parse correctness at various sizes ─────────────────────────────────────

TEST_F(StressTestFixture, Parse1Joint) {
  auto path = generate_tsv(temp_dir_, 1, 42);
  auto joints = dhcalc::parse_dh_table(path);
  EXPECT_EQ(joints.size(), 1u);
}

TEST_F(StressTestFixture, Parse10Joints) {
  auto path = generate_tsv(temp_dir_, 10, 100);
  auto joints = dhcalc::parse_dh_table(path);
  EXPECT_EQ(joints.size(), 10u);
}

TEST_F(StressTestFixture, Parse100Joints) {
  auto path = generate_tsv(temp_dir_, 100, 200);
  auto joints = dhcalc::parse_dh_table(path);
  EXPECT_EQ(joints.size(), 100u);
}

TEST_F(StressTestFixture, Parse1000Joints) {
  auto path = generate_tsv(temp_dir_, 1000, 300);
  auto joints = dhcalc::parse_dh_table(path);
  EXPECT_EQ(joints.size(), 1000u);
}

TEST_F(StressTestFixture, Parse10000Joints) {
  auto path = generate_tsv(temp_dir_, 10000, 400);
  auto joints = dhcalc::parse_dh_table(path);
  EXPECT_EQ(joints.size(), 10000u);
}

// ─── FK result is always a valid homogeneous transform ─────────────────────

TEST_F(StressTestFixture, FKResultIsValidTransform) {
  const std::vector<size_t> sizes = {1, 5, 20, 100, 1000, 10000};

  for (std::size_t size : sizes) {
    SCOPED_TRACE("Testing size: " + std::to_string(size));

    auto path = generate_tsv(temp_dir_, size, static_cast<unsigned>(size * 7));
    auto joints = dhcalc::parse_dh_table(path);
    auto T = dhcalc::compute_FK(joints);

    ASSERT_EQ(T.nrows(), 4u);
    ASSERT_EQ(T.ncols(), 4u);

    // Last row must be [0, 0, 0, 1]
    EXPECT_NEAR(get_matrix_elem(T, 3, 0), 0.0, kEps) << "size=" << size;
    EXPECT_NEAR(get_matrix_elem(T, 3, 1), 0.0, kEps) << "size=" << size;
    EXPECT_NEAR(get_matrix_elem(T, 3, 2), 0.0, kEps) << "size=" << size;
    EXPECT_NEAR(get_matrix_elem(T, 3, 3), 1.0, kEps) << "size=" << size;

    // Extract rotation part and check orthonormality: R^T * R ≈ I
    // Build 3x3 rotation matrix
    double R[3][3];
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        R[i][j] = get_matrix_elem(T, i, j);
      }
    }

    // Compute R^T * R manually
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        double sum = 0.0;
        for (int k = 0; k < 3; ++k) {
          sum += R[k][r] * R[k][c]; // R^T[r][k] * R[k][c]
        }
        double expected = (r == c) ? 1.0 : 0.0;
        EXPECT_NEAR(sum, expected, 1e-6)
            << "size=" << size << " at (" << r << "," << c << ")";
      }
    }
  }
}

// ─── Performance: parse timing ──────────────────────────────────────────────

TEST_F(StressTestFixture, ParsePerformance) {
  const std::vector<size_t> cases = {
      100,
      1000,
      5000,
      10000,
  };

  std::cout << "\n[ParsePerf] Timing parse + FK for generated tables:\n";
  std::cout << "[ParsePerf] " << std::string(50, '-') << "\n";

  for (const size_t joint_count : cases) {
    auto path = generate_tsv(temp_dir_, joint_count,
                             static_cast<unsigned>(joint_count * 21));

    auto t0 = std::chrono::steady_clock::now();
    auto joints = dhcalc::parse_dh_table(path);
    auto t1 = std::chrono::steady_clock::now();
    auto T = dhcalc::compute_FK(joints);
    auto t2 = std::chrono::steady_clock::now();

    auto parse_us =
        std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    auto fk_us =
        std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    auto total_us = parse_us + fk_us;

    (void)T; // suppress unused warning

    std::cout << "[ParsePerf]  " << joint_count
              << " joints  =>  parse: " << parse_us << " µs, FK: " << fk_us
              << " µs, total: " << total_us << " µs\n";

    EXPECT_EQ(joints.size(), joint_count);
  }

  std::cout << "[ParsePerf] " << std::string(50, '-') << "\n";
}

// ─── Consistency: same seed produces same result ────────────────────────────

TEST_F(StressTestFixture, DeterministicWithSameSeed) {
  auto path1 = generate_tsv(temp_dir_, 200, 9999);
  auto path2 = generate_tsv(temp_dir_, 200, 9999);

  auto joints1 = dhcalc::parse_dh_table(path1);
  auto joints2 = dhcalc::parse_dh_table(path2);

  ASSERT_EQ(joints1.size(), joints2.size());

  auto T1 = dhcalc::compute_FK(joints1);
  auto T2 = dhcalc::compute_FK(joints2);

  // Compare all elements
  for (size_t r = 0; r < 4; ++r) {
    for (size_t c = 0; c < 4; ++c) {
      double val1 = get_matrix_elem(T1, r, c);
      double val2 = get_matrix_elem(T2, r, c);
      EXPECT_NEAR(val1, val2, kEps) << "at (" << r << "," << c << ")";
    }
  }
}

// ─── Edge: very large r and d values ────────────────────────────────────────

TEST_F(StressTestFixture, LargeValues) {
  std::ofstream out(temp_dir_ / "large.dh.tsv");
  out << "theta_deg\talpha_deg\tr\td\n";
  out << "0\t0\t1e10\t1e10\n";
  out << "0\t0\t1e10\t1e10\n";
  out.close();

  auto joints = dhcalc::parse_dh_table(temp_dir_ / "large.dh.tsv");
  ASSERT_EQ(joints.size(), 2u);

  auto T = dhcalc::compute_FK(joints);

  // Last row should still be [0, 0, 0, 1]
  EXPECT_NEAR(get_matrix_elem(T, 3, 3), 1.0, kEps);

  // Position should be approximately 2e10 in x and z
  EXPECT_NEAR(get_matrix_elem(T, 0, 3), 2e10, 1e5);
  EXPECT_NEAR(get_matrix_elem(T, 2, 3), 2e10, 1e5);
}

// ─── Edge: very small values ────────────────────────────────────────────────

TEST_F(StressTestFixture, VerySmallValues) {
  std::ofstream out(temp_dir_ / "small.dh.tsv");
  out << "theta_rad\talpha_rad\tr\td\n";
  out << "1e-15\t1e-15\t1e-15\t1e-15\n";
  out.close();

  auto joints = dhcalc::parse_dh_table(temp_dir_ / "small.dh.tsv");
  ASSERT_EQ(joints.size(), 1u);

  auto T = dhcalc::compute_FK(joints);

  // Should still be valid transform
  EXPECT_NEAR(get_matrix_elem(T, 3, 3), 1.0, kEps);
}

// ─── Edge: zero values ──────────────────────────────────────────────────────

TEST_F(StressTestFixture, AllZeros) {
  std::ofstream out(temp_dir_ / "zeros.dh.tsv");
  out << "theta_deg\talpha_deg\tr\td\n";
  for (int i = 0; i < 10; ++i) {
    out << "0\t0\t0\t0\n";
  }
  out.close();

  auto joints = dhcalc::parse_dh_table(temp_dir_ / "zeros.dh.tsv");
  ASSERT_EQ(joints.size(), 10u);

  auto T = dhcalc::compute_FK(joints);

  // Should be identity matrix
  for (size_t r = 0; r < 4; ++r) {
    for (size_t c = 0; c < 4; ++c) {
      double expected = (r == c) ? 1.0 : 0.0;
      EXPECT_NEAR(get_matrix_elem(T, r, c), expected, kEps);
    }
  }
}
