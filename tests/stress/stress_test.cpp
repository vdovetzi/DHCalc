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
  const std::vector<size_t> sizes = {1, 5, 20, 100, 500, 1000, 10000, 100000};

  for (std::size_t size : sizes) {
    auto path = generate_tsv(temp_dir_, size, static_cast<unsigned>(size * 7));
    auto joints = dhcalc::parse_dh_table(path);
    auto T = dhcalc::compute_FK(joints);

    // Last row must be [0, 0, 0, 1]
    EXPECT_NEAR(T(3, 0), 0.0, kEps) << "size=" << size;
    EXPECT_NEAR(T(3, 1), 0.0, kEps) << "size=" << size;
    EXPECT_NEAR(T(3, 2), 0.0, kEps) << "size=" << size;
    EXPECT_NEAR(T(3, 3), 1.0, kEps) << "size=" << size;

    // Rotation part must be orthonormal: R^T * R ≈ I
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Matrix3d RtR = R.transpose() * R;
    for (Eigen::Index r = 0; r < 3; ++r) {
      for (Eigen::Index c = 0; c < 3; ++c) {
        EXPECT_NEAR(RtR(r, c), (r == c) ? 1.0 : 0.0, 1e-6)
            << "size=" << size << " at (" << r << "," << c << ")";
      }
    }
  }
}

// ─── Performance: parse timing ──────────────────────────────────────────────

TEST_F(StressTestFixture, ParsePerformance) {
  const std::vector<size_t> cases = {
      100, 1000, 5000, 10000, 50000, 100000, 500000, 1000000, 5000000,
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

  // Files with same seed should produce identical content, hence identical FK
  auto joints1 = dhcalc::parse_dh_table(path1);
  auto joints2 = dhcalc::parse_dh_table(path2);

  ASSERT_EQ(joints1.size(), joints2.size());

  auto T1 = dhcalc::compute_FK(joints1);
  auto T2 = dhcalc::compute_FK(joints2);

  for (Eigen::Index r = 0; r < 4; ++r) {
    for (Eigen::Index c = 0; c < 4; ++c) {
      EXPECT_DOUBLE_EQ(T1(r, c), T2(r, c));
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
  EXPECT_NEAR(T(3, 3), 1.0, kEps);
  EXPECT_NEAR(T(0, 3), 2e10, 1.0);
  EXPECT_NEAR(T(2, 3), 2e10, 1.0);
}
