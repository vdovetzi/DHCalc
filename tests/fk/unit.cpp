#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <string>
#include <vector>

#include "dhcalc/kinematics.hpp"

static constexpr double kDeg2Rad = std::numbers::pi_v<double> / 180.0;
static constexpr double kEps = 1e-10;

// ─── Identity matrix ────────────────────────────────────────────────────────

TEST(IdentityMatrix, IsIdentity) {
  auto I = dhcalc::identity_matrix();

  for (Eigen::Index r = 0; r < 4; ++r) {
    for (Eigen::Index c = 0; c < 4; ++c) {
      EXPECT_NEAR(I(r, c), (r == c) ? 1.0 : 0.0, kEps)
          << "at (" << r << ", " << c << ")";
    }
  }
}

// ─── Single joint transform ────────────────────────────────────────────────

TEST(JointTransform, IdentityWhenAllZero) {
  dhcalc::DHFrameParameters joint{};
  auto T = dhcalc::compute_H(joint);

  for (Eigen::Index r = 0; r < 4; ++r) {
    for (Eigen::Index c = 0; c < 4; ++c) {
      EXPECT_NEAR(T(r, c), (r == c) ? 1.0 : 0.0, kEps)
          << "at (" << r << ", " << c << ")";
    }
  }
}

TEST(JointTransform, PureTranslationAlongX) {
  dhcalc::DHFrameParameters joint{};
  joint.r = 5.0;

  auto T = dhcalc::compute_H(joint);

  EXPECT_NEAR(T(0, 3), 5.0, kEps);
  EXPECT_NEAR(T(1, 3), 0.0, kEps);
  EXPECT_NEAR(T(2, 3), 0.0, kEps);
}

TEST(JointTransform, PureTranslationAlongZ) {
  dhcalc::DHFrameParameters joint{};
  joint.d = 3.0;

  auto T = dhcalc::compute_H(joint);

  EXPECT_NEAR(T(0, 3), 0.0, kEps);
  EXPECT_NEAR(T(1, 3), 0.0, kEps);
  EXPECT_NEAR(T(2, 3), 3.0, kEps);
}

TEST(JointTransform, Rotation90Theta) {
  dhcalc::DHFrameParameters joint{};
  joint.theta_radians = 90.0 * kDeg2Rad;

  auto T = dhcalc::compute_H(joint);

  // First column: [cos90, sin90, 0, 0]
  EXPECT_NEAR(T(0, 0), 0.0, kEps);
  EXPECT_NEAR(T(1, 0), 1.0, kEps);
  EXPECT_NEAR(T(2, 0), 0.0, kEps);
}

TEST(JointTransform, Rotation90Alpha) {
  dhcalc::DHFrameParameters joint{};
  joint.alpha_radians = 90.0 * kDeg2Rad;

  auto T = dhcalc::compute_H(joint);

  // Column 1 should be [0, 0, 1, 0] (sin_alpha parts)
  EXPECT_NEAR(T(0, 1), 0.0, kEps);
  EXPECT_NEAR(T(1, 1), 0.0, kEps);
  EXPECT_NEAR(T(2, 1), 1.0, kEps);
}

TEST(JointTransform, CombinedThetaAlpha) {
  dhcalc::DHFrameParameters joint{};
  joint.theta_radians = 45.0 * kDeg2Rad;
  joint.alpha_radians = -90.0 * kDeg2Rad;
  joint.r = 2.0;
  joint.d = 1.0;

  auto T = dhcalc::compute_H(joint);

  const double c45 = std::cos(45.0 * kDeg2Rad);
  const double s45 = std::sin(45.0 * kDeg2Rad);

  EXPECT_NEAR(T(0, 0), c45, kEps);
  EXPECT_NEAR(T(1, 0), s45, kEps);
  EXPECT_NEAR(T(0, 3), 2.0 * c45, kEps);
  EXPECT_NEAR(T(1, 3), 2.0 * s45, kEps);
  EXPECT_NEAR(T(2, 3), 1.0, kEps);
  EXPECT_NEAR(T(3, 3), 1.0, kEps);
}

// ─── Forward kinematics chains ─────────────────────────────────────────────

TEST(ForwardKinematics, EmptyChainIsIdentity) {
  std::vector<dhcalc::DHFrameParameters> joints;
  auto T = dhcalc::compute_FK(joints);

  for (Eigen::Index r = 0; r < 4; ++r) {
    for (Eigen::Index c = 0; c < 4; ++c) {
      EXPECT_NEAR(T(r, c), (r == c) ? 1.0 : 0.0, kEps);
    }
  }
}

TEST(ForwardKinematics, SingleJointMatchesTransform) {
  dhcalc::DHFrameParameters joint{};
  joint.theta_radians = 30.0 * kDeg2Rad;
  joint.alpha_radians = -45.0 * kDeg2Rad;
  joint.r = 1.5;
  joint.d = 0.3;

  auto T_single = dhcalc::compute_H(joint);
  auto T_fk = dhcalc::compute_FK({joint});

  for (Eigen::Index r = 0; r < 4; ++r) {
    for (Eigen::Index c = 0; c < 4; ++c) {
      EXPECT_NEAR(T_fk(r, c), T_single(r, c), kEps)
          << "at (" << r << ", " << c << ")";
    }
  }
}

TEST(ForwardKinematics, TwoLinkPlanarArm) {
  // Two-link planar arm: L1 = L2 = 1.0, theta1 = 90°, theta2 = 0°
  // Expected end effector: x = 0, y = 2
  dhcalc::DHFrameParameters j1{};
  j1.theta_radians = 90.0 * kDeg2Rad;
  j1.r = 1.0;

  dhcalc::DHFrameParameters j2{};
  j2.theta_radians = 0.0;
  j2.r = 1.0;

  auto T = dhcalc::compute_FK({j1, j2});

  EXPECT_NEAR(T(0, 3), 0.0, kEps); // x ≈ 0
  EXPECT_NEAR(T(1, 3), 2.0, kEps); // y = 2
  EXPECT_NEAR(T(2, 3), 0.0, kEps); // z = 0
}

TEST(ForwardKinematics, TwoLinkPlanarArmBent) {
  // L1 = L2 = 1.0, theta1 = 0°, theta2 = 90°
  // After first link: at (1, 0). Second rotates 90° from that frame.
  // End effector: x = 1, y = 1
  dhcalc::DHFrameParameters j1{};
  j1.theta_radians = 0.0;
  j1.r = 1.0;

  dhcalc::DHFrameParameters j2{};
  j2.theta_radians = 90.0 * kDeg2Rad;
  j2.r = 1.0;

  auto T = dhcalc::compute_FK({j1, j2});

  EXPECT_NEAR(T(0, 3), 1.0, kEps);
  EXPECT_NEAR(T(1, 3), 1.0, kEps);
  EXPECT_NEAR(T(2, 3), 0.0, kEps);
}

TEST(ForwardKinematics, ThreeJointSpatial) {
  // Joint 1: theta=0, alpha=-90, r=0, d=1  (rotates frame)
  // Joint 2: theta=90, alpha=0, r=1, d=0   (translate + rotate in new frame)
  // Joint 3: theta=0, alpha=0, r=1, d=0    (extend along new X)
  dhcalc::DHFrameParameters j1{"j1", 0.0, -90.0 * kDeg2Rad, 0.0, 1.0};
  dhcalc::DHFrameParameters j2{"j2", 90.0 * kDeg2Rad, 0.0, 1.0, 0.0};
  dhcalc::DHFrameParameters j3{"j3", 0.0, 0.0, 1.0, 0.0};

  auto T = dhcalc::compute_FK({j1, j2, j3});

  // Verify it's still a valid homogeneous transform (last row)
  EXPECT_NEAR(T(3, 0), 0.0, kEps);
  EXPECT_NEAR(T(3, 1), 0.0, kEps);
  EXPECT_NEAR(T(3, 2), 0.0, kEps);
  EXPECT_NEAR(T(3, 3), 1.0, kEps);

  // Verify rotation part is orthonormal
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  Eigen::Matrix3d RtR = R.transpose() * R;
  for (Eigen::Index r = 0; r < 3; ++r) {
    for (Eigen::Index c = 0; c < 3; ++c) {
      EXPECT_NEAR(RtR(r, c), (r == c) ? 1.0 : 0.0, kEps);
    }
  }
}

TEST(ForwardKinematics, StanfordArmHomePosition) {
  // Stanford arm at home (all theta = 0)
  // https://en.wikipedia.org/wiki/Stanford_manipulator
  std::vector<dhcalc::DHFrameParameters> joints = {
      {"1", 0.0, -90.0 * kDeg2Rad, 0.0, 0.412},
      {"2", 0.0, 90.0 * kDeg2Rad, 0.0, 0.154},
      {"3", 0.0, 0.0, 0.0, 0.500},
      {"4", 0.0, -90.0 * kDeg2Rad, 0.0, 0.0},
      {"5", 0.0, 90.0 * kDeg2Rad, 0.0, 0.0},
      {"6", 0.0, 0.0, 0.0, 0.0},
  };

  auto T = dhcalc::compute_FK(joints);

  // At home position with all zero thetas the successive alpha rotations
  // (-90, +90, 0, -90, +90, 0) cancel and the result should be identity
  // rotation with translation (0, d2, d1+d3).
  EXPECT_NEAR(T(0, 3), 0.0, kEps);
  EXPECT_NEAR(T(1, 3), 0.154, kEps);
  EXPECT_NEAR(T(2, 3), 0.412 + 0.500, kEps);

  // Rotation block should be identity since alpha pairs cancel
  for (Eigen::Index r = 0; r < 3; ++r) {
    for (Eigen::Index c = 0; c < 3; ++c) {
      EXPECT_NEAR(T(r, c), (r == c) ? 1.0 : 0.0, kEps)
          << "rotation at (" << r << ", " << c << ")";
    }
  }
}

// ─── Associativity check ───────────────────────────────────────────────────

TEST(ForwardKinematics, AssociativityThreeJoints) {
  dhcalc::DHFrameParameters j1{"a", 10.0 * kDeg2Rad, -30.0 * kDeg2Rad, 1.0,
                               0.5};
  dhcalc::DHFrameParameters j2{"b", 20.0 * kDeg2Rad, 45.0 * kDeg2Rad, 0.5, 0.3};
  dhcalc::DHFrameParameters j3{"c", -15.0 * kDeg2Rad, 60.0 * kDeg2Rad, 0.8,
                               0.1};

  auto T_all = dhcalc::compute_FK({j1, j2, j3});

  // (T1 * T2) * T3 should equal T1 * (T2 * T3)
  auto T12 = dhcalc::compute_FK({j1, j2});
  auto T3 = dhcalc::compute_H(j3);
  dhcalc::Matrix4 T_left = T12 * T3;

  auto T1 = dhcalc::compute_H(j1);
  auto T23 = dhcalc::compute_FK({j2, j3});
  dhcalc::Matrix4 T_right = T1 * T23;

  for (Eigen::Index r = 0; r < 4; ++r) {
    for (Eigen::Index c = 0; c < 4; ++c) {
      EXPECT_NEAR(T_all(r, c), T_left(r, c), kEps);
      EXPECT_NEAR(T_all(r, c), T_right(r, c), kEps);
    }
  }
}

// ─── Format matrix ─────────────────────────────────────────────────────────

TEST(FormatMatrix, IdentityContainsOnesAndZeros) {
  auto text = dhcalc::format_matrix(dhcalc::identity_matrix(), 2);

  EXPECT_NE(text.find("1.00"), std::string::npos);
  EXPECT_NE(text.find("0.00"), std::string::npos);
}

TEST(FormatMatrix, PrecisionZero) {
  auto text = dhcalc::format_matrix(dhcalc::identity_matrix(), 0);

  EXPECT_NE(text.find("1"), std::string::npos);
}

TEST(FormatMatrix, NegativePrecisionThrows) {
  EXPECT_THROW(dhcalc::format_matrix(dhcalc::identity_matrix(), -1),
               std::invalid_argument);
}

TEST(FormatMatrix, FourRows) {
  auto text = dhcalc::format_matrix(dhcalc::identity_matrix(), 4);

  // Count '[' characters — should be 4
  int bracket_count = 0;
  for (char ch : text) {
    if (ch == '[') {
      ++bracket_count;
    }
  }
  EXPECT_EQ(bracket_count, 4);
}
