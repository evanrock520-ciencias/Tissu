#include "gtest/gtest.h"
#include <string>
#include "physics/VolumeConstraint.hpp"
#include "utils/Logger.hpp"

using namespace Tissu;

class VolumeConstraintTest : public ::testing::Test {
protected:
    class TestableVolumeConstraint : public VolumeConstraint {
        public:
            using VolumeConstraint::VolumeConstraint;
            using VolumeConstraint::computeVolume;
    };

    void SetUp() override {
        particles.emplace_back(Eigen::Vector3d( 1.0,  0.0, -0.707));
        particles.emplace_back(Eigen::Vector3d(-1.0,  0.0, -0.707));
        particles.emplace_back(Eigen::Vector3d( 0.0,  1.0,  0.707));
        particles.emplace_back(Eigen::Vector3d( 0.0, -1.0,  0.707));

        triangles.push_back({0, 1, 2});
        triangles.push_back({0, 2, 3});
        triangles.push_back({0, 3, 1});
        triangles.push_back({1, 3, 2});
    }

    std::vector<Particle> particles;
    std::vector<Triangle> triangles;
};

TEST_F(VolumeConstraintTest, ConvergesToRestVolume) {
    TestableVolumeConstraint constraint(triangles, particles, 0.0);
    double restVolume = constraint.getRestVolume();

    for (int idx = 0; idx < 100; idx++)
        constraint.solve(particles, 0.016);

    double currentVolume = constraint.computeVolume(particles);
    EXPECT_NEAR(currentVolume, restVolume, 1e-4);
}