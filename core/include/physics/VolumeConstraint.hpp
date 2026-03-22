#pragma once

#include "math/Types.hpp"
#include "physics/Constraint.hpp"
#include "physics/Particle.hpp"
#include <vector>
namespace Tissu {

class VolumeConstraint : Constraint {
public:
    VolumeConstraint(const std::vector<Triangle>& triangles, const std::vector<Particle>& particles, double compliance);
    void solve(std::vector<Particle>& particles, double dt) override;

    inline double getRestVolume() { return m_restVolume; };
    
protected:
    double computeVolume(const std::vector<Particle>& particles) const;

private:
    std::vector<Triangle> m_triangles;
    double m_restVolume;
};
}