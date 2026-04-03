/*
 * Copyright 2026 Evan M.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#pragma once

#include <algorithm>
namespace Tissu {

/**
 * @brief Represents a triangle with vertices a,b,c.
 * 
 */
struct Triangle {
    int a, b, c;
    Triangle(int _a, int _b, int _c) : a(_a), b(_b), c(_c) {}
};

/**
 * @brief Represents a conecction between two vertices.
 * 
 */
struct Edge {
    int v1, v2;
    Edge(int a, int b) : v1(std::min(a, b)), v2(std::max(a, b)) {}
    bool operator<(const Edge& other) const {
    return v1 < other.v1 || (v1 == other.v1 && v2 < other.v2);
    }
};

/**
 * @struct ClothMaterial
 * @brief Defines the physical properties of a simulated cloth.
 * 
 */
struct ClothMaterial {
public:
    ClothMaterial(double _density, double _structuralCompliance, double _shearCompliance, double _bendingCompliance)
    : density(_density), structuralCompliance(_structuralCompliance), shearCompliance(_shearCompliance), bendingCompliance(_bendingCompliance) {}

    /**
     * @brief Defines the properties of a cloth material.
     * 
     */
    ClothMaterial() : density(0.1), structuralCompliance(1e-6), shearCompliance(1e-6), bendingCompliance(0.01) {}

    inline double getDensity() const { return density; }
    inline double getStructuralCompliance() const { return structuralCompliance; }
    inline double getShearCompliance() const { return shearCompliance; }
    inline double getBendingCompliance() const { return bendingCompliance; }

    inline void setDensity(double density) { this->density = density; }
    inline void setStructuralCompliance(double structuralCompliance) { this->structuralCompliance = structuralCompliance; }
    inline void setShearCompliance(double shearCompliance) { this->shearCompliance = shearCompliance; }
    inline void setBendingCompliance(double bendingCompliance) { this->bendingCompliance = bendingCompliance; }
    

private:
    double density;              ///< Mass per unit area in kg/m². Controls how heavy the cloth feels under gravity.
    double structuralCompliance; ///< Resistance to stretching along edges. Lower = less stretch. Typical range: [0, 1e-6].
    double shearCompliance;      ///< Resistance to in-plane shearing (diagonal deformation). Lower = stiffer weave. Typical range: [0, 1e-6].
    double bendingCompliance;    ///< Resistance to folding between adjacent triangles. Higher = softer drape. Typical range: [1e-4, 0.1].
};

}