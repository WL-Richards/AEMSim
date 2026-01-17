#pragma once
#include "AerodynamicParametersBase.hpp"

class SphericalAeroParameters : public AerodynamicParametersBase<SphericalAeroParameters>
{
public:
    // Radius of the sphere
    double RadiusM;

    /**
     * Retrieve the area of a sphere
     * @return 
     */
    [[nodiscard]] double getArea() const override { return chrono::CH_PI * RadiusM * RadiusM; }

    std::shared_ptr<SphericalAeroParameters> withRadiusM(const double radius)
    {
        RadiusM = radius;
        return this->shared_from_this();
    }
    
};
