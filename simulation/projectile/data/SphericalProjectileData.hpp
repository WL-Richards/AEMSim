#pragma once

#include "../../parameters/PhysicalParametersBase.hpp"
#include "../../parameters/SphericalAeroParameters.hpp"

#define CREATE_FUEL_PROJECTILE_DATA() std::make_shared<SphericalProjectileData>()->withAerodynamicParameters(                       \
                                                                                (std::make_shared<SphericalAeroParameters>())       \
                                                                                        ->withRadiusM(0.075)                        \
                                                                                        ->withDragCoefficient(0.47)                 \
                                                                                        ->withMagnusCoefficient(0.02)               \
                                                                            )                                                       \
                                                                            ->withPhysicalParameters(                               \
                                                                                std::make_shared<PhysicalParametersBase>()          \
                                                                                            ->withDensityKgM3(121.7)                \
                                                                                            ->withFrictionCoefficient(0.7)          \
                                                                                            ->withRestitutionCoefficient(0.65)       \
                                                                            )

class SphericalProjectileData : public std::enable_shared_from_this<SphericalProjectileData>
{
public:
    // Aerodynamic parameters representing this projectile
    std::shared_ptr<SphericalAeroParameters> AerodynamicParameters;

    // Describes pyhsical properties about the object
    std::shared_ptr<PhysicalParametersBase> PhysicalParameters;
    
    std::shared_ptr<SphericalProjectileData> withAerodynamicParameters(std::shared_ptr<SphericalAeroParameters> params)
    {
        AerodynamicParameters = params;
        return shared_from_this();
    }

    std::shared_ptr<SphericalProjectileData> withPhysicalParameters(std::shared_ptr<PhysicalParametersBase> params)
    {
        PhysicalParameters = params;
        return shared_from_this();
    }
};
