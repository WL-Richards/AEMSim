#pragma once
#include <memory>

class PhysicalParametersBase : public std::enable_shared_from_this<PhysicalParametersBase>
{
public:
    /// The coefficient of friction that roughly represents this object
    double FrictionCoefficient = 0.6f;

    // How bouncy this physical object is
    double RestitutionCoefficient = 0.1f;

    // Density of the projectile in kg/m³
    double Density = 1.0; // kg/m³
    
    std::shared_ptr<PhysicalParametersBase> withFrictionCoefficient(double frictionCoeff)
    {
        FrictionCoefficient = frictionCoeff;
        return this->shared_from_this();
    }
    
    std::shared_ptr<PhysicalParametersBase> withRestitutionCoefficient(double restituitionCoeff)
    {
        RestitutionCoefficient = restituitionCoeff;
        return this->shared_from_this();
    }
    
    std::shared_ptr<PhysicalParametersBase> withDensityKgM3(double density)
    {
        Density = density;
        return this->shared_from_this();
    }
};
