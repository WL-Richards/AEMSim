#pragma once
#include <chrono/core/ChVector3.h>

/**
 * @brief Base class describing aerodynamic parameters for a simulated object.
 *
 * This class provides a reusable set of aerodynamic coefficients and
 * environmental parameters used to compute drag, Magnus forces, and optional
 * spin decay. It is designed using the Curiously Recurring Template Pattern
 * (CRTP) to enable a fluent `.withX()` configuration API that preserves the
 * derived type during method chaining.
 *
 * Derived classes are expected to:
 *  - Represent a concrete aerodynamic shape (e.g., sphere, projectile, disk)
 *  - Implement the required area() function
 *
 * Example usage:
 * @code
 * class SphereAero : public AerodynamicParametersBase<SphereAero> {
 * public:
 *     explicit SphereAero(double r) : radius(r) {}
 *
 *     double area() const override {
 *         return CH_C_PI * radius * radius;
 *     }
 *
 * private:
 *     double radius;
 * };
 *
 * SphereAero aero(0.05);
 * aero.withAirDensity(1.18)
 *     .withDragCoefficient(0.47)
 *     .withMagnusCoefficient(0.02);
 * @endcode
 *
 * @tparam Derived The concrete aerodynamic parameter type (CRTP).
 */
template <class Derived>
class AerodynamicParametersBase : public std::enable_shared_from_this<Derived>
{
public:

    /**
     * @name Environmental Parameters
     * @{
     */

    /// Air density in kg/m^3 (default: 1.225 kg/m^3 at sea level).
    double AirDensity = 1.225;

    /**
     * Wind velocity vector in world coordinates (m/s).
     *
     * This represents ambient airflow relative to the world frame.
     * A value of zero indicates still air.
     */
    chrono::ChVector3d WindVeloctiy = chrono::VNULL;

    /** @} */

    /**
     * @name Aerodynamic Coefficients
     * @{
     */

    /**
     * Drag coefficient (dimensionless).
     *
     * Typical values:
     *  - Sphere: ~0.47
     *  - Smooth projectile: 0.2–0.3
     */
    double DragCoefficient = 0.25;

    /**
     * Magnus (lift) coefficient (dimensionless).
     *
     * Used to model lift forces induced by spin.
     * Typically small (≈ 0.01–0.05).
     */
    double MagnusCoefficient = 0.02;

    /**
     * Optional angular velocity decay rate (1/s).
     *
     * Used for crude spin damping models where angular velocity
     * decays exponentially over time.
     * Set to zero to disable spin decay.
     */
    double SpinRateDecay = 0.0;

    /** @} */

    /**
     * @name Fluent Configuration Interface
     *
     * These methods follow a fluent "withX" style API and return a reference
     * to the derived type, allowing method chaining while preserving static
     * type information.
     * @{
     */

    /// Set air density in kg/m^3.
    std::shared_ptr<Derived> withAirDensity(double densityKgM3)
    {
        AirDensity = densityKgM3;
        return this->shared_from_this();
    }

    /// Set wind velocity in world coordinates (m/s).
    std::shared_ptr<Derived> withWindVelocity(const chrono::ChVector3d& velocityMS)
    {
        WindVeloctiy = velocityMS;
        return this->shared_from_this();
    }

    /// Set the drag coefficient (dimensionless).
    std::shared_ptr<Derived> withDragCoefficient(double dragCoeff)
    {
        DragCoefficient = dragCoeff;
        return this->shared_from_this();
    }

    /// Set the Magnus (lift) coefficient (dimensionless).
    std::shared_ptr<Derived> withMagnusCoefficient(double magnusCoeff)
    {
        MagnusCoefficient = magnusCoeff;
        return this->shared_from_this();
    }

    /// Set the angular velocity decay rate (1/s).
    std::shared_ptr<Derived> withSpinRateDecay(double spinRateDecay)
    {
        SpinRateDecay = spinRateDecay;
        return this->shared_from_this();
    }

    /** @} */

    /**
     * @brief Virtual destructor for safe polymorphic destruction.
     */
    virtual ~AerodynamicParametersBase() = default;

    /**
     * @brief Return the reference area used for aerodynamic force computation.
     *
     * This area is typically the projected frontal area normal to the
     * velocity vector (e.g., πr² for a sphere).
     *
     * @return Area in square meters (m²).
     */
    virtual double getArea() const = 0;

};
