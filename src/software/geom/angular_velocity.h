#pragma once

#include "software/geom/angle.h"

/**
 * A typesafe representation of an angular velocity, using the Angle class.
 *
 * This class helps prevent accidentally combining values in degrees and radians
 * per second without proper conversion.
 *
 */
class AngularVelocity : public Angle
{
   public:
    explicit constexpr AngularVelocity();
    explicit constexpr AngularVelocity(const Angle& angle);

    static constexpr AngularVelocity zero();
    static constexpr AngularVelocity quarter();
    static constexpr AngularVelocity half();
    static constexpr AngularVelocity threeQuarter();
    static constexpr AngularVelocity full();
    static constexpr AngularVelocity fromRadians(double rad);
    static constexpr AngularVelocity fromDegrees(double deg);

    constexpr AngularVelocity abs() const;
    constexpr AngularVelocity minDiff(const AngularVelocity& other);

    // Delete methods that do not apply to angular velocity
    static Angle asin(double)                     = delete;
    static Angle acos(double)                     = delete;
    static Angle atan(double)                     = delete;
    constexpr Angle mod(Angle) const              = delete;
    constexpr Angle remainder(const Angle&) const = delete;
    double sin() const                            = delete;
    double cos() const                            = delete;
    double tan() const                            = delete;
    constexpr Angle clamp() const                 = delete;
    constexpr Angle minDiff(const Angle&) const   = delete;
};

inline constexpr AngularVelocity::AngularVelocity() : Angle(0.0) {}

inline constexpr AngularVelocity::AngularVelocity(const Angle& angle) : Angle(angle) {}

inline constexpr AngularVelocity AngularVelocity::zero()
{
    return AngularVelocity(Angle::zero());
}

inline constexpr AngularVelocity AngularVelocity::quarter()
{
    return AngularVelocity(Angle::quarter());
}

inline constexpr AngularVelocity AngularVelocity::half()
{
    return AngularVelocity(Angle::half());
}

inline constexpr AngularVelocity AngularVelocity::threeQuarter()
{
    return AngularVelocity(Angle::threeQuarter());
}

inline constexpr AngularVelocity AngularVelocity::full()
{
    return AngularVelocity(Angle::full());
}

inline constexpr AngularVelocity AngularVelocity::fromRadians(double rad)
{
    return AngularVelocity(Angle::fromRadians(rad));
}

inline constexpr AngularVelocity AngularVelocity::fromDegrees(double deg)
{
    return AngularVelocity(Angle::fromDegrees(deg));
}

inline constexpr AngularVelocity AngularVelocity::abs() const
{
    return AngularVelocity::fromRadians(toRadians() < 0 ? -toRadians() : toRadians());
}

inline constexpr AngularVelocity AngularVelocity::minDiff(const AngularVelocity& other)
{
    return AngularVelocity::fromRadians(std::fabs(this->toRadians() - other.toRadians()));
}
