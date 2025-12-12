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

    static constexpr AngularVelocity zero();
    static constexpr AngularVelocity fromRadians(double rad);
    static constexpr AngularVelocity fromDegrees(double deg);

    constexpr AngularVelocity abs() const;

    // NOTE: kind of weird we have two ways of create zero angle / angular_velocity
    // static constexpr Angle zero()                 = delete;

    // Delete methods that do not apply to angular velocity
    static constexpr Angle quarter()              = delete;
    static constexpr Angle half()                 = delete;
    static constexpr Angle threeQuarter()         = delete;
    static constexpr Angle full()                 = delete;
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

   private:
    explicit constexpr AngularVelocity(double rads);
};

inline constexpr AngularVelocity::AngularVelocity() : Angle(0.0) {}

inline constexpr AngularVelocity::AngularVelocity(double rads) : Angle(rads) {}

inline constexpr AngularVelocity AngularVelocity::zero()
{
    return AngularVelocity(0.0);
}

inline constexpr AngularVelocity AngularVelocity::fromRadians(double rad)
{
    return AngularVelocity(rad);
}

inline constexpr AngularVelocity AngularVelocity::fromDegrees(double deg)
{
    return AngularVelocity::fromRadians(deg / 180.0 * M_PI);
}

inline constexpr AngularVelocity AngularVelocity::abs() const
{
    return AngularVelocity::fromRadians(toRadians() < 0 ? -toRadians() : toRadians());
}
