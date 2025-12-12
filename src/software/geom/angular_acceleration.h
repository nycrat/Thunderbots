#pragma once

#include "software/geom/angle.h"
/**
 * We also use variables of type 'Angle' to represent angular acceleration, since they
 * are essentially represented the same. This typedef allows us to refer to Angles as
 * AngularAcceleration, which makes the interfaces more intuitive.
 * TODO (#3093): Not all methods of Angle class make sense for AngularAcceleration!
 *  E.g. Angle::clamp does not make sense in the context of AngularAcceleration, as
 *  360 deg/s^2 is different from 0 deg/s^2, but 360 deg is the same as 0 deg.
 */

/**
 * A typesafe representation of an angular acceleration, using the Angle class.
 *
 * This class helps prevent accidentally combining values in degrees and radians
 * per seconds squared without proper conversion.
 *
 */
class AngularAcceleration : public Angle
{
   public:
    explicit constexpr AngularAcceleration();

    static constexpr AngularAcceleration zero();
    static constexpr AngularAcceleration fromRadians(double rad);
    static constexpr AngularAcceleration fromDegrees(double deg);

    constexpr AngularAcceleration abs() const;

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
    explicit constexpr AngularAcceleration(double rads);
};

inline constexpr AngularAcceleration::AngularAcceleration() : Angle(0.0) {}

inline constexpr AngularAcceleration::AngularAcceleration(double rads) : Angle(rads) {}

inline constexpr AngularAcceleration AngularAcceleration::zero()
{
    return AngularAcceleration(0.0);
}

inline constexpr AngularAcceleration AngularAcceleration::fromRadians(double rad)
{
    return AngularAcceleration(rad);
}

inline constexpr AngularAcceleration AngularAcceleration::fromDegrees(double deg)
{
    return AngularAcceleration::fromRadians(deg / 180.0 * M_PI);
}

inline constexpr AngularAcceleration AngularAcceleration::abs() const
{
    return AngularAcceleration::fromRadians(toRadians() < 0 ? -toRadians() : toRadians());
}
