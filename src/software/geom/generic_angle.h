#pragma once

#include <cmath>
#include <ostream>

#include "software/geom/geom_constants.h"

class _AngleBase;
class _AngleVelocity;
class _AngleAcceleration;

/**
 * A typesafe representation of a generic angle.
 *
 * This class helps prevent accidentally combining values in degrees and radians
 * without proper conversion.
 *
 * @tparam T The type of angle being represented
 */
template <typename T>
class GenericAngle final
{
   public:
    /**
     * The zero angle.
     */
    static constexpr GenericAngle<T> zero();

    /**
     * The quarter-turn angle (90°).
     */
    static constexpr GenericAngle<T> quarter();

    /**
     * The half-turn angle (180°).
     */
    static constexpr GenericAngle<T> half();

    /**
     * The three-quarter turn angle (270°).
     */
    static constexpr GenericAngle<T> threeQuarter();

    /**
     * The full-turn angle (360°).
     */
    static constexpr GenericAngle<T> full();

    /**
     * Constructs an angle from a value in radians.
     *
     * @param rad the angle in radians.
     *
     * @return the constructed angle
     */
    static constexpr GenericAngle<T> fromRadians(double rad);

    /**
     * Constructs an angle from a value in degrees.
     *
     * @param deg the angle in degrees
     *
     * @return the constructed angle
     */
    static constexpr GenericAngle<T> fromDegrees(double deg);

    /**
     * Computes the arcsine of a value.
     *
     * @param x the value.
     *
     * @return the angle.
     */
    static GenericAngle<T> asin(double x);

    /**
     * Computes the arccosine of a value.
     *
     * @param x the value.
     *
     * @return the angle.
     */
    static GenericAngle<T> acos(double x);

    /**
     * Computes the arctangent of a value.
     *
     * @param x the value.
     *
     * @return the angle.
     */
    static GenericAngle<T> atan(double x);

    /**
     * Constructs the "zero" angle.
     */
    explicit constexpr GenericAngle<T>();

    /**
     * Converts this angle to a value in radians.
     *
     * @return the number of radians in this angle in the range [0, 2PI)
     */
    constexpr double toRadians() const;

    /**
     * Converts this angle to a value in degrees.
     *
     * @return the number of degrees in this angle in the range [0,360)
     */
    constexpr double toDegrees() const;

    /**
     * Computes the modulus of a division between this angle and another
     * angle.
     *
     * @param divisor the divisor.
     *
     * @return the modulus of this Angle ÷ divisor.
     */
    constexpr GenericAngle<T> mod(GenericAngle<T> divisor) const;

    /**
     * Computes the remainder of a division between this angle and
     * another angle.
     *
     * @param divisor the divisor.
     *
     * @return the remainder of this Angle ÷ divisor.
     */
    constexpr GenericAngle<T> remainder(const GenericAngle<T>& divisor) const;

    /**
     * Returns the absolute value of this angle.
     *
     * @return the absolute value of this angle.
     */
    constexpr GenericAngle<T> abs() const;

    /**
     * Checks whether the angle is finite.
     *
     * @return true if the angle is finite, and false if it is ±∞ or NaN.
     */
    bool isFinite() const;

    /**
     * Computes the sine of this angle.
     *
     * @return the sine of this angle.
     */
    double sin() const;

    /**
     * Computes the cosine of this angle.
     *
     * @return the cosine of this angle.
     */
    double cos() const;

    /**
     * Computes the tangent of this angle.
     *
     * @return the tangent of this angle.
     */
    double tan() const;

    /**
     * Limits this angle to [−π, π].
     *
     * The angle is rotated by a multiple of 2π until it lies within the target
     * interval.
     *
     * @return the clamped angle.
     */
    constexpr GenericAngle<T> clamp() const;

    /**
     * Returns the smallest possible rotational difference between this angle
     * and another angle.
     *
     * @param other the second angle.
     *
     * @return the angle between this Angle and other, in the range [0, π].
     */
    constexpr GenericAngle<T> minDiff(const GenericAngle<T>& other) const;

   private:
    /**
     * The measurement in radians of this Angle.
     */
    double rads;

    explicit constexpr GenericAngle<T>(double rads);
};

/**
 * Negates an angle.
 *
 * @param angle the angle to negate.
 *
 * @return the negated angle
 */
template <typename T>
constexpr GenericAngle<T> operator-(const GenericAngle<T>& angle)
    __attribute__((warn_unused_result));

/**
 * Adds two angles.
 *
 * @param x the first addend.
 * @param y the second addend.
 *
 * @return the sum of the angles
 */
template <typename T>
constexpr GenericAngle<T> operator+(const GenericAngle<T>& x, const GenericAngle<T>& y)
    __attribute__((warn_unused_result));

/**
 * Subtracts two angles.
 *
 * @param x the minuend.
 *
 * @param y the subtrahend.
 *
 * @return the difference between the minuend and subtrahend.
 */
template <typename T>
constexpr GenericAngle<T> operator-(const GenericAngle<T>& x, const GenericAngle<T>& y)
    __attribute__((warn_unused_result));

/**
 * Multiplies an angle by a scalar factor.
 *
 * @param angle the angle.
 * @param scale the scalar factor.
 *
 * @return the product of the angle and the scalar factor
 */
template <typename T>
constexpr GenericAngle<T> operator*(const GenericAngle<T>& angle, double scale)
    __attribute__((warn_unused_result));

/**
 * Multiplies an angle by a scalar factor.
 *
 * @param scale the scalar factor.
 * @param angle the angle.
 *
 * @return the product of the angle and the scalar factor
 */
template <typename T>
constexpr GenericAngle<T> operator*(double scale, const GenericAngle<T>& angle)
    __attribute__((warn_unused_result));

/**
 * Divides an angle by a scalar divisor.
 *
 * @param angle the angle.
 * @param divisor the scalar divisor.
 *
 * @return the quotient of this Angle ÷ the divisor.
 */
template <typename T>
constexpr GenericAngle<T> operator/(const GenericAngle<T>& angle, double divisor)
    __attribute__((warn_unused_result));

/**
 * Divides two angles.
 *
 * @param x the divident.
 * @param y the divisor.
 *
 * @return the quotient of the divident ÷ the divisor.
 */
template <typename T>
constexpr double operator/(const GenericAngle<T>& x, const GenericAngle<T>& y)
    __attribute__((warn_unused_result));

/**
 * Adds an angle to another angle.
 *
 * @param x the angle to add to.
 * @param y the angle to add.
 *
 * @return the new angle x
 */
template <typename T>
GenericAngle<T>& operator+=(GenericAngle<T>& x, const GenericAngle<T>& y);

/**
 * Subtracts an angle from an angle.
 *
 * @param x the angle to subtract from.
 * @param y the angle to subtract.
 *
 * @return the new angle x
 */
template <typename T>
GenericAngle<T>& operator-=(GenericAngle<T>& x, const GenericAngle<T>& y);

/**
 * Scales an angle by a factor.
 *
 * @param angle the angle to scale.
 * @param scale the scalar factor.
 *
 * @return the scaled angle.
 */
template <typename T>
GenericAngle<T>& operator*=(GenericAngle<T>& angle, double scale);

/**
 * Divides an angle by a scalar divisor.
 *
 * @param angle the angle to scale.
 *
 * @param divisor the scalar divisor.
 *
 * @return the scaled angle.
 */
template <typename T>
GenericAngle<T>& operator/=(GenericAngle<T>& angle, double divisor);

/**
 * Compares two angles.
 *
 * @param x the first angle.
 *
 * @param y the second angle.
 *
 * @return true if x is strictly less than y, and false otherwise
 */
template <typename T>
constexpr bool operator<(const GenericAngle<T>& x, const GenericAngle<T>& y);

/**
 * Compares two angles.
 *
 * @param x the first angle.
 * @param y the second angle.
 *
 * @return true if x is strictly greater than y, and false otherwise.
 */
template <typename T>
constexpr bool operator>(const GenericAngle<T>& x, const GenericAngle<T>& y);

/**
 * Compares two angles.
 *
 * @param x the first angle.
 * @param y the second angle.
 *
 * @return true if x is less than or equal to y, and false otherwise.
 */
template <typename T>
constexpr bool operator<=(const GenericAngle<T>& x, const GenericAngle<T>& y);

/**
 * Compares two angles.
 *
 * @param x the first angle.
 * @param y the second angle.
 *
 * @return true if x is greater than or equal to y, and false otherwise.
 */
template <typename T>
constexpr bool operator>=(const GenericAngle<T>& x, const GenericAngle<T>& y);

/**
 * Compares two angles for equality
 *
 * @param x the first angle.
 * @param y the second angle.
 *
 * @return true if x is equal to y, and false otherwise.
 */
template <typename T>
bool operator==(const GenericAngle<T>& x, const GenericAngle<T>& y);

/**
 * Compares two angles for inequality.
 *
 * @param x the first angle.
 * @param y the second angle.
 *
 * @return true if x is not equal to y, and false otherwise
 */
template <typename T>
constexpr bool operator!=(const GenericAngle<T>& x, const GenericAngle<T>& y);

/**
 * Prints an Angle to a stream
 *
 * @param os the stream to print to
 * @param a the Point to print
 *
 * @return the stream with the Angle printed
 */
template <typename T>
inline std::ostream& operator<<(std::ostream& os, const GenericAngle<T>& a);

template <typename T>
inline constexpr GenericAngle<T> GenericAngle<T>::zero()
{
    return GenericAngle<T>();
}

template <typename T>
inline constexpr GenericAngle<T> GenericAngle<T>::quarter()
{
    return GenericAngle<T>(M_PI / 2.0);
}

template <typename T>
inline constexpr GenericAngle<T> GenericAngle<T>::half()
{
    return GenericAngle<T>(M_PI);
}

template <typename T>
inline constexpr GenericAngle<T> GenericAngle<T>::threeQuarter()
{
    return GenericAngle<T>(3.0 / 2.0 * M_PI);
}

template <typename T>
inline constexpr GenericAngle<T> GenericAngle<T>::full()
{
    return GenericAngle<T>(2.0 * M_PI);
}

template <typename T>
inline constexpr GenericAngle<T> GenericAngle<T>::fromRadians(double rad)
{
    return GenericAngle<T>(rad);
}

template <typename T>
inline constexpr GenericAngle<T> GenericAngle<T>::fromDegrees(double deg)
{
    return GenericAngle<T>(deg / 180.0 * M_PI);
}

template <typename T>
inline GenericAngle<T> GenericAngle<T>::asin(double x)
{
    return GenericAngle<T>::fromRadians(std::asin(x));
}

template <typename T>
inline GenericAngle<T> GenericAngle<T>::acos(double x)
{
    return fromRadians(std::acos(x));
}

template <typename T>
inline GenericAngle<T> GenericAngle<T>::atan(double x)
{
    return GenericAngle<T>::fromRadians(std::atan(x));
}

template <typename T>
inline constexpr GenericAngle<T>::GenericAngle() : rads(0.0)
{
}

template <typename T>
inline constexpr double GenericAngle<T>::toRadians() const
{
    return rads;
}

template <typename T>
inline constexpr double GenericAngle<T>::toDegrees() const
{
    return rads / M_PI * 180.0;
}

template <typename T>
inline constexpr GenericAngle<T> GenericAngle<T>::mod(GenericAngle<T> divisor) const
{
    if (divisor.toRadians() < FIXED_EPSILON)
    {
        return GenericAngle<T>::fromRadians(toRadians());
    }
    else
    {
        return GenericAngle<T>::fromRadians(
            toRadians() -
            static_cast<double>(static_cast<long>(toRadians() / divisor.toRadians())) *
                divisor.toRadians());
    }
}

template <typename T>
inline constexpr GenericAngle<T> GenericAngle<T>::remainder(
    const GenericAngle<T>& divisor) const
{
    return GenericAngle<T>::fromRadians(
        toRadians() - static_cast<double>(static_cast<long>(
                          (toRadians() / divisor.toRadians()) >= 0
                              ? (toRadians() / divisor.toRadians() + 0.5)
                              : (toRadians() / divisor.toRadians() - 0.5))) *
                          divisor.toRadians());
}

template <typename T>
inline constexpr GenericAngle<T> GenericAngle<T>::abs() const
{
    return GenericAngle<T>::fromRadians(toRadians() < 0 ? -toRadians() : toRadians());
}

template <typename T>
inline bool GenericAngle<T>::isFinite() const
{
    return std::isfinite(toRadians());
}

template <typename T>
inline double GenericAngle<T>::sin() const
{
    return std::sin(toRadians());
}

template <typename T>
inline double GenericAngle<T>::cos() const
{
    return std::cos(toRadians());
}

template <typename T>
inline double GenericAngle<T>::tan() const
{
    return std::tan(toRadians());
}

template <typename T>
inline constexpr GenericAngle<T> GenericAngle<T>::clamp() const
{
    return remainder(GenericAngle<T>::full());
}

template <typename T>
inline constexpr GenericAngle<T> GenericAngle<T>::minDiff(
    const GenericAngle<T>& other) const
{
    return (*this - other).clamp().abs();
}

template <typename T>
inline constexpr GenericAngle<T>::GenericAngle(double rads) : rads(rads)
{
}

template <typename T>
inline constexpr GenericAngle<T> operator-(const GenericAngle<T>& angle)
{
    return GenericAngle<T>::fromRadians(-angle.toRadians());
}

template <typename T>
inline constexpr GenericAngle<T> operator+(const GenericAngle<T>& x,
                                           const GenericAngle<T>& y)
{
    return GenericAngle<T>::fromRadians(x.toRadians() + y.toRadians());
}

template <typename T>
inline constexpr GenericAngle<T> operator-(const GenericAngle<T>& x,
                                           const GenericAngle<T>& y)
{
    return GenericAngle<T>::fromRadians(x.toRadians() - y.toRadians());
}

template <typename T>
inline constexpr GenericAngle<T> operator*(const GenericAngle<T>& angle, double scale)
{
    return GenericAngle<T>::fromRadians(angle.toRadians() * scale);
}

template <typename T>
inline constexpr GenericAngle<T> operator*(double scale, const GenericAngle<T>& angle)
{
    return GenericAngle<T>::fromRadians(scale * angle.toRadians());
}

template <typename T>
inline constexpr GenericAngle<T> operator/(const GenericAngle<T>& angle, double divisor)
{
    return GenericAngle<T>::fromRadians(angle.toRadians() / divisor);
}

template <typename T>
inline constexpr double operator/(const GenericAngle<T>& x, const GenericAngle<T>& y)
{
    return x.toRadians() / y.toRadians();
}

template <typename T>
inline GenericAngle<T>& operator+=(GenericAngle<T>& x, const GenericAngle<T>& y)
{
    return x = x + y;
}

template <typename T>
inline GenericAngle<T>& operator-=(GenericAngle<T>& x, const GenericAngle<T>& y)
{
    return x = x - y;
}

template <typename T>
inline GenericAngle<T>& operator*=(GenericAngle<T>& angle, double scale)
{
    return angle = angle * scale;
}

template <typename T>
inline GenericAngle<T>& operator/=(GenericAngle<T>& angle, double divisor)
{
    return angle = angle / divisor;
}

template <typename T>
inline constexpr bool operator<(const GenericAngle<T>& x, const GenericAngle<T>& y)
{
    return x.toRadians() < y.toRadians();
}

template <typename T>
inline constexpr bool operator>(const GenericAngle<T>& x, const GenericAngle<T>& y)
{
    return x.toRadians() > y.toRadians();
}

template <typename T>
inline constexpr bool operator<=(const GenericAngle<T>& x, const GenericAngle<T>& y)
{
    return x.toRadians() <= y.toRadians();
}

template <typename T>
inline constexpr bool operator>=(const GenericAngle<T>& x, const GenericAngle<T>& y)
{
    return x.toRadians() >= y.toRadians();
}

template <typename T>
inline bool operator==(const GenericAngle<T>& x, const GenericAngle<T>& y)
{
    GenericAngle<T> diff = x.clamp().minDiff(y.clamp());
    return diff.toRadians() <= FIXED_EPSILON;
}

template <typename T>
inline constexpr bool operator!=(const GenericAngle<T>& x, const GenericAngle<T>& y)
{
    return x.toRadians() != y.toRadians();
}

template <typename T>
inline std::ostream& operator<<(std::ostream& os, const GenericAngle<T>& a)
{
    os << a.toRadians() << "R";
    return os;
}
