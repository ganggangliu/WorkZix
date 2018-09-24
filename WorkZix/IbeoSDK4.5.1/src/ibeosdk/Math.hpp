//======================================================================
/*! \file Math.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Sep 29, 2013
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_MATH_HPP_SEEN
#define IBEOSDK_MATH_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <limits>
#include <cassert>
#include <cmath>
#if _MSC_VER == 1600
#ifdef _M_X64
#include <math.h>
#endif // _M_X64
#endif //_MSC_VER == 1600

//======================================================================

namespace ibeosdk {

//======================================================================

#ifdef _WIN32
/*!\brief rename VC _isnan function as isnan for compatibility
 *///-------------------------------------------------------------------
inline bool isnan(const double d)
{
	return 0 != _isnan(d);
}
#endif

//======================================================================
/*!\brief Shortcut for the double NaN value.
 *///-------------------------------------------------------------------
const double NaN_double = std::numeric_limits<double>::quiet_NaN();

const double twoPi = 2.0 * M_PI;
const double rad2deg = 180.0 / M_PI;
const double deg2rad = M_PI / 180.0;

//======================================================================
/*!\brief Shortcut for the float NaN value.
 *///-------------------------------------------------------------------
const float NaN = std::numeric_limits<float>::quiet_NaN();

//======================================================================

const float twoPif = float(2.0 * M_PI);
const float rad2degf = float(180.0 / M_PI);
const float deg2radf = float(M_PI / 180.0);

//======================================================================
/*!\brief Tests whether two \c float values are nearly equal.
 * \param[in] a  First value to be compared with second value.
 * \param[in] b  Second value to be compared with first value.
 * \return \c true if the two \c float numbers are equal in
 *         terms of the machine precision, which means their
 *         difference must be less than 1E-6.
 *///-------------------------------------------------------------------
inline bool fuzzyCompare(const float a, const float b)
{
	return std::abs(a - b) < 1E-6f;
}

//======================================================================
/*!\brief Tests whether two \c float values are nearly equal.
 * \param[in] a  First value to be compared with second value.
 * \param[in] b  Second value to be compared with first value.
 * \return \c true if the two \c float numbers are equal in
 *         terms of the machine precision, which means their
 *         difference must be less than 1E-6.
 *///-------------------------------------------------------------------
inline bool fuzzyCompare(const double a, const double b)
{
	return std::abs(a - b) < 1E-6;
}

//======================================================================

inline bool floatEqual(const float a, const float b)
{
	return ((isnan(a) && isnan(b)) || a == b);
}

//======================================================================

inline bool floatInequal(const float a, const float b)
{
	return !floatEqual(a,b);
}

//======================================================================
/*!\brief Normalize the given angle.
 *
 * Normalizes an angle given in radians by adding or subtracting an integer
 * multiple of 2*pi so that the resulting angle is in the half-open interval
 * (-pi,+pi]. The current implementation takes O(1) time, i.e. the time of
 * execution has a fixed upper boundary independend from the angle.
 *
 * \param[in] angleInRad  Angle to be normalized given in rad.
 * \return The normalized angle in (-pi, +pi].
 * \todo check whether (-pi, +pi] or [-pi, +pi) is correct.
 *///-------------------------------------------------------------------
inline float normalizeRadians (float angleInRad)
{
	if (std::abs(angleInRad) > (3.0f * M_PI)) {
		// For numerical stability we must use this sin/cos/atan2
		// implementation even though it might consume more cycles.
		// Note that radians = -pi converts to atan2(0,-1) = +pi!
		angleInRad = std::atan2 (std::sin(angleInRad), std::cos(angleInRad));
		// radians now in (-pi,+pi]
	} // if
	else {
		// fast version for "normal" out of range values
		while (angleInRad > M_PI) {
			angleInRad -= twoPif;
		} // while
		while (angleInRad <= float(-M_PI)) {
			angleInRad += twoPif;
		} // while
	} // else
	return angleInRad;
}

//======================================================================
/*!\brief Normalize the given angle.
 *
 * Normalizes an angle given in radians by adding or subtracting an integer
 * multiple of 2*pi so that the resulting angle is in the half-open interval
 * (-pi,+pi]. The current implementation takes O(1) time, i.e. the time of
 * execution has a fixed upper boundary independend from the angle.
 *
 * \param[in] angleInRad  Angle to be normalized given in rad.
 * \return The normalized angle in (-pi, +pi].
 * \todo check whether (-pi, +pi] or [-pi, +pi) is correct.
 *///-------------------------------------------------------------------
inline double normalizeRadians (double angleInRad)
{
	if (std::abs(angleInRad) > (3.0f * M_PI)) {
		// For numerical stability we must use this sin/cos/atan2
		// implementation even though it might consume more cycles.
		// Note that radians = -pi converts to atan2(0,-1) = +pi!
		angleInRad = std::atan2 (std::sin(angleInRad), std::cos(angleInRad));
		// radians now in (-pi,+pi]
	} // if
	else {
		// fast version for "normal" out of range values
		while (angleInRad > M_PI) {
			angleInRad -= twoPi;
		} // while
		while (angleInRad <= -M_PI) {
			angleInRad += twoPi;
		} // while
	} // else
	return angleInRad;
}

//======================================================================

} // namespace ibeosdk

//======================================================================

#endif // IBEOSDK_MATH_HPP_SEEN
