//======================================================================
/*! \file TestSupport.cpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Sep 29, 2015
 *///-------------------------------------------------------------------

//======================================================================

#include "TestSupport.hpp"

//======================================================================

namespace ibeosdk {
namespace unittests {

//======================================================================

TestSupport::~TestSupport() {}

//======================================================================

template<>
bool TestSupport::getRandValue() { return (rand()%2) ? true : false; }

template<>
uint8_t TestSupport::getRandValue() { return uint8_t(rand()%(1<<8)); }

template<>
uint8_t TestSupport::getRandValue(const uint8_t rangeMax) { return uint8_t(rand()%uint16_t(rangeMax+1)); }

template<>
uint16_t TestSupport::getRandValue() { return uint16_t(rand()%(1<<16)); }

template<>
uint16_t TestSupport::getRandValue(const uint16_t rangeMax) { return uint16_t(uint32_t(rand())%uint32_t(rangeMax+1)); }

template<>
int16_t TestSupport::getRandValue() { return int16_t((rand()%(1<<16))-(1<<15)); }

template<>
uint32_t TestSupport::getRandValue() { return uint32_t(rand()); }

template<>
float TestSupport::getRandValue()
{
	const uint32_t r = uint32_t(rand());
	return *reinterpret_cast<const float*>(&r);
}

template<>
Point2d TestSupport::getRandValue() { return Point2d(getRandValue<int16_t>(), getRandValue<int16_t>()); }

template<>
PointSigma2d TestSupport::getRandValue() { return PointSigma2d(getRandValue<uint16_t>(), getRandValue<uint16_t>()); }

template<>
RefPointBoxLocation TestSupport::getRandValue() { return RefPointBoxLocation(getRandValue<uint8_t>()); }

template<>
ObjectClass TestSupport::getRandValue() { return ObjectClass(getRandValue<uint8_t>()); }

template<>
NTPTime TestSupport::getRandValue()
{
	const uint64_t t = (uint64_t(getRandValue<uint16_t>()) << 48)
			| (uint64_t(getRandValue<uint16_t>()) << 32)
			| (uint64_t(getRandValue<uint16_t>()) << 16)
			| (uint64_t(getRandValue<uint16_t>()));
	return NTPTime(t);
}

//======================================================================

} // namespace unittests
} // namespace ibeosdk

//======================================================================
