//==============================================================================
/**
 * \file LaneSupportPoint.cpp
 * \brief Support point for lane arcspline
 *
 * -----------------------------------------------------------------------------
 * \copyright &copy; 2014 Ibeo Automotive Systems GmbH, Hamburg, Germany
 *
 * \date   Oct 9, 2014
 * \author Stefan Kaufmann (stk)
 */
//==============================================================================

#include <ibeosdk/datablocks/snippets/LaneSupportPoint.hpp>

namespace ibeosdk{
namespace lanes{

//==============================================================================

	std::string const LaneSupportPoint::VERSION = "1.0.0";

//==============================================================================

LaneSupportPoint::LaneSupportPoint(
	const PositionWgs84& point,
	const Point2dFloat& lineOffsetLeft,
	const Point2dFloat& lineOffsetRight):
	m_point(point),
	m_lineOffsetLeft(lineOffsetLeft),
	m_lineOffsetRight(lineOffsetRight)
{

}

//==============================================================================

LaneSupportPoint::LaneSupportPoint()
{
	m_point = PositionWgs84();
	m_lineOffsetLeft = Point2dFloat();
	m_lineOffsetRight = Point2dFloat();
}

//==============================================================================

PositionWgs84 LaneSupportPoint::getPoint() const
{
	return m_point;
}

//==============================================================================

Point2dFloat LaneSupportPoint::getLeftOffset() const
{
	return m_lineOffsetLeft;
}

//==============================================================================

Point2dFloat LaneSupportPoint::getRightOffset() const
{
	return m_lineOffsetRight;
}

//==============================================================================

std::streamsize LaneSupportPoint::getSerializedSize() const
{
	return 	8 +                                          // double getLongitudeInDeg
					8 +                                          // double getLatitudeInDeg
					8 +                                          // double getCourseAngleInDeg
					m_lineOffsetRight.getSerializedSize() +      // Point2DFloat
					m_lineOffsetLeft.getSerializedSize();        // Point2DFloat
}

//==============================================================================

bool LaneSupportPoint::serialize(std::ostream& os) const
{
	const std::istream::pos_type startPos = os.tellp();

	ibeosdk::writeBE(os, m_point.getLongitudeInDeg());
	ibeosdk::writeBE(os, m_point.getLatitudeInDeg());
	ibeosdk::writeBE(os, m_point.getCourseAngleInDeg());

	ibeosdk::writeBE(os, m_lineOffsetLeft);
	ibeosdk::writeBE(os, m_lineOffsetRight);

	return !os.fail() && ((os.tellp() - startPos) == this->getSerializedSize());
}

//==============================================================================

bool LaneSupportPoint::deserialize(std::istream& is)
{
	const std::istream::pos_type startPos = is.tellg();

	double lon, lat, course;

	ibeosdk::readBE(is, lon);
	ibeosdk::readBE(is, lat);
	ibeosdk::readBE(is, course);

	m_lineOffsetLeft.deserialize(is);
	m_lineOffsetRight.deserialize(is);

	m_point.setLongitudeInDeg(lon);
	m_point.setLatitudeInDeg(lat);
	m_point.setCourseAngleInDeg(course);

	return !is.fail() && ((is.tellg() - startPos) == this->getSerializedSize());
}

//==============================================================================

} // namespace lanes
} // namespace ibeo
