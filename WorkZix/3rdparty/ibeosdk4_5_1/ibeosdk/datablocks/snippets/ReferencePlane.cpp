//======================================================================
/*! \file ReferencePlane.cpp
 *\verbatim
 * ------------------------------------------
 *  (C) 2016 Ibeo Automotive Systems GmbH, Hamburg, Germany
 * ------------------------------------------
 *
 *  Created on: Mar 15, 2016
 *          by: Kristian Bischoff
 *\endverbatim
 *///-------------------------------------------------------------------
//======================================================================

#include <ibeosdk/datablocks/snippets/ReferencePlane.hpp>
#include <ibeosdk/datablocks/snippets/PointBase.hpp>

//======================================================================

namespace ibeosdk {

//==============================================================================

ReferencePlane::ReferencePlane(const VehicleStateBasicEcu2808& vsb)
  : m_rotationMatrixIsValid(false),
    m_rotationMatrix()
{
	const GpsPoint vsRef(vsb.getLongitude(), vsb.getLatitude(), vsb.getAltitude());
	GpsPoint vsGlobal;
	const Point3dFloat vsOffset = Point3dFloat((float)vsb.getXPosition(), (float)vsb.getYPosition(), vsb.getZPosition());
	PointBase::transformToGlobalCoordinates(ReferencePlane(vsRef), vsOffset, vsGlobal );

	setGpsPoint(vsGlobal);
	// set angle to 0 if the corresponding vsb entry is NaN
	(isnan(vsb.getCourseAngle()))        ? setYaw(.0)   : setYaw(vsb.getCourseAngle());
	(isnan(vsb.getVehiclePitchAngle()))  ? setPitch(.0) : setPitch(vsb.getVehiclePitchAngle());
	(isnan(vsb.getVehicleRollAngle()))   ? setRoll(.0)  : setRoll(vsb.getVehicleRollAngle());
}

//==============================================================================

bool ReferencePlane::operator==(const ReferencePlane& other) const
{
	return m_gpsPoint == other.m_gpsPoint
	    && m_yaw == other.m_yaw
	    && m_pitch == other.m_pitch
	    && m_roll == other.m_roll;
}

//==============================================================================

bool ReferencePlane::operator!=(const ReferencePlane& other) const
{
	return !(*this == other);
}

//======================================================================

std::streamsize ReferencePlane::getSerializedSize() const
{
	return std::streamsize(3* sizeof(float)) + m_gpsPoint.getSerializedSize();
}

//======================================================================

bool ReferencePlane::deserialize(std::istream& is)
{
	const std::istream::pos_type startPos = is.tellg();

	m_gpsPoint.deserialize(is);
	ibeosdk::readBE(is, m_yaw);
	ibeosdk::readBE(is, m_pitch);
	ibeosdk::readBE(is, m_roll);

	return !is.fail() && ((is.tellg() - startPos) == this->getSerializedSize());
}

//======================================================================

bool ReferencePlane::serialize(std::ostream& os) const
{
	const std::istream::pos_type startPos = os.tellp();

	m_gpsPoint.serialize(os);
	ibeosdk::writeBE(os, m_yaw);
	ibeosdk::writeBE(os, m_pitch);
	ibeosdk::writeBE(os, m_roll);

	return !os.fail() && ((os.tellp() - startPos) == this->getSerializedSize());
}

//======================================================================

} // namespace ibeosdk

//======================================================================
