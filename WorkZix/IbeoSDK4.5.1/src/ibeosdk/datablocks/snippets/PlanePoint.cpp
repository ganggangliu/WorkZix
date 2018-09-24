//======================================================================
/*! \file PlanePoint.cpp
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

#include <ibeosdk/datablocks/snippets/PlanePoint.hpp>

#include <ibeosdk/datablocks/PointCloudPlane7510.hpp>

#include <ibeosdk/io.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

PlanePointProxy::PlanePointProxy(PlanePoint* const point, PointCloudPlane7510* const cloud)
  : PointProxy(point, cloud)
{}

//==============================================================================

GpsPoint PlanePointProxy::getGlobalCoordinates() const
{
	PlanePoint* point = static_cast<PlanePoint*>(m_point);
	PointCloudPlane7510* cloud = static_cast<PointCloudPlane7510*>(m_cloud);

	GpsPoint ret;
	PointBase::transformToGlobalCoordinates(cloud->getReferencePlane(), point->getPoint3D(), ret);

	return ret;
}

//==============================================================================

Point3dFloat PlanePointProxy::getOffsetFromReferencePoint() const
{
	return static_cast<PlanePoint*>(m_point)->getPoint3D();
}

//======================================================================





//======================================================================
//======================================================================
//======================================================================





//======================================================================

std::streamsize PlanePoint::getSerializedSize_static()
{
	return Point3dFloat::getSerializedSize_static();
}

//======================================================================

std::streamsize PlanePoint::getSerializedSize() const
{
	return getSerializedSize_static();
}

//======================================================================

bool PlanePoint::deserialize(std::istream& is)
{
	const std::istream::pos_type startPos = is.tellg();

	m_point.deserialize(is);

	return !is.fail() && ((is.tellg() - startPos) == this->getSerializedSize());
}

//======================================================================

bool PlanePoint::serialize(std::ostream& os) const
{
	const std::istream::pos_type startPos = os.tellp();

	m_point.serialize(os);

	return !os.fail() && ((os.tellp() - startPos) == this->getSerializedSize());
}

//======================================================================

} //namespace ibeosdk

//======================================================================
