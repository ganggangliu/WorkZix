//======================================================================
/*! \file PlanePoint.hpp
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

#ifndef IBEOSDK_PLANEPOINT_HPP_SEEN
#define IBEOSDK_PLANEPOINT_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/datablocks/snippets/PointBase.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

class PointCloudPlane7510;
class PlanePoint;

//======================================================================

class PlanePointProxy : public PointProxy {
public:
	PlanePointProxy(PlanePoint* const point, PointCloudPlane7510* const cloud);
	virtual ~PlanePointProxy() {}

public:
	virtual GpsPoint getGlobalCoordinates() const;
	virtual Point3dFloat  getOffsetFromReferencePoint() const;
}; // PlanePointProxy

//======================================================================

//======================================================================
//======================================================================

class PlanePoint : public PointBase {
public:
	PlanePoint() : PointBase() {}
	PlanePoint(const Point3dFloat point) : PointBase(), m_point(point) {}

public:
	static std::streamsize getSerializedSize_static();

public:
	virtual std::streamsize getSerializedSize() const;
	virtual bool deserialize(std::istream& is);
	virtual bool serialize(std::ostream& os) const;

public:
	const Point3dFloat& getPoint3D() const { return m_point; }
	Point3dFloat& point3D(){ return m_point; }
	void setPoint3D(const Point3dFloat& point) { m_point = point; }

private:
	Point3dFloat m_point;
}; // PlanePoint

//======================================================================

} // namespace ibeosdk

//======================================================================

#endif // IBEOSDK_PLANEPOINT_HPP_SEEN

//======================================================================
