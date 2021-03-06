//======================================================================
/*! \file PointBase.hpp
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

#ifndef IBEOSDK_POINTBASE_HPP_SEEN
#define IBEOSDK_POINTBASE_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/datablocks/snippets/Snippet.hpp>
#include <ibeosdk/datablocks/snippets/GpsPoint.hpp>
#include <ibeosdk/datablocks/snippets/ReferencePlane.hpp>
#include <ibeosdk/datablocks/snippets/PointCloudBase.hpp>

//======================================================================

namespace ibeosdk {

class PointBase;

//======================================================================

class PointProxy {
private:
	PointProxy(PointBase* const point, PointCloudBase* const cloud);
	virtual ~PointProxy() {}

public:
	virtual GpsPoint getGlobalCoordinates() const = 0;
	virtual GpsPoint getReferencePointCoordinates() const;

	virtual const ReferencePlane& getReferencePlane() const;
	virtual ReferencePlane& referencePlane();

	virtual Point3dFloat getOffsetFromReferencePoint() const = 0;

	virtual pointKind::PointKind getKind() const;
	virtual pointType::PointType getType() const;

public: // Accessor functions
	virtual float  getEpw() const;
	virtual void   setEpw(const float epw);
	virtual bool   hasEpw() const;

	virtual uint32_t getFlags() const;
	virtual void   setFlags(const uint32_t flags);
	virtual bool   hasFlags() const;

protected:
	PointBase*      m_point;
	PointCloudBase* m_cloud;

private:
	friend class GlobalPointProxy;
	friend class PlanePointProxy;
}; // PointProxy

//======================================================================

//======================================================================
//======================================================================

class PointBase: public Snippet {
public:
	PointBase() : m_epw(.0f), m_flags(0) {}
	virtual ~PointBase() {}

public:
	static std::streamsize getSerializedSizeWithType_static(const pointType::PointType& type);

public:
	static void transformToGlobalCoordinates(const ReferencePlane& refPlane,
	                                         const Point3dFloat& offset,
	                                         GpsPoint& globalPoint);

	static void transformToRelativeCoordinates(const ReferencePlane& origin,
	                                           const GpsPoint& globalPoint,
	                                           Point3dFloat& offset);

	static void transformToRelativeCoordinates(const ReferencePlane& originRefPlane,
	                                           const EcefPoint& originRefPointsEcef,
	                                           const Matrix3dFloat& invTargetRotMatrix,
	                                           const GpsPoint& globalPoint,
	                                           Point3dFloat& offset);

	static void transformToShiftedReferencePlane(const ReferencePlane& origin,
	                                             const ReferencePlane& target,
	                                             const Point3dFloat& originOffset,
	                                             Point3dFloat& offset);

	static void transformToShiftedReferencePlane(const ReferencePlane& originRefPlane,
	                                             const ReferencePlane& targetRefPlane,
	                                             const EcefPoint& targetRefPointsEcef,
	                                             const Matrix3dFloat& invTargetRotMatrix,
	                                             const Point3dFloat& originOffset,
	                                             Point3dFloat& offsetTarget);
public:
	virtual bool operator==(const PointBase& other) const;

public:
	//TODO add comments not implemented
	virtual std::streamsize getSerializedSize() const;
	virtual bool serialize(std::ostream& os) const;
	virtual bool deserialize(std::istream& is);

	//todo this is preferred
	virtual std::streamsize getSerializedSizeWithType(const pointType::PointType& type) const;
	virtual bool deserializeWithType(std::istream& is, const pointType::PointType& type);
	virtual bool serializeWithType(std::ostream& os, const pointType::PointType& type) const;

public:
	virtual float getEpw() const { return m_epw; }
	virtual void setEpw(const float epw) { m_epw = epw; }

	virtual uint32_t getFlags() const { return m_flags; }
	virtual void setFlags(const uint32_t flags) { m_flags = flags; }

private:
	float m_epw;
	uint32_t m_flags;
}; // PointBase

//======================================================================

} // namespace ibeosdk

//======================================================================

#endif // IBEOSDK_POINTBASE_HPP_SEEN

//======================================================================
