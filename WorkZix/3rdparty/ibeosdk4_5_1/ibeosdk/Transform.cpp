//==============================================================================
/**
 * \file Transform.cpp
 * \brief Transformation helper class
 *
 * -----------------------------------------------------------------------------
 * \copyright &copy; 2016 Ibeo Automotive Systems GmbH, Hamburg, Germany
 *
 * \date   21.1.2016
 * \author Stefan Kaufmann (stk)
 */
//==============================================================================

#include "Transform.hpp"

namespace ibeosdk {

//==============================================================================

TransformationMatrix2dFloat Transform::transformToGlobal(const TransformationMatrix2dFloat& ref2Global, const TransformationMatrix2dFloat& rel2ref)
{
	return ref2Global * rel2ref;
}

//==============================================================================

TransformationMatrix2dFloat Transform::getTransformationSystem(const VehicleStateBasicEcu2808& vs)
{
	return TransformationMatrix2dFloat( vs.getCourseAngle(), Point2dFloat(float(vs.getXPosition()), float(vs.getYPosition())) );
}

//==============================================================================

TransformationMatrix2dFloat Transform::getTransformationSystem(const VehicleStateBasicEcu& vs)
{
	return TransformationMatrix2dFloat( vs.getCourseAngle(), Point2dFloat( float(vs.getXPos())/10000.0f, float(vs.getYPos())/10000.0f) );
}

//==============================================================================

TransformationMatrix2dFloat Transform::getTransformationSystem(const ObjectEcuEtDyn& object)
{
	return TransformationMatrix2dFloat(object.getCourseAngle(), object.getReferencePointCoord());
}

//==============================================================================


} // namespace ibeosdk