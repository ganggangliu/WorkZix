//==============================================================================
/**
 * \file Transform.hpp
 * \brief Transformation helper class
 *
 * -----------------------------------------------------------------------------
 * \copyright &copy; 2016 Ibeo Automotive Systems GmbH, Hamburg, Germany
 *
 * \date   21.1.2016
 * \author Stefan Kaufmann (stk)
 */
//==============================================================================

#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP

#include <ibeosdk/TransformationMatrix2dFloat.hpp>
#include <ibeosdk/datablocks/snippets/ObjectEcuEtDyn.hpp>
#include <ibeosdk/datablocks/VehicleStateBasicEcu.hpp>
#include <ibeosdk/datablocks/VehicleStateBasicEcu2808.hpp>

namespace ibeosdk{

class Transform
{

public:
	static TransformationMatrix2dFloat getTransformationSystem(const ObjectEcuEtDyn& object);
	static TransformationMatrix2dFloat getTransformationSystem(const VehicleStateBasicEcu& vs);
	static TransformationMatrix2dFloat getTransformationSystem(const VehicleStateBasicEcu2808& vs);

	static TransformationMatrix2dFloat transformToGlobal(const TransformationMatrix2dFloat& ref2Global, const TransformationMatrix2dFloat& rel2ref);

private:

};

} // namespace ibeosdk;

#endif // TRANSFORM_HPP