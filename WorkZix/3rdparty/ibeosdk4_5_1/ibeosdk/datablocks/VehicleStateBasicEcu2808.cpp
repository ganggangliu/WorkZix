//======================================================================
/*! \file VehicleStateBasicEcu2808.cpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Oct 5, 2015
 *///-------------------------------------------------------------------

//======================================================================

#include <ibeosdk/datablocks/VehicleStateBasicEcu2808.hpp>
#include <ibeosdk/DataBlockBufferProvider.hpp>
#include <ibeosdk/io.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
// Specializations for RegisteredDataBlock<VehicleStateBasicEcu2808>
//======================================================================

template<>
const DataTypeId ibeosdk::RegisteredDataBlock<VehicleStateBasicEcu2808>::dataBlockId = DataTypeId(DataTypeId::DataType_EcuVehicleStateBasic2808);
template<>
const DataBlock::DataBlockRegisterId ibeosdk::RegisteredDataBlock<VehicleStateBasicEcu2808>::registerIdInitial =
		DataBlockRegisterId(ibeosdk::RegisteredDataBlock<VehicleStateBasicEcu2808>::dataBlockId, ibeosdk::RegisteredDataBlock<VehicleStateBasicEcu2808>::create);

class IdcFile;
class IbeoEcu;

template<>
const DataBlock::DataBlockRegisterId ibeosdk::RegisteredDataBlock<VehicleStateBasicEcu2808>::registerId =
		DataBlockBufferProviderGlobal<IdcFile>::getInstance().registerDataBlock(
		DataBlockBufferProviderGlobal<IbeoEcu>::getInstance().registerDataBlock(registerIdInitial)
		);

//======================================================================

VehicleStateBasicEcu2808::VehicleStateBasicEcu2808()
  : m_microseconds(0),
    m_timestamp(),
    m_sources(0),
    m_blindPredictionAge(0),
    m_latitude(.0F),
    m_latitudeSigma(.0F),
    m_longitude(.0F),
    m_longitudeSigma(.0F),
    m_altitude(.0F),
    m_altitudeSigma(.0F),

    m_xPosition(.0F),
    m_xPositionSigma(.0F),
    m_yPosition(.0F),
    m_yPositionSigma(.0F),
    m_zPosition(.0F),
    m_zPositionSigma(.0F),

    m_xyCorrelation(.0F),
    m_xzCorrelation(.0F),
    m_yzCorrelation(.0F),

    m_courseAngle(.0F),
    m_courseAngleSigma(.0F),
    m_headingAngle(.0F),
    m_headingAngleSigma(.0F),

    m_vehiclePitchAngle(.0F),
    m_vehiclePitchAngleSigma(.0F),
    m_vehicleRollAngle(.0F),
    m_vehicleRollAngleSigma(.0F),

    m_vehicleVelocity(.0F),
    m_vehicleVelocitySigma(.0F),

    m_yawRate(.0F),
    m_yawRateSigma(.0F),

    m_longitudinalAccelearation(.0F),
    m_longitudinalAccelearationSigma(.0F),
    m_crossAccelearation(.0F),
    m_crossAccelearationSigma(.0F),

    m_steerAngle(.0F),
    m_steeringWheelAngle(.0F),

    m_vehicleWidth(.0F),
    m_minTurningCycle(.0F),

    m_vehicleFrontToFrontAxle(.0F),
    m_frontAxleToRearAxle(.0F),
    m_rearAxleToVehicleRear(.0F),
    m_rearAxleToOrigin(.0F),
    m_stearRatioCoeff0(.0F),
    m_stearRatioCoeff1(.0F),
    m_stearRatioCoeff2(.0F),
    m_stearRatioCoeff3(.0F)
{}

//======================================================================

VehicleStateBasicEcu2808::~VehicleStateBasicEcu2808() {}

//======================================================================

DataTypeId VehicleStateBasicEcu2808::getDataType() const { return dataBlockId; }

//======================================================================

bool VehicleStateBasicEcu2808::deserialize(std::istream& is, const IbeoDataHeader& dh)
{
	const std::istream::pos_type startPos = is.tellg();

	lock();

	ibeosdk::readBE(is, m_microseconds);
	ibeosdk::readBE(is, m_timestamp);
	ibeosdk::readBE(is, m_sources);
	ibeosdk::readBE(is, m_blindPredictionAge);

	ibeosdk::readBE(is, m_latitude);
	ibeosdk::readBE(is, m_latitudeSigma);
	ibeosdk::readBE(is, m_longitude);
	ibeosdk::readBE(is, m_longitudeSigma);
	ibeosdk::readBE(is, m_altitude);
	ibeosdk::readBE(is, m_altitudeSigma);

	ibeosdk::readBE(is, m_xPosition);
	ibeosdk::readBE(is, m_xPositionSigma);
	ibeosdk::readBE(is, m_yPosition);
	ibeosdk::readBE(is, m_yPositionSigma);
	ibeosdk::readBE(is, m_zPosition);
	ibeosdk::readBE(is, m_zPositionSigma);

	ibeosdk::readBE(is, m_xyCorrelation);
	ibeosdk::readBE(is, m_xzCorrelation);
	ibeosdk::readBE(is, m_yzCorrelation);

	ibeosdk::readBE(is, m_courseAngle);
	ibeosdk::readBE(is, m_courseAngleSigma);
	ibeosdk::readBE(is, m_headingAngle);
	ibeosdk::readBE(is, m_headingAngleSigma);

	ibeosdk::readBE(is, m_vehiclePitchAngle);
	ibeosdk::readBE(is, m_vehiclePitchAngleSigma);
	ibeosdk::readBE(is, m_vehicleRollAngle);
	ibeosdk::readBE(is, m_vehicleRollAngleSigma);

	ibeosdk::readBE(is, m_vehicleVelocity);
	ibeosdk::readBE(is, m_vehicleVelocitySigma);

	ibeosdk::readBE(is, m_yawRate);
	ibeosdk::readBE(is, m_yawRateSigma);

	ibeosdk::readBE(is, m_longitudinalAccelearation);
	ibeosdk::readBE(is, m_longitudinalAccelearationSigma);
	ibeosdk::readBE(is, m_crossAccelearation);
	ibeosdk::readBE(is, m_crossAccelearationSigma);

	ibeosdk::readBE(is, m_steerAngle);
	ibeosdk::readBE(is, m_steeringWheelAngle);

	ibeosdk::readBE(is, m_vehicleWidth);
	ibeosdk::readBE(is, m_minTurningCycle);

	ibeosdk::readBE(is, m_vehicleFrontToFrontAxle);
	ibeosdk::readBE(is, m_frontAxleToRearAxle);
	ibeosdk::readBE(is, m_rearAxleToVehicleRear);
	ibeosdk::readBE(is, m_rearAxleToOrigin);
	ibeosdk::readBE(is, m_stearRatioCoeff0);
	ibeosdk::readBE(is, m_stearRatioCoeff1);
	ibeosdk::readBE(is, m_stearRatioCoeff2);
	ibeosdk::readBE(is, m_stearRatioCoeff3);

	unlock();

	return !is.fail()
	       && ((is.tellg() - startPos) == this->getSerializedSize())
	       && this->getSerializedSize() == dh.getMessageSize();
}


//======================================================================

bool VehicleStateBasicEcu2808::serialize(std::ostream& os) const
{
	const std::istream::pos_type startPos = os.tellp();

	lock();

	ibeosdk::writeBE(os, m_microseconds);
	ibeosdk::writeBE(os, m_timestamp);
	ibeosdk::writeBE(os, m_sources);
	ibeosdk::writeBE(os, m_blindPredictionAge);

	ibeosdk::writeBE(os, m_latitude);
	ibeosdk::writeBE(os, m_latitudeSigma);
	ibeosdk::writeBE(os, m_longitude);
	ibeosdk::writeBE(os, m_longitudeSigma);
	ibeosdk::writeBE(os, m_altitude);
	ibeosdk::writeBE(os, m_altitudeSigma);

	ibeosdk::writeBE(os, m_xPosition);
	ibeosdk::writeBE(os, m_xPositionSigma);
	ibeosdk::writeBE(os, m_yPosition);
	ibeosdk::writeBE(os, m_yPositionSigma);
	ibeosdk::writeBE(os, m_zPosition);
	ibeosdk::writeBE(os, m_zPositionSigma);

	ibeosdk::writeBE(os, m_xyCorrelation);
	ibeosdk::writeBE(os, m_xzCorrelation);
	ibeosdk::writeBE(os, m_yzCorrelation);

	ibeosdk::writeBE(os, m_courseAngle);
	ibeosdk::writeBE(os, m_courseAngleSigma);
	ibeosdk::writeBE(os, m_headingAngle);
	ibeosdk::writeBE(os, m_headingAngleSigma);

	ibeosdk::writeBE(os, m_vehiclePitchAngle);
	ibeosdk::writeBE(os, m_vehiclePitchAngleSigma);
	ibeosdk::writeBE(os, m_vehicleRollAngle);
	ibeosdk::writeBE(os, m_vehicleRollAngleSigma);

	ibeosdk::writeBE(os, m_vehicleVelocity);
	ibeosdk::writeBE(os, m_vehicleVelocitySigma);

	ibeosdk::writeBE(os, m_yawRate);
	ibeosdk::writeBE(os, m_yawRateSigma);

	ibeosdk::writeBE(os, m_longitudinalAccelearation);
	ibeosdk::writeBE(os, m_longitudinalAccelearationSigma);
	ibeosdk::writeBE(os, m_crossAccelearation);
	ibeosdk::writeBE(os, m_crossAccelearationSigma);

	ibeosdk::writeBE(os, m_steerAngle);
	ibeosdk::writeBE(os, m_steeringWheelAngle);

	ibeosdk::writeBE(os, m_vehicleWidth);
	ibeosdk::writeBE(os, m_minTurningCycle);

	ibeosdk::writeBE(os, m_vehicleFrontToFrontAxle);
	ibeosdk::writeBE(os, m_frontAxleToRearAxle);
	ibeosdk::writeBE(os, m_rearAxleToVehicleRear);
	ibeosdk::writeBE(os, m_rearAxleToOrigin);
	ibeosdk::writeBE(os, m_stearRatioCoeff0);
	ibeosdk::writeBE(os, m_stearRatioCoeff1);
	ibeosdk::writeBE(os, m_stearRatioCoeff2);
	ibeosdk::writeBE(os, m_stearRatioCoeff3);

	unlock();

	return !os.fail() && ((os.tellp() - startPos) == this->getSerializedSize());
}

//======================================================================

PositionWgs84 VehicleStateBasicEcu2808::getReferencePoint() const
{
	PositionWgs84 out;
	if (m_latitude == m_latitude && m_longitude == m_longitude)
	{
		out.setLatitudeInDeg(m_latitude);
		out.setLongitudeInDeg(m_longitude);
	}
	else
	{
		out.setLatitudeInDeg(0);
		out.setLongitudeInDeg(0);
	}
	out.setAltitudeInMeterMSL(m_altitude);
	out.setCourseAngleInDeg(0);

	return out;
}


}// namespace ibeosdk

//======================================================================
