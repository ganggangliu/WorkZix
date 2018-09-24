//======================================================================
/*! \file ScannerInfo.cpp
 *
 * \copydoc Copyright
 * \author Mario Brumm (mb)
 * \date Apr 26, 2012
 *///-------------------------------------------------------------------

#include <ibeosdk/datablocks/snippets/ScannerInfo.hpp>
#include <ibeosdk/io.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

ScannerInfo::ScannerInfo()
  : m_deviceId(0),
    m_scannerType(ScannerType_Invalid),
    m_scanNumber(0),
    m_scannerStatus(0),
    m_startAngle(0.),
    m_endAngle(0.),
    m_scanStartTime(0),
    m_scanEndTime(0),
    m_scanStartTimeFromDevice(0),
    m_scanEndTimeFromDevice(0),
    m_scanFrequency(0.),
    m_beamTilt(0.),
    m_scanFlags(0),
    m_yawAngle(0.),
    m_pitchAngle(0.),
    m_rollAngle(0.),
    m_offsetX(0.),
    m_offsetY(0.),
    m_offsetZ(0.),
    m_ri(8)
{}

//======================================================================

ScannerInfo::ScannerInfo(const ScannerInfo& other)
  : m_deviceId(other.m_deviceId),
    m_scannerType(other.m_scannerType),
    m_scanNumber(other.m_scanNumber),
    m_scannerStatus(other.m_scannerStatus),
    m_startAngle(other.m_startAngle),
    m_endAngle(other.m_endAngle),
    m_scanStartTime(other.m_scanStartTime),
    m_scanEndTime(other.m_scanEndTime),
    m_scanStartTimeFromDevice(other.m_scanStartTimeFromDevice),
    m_scanEndTimeFromDevice(other.m_scanEndTimeFromDevice),
    m_scanFrequency(other.m_scanFrequency),
    m_beamTilt(other.m_beamTilt),
    m_scanFlags(other.m_scanFlags),
    m_yawAngle(other.m_yawAngle),
    m_pitchAngle(other.m_pitchAngle),
    m_rollAngle(other.m_rollAngle),

    m_offsetX(other.m_offsetX),
    m_offsetY(other.m_offsetY),
    m_offsetZ(other.m_offsetZ),
    m_ri(other.m_ri.size()) // only set the size here
{
	m_ri = other.m_ri;
}

//======================================================================

ScannerInfo::~ScannerInfo() {}

//======================================================================

bool ScannerInfo::deserialize(std::istream& is)
{
	const std::istream::pos_type startPos = is.tellg();

	ibeosdk::readBE(is, m_deviceId);

	UINT8 st;
	ibeosdk::readBE(is, st);
	m_scannerType = ScannerType(st);

	ibeosdk::readBE(is, m_scanNumber);

	ibeosdk::readBE(is, m_scannerStatus);

	ibeosdk::readBE(is, m_startAngle);
	ibeosdk::readBE(is, m_endAngle);

	ibeosdk::readBE(is, m_scanStartTime);
	ibeosdk::readBE(is, m_scanEndTime);

	ibeosdk::readBE(is, m_scanStartTimeFromDevice);
	ibeosdk::readBE(is, m_scanEndTimeFromDevice);

	ibeosdk::readBE(is, m_scanFrequency);
	ibeosdk::readBE(is, m_beamTilt);

	ibeosdk::readBE(is, m_scanFlags);

	ibeosdk::readBE(is, m_yawAngle);
	ibeosdk::readBE(is, m_pitchAngle);
	ibeosdk::readBE(is, m_rollAngle);

	ibeosdk::readBE(is, m_offsetX);
	ibeosdk::readBE(is, m_offsetY);
	ibeosdk::readBE(is, m_offsetZ);

	for (unsigned int i=0; i<nbOfResolutionInfo; i++) {
		m_ri[i].deserialize(is);
	}

	return !is.fail() && ((is.tellg() - startPos) == this->getSerializedSize());
}
//======================================================================

//static
std::streamsize ScannerInfo::getSerializedSize_static()
{
	return 2 * std::streamsize(sizeof(UINT8))
		+ std::streamsize(sizeof(UINT16))
		+ std::streamsize(sizeof(UINT32))
		+ 2*std::streamsize(sizeof(float))
		+ 4*std::streamsize(sizeof(NTPTime))
		+ 2*std::streamsize(sizeof(float))
		+ std::streamsize(sizeof(UINT32))
		+ 6*std::streamsize(sizeof(float))
		+ 8* ResolutionInfo::getSerializedSize_static();
}

//======================================================================

bool ScannerInfo::serialize(std::ostream& os) const
{
	const std::istream::pos_type startPos = os.tellp();

	ibeosdk::writeBE(os, m_deviceId);

	ibeosdk::writeBE(os, uint8_t(m_scannerType));

	ibeosdk::writeBE(os, m_scanNumber);

	ibeosdk::writeBE(os, m_scannerStatus);

	ibeosdk::writeBE(os, m_startAngle);
	ibeosdk::writeBE(os, m_endAngle);

	ibeosdk::writeBE(os, m_scanStartTime);
	ibeosdk::writeBE(os, m_scanEndTime);

	ibeosdk::writeBE(os, m_scanStartTimeFromDevice);
	ibeosdk::writeBE(os, m_scanEndTimeFromDevice);

	ibeosdk::writeBE(os, m_scanFrequency);
	ibeosdk::writeBE(os, m_beamTilt);

	ibeosdk::writeBE(os, m_scanFlags);

	ibeosdk::writeBE(os, m_yawAngle);
	ibeosdk::writeBE(os, m_pitchAngle);
	ibeosdk::writeBE(os, m_rollAngle);

	ibeosdk::writeBE(os, m_offsetX);
	ibeosdk::writeBE(os, m_offsetY);
	ibeosdk::writeBE(os, m_offsetZ);

	for (unsigned int i=0; i<nbOfResolutionInfo; i++) {
		m_ri[i].serialize(os);
	}

	return !os.fail() && ((os.tellp() - startPos) == this->getSerializedSize());
}

//======================================================================

} // namespace ibeosdk

//======================================================================
