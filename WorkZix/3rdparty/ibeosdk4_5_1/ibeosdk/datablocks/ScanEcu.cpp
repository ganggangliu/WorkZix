//======================================================================
/*! \file ScanEcu.cpp
 *
 * \copydoc Copyright
 * \author Mario Brumm (mb)
 * \date Apr 25, 2012
 *///-------------------------------------------------------------------

#include <ibeosdk/datablocks/ScanEcu.hpp>
#include <ibeosdk/DataBlockBufferProvider.hpp>
#include <ibeosdk/io.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
// Specializations for RegisteredDataBlock<ScanEcu>
//======================================================================

template<>
const DataTypeId ibeosdk::RegisteredDataBlock<ScanEcu>::dataBlockId = DataTypeId(DataTypeId::DataType_EcuScan);
template<>
const DataBlock::DataBlockRegisterId ibeosdk::RegisteredDataBlock<ScanEcu>::registerIdInitial =
		DataBlockRegisterId(ibeosdk::RegisteredDataBlock<ScanEcu>::dataBlockId, ibeosdk::RegisteredDataBlock<ScanEcu>::create);

class IdcFile;
class IbeoEcu;

template<>
const DataBlock::DataBlockRegisterId ibeosdk::RegisteredDataBlock<ScanEcu>::registerId =
		DataBlockBufferProviderGlobal<IdcFile>::getInstance().registerDataBlock(
		DataBlockBufferProviderGlobal<IbeoEcu>::getInstance().registerDataBlock(registerIdInitial)
		);

//======================================================================

ScanEcu::ScanEcu()
  : m_scanStartTime(),
    m_endTimeOffset(),
    m_flags(),
    m_scanNumber(),
    m_nbOfScanPoints(0),
    m_nbOfScannerInfos(),
    m_reserved0(0),
    m_reserved1(0),
    m_scannerInfos(8),
    m_scanPoints(1024)
{}

//======================================================================

ScanEcu::~ScanEcu() {}

//======================================================================

DataTypeId ScanEcu::getDataType() const { return dataBlockId; }

//======================================================================

bool ScanEcu::deserialize(std::istream& is, const IbeoDataHeader& dh)
{
	const std::istream::pos_type startPos = is.tellg();

	lock();

	ibeosdk::readBE(is, m_scanStartTime);
	ibeosdk::readBE(is, m_endTimeOffset);

	ibeosdk::readBE(is, m_flags);
	ibeosdk::readBE(is, m_scanNumber);
	ibeosdk::readBE(is, m_nbOfScanPoints);
	ibeosdk::readBE(is, m_nbOfScannerInfos);

	ibeosdk::readBE(is, m_reserved0);
	ibeosdk::readBE(is, m_reserved1);

	if (m_scannerInfos.size() != m_nbOfScannerInfos)
		m_scannerInfos.resize(m_nbOfScannerInfos);

	for (unsigned int i=0; i<m_nbOfScannerInfos; i++) {
		m_scannerInfos[i].deserialize(is);
	}

	m_scanPoints.resize(m_nbOfScanPoints);

	for (unsigned int i=0; i<m_nbOfScanPoints; i++) {
		m_scanPoints[i].deserialize(is);
	}

	unlock();

	return !is.fail()
	       && ((is.tellg() - startPos) == this->getSerializedSize())
	       && this->getSerializedSize() == dh.getMessageSize();
}

//======================================================================

bool ScanEcu::serialize(std::ostream& os) const
{
	const std::istream::pos_type startPos = os.tellp();

	lock();

	ibeosdk::writeBE(os, m_scanStartTime);
	ibeosdk::writeBE(os, m_endTimeOffset);

	ibeosdk::writeBE(os, m_flags);
	ibeosdk::writeBE(os, m_scanNumber);
	ibeosdk::writeBE(os, m_nbOfScanPoints);
	ibeosdk::writeBE(os, m_nbOfScannerInfos);

	ibeosdk::writeBE(os, m_reserved0);
	ibeosdk::writeBE(os, m_reserved1);

	for (unsigned int i=0; i<m_nbOfScannerInfos; i++) {
		m_scannerInfos[i].serialize(os);
	}

	for (unsigned int i=0; i<m_nbOfScanPoints; i++) {
		m_scanPoints[i].serialize(os);
	}

	unlock();

	return !os.fail() && ((os.tellp() - startPos) == this->getSerializedSize());
}

//======================================================================

std::streamsize ScanEcu::getSerializedSize() const
{
	return std::streamsize(sizeof(NTPTime))
		+ std::streamsize(2*sizeof(UINT32))
		+ std::streamsize(2*sizeof(UINT16))
		+ std::streamsize(sizeof(UINT8))
		+ 3
		+ std::streamsize(m_nbOfScannerInfos) * ScannerInfo::getSerializedSize_static()
		+ std::streamsize(this->m_nbOfScanPoints) * ScanPointEcu::getSerializedSize_static();
}

//======================================================================

} // namespace ibeosdk

//======================================================================
