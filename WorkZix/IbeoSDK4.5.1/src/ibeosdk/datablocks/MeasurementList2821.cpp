//======================================================================
/*! \file MeasurementList2821.cpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Aug 11, 2014
 *///-------------------------------------------------------------------
//======================================================================

#include <ibeosdk/datablocks/MeasurementList2821.hpp>
#include <ibeosdk/DataBlockBufferProvider.hpp>
#include <ibeosdk/io.hpp>
#include <ibeosdk/bufferIO.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
// Specializations for RegisteredDataBlock<MeasurementList2821>
//======================================================================

template<>
const DataTypeId ibeosdk::RegisteredDataBlock<MeasurementList2821>::dataBlockId = DataTypeId(DataTypeId::DataType_MeasurementList2821);

template<>
const DataBlock::DataBlockRegisterId ibeosdk::RegisteredDataBlock<MeasurementList2821>::registerIdInitial =
		DataBlockRegisterId(ibeosdk::RegisteredDataBlock<MeasurementList2821>::dataBlockId, ibeosdk::RegisteredDataBlock<MeasurementList2821>::create);

class IdcFile;
class IbeoEcu;
class IbeoScala;

template<>
const DataBlock::DataBlockRegisterId ibeosdk::RegisteredDataBlock<MeasurementList2821>::registerId =
		DataBlockBufferProviderGlobal<IdcFile>::getInstance().registerDataBlock(
		DataBlockBufferProviderGlobal<IbeoEcu>::getInstance().registerDataBlock(
		DataBlockBufferProviderGlobal<IbeoScala>::getInstance().registerDataBlock(registerIdInitial)
		));

//======================================================================
//======================================================================
//======================================================================

MeasurementList2821::MeasurementList2821()
  : RegisteredDataBlock<MeasurementList2821>(),
    m_microseconds(0),
    m_timestamp(),
    m_listName(),
    m_groupName(),
    m_mList()
{}

//======================================================================

MeasurementList2821::~MeasurementList2821() {}

//======================================================================

std::streamsize MeasurementList2821::getSerializedSize() const
{
	return std::streamsize(4 + sizeof(NTPTime))
			+ std::streamsize(m_listName.size() + 1)
			+ std::streamsize(m_groupName.size() + 1)
			+ m_mList.getSerializedSize();
}

//======================================================================

DataTypeId MeasurementList2821::getDataType() const { return dataBlockId; }

//======================================================================

bool MeasurementList2821::serialize(std::ostream& os) const
{
	const std::istream::pos_type startPos = os.tellp();

	lock();

	ibeosdk::writeBE(os, this->m_microseconds);    // 4
	ibeosdk::writeBE(os, this->m_timestamp);       // 8

	os << m_listName;  ibeosdk::write(os, std::string::value_type(0));
	os << m_groupName; ibeosdk::write(os, std::string::value_type(0));
	m_mList.serialize(os);

	unlock();

	return !os.fail() && ((os.tellp() - startPos) == this->getSerializedSize());
}

//======================================================================

bool MeasurementList2821::deserialize(std::istream& is, const IbeoDataHeader& dh)
{
	const std::istream::pos_type startPos = is.tellg();

	lock();

	ibeosdk::readBE(is, this->m_microseconds);    // 4
	ibeosdk::readBE(is, this->m_timestamp);       // 8

	getline(is, m_listName, std::string::value_type(0));
	getline(is, m_groupName, std::string::value_type(0));

	m_mList.deserialize(is);

	unlock();

	return !is.fail()
	       && ((is.tellg() - startPos) == this->getSerializedSize())
	       && this->getSerializedSize() == dh.getMessageSize();
}

//======================================================================

}// namespace ibeosdk

//======================================================================
