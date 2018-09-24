//======================================================================
/*! \file FrameIndexEntry.cpp
 *
 * \copydoc Copyright
 * \author Kai-Uwe von Deylen (kd)
 * \date May 11, 2015
 *///-------------------------------------------------------------------
//======================================================================

#include <ibeosdk/datablocks/snippets/FrameIndexEntry.hpp>
#include <ibeosdk/io.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================	

std::streamsize FrameIndexEntry::getSerializedSize_static()
{
	return std::streamsize(
	       8
	     + 8
	     + 1);
}

//======================================================================

FrameIndexEntry::FrameIndexEntry()
  : m_filePosition(0),
    m_timestamp(),
    m_deviceId(255)
{}

//======================================================================

FrameIndexEntry::FrameIndexEntry(UINT64 filePos, NTPTime ts, UINT8 deviceId)
  : m_filePosition(filePos),
    m_timestamp(ts),
    m_deviceId(deviceId)
{}

//======================================================================

FrameIndexEntry::~FrameIndexEntry() {}

//======================================================================

std::streamsize FrameIndexEntry::getSerializedSize() const
{
	return getSerializedSize_static();
}

//======================================================================

bool FrameIndexEntry::deserialize(std::istream& is)
{
	const std::istream::pos_type startPos = is.tellg();

	ibeosdk::readBE(is, m_filePosition);
	ibeosdk::readBE(is, m_timestamp);
	ibeosdk::readBE(is, m_deviceId);

	return !is.fail()
	       && ((is.tellg() - startPos) == this->getSerializedSize());
}

//======================================================================

bool FrameIndexEntry::serialize(std::ostream& os) const
{
	const std::istream::pos_type startPos = os.tellp();

	ibeosdk::writeBE(os, m_filePosition);
	ibeosdk::writeBE(os, m_timestamp);
	ibeosdk::writeBE(os, m_deviceId);

	return !os.fail() && ((os.tellp() - startPos) == this->getSerializedSize());
}

//======================================================================

} // namespace ibeosdk

//======================================================================
