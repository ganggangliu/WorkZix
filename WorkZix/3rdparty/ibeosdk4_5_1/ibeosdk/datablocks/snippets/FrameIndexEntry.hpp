//======================================================================
/*! \file FrameIndexEntry.hpp
 *
 * \copydoc Copyright
 * \author Kai-Uwe von Deylen (kd)
 * \date May 11, 2015
 *///-------------------------------------------------------------------
#ifndef IBEOSDK_FRAMEINDEXENTRY_HPP_SEEN
#define IBEOSDK_FRAMEINDEXENTRY_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/datablocks/snippets/Snippet.hpp>
#include <ibeosdk/Time.hpp>
#include <ibeosdk/inttypes.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

class FrameIndexEntry : public Snippet {

public:
	static std::streamsize getSerializedSize_static();

public:
	FrameIndexEntry();
	FrameIndexEntry(UINT64 filePos, NTPTime ts, UINT8 deviceId);
	virtual ~FrameIndexEntry();

public:
	virtual std::streamsize getSerializedSize() const;
	virtual bool deserialize(std::istream& is);
	virtual bool serialize(std::ostream& os) const;

public:
	UINT64 getFilePosition() const { return m_filePosition; }
	NTPTime getTimestamp() const { return m_timestamp; }
	UINT8 getDeviceId() const { return m_deviceId; }

public:
	void setFilePosition(const UINT64 filePosition)  { m_filePosition = filePosition; }
	void setTimestamp(const NTPTime timestamp)  { m_timestamp = timestamp; }
	void setDeviceId(const UINT8 deviceId)  { m_deviceId = deviceId; }

protected:
	UINT64   m_filePosition;  ///< position of the frame in the file
	NTPTime  m_timestamp;     ///< timestamp of this frame
	UINT8    m_deviceId;      ///< device id that defined the frame
	 
}; // FrameIndexEntry

//======================================================================

}// namespace ibeosdk

//======================================================================

#endif // IBEOSDK_FRAMEINDEXENTRY_HPP_SEEN

//======================================================================
