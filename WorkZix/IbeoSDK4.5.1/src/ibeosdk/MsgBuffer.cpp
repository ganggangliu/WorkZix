//======================================================================
/*! \file MsgBuffer.cpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Jun 28, 2012
 *///-------------------------------------------------------------------

#include <ibeosdk/MsgBuffer.hpp>
#include <ibeosdk/Log.hpp>
#include <ibeosdk/inttypes.hpp>

#include <boost/array.hpp>
#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/stream.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

MsgBuffer::MsgBuffer(boost::asio::ip::tcp::socket* recvSocket, const int bufSize)
  : m_socket(recvSocket),
    m_bufCapacity(bufSize),
    m_ptr(new char[uint32_t(bufSize)]),
    m_insAt(0),
    m_processedTo(-1),
    m_scheduledForDeletionTo(-1),
    m_dh(),
    m_dhAt(-1),
    m_bodyAt(-1),
    m_bodySize(-1),
    m_missingBytes(-1),
    m_needToRecvData(true)
{
	assert(this->m_bufCapacity > IbeoDataHeader::getHeaderSize());
}

//======================================================================

MsgBuffer::~MsgBuffer()
{
	if (m_ptr) delete[] m_ptr;
}

//======================================================================

bool MsgBuffer::recvMsg(const IbeoDataHeader*& recvDataHeader, const char*& startOfMsgBody)
{
	recvDataHeader = NULL;
	startOfMsgBody = NULL;

	if (this->m_processedTo >= this->m_bufCapacity) {
		// if the complete buffer is already processed
		// up to its capacity, clear it.
		this->clearBufferCompletely();
	}
	else if (this->m_processedTo >= this->m_bufCapacity-IbeoDataHeader::getHeaderSize()) {
		// if there is not enough room for at least an
		// incomming header, remove all already processed
		// bytes by moving the later bytes forward.
		this->removeScheduledBytes();
	}

	int recvLen = 0;
	if (this->m_needToRecvData) {
		const int bytesToRead = this->m_bufCapacity - this->m_insAt;
		char* const recvBuf = this->m_ptr + this->m_insAt;
		boost::system::error_code error;
		recvLen = int(m_socket->read_some(boost::asio::buffer(recvBuf, std::size_t(bytesToRead)), error));
		m_insAt += recvLen;

		if (error == boost::asio::error::eof) {
			logError << "TCP/IP eof error occured  -  " << error.message() << std::endl;
			throw error;
		}
		else if (error) {
			logError << "TCP/IP error occured  -  " << error.message() << std::endl;
			throw error;
		}
	} // if

	if (this->m_missingBytes>0) {
		// not enough bytes to complete the expected body?
		if (recvLen < this->m_missingBytes) {
			// register the recevied bytes
			this->m_missingBytes -= recvLen;
			this->m_processedTo = this->m_insAt-1;
			// and return without an result
			this->m_needToRecvData = true;
			return false;
		} // if (recvLen < this->missingBytes)
		else {
			recvDataHeader = &this->m_dh;
			startOfMsgBody = this->m_ptr + this->m_bodyAt;
			this->m_processedTo +=  this->m_missingBytes;
			this->m_missingBytes = -1;
			this->m_needToRecvData = false;
			return true;
		}
	} // if (missingByte)

	// no message in progress here, dh is invalid
	// all data already processed up to the last call
	// can be deleted.
	this->m_scheduledForDeletionTo = this->m_processedTo;

	if (this->m_processedTo >= this->m_bufCapacity-IbeoDataHeader::getHeaderSize()) {
		// there is not enough room left in the buffer
		// for an IbeoDataHeader. Cleaning up.
		this->removeScheduledBytes();
	}

	// m_processedTo is at least 1 less than m_insAt. In that case there is no valid
	// byte in the buffer, that has not already been processed.
	const int validBytes = this->m_insAt - (this->m_processedTo + 1);
	const int startBufferAt = this->m_processedTo + 1;

	boost::iostreams::stream<boost::iostreams::array_source> strm(this->m_ptr+startBufferAt, validBytes);
	std::istream& s = static_cast<std::istream&>(strm);
	if (IbeoDataHeader::moveToMagicWord(s)) {
		if (!m_dh.deserialize(s)) { // dh could not completely been read
			this->removeScheduledBytes();
			this->m_needToRecvData = true;
			return false;
		}
		const int bufPos = this->m_processedTo + int(s.tellg()) +1;
		this->m_dhAt = bufPos - IbeoDataHeader::getHeaderSize();
		this->m_bodyAt = bufPos;
		if (this->m_dh.getMessageSize() <= int64_t(this->m_bufCapacity))
			this->m_bodySize = int(this->m_dh.getMessageSize());
		else {
			logError << "Illegal message body size occured, buffer size (" << this->m_bufCapacity << " Bytes) exceeded: "
					<< this->m_dh.getMessageSize() << " Bytes in received DataBlock: "
					<< "0x" << std::hex << m_dh.getDataType() << std::dec
					<< " Set to 0."
					<< std::endl;
			this->m_bodySize = -1;
			this->m_processedTo = this->m_bodyAt;
			this->m_missingBytes = -1;
			this->m_needToRecvData = false;
			return false;
		} // else

		if (this->m_bodySize <= this->m_insAt - bufPos) {
			recvDataHeader = &this->m_dh;
			startOfMsgBody = this->m_ptr + this->m_bodyAt;
			this->m_processedTo = this->m_bodyAt + this->m_bodySize -1;
			this->m_needToRecvData = false;
			return true;
		}
		else { // message is not yet completely in the message buffer
			if (this->m_bodySize > this->m_bufCapacity - IbeoDataHeader::getHeaderSize()) {
				// and it will not never fit into the buffer.
				logError << "Expected message size " << IbeoDataHeader::getHeaderSize()+this->m_bodySize
								 << " is larger than the available msg buffer size: " << this->m_bufCapacity
								 << " Message will be ignored." << std::endl;
				// since the expected size is bigger than the msg buffer, we
				// can assume all data be part of that message and ignore it,
				// means delete it.
				this->clearBufferCompletely();
				this->m_needToRecvData = true;
				return false;
			}
			else { // message will fit into the message buffer
				// will the rest space be enough?
				if (this->m_bufCapacity - this->m_insAt < this->m_bodySize) {
					// if not make room by deleting old data
					this->removeScheduledBytes();
				}

				this->m_missingBytes = this->m_bodySize - (this->m_insAt - this->m_bodyAt);
				this->m_processedTo = this->m_insAt-1;
				this->m_needToRecvData = true;
				return false;
			}
		}
	}
	else { // no valid data in MsgBuffer
		this->clearBufferPreserve3Bytes();
		this->m_needToRecvData = true;
	}
	return false;
}

//======================================================================

void MsgBuffer::clearBufferCompletely()
{
	this->m_insAt = 0;
	this->m_processedTo = -1;
	this->m_scheduledForDeletionTo = -1;
	this->m_dhAt = -1;
	this->m_bodyAt = -1;
	this->m_bodySize = -1;
	this->m_missingBytes = -1;
	this->m_needToRecvData = true;
}

//======================================================================

void MsgBuffer::clearBufferPreserve3Bytes()
{
	const int nbOfBytesInBuffer = this->m_insAt-this->m_processedTo -1;
	const int preserve = (nbOfBytesInBuffer > 3) ? 3 : nbOfBytesInBuffer;

	if (preserve == 0) {
		clearBufferCompletely();
		return;
	}
	// preserve > 0 here
	memmove(this->m_ptr, this->m_ptr + this->m_insAt - preserve, std::size_t(preserve));

	this->m_insAt = preserve;
	this->m_processedTo = -1;
	this->m_scheduledForDeletionTo = -1;
	this->m_dhAt = -1;
	this->m_bodyAt = -1;
	this->m_bodySize = -1;
	this->m_missingBytes = -1;
	this->m_needToRecvData = true;

	if (nbOfBytesInBuffer> preserve)
		logWarning << nbOfBytesInBuffer-preserve << " Bytes skipped" << std::endl;

}

//======================================================================

void MsgBuffer::removeScheduledBytes()
{
	if (this->m_scheduledForDeletionTo == -1) return;
	if (this->m_scheduledForDeletionTo >= this->m_bufCapacity
			|| this->m_insAt <= this->m_processedTo) {
		clearBufferCompletely();
		return;
	}

	memmove(this->m_ptr,
	        this->m_ptr+this->m_scheduledForDeletionTo,
	        std::size_t(this->m_insAt - this->m_scheduledForDeletionTo));

	MsgBuffer::reducePos(this->m_insAt, this->m_scheduledForDeletionTo, 0);
	MsgBuffer::reducePos(this->m_processedTo, this->m_scheduledForDeletionTo);
	MsgBuffer::reducePos(this->m_dhAt, this->m_scheduledForDeletionTo);
	MsgBuffer::reducePos(this->m_bodyAt, this->m_scheduledForDeletionTo);
	this->m_scheduledForDeletionTo = -1;
}

//======================================================================

// static
void MsgBuffer::reducePos(int& pos, const int nbOfBytes, const int lowerLimit)
{
	pos -= nbOfBytes;
	pos = (pos < lowerLimit) ? lowerLimit : pos;
}

//======================================================================

} // namespace ibeosdk

//======================================================================
