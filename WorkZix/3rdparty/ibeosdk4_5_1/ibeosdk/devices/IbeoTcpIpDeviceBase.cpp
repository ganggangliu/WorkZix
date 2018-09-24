//======================================================================
/*! \file IbeoTcpIpDeviceBase.cpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Jan 29, 2014
 *///-------------------------------------------------------------------
//======================================================================

#include <ibeosdk/devices/IbeoTcpIpDeviceBase.hpp>

#include <ibeosdk/inttypes.hpp>
#include <ibeosdk/MsgBuffer.hpp>
#include <ibeosdk/Log.hpp>
#include <ibeosdk/EventMonitor.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iomanip>

//======================================================================

namespace ibeosdk {

//======================================================================

IbeoTcpIpDeviceBase::IbeoTcpIpDeviceBase(const std::string& ip, const unsigned short port)
  : m_strIP(ip),
    m_port(port),
    m_ioService(),
    m_socket(NULL),
	m_replyPtr(NULL),
	m_expectedReplyId(CommandId(CommandId::CmdLuxReset)),
	m_eventReplyReceived(m_eventMonitor.newEvent()),
	m_sizeOfPrevCommand(0)
{}

//======================================================================

IbeoTcpIpDeviceBase::~IbeoTcpIpDeviceBase()
{
	this->disconnect();
}

//======================================================================

void IbeoTcpIpDeviceBase::getConnected()
{
	if (m_thread) return;

	Lock lock(m_mutex);
	m_thread = new boost::thread(&IbeoTcpIpDeviceBase::dataReceiveThread, this);

	while ((this->m_threadState==TS_NotRunning || this->m_threadState==TS_Starting)) {
		m_recvCondition.wait(lock);
	}
}

//======================================================================

void IbeoTcpIpDeviceBase::disconnect()
{
	IbeoDeviceBase::disconnect();

	if (m_socket != NULL) {
		m_socket->close();
		delete m_socket;
		m_socket = NULL;
	}
}

//======================================================================

bool IbeoTcpIpDeviceBase::isConnected()
{
	if (m_socket == NULL)
		return false;

	if (!m_socket->is_open())
		return false;

	return IbeoDeviceBase::isConnected();
}

//======================================================================

void IbeoTcpIpDeviceBase::dataReceiveThread()
{
	{
		Lock lock(this->m_mutex);
		m_threadState = TS_Starting;
	}

	try {
		m_socket = new boost::asio::ip::tcp::socket(m_ioService);
	}
	catch (const std::exception& e) {
		logError << "Failed to allocate socket. - "
				<< e.what() << std::endl;
		Lock lock(this->m_mutex);
		m_threadState = TS_StartFailed;
		m_recvCondition.notify_all();
		return;
	}

	boost::system::error_code ec;
	boost::asio::ip::address ipAdr = boost::asio::ip::address::from_string(m_strIP, ec);
	if (ec) {
		logError << "Failed to connect to device (" << m_strIP << ") " << ec.message() << std::endl;
		Lock lock(this->m_mutex);
		m_threadState = TS_StartFailed;
		m_recvCondition.notify_all();
		return;
	}

	boost::asio::ip::tcp::endpoint ep(ipAdr, m_port);
	m_socket->connect(ep, ec);

	if (ec) {
		logError << "Failed to connect to device (" << ep << ") " << ec.message() << std::endl;
		Lock lock(this->m_mutex);
		m_threadState = TS_StartFailed;
		m_recvCondition.notify_all();
		return;
	}

	logDebug << "Connected to device on " << ep << std::endl;

	try {
		boost::asio::ip::tcp::no_delay option(true);
		m_socket->set_option(option);
	}
	catch (const std::exception& e) {
		logError << "Failed to allocate socket. - "
				<< e.what() << std::endl;
		Lock lock(this->m_mutex);
		m_threadState = TS_StartFailed;
		m_recvCondition.notify_all();
		return;
	}


	boost::system::error_code error;
	const char* bodyBuf;
	const IbeoDataHeader* dh;
	ibeosdk::MsgBuffer msgBuf(this->m_socket, msgBufferSize);

	{
		Lock lock(this->m_mutex);
		m_threadState = TS_Running;
		m_recvCondition.notify_all();
	}

	try {
		while (this->m_threadState != TS_Stopping && m_socket->is_open()) {
			if (msgBuf.recvMsg(dh, bodyBuf)) {
				logDebug2 << "Received DataBlock of type " << toHex(dh->getDataType()) << std::dec << std::endl;

				if ((dh->getDataType() == DataTypeId::DataType_Reply)) {
					onReceiveCommandReply(*dh, bodyBuf);
					continue; // do not inform any streamer or listener
				}

				if (!m_streamers.empty()) {
					// Message has been completely received. So, we can notify all streamers
					notifyStreamers(*dh, bodyBuf);

				}
				if (!m_listeners.empty()) {
					// Message has been completely received. So, we can parse it and notify all clients
					boost::iostreams::stream<boost::iostreams::array_source> strm(bodyBuf, dh->getMessageSize());
					std::istream& s = (std::istream&)strm;
					notifyListeners(onData(*dh, s));
				}
			}
		}
	}
	catch (const boost::system::error_code& errorCode) {
		logError << " After Error is socket open?: " << this->m_socket->is_open() << " " << errorCode.message() << std::endl;
		m_threadState = TS_RunFailed;
		m_recvCondition.notify_all();
	}

	if (!m_socket->is_open() && this->m_threadState != TS_Stopping) {
		Lock lock(this->m_mutex);
		m_threadState = TS_RunFailed;
		m_recvCondition.notify_all();
	}
}

//======================================================================

void IbeoTcpIpDeviceBase::onReceiveCommandReply(const IbeoDataHeader& dh, const char* bodyBuf)
{
	{
		const uint8_t* bb = reinterpret_cast<const uint8_t*>(bodyBuf);
		logDebug << "INCOMMING REPLY: " << toHex(uint16_t(*(bb+1) + ((*(bb))<<8)))
				<< "  Size: " << dh.getMessageSize() << std::endl;
	}

	if (m_replyPtr != NULL) {
		boost::iostreams::stream<boost::iostreams::array_source> strm(bodyBuf, dh.getMessageSize());
		std::istream& s = (std::istream&)strm;
		if (m_replyPtr->deserializeFromStream(s, dh)) {
			logDebug2 << "Received Reply -- good" << std::endl;
		}
		else {
			logDebug2 << "Received Reply -- failed" << std::endl;
			if (m_replyPtr)
				m_replyPtr->setErrorReply();
		}

		// Wake up main thread waiting in command()
		m_eventMonitor.signal(m_eventReplyReceived);
	}
	else {
		logDebug2 << "INCOMMING REPLY UNEXPECTED " << std::endl;
	}
}

//======================================================================

statuscodes::Codes IbeoTcpIpDeviceBase::sendCommand(const CommandBase& cmd)
{
	if (!isConnected()) {
		// cannot send a command if the device is not connected.
		logError << " command (id " << toHex(cmd.getCommandId()) << ") failed " << statuscodes::NotConnected << std::endl;
		return statuscodes::NotConnected;
	}

	// Send command
	logDebug2 << "Sending " << " command (id " << toHex(cmd.getCommandId()) << ") " << "; expecting no reply" << std::endl;

	const statuscodes::Codes errorCode = sendCommandInternal(cmd);
	logDebug2 << "Done. " << errorCode << std::endl;

	return errorCode;
}

//======================================================================

statuscodes::Codes IbeoTcpIpDeviceBase::sendCommand(const CommandBase& cmd,
                                                    CommandReplyBase& reply,
                                                    const boost::posix_time::time_duration timeOut)
{
	if (!isConnected()) {
		// cannot send a command if the device is not connected.
		logError << " command (id " << toHex(cmd.getCommandId()) << ") failed  Reply: " << toHex(reply.getCommandId()) << " " << statuscodes::NotConnected << std::endl;
		return statuscodes::NotConnected;
	}

	if (cmd.getCommandId() != reply.getCommandId()) {
		logError << " command (id " << toHex(cmd.getCommandId()) << ") failed  Reply: " << toHex(reply.getCommandId()) << " " << statuscodes::MismatchingCommandAndReply << std::endl;
		return statuscodes::MismatchingCommandAndReply;
	}

	{
		// Pass reply buffer to onReceiveCommandReply() which is running
		// in the reception thread
		boost::recursive_timed_mutex::scoped_lock criticalSection(m_rMutex);
		assert (!m_replyPtr); // previous reply must have been completely processed
		m_replyPtr = &reply;
		m_expectedReplyId = cmd.getCommandId();
	}

	statuscodes::Codes errorCode = statuscodes::EverythingOk;
	try {
		// Send command
		logDebug2 << "Sending " << " command (id " << toHex(cmd.getCommandId()) << ")" << std::endl;

		if (sendCommandInternal(cmd) != statuscodes::EverythingOk)
			throw statuscodes::SendingCommandFailed;

		// Wait for reply. The event m_eventReplyReceived will be signaled by
		EventMonitor::Mask event = m_eventMonitor.wait(timeOut, m_eventReplyReceived);

		if (event == EventMonitor::TimeOutEvent)
			throw statuscodes::TimeOut;
		else if ((cmd.getCommandId() | CommandReplyBase::errorMask)!= (reply.getCommandId() | CommandReplyBase::errorMask))
			throw statuscodes::ReplyMismatch;
		else if (reply.isErrorReply())
			throw statuscodes::ReceiveCommandErrorReply;
	} // try
	catch (const statuscodes::Codes& ec) {
		errorCode = ec;
		logError << " command (id " << toHex(cmd.getCommandId()) << ") failed  Reply: " << toHex(reply.getCommandId()) << " " << ec << std::endl;
	} // catch

	{
		// Clear reply buffer for reception thread
		boost::recursive_timed_mutex::scoped_lock criticalSection(m_rMutex);
		assert (m_replyPtr);
		m_replyPtr = NULL;
		m_expectedReplyId = CommandId(0x7fff);
	}

	return errorCode;
}

//======================================================================

statuscodes::Codes  IbeoTcpIpDeviceBase::sendCommandInternal(const CommandBase& cmd)
{
	boost::posix_time::ptime timeout = boost::posix_time::microsec_clock::universal_time() + boost::posix_time::milliseconds(50);
	boost::recursive_timed_mutex::scoped_timed_lock criticalSection(m_rMutex, timeout);

	if (criticalSection) { // if we have got thread-safe access in time
		if (isConnected()) {
			const uint32_t cmdLength = uint32_t(cmd.getCommandLength());
			const uint32_t sizeOfThisMsg = cmdLength;
			const uint32_t sizeOfBuffer = sizeOfThisMsg + IbeoDataHeader::getHeaderSize();

			boost::scoped_ptr<char> sendBuf(new char[sizeOfBuffer]);
			boost::iostreams::stream<boost::iostreams::array> strm(sendBuf.get(), sizeOfBuffer);
			std::ostream& s = (std::ostream&)strm;

			IbeoDataHeader header(DataTypeId::DataType_Command,
			                      m_sizeOfPrevCommand,
			                      sizeOfThisMsg,
			                      1,
			                      NTPTime());

			logDebug2 << "Sending command with " <<  sizeOfThisMsg << " bytes"<< std::endl;

			header.serialize(s);
			cmd.serializeToStream(s);
			if (!s.good())
				return statuscodes::FailedToPrepareSendingCommand;

			const size_t nbOfBytesSend = m_socket->send(boost::asio::buffer(sendBuf.get(), sizeOfBuffer));
			if (nbOfBytesSend != size_t(sizeOfBuffer)) {
				logError << "Send " << nbOfBytesSend << " expected: " << sizeOfBuffer << std::endl;

				return statuscodes::SendingCommandFailed;
			}

			m_sizeOfPrevCommand = uint32_t(sizeOfThisMsg);

			return statuscodes::EverythingOk;
		}
		return statuscodes::NotConnected;
	}

	logError << "Timeout while locking critical section" << std::endl;
	return statuscodes::TimeOutCriticalSection;
}
//======================================================================

}// namespace ibeosdk

//======================================================================
