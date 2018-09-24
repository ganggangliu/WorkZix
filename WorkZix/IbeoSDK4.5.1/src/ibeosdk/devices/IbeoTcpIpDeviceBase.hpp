//======================================================================
/*! \file IbeoTcpIpDeviceBase.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Jan 29, 2014
 *///-------------------------------------------------------------------
#ifndef IBEOSDK_IBEOTCPIPDEVICEBASE_HPP_SEEN
#define IBEOSDK_IBEOTCPIPDEVICEBASE_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/devices/IbeoDeviceBase.hpp>

#include <boost/asio.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!\class IbeoTcpIpDeviceBase
 * \brief Base class for all Ibeo devices connected via TCP/IP.
 * \author Mario Brumm (mb)
 * \version 0.1
 * \date Apr 24, 2012
 *///-------------------------------------------------------------------
class IbeoTcpIpDeviceBase : public IbeoDeviceBase {
public:
	typedef CommandReplyBase* CommandReplyBasePtr;
	typedef boost::recursive_timed_mutex RecursiveMutex;

protected:
	//========================================
	/*!\brief Create an IbeoScanner.
	 *
	 * The constructor will be called by derived
	 * scanner classes also.
	 *
	 * \param[in] ip    IP address of the scanner
	 *                  to be connected with.
	 * \param[in] port  Port number for the connection
	 *                  with the scanner.
	 *///-------------------------------------
	IbeoTcpIpDeviceBase(const std::string& ip, const unsigned short port = 12002);

public:
	//========================================
	/*!\brief Destrutor of the IbeoDevice class.
	 *
	 * Stopping the receiving thread and
	 * disconnecting the hardware device.
	 *///-------------------------------------
	virtual ~IbeoTcpIpDeviceBase();

private:
	IbeoTcpIpDeviceBase(const IbeoTcpIpDeviceBase&); // forbidden
	IbeoTcpIpDeviceBase& operator=(const IbeoTcpIpDeviceBase&); // forbidden

public:
	//========================================
	/*!\brief Establish the connection to the
	 *        hardware.
	 *
	 * Starting the receiving thread.
	 *///-------------------------------------
	virtual void getConnected();

	//========================================
	/*!\brief Disconnect the TCP/IP connection
	 *        to the hardware device.
	 *///-------------------------------------
	virtual void disconnect();

	//========================================
	/*!\brief Checks whether the TCP/IP connection
	 *        to the hardware device is there and
	 *        the receiving thread is still running.
	 * \return \c true if messages from the hardware
	 *         can still be received. \c false otherwise.
	 *///-------------------------------------
	virtual bool isConnected();

	//========================================
	/*!\brief Main function of the receive thread.
	 *///-------------------------------------
	virtual void dataReceiveThread();

public:
	//========================================
	/*!\brief Send a command which expects no reply.
	 * \param[in] cmd  Command to be sent.
	 * \return The result of the operation.
	 * \sa ErrorCode
	 *///-------------------------------------
	virtual statuscodes::Codes sendCommand(const CommandBase& cmd);

	//========================================
	/*!\brief Send a command and wait for a reply.
	 *
	 * The command will be sent. The calling thread
	 * will sleep until a reply has been received
	 * but not longer than the number of milliseconds
	 * given in \a timeOut.
	 *
	 * \param[in]       cmd    Command to be sent.
	 * \param[in, out]  reply  The reply container for
	 *                         the reply to be stored into.
	 * \param[in]       timeOut  Number of milliseconds to
	 *                           wait for a reply.
	 * \return The result of the operation.
	 * \sa ErrorCode
	 *///-------------------------------------
	virtual statuscodes::Codes sendCommand(const CommandBase& cmd,
	                                       CommandReplyBase& reply,
	                                       const boost::posix_time::time_duration timeOut = boost::posix_time::milliseconds(500));

protected:
	//========================================
	/*!\brief Send the command to the device.
	 * \param[in] command    Command to be sent.
	 * \return The result of the operation.
	 * \sa ErrorCode
	 *///-------------------------------------
	statuscodes::Codes sendCommandInternal(const CommandBase& command);

	//========================================
	/*!\brief Handle the reception of a command reply and signal
	 *        the reception to the sending thread.
	 * \param[in] dh       Header that came along with that
	 *                     CommandReply DataBlock.
	 * \param[in] bodyBuf  Buffer that contains the
	 *                     (still serialized) body of
	 *                     the received CommandReply DataBlock.
	 *///-------------------------------------
	virtual void onReceiveCommandReply(const IbeoDataHeader& dh, const char* bodyBuf);

protected:
	static const int msgBufferSize = 4*65536; // 4*64K

protected:
	//========================================
	/*!\brief TCP/IP address of the device as string.
	 *///-------------------------------------
	std::string m_strIP;
	//========================================
	/*!\brief Port number of the device.
	 *///-------------------------------------
	unsigned short m_port;

	boost::asio::io_service m_ioService;
	boost::asio::ip::tcp::socket* m_socket;

	//RecursiveMutex m_rMutex;
	boost::recursive_timed_mutex m_rMutex;

	CommandReplyBasePtr m_replyPtr;
	CommandId m_expectedReplyId;
	EventMonitor::Mask m_eventReplyReceived;
	uint32_t m_sizeOfPrevCommand;
}; // IbeoEthernetDeviceBase

//======================================================================

} // namespace ibeosdk

//======================================================================

#endif // IBEOSDK_IBEOTCPIPDEVICEBASE_HPP_SEEN

//======================================================================
