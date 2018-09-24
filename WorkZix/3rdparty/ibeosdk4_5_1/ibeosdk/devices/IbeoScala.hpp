//======================================================================
/*! \file IbeoScala.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer
 * \date Oct 04, 2013
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_IBEOSCALA_HPP_SEEN
#define IBEOSDK_IBEOSCALA_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/devices/IbeoDevice.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!\class IbeoScala
 * \brief Class to connect to a Scala sensor.
 * \author Jan Christian Dittmer (jcd)
 * \version 0.1
 * \date Oct 1, 2013
 *
 *///-------------------------------------------------------------------
class IbeoScala : public IbeoDevice<IbeoScala> {
public:
	//========================================
	/*!\brief Create an IbeoScala (connection class).
	 *
	 * This constructor will create an IbeoLUX class object
	 * which will try to connect to a Scala sensor,
	 * using the given IP address and port number.
	 *
	 * \param[in] ip    IP address of the scanner
	 *                  to be connected with.
	 * \param[in] port  Port number for the connection
	 *                  with the scanner.
	 *///-------------------------------------
	IbeoScala(const std::string& ip, const unsigned short port = 12004);

	//========================================
	/*!\brief Destructor.
	 *
	 * Will disconnect before destruction.
	 *///-------------------------------------
	virtual ~IbeoScala();


private: // not supported
	using IbeoDevice<IbeoScala>::sendCommand;
}; // IbeoScala

//======================================================================

} // namespace ibeosdk

//======================================================================

#endif // IBEOSDK_IBEOSCALA_HPP_SEEN

//======================================================================

