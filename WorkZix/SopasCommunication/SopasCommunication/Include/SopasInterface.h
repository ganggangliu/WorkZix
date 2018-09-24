#ifndef __SOPAS_INTERFACE_H__
#define __SOPAS_INTERFACE_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "SopasBasicTypes.h"
#include "CommunicationHandler.h"
#include "SopasLoginLevel.h" // Make sure that namespace is available for all users of this class...
#include <string>

// Forward declarations
class CBinaryDataStream;
class CProtocol;

/*======================================================================================*/
/*! \defgroup common Common framework

    \brief Classes that provide basic functions usable by all SOPAS devices.    

    These classes provide basic functionality like communication, framing and protocols.
    Also some basic SOPAS commands like getting version numbers from a device are part
    of this. The \ref datatypes "data types" are also common for all devices.

    This framework part has a version number which can be queried by calling
    CSopasInterface::GetFrameworkVersion(). There are also device specific extensions 
    for this framework.    

 */
/*======================================================================================*/


/*======================================================================================*/
/*! \brief Provides basic functions common to all SICK devices

    \ingroup common
   
    This class provides some basic SOPAS functions like logging in to the device or 
    getting a version string. These functions are available for all SICK devices.
    Devices may inherit from this class to add device specific functions.

    All this functions basically call a SOPAS method or read a SOPAS variable from
    the device. The class uses the given CProtocol to communicate with the device.

    \note A connection to the device must have already been established before using any
      of this functions.
    \note It is best practise to encapsulate own functions that communicate with the 
      device into a class that derives from CSopasInterface or its descendants. That
      way the functions have access to the CProtocol reference stored inside of this class.      
 */
/*======================================================================================*/
class CSopasInterface
{
public:
  explicit CSopasInterface(CProtocol& rProtocol);
    //!< Constructor assigns a protocol to communicate with.
  virtual ~CSopasInterface(void);
    //!< Destructor deletes protocol.
  
  bool GetVersionString(std::string& rName, std::string& rVersion);
    //!< Gets name and version string from a device.
  bool Login(const CSopasLoginLevel& level);
    //!< Logs in to device using given level.
  bool Logout(void);
    //!< Logs out from device.

  SOPAS_UInt16 GetSopasErrorCode(void) const;
    //!< Get error code of last operation.
  CProtocol& GetProtocol(void);
    //!< Gets access to underlying protocol.

  void SetAsyncCallback(CCommunicationHandler::SopasAsyncCallback pCallback, void* pUser = 0);
    //!< \brief Sets callback function for asynchronous events or 
    //!< device method answers (function pointer).
  void SetAsyncCallback(CCommunicationHandler::IAsyncCallback* pCallback);
    //!< \brief Sets callback function for asynchronous events or 
    //!< device method answers (Implementing class).
  void PollAsyncAnswers(int iPollTimeMs, 
    CCommunicationHandler::SopasAsyncCallback pCallback = 0);
    //!< Poll for asynchronous events or device method answers.    

  static std::string GetFrameworkVersion(void);
    //!< Gets the version number of the common part of this framework
protected:
  CProtocol& m_Protocol; 
    //!< Protocol to use for communication.

private:
  CSopasInterface(const CSopasInterface&);
    //!< Do not allow copying
  CSopasInterface& operator=(const CSopasInterface&);
    //!< Do not allow copying

  static const std::string FRAMEWORK_COMMON_VERSION;
    //!< Framework version string of common part
};


#endif