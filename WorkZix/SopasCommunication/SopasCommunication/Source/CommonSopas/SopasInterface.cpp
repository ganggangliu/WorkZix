/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "SopasInterface.h"
#include "Protocol.h"
#include "Deserializer.h"
#include "Serializer.h"
#include "BinaryDataStream.h"

// --- Define version number of framework --- //
const std::string CSopasInterface::FRAMEWORK_COMMON_VERSION = "1.02";

/*======================================================================================*/
/*! The protocol also defines the framing and the communication channel to use
    (CFramer, CCommunication).

    \param rProtocol The protocol to use for communication.

    \attention The scope of \e rProtocol must last longer than
    the scope of this CSopasInterface instance.
 */
/*======================================================================================*/
CSopasInterface::CSopasInterface(CProtocol& rProtocol) : 
  m_Protocol(rProtocol)
{  
}

CSopasInterface::~CSopasInterface()
{
}

/*======================================================================================*/
/*! The common framework part is used by all devices. There are device specific 
    derivates of this class which have a version of their own. Use the according 
    GetDeviceFrameworkVersion() functions to get their version number.
    
    \note This number represents the version of the whole common part of this framework. 

    \return The version number of the common framework (which CSopasInterface is part of).
 */
/*======================================================================================*/
std::string CSopasInterface::GetFrameworkVersion(void)
{
  return FRAMEWORK_COMMON_VERSION;
}

/*======================================================================================*/
/*! The out parameter only contains valid data if this function returns <em>true</em>.
    See GetSopasErrorCode() to check for possible error conditions.

    The function reads the device variable <em>DeviceIdent</em>.

    \param[out] rName The name read from the device. 
    \param[out] rVersion The version string read from the device. 

    \return <em>true</em> if version string was received successfully. <em>false</em> otherwise.    
 */
/*======================================================================================*/  
bool CSopasInterface::GetVersionString(std::string& rName, std::string& rVersion)
{
  bool bRetVal = false;

  // Build request, send it and wait for answer
  // Variable: "DeviceIdent", index 0
  CBinaryDataStream* pReceivedData = m_Protocol.ReadVariable("DeviceIdent"); 
  if (pReceivedData != 0)
  {
    CDeserializer& rDeserializer = m_Protocol.GetDeserializer();
    bRetVal = rDeserializer.Deserialize(*pReceivedData, rName);
    bRetVal &= rDeserializer.Deserialize(*pReceivedData, rVersion);

    // All data should have been read, check it (optional)
    bRetVal &= pReceivedData->IsReadPosAtEnd();

    // All data processed, delete it
    delete pReceivedData;
  }

  return bRetVal;
}

/*======================================================================================*/
/*! \copydetails CCommunicationHandler::GetSopasErrorCode                               */
/*======================================================================================*/
SOPAS_UInt16 CSopasInterface::GetSopasErrorCode(void) const
{
  return m_Protocol.GetSopasErrorCode();
}

/*======================================================================================*/
/*! The user may not access variables which need a certain userlevel anymore afterwards.
    Use Login() to gain access to according userlevels. See GetSopasErrorCode() 
    to check for possible error conditions.

    The function calls the device method <em>Run</em>.

    \return <em>true</em> if logged out sucessfully. <em>false</em> otherwise.    
 */
/*======================================================================================*/  
bool CSopasInterface::Logout(void)
{
  bool bRetVal = false;

  // Build request, send it and wait for answer
  // Method: "Run", index 2
  CBinaryDataStream* pReceivedData = m_Protocol.InvokeMethod("Run", 0);
  if (pReceivedData != 0)
  {
    CDeserializer& rDeserializer = m_Protocol.GetDeserializer();
    SOPAS_Bool result;
    if (rDeserializer.Deserialize(*pReceivedData, result) == true)
    {
      bRetVal = (result == 1);
    }

    // All data should have been read, check it (optional)
    bRetVal &= pReceivedData->IsReadPosAtEnd();

    // All data processed, delete it
    delete pReceivedData;
  }

  return bRetVal;
}

/*======================================================================================*/
/*! Login to gain access to variables and methods that need a certain userlevel. Also
    see Logout().
    See GetSopasErrorCode() to check for possible error conditions.

    The function calls the device method <em>SetAccessMode</em>.

    \param level The userlevel including password hash to set.

    \return <em>true</em> if logged in successfully. <em>false</em> otherwise.    

    \note The login function also needs a password. The passwords for levels
    <em>RUN</em>, <em>MAINTENANCE</em> and <em>AUTHORIZED CLIENT</em> are supplied in the 
    SopasLoginLevel namespace. You can create an own global instance of CSopasLoginLevel 
    for the <em>SERVICE</em> level if you've got the according password hash. 
    Contact SICK to get the password hash. You also need to create an own instance if
    you changed the password for a certain login. An example is shown in the class
    description of CSopasLoginLevel.
 */
/*======================================================================================*/  
bool CSopasInterface::Login(const CSopasLoginLevel& level)
{
  bool bRetVal = false;

  // Calling sopas method:
  // SetAccessMode (0)
  // NewMode: Int8
  // Password (hash): UInt32
  // Return value: Bool
  SOPAS_Int8 txLevel = level.GetLevel();
  SOPAS_UInt32 txPassword = level.GetPasswordHash();

  // Create stream to store serialized parameters
  // Using exact stream size, stream may also be bigger
  CSerializer& rSerializer = m_Protocol.GetSerializer();  
  size_t minStreamSize = rSerializer.GetSerializedSize(txLevel) + rSerializer.GetSerializedSize(txPassword);
  CBinaryDataStream serializedParamStream(static_cast<unsigned int>(minStreamSize));

  // Serialize parameters into stream
  bool serialized = true;
  serialized &= rSerializer.Serialize(serializedParamStream, txLevel);
  serialized &= rSerializer.Serialize(serializedParamStream, txPassword);

  if (serialized == true)
  {    
    // Build request, send it and wait for answer
    // Method: "SetAccessMode", index 0
    CBinaryDataStream* pReceivedData = m_Protocol.InvokeMethod("SetAccessMode", &serializedParamStream);
    if (pReceivedData != 0)
    {
      CDeserializer& rDeserializer = m_Protocol.GetDeserializer();
      SOPAS_UInt8 result;
      if (rDeserializer.Deserialize(*pReceivedData, result) == true)
      {
        bRetVal = (result == 1);
      }

      // All data should have been read, check it (optional)
      bRetVal &= pReceivedData->IsReadPosAtEnd();

      // All data processed, delete it
      delete pReceivedData;
    }
  }

  return bRetVal;
}

/*======================================================================================*/
/*! \copydetails CCommunicationHandler::PollAsyncAnswers
 */
/*======================================================================================*/
void CSopasInterface::PollAsyncAnswers(int iPollTimeMs, CCommunicationHandler::SopasAsyncCallback pCallback)
{
  m_Protocol.PollAsyncAnswers(iPollTimeMs, pCallback);
}

/*======================================================================================*/
/*! \copydetails CCommunicationHandler::SetAsyncCallback
     (CCommunicationHandler::SopasAsyncCallback)
 */
/*======================================================================================*/
void CSopasInterface::SetAsyncCallback(CCommunicationHandler::SopasAsyncCallback pCallback, void* pUser/* = 0*/)
{
  m_Protocol.SetAsyncCallback(pCallback, pUser);
}

/*======================================================================================*/
/*! \copydetails CCommunicationHandler::SetAsyncCallback
     (CCommunicationHandler::IAsyncCallback)
 */
/*======================================================================================*/
void CSopasInterface::SetAsyncCallback(CCommunicationHandler::IAsyncCallback* pCallback)
{
  m_Protocol.SetAsyncCallback(pCallback);
}


/*======================================================================================*/
/*! Use the protocol to directly access variables or events not offered by the
    derived classes of CSopasInterface.

    \note It is best practise to implement device related functions in a derived class
      so that the protocol can be accessed directly.

    \return The protocol as passed by the constructor of this clas.
 */
/*======================================================================================*/ 
CProtocol& CSopasInterface::GetProtocol(void)
{
  return m_Protocol;
}
