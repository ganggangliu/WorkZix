/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "Lms100SopasInterface.h"
#include "Lms100Scan.h"
#include "Lms100ScanConfig.h"
#include "Lms100State.h"
#include "Lms100ScanDataConfig.h"
#include "Protocol.h"
#include "BinaryDataStream.h"
#include "Deserializer.h"
#include "Serializer.h"

// --- Define version number of LMS100 framework --- //
const std::string CLms100SopasInterface::FRAMEWORK_DEVICE_VERSION = "1.02";

/*======================================================================================*/
/*! \copydetails CSopasInterface::CSopasInterface(CProtocol&)                           */
/*======================================================================================*/
CLms100SopasInterface::CLms100SopasInterface(CProtocol& rProtocol) : 
  CSopasInterface(rProtocol)
{
}

CLms100SopasInterface::~CLms100SopasInterface(void)
{
}

/*======================================================================================*/
/*! To get the version of the underlying common part call 
    CSopasInterface::GetFrameworkVersion() instead.
    
    \note This number represents the version of the whole LMS100 specific part of 
      this framework. 

    \return The version number of the LMS100 framework.
 */
/*======================================================================================*/
std::string CLms100SopasInterface::GetDeviceFrameworkVersion(void)
{
  return FRAMEWORK_DEVICE_VERSION;
}

/*======================================================================================*/
/*! Reads one scan from connected LMS100 and decodes it.
    The received scandata is written to the paramter <em>rData</em>. The parameter
    is not cleared before but overwritten instead.

    The function reads the device variable <em>ScanData (LMDscandata)</em>.

    \note The scandata contains \link SOPAS_FlexArray SOPAS_FlexArrays \endlink
      which are used for optional parameters. The data is not cleared so array data is 
      only valid for elements according to the array size. Any index above the current 
      size contains invalid data.

    \param[out] rData The scandata received. This structure is filled accordingly and
      only contains valid data if this function returns <em>true</em>.

    \return <em>true</em> if scandata could be received and decoded successfully. 
      <em>false</em> otherwise.
 */
/*======================================================================================*/
bool CLms100SopasInterface::GetScanData(SLms100Scan& rData)
{
  bool bRetVal = false;

  // Scandata could be cleared before but instead is just overwritten    
  // Build request
  CBinaryDataStream* pReceivedData = m_Protocol.ReadVariable("LMDscandata");

  // Process answer. 
  if(pReceivedData != 0)
  {
    bRetVal = rData.Deserialize(m_Protocol.GetDeserializer(), *pReceivedData);
  }

  // All data processed, delete it
  delete pReceivedData;

  return bRetVal;
}

/*======================================================================================*/
/*! Reads scandata config from connected LMS100 and decodes it.
    The received scandata config is written to the paramter <em>rData</em>. The parameter
    is not cleared before but overwritten instead.

    The function reads the device variable <em>ScanDataConfig (LMDscandatacfg)</em>.

    \param[out] rData The scandata config received. This structure is filled accordingly 
      and only contains valid data if this function returns <em>true</em>.

    \return <em>true</em> if scandata config could be received and decoded successfully. 
      <em>false</em> otherwise.
 */
/*======================================================================================*/
bool CLms100SopasInterface::GetScanDataConfig(SLms100ScanDataConfig& rData)
{
  bool bRetVal = false;

  // Scandata could be cleared before but instead is just overwritten    
  // Build request
  CBinaryDataStream* pReceivedData = m_Protocol.ReadVariable("LMDscandatacfg");

  // Process answer. 
  if(pReceivedData != 0)
  {
    bRetVal = rData.Deserialize(m_Protocol.GetDeserializer(), *pReceivedData);
  }

  // All data processed, delete it
  delete pReceivedData;

  return bRetVal;
}

/*======================================================================================*/
/*! See documentation of LMS100 for details. The called device method has a return value
    which is mapped to the according enum #EMeasureError. 

    The function calls the device method <em>mStartMeasure (LMCstartmeas)</em>.

    \param[out] pReturn The return value of the device method, mapped to an enum.
      Only contains valid data if this function returns <em>true</em>. Parameter
      is optional, may be null.

    \return <em>true</em> if the method could be called and the return value
      could be decoded successfully. <em>false</em> otherwise.
 */
/*======================================================================================*/
bool CLms100SopasInterface::StartMeasure(EMeasureError* pReturn)
{
  bool bRetVal = false;

  // Build request. Method has no parameters
  CBinaryDataStream* pReceivedData = m_Protocol.InvokeMethod("LMCstartmeas", 0);
  if (pReceivedData != 0)
  {
    // Get return value of method
    CDeserializer& rDeserializer = m_Protocol.GetDeserializer();
    SOPAS_UInt8 result;
    if (rDeserializer.Deserialize(*pReceivedData, result) == true)
    {
      if((result == SOPAS_ERR_NO_ERR) || (result == SOPAS_ERR_STATE_CHANGE_NOT_PERMITTED))
      {
        bRetVal = true;
        // Set SOPAS return value (optional)
        if(pReturn != 0)
        {
          *pReturn = static_cast<EMeasureError>(result);
        }
      }
    }

    // All data should have been read, check it
    bRetVal &= pReceivedData->IsReadPosAtEnd();

    // All data processed, delete it
    delete pReceivedData;
  }

  return bRetVal;
}

/*======================================================================================*/
/*! See documentation of LMS100 for details. The called device method has a return value
    which is mapped to the according enum #EMeasureError. 

    The function calls the device method <em>mStopMeasure (LMCstopmeas)</em>.

    \param[out] pReturn The return value of the device method, mapped to an enum.
      Only contains valid data if this function returns <em>true</em>. Parameter
      is optional, may be null.

    \return <em>true</em> if the method could be called and the return value
      could be decoded successfully. <em>false</em> otherwise.
 */
/*======================================================================================*/
bool CLms100SopasInterface::StopMeasure(EMeasureError* pReturn)
{
  bool bRetVal = false;

  // Build request
  CBinaryDataStream* pReceivedData = m_Protocol.InvokeMethod("LMCstopmeas", 0);
  if (pReceivedData != 0)
  {
    // Get return value of method
    CDeserializer& rDeserializer = m_Protocol.GetDeserializer();
    SOPAS_UInt8 result;
    if (rDeserializer.Deserialize(*pReceivedData, result) == true)
    {
      // Convert result to enum
      if((result == SOPAS_ERR_NO_ERR) || (result == SOPAS_ERR_STATE_CHANGE_NOT_PERMITTED))
      {
        bRetVal = true;
        // Set SOPAS return value (optional)
        if(pReturn != 0)
        {
          *pReturn = static_cast<EMeasureError>(result);
        }
      }
    }

    // All data should have been read, check it
    bRetVal &= pReceivedData->IsReadPosAtEnd();

    // All data processed, delete it
    delete pReceivedData;
  }

  return bRetVal;
}

/*======================================================================================*/
/*! See documentation of LMS100 for details. On success the parameter <em>rState</em>
    contains the current device state.

    The function reads the device variable <em>DeviceStateLMS (STlms)</em>.

    \param[out] rState The state of the LMS100.
      Only contains valid data if this function returns <em>true</em>.

    \return <em>true</em> if the variable could be read successfully. 
      <em>false</em> otherwise.
 */
/*======================================================================================*/
bool CLms100SopasInterface::GetState(SLms100State& rState)
{
  bool bRetVal = false;

  // state could be cleared before but instead is just overwritten    
  // Build request
  CBinaryDataStream* pReceivedData = m_Protocol.ReadVariable("STlms");

  // Process answer.
  if(pReceivedData != 0)
  {
    bRetVal = rState.Deserialize(m_Protocol.GetDeserializer(), *pReceivedData);
  }

  // All data processed, delete it
  delete pReceivedData;

  return bRetVal;
}

/*======================================================================================*/
/*! See documentation of LMS100 for details. This function cannot be called in any 
    operating mode of the LMS100 and additional steps might be necessary to activate the
    new settings. Invalid parameter combinations are not checked here but will lead
    to an according return value of the device method (<em>pErrorCode</em>).

    The functions calls the device method <em>SetScanConfig (mLMPsetscancfg)</em>.

    \note The function returns the scanconfig of the device. This is not compared here 
      instead just the error code is returned.

    \param udiScanFrequency Desired scan frequency in 1/100Hz.
    \param udiAngleResolution Desired angular resolution in 1/10000 degree.
    \param iStartAngle Desired start angle of measurement in 1/10000 degree.
    \param iStopAngle Desired stop angle of measurement in 1/10000 degree.
    \param[out] pErrorCode The error code returned from the device method. Check it
      in case invalid parameter combinations were used. Parameter is optional, 
      may be \e null. Only contains valid data if this function returns <em>true</em>.

    \return <em>true</em> if the method could be called and the return value
      could be decoded successfully. <em>false</em> otherwise.
 */
/*======================================================================================*/
bool CLms100SopasInterface::SetScanConfig(SOPAS_UInt32 udiScanFrequency, SOPAS_UInt32 udiAngleResolution, 
                                          SOPAS_Int32 iStartAngle, SOPAS_Int32 iStopAngle, EScanConfigError* pErrorCode)
{
  bool bRetVal = false;

  // Create stream to store serialized method parameters
  // Using exact stream size here, stream may also be bigger
  SLms100ScanConfig soConfig(udiScanFrequency, udiAngleResolution, iStartAngle, iStopAngle);
  
  CSerializer& rSerializer = m_Protocol.GetSerializer(); 
  size_t ulMinStreamSize = soConfig.GetSerializedSize(rSerializer);
  CBinaryDataStream serializedParamStream(static_cast<unsigned int>(ulMinStreamSize));

  if(soConfig.Serialize(rSerializer, serializedParamStream) == true)
  {
    // Build request. Add serialized parameters
    CBinaryDataStream* pReceivedData = m_Protocol.InvokeMethod("mLMPsetscancfg", &serializedParamStream);
    if (pReceivedData != 0)
    {
      // Get return value of method
      CDeserializer& rDeserializer = m_Protocol.GetDeserializer();
      SOPAS_UInt8 eScanConfigError;
      if (rDeserializer.Deserialize(*pReceivedData, eScanConfigError) == true)
      {
        // Set error code when transmission was successful (optional)
        if(pErrorCode != 0)
        {
          *pErrorCode = static_cast<EScanConfigError>(eScanConfigError);
        }

        // The new scanconfig is returned
        // Just check, that data is present. Do not compare it here
        SLms100ScanConfig soResultConfig;
        bRetVal = soResultConfig.Deserialize(rDeserializer, *pReceivedData);
      }

      // All data should have been read, check it
      bRetVal &= pReceivedData->IsReadPosAtEnd();

      // All data processed, delete it
      delete pReceivedData;
    }
  }

  return bRetVal;
}

/*======================================================================================*/
/*! The scandata must be received by calling PollAsyncAnswers(). The registered callback
    is called (see SetAsyncCallback()).

    \note In the callback check that the type SDecodedAnswer::eAnswerType is 
      #SDecodedAnswer::EVENT_RESULT and SDecodedAnswer::coName must be "LMDscandata".
    
    \param bSubscribe \e true if event should be subscribed, \e false if event should
      be unsubscribed.
    
    \return \e true if the event subscription was successful. \e false otherwise.
 */
/*======================================================================================*/
bool CLms100SopasInterface::SubscribeScanDataEvent(bool bSubscribe)
{
  return m_Protocol.SubscribeEvent("LMDscandata", bSubscribe);
}