#ifndef __LMS100_INTERFACE_H__
#define __LMS100_INTERFACE_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "SopasInterface.h"

// Forward declarations
struct SLms100Scan;
struct SLms100State;
struct SLms100ScanDataConfig;

/*======================================================================================*/
/*! \defgroup lms100 LMS100 framework

    \brief Classes that provide basic functions usable by the LMS100.    

    These classes provide basic functionality like getting scandata and device state
    from a LMS100. They extend the \ref common "common framework".

    This framework part has a version number which can be queried by calling
    CLms100SopasInterface::GetDeviceFrameworkVersion(). 

 */
/*======================================================================================*/


/*======================================================================================*/
/*! \brief Provides basic functions for LMS100

    \ingroup lms100

    This class extends the CSopasInterface (which provides functions like logging in
    and getting a version string) by adding some general functions supported by the
    LMS100 device. This are functions like getting scan data, setting the scan 
    configuration and getting the current state of the LMS100.

    Because this class inherits from CSopasInterface it also provides the basic functions
    common to all SOPAS devices.
 */
/*======================================================================================*/
class CLms100SopasInterface : public CSopasInterface
{
public:
  explicit CLms100SopasInterface(CProtocol& rProtocol);
    //!< Constructor assigns a protocol to communicate with.
  virtual ~CLms100SopasInterface(void);
    //!< Destructor deletes protocol.

  bool GetScanData(SLms100Scan& pData);
    //!< Reads one scan from the LMS100.
  bool GetScanDataConfig(SLms100ScanDataConfig& rData);
    //!< Gets scandata config from the LMS100.
  bool GetState(SLms100State& pState);
    //!< Get the current state of the LMS100.

  bool SubscribeScanDataEvent(bool bSubscribe);
    //!< Subscribe to or unsubscribe from scandata events (asynchronous)

  //! Device return value of method call for StartMeasure() and StopMeasure()
  enum EMeasureError
  {
    SOPAS_ERR_NO_ERR = 0,
    SOPAS_ERR_STATE_CHANGE_NOT_PERMITTED
  };

  //! Device return value of method call for SetScanConfig()
  enum EScanConfigError 
  {
    SCE_OK = 0,
    SCE_ERROR_FREQ,
    SCE_ERROR_RES,
    SCE_ERROR_FREQ_RES_COMB,
    SCE_ERROR_NUM_RANGE,
    SCE_ERROR
  };

  bool StartMeasure(EMeasureError* pReturn);
    //!< Starts measuring mode of LMS100.
  bool StopMeasure(EMeasureError* pReturn);
    //!< Stops measuring mode of LMS100.

  bool SetScanConfig(SOPAS_UInt32 udiScanFrequency, SOPAS_UInt32 udiAngleResolution, 
    SOPAS_Int32 iStartAngle, SOPAS_Int32 iStopAngle, EScanConfigError* pErrorCode);
    //!< Sets parameters for scan operation.

  static std::string GetDeviceFrameworkVersion(void);
    //!< Gets the version number of the LMS100 part of this framework

private:
  CLms100SopasInterface(const CLms100SopasInterface&);
    //!< Do not allow copying
  CLms100SopasInterface& operator=(const CLms100SopasInterface&);
    //!< Do not allow copying

  static const std::string FRAMEWORK_DEVICE_VERSION;
    //!< Framework version string of LMS100 part
};

#endif