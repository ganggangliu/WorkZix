#ifndef __LMS100_SCANDATA_CONFIG_H__
#define __LMS100_SCANDATA_CONFIG_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "SopasBasicTypes.h"
#include "SopasXByte.h"

// Forward declarations
class CDeserializer;
class CBinaryDataStream;

/*======================================================================================*/
/*! \brief Scandata config structure for LMS100

    \ingroup lms100

    This structure represents the configuration of all fields and channels that are output 
    with a scan data telegram. All data members are public for easy access. Use the 
    Deserialize() function to fill this structure from a request's answer on your own. 
    CLms100SopasInterface::GetScanDataConfig() requests the configuration from a LMS100 and 
    fills this structure accordingly.

    See LMS100 documentation for detailed information about the data members of this
    structure.

    \attention Do not use memset on this structure because it contains #SOPAS_XByte members 
      in the embedded structs which keep structure information beside the variable's value. 
      This may not be overwritten! Use Clear() instead.
 */
/*======================================================================================*/
struct SLms100ScanDataConfig
{
  /*======================================================================================*/
  /* Part of SLms100ScanDataConfig, not documented further.                                         */
  /*======================================================================================*/
  struct SRemDataConfig
  {
    enum EDataType {REM_DATA_8BIT, REM_DATA_16BIT};
    enum EContentType {REM_CONT_DIGIT};

    SRemDataConfig(void);

    void Clear(void);
      //!< Sets all data members to zero
    bool SetDataTypeFromSopas(SOPAS_UInt8 usiDataType);
      //!< Casts number to according enum value and updates the value in this struct.
    bool SetContentTypeFromSopas(SOPAS_UInt8 usiContentType);
      //!< Casts number to according enum value and updates the value in this struct.

    SOPAS_Bool bEnable;  
    EDataType eDataType;
    EContentType eContentType;
  };

  SLms100ScanDataConfig(void);

  void Clear(void);
    //!< Sets all data members to zero
  bool Deserialize(CDeserializer& rDeserializer, CBinaryDataStream& rReceivedData);
    //!< \brief Deserialize variable read answer of <em>ScanDataConfig (LMDscandatacfg)</em> 
    //!< into this structure

  SOPAS_XByte DistDataConfig;
  SRemDataConfig RemDataConfig;
  SOPAS_XByte EnableEncoderBlock;
  SOPAS_Bool bEnablePositionBlock;
  SOPAS_Bool bEnableDeviceName;
  SOPAS_Bool bEnableCommentBlock;
  SOPAS_Bool bEnableTimeBlock;
  SOPAS_UInt16 uiOutputInterval;
};

#endif