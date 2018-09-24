#ifndef __LMS100_SCAN_H__
#define __LMS100_SCAN_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "SopasBasicTypes.h"
#include "SopasFlexArray.h"
#include "SopasXByte.h"
#include "SopasString.h"

// Forward declarations
class CDeserializer;
class CBinaryDataStream;

/*======================================================================================*/
/* Part of SLms100Scan, not documented further.                                         */
/*======================================================================================*/
struct SDeviceBlock
{
  SDeviceBlock(void);

  void Clear(void);

  SOPAS_UInt16 uiIdent;
  SOPAS_UInt32 udiSerialNo;
  SOPAS_XByte xbState; // XByte: Internal bit structure has to be defined separatedly (see c'tor)
};

/*======================================================================================*/
/* Part of SLms100Scan, not documented further.                                         */
/*======================================================================================*/
struct SStatusBlock
{
  SStatusBlock(void);

  void Clear(void);

  SOPAS_UInt16 uiTelegramCount;
  SOPAS_UInt16 uiScanCount;
  SOPAS_UInt32 udiSystemCountScan;
  SOPAS_UInt32 udiSystemCountTransmit;
  SOPAS_XByte xbInputs;  // XByte: Internal bit structure has to be defined separatedly (see c'tor)
  SOPAS_XByte xbOutputs; // XByte: Internal bit structure has to be defined separatedly (see c'tor) 
  SOPAS_UInt16 uiReserved;
};

/*======================================================================================*/
/* Part of SLms100Scan, not documented further.                                         */
/*======================================================================================*/
struct SMeasurementParam1Block
{
  SMeasurementParam1Block(void);

  void Clear(void);

  SOPAS_UInt32 udiScanFreq;
  SOPAS_UInt32 udiMeasFreq;
};

/*======================================================================================*/
/* Part of SLms100Scan, not documented further.                                         */
/*======================================================================================*/
struct SEncoderBlock
{
  SEncoderBlock(void);

  void Clear(void);

  // Scandata format of LMS100 has changed. For now support both versions with the new one
  // as default.
#ifndef LMS100_OLD_SCANDATA 
  SOPAS_UInt32 udiEncoderPos;
#else
  SOPAS_UInt16 udiEncoderPos;
#endif
  SOPAS_Int16 iEncoderSpeed;
};

/*======================================================================================*/
/* Part of SLms100Scan, not documented further.                                         */
/*======================================================================================*/
struct SDataChannelHdr
{
  static const int MAX_MEAS_DATA = 1082;

  SDataChannelHdr(void);

  void Clear(void);

  SOPAS_FixString aContentType; // Fix length: 4
  SOPAS_Float dScaleFactor;
  SOPAS_Float dScaleOffset;
  SOPAS_Int32 diStartAngle;
  SOPAS_UInt16 uiAngleRes;
};

/*======================================================================================*/
/* Part of SLms100Scan, not documented further.                                         */
/*======================================================================================*/
struct SDataChannel16
{
  void Clear(void);

  SDataChannelHdr DataChannelHdr;
  
  SOPAS_FlexArray<SOPAS_UInt16, SDataChannelHdr::MAX_MEAS_DATA> aData;
};

/*======================================================================================*/
/* Part of SLms100Scan, not documented further.                                         */
/*======================================================================================*/
struct SDataChannel8
{
  void Clear(void);

  SDataChannelHdr DataChannelHdr;

  SOPAS_FlexArray<SOPAS_UInt8, SDataChannelHdr::MAX_MEAS_DATA> aData;
};

/*======================================================================================*/
/* Part of SLms100Scan, not documented further.                                         */
/*======================================================================================*/
struct SPositionBlock
{
  SPositionBlock(void);

  void Clear(void);

  SOPAS_Float dXpos;
  SOPAS_Float dYpos;
  SOPAS_Float dZpos;
  SOPAS_Float dXrot;
  SOPAS_Float dYrot;
  SOPAS_Float dZrot;

  SOPAS_XByte xbRotMode; // XByte: Internal bit structure has to be defined separatedly (see c'tor) 
};

/*======================================================================================*/
/* Part of SLms100Scan, not documented further.                                         */
/*======================================================================================*/
struct SEventBlock
{
  SEventBlock(void);

  void Clear(void);

  SOPAS_FixString aEventType; // Length 4
  SOPAS_UInt32 udiEncoderPos;
  SOPAS_UInt32 udiSystemCount;
  SOPAS_Int32 diAngle;
};

/*======================================================================================*/
/* Part of SLms100Scan, not documented further.                                         */
/*======================================================================================*/
struct SDateTime
{
  SDateTime(void);

  void Clear(void);

  SOPAS_UInt16 uiYear;
  SOPAS_UInt8 usiMonth;
  SOPAS_UInt8 usiDay;
  SOPAS_UInt8 usiHour;
  SOPAS_UInt8 usiMinute;
  SOPAS_UInt8 usiSec;  
#ifndef LMS100_OLD_SCANDATA
  SOPAS_UInt32 udiUSec;
#else
  SOPAS_Int8 siTimeZone;
  SOPAS_UInt16 uiMSec;
#endif
};

/*======================================================================================*/
/*! \brief Scandata structure for LMS100

    \ingroup lms100

    This structure represents all data fields of a LMS100 scan in the appropriate order
    (i.e. the same order they are transferred via the communication channels).
    All data members are public for easy access. Use the Deserialize() function to fill
    this structure from a request's answer on your own. 
    CLms100SopasInterface::GetScanData() requests scandata from a LMS100 and fills this 
    structure accordingly.

    See LMS100 documentation for detailed information about the data members of this
    structure.

    \attention Do not use memset on this structure because it contains #SOPAS_XByte members 
      in the embedded structs which keep structure information beside the variable's value. 
      This may not be overwritten! Use Clear() instead.
 */
/*======================================================================================*/
struct SLms100Scan
{
  static const int MAX_ENCODER_NUM = 3;
  static const int MAX_CHANNEL_NUM = 4;

  SLms100Scan(void);
  
  void Clear(void);
    //!< Sets all data members to zero (or zero length for flex-types)

  bool Deserialize(CDeserializer& rDeserializer, CBinaryDataStream& rReceivedData);
    //!< \brief Deserialize variable read answer of <em>ScanData (LMDscandata)</em> 
    //!< into this structure

  SOPAS_UInt16 uiVersionNo;
  
  SDeviceBlock DeviceBlock;

  SStatusBlock StatusBlock;

  SMeasurementParam1Block MeasurementParam1Block;  
  
  SOPAS_FlexArray<SEncoderBlock, MAX_ENCODER_NUM> aEncoderBlock;
  
  SOPAS_FlexArray<SDataChannel16, MAX_CHANNEL_NUM> aDataChannel16;

  SOPAS_FlexArray<SDataChannel8, MAX_CHANNEL_NUM> aDataChannel8;

  SOPAS_FlexArray<SPositionBlock, 1> aPositionBlock;

  SOPAS_FlexArray<std::string, 1> aDeviceName;

  SOPAS_FlexArray<std::string, 1> aCommentBlock;

  SOPAS_FlexArray<SDateTime, 1> aTimeBlock;

#ifndef LMS100_OLD_SCANDATA
  SOPAS_FlexArray<SEventBlock, 1> aEventBlock;
#endif
};

#endif
