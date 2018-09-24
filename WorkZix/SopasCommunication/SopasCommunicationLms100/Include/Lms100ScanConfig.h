#ifndef __LMS100_SCAN_CONFIG_H__
#define __LMS100_SCAN_CONFIG_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "SopasBasicTypes.h"

// Forward declarations
class CDeserializer;
class CSerializer;
class CBinaryDataStream;

struct SRange
{
  SRange(void);

  void Clear(void);
    //!< Sets all data members to zero (or zero length for flex-types)

  SOPAS_UInt32 udiAngleRes;
  SOPAS_Int32 iStartAngle;
  SOPAS_Int32 iStopAngle;
};

/*======================================================================================*/
/*! \brief Scan config structure for LMS100

    \ingroup lms100

    This structure represents the scan configuration (including scan frequency and 
    angle resolution). All data members are public for easy access. This struct
    offers a Serialize() function to be used by CLms100SopasInterface::SetScanConfig().
    Use the Deserialize() function to fill this structure from a request's answer on your own. 

    See LMS100 documentation for detailed information about the data members of this
    structure.

    \attention Do not use memset on this structure because it may contain members later
      that keep structure information beside the variable's value. 
      This may not be overwritten! Use Clear() instead.
 */
/*======================================================================================*/
struct SLms100ScanConfig
{
  static const int LMS_MAX_RANGES = 1;

  SLms100ScanConfig(void);
  SLms100ScanConfig(SOPAS_UInt32 udiScanFrequency, SOPAS_UInt32 udiAngleResolution, 
    SOPAS_Int32 iStartAngle, SOPAS_Int32 iStopAngle);
  
  void Clear(void);
    //!< Sets all data members to zero (or zero length for flex-types)

  bool Deserialize(CDeserializer& rDeserializer, CBinaryDataStream& rReceivedData);
    //!< \brief Deserialize method result of <em>SetScanConfig (mLMPsetscancfg)</em> 
    //!< into this structure

  bool Serialize(CSerializer& rSerializer, CBinaryDataStream& rSerializedParamStream) const;
    //!< \brief Sserialize method params of <em>SetScanConfig (mLMPsetscancfg)</em> 
    //!< into this structure

  size_t GetSerializedSize(CSerializer& rSerializer) const;
    //!< Get size of this structure for serialization

  SOPAS_UInt32 udiScanFreq;

#ifndef LMS100_OLD_SCANDATA
  SOPAS_UInt16 uiLength; 
#endif

  SRange aRange[1]; // Fixed length array
};

#endif