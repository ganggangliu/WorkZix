/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "Lms100ScanConfig.h"
#include "Deserializer.h"
#include "Serializer.h"
#include "BinaryDataStream.h"

SRange::SRange(void)
{
  Clear();
}

void SRange::Clear(void)
{
  udiAngleRes = 0;
  iStartAngle = 0;
  iStopAngle = 0;
}

SLms100ScanConfig::SLms100ScanConfig(void)
{
  Clear();
}

void SLms100ScanConfig::Clear(void)
{
#ifndef LMS100_OLD_SCANDATA
  uiLength = LMS_MAX_RANGES; // Value is fixed for LMS100
#endif
  udiScanFreq = 0;
  aRange[0].Clear();
}

SLms100ScanConfig::SLms100ScanConfig(SOPAS_UInt32 udiScanFrequency, SOPAS_UInt32 udiAngleResolution, 
  SOPAS_Int32 iStartAngle, SOPAS_Int32 iStopAngle)
{
#ifndef LMS100_OLD_SCANDATA
  uiLength = LMS_MAX_RANGES; // Value is fixed for LMS100
#endif

  udiScanFreq = udiScanFrequency;
  aRange[0].udiAngleRes = udiAngleResolution;
  aRange[0].iStartAngle = iStartAngle;
  aRange[0].iStopAngle = iStopAngle;
}

bool SLms100ScanConfig::Deserialize(CDeserializer& rDeserializer, CBinaryDataStream& rReceivedData)
{
  // Assume true. Might be changed to false by any of the following commands
  bool bRetVal = true;

  bRetVal &= rDeserializer.Deserialize(rReceivedData, udiScanFreq);
#ifndef LMS100_OLD_SCANDATA
  bRetVal &= rDeserializer.Deserialize(rReceivedData, uiLength);
  bRetVal &= (uiLength == LMS_MAX_RANGES); // Value is fixed for LMS100
#endif
  bRetVal &= rDeserializer.Deserialize(rReceivedData, aRange[0].udiAngleRes);
  bRetVal &= rDeserializer.Deserialize(rReceivedData, aRange[0].iStartAngle);
  bRetVal &= rDeserializer.Deserialize(rReceivedData, aRange[0].iStopAngle);

  return bRetVal;
}

size_t SLms100ScanConfig::GetSerializedSize(CSerializer& rSerializer) const
{
  size_t ulMinStreamSize = rSerializer.GetSerializedSize(udiScanFreq) + 
#ifndef LMS100_OLD_SCANDATA
    rSerializer.GetSerializedSize(uiLength) +
#endif
    rSerializer.GetSerializedSize(aRange[0].udiAngleRes) +
    rSerializer.GetSerializedSize(aRange[0].iStartAngle) + 
    rSerializer.GetSerializedSize(aRange[0].iStopAngle);

  return ulMinStreamSize;
}

bool SLms100ScanConfig::Serialize(CSerializer& rSerializer, CBinaryDataStream& rSerializedParamStream) const
{
  // Serialize the parameters. The array "aRange" contains only 1 element
  bool bSerialized = rSerializer.Serialize(rSerializedParamStream, udiScanFreq);
#ifndef LMS100_OLD_SCANDATA
  bSerialized &= rSerializer.Serialize(rSerializedParamStream, uiLength);
#endif
  bSerialized &= rSerializer.Serialize(rSerializedParamStream, aRange[0].udiAngleRes);
  bSerialized &= rSerializer.Serialize(rSerializedParamStream, aRange[0].iStartAngle);
  bSerialized &= rSerializer.Serialize(rSerializedParamStream, aRange[0].iStopAngle);

  return bSerialized;
}