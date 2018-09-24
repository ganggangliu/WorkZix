/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "Lms100Scan.h"
#include "Deserializer.h"
#include "BinaryDataStream.h"
#include <assert.h>

SDeviceBlock::SDeviceBlock(void)
{
  // Define bit structure of XBytes
#ifndef LMS100_OLD_SCANDATA
  bool bSuccess = xbState.AddUnsignedBitfield("bDeviceError", 1); // Value is initialized to 0
  bSuccess &= xbState.AddUnsignedBitfield("bContaminationWarning", 1); // Value is initialized to 0
  bSuccess &= xbState.AddUnsignedBitfield("bContaminationError", 1); // Value is initialized to 0
  bSuccess &= xbState.AddUnsignedBitfield("Reserved", 13);
#else
  bool bSuccess = xbState.AddUnsignedBitfield("uiStatus", 16); // Value is initialized to 0
#endif
  assert(bSuccess);

  Clear();
}
void SDeviceBlock::Clear(void)
{
  uiIdent = 0;
  udiSerialNo = 0;
  xbState.Clear();
}

SStatusBlock::SStatusBlock(void)
{
  // Define bit structure of XBytes
  bool bSuccess = xbInputs.AddUnsignedBitfield("uiInputs", 16); // Value is initialized to 0
  bSuccess &= xbOutputs.AddUnsignedBitfield("uiOutputs", 16); // Value is initialized to 0
  assert(bSuccess);

  Clear();
}

void SStatusBlock::Clear(void)
{
  uiTelegramCount = 0;
  uiScanCount = 0;
  udiSystemCountScan = 0;
  udiSystemCountTransmit = 0;
  xbInputs.Clear();
  xbOutputs.Clear();
  uiReserved = 0;    
}

SMeasurementParam1Block::SMeasurementParam1Block(void)
{
  Clear();
}

void SMeasurementParam1Block::Clear(void)
{
  udiScanFreq = 0;
  udiMeasFreq = 0;
}

SEncoderBlock::SEncoderBlock(void)
{
  Clear();
}

void SEncoderBlock::Clear(void)
{
  udiEncoderPos = 0;
  iEncoderSpeed = 0;
}

SDataChannelHdr::SDataChannelHdr(void) : 
#ifndef LMS100_OLD_SCANDATA
aContentType(5)
#else
aContentType(4)
#endif
{
  Clear();
}

void SDataChannelHdr::Clear(void)
{
  aContentType.data.clear();
  dScaleFactor = 0.0f;
  dScaleOffset = 0.0f;
  diStartAngle = 0;
  uiAngleRes = 0;
}

void SDataChannel16::Clear(void)
{
  DataChannelHdr.Clear();
  aData.Clear();
}

void SDataChannel8::Clear(void)
{
  DataChannelHdr.Clear();
  aData.Clear();
}

SPositionBlock::SPositionBlock(void)
{
  // Define bit structure of XBytes
  // Note: This is an EnumX originally but it's just used as unsigned bitfield
  bool bSuccess = xbRotMode.AddUnsignedBitfield("Mode", 8);
  assert(bSuccess);

  Clear();
}

void SPositionBlock::Clear(void)
{
  dXpos = 0.0f;
  dYpos = 0.0f;
  dZpos = 0.0f;
  dXrot = 0.0f;
  dYrot = 0.0f;
  dZrot = 0.0f;
  xbRotMode.Clear();
}

SEventBlock::SEventBlock(void) : aEventType(4)
{
  Clear();
}

void SEventBlock::Clear(void)
{
  aEventType.data.clear();
  udiEncoderPos = 0;
  udiSystemCount = 0;
  diAngle = 0;
}

SDateTime::SDateTime(void)
{
  Clear();
}

void SDateTime::Clear(void)
{
  uiYear = 0;
  usiMonth = 0;
  usiDay = 0;
  usiHour = 0;
  usiMinute = 0;
  usiSec = 0;    
#ifndef LMS100_OLD_SCANDATA
  udiUSec = 0;
#else
  siTimeZone = 0;
  uiMSec = 0;
#endif
}

SLms100Scan::SLms100Scan(void)
{
  // All other members are set to 0 by their constructor
  uiVersionNo = 0;
}

void SLms100Scan::Clear(void)
{    
  uiVersionNo = 0;

  DeviceBlock.Clear();
  StatusBlock.Clear();
  MeasurementParam1Block.Clear();
  aEncoderBlock.Clear();
  aDataChannel16.Clear();
  aDataChannel8.Clear();
  aPositionBlock.Clear();
  aDeviceName.Clear();
  aCommentBlock.Clear();
  aTimeBlock.Clear();
#ifndef LMS100_OLD_SCANDATA
  aEventBlock.Clear();
#endif
}

/*======================================================================================*/
/*! This function is used by CLms100SopasInterface::GetScanData() but can also
be used by other implementations to deserialize scandata from a device answer e.g. when
deserializing a scandata event.

The read pointer of the received data must point to the first element of the scandata
to be deserialized and the data must end at the last scandata byte. Otherwise this
function will report an error.

\note The members of this structure are not cleared before. They are just overwritten.

\attention If this function returns \e false the data contained in this structure is
invalid and may not be used.

\param rDeserializer The according deserializer to use. See 
CProtocol::GetDeserializer().
\param rReceivedData The received data to be deserialized.

\return \e true if the data could be deserialized successfully. \e false otherwise.
*/
/*======================================================================================*/
bool SLms100Scan::Deserialize(CDeserializer& rDeserializer, CBinaryDataStream& rReceivedData)
{
  // Assume true. Might be changed to false by any of the following commands
  bool bRetVal = true;

  // Deserialize version
  bRetVal &= rDeserializer.Deserialize(rReceivedData, uiVersionNo);

  // Deseralize DeviceBlock
  bRetVal &= rDeserializer.Deserialize(rReceivedData, DeviceBlock.uiIdent);
  bRetVal &= rDeserializer.Deserialize(rReceivedData, DeviceBlock.udiSerialNo);
  bRetVal &= rDeserializer.Deserialize(rReceivedData, DeviceBlock.xbState);

  // Deserialize StatusBlock    
  bRetVal &= rDeserializer.Deserialize(rReceivedData, StatusBlock.uiTelegramCount);
  bRetVal &= rDeserializer.Deserialize(rReceivedData, StatusBlock.uiScanCount);
  bRetVal &= rDeserializer.Deserialize(rReceivedData, StatusBlock.udiSystemCountScan);
  bRetVal &= rDeserializer.Deserialize(rReceivedData, StatusBlock.udiSystemCountTransmit);
  bRetVal &= rDeserializer.Deserialize(rReceivedData, StatusBlock.xbInputs);
  bRetVal &= rDeserializer.Deserialize(rReceivedData, StatusBlock.xbOutputs);
  bRetVal &= rDeserializer.Deserialize(rReceivedData, StatusBlock.uiReserved);

  // Deserialize MeasurementParam1Block    
  bRetVal &= rDeserializer.Deserialize(rReceivedData, MeasurementParam1Block.udiScanFreq);
  bRetVal &= rDeserializer.Deserialize(rReceivedData, MeasurementParam1Block.udiMeasFreq);

  // Deserialize aEncoderBlock (FlexArray)        
  bRetVal &= rDeserializer.Deserialize(rReceivedData, aEncoderBlock.uiFlexArrayLength);
  if(aEncoderBlock.IsLengthValid() == true)
  {
    for(SOPAS_UInt16 i = 0; i < aEncoderBlock.uiFlexArrayLength; ++i)
    {
      SEncoderBlock& rEncoderBlock = aEncoderBlock.aFlexArrayData[i];
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rEncoderBlock.udiEncoderPos);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rEncoderBlock.iEncoderSpeed);
    }
  }
  else
  {
    bRetVal = false;
  }

  // Deserialize aDataChannel16 (FlexArray)
  bRetVal &= rDeserializer.Deserialize(rReceivedData, aDataChannel16.uiFlexArrayLength);
  if(aDataChannel16.IsLengthValid() == true)
  {
    for(SOPAS_UInt16 i = 0; i < aDataChannel16.uiFlexArrayLength; ++i)
    {
      SDataChannel16& rDataChannel16 = aDataChannel16.aFlexArrayData[i];
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rDataChannel16.DataChannelHdr.aContentType); // Fix length string
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rDataChannel16.DataChannelHdr.dScaleFactor);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rDataChannel16.DataChannelHdr.dScaleOffset);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rDataChannel16.DataChannelHdr.diStartAngle);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rDataChannel16.DataChannelHdr.uiAngleRes);

      // Deserialize scandata (FlexArray of simple types)
      bRetVal &= rDeserializer.DeserializeArray(rReceivedData, rDataChannel16.aData);
    }
  }
  else
  {
    bRetVal = false;
  }

  // Deserialize aDataChannel8 (FlexArray)
  bRetVal &= rDeserializer.Deserialize(rReceivedData, aDataChannel8.uiFlexArrayLength);
  if(aDataChannel8.IsLengthValid() == true)
  {
    for(SOPAS_UInt16 i = 0; i < aDataChannel8.uiFlexArrayLength; ++i)
    {
      SDataChannel8& rDataChannel8 = aDataChannel8.aFlexArrayData[i];
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rDataChannel8.DataChannelHdr.aContentType); // Fix length string
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rDataChannel8.DataChannelHdr.dScaleFactor);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rDataChannel8.DataChannelHdr.dScaleOffset);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rDataChannel8.DataChannelHdr.diStartAngle);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rDataChannel8.DataChannelHdr.uiAngleRes);

      // Deserialize scandata (FlexArray of simple types)
      bRetVal &= rDeserializer.DeserializeArray(rReceivedData, rDataChannel8.aData);
    }
  }
  else
  {
    bRetVal = false;
  }

  // Deserialize aPositionblock (FlexArray)
  bRetVal &= rDeserializer.Deserialize(rReceivedData, aPositionBlock.uiFlexArrayLength);
  if(aPositionBlock.IsLengthValid() == true)
  {
    for(SOPAS_UInt16 i = 0; i < aPositionBlock.uiFlexArrayLength; ++i)        
    {
      SPositionBlock& rPositionBlock = aPositionBlock.aFlexArrayData[i];
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rPositionBlock.dXpos);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rPositionBlock.dYpos);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rPositionBlock.dZpos);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rPositionBlock.dXrot);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rPositionBlock.dYrot);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rPositionBlock.dZrot);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rPositionBlock.xbRotMode);
    }
  }
  else
  {
    bRetVal = false;
  }

  // Deserialize aDeviceName (FlexArray of simple type)
  bRetVal &= rDeserializer.DeserializeArray(rReceivedData, aDeviceName);

  // Deserialize aCommentBlock (FlexArray of simple type)
  bRetVal &= rDeserializer.DeserializeArray(rReceivedData, aCommentBlock);

  // Deserialize aTimeBlock (FlexArray)
  bRetVal &= rDeserializer.Deserialize(rReceivedData, aTimeBlock.uiFlexArrayLength);
  if(aTimeBlock.IsLengthValid() == true)
  {
    for(SOPAS_UInt16 i = 0; i < aTimeBlock.uiFlexArrayLength; ++i)        
    {
      SDateTime& rTimeBlock = aTimeBlock.aFlexArrayData[i];
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rTimeBlock.uiYear);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rTimeBlock.usiMonth);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rTimeBlock.usiDay);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rTimeBlock.usiHour);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rTimeBlock.usiMinute);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rTimeBlock.usiSec);        
#ifndef LMS100_OLD_SCANDATA
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rTimeBlock.udiUSec);
#else
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rTimeBlock.siTimeZone);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rTimeBlock.uiMSec);
#endif
    }
  }
  else
  {
    bRetVal = false;
  }

#ifndef LMS100_OLD_SCANDATA
  // Deserialize aEventBlock (FlexArray)
  bRetVal &= rDeserializer.Deserialize(rReceivedData, aEventBlock.uiFlexArrayLength);
  if(aEventBlock.IsLengthValid() == true)
  {
    for(SOPAS_UInt16 i = 0; i < aEventBlock.uiFlexArrayLength; ++i)        
    {
      SEventBlock& rEventBlock = aEventBlock.aFlexArrayData[i];
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rEventBlock.aEventType);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rEventBlock.udiEncoderPos);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rEventBlock.udiSystemCount);
      bRetVal &= rDeserializer.Deserialize(rReceivedData, rEventBlock.diAngle);        
    }
  }
  else
  {
    bRetVal = false;
  }
#endif

  // All data should have been read, check it (optional)
  bRetVal &= rReceivedData.IsReadPosAtEnd();

  return bRetVal;
}