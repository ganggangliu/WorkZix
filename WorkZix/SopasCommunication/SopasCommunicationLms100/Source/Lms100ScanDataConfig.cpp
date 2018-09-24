/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "Lms100ScanDataConfig.h"
#include "Deserializer.h"
#include "BinaryDataStream.h"
#include <assert.h>

SLms100ScanDataConfig::SRemDataConfig::SRemDataConfig(void)
{
  Clear();
}

void SLms100ScanDataConfig::SRemDataConfig::Clear(void)
{
  bEnable = 0;
  eDataType = REM_DATA_8BIT;
  eContentType = REM_CONT_DIGIT;
}

/*======================================================================================*/
/*! Just casts the given value to EDataType enum if it is within a valid range.
    The variable #eDataType is updated accordingly.

    \param usiDataType The value to set.
    
    \return \e true if eDataType could be updated. \e false otherwise. eDataType remains
      unchanged then.
 */
/*======================================================================================*/
bool SLms100ScanDataConfig::SRemDataConfig::SetDataTypeFromSopas(SOPAS_UInt8 usiDataType)
{
  bool retVal = false;
  if(usiDataType <= REM_DATA_16BIT)
  {
    eDataType = static_cast<EDataType>(usiDataType);
    retVal = true;
  }

  return retVal;
}

/*======================================================================================*/
/*! Just casts the given value to EContentType enum if it is within a valid range.
    The variable #eContentType is updated accordingly.

    \param usiContentType The value to set.
    
    \return \e true if eContentType could be updated. \e false otherwise. eContentType
     remains unchanged then.
 */
/*======================================================================================*/
bool SLms100ScanDataConfig::SRemDataConfig::SetContentTypeFromSopas(SOPAS_UInt8 usiContentType)
{
  bool retVal = false;
  if(usiContentType <= REM_CONT_DIGIT)
  {
    eContentType = static_cast<EContentType>(usiContentType);
    retVal = true;
  }

  return retVal;
}

SLms100ScanDataConfig::SLms100ScanDataConfig(void)
{
  bool bSuccess = DistDataConfig.AddUnsignedBitfield("bEnableChannel1", 1);
  bSuccess &= DistDataConfig.AddUnsignedBitfield("bEnableChannel2", 1);
  bSuccess &= DistDataConfig.AddUnsignedBitfield("bEnableChannel3", 1);
  bSuccess &= DistDataConfig.AddUnsignedBitfield("bEnableChannel4", 1);
  bSuccess &= DistDataConfig.AddUnsignedBitfield("bEnableChannel5", 1);
  bSuccess &= DistDataConfig.AddUnsignedBitfield("Reserved", 11);
  assert(bSuccess);

  bSuccess = EnableEncoderBlock.AddUnsignedBitfield("bEnableEnc1", 1);
  bSuccess &= EnableEncoderBlock.AddUnsignedBitfield("bEnableEnc2", 1);
  bSuccess &= EnableEncoderBlock.AddUnsignedBitfield("bEnableEnc3", 1);
  bSuccess &= EnableEncoderBlock.AddUnsignedBitfield("Reserved", 13);
  assert(bSuccess);
}

void SLms100ScanDataConfig::Clear(void)
{
  DistDataConfig.Clear();
  RemDataConfig.Clear(); 
  EnableEncoderBlock.Clear();
  bEnablePositionBlock = 0;
  bEnableDeviceName = 0;
  bEnableCommentBlock = 0;
  bEnableTimeBlock = 0;
  uiOutputInterval = 0;
}

/*======================================================================================*/
/*! This function is used by CLms100SopasInterface::GetScanDataConfig() but can also
be used by other implementations to deserialize scandata config from a device answer.

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
bool SLms100ScanDataConfig::Deserialize(CDeserializer& rDeserializer, CBinaryDataStream& rReceivedData)
{
  // Assume true. Might be changed to false by any of the following commands
  bool bRetVal = true;
  
  bRetVal &= rDeserializer.Deserialize(rReceivedData, DistDataConfig);

  bRetVal &= rDeserializer.Deserialize(rReceivedData, RemDataConfig.bEnable);
  SOPAS_UInt8 usiEnumWrapper;
  bRetVal &= rDeserializer.Deserialize(rReceivedData, usiEnumWrapper);
  bRetVal &=  RemDataConfig.SetDataTypeFromSopas(usiEnumWrapper);
  bRetVal &= rDeserializer.Deserialize(rReceivedData, usiEnumWrapper);
  bRetVal &=  RemDataConfig.SetContentTypeFromSopas(usiEnumWrapper);
  
  bRetVal &= rDeserializer.Deserialize(rReceivedData, EnableEncoderBlock);
  bRetVal &= rDeserializer.Deserialize(rReceivedData, bEnablePositionBlock);
  bRetVal &= rDeserializer.Deserialize(rReceivedData, bEnableDeviceName);
  bRetVal &= rDeserializer.Deserialize(rReceivedData, bEnableCommentBlock);
  bRetVal &= rDeserializer.Deserialize(rReceivedData, bEnableTimeBlock);
  bRetVal &= rDeserializer.Deserialize(rReceivedData, uiOutputInterval);

  bRetVal &= rReceivedData.IsReadPosAtEnd();

  return bRetVal;
}