/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/
#include "Lms100State.h"
#include "Deserializer.h"
#include "BinaryDataStream.h"

SLms100State::SLms100State(void)
{
  Clear();
}

/*======================================================================================*/
/*! This function is used by CLms100SopasInterface::GetState() but can also
    be used by other implementations to deserialize the device state from a device answer.

    The read pointer of the received data must point to the first element of the device 
    state to be deserialized and the data must end at the last device state byte. 
    Otherwise this function will report an error.

    \note The members of this structure are not cleared before. They are just overwritten.

    \attention If this function returns \e false the data contained in this structure is
      invalid and may not be used.

    \param rDeserializer The according deserializer to use. See 
      CProtocol::GetDeserializer().
    \param rReceivedData The received data to be deserialized.

    \return \e true if the data could be deserialized successfully. \e false otherwise.
 */
/*======================================================================================*/
bool SLms100State::Deserialize(CDeserializer& rDeserializer, CBinaryDataStream& rReceivedData)
{
  // Assume true. Might be changed to false by any of the following commands
  bool bRetVal = true;

  // Deseralize Enum16 as UInt16 and convert it to cpp-enum
  SOPAS_UInt16 uiState;        
  bRetVal &= rDeserializer.Deserialize(rReceivedData, uiState);
  bRetVal &= SetFsmStateFromSopas(uiState);

  // Deserialize all other simple types
  bRetVal &= rDeserializer.Deserialize(rReceivedData, bTempOutOfRange);
  bRetVal &= rDeserializer.Deserialize(rReceivedData, szTime);
  bRetVal &= rDeserializer.Deserialize(rReceivedData, szDate);

  // Deserialize fix length array
  for(int i = 0; i < STATUS_LEDS_SIZE; ++i)
  {
    bRetVal &= rDeserializer.Deserialize(rReceivedData, bStatusLeds[i]);
  }

  // All data should have been read, check it (optional)
  bRetVal &= rReceivedData.IsReadPosAtEnd();

  return bRetVal;
}

/*======================================================================================*/
/*! Just casts the given value to EFSMState enum if it is within a valid range.
    The variable #eFSMState is updated accordingly.

    \param uiState The value to set.
    
    \return \e true if eFSMState could be updated. \e false otherwise. eFSMState remains
      unchanged then.
 */
/*======================================================================================*/
bool SLms100State::SetFsmStateFromSopas(SOPAS_UInt16 uiState)
{
  bool retVal = false;
  if(uiState <= LMSSTATE_MAXSTATE)
  {
    eFSMState = static_cast<EFSMState>(uiState);
    retVal = true;
  }

  return retVal;
}

void SLms100State::Clear(void)
{
  eFSMState = LMSSTATE_UNDEFINED;
  bTempOutOfRange = false;
  szTime.clear();
  szDate.clear();
  (void*)memset(&bStatusLeds[0], 0, sizeof(bStatusLeds));
}
