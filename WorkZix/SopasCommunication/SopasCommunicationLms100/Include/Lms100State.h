#ifndef __LMS100_STATE_H__
#define __LMS100_STATE_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "SopasBasicTypes.h"
#include "SopasString.h"

// Forward declarations
class CDeserializer;
class CBinaryDataStream;

/*======================================================================================*/
/*! \brief State structure for LMS100

    \ingroup lms100

    This structure represents all data fields of a LMS100 state in the appropriate order
    (i.e. the same order they are transferred via the communication channels).
    All data members are public for easy access. Use the Deserialize() function to fill
    this structure from a request's answer on your own. CLms100SopasInterface::GetState()
    requests the state from a LMS100 and fills this structure accordingly.

    See LMS100 documentation for detailed information about the data members of this
    structure.

    \note There is a Clear() function to set all members to 0.
 */
/*======================================================================================*/
struct SLms100State
{  
  static const int STATUS_LEDS_SIZE = 3;

  SLms100State(void);

  //! Sets all data members to 0.
  void Clear(void);

  bool Deserialize(CDeserializer& rDeserializer, CBinaryDataStream& rReceivedData);
    //!< \brief Deserialize variable read answer of <em>DeviceStateLMS (STlms)</em>
    //!< into this structure

  //! Device state enum. Is a Enum16 in the device.
  enum EFSMState 
  {
    LMSSTATE_UNDEFINED = 0,
    LMSSTATE_INIT,
    LMSSTATE_CFG,
    LMSSTATE_IDLE,
    LMSSTATE_ROTATE,
    LMSSTATE_PREPARE,
    LMSSTATE_READY,
    LMSSTATE_MEASURE,
    LMSSTATE_UPDATEFW,
    LMSSTATE_PROD,
    LMSSTATE_ERROR,
    LMSSTATE_MAXSTATE
  } eFSMState;
  
  SOPAS_Bool bTempOutOfRange;  
  SOPAS_FlexString szTime;
  SOPAS_FlexString szDate;

  SOPAS_UInt32 bStatusLeds[STATUS_LEDS_SIZE]; // Fixed length array

private: 
  bool SetFsmStateFromSopas(SOPAS_UInt16 uiState);
    //!< Casts number to according enum value and updates the value in this struct.
};

#endif