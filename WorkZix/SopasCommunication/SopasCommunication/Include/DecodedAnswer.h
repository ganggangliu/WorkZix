#ifndef __DECODED_ANSWER_H__
#define __DECODED_ANSWER_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "SopasBasicTypes.h"
#include <string>

/*======================================================================================*/
/*! \brief Contains information about the type and name/index of incoming SOPAS answers.

    This structure is used to keep information about incoming answers. When an answer
    is processed the type and addressing (name or index) is stored within this structure.

    The following convention applies: If addressing is "by name" then the #coName member is
    not empty. If addressing is "by index" then #coName is an empty string.

    If an error code has been received then #iIndexOrErrorCode contains the error code
    and #eAnswerType is set to #ERROR_CODE. Otherwise #iIndexOrErrorCode contains the
    index if the request was "by index".
   
    Instances of this class can be compared and copied.

    This class is used internally but is also part of the asynchronous callbacks.
    It is used to identify which answer (event or asynchronous method result) 
    caused the callback.
 */
/*======================================================================================*/
struct SDecodedAnswer
{
  //! Determines the type of the answer.
  enum EAnswerType
  {
    READ_VARIABLE,  //!< Answer to a "read variable" request
    WRITE_VARIABLE, //!< Answer to a "write variable" request
    METHOD_ACK,     //!< Acknowledge to an asynchronous method invocation
    METHOD_RESULT,  //!< Return value of a method (synchronous or asynchronous)
    EVENT_ACK,      //!< Acknowledge to an event subscription or unsubscription
    EVENT_RESULT,   //!< Aasynchronous event data
    ERROR_CODE,     //!< SOPAS error code (stored in iIndexOrErrorCode)
    INVALID_ANSWER  //!< No valid SOPAS answer
  };

  SDecodedAnswer(void);
    //!< Constructor sets answer type to invalid
  SDecodedAnswer(EAnswerType answerType, const std::string& name, SOPAS_UInt16 indexOrErrorCode);
    //!< Constructor sets parameters accordingly.

  bool operator==(const SDecodedAnswer& rOther) const;
    //!< Comparison operator compares all elements of this structure.
  bool operator!=(const SDecodedAnswer& rOther) const;
    //!< Compares all elements of this structure for inequality.

  EAnswerType eAnswerType;
    //!< Type of answer
  std::string coName;
    //!< Communication name of answer or \e empty if request was "by index".
  SOPAS_UInt16 iIndexOrErrorCode;
    //!< SOPAS index if coName is empty. If eAnswerType is #ERROR_CODE then it
    //!< contains the error code of the last operation.
};

#endif