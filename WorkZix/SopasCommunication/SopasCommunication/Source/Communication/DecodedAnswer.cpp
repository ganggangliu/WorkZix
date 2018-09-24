/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "DecodedAnswer.h"

SDecodedAnswer::SDecodedAnswer(void) : 
  eAnswerType(INVALID_ANSWER), iIndexOrErrorCode(0) 
{
}

SDecodedAnswer::SDecodedAnswer(EAnswerType answerType, const std::string& name, 
  SOPAS_UInt16 indexOrErrorCode) : 
eAnswerType(answerType), coName(name), iIndexOrErrorCode(indexOrErrorCode) 
{
}

bool SDecodedAnswer::operator==(const SDecodedAnswer& rOther) const
{
  bool bRetVal = true;
  bRetVal &= (eAnswerType == rOther.eAnswerType);
  bRetVal &= (coName.compare(rOther.coName) == 0);
  bRetVal &= (iIndexOrErrorCode == rOther.iIndexOrErrorCode);

  return bRetVal;
}

bool SDecodedAnswer::operator!=(const SDecodedAnswer& rOther) const
{
  return !operator==(rOther);
}
