#include "SopasString.h"

/*======================================================================================*/
/*! The length cannot be changed. The string data itself will be empty.
   
    \param uiStringLength Fixed length of the string.
 */
/*======================================================================================*/
SOPAS_FixString::SOPAS_FixString(SOPAS_UInt16 uiStringLength) : uiLength(uiStringLength)
{
}

/*======================================================================================*/
/*! The data may be shorter or longer than the given length. Only the fixed length part
    will be used though.
   
    \param uiStringLength Fixed length of the string.
    \param rData Initial string.
 */
/*======================================================================================*/
SOPAS_FixString::SOPAS_FixString(SOPAS_UInt16 uiStringLength, const std::string& rData) :
  uiLength(uiStringLength), data(rData)
{
}
  
/*======================================================================================*/
/*! The data might be changed later but the length stays fixed.
   
    \param rData Initial string.
 */
/*======================================================================================*/
SOPAS_FixString::SOPAS_FixString(const std::string& rData) : 
  uiLength(rData.length()), data(rData)
{  
}

/*======================================================================================*/
/*! Gets the actual string. If it's shorter than the fixed length the whole string 
    will be returned. It is not padded with zeroes here though! If the string is longer
    it is clipped to the fixed length.
   
    \return The string data clipped to the fixed length.
 */
/*======================================================================================*/
std::string SOPAS_FixString::GetClippedString(void) const
{
  return data.substr(0, uiLength);
}