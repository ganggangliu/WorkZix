#ifndef __SOPAS_STRING_H__
#define __SOPAS_STRING_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "SopasBasicTypes.h"
#include <string>

/*======================================================================================*/
/*! \ingroup datatypes

    \brief String with flexible length for SOPAS serialization and deserialization.
   
    A SOPAS FlexString can store data up to a maximum length but will be transferred 
    using its current length only. The type is just represented by a std::string using
    the string length to identify the amount of data to transfer.
    
    \note The maximum length of the string is known to the device only unless
    according documentation is available. The maximum length is not checked
    by this representation.

    \attention SOPAS strings may even contain 0 characters. std::string may not!
 */
/*======================================================================================*/
typedef std::string SOPAS_FlexString;
  
/*======================================================================================*/
/*! \ingroup datatypes

    \brief String with fixed length for SOPAS serialization and deserialization.
   
    A fixed length string will always be transferred using its full length. If the
    contained data is shorter it will be padded with zeroes. To allow easy access
    the data is public but the initially set length cannot be changed.

    \note The string might grow longer but only the fixed length will be
     used for transmissions later

    \attention SOPAS strings may even contain 0 characters. std::string may not!
 */
/*======================================================================================*/
struct SOPAS_FixString
{
  explicit SOPAS_FixString(SOPAS_UInt16 uiStringLength);
    //!< Constructor defines the length of the string.
  explicit SOPAS_FixString(const std::string& rData); 
    //!< Constructor which sets length and data by the given string.
  SOPAS_FixString(SOPAS_UInt16 uiStringLength, const std::string& rData);
    //!< Constructor which sets length and data separately.

  std::string GetClippedString(void) const;
    //!< Gets the string up to the fixed length.

  const size_t uiLength;
    //!< Length of the string. Not changeable.
  std::string data;
    //!< String data itself. Might be longer than uiLength.

private:
  SOPAS_FixString(const SOPAS_FixString&);
    //!< Fix length prevents copying
  SOPAS_FixString& operator=(const SOPAS_FixString&);
    //!< Fix length prevents copying
};


#endif