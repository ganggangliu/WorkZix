#ifndef __ASCII_PARSER_H__
#define __ASCII_PARSER_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include <limits>

/*======================================================================================*/
/*! \brief Helper class to parse ASCII strings in decimal notation

    \ingroup common
 
    This class parses decimal numbers of any built-in type. The type is predetermined
    by the template parameter so it is checked that the value is valid for the given 
    type.

    \note 64 bit integers are not supported.
 */
/*======================================================================================*/
template <class T>
class CAsciiParser
{
public:  
  CAsciiParser(void) {}
    //!< No initialization necessary.
  
  bool Parse(const char* pString, T& rResult);
    //!< Tries to parses decimal value of given type

private:
  CAsciiParser(const CAsciiParser&);
    //!< Do not allow copying
  CAsciiParser& operator=(const CAsciiParser&);
    //!< Do not allow copying
};

/*======================================================================================*/
/*! Floating point and integers are supported. Also a range check is done even for signed
    values. So for example -129 will not be valid for #SOPAS_Int8 which ranges from -128
    to 127.

    \param pString The string to be parsed. May only contain the value without any
      surrounding characters.
    \param[out] rResult The parsed value. Only valid when this function returns \e true.

    \return \e true if the value could be parsed successfully. \e false otherwise. Also 
      see note below.

    \note The range check will only work up to 32-bit integers. If the number is
      integer and bigger than that it will be clipped and no error can be detected.
      This is a limitation of \e strtol and \e strtoul which are used here.
 */
/*======================================================================================*/
template <class T>
bool CAsciiParser<T>::Parse(const char* pString, T& rResult)
{
  char* pEndCharacter;

  // Using biggest type (double/unsigned long/signed long) to convert and perform a rangecheck afterwards
  // Does not work for any larger types like "int64"
  bool retVal = false;
  if(pString != 0)
  {
    if(std::numeric_limits<T>::is_integer == true)
    {      
      if(std::numeric_limits<T>::is_signed == true)
      {
        // Signed integer
        // NOTE: strtol does not detect too big numbers for int32! They are clipped to INT_MIN and INT_MAX
        long l = strtol(pString, &pEndCharacter, 10);

        // String was extracted until separator ' '. So the string must have been fully parsed until string terminator.
        // Also check conversion results
        if(*pEndCharacter == 0)
        {
          // Errno may be set but should not be used in a multithreading environment.
          // The result may be 0, LONG_MAX or LONG_MIN for illegal conversions. Those 
          // values cannot be distinguished from valid results (unless using errno). So
          // they are ignored here!

          // Check limits. Signed types must be within (min, max).        

          if((static_cast<long>(std::numeric_limits<T>::min()) <= l) && (l <= static_cast<long>(std::numeric_limits<T>::max())))
          {
            rResult = static_cast<T>(l); // Prevent compiler warning
            retVal = true;
          }
        }
      }
      else
      {
        // Unsigned integer
        if(pString[0] != '-')
        {
          // strtoul converts including wrap-around for negative numbers.
          // Prevent starting sign.
          // NOTE: strtoul does not detect too big numbers for uint32! They are clipped to UINT_MIN and UINT_MAX
          unsigned long l = strtoul(pString, &pEndCharacter, 10);

          // String was extracted until separator ' '. So the string must have been fully parsed until string terminator.
          // Also check conversion results
          if(*pEndCharacter == 0)
          {
            // Errno may be set but should not be used in a multithreading environment.
            // The result may be 0 or ULONG_MAX for illegal conversions. Those 
            // values cannot be distinguished from valid results (unless using errno). So
            // they are ignored here!

            // Check limits. Unsigned types must be within (0, max).                  
            if(l <= static_cast<unsigned long>(std::numeric_limits<T>::max()))
            {
              rResult = static_cast<T>(l); // Prevent compiler warning
              retVal = true;
            }
          }
        }
      }
    }
    else
    {
      // Floating point types (float, double)            
      double d = strtod(pString, &pEndCharacter);
      // String was extracted until separator ' '. So the string must have been fully parsed until string terminator.
      // Also check conversion results
      if((*pEndCharacter == 0) && (d != HUGE_VAL) && (d != -HUGE_VAL))
      {
        // Errno may be set but should not be used in a multithreading environment.
        // Also the result may be 0 for underflow which is OK.

        // Check limits. Floating point types are always signed, so value must be within (-max, max).        
        double dMax = static_cast<double>(std::numeric_limits<T>::max());
        if((-dMax <= d) && (d <= dMax))
        {
          rResult = static_cast<T>(d); // Prevent compiler warning
          retVal = true;
        }
      }
    }
  }

  return retVal;
}

#endif