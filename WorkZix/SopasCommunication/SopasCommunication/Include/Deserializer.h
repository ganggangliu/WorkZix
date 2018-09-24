#ifndef __DESERIALIZER_H__
#define __DESERIALIZER_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "SopasBasicTypes.h"
#include "SopasFlexArray.h"
#include "SopasXByte.h"
#include "SopasString.h"

// Forward declarations
class CBinaryDataStream;

/*======================================================================================*/
/*! \brief Defines interface for data type deserializer.

    \ingroup common

    Inherit from this class to create a data type deserializer for your protocol.
    The deserializer extracts any supported data type out of a stream and advances
    the stream accordingly. The deserializer must be able to extract data
    without any additional information besides the data type to be extracted. This
    is the case for COLA-A and COLA-B protocols. COLA-A uses a separator and COLA-B
    uses fixed length for each datafield.

    The data type to be extracted is determined by the overloaded function that
    is called. For example (Note that the links in the code unfortunately do not 
    resolve to the correct overloads.):
    \code
bool DeserializeMyData(CBinaryDataStream& rStream, CDeserializer& rDeserializer)
{
  SOPAS_UInt16 usData1;
  SOPAS_Double dData2;
  bool bSuccess = rDeserializer.Deserialize(rStream, usData1);  // Extract SOPAS_UInt16
  bSuccess &= rDeserializer.Deserialize(rStream, dData2);       // Extract SOPAS_Double

  return bSuccess;
}
    \endcode    

    The deserializer depends on the protocol that is used. So use 
    CProtocol::GetDeserializer() to get the right deserializer connected to the
    currently selected protocol.

  \note All deserializer functions return a boolean value to indicate a successful 
    deserialization. If an error occured during extracting the read pointer of the 
    CBinaryDataStream will have moved to any position. The remaining data cannot be 
    extracted in that case and the result value will be invalid.
 */
/*======================================================================================*/
class CDeserializer
{    
public:  
  bool Deserialize(CBinaryDataStream& rInputStream, SOPAS_XByte& rResult);
    //!< Deserialize a SOPAS_XByte.

  // --- Deserialization of simple compound types --- //
  // Uses deserialization of simple types so it will only work for simple types listed below
  bool Deserialize(CBinaryDataStream& rInputStream, SOPAS_FlexString& rResult);
    //!< Deserialize a #SOPAS_FlexString.

  template <class T, SOPAS_UInt16 MAX_SIZE>
    bool DeserializeArray(CBinaryDataStream& rInputStream, SOPAS_FlexArray<T, MAX_SIZE>& rResult);
    //!< Deserialize a flex array consisting of simple types.
  template <class T>
    bool DeserializeArray(CBinaryDataStream& rInputStream, SOPAS_UInt16 uiLength, T* pResult);
    //!< Deserialize a fix length array consisting of simple types.

  // --- Deserialization of simple types, depends on protocol --- //
  // Defines interface for children
  virtual bool Deserialize(CBinaryDataStream& rInputStream, SOPAS_FixString& rResult) = 0;
    //!< Deserialize a #SOPAS_FixString. See implemenation of derived classes for more information.

  virtual bool Deserialize(CBinaryDataStream& rInputStream, SOPAS_Int8& rResult) = 0;
    //!< Deserialize a #SOPAS_Int8. See implemenation of derived classes for more information.
  virtual bool Deserialize(CBinaryDataStream& rInputStream, SOPAS_Int16& rResult) = 0;
    //!< Deserialize a #SOPAS_Int16. See implemenation of derived classes for more information.
  virtual bool Deserialize(CBinaryDataStream& rInputStream, SOPAS_Int32& rResult) = 0;
    //!< Deserialize a #SOPAS_Int32. See implemenation of derived classes for more information.

  virtual bool Deserialize(CBinaryDataStream& rInputStream, SOPAS_UInt8& rResult) = 0;
    //!< Deserialize a #SOPAS_UInt8. See implemenation of derived classes for more information.
  virtual bool Deserialize(CBinaryDataStream& rInputStream, SOPAS_UInt16& rResult) = 0;
    //!< Deserialize a #SOPAS_UInt16. See implemenation of derived classes for more information.
  virtual bool Deserialize(CBinaryDataStream& rInputStream, SOPAS_UInt32& rResult) = 0;
    //!< Deserialize a #SOPAS_UInt32. See implemenation of derived classes for more information.

  virtual bool Deserialize(CBinaryDataStream& rInputStream, SOPAS_Float& rResult) = 0;
    //!< Deserialize a #SOPAS_Float. See implemenation of derived classes for more information.
  virtual bool Deserialize(CBinaryDataStream& rInputStream, SOPAS_Double& rResult) = 0;
    //!< Deserialize a #SOPAS_Double. See implemenation of derived classes for more information.
};

/*======================================================================================*/
/*! This is a helper function to deserialize fixed length arrays of simple types. 
    This common function uses the specialized overloads for simple types to extract 
    an array. An according overload must exist, i.e. the simple type must be supported.

    There is another helper function for flex arrays. See \link 
      DeserializeArray(CBinaryDataStream&, SOPAS_FlexArray<T, MAX_SIZE>&) the according 
      overload \endlink.

    \param rInputStream The data stream to extract the values from.
    \param uiLength The array length. Determines how many elements are extracted
      from the stream.
    \param pResult The array the deserialized values are written to. Must be able
      to store uiLength elements. Only valid when this function returns <em>true</em>.

    \return <em>true</em> if the given number of elements could be extracted from the 
      stream. <em>false</em> otherwise.

    \note The array length cannot be checked so the memory pointed to by <em>pResult</em>
      must be big enough.
 */
/*======================================================================================*/
template <class T>
bool CDeserializer::DeserializeArray(CBinaryDataStream& rInputStream, SOPAS_UInt16 uiLength, T* pResult)
{
  bool bRetVal = false;

  if(pResult != 0)
  {
    // Empty arrays are allowed, function just returns true
    bRetVal = true;

    // Try to read according number of simple elements from stream
    SOPAS_UInt16 uiCurElement = 0;
    while((bRetVal == true) && (uiCurElement < uiLength))
    {
      bRetVal &= Deserialize(rInputStream, pResult[uiCurElement]);
      ++uiCurElement;
    }
  }

  return bRetVal;
}

/*======================================================================================*/
/*! This is a helper function to deserialize flex length arrays of simple types. 
    This common function uses the specialized overloads for simple types to extract 
    an array. An according overload must exist, i.e. the simple type must be supported.
    The length of the array is extracted from the stream.

    \param rInputStream The data stream to extract the values from.
    \param rResult The SOPAS_FlexArray the deserialized values are written to. Must be able
      to store uiLength elements. Only valid when this function returns <em>true</em>.

    \return <em>true</em> if an according number of elements could be extracted from the 
      stream and store in <em>pResult</em>. <em>false</em> otherwise.

    \note This function calls DeserializeArray(CBinaryDataStream&, SOPAS_UInt16, T*) 
      so those restrictions also apply.
 */
/*======================================================================================*/
template <class T, SOPAS_UInt16 MAX_SIZE>
bool CDeserializer::DeserializeArray(CBinaryDataStream& rInputStream, SOPAS_FlexArray<T, MAX_SIZE>& rResult)
{
  bool bRetVal = false;  

  // Find length indicator (separated by space)  
  if (Deserialize(rInputStream, rResult.uiFlexArrayLength) == true)
  {
    // Check that data fits into maximum length of FlexArray
    if(rResult.IsLengthValid() == true)
    {
      // Deserialize fixed array using detected length
      bRetVal = DeserializeArray(rInputStream, rResult.uiFlexArrayLength, rResult.aFlexArrayData);
    }
  }

  return bRetVal;
}

#endif