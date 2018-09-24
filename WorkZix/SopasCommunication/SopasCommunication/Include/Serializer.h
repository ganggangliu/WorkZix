#ifndef __SERIALIZER_H__
#define __SERIALIZER_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "SopasBasicTypes.h"
#include "SopasXByte.h"
#include "SopasFlexArray.h"
#include "SopasString.h"

// Forward declarations
class CBinaryDataStream;

/*======================================================================================*/
/*! \brief Defines interface for data type serializer.

    \ingroup common

    Inherit from this class to create a data type serializer for your protocol.
    The serializer writes any supported data type to a stream and advances
    the stream accordingly. The serializer must be able to write data
    without any additional information besides the data type to be written. This
    is the case for COLA-A and COLA-B protocols. COLA-A uses a separator and COLA-B
    uses fixed length for each datafield.

    The data type to be written is determined by the overloaded function that
    is called. For example (Note that the links in the code unfortunately do not 
    resolve to the correct overloads.):
    \code
bool SerializeMyData(CBinaryDataStream& rStream, CSerializer& rSerializer)
{
  SOPAS_UInt16 usData1 = 42;
  SOPAS_Double dData2 = 3.14;
  bool bSuccess = rSerializer.Serialize(rStream, usData1);  // Write SOPAS_UInt16
  bSuccess &= rSerializer.Serialize(rStream, dData2);       // Write SOPAS_Double

  return bSuccess;
}
    \endcode

    The necessary size of the CBinaryDataStream can be estimated or calculated
    accordingly. If the stream is too small an error will be reported.

    \code
    // Just use a number that is big enough...
    size_t ulStreamSize = 100; 

    // ... or calculate instead:
    SOPAS_UInt16 usData1 = 42;
    SOPAS_Double dData2 = 3.14;
    ulStreamSize = rSerializer.GetSerializedSize(usData1) + rSerializer.GetSerializedSize(dData2);

    CBinaryDataStream myStream(ulStreamSize);
    \endcode

    The serializer depends on the protocol that is used. So use 
    CProtocol::GetSerializer() to get the right serializer connected to the
    currently selected protocol.

  \note All serializer functions return a boolean value to indicate a successful 
    serialization. If an error occured during writing the write pointer of the 
    CBinaryDataStream will have moved to any position. The remaining data cannot be 
    written correctly in that case and writing must be aborted.
 */
/*======================================================================================*/
class CSerializer
{
public:  
  bool Serialize(CBinaryDataStream& rOutputStream, const SOPAS_XByte& rValue);
    //!< Serialize a SOPAS_XByte.
  size_t GetSerializedSize(const SOPAS_XByte& rValue) const;
    //!< Get necessary stream size for given SOPAS_XByte.

  // --- Serialization of simple compound types --- //
  // Uses serialization of simple types so it will only work for simple types listed below
  bool Serialize(CBinaryDataStream& rOutputStream, 
    const SOPAS_FlexString &rValue);
    //!< Serialize a #SOPAS_FlexString.
  size_t GetSerializedSize(const SOPAS_FlexString& rValue) const;
    //!< Get necessary stream size for given #SOPAS_FlexString.
  template <class T, SOPAS_UInt16 MAX_SIZE>
    bool SerializeArray(CBinaryDataStream& rOutputStream, const SOPAS_FlexArray<T, MAX_SIZE>& rValue);
    //!< Serialize a flex array consisting of simple types.
  template <class T>
    bool SerializeArray(CBinaryDataStream& rOutputStream, SOPAS_UInt16 uiLength, const T* pValue);  
    //!< Serialize a fix length array consisting of simple types.

  template <class T, SOPAS_UInt16 MAX_SIZE>
    size_t GetSerializedSizeArray(const SOPAS_FlexArray<T, MAX_SIZE>& rValue) const;
    //!< Get necessary stream size for given SOPAS_FlexArray.
  template <class T>
    size_t GetSerializedSizeArray(SOPAS_UInt16 uiLength, const T* pValue) const;
      //!< Get necessary stream size for given fix array.

  // --- Serialization of simple types, depends on protocol --- //
  // Defines interface for children
  virtual size_t GetSerializedSize(const SOPAS_FixString& rValue) const = 0;
    //!< Get necessary stream size for given #SOPAS_FixString.

  virtual size_t GetSerializedSize(SOPAS_Int8 value) const = 0;
    //!< Get necessary stream size for given #SOPAS_Int8.
  virtual size_t GetSerializedSize(SOPAS_Int16 value) const = 0;
    //!< Get necessary stream size for given #SOPAS_Int16.
  virtual size_t GetSerializedSize(SOPAS_Int32 value) const = 0;
    //!< Get necessary stream size for given #SOPAS_Int32.

  virtual size_t GetSerializedSize(SOPAS_UInt8 value) const = 0;
    //!< Get necessary stream size for given #SOPAS_UInt8.
  virtual size_t GetSerializedSize(SOPAS_UInt16 value) const = 0;
    //!< Get necessary stream size for given #SOPAS_UInt16.
  virtual size_t GetSerializedSize(SOPAS_UInt32 value) const = 0;
    //!< Get necessary stream size for given #SOPAS_UInt32.
  
  virtual size_t GetSerializedSize(SOPAS_Float value) const = 0;
    //!< Get necessary stream size for given #SOPAS_Float.
  virtual size_t GetSerializedSize(SOPAS_Double value) const = 0;
    //!< Get necessary stream size for given #SOPAS_Double.

  virtual bool Serialize(CBinaryDataStream& rStream, 
    const SOPAS_FixString &rValue) = 0;
    //!< Serialize a #SOPAS_FixString. See implemenation of derived classes for more information.   

  virtual bool Serialize(CBinaryDataStream& rOutputStream, SOPAS_Int8 value) = 0;
    //!< Serialize a #SOPAS_Int8. See implemenation of derived classes for more information.
  virtual bool Serialize(CBinaryDataStream& rOutputStream, SOPAS_Int16 value) = 0;
    //!< Serialize a #SOPAS_Int16. See implemenation of derived classes for more information.
  virtual bool Serialize(CBinaryDataStream& rOutputStream, SOPAS_Int32 value) = 0;
    //!< Serialize a #SOPAS_Int32. See implemenation of derived classes for more information.

  virtual bool Serialize(CBinaryDataStream& rOutputStream, SOPAS_UInt8 value) = 0;
    //!< Serialize a #SOPAS_UInt8. See implemenation of derived classes for more information.
  virtual bool Serialize(CBinaryDataStream& rOutputStream, SOPAS_UInt16 value) = 0;
    //!< Serialize a #SOPAS_UInt16. See implemenation of derived classes for more information.
  virtual bool Serialize(CBinaryDataStream& rOutputStream, SOPAS_UInt32 value) = 0;
    //!< Serialize a #SOPAS_UInt32. See implemenation of derived classes for more information.

  virtual bool Serialize(CBinaryDataStream& rOutputStream, SOPAS_Float value) = 0;
    //!< Serialize a #SOPAS_Float. See implemenation of derived classes for more information.
  virtual bool Serialize(CBinaryDataStream& rOutputStream, SOPAS_Double value) = 0;
    //!< Serialize a #SOPAS_Double. See implemenation of derived classes for more information.
};

/*======================================================================================*/
/*! This is a helper function to serialize fixed length arrays of simple types. 
    This common function uses the specialized overloads for simple types to write 
    an array. An according overload must exist, i.e. the simple type must be supported.

    There is another helper function for flex arrays. See \link 
      SerializeArray(CBinaryDataStream&, const SOPAS_FlexArray<T, MAX_SIZE>&) the according 
      overload \endlink.

    \param rOutputStream The data stream to write the values to.
    \param uiLength The array length. Determines how many elements are written
      from the stream.
    \param pValue The array containing the values to be written to the stream. Must 
      contain at least <em>uiLength</em> elements. This cannot be checked.

    \return <em>true</em> if the values could be written to the stream. <em>false</em> 
      otherwise (e.g. stream is too small).
 */
/*======================================================================================*/
template <class T>
bool CSerializer::SerializeArray(CBinaryDataStream& rOutputStream, SOPAS_UInt16 uiLength, const T* pValue)
{
  bool bRetVal = false;

  if(pValue != 0)
  {
    // Empty arrays are allowed, function just returns true
    bRetVal = true;

    // Just serialize all single elements of the array
    for(SOPAS_UInt16 uiCurElement = 0; uiCurElement < uiLength; ++uiCurElement)
    {
      bRetVal &= Serialize(rOutputStream, pValue[uiCurElement]);
    }
  }

  return bRetVal;
}

/*======================================================================================*/
/*! This is a helper function to serialize flex length arrays of simple types. 
    This common function uses the specialized overloads for simple types to write 
    an array. An according overload must exist, i.e. the simple type must be supported.
    The length of the array is written to the stream first.

    \param rOutputStream The data stream to write the values to.
    \param rValue The SOPAS_FlexArray containing the values to be written to the stream.      

    \return <em>true</em> if the values could be written to the stream. <em>false</em> 
      otherwise (e.g. stream is too small).
 */
/*======================================================================================*/
template <class T, SOPAS_UInt16 MAX_SIZE>
bool CSerializer::SerializeArray(CBinaryDataStream& rOutputStream, const SOPAS_FlexArray<T, MAX_SIZE>& rValue)
{
  bool bRetVal = false;

  // Check if array length isn't too big
  if(rValue.IsLengthValid() == true)
  {
    // Empty arrays are allowed, function just returns true  
    bRetVal = true;

    // Serialize length
    bRetVal &= Serialize(rOutputStream, rValue.uiFlexArrayLength);
    // Serialize array elements
    bRetVal &= SerializeArray(rOutputStream, rValue.uiFlexArrayLength, rValue.aFlexArrayData);
  }

  return bRetVal;
}

/*======================================================================================*/
/*! This function adds up the number of bytes needed to store the array length counter
    and the according number of array elements of the given types.

    It uses the according overloads of GetSerializedSize() for each supported simple type.
    See according inherited implementation of simple types for details.
    
    \param rValue The SOPAS_FlexArray containing the values to be serialized.

    \return The number of bytes needed to store the given data as a serialized stream.

    \note The serialized length may depend on the actual value of <em>rValue</em>. This
      is protocol and implementation specific. So do not use a dummy <em>rValue</em> with
      just the correct type but use the actual value to be serialized instead.
 */
/*======================================================================================*/
template <class T, SOPAS_UInt16 MAX_SIZE>
  size_t CSerializer::GetSerializedSizeArray(const SOPAS_FlexArray<T, MAX_SIZE>& rValue) const
{
  size_t ulCompoundSize = 0;

  // Serialization takes length counter...
  ulCompoundSize += GetSerializedSize(rValue.uiFlexArrayLength);
  // ... and each single array element
  ulCompoundSize += GetSerializedSizeArray(rValue.uiFlexArrayLength, rValue.aFlexArrayData);

  return ulCompoundSize;
}

/*======================================================================================*/
/*! This function adds up the number of bytes needed to store the according number of 
    array elements of the given types.

    It uses the according overloads of GetSerializedSize() for each supported simple type.
    See according inherited implementation of simple types for details.
    
    \param uiLength The length of the fix array.
    \param pValue The array containing the values to be serialized. Must 
      contain at least <em>uiLength</em> elements. This cannot be checked. 

    \return The number of bytes needed to store the given data as a serialized stream.

    \note The serialized length may depend on the actual value of <em>rValue</em>. This
      is protocol and implementation specific. So do not use a dummy <em>rValue</em> with
      just the correct type but use the actual value to be serialized instead.
 */
/*======================================================================================*/
template <class T>
  size_t CSerializer::GetSerializedSizeArray(SOPAS_UInt16 uiLength, const T* pValue) const
{
  size_t ulCompoundSize = 0;

  // Serialization takes each single array element, so sum up sizes
  for(SOPAS_UInt16 uiCurElement = 0; uiCurElement < uiLength; ++uiCurElement)
  {
    ulCompoundSize += GetSerializedSize(pValue[uiCurElement]);
  }

  return ulCompoundSize;
}

#endif