#ifndef __SOPAS_FLEXARRAY_H__
#define __SOPAS_FLEXARRAY_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include <memory.h>
#include "SopasBasicTypes.h"

/*======================================================================================*/
/*! \ingroup datatypes

    \brief Flex array of arbitrary type

    This struct stores an array of arbitrary type and stores the number of elements
    contained. Only the number of elements currently contained are transferred to/
    from the device. A maximum size is defined which may not be exceeded.

    \note If you set a data element of this array make sure that the uiFlexArrayLength
      is set accordingly. Otherwise not all data elements might be transferred or
      garbage is transferred if data is not properly initialized and uiFlexArrayLength
      is too big.

    \note This implementation is able to store any type of data. The usage should be 
    limited to simple types and strings though because the serializer and deserializer
    only can work with simple types. Flex arrays of structs cannot be serialized or 
    deserialized by this framework and must be processed element by element instead.

    \attention The member data is public to allow easy access. The uiFlexArrayLength
    sets the number of currently contained elements. This is important for transferring
    data. This value may not exceed MAX_SIZE. This is not checked! Best practise for
    deserialization: Extract the length of this flex array and write it to
    #uiFlexArrayLength. Use IsLengthValid() to check that the following array data
    will fit into the array.

    \section FlexArrayExample Example
    \code
{
  SOPAS_FlexArray<SOPAS_UInt16, 10> myArray;
  myArray.aFlexArrayData[0] = 23; // Right now this element would not be serialized.
  myArray.uiFlexArrayLength = 1;  // Now the first element (23) would be serialized.

  myArray.uiFlexArrayLength = 2;  // Danger: Second element would also be serialized
                                  // but it has not been initialized properly.

  // ...
  myArray.uiFlexArrayLength = 3; // Array contains 3 elements now.
  SOPAS_UInt16 x = myArray.aFlexArrayData[3]; // Danger: Do not access elements beyond
                                              // uiFlexArrayLength.
}
    \endcode
 */
/*======================================================================================*/
template <class T, SOPAS_UInt16 MAX_SIZE>
struct SOPAS_FlexArray
{  
  SOPAS_FlexArray(void);
    //!< Constructor initializes current length.
  void Clear(void);
    //!< Sets array length to 0.

  SOPAS_UInt16 uiFlexArrayLength;
    //!< Number of elements that are currently contained in this array.
  T aFlexArrayData[MAX_SIZE];
    //!< Array of data up to MAX_SIZE in length.
  bool IsLengthValid(void) const;
    //!< Checks that current length doesn't exceed maximum length
};

/*======================================================================================*/
/*! The initial array length is set to 0.

    \note The array contents are not initialized because they are of arbitray type
    and might contain pointers. Make sure that the data up to the count of 
    uiFlexArrayLength is properly set before using this array for serialization.
 */
/*======================================================================================*/
template <class T, SOPAS_UInt16 MAX_SIZE>
SOPAS_FlexArray<T, MAX_SIZE>::SOPAS_FlexArray(void) : uiFlexArrayLength(0)
{
}

/*======================================================================================*/
/*! Only checks the currently used size (#uiFlexArrayLength) against the maximum
    size defined for this array. This function is usually used during deserialization
    when the flex length is known (and written to #uiFlexArrayLength) but before the 
    array contents are written to this structure.

    \note This struct is not protected against writing beyond the array limits.

    \return \e true if the currently set length fits into the array. \e false otherwise.
 */
/*======================================================================================*/
template <class T, SOPAS_UInt16 MAX_SIZE>
bool SOPAS_FlexArray<T, MAX_SIZE>::IsLengthValid(void) const
{
  return uiFlexArrayLength <= MAX_SIZE;
}

/*======================================================================================*/
/*! The initial array length is set to 0.

    See note for SOPAS_FlexArray() constructor on clearing the array data!
 */
/*======================================================================================*/
template <class T, SOPAS_UInt16 MAX_SIZE>
void SOPAS_FlexArray<T, MAX_SIZE>::Clear(void)
{
  // Structure is not cleared because it's an arbitrary type (see ctor)
  uiFlexArrayLength = 0;
}

#endif