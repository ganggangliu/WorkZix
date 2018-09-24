#ifndef __SOPAS_BASICTYPES_H__
#define __SOPAS_BASICTYPES_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

/*======================================================================================*/
/*! \defgroup datatypes Data types

    \brief Data types that should be used internally for SOPAS serialization and 
    deserialization.

    \section overview Overview
    Sopas defines a set of datatypes that are used on the device. The
    data type has to be known for all serialization and deserialization functions (see 
    CSerializer and CDeserializer). Typedefs have been added for all SOPAS data types
    that are supported by this framework. They should be used whenever a value is written
    to the device or read from the device.

    \section simpletypes Simple types
    The following simple types are supported by this framework and should be sufficient 
    for must applications.

    - #SOPAS_Int8
    - #SOPAS_Int16
    - #SOPAS_Int32
    - #SOPAS_UInt8
    - #SOPAS_UInt16
    - #SOPAS_UInt32
    - #SOPAS_Bool
    - #SOPAS_Float
    - #SOPAS_Double        

    \section complex Complex types
    The following complex or compound types are supported by this framework:

    - #SOPAS_FlexArray
    - #SOPAS_FlexString
    - #SOPAS_FixString
    - #SOPAS_XByte

    \note The FlexArray is only able to handle simple types and strings.
    \note There is also support for fix length arrays of simple types. Just create an array
      (e.g. <em>SOPAS_UInt8 x[8];</em>) and use the according serializer- or
      deserializer functions (i.e. 
      \link CSerializer::SerializeArray(CBinaryDataStream&, SOPAS_UInt16, const T*) SerializeArray() \endlink
      and 
      \link CDeserializer::DeserializeArray(CBinaryDataStream&, SOPAS_UInt16, T*) DeserializeArray() \endlink).

    \section unsupported Unsupported types
    The following types are not supported by this framework:    
    - Enum8/Enum16: Can be serialized and deserialized as #SOPAS_UInt8 or 
      #SOPAS_UInt16 respectively. The data names are known on the device only
      and have to be replaced by according numbers.
    - 64 bit integers are not supported.
    - Arrays (fix or flex) of complex types are not supported but can be
      serialized and deserialized element by element.
    - Structs are not supported but can be serialized and deserialized 
      element by element.      

  @{
 */
/*======================================================================================*/

typedef signed char  SOPAS_Int8;
  //!< 8-bit signed integer for SOPAS serialization and deserialization.
typedef signed short SOPAS_Int16;
  //!< 16-bit signed integer for SOPAS serialization and deserialization.
typedef signed long  SOPAS_Int32;
  //!< 32-bit signed integer for SOPAS serialization and deserialization.

typedef unsigned char  SOPAS_UInt8;
  //!< 8-bit unsigned integer for SOPAS serialization and deserialization.
typedef unsigned short SOPAS_UInt16;
  //!< 16-bit unsigned integer for SOPAS serialization and deserialization.
typedef unsigned long  SOPAS_UInt32;
  //!< 32-bit unsigned integer for SOPAS serialization and deserialization.

typedef float  SOPAS_Float;
  //!< 32-bit floating point for SOPAS serialization and deserialization.
typedef double SOPAS_Double;
  //!< 64-bit floating point for SOPAS serialization and deserialization.

typedef SOPAS_UInt8 SOPAS_Bool; 
  //!< Bool for SOPAS serialization and deserialization. Is just treated as #SOPAS_UInt8.

#endif

/*! @} */
