/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "ColaBProtocol.h"
#include "BinaryDataStream.h"

/*======================================================================================*/
/*! \copydetails CProtocol::CProtocol(CCommunication&,CFramer&)                         */
/*======================================================================================*/
CColaBProtocol::CColaBProtocol(CCommunication& rCommunication, CFramer& rFramer) : 
  CProtocol(rCommunication, rFramer)  
{
}

/*======================================================================================*/
/*! \copydetails CProtocol::~CProtocol()                                                */
/*======================================================================================*/
CColaBProtocol::~CColaBProtocol(void)
{
}


/*======================================================================================*/
/*! This class uses the CBinarySerializer to serialize values.

    \return The serializer to use for parameters when calling InvokeMethod() or 
      WriteVariable().
 */
/*======================================================================================*/
CSerializer& CColaBProtocol::GetSerializer(void)
{  
  return m_Serializer;
}

/*======================================================================================*/
/*! This class uses the CBinarySerializer to deserialize values.

    \return The deserializer to use for answers when calling InvokeMethod() or 
      ReadVariable().
 */
/*======================================================================================*/
CDeserializer& CColaBProtocol::GetDeserializer(void)
{
  return m_Deserializer;
}

/*======================================================================================*/
/*! The communication name is separated from the command by a space. The next
    parameter in the datastream is separated by another space which is mandatory and does
    not belong to the next parameter. The space is not appended to the extracted name
    here but removed from the stream.

    \param rInputStream The datastream to extract the communication name from. The
      read pointer must point right behind the SOPAS command.
    \param[out] rCommName The extracted communication name.

    \return \e true if a name could be extracted from the stream. \e false otherwise.
 */
/*======================================================================================*/
bool CColaBProtocol::DeserializeCommName(CBinaryDataStream& rInputStream, std::string& rCommName)
{
  // After command there must be a separator (even for COLA-B)
  bool bSuccess = (rInputStream.ReadChar() == ' ');

  // Find next separator (end of string)
  // If no separator has been found comparison is done until end of answer
  int i = rInputStream.FindIndexOfNext(' ');

  // Extract string until separator (name may not include spaces)
  rCommName = rInputStream.ReadString(i - rInputStream.GetReadIndex());
  bSuccess &= (rCommName.empty() == false);
  // Remove extra space that separates name from parameters (COLA-B specific)
  bSuccess &= (rInputStream.ReadChar() == ' ');

  return bSuccess;
}

/*======================================================================================*/
/*! The COLA-B protocol needs to append a space at the end of the communication name.

    \param rCommName The communication name to be serialized.   

    \return The number of bytes needed to serialize the given name.
 */
/*======================================================================================*/
size_t CColaBProtocol::GetSerializedCommNameSize(const SOPAS_FixString& rCommName)
{
  // COLA-B needs to add a space after communication name in any case
  return m_Serializer.GetSerializedSize(rCommName) + 1;
}

/*======================================================================================*/
/*! The COLA-B protocol needs to append a space at the end of the communication name.

    \param rOutputStream The stream to serialize the communication name to.
    \param rCommName The communication name to be serialized.   

    \return \e true if the name could be written to the stream. \e false otherwise.
 */
/*======================================================================================*/
bool CColaBProtocol::SerializeCommName(CBinaryDataStream& rOutputStream, const SOPAS_FixString& rCommName)
{
  // COLA-B needs to add a space after communication name in any case
  SOPAS_FixString tmp(rCommName.GetClippedString() + ' ');
  return m_Serializer.Serialize(rOutputStream, tmp);
}
