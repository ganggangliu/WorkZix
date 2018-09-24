/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "Protocol.h"
#include "BinaryDataStream.h"
#include "Deserializer.h"
#include "Serializer.h"

/*======================================================================================*/
/*! \param rCommunication The communication channel to use for communication.
    \param rFramer The framing to apply to each command.

    \attention The scope of \e rCommunication and \e rFramer must last longer than
    the scope of this CProtocol instance.
 */
/*======================================================================================*/
CProtocol::CProtocol(CCommunication& rCommunication, CFramer& rFramer)
{
  // Comm handler is on heap. Otherwise "this" would have to be used in initializer list
  m_pCommHandler = new CCommunicationHandler(rCommunication, rFramer, *this);
}

CProtocol::~CProtocol(void)
{
  delete m_pCommHandler;
}

/*======================================================================================*/
/*! \copydetails CCommunicationHandler::PollAsyncAnswers
 */
/*======================================================================================*/
void CProtocol::PollAsyncAnswers(int iPollTimeMs, CCommunicationHandler::SopasAsyncCallback pCallback, void* pUser/* = 0*/)
{
  m_pCommHandler->PollAsyncAnswers(iPollTimeMs, pCallback, pUser);
}

/*======================================================================================*/
/*! \copydetails CCommunicationHandler::GetSopasErrorCode                               */
/*======================================================================================*/
SOPAS_UInt16 CProtocol::GetSopasErrorCode(void) const
{  
  return m_pCommHandler->GetSopasErrorCode();
}

/*======================================================================================*/
/*! \copydetails CCommunicationHandler::SetAsyncCallback
     (CCommunicationHandler::SopasAsyncCallback)
 */
/*======================================================================================*/
void CProtocol::SetAsyncCallback(CCommunicationHandler::SopasAsyncCallback pCallback, void* pUser/* = 0*/)
{
  m_pCommHandler->SetAsyncCallback(pCallback, pUser);
}

/*======================================================================================*/
/*! \copydetails CCommunicationHandler::SetAsyncCallback
     (CCommunicationHandler::IAsyncCallback)
 */
/*======================================================================================*/
void CProtocol::SetAsyncCallback(CCommunicationHandler::IAsyncCallback* pCallback)
{
  m_pCommHandler->SetAsyncCallback(pCallback);
}

/*======================================================================================*/
/*! Reads a variable from the device. This function uses the given index to select the
    variable (read by index). Note that many indices are not fixed and might change
    for later releases. Use read by name preferably.

    This function sends an according request to the active communication channel and
    waits for an answer (see SendAndReceive()). If a valid answer has been received 
    the answer's contents are returned. Use the deserializer (GetDeserializer()) 
    to decode the answer. The read pointer of the answer will directly point to 
    the data to be decoded. The structure of the according variable is described 
    in the device documentation.

    \note The answer must be deleted by the caller.

    \param uiIndex The variable's index. 

    \return The answer to the request (pointing to the contents) or \e null if 
      - no answer has been received
      - an invalid answer has been received (e.g. to another request)
      - a device error code has been received. Use GetSopasErrorCode() to check.
 */
/*======================================================================================*/
CBinaryDataStream* CProtocol::ReadVariable(SOPAS_UInt16 uiIndex)
{  
  return BuildCommandWithParams("sRI", SDecodedAnswer::READ_VARIABLE, uiIndex, 0);
}

/*======================================================================================*/
/*! Reads a variable from the device. This function uses the given communication name
    to select the variable (read by name).

    This function sends an according request to the active communication channel and
    waits for an answer (see SendAndReceive()). If a valid answer has been received 
    the answer's contents are returned. Use the deserializer (GetDeserializer()) 
    to decode the answer. The read pointer of the answer will directly point to 
    the data to be decoded. The structure of the according variable is described 
    in the device documentation.

    \note The answer must be deleted by the caller.

    \param rName The variable's communication name. 

    \return The answer to the request (pointing to the contents) or \e null if 
      - no answer has been received
      - an invalid answer has been received (e.g. to another request)
      - a device error code has been received. Use GetSopasErrorCode() to check.
 */
/*======================================================================================*/
CBinaryDataStream* CProtocol::ReadVariable(const std::string &rName)
{
  return BuildCommandWithParams("sRN", SDecodedAnswer::READ_VARIABLE, SOPAS_FixString(rName), 0);
}

/*======================================================================================*/
/*! Calls a device method. This function uses the given index to select the
    method. Note that many indices are not fixed and might change for later releases. 
    Use read by name preferably.

    This function sends an according request to the active communication channel and
    waits for an answer (see SendAndReceive()). Because the method may have a parameter of 
    any type supported by SOPAS (even a struct, an array or empty) the user must serialize 
    the data himself. See explanation for WriteVariable() on how to do this.
    
    The method also returns an answer that can be of any type or empty. The user must
    deserialize the data himself. See explanation for ReadVariable() on how to do this.

    The structure of the according method parameter and its return value is described in the 
    device documentation.

    \note The answer must be deleted by the caller.

    \param uiIndex The method's index.
    \param pSerializedParams A datastream containing the method parameters in serialized
      form as needed by this protocol. Or \e null if the method doesn't have any parameters.

    \return The answer to the request (pointing to the contents). If the method has no 
      return value a CBinaryDataStream should be returned with its read
      position at the end (CBinaryDataStream::IsReadPosAtEnd() returns \e true).
      The function returns \e null if 
      - no answer has been received
      - an invalid answer has been received (e.g. to another request)
      - a device error code has been received. Use GetSopasErrorCode() to check.
 */
/*======================================================================================*/
CBinaryDataStream* CProtocol::InvokeMethod(SOPAS_UInt16 uiIndex, const CBinaryDataStream* pSerializedParams)
{
  
  return BuildCommandWithParams("sMI", SDecodedAnswer::METHOD_RESULT, uiIndex, pSerializedParams);
}

/*======================================================================================*/
/*! Calls a device method. This function uses the given communication name to select the
    method. 

    This function sends an according request to the active communication channel and
    waits for an answer (see SendAndReceive()). Because the method may have a parameter of 
    any type supported by SOPAS (even a struct, an array or empty) the user must serialize 
    the data himself. See explanation for WriteVariable() on how to do this.
    
    The method also returns an answer that can be of any type or empty. The user must
    deserialize the data himself. See explanation for ReadVariable() on how to do this.

    The structure of the according method parameter and its return value is described in the 
    device documentation.

    \note The answer must be deleted by the caller.

    \param rName The method's communication name.
    \param pSerializedParams A datastream containing the method parameters in serialized
      form as needed by this protocol. Or \e null if the method doesn't have any parameters.

    \return The answer to the request (pointing to the contents). If the method has no 
      return value a CBinaryDataStream should be returned with its read
      position at the end (CBinaryDataStream::IsReadPosAtEnd() returns \e true).
      The function returns \e null if 
      - no answer has been received
      - an invalid answer has been received (e.g. to another request)
      - a device error code has been received. Use GetSopasErrorCode() to check.
 */
/*======================================================================================*/
CBinaryDataStream* CProtocol::InvokeMethod(const std::string &rName, const CBinaryDataStream* pSerializedParams)
{
  return BuildCommandWithParams("sMN", SDecodedAnswer::METHOD_RESULT, SOPAS_FixString(rName), pSerializedParams);
}

/*======================================================================================*/
/*! Writes a device variable. This function uses the given communication name to select 
    the variable (write by name). 

    This function sends an according request to the active communication channel and
    waits for an answer (see SendAndReceive()). Because the variable may be of any
    type supported by SOPAS (even a struct or an array) the user must serialize the data
    himself. Use the protocol's serializer (GetSerializer()) to write the data to a 
    CBinaryDataStream. The stream must be big enough to contain the serialized data. 
    You can either estimate the serialized size or calculate the exact size in advance 
    by using the according overloads of 
    \link CSerializer::GetSerializedSize(SOPAS_UInt8)const=0 CSerializer::GetSerializedSize() \endlink. 
    The structure of the according variable is described in the device documentation.

    The request string is framed and sent using current protocol and communication channel. 
    This function then waits for an answer, unframes it and check that it matches the 
    request (see CheckAnswer()).

    \param rName The variable's communication name.
    \param pSerializedVariable A datastream containing the variable content in serialized
      form as needed by this protocol. May not be empty.

    \return \e true if the variable was successfully written to the device. 
      \e false otherwise i.e.:
      - no answer has been received
      - an invalid answer has been received (e.g. to another request)
      - a device error code has been received. Use GetSopasErrorCode() to check.
 */
/*======================================================================================*/
bool CProtocol::WriteVariable(const std::string &rName, const CBinaryDataStream* pSerializedVariable)
{
  bool bRetVal = false;
  CBinaryDataStream* pReceivedData = BuildCommandWithParams("sWN", SDecodedAnswer::WRITE_VARIABLE, SOPAS_FixString(rName), pSerializedVariable);
  // If received data is not null then a valid answer has been received (no SOPAS error code)
  if (pReceivedData != 0)
  {
    // Is read pos at end? Then everything is OK, answer has been processed completely
    bRetVal = pReceivedData->IsReadPosAtEnd();
    delete pReceivedData;
  }
  return bRetVal;
}

/*======================================================================================*/
/*! Writes a device variable. This function uses the given index to select the
    variable (write by index). Note that many indices are not fixed and might change
    for later releases. Use read by name preferably.

    This function sends an according request to the active communication channel and
    waits for an answer (see SendAndReceive()). Because the variable may be of any
    type supported by SOPAS (even a struct or an array) the user must serialize the data
    himself. Use the protocol's serializer (GetSerializer()) to write the data to a 
    CBinaryDataStream. The stream must be big enough to contain the serialized data. 
    You can either estimate the serialized size or calculate the exact size in advance 
    by using the according overloads of 
    \link CSerializer::GetSerializedSize(SOPAS_UInt8)const=0 CSerializer::GetSerializedSize() \endlink. 
    The structure of the according variable is described in the device documentation.

    The request string is framed and sent using current protocol and communication channel. 
    This function then waits for an answer, unframes it and check that it matches the 
    request (see CheckAnswer()).

    \param uiIndex The variable's index.
    \param pSerializedVariable A datastream containing the variable content in serialized
      form as needed by this protocol. May not be empty.

    \return \e true if the variable was successfully written to the device. 
      \e false otherwise i.e.:
      - no answer has been received
      - an invalid answer has been received (e.g. to another request)
      - a device error code has been received. Use GetSopasErrorCode() to check.
 */
/*======================================================================================*/
bool CProtocol::WriteVariable(SOPAS_UInt16 uiIndex, const CBinaryDataStream* pSerializedVariable)
{
  bool bRetVal = false;
  CBinaryDataStream* pReceivedData = BuildCommandWithParams("sWI", SDecodedAnswer::WRITE_VARIABLE, uiIndex, pSerializedVariable);
  // If received data is not null then a valid answer has been received (no SOPAS error code)
  if (pReceivedData != 0)
  {
    // Is read pos at end? Then everything is OK, answer has been processed completely
    bRetVal = pReceivedData->IsReadPosAtEnd();
    delete pReceivedData;
  }
  return bRetVal;
}

/*======================================================================================*/
/*! Subscribes or unsubscribes an event on the device. This function uses the given 
    name to select the event.
   
    Event subscription returns immediately with a synchronous acknowledge. The events
    are transmitted by the device at any time and depends on the event. So this function 
    returns when the acknowledge has been received. After that you need to call 
    PollAsyncAnswers() periodically. When an event is available the registered
    callback will be called. Use the SDecodedAnswer parameter of the callback to 
    determine whether the callback contains the event matching this request.

    The data of the event can be deserialized in the callback using the given deserializer.

    The structure of the according method parameter and its return value is described in the 
    device documentation.

    \param rName The event's communication name.
    \param bSubscribe \e true to subscribe the event, \e false to unsubscribe.

    \return \e true if the subscription was successfully, \e false otherwise.
 */
/*======================================================================================*/
bool CProtocol::SubscribeEvent(const std::string &rName, bool bSubscribe)
{
  bool bRetVal = false;

  // Get serializer of according protocol
  CSerializer& serializer = GetSerializer();
  
  SOPAS_Bool boolParam = bSubscribe; // Convert for serialization
  CBinaryDataStream param(static_cast<unsigned int>(serializer.GetSerializedSize(boolParam)));
  serializer.Serialize(param, boolParam);

  CBinaryDataStream* pReceivedData = BuildCommandWithParams("sEN", 
    SDecodedAnswer::EVENT_ACK, SOPAS_FixString(rName), &param);

  // If received data is not null then a valid answer has been received (no SOPAS error code)
  if (pReceivedData != 0)
  {
    // Is read pos at end event state set? Then everything is OK, answer has been processed completely
    SOPAS_Bool bSubscribeResult;
    bRetVal = GetDeserializer().Deserialize(*pReceivedData, bSubscribeResult);
    bRetVal &= pReceivedData->IsReadPosAtEnd();
    bRetVal &= (boolParam == bSubscribeResult);
    delete pReceivedData;

    if(bRetVal == true)
    {
      if(bSubscribe == 1)
      {
        // Remember async request        
        m_pCommHandler->RegisterEvent(rName);
      }
      else
      {
        // Clear open request
        m_pCommHandler->UnregisterEvent(rName);
      }
    }
  }

  return bRetVal;
}

/*======================================================================================*/
/*! Subscribes or unsubscribes an event on the device. This function uses the given 
    index to select the event.
   
    Event subscription returns immediately with a synchronous acknowledge. The events
    are transmitted by the device at any time and depends on the event. So this function 
    returns when the acknowledge has been received. After that you need to call 
    PollAsyncAnswers() periodically. When an event is available the registered
    callback will be called. Use the SDecodedAnswer parameter of the callback to 
    determine whether the callback contains the event matching this request.

    The data of the event can be deserialized in the callback using the given deserializer.

    The structure of the according method parameter and its return value is described in the 
    device documentation.

    \param uiIndex The event index.
    \param bSubscribe \e true to subscribe the event, \e false to unsubscribe.

    \return \e true if the subscription was successfully, \e false otherwise.
 */
/*======================================================================================*/
bool CProtocol::SubscribeEvent(SOPAS_UInt16 uiIndex, bool bSubscribe)
{
  bool bRetVal = false;

  // Get serializer of according protocol
  CSerializer& serializer = GetSerializer();
  
  SOPAS_Bool boolParam = bSubscribe; // Convert for serialization
  CBinaryDataStream param(static_cast<unsigned int>(serializer.GetSerializedSize(boolParam)));
  serializer.Serialize(param, boolParam);

  CBinaryDataStream* pReceivedData = BuildCommandWithParams("sEI", 
    SDecodedAnswer::EVENT_ACK, uiIndex, &param);

  // If received data is not null then a valid answer has been received (no SOPAS error code)
  if (pReceivedData != 0)
  {
    // Is read pos at end event state set? Then everything is OK, answer has been processed completely
    SOPAS_Bool bSubscribeResult;
    bRetVal = GetDeserializer().Deserialize(*pReceivedData, bSubscribeResult);
    bRetVal &= pReceivedData->IsReadPosAtEnd();
    bRetVal &= (boolParam == bSubscribeResult);

    if(bRetVal == true)
    {
      if(bSubscribe == 1)
      {
        // Remember async request        
        m_pCommHandler->RegisterEvent(uiIndex);
      }
      else
      {
        // Clear open request
        m_pCommHandler->UnregisterEvent(uiIndex);
      }
    }
  }
  return bRetVal;

}

/*======================================================================================*/
/*! Calls an asynchronous device method. This function uses the given communication 
    name to select the method.
   
    Asynchronous functions return immediately with a synchronous acknowledge. The method
    result can be sent by the device at any time depending on the method. So this function 
    returns when the acknowledge has been received. After that you need to call 
    PollAsyncAnswers() periodically. When the method result is available the registered
    callback will be called. Use the SDecodedAnswer parameter of the callback to 
    determine whether the callback contains the result of this request.

    For further details (serializing the parameters and deserializing the answer)
    see InvokeMethod() which is used for synchronous device methods. 

    The structure of the according method parameter and its return value is described in the 
    device documentation.

    \param rName The method's communication name.
    \param pSerializedVariable A datastream containing the method parameters in serialized
      form as needed by this protocol. Or \e null if the method doesn't have any parameters.

    \return \e true if the method was acknowledged (i.e. processing on the device has 
      started), \e false otherwise.
 */
/*======================================================================================*/
bool CProtocol::InvokeAsyncMethod(const std::string &rName, const CBinaryDataStream* pSerializedVariable)
{
  bool bRetVal = false;

  // Remember async request. Must be done before synchronous acknowledge:
  // In case sMA and sAN arrive within one bulk of received data the method must be registered.
  // If no synchroinous acknowledge arrives the registration is taken back.
  m_pCommHandler->RegisterAsyncMethod(rName);

  CBinaryDataStream* pReceivedData = BuildCommandWithParams("sMN", SDecodedAnswer::METHOD_ACK, 
    SOPAS_FixString(rName), pSerializedVariable);
  // If received data is not null then a valid answer has been received (no SOPAS error code)
  if (pReceivedData != 0)
  {
    // Is read pos at end? Then everything is OK, answer has been processed completely
    bRetVal = pReceivedData->IsReadPosAtEnd();
    delete pReceivedData;
  }

  if(bRetVal == false)
  {
    // No synchronous acknowledge received, take back registration
    m_pCommHandler->UnregisterAsyncMethod(rName);
  }

  return bRetVal;
}

/*======================================================================================*/
/*! Calls an asynchronous device method. This function uses the given index to select 
    the method.
   
    Asynchronous functions return immediately with a synchronous acknowledge. The method
    result can be sent by the device at any time depending on the method. So this function 
    returns when the acknowledge has been received. After that you need to call 
    PollAsyncAnswers() periodically. When the method result is available the registered
    callback will be called. Use the SDecodedAnswer parameter of the callback to 
    determine whether the callback contains the result of this request.

    For further details (serializing the parameters and deserializing the answer)
    see InvokeMethod() which is used for synchronous device methods. 

    The structure of the according method parameter and its return value is described in the 
    device documentation.

    \param uiIndex The method's index.
    \param pSerializedVariable A datastream containing the method parameters in serialized
      form as needed by this protocol. Or \e null if the method doesn't have any parameters.

    \return \e true if the method was acknowledged (i.e. processing on the device has 
      started), \e false otherwise.
 */
/*======================================================================================*/
bool CProtocol::InvokeAsyncMethod(SOPAS_UInt16 uiIndex, const CBinaryDataStream* pSerializedVariable)
{
  bool bRetVal = false;

  // Remember async request. Must be done before synchronous acknowledge:
  // In case sMA and sAI arrive within one bulk of received data the method must be registered.
  // If no synchroinous acknowledge arrives the registration is taken back.
  m_pCommHandler->RegisterAsyncMethod(uiIndex);

  CBinaryDataStream* pReceivedData = BuildCommandWithParams("sMI", SDecodedAnswer::METHOD_ACK, 
    uiIndex, pSerializedVariable);
  // If received data is not null then a valid answer has been received (no SOPAS error code)
  if (pReceivedData != 0)
  {
    // Is read pos at end? Then everything is OK, answer has been processed completely
    bRetVal = pReceivedData->IsReadPosAtEnd();
    delete pReceivedData;
  }

  if(bRetVal == false)
  {
    // No synchronous acknowledge received, take back registration
    m_pCommHandler->UnregisterAsyncMethod(uiIndex);
  }

  return bRetVal;
}

/*======================================================================================*/
/*! The command is extracted to determine whether it matches an open request.
    Every command in a SOPAS request is followed by the name or index of the item 
    (variable, method or event) that has been requested. As a special case an error 
    code might be sent by the device.

    The asynchronous commands encode the type of addressing (index or name). The
    synchronous commands do not so the type must be remembered when executing the
    request.
    
    \param rInputStream The answer to process.
    \param[out] rDeterminedAnswer The detected command (only SDecodedAnswer::eAnswerType
      is set).

    \return The addressing that has to be decoded next. Might also be an error code,
      unknown or invalid.
 */
/*======================================================================================*/
CProtocol::EExtractType CProtocol::ExtractCommand(CBinaryDataStream& rInputStream, SDecodedAnswer& rDeterminedAnswer)
{
  EExtractType eExtractType = EXTRACT_INVALID;
  if(rInputStream.ReadChar() == 's')
  {
    const std::string& command = rInputStream.ReadString(2);
    if(command.length() == 2)
    {
      eExtractType = EXTRACT_UNKNOWN;

      // Asynchronous answers, call type is not encoded (name/index).       
      if(command.compare("RA") == 0)
      {
        rDeterminedAnswer.eAnswerType = SDecodedAnswer::READ_VARIABLE;
      }
      else if(command.compare("WA") == 0)
      {
        rDeterminedAnswer.eAnswerType = SDecodedAnswer::WRITE_VARIABLE;
      }
      else if(command.compare("MA") == 0)
      {
        rDeterminedAnswer.eAnswerType = SDecodedAnswer::METHOD_ACK;
      }
      else if(command.compare("EA") == 0)
      {
        rDeterminedAnswer.eAnswerType = SDecodedAnswer::EVENT_ACK;
      }
      // Asynchronous answers, call type is encoded (name/index)
      else if(command.compare("AN") == 0)
      {
        eExtractType = EXTRACT_NAME;
        rDeterminedAnswer.eAnswerType = SDecodedAnswer::METHOD_RESULT;
      }
      else if(command.compare("AI") == 0)
      {
        eExtractType = EXTRACT_INDEX;
        rDeterminedAnswer.eAnswerType = SDecodedAnswer::METHOD_RESULT;
      }
      else if(command.compare("SN") == 0)
      {
        eExtractType = EXTRACT_NAME;
        rDeterminedAnswer.eAnswerType = SDecodedAnswer::EVENT_RESULT;
      }
      else if(command.compare("SI") == 0)
      {
        eExtractType = EXTRACT_INDEX;
        rDeterminedAnswer.eAnswerType = SDecodedAnswer::EVENT_RESULT;
      }
      // Error code
      else if(command.compare("FA") == 0)
      {
        eExtractType = EXTRACT_ERROR_CODE;
        rDeterminedAnswer.eAnswerType = SDecodedAnswer::ERROR_CODE;
      }
      // Not supported / invalid
      else
      {
        eExtractType = EXTRACT_INVALID;
      }
    }
  }

  return eExtractType;
}

/*======================================================================================*/
/*! The command and the name/index following are extracted. For synchronous requests
    the addressing type is not encoded in the answer. So the type of the request
    is remembered and used here.

    The return value contains the name/index and type of answer or the error code.

    \param rInputStream The answer to process.
    \param bLastRequestByIndex Set to \e true if the last synchronous request was made
      by inex or \e false if request was by name. Asynchronous answers encode the 
      addressing type within the answer and don't need this parameter.

    \return The type and name/index of the answer. Or a SOPAS error code.
 */
/*======================================================================================*/
SDecodedAnswer CProtocol::ExtractAnswer(CBinaryDataStream& rInputStream, bool bLastRequestByIndex)
{
  SDecodedAnswer soAnswer; // Is set to invalid by its ctor

  bool bSuccess = true; // Might be set to false by deserialization
  EExtractType eExtractType = ExtractCommand(rInputStream, soAnswer);

  // Clear answer, just in case...
  soAnswer.iIndexOrErrorCode = 0;
  soAnswer.coName = "";

  // If request type was not encoded (i.e. synchronous answers) then use type of last request
  if(eExtractType == EXTRACT_UNKNOWN)
  {
    if(bLastRequestByIndex == true)
    {
      eExtractType = EXTRACT_INDEX;
    }
    else
    {
      eExtractType = EXTRACT_NAME;
    }
  }

  // Use deserializer of according protocol
  CDeserializer& deserializer = GetDeserializer();

  // Extract index/name/errorcode
  if(eExtractType == EXTRACT_INDEX)
  {
    // Deserialize answer
    bSuccess = deserializer.Deserialize(rInputStream, soAnswer.iIndexOrErrorCode);
  }
  else if(eExtractType == EXTRACT_NAME)
  {
    bSuccess = DeserializeCommName(rInputStream, soAnswer.coName);
  }
  else if(eExtractType == EXTRACT_ERROR_CODE)
  {
    // Deserialize answer
    bSuccess = deserializer.Deserialize(rInputStream, soAnswer.iIndexOrErrorCode);
    bSuccess &= rInputStream.IsReadPosAtEnd(); // No more data allowed
  }
  else
  {
    // Invalid answer in ExtractCommand
  }

  if(bSuccess == false)
  {
    soAnswer.eAnswerType = SDecodedAnswer::INVALID_ANSWER;
  }

  return soAnswer;
}

/*======================================================================================*/
/*! Uses the given command, the index/name and the serialized optional parameters to
    build a string, send it and get the answer (see CCommunicationHandler). If a 
    matching answer or a SOPAS error code has been received it is returned afterwards.

    \param rCmd The command to send.    
    \param eExpectedAnswer The expected type of answer to be received (synchronous answer).
    \param rName Communication name to use for the request. 
    \param pSerializedParams Any parameters in serialized form or \e null if no parameters
      should be sent.

    \return The received answer with the read pointer already pointing to the answer's 
      contents. Or \e null if any error occurred.
 */
/*======================================================================================*/
CBinaryDataStream* CProtocol::BuildCommandWithParams(const std::string& rCmd, SDecodedAnswer::EAnswerType eExpectedAnswer,
  const SOPAS_FixString& rName, const CBinaryDataStream* pSerializedParams)
{
  // Get length of parameters if given.   
  int paramLength = 0;
  if (pSerializedParams != 0)
  {
    // If parameters are given add space for params
    paramLength = pSerializedParams->GetUsedLength();
  }

  // Create data stream and add command and serialized name of method
  // Using exact stream size, stream may also be bigger
  size_t minStreamSize = rCmd.length() + paramLength +
    GetSerializedCommNameSize(rName); // command + name/index + params
  CBinaryDataStream coDataStream(static_cast<unsigned int>(minStreamSize));
  // Write command to stream (do not serialize because separator would be added)
  bool bSuccess = coDataStream.WriteToStream(rCmd);
  // Serialize name as fix length string
  bSuccess &= SerializeCommName(coDataStream, rName);  

  // Add parameters if given
  if (pSerializedParams != 0)
  {
    bSuccess &= coDataStream.WriteToStream(*pSerializedParams);
  }

  // Serialization was successful and can be sent?
  CBinaryDataStream* pReceivedData = 0;
  if(bSuccess == true)
  {
    pReceivedData = m_pCommHandler->ProcessSyncRequest(rName, coDataStream, eExpectedAnswer);
  }

  return pReceivedData;
}

/*======================================================================================*/
/*! Uses the given command, the index/name and the serialized optional parameters to
    build a string, send it and get the answer (see CCommunicationHandler). If a 
    matching answer or a SOPAS error code has been received it is returned afterwards.

    \param rCmd The command to send.    
    \param eExpectedAnswer The expected type of answer to be received (synchronous answer).
    \param uiIndex Communication index to use for the request.
    \param pSerializedParams Any parameters in serialized form or \e null if no parameters
      should be sent.

    \return The received answer with the read pointer already pointing to the answer's 
      contents. Or \e null if any error occurred.
 */
/*======================================================================================*/
CBinaryDataStream* CProtocol::BuildCommandWithParams(const std::string& rCmd, SDecodedAnswer::EAnswerType eExpectedAnswer,
  SOPAS_UInt16 uiIndex, const CBinaryDataStream* pSerializedParams)
{
  // Get length of parameters if given.   
  int paramLength = 0;
  if (pSerializedParams != 0)
  {
    // If parameters are given add space for params
    paramLength = pSerializedParams->GetUsedLength();
  }

  // Create data stream and add command and serialized name of method
  // Using exact stream size, stream may also be bigger
  CSerializer& rSerializer = GetSerializer();
  size_t minStreamSize = rCmd.length() + paramLength +
    rSerializer.GetSerializedSize(uiIndex); // command + name/index + params
  CBinaryDataStream coDataStream(static_cast<unsigned int>(minStreamSize));
  // Write command to stream (do not serialize because separator would be added)
  bool bSuccess = coDataStream.WriteToStream(rCmd);
  // Serialize index as UInt16 accordingly
  bSuccess &= rSerializer.Serialize(coDataStream, uiIndex);

  // Add parameters if given
  if (pSerializedParams != 0)
  {
    bSuccess &= coDataStream.WriteToStream(*pSerializedParams);
  }

  // Serialization was successful and can be sent?
  CBinaryDataStream* pReceivedData = 0;
  if(bSuccess == true)
  {
    pReceivedData = m_pCommHandler->ProcessSyncRequest(uiIndex, coDataStream, eExpectedAnswer);
  }

  return pReceivedData;
}
