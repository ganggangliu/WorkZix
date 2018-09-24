/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "CommunicationHandler.h"
#include "Framer.h"
#include "Communication.h"
#include "BinaryDataStream.h"
#include "Protocol.h"

#ifdef WIN32
  #include <Windows.h> // Sleep
#endif

#include <time.h>
#include <algorithm>
#include <assert.h>

/*======================================================================================*/
/*! \param rCommunication The communication channel to use for communication.
    \param rFramer The framing to apply to each command.
    \param rProtocol The protocol to use to decode answers.

    \attention The scope of \e rCommunication, \e rFramer and \e rProtocol must last 
    longer than the scope of this CCommunicationHandler instance.
 */
/*======================================================================================*/
CCommunicationHandler::CCommunicationHandler(CCommunication& rCommunication, CFramer& rFramer, 
  CProtocol& rProtocol) :
  m_Communication(rCommunication), m_Framer(rFramer), m_Protocol(rProtocol), 
  m_bLastRequestByIndex(false), m_LastSopasErrorCode(0), m_pFctCallback(0), m_pUser(0), m_pClassCallback(0)
{
  m_AsyncMethodRequests.reserve(10);
  m_SubscribedEvents.reserve(20);
}

CCommunicationHandler::~CCommunicationHandler(void)
{
}

CCommunicationHandler::IAsyncCallback::~IAsyncCallback(void)
{
}

/*======================================================================================*/
/*! This callback function is called by PollAsyncAnswers() or by any synchronous request
    when an asynchronous answer is received. The callback function is executed in the 
    context of the caller. All asynchronous answers (asynchronous method answers and 
    subscribed events) will be delegated to the callback.

    There is another version using a \link SetAsyncCallback(IAsyncCallback*)
    class-based callback\endlink. They can be used mutually exclusive, when this 
    function pointer version is registered the class-based callback will be disabled.
    
    The callback function gets all parameters necessary to decode the data:
    - A SDecodedAnswer struct which determines the type of callback (event or 
      asynchronous method answer) as well as the name or index of the request.
      Must be used to distinguish the subscribed events and open method requests.
    - A CBinaryDataStream pointing to the start of the data to be decoded (i.e.
      method answer or event contents).
    - A CDeserializer of the current protocol to deserialize the data.

    The SDecodedAnswer parameter will be either SDecodedAnswer::EVENT_RESULT or
    SDecodedAnswer::METHOD_RESULT. No other answer types will be passed to the callback.
    If an answer to request by index arrives the SDecodedAnswer::coName will be empty
    and SDecodedAnswer::iIndexOrErrorCode will contain the according value. For a request
    by name the SDecodedAnswer::coName will contain the name of the event or method.

    \note keep processing time in callback short because the whole framework is blocked
      during callback.
    \attention Do not send any SOPAS request or poll for SOPAS events during callback.

    \param pCallback The callback function or \e null to disable callback.
 */
/*======================================================================================*/ 
void CCommunicationHandler::SetAsyncCallback(CCommunicationHandler::SopasAsyncCallback pCallback, void* pUser)
{
  m_pFctCallback = pCallback;
  m_pUser = pUser;
  m_pClassCallback = 0; // deactivate other callback
}

/*======================================================================================*/
/*! This callback function is called by PollAsyncAnswers() or by any synchronous request
    when an asynchronous answer is received. The callback function is executed in the 
    context of the caller. All asynchronous answers (asynchronous method answers and 
    subscribed events) will be delegated to the callback.

    There is another version using a \link SetAsyncCallback(SopasAsyncCallback)
    function pointer callback\endlink. They can be used mutually exclusive, when this 
    class-based version is registered the function pointer callback will be disabled.

    Derive from the IAsyncCallback class and implement the IAsyncCallback::AsyncCallback()
    function to process the async answers.
    
    The callback function gets all parameters necessary to decode the data:
    - A SDecodedAnswer struct which determines the type of callback (event or 
      asynchronous method answer) as well as the name or index of the request.
      Must be used to distinguish the subscribed events and open method requests.
    - A CBinaryDataStream pointing to the start of the data to be decoded (i.e.
      method answer or event contents).
    - A CDeserializer of the current protocol to deserialize the data.

    The SDecodedAnswer parameter will be either SDecodedAnswer::EVENT_RESULT or
    SDecodedAnswer::METHOD_RESULT. No other answer types will be passed to the callback.
    If an answer to request by index arrives the SDecodedAnswer::coName will be empty
    and SDecodedAnswer::iIndexOrErrorCode will contain the according value. For a request
    by name the SDecodedAnswer::coName will contain the name of the event or method.

    \note keep processing time in callback short because the whole framework is blocked
      during callback.
    \attention Do not send any SOPAS request or poll for SOPAS events during callback.
    \attention Make sure that the instance of the callback class stays in scope longer
      than any polling will take place. Otherwise disable this callback before the
      instance leaves scope!

    \param pCallback The callback class or \e null to disable callback.
 */
/*======================================================================================*/ 
void CCommunicationHandler::SetAsyncCallback(CCommunicationHandler::IAsyncCallback* pCallback)
{
  m_pClassCallback = pCallback; 
  m_pFctCallback = 0; // deactivate other callback
}

/*======================================================================================*/
/*! This helper function takes the given data which should be a complete protocol message
    and adds a framing, sends the data and waits for an answer. If an answer is received
    within 15s the framing is removed and the data is returned.    

    \param rDataToSend The data to be framed and sent.

    \return The received data without framing. 0 if any error occurred or no data has
      been received.
    
    \note This function uses busy waiting until an answer is received. To reduce processor
      load a Sleep() command is used. This command is not portable and will only be compiled
      if <em>WIN32</em> is defined. Read performance may be reduced if the CCommunication buffers
      are small and the answer is big.
 */
/*======================================================================================*/
CBinaryDataStream* CCommunicationHandler::SendAndReceive(const CBinaryDataStream& rDataToSend)
{
  std::auto_ptr<CBinaryDataStream> coReceivedDataStream(0);

  // Add framing. Data is deleted at the end of function automatically
  std::auto_ptr<CBinaryDataStream> coFramedData(m_Framer.AddFraming(rDataToSend));

  // Send request
  bool bMatchingSyncAnswerReceived = false;  
  if ((coFramedData.get() != 0) && (m_Communication.WriteBytes(*coFramedData) == true))
  {
    // Communication timeout to wait for an answer
    static const int TIMEOUT_IN_S = 15;
    clock_t time_now = clock();
    const clock_t time_end = time_now + TIMEOUT_IN_S * CLOCKS_PER_SEC;

    // Read incoming data until matching answer is decoded or timeout occurred
    do
    {
      // Data is deleted at the end of do-loop automatically
      std::auto_ptr<CBinaryDataStream> coReceivedFramedData(m_Communication.ReadBytes());

      // Process data if received
      if(coReceivedFramedData.get() != 0)
      {
        // Remove framing until all input data has been processed
        do
        {
          // Data is deleted at the end of do-loop automatically
          std::auto_ptr<CBinaryDataStream> coUnframedData(m_Framer.RemoveFraming(*coReceivedFramedData));

          if (bMatchingSyncAnswerReceived == false)
          {
            // Check if received command matches request
            // auto_ptr will not delete object if it is a valid sync answer
            // Async callbacks are also processed here!
            bMatchingSyncAnswerReceived = HandleSyncAnswer(coUnframedData, coReceivedDataStream);
          }
          else
          {
            // Only process async requests.
            // If a further valid sync request is detected it is ignored here 
            // (shouldn't occur, those answers only follow synchronous requests and 
            // no new request has been made inbetween)
            HandleAsyncAnswer(coUnframedData, 0);
          }
        } while (coReceivedFramedData->IsReadPosAtEnd() == false);
      }

#ifdef WIN32
      Sleep(0); // Keep the program reactive
#endif

      // Update current time for timeout checking
      time_now = clock();
    } while ((bMatchingSyncAnswerReceived == false) && (time_now < time_end));
  }

  // If no matching answer has been received do not accept it in later calls.
  if(bMatchingSyncAnswerReceived == false)
  {
    m_LastSyncRequest.eAnswerType = SDecodedAnswer::INVALID_ANSWER;
  }

  return coReceivedDataStream.release();
}

/*======================================================================================*/
/*! This function checks the given answer in the following order for:
    - A SOPAS error code
    - A matching answer to the last synchronous request.
    - An asynchronous answer (see HandleAsyncAnswer())
    - Invalid answers are discarded.

    It uses CProtocol::ExtractAnswer to determine the type of answer and extracts the
    name or index.

    This function discards answers that are not well formed SOPAS protocol answers.
    If they are well formed but not a matching answer for the request they are passed
    to HandleAsyncAnswer() and might be discarded there.

    \param rAnswer The answer to process. The read pointer must be at the first position.
    \param[out] rAnswerPayload Will contain the answer to the synchronous request
      if a matching synchronous answer has been received (ownership transfer from \e rAnswer). 
      The read pointer will point to the start of the payload that has to be deserialized by 
      the caller.

    \return \e true if the answer matches the last synchronous request (i.e. an error code
      or the answer to the request). \e false otherwise.
 */
/*======================================================================================*/
bool CCommunicationHandler::HandleSyncAnswer(std::auto_ptr<CBinaryDataStream>& rAnswer, std::auto_ptr<CBinaryDataStream>& rAnswerPayload)
{
  bool bSyncAnswerReceived = false;

  // Check if received command matches request
  if(rAnswer.get() != 0)
  {
    SDecodedAnswer answer = m_Protocol.ExtractAnswer(*rAnswer, m_bLastRequestByIndex);
    if (answer.eAnswerType != SDecodedAnswer::INVALID_ANSWER)
    {
      if(answer.eAnswerType == SDecodedAnswer::ERROR_CODE)
      {
        m_LastSopasErrorCode = answer.iIndexOrErrorCode;
        bSyncAnswerReceived = true;
      }
      else if(answer == m_LastSyncRequest)
      {
        // TODO: auto_ptr assignment. Confusing method signature
        rAnswerPayload = rAnswer; // Transfer ownership to 2nd parameter
        m_LastSyncRequest.eAnswerType = SDecodedAnswer::INVALID_ANSWER; // Do not react to same answer again
        bSyncAnswerReceived = true;
      }
      else
      {
        // HandleAsyncAnswer determines command again. Reset read pointer.
        rAnswer.get()->ResetReadPointer();
        HandleAsyncAnswer(rAnswer, 0);
      }
    }
    else
    {
#ifdef COMMUNICATION_DEBUG
      printf("(HandleSyncAnswer): Invalid answer\n");
#endif
    }
  }

  return bSyncAnswerReceived;
}

/*======================================================================================*/
/*! This function checks the given answer for:
    - Asynchronous method results (if requested before).
    - Events (if subscribed before).

    All other answers (i.e. events that were not subscribed, asynchronous method answers
    that have not been requested, not well-formed answers or answers to synchronous 
    requests) are discarded.    

    It uses CProtocol::ExtractAnswer to determine the type of answer and extracts the
    name or index. If a registered request is detected the registered callback is called.
    The callback can be overridden by setting \e pTemporaryCallback. The payload
    of the answer is passed to the callback and deleted here afterwards.

    When events are subscribed or unsubscribed they must be registered here by calling 
    RegisterEvent() or UnregisterEvent(). 
    
    Asynchronous methods must be registered here by calling RegisterAsyncMethod(). 
    If a matching answer is received the registration is cleared.

    \param rAnswer The answer to process. The read pointer must be at the first position.
    \param pTemporaryCallback Overrides registered callback (SetAsyncCallback()) for this
      call. If a registered asynchronous answer is received, this callback is used instead.
      Set to \e null to use the registered callback (default).
 */
/*======================================================================================*/
void CCommunicationHandler::HandleAsyncAnswer(std::auto_ptr<CBinaryDataStream>& rAnswer, SopasAsyncCallback pTemporaryCallback, void* pUser/* = 0*/)
{
  // Use temporary callback if selected. Otherwise use registered one.
  SopasAsyncCallback pSelectedCallback = m_pFctCallback;
  void* pSelectedUser = m_pUser;
  if(pTemporaryCallback != 0)
  {
    pSelectedCallback = pTemporaryCallback;
	pSelectedUser = pUser;
  }

  // Check if received command matches request
  if(rAnswer.get() != 0)
  {
    SDecodedAnswer answer = m_Protocol.ExtractAnswer(*rAnswer, true); // Last request type is not important here.
    if (answer.eAnswerType != SDecodedAnswer::INVALID_ANSWER)
    {
      // Is answer for a requested async call?
      if(answer.eAnswerType == SDecodedAnswer::METHOD_RESULT)
      {
        // Find answer in requests
        std::vector<SDecodedAnswer>::iterator iter = std::find(m_AsyncMethodRequests.begin(),
          m_AsyncMethodRequests.end(), answer);
        if(iter != m_AsyncMethodRequests.end())
        {
          // Async method answer found, callback. rAnswer is deleted automatically
          // Use either function pointer callback or class callback. Temporary callback has priority.
          if(pSelectedCallback != 0)
          {
            pSelectedCallback(*iter, *rAnswer, m_Protocol.GetDeserializer(), pSelectedUser);
          }
          else
          {
            if(m_pClassCallback != 0)
            {
              m_pClassCallback->AsyncCallback(*iter, *rAnswer, m_Protocol.GetDeserializer());
            }
          }

          // Remove registered request, answer only allowed once
          iter = m_AsyncMethodRequests.erase(iter);
        }
        else
        {
#ifdef COMMUNICATION_DEBUG
          printf("(HandleAsyncAnswer): Unregistered method answer received\n");
#endif
        }
      }
      else if(answer.eAnswerType == SDecodedAnswer::EVENT_RESULT)
      {
        // Find answer in requests
        std::vector<SDecodedAnswer>::const_iterator iter = std::find(m_SubscribedEvents.begin(),
          m_SubscribedEvents.end(), answer);
        if(iter != m_SubscribedEvents.end())
        {
          // Event found, callback. rAnswer is deleted automatically
          // Use either function pointer callback or class callback. Temporary callback has priority.
          if(pSelectedCallback != 0)
          {
            pSelectedCallback(*iter, *rAnswer, m_Protocol.GetDeserializer(), pSelectedUser);
          }
          else
          {
            if(m_pClassCallback != 0)
            {
              m_pClassCallback->AsyncCallback(*iter, *rAnswer, m_Protocol.GetDeserializer());
            }
          }
        }
        else
        {
#ifdef COMMUNICATION_DEBUG
          printf("(HandleAsyncAnswer): Unregistered event received\n");
#endif
        }
      }
      else
      {
#ifdef COMMUNICATION_DEBUG
        printf("(HandleAsyncAnswer): Unexpected async\n");
#endif
      }
    }
    else
    {
#ifdef COMMUNICATION_DEBUG
      printf("(HandleAsyncAnswer): Invalid answer\n");
#endif
    }
  }
}

/*======================================================================================*/
/*! Checks if asynchronous answers have been received. The answers will be decoded and
    the registered callback (see SetAsyncCallback()) will be called, if:
    - It is an event and the event was subscribed by this framework before.
    - It is an asynchronous method answer and the request was sent by the framework 
      before.

    All other answers are discarded directly. The received data will be discarded after
    the callback has been processed. If no callback is registered the answer is
    discarded, too.

    \note Synchronous answers are discarded here. But if you make a synchronous request
      the asynchronous answers will be processed (i.e. PollAsyncAnswers() is called 
      internally) while the synchronous request waits for an answer.

    \param iPollTimeMs The polling time in milliseconds. After the time is expired
      execution is passed back to the caller. On WIN32 platforms a Sleep(0) is executed
      during polling to increase responsiveness.
    \param pCallback Can be used to override the registered callback. Is valid
      for this polling period only. If set it will override class-based or function 
      pointer callback. Set to \e null if you want to use the registered
      callback (default).
 */
/*======================================================================================*/ 
void CCommunicationHandler::PollAsyncAnswers(int iPollTimeMs, SopasAsyncCallback pCallback, void* pUser/* = 0*/)
{
  // Communication timeout to wait for an answer
  clock_t time_now = clock();
  double deltaTime = static_cast<double>(iPollTimeMs * CLOCKS_PER_SEC) / 1000.0f;
  const clock_t time_end = time_now + static_cast<clock_t>(deltaTime + 0.5f);

  // Read incoming data until matching answer is decoded or timeout occurred
  do
  {
    // Data is deleted at the end of do-loop automatically
    std::auto_ptr<CBinaryDataStream> coReceivedFramedData(m_Communication.ReadBytes());

    // Process data if received
    if(coReceivedFramedData.get() != 0)
    {
      // Remove framing until all input data has been processed
      do
      {
        // Data is deleted at the end of do-loop automatically
        std::auto_ptr<CBinaryDataStream> coUnframedData(m_Framer.RemoveFraming(*coReceivedFramedData));

        if (coUnframedData.get() != 0)
        {
          // Framer has finished one frame if pUnframedData is not null.
          // coUnframedData can be deserialized in callback. 
          // In any way it is deleted here afterwards!
          HandleAsyncAnswer(coUnframedData, pCallback, pUser);
        }
      } while (coReceivedFramedData->IsReadPosAtEnd() == false);
    }

#ifdef WIN32
    Sleep(1); // Keep the program reactive
#endif

    // Update current time for timeout checking
    time_now = clock();
  } while (1/*time_now < time_end*/);
}

/*======================================================================================*/
/*! The error code is extracted if the device reports an error by answering with \e sFA.
    See description of error checking in the \link CProtocol class description\endlink.
    The error code is reset to 0 for each new request.

    \return The error code of the last request.
 */
/*======================================================================================*/
SOPAS_UInt16 CCommunicationHandler::GetSopasErrorCode(void) const
{
  return m_LastSopasErrorCode;
}

/*======================================================================================*/
/*! Just remembers the type and name of the current request to check for matching answer.

    \param rName The communication name of the device variable, method or event.
    \param eAnswerType The type of the answer.
 */
/*======================================================================================*/
void CCommunicationHandler::RegisterSyncRequest(const SOPAS_FixString& rName, SDecodedAnswer::EAnswerType eAnswerType)
{
  m_LastSyncRequest.coName = rName.data;
  m_LastSyncRequest.iIndexOrErrorCode = 0;
  m_LastSyncRequest.eAnswerType = eAnswerType;

  m_bLastRequestByIndex = false;
  m_LastSopasErrorCode = 0; // No error
}

/*======================================================================================*/
/*! Just remembers the type and name of the current request to check for matching answer.

    \param uiIndex The index of the device variable, method or event.
    \param eAnswerType The type of the answer.
 */
/*======================================================================================*/
void CCommunicationHandler::RegisterSyncRequest(SOPAS_UInt16 uiIndex, SDecodedAnswer::EAnswerType eAnswerType)
{
  m_LastSyncRequest.coName = "";
  m_LastSyncRequest.iIndexOrErrorCode = uiIndex;
  m_LastSyncRequest.eAnswerType = eAnswerType;

  m_bLastRequestByIndex = true;
  m_LastSopasErrorCode = 0; // No error
}

/*======================================================================================*/
/*! Events need to be registered here otherwise they are discarded during polling. 
    Events should only be registered once, this is not checked.

    \param rName The communication name of the event to register.
 */
/*======================================================================================*/
void CCommunicationHandler::RegisterEvent(const std::string& rName)
{
  // Remember async request
  SDecodedAnswer answer(SDecodedAnswer::EVENT_RESULT, rName, 0);
  m_SubscribedEvents.push_back(answer);
}

/*======================================================================================*/
/*! Events need to be registered here otherwise they are discarded during polling. 
    Events should only be registered once, this is not checked.

    \param uiIndex The event index of the event to register.
 */
/*======================================================================================*/
void CCommunicationHandler::RegisterEvent(SOPAS_UInt16 uiIndex)
{
  SDecodedAnswer answer(SDecodedAnswer::EVENT_RESULT, "", uiIndex);
  m_SubscribedEvents.push_back(answer);
}

/*======================================================================================*/
/*! If events are unsubscribe at the device they should also be unregistered here.
    If the given event hasn't been registered before the list of registered events remains 
    unchanged.

    \param rName The communication name of the event to unregister.
 */
/*======================================================================================*/
void CCommunicationHandler::UnregisterEvent(const std::string& rName)
{
  SDecodedAnswer answerToFind(SDecodedAnswer::EVENT_RESULT, rName, 0);
  std::vector<SDecodedAnswer>::iterator iter = std::find(m_SubscribedEvents.begin(),
    m_SubscribedEvents.end(), answerToFind);
  if(iter != m_SubscribedEvents.end())
  {
    iter = m_SubscribedEvents.erase(iter);
  }
}

/*======================================================================================*/
/*! If events are unsubscribe at the device they should also be unregistered here.
    If the given event hasn't been registered before the list of registered events remains 
    unchanged.

    \param uiIndex The event index of the event to unregister.
 */
/*======================================================================================*/
void CCommunicationHandler::UnregisterEvent(SOPAS_UInt16 uiIndex)
{
  SDecodedAnswer answerToFind(SDecodedAnswer::EVENT_RESULT, "", uiIndex);
  std::vector<SDecodedAnswer>::iterator iter = std::find(m_SubscribedEvents.begin(),
    m_SubscribedEvents.end(), answerToFind);
  if(iter != m_SubscribedEvents.end())
  {
    iter = m_SubscribedEvents.erase(iter);
  }
}

/*======================================================================================*/
/*! Asynchronous method calls need to be registered here otherwise the answer is 
    discarded during polling. When the according answer has been received the method
    is unregistered automatically.

    \param rName The communication name of the method to register.
 */
/*======================================================================================*/
void CCommunicationHandler::RegisterAsyncMethod(const std::string& rName)
{
  SDecodedAnswer answer(SDecodedAnswer::METHOD_RESULT, rName, 0);
  m_AsyncMethodRequests.push_back(answer);
}

/*======================================================================================*/
/*! If an asynchronous method call has been registered but was not acknowledged by the 
    device it should be unregistered here. The device did not start execution of
    the asynchronous method. So the according asynchronous answer shall not be accepted 
    here. 
    
    If the given method hasn't been registered before the list of registered methods 
    remains unchanged.

    \param rName The communication name of the method to unregister.
 */
/*======================================================================================*/
void CCommunicationHandler::UnregisterAsyncMethod(const std::string &rName)
{
  SDecodedAnswer answerToFind(SDecodedAnswer::METHOD_RESULT, rName, 0);
  std::vector<SDecodedAnswer>::iterator iter = std::find(m_AsyncMethodRequests.begin(),
    m_AsyncMethodRequests.end(), answerToFind);
  if(iter != m_AsyncMethodRequests.end())
  {
    iter = m_AsyncMethodRequests.erase(iter);
  }
}

/*======================================================================================*/
/*! If an asynchronous method call has been registered but was not acknowledged by the 
    device it should be unregistered here. The device did not start execution of
    the asynchronous method. So the according asynchronous answer shall not be accepted 
    here. 
    
    If the given method hasn't been registered before the list of registered methods 
    remains unchanged.

    \param uiIndex The method index of the method to unregister.
 */
/*======================================================================================*/
void CCommunicationHandler::UnregisterAsyncMethod(SOPAS_UInt16 uiIndex)
{
  SDecodedAnswer answerToFind(SDecodedAnswer::METHOD_RESULT, "", uiIndex);
  std::vector<SDecodedAnswer>::iterator iter = std::find(m_AsyncMethodRequests.begin(),
    m_AsyncMethodRequests.end(), answerToFind);
  if(iter != m_AsyncMethodRequests.end())
  {
    iter = m_AsyncMethodRequests.erase(iter);
  }
}

/*======================================================================================*/
/*! Asynchronous method calls need to be registered here otherwise the answer is 
    discarded during polling. When the according answer has been received the method
    is unregistered automatically.

    \param uiIndex The event index of the method to register.
 */
/*======================================================================================*/
void CCommunicationHandler::RegisterAsyncMethod(SOPAS_UInt16 uiIndex)
{
  SDecodedAnswer answer(SDecodedAnswer::METHOD_RESULT, "", uiIndex);
  m_AsyncMethodRequests.push_back(answer);
}
