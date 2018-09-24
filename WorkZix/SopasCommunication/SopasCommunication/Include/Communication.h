#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

// Forward declarations
class CBinaryDataStream;

/*======================================================================================*/
/*! \brief Defines interface for communication channels.

    \ingroup common

    Create your own communication channel (e.g. TCP/IP or serial) that is used in your
    system by inheriting from this class. The communication channel can then be used
    by the framer- and protocol-classes (see CFramer and CProtocol).

    \note The setup of the communication channel is not defined by this interface.
      The user must call according setup functions and can then attach the communication
      channel to the selected CProtocol instance.
 */
/*======================================================================================*/
class CCommunication
{
public:
  virtual CBinaryDataStream* ReadBytes(void) = 0;
    //!< Read data from communication channel if data is available.
  virtual bool WriteBytes(const CBinaryDataStream& rData) = 0;
    //!< Write data to communication channel.
};

/*======================================================================================*/
/*! \fn virtual CBinaryDataStream* CCommunication::ReadBytes(void) = 0;

    Implement this function for your communication channel to read data from it.
    This method mustn't block the executing thread. Instead it should peek for data
    and return if no data is available for reading on the communication channel. 
    
    The data shall be written to a new CBinaryDataStream object which 
    automatically stores the count of data bytes.

    \note The returned object must be deleted by the caller.
    
    \return A CBinaryDataStream object containing all data read or 0 if no data
      is available.
 */
/*======================================================================================*/

/*======================================================================================*/
/*! \fn virtual bool CCommunication::WriteBytes(const CBinaryDataStream& rData) = 0;

    Implement this function for your communication channel to write data to it.
    This method mustn't block the executing thread.
       
    \param rData The data to send. All data written to the CBinaryDataStream must be sent.
      The maximum stream size is to be ignored.

    \return <em>true</em> if the data could be sent successfully. <em>false</em> otherwise.
    
 */
/*======================================================================================*/
#endif