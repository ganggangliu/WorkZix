#ifndef __TCP_COMMUNICATION_H__
#define __TCP_COMMUNICATION_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

// Note: This class contains managed CLI-code
// and is not compileable with a standard c++ compiler.

#include "Communication.h"
#include <vcclr.h>

using namespace System::Net;
using namespace System::Net::Sockets;

/*======================================================================================*/
/*! \brief Communication channel for TCP/IP communication.

    \ingroup common

    Implementation of TCP/IP communication. It allows to set IP-address and port.

    \note This implementation uses the .NET-framework and only works in C++/CLI-projects.
 */
/*======================================================================================*/
class CTcpCommunication : public CCommunication
{  
public:
  CTcpCommunication(void);
    //!< Initialize class.

  bool Connect(IPAddress^ targetAddress, int targetPort);
    //!< Connect to SOPAS device using address and port.
  bool Connect(IPAddress^ targetAddress);
    //!< Connect to SOPAS device using SOPAS default port.
  void Disconnect(void);
    //!< Disconnect from SOPAS device.

  // ICommunication interface
  virtual CBinaryDataStream* ReadBytes(void);
    //!< Implementation of CCommunication interface: Read data from connection.
  virtual bool WriteBytes(const CBinaryDataStream& rData);
    //!< Implementation of CCommunication interface: Write data to connection.

private:
  CTcpCommunication(const CTcpCommunication&);
    //!< Do not allow copying
  CTcpCommunication& operator=(const CTcpCommunication&);
    //!< Do not allow copying

  const static int READ_BUFFER_SIZE = 40000;
    //!< \brief Buffer size for read buffer. 
    //!< Should be sufficient to contain scandata to improve performance.

  gcroot<TcpClient^> client;
    //!< Reference to TcpClient object.
  gcroot<NetworkStream^> stream;
    //!< Reference to unerlying NetworkStream object.
};

#endif