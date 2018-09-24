#ifndef __SERIAL_COMMUNICATION_H__
#define __SERIAL_COMMUNICATION_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

// Note: This class contains managed CLI-code
// and is not compileable with a standard c++ compiler.

#include "Communication.h"
#include <vcclr.h>

using namespace System::IO::Ports;
using namespace System;

/*======================================================================================*/
/*! \brief Communication channel for serial communication.

    \ingroup common

    Implementation of serial communication. It allows to set port, baud rate, parity,
    data bits and stop bits.

    \note This implementation uses the .NET-framework and only works in C++/CLI-projects.
 */
/*======================================================================================*/
class CSerialCommunication : public CCommunication
{
public:
  CSerialCommunication(void);
    //!< Initialize class.

  bool Connect(String^ portName, int baudRate, Parity parity, int dataBits, StopBits stopBits);
    //!< Open serial port using given parameters.
  bool Connect(String^ portName, int baudRate);
    //!< Open serial port using default parameters.
  void Disconnect(void);
    //!< Close serial port.

  // ICommunication interface
  virtual CBinaryDataStream* ReadBytes(void);
    //!< Implementation of CCommunication interface: Read data from serial port.
  virtual bool WriteBytes(const CBinaryDataStream& rData);
    //!< Implementation of CCommunication interface: Write data to serial port.

private:
  CSerialCommunication(const CSerialCommunication&);
    //!< Do not allow copying
  CSerialCommunication& operator=(const CSerialCommunication&);
    //!< Do not allow copying

  const static int READ_BUFFER_SIZE = 40000;
    //!< \brief Buffer size for read buffer. 
    //!< Should be sufficient to contain scandata to improve performance.

  gcroot<SerialPort^> port;
    //!< Reference to SerialPort object.
};

#endif