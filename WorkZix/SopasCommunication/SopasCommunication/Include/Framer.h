#ifndef __FRAMER_H__
#define __FRAMER_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

// Forward declarations
class CBinaryDataStream;

/*======================================================================================*/
/*! \brief Defines interface for protocol framing.

    \ingroup common

    Inherit from this class to create a framing for a protocol (e.g. add message separators
    or checksums). The framing can be selected independently of the protocol. 
    The AddFraming() and RemoveFraming() methods return a new datastream so it's 
    even possible for framings to change the data to be transmitted.
    
    \note There might be framings that won't work for any type of data (e.g. binary). 
      This is not checked and it's up to the user to select a valid pair of protocol
      and framing.
 */
/*======================================================================================*/
class CFramer
{
public:
	virtual CBinaryDataStream* AddFraming(const CBinaryDataStream& rUnframedData) = 0;
    //!< Add framing to given data.
	virtual CBinaryDataStream* RemoveFraming(CBinaryDataStream& rFramedData) = 0;
    //!< Remove framing from given data.
};

/*======================================================================================*/
/*! \fn virtual CBinaryDataStream* CFramer::AddFraming(const CBinaryDataStream& rUnframedData) = 0;

    Implement this function for your framer to add a framing to the given datastream.
    The data will be a complete message to be transmitted. Add a framing according to
    your needs and return it as a new CBinaryDataStream object which automatically 
    stores the count of data bytes.

    \note The returned object must be deleted by the caller.

    \param rUnframedData The source data to be transmitted. 
    
    \return A CBinaryDataStream object containing the framed data to be written to a
      communication channel. <em>null</em> if any kind of error occured.
 */
/*======================================================================================*/

/*======================================================================================*/
/*! \fn virtual CBinaryDataStream* CFramer::RemoveFraming(CBinaryDataStream& rFramedData) = 0;

    Implement this function for your framer to remove a framing from the given datastream.
    The input data will be a block of received data from the communication channel.
    
    After the framing has been removed the data should look like the original input of 
    the AddFraming() method. The unframed data shall be returned as a new CBinaryDataStream 
    object which automatically stores the count of data bytes.

    \note This method must be able to handle partial frames and multiple frames within
      <em>rFramedData</em> because this method is called each time a block of data has
      arrived. 
      - For partial frames the method must store the partial decoded data and
        resume decoding on the next call. 
      - If there is more than one frame within the data this method must return 
        the first decoded frame. It will be called from outside until the <em>rFramedData</em>
        has been processed completely.
      - Invalid data without any valid framing is to be ignored until a valid frame has been
        detected.
    \note The returned object must be deleted by the caller.

    \param rFramedData The source data to be unframed. 
    
    \return A CBinaryDataStream object containing the unframed data. 
      <em>null</em> if no data could be unframed (yet) or invalid data arrived.
 */
/*======================================================================================*/

#endif