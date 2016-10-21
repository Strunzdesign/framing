/**
 * \file      Frame.h
 * \brief     This file contains the header declaration of class Frame
 * \author    Florian Evers, florian-evers@gmx.de
 * \copyright BSD 3 Clause licence
 *
 * Copyright (c) 2016, Florian Evers, florian-evers@gmx.de
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 *     (1) Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer. 
 * 
 *     (2) Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.  
 *     
 *     (3)The name of the author may not be used to
 *     endorse or promote products derived from this software without
 *     specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FRAME_H
#define FRAME_H

#include <stddef.h>
#include <vector>
#include <assert.h>

/*! \class Frame
 *  \brief Class Frame
 * 
 *  This is the base class of arbitrary frames that can be easily sent and received with the help of FrameEndpoint
 *  entities. The serializer and deserializer methods are purely virtual and have to be implemented in the derived
 *  class. Besides that, all members, getters and setters related to the specific fields of a user-defined frame
 *  also have to be provided in the derived class.
 * 
 *  Thus, if creating a derived frame for a specific frame, one solely has to concentrate on the relevant stuff:
 *  - which data has to be communicated via this frame (members) and
 *  - how are the data arranged in the frame, i.e., the frame format, by specifying the serializer and deserializer.
 *  - All sending / receiving / creation of incoming frames entities is performed "under the hood" by a FrameEndpoint entity.
 */
class Frame {
public:
    /*! \brief  The constructor of Frame objects
     * 
     *  All internal members are initialized here. No need for the user of this class to adapt something here.
     */
    Frame(): m_BytesRemaining(0) {}
    
    /*! \brief  The destructor of Frame objects
     */
    virtual ~Frame(){}

    /*! \brief  The purely virtual serializer method
     * 
     *  This purely virtual serializer method has to be implemented in the derived class. Here the user specifies
     *  the format of the frame's fields for frames that are transmitted. This method is solely called by an
     *  FrameEndpoint entity.
     * 
     *  \return A buffer containing the serialized frame ready for transmission
     */
    virtual const std::vector<unsigned char> Serialize() const = 0;
    
    /*! \brief  Query the amount of outstanding data during reception for frame assembly
     * 
     *  This method is called by a FrameEndpoint entity in the phase of reception. It is used to query the amount of
     *  bytes that are required to assemble a full frame of the specific type by reading from a TCP socket.
     *  Depending on the internal structure of a frame it is possible that it read in multiple stages, e.g., if
     *  at first a length field of a known length has to be read followed by the remainder containing a dynamic amount
     *  of bytes. Such a logic has to be implemented in the Deserialize() method of the derived class.
     * 
     *  \return size_t the amount subsequent bytes required to assemble the frame
     */
    size_t BytesNeeded() const { return m_BytesRemaining; }

    /*! \brief  Deliver a chunk of received bytes in the phase of deserialization
     * 
     *  This method is called by a FrameEndpoint entity in the phase of reception. Here, a chunk of data is copied to
     *  an internal buffer for later evaluation within the Deserialze() method. The Deserialize() method is called
     *  automatically if the required amount of bytes is available.
     * 
     *  \param  a_ReadBuffer the incoming buffer of bytes containing the raw byte stream read from the TCP socket
     *  \param  a_ReadBufferOffset the read offset within the provided incoming byte buffer to start with
     *  \param  a_BytesAvailable the remaining amount of bytes in the read buffer to evaluate
     * 
     *  \retval true no error occured
     *  \retval false a protocol violation was detected. The stream is invalid now and the TCP socket must be closed
     *  \return Indicates success or failure of parsing the provided chunk of bytes
     */
    bool ParseBytes(const unsigned char *a_ReadBuffer, size_t &a_ReadBufferOffset, size_t &a_BytesAvailable) {
        // Checks
        assert(a_ReadBuffer);
        assert(a_BytesAvailable);
        assert(m_BytesRemaining);

        // Parse the frame
        bool l_bSuccess = true;
        while ((m_BytesRemaining) && (a_BytesAvailable)) {
            // Determine the amount of bytes to consume
            size_t l_BytesToCopy = m_BytesRemaining;
            if (a_BytesAvailable < l_BytesToCopy) {
                l_BytesToCopy = a_BytesAvailable;
            } // if

            // Consume bytes from the provided buffer
            m_Buffer.insert(m_Buffer.end(), &a_ReadBuffer[a_ReadBufferOffset], (&a_ReadBuffer[a_ReadBufferOffset] + l_BytesToCopy));
            a_ReadBufferOffset += l_BytesToCopy;
            a_BytesAvailable   -= l_BytesToCopy;
            m_BytesRemaining   -= l_BytesToCopy;
            if (m_BytesRemaining == 0) {
                // A subsequent chunk of data is ready
                if (Deserialize() == false) {
                    // Failed to parse the received bytes
                    l_bSuccess = false;
                    break;
                } // else
            } // if
        } // while

        return l_bSuccess;
    }

protected:
    /*! \brief  The purely virtual deserializer method
     * 
     *  This purely virtual deserializer method has to be implemented in the derived class. Here the user specifies
     *  the format of the frame's fields for frames that are received. This method is solely called by the ParseBytes()
     *  method after a specified amount of bytes was successfully received.
     * 
     *  \retval true no error occured
     *  \retval false a protocol violation was detected
     *  \return Indicates success or failure of parsing the provided chunk of bytes
     */
    virtual bool Deserialize() = 0;

    // Members
    std::vector<unsigned char> m_Buffer; //!< The buffer containing partly received frames or higher-layer payload
    size_t m_BytesRemaining; //!< The amount of bytes required to finalize frame assembly during reception
};

#endif // FRAME_H
