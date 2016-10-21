/**
 * \file      FrameEndpoint.h
 * \brief     This file contains the header declaration of class FrameEndpoint
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

#ifndef FRAME_ENDPOINT_H
#define FRAME_ENDPOINT_H

#include <iostream>
#include <memory>
#include <map>
#include <deque>
#include <boost/asio.hpp>
#include <assert.h>
#include "Frame.h"

/*! \class FrameEndpoint
 *  \brief Class FrameEndpoint
 * 
 *  This is a convenience class to take full control over a provided connected TCP socket in order to
 *  receive and transmit used-defined frames. The user has to provide callbacks to be notified on
 *  reception of incoming frames and to be notified if errors occur. Thus, this class offers a
 *  fully asynchronous interface.
 * 
 *  Frame objects representing received frames are created internally. For this purpose, the
 *  user has to register frame factory callbacks. These frame factories are selected according to the
 *  received stream of bytes and offer a very easy interface to the user. Currently, this frame selection
 *  scheme requires that the type of frame can be determined by evaluation of the very first byte of a frame.
 *  If this is not possible, e.g., because multiple types of frames exist which differ in "later" bytes,
 *  this FrameEndpoint class is not suitable!
 */
class FrameEndpoint: public std::enable_shared_from_this<FrameEndpoint>  {
public:
    /*! \brief  The constructor of FrameEndpoint objects
     * 
     *  All internal members are initialized here. The ownership of the provided TCP socket is transferred, so the user MUST NOT
     *  perform any operations on it after registration.
     * 
     *  \param  a_IOService the boost IO service object
     *  \param  a_TcpSocket the connected TCP socket
     *  \param  a_FrameTypeMask the mask used while looking at the "first byte" of each frame to determine its type
     */
    FrameEndpoint(boost::asio::io_service& a_IOService, boost::asio::ip::tcp::socket& a_TcpSocket, uint8_t a_FrameTypeMask = 0xFF): m_IOService(a_IOService), m_TcpSocket(std::move(a_TcpSocket)), m_FrameTypeMask(a_FrameTypeMask) {
        // Initalize all remaining members
        m_SEPState = SEPSTATE_DISCONNECTED;
        m_bWriteInProgress = false;
        m_bShutdown = false;
        m_bStarted = false;
        m_bStopped = false;
        m_bReceiving = false;
        m_BytesInReadBuffer = 0;
        m_ReadBufferOffset = 0;
        m_SendBufferOffset = 0;
    }
    
    /*! \brief  The destructor of FrameEndpoint objects
     * 
     *  On destruction, it is assured that the provided TCP socket is closed correctly. No subsequent callbacks are triggered.
     */
    ~FrameEndpoint() {
        // Drop all callbacks and assure that Close() was called
        m_OnFrameCallback  = nullptr;
        m_OnClosedCallback = nullptr;
        Close();
    }
    
    /*! \brief  Forget all provided frame factory callbacks
     * 
     *  This method is use to clear the list of provided frame factory callbacks. After this, a new and completely different
     *  set of frame factory callbacks can be provided together with a changed frame type mask. This allows changing the
     *  user-defined protocol regarding a TCP socket for which a different protocol was specified earlier.
     * 
     *  \param  a_FrameTypeMask the mask used while looking at the "first byte" of each frame to determine its type
     */
    void ResetFrameFactories(uint8_t a_FrameTypeMask = 0xFF) {
        // Drop all frame factories and copy the new filter mask
        m_FrameFactoryMap.clear();
        m_FrameTypeMask = a_FrameTypeMask;
    }
    
    /*! \brief  Register one subsequent frame factory callback
     * 
     *  This method is use to clear the list of provided frame factory callbacks. After this, a new and completely different
     *  set of frame factory callbacks can be provided together with a changed frame type mask. This allows changing the
     *  user-defined protocol regarding a TCP socket for which a different protocol was specified earlier.
     * 
     *  \param  a_FrameType the frame type together with the frame type mask must match to activate the respective frame factory callback
     *  \param  a_FrameFactory the frame factory callback that is invoked if the frame type matches
     */
    void RegisterFrameFactory(unsigned char a_FrameType, std::function<std::shared_ptr<Frame>(void)> a_FrameFactory) {
        // Check that there is no frame factory for the specified frame type yet. If not then add it.
        assert(a_FrameFactory);
        unsigned char l_EffectiveFrameType = (a_FrameType & m_FrameTypeMask);
        assert(m_FrameFactoryMap.find(l_EffectiveFrameType) == m_FrameFactoryMap.end());
        m_FrameFactoryMap[l_EffectiveFrameType] = a_FrameFactory;
    }
    
    /*! \brief  A getter to query whether this frame endpoint entity was already started
     * 
     *  \retval true the frame endpoint is currently running
     *  \retval false the frame endpoint is currently not running
     *  \return bool indicates whether this frame endpoint entity was already started
     */
    bool GetWasStarted() const {
        return m_bStarted;
    }
    
    /*! \brief  Start the frame endpoint entity
     * 
     *  The frame endpoint starts reading data from the socket and creates a stream of frame objects according to the provided
     *  frame factory callback methods. Frames enqued for transmission are delivered in sequence. Be sure that you DO NOT call
     *  this method twice, i.e., on a frame endpoint entity that is already running. This will trigger an assertion.
     */
    void Start() {
        // There must be at least one frame factory available!
        assert(m_FrameFactoryMap.empty() == false);
        assert(m_bStarted == false);
        assert(m_bStopped == false);
        assert(m_SEPState == SEPSTATE_DISCONNECTED);
        assert(m_bWriteInProgress == false);
        assert(m_bReceiving == false);
        m_SendBufferOffset = 0;
        m_bStarted = true;
        m_SEPState = SEPSTATE_CONNECTED;
        auto self(shared_from_this());
        TriggerNextFrame();
        if (!m_SendQueue.empty()) {
            m_IOService.post([this, self](){ DoWrite(); });
        } // if
    }
    
    /*! \brief  Tear a frame endpoint entity down
     * 
     *  This method should be called instead of Close() if you want to assure a proper teardown sequence of the provided TCP socket.
     *  This assures that all frames enqueued for transmission are delivered before the socket is closed. You SHOULD NOT call
     *  Close() anymore, as this is performed internally as soon as the send queue becomes empty. You SHOULD NOT send subsequent
     *  frames after calling this method as you'll get no feedback whether the respective frame will be transmitted or not.
     */
    void Shutdown() {
        m_bShutdown = true;
    }
    
    /*! \brief  Close the frame endpoint entity
     * 
     *  Invoking this method will instantly stop reception of incoming and transmission of outgoing frames. Frames pending in the send queue
     *  will be lost. No problems arise if this method is called multiple times. Consider calling Shutdown() instead.
     */
    void Close() {
        if (m_bStarted && (!m_bStopped)) {
            m_bStopped = true;
            m_bReceiving = false;
            m_TcpSocket.cancel();
            m_TcpSocket.close();
            if (m_OnClosedCallback) {
                m_OnClosedCallback();
            } // if
        } // if
    }
    
    /*! \brief  Trigger delivery of the next incoming frame
     * 
     *  Invoking this method triggers read and delivery of the next incoming frame. This allows an asynchronous processing of frames.
     *  For such an asynchronous mode of operation, the provided OnFrameCallback must return the value "false" in order to stop automatic
     *  delivery of subsequent frames. This provides unlimited time to consume a frame and stalls the TCP socket, but requires a later call
     *  to this method in order to continue receiving the next incoming frame.
     */
    void TriggerNextFrame() {
        // Checks
        if (m_bReceiving) {
            return;
        } // if
        
        auto self(shared_from_this());
        m_IOService.post([this,self](){
            if ((m_bStarted) && (!m_bStopped) && (m_SEPState == SEPSTATE_CONNECTED)) {
                // Consume all bytes as long as frames are consumed
                bool l_bDeliverSubsequentFrames = true;
                while ((m_ReadBufferOffset < m_BytesInReadBuffer) && (l_bDeliverSubsequentFrames) && (!m_bStopped)) {
                    l_bDeliverSubsequentFrames = EvaluateReadBuffer();
                } // while
                
                if ((m_ReadBufferOffset == m_BytesInReadBuffer) && (l_bDeliverSubsequentFrames) && (!m_bStopped)) {
                    // No bytes available anymore / yet
                    ReadNextChunk();
                } // if
            } // if
        }); // post
    }
    
    /*! \brief  Enqueue a frame for transmission
     * 
     *  Use this method to enqueue a frame for transmission. The user can select between quasi-synchonous and
     *  asynchronous behavior. In the first case, one has to evaluate the return value to check whether a provided
     *  frame was accepted or denied due to a full transmission queue. For asynchronous behavior the user has to provide
     *  a callback method on a per-packet basis that is called after the frame was successfully transmitted.
     *  Currently this callback method is only called on success but never on error, e.g., if the socket is closed.
     * 
     *  \param  a_Frame the frame to send
     *  \param  a_OnSendDoneCallback a callback to be invoked if this frame was sent
     * 
     *  \retval true the frame was successfully enqueued
     *  \retval false there was a problem enqueueing the frame
     *  \return bool indicates whether the frame was successfully enqueued for transmission
     */
    bool SendFrame(const Frame& a_Frame, std::function<void()> a_OnSendDoneCallback = nullptr) {
        if (m_SEPState == SEPSTATE_SHUTDOWN) {
            if (a_OnSendDoneCallback) {
                m_IOService.post([a_OnSendDoneCallback](){ a_OnSendDoneCallback(); });
            } // if

            return false;
        } // if

        // TODO: check size of the queue. If it reaches a specific limit: kill the socket to prevent DoS attacks
        if (m_SendQueue.size() >= 50) {
            if (a_OnSendDoneCallback) {
                m_IOService.post([a_OnSendDoneCallback](){ a_OnSendDoneCallback(); });
            } // if

            // TODO: check what happens if this is caused by an important packet, e.g., a keep alive or an echo response packet
            return false;
        } // if
        
        m_SendQueue.emplace_back(std::make_pair(a_Frame.Serialize(), a_OnSendDoneCallback));
        if ((!m_bWriteInProgress) && (!m_SendQueue.empty()) && (m_SEPState == SEPSTATE_CONNECTED)) {
            DoWrite();
        } // if
        
        return true;
    }
    
    /*! \brief  Provide the callback method for handling of incoming frames
     * 
     *  The user should provide one callback method to be able to handle incoming frames. For each incoming
     *  frame the callback method is called once.
     * 
     *  \param  a_OnFrameCallback the callback method that is invoked on reception of incoming frames
     */
    void SetOnFrameCallback(std::function<bool(std::shared_ptr<Frame>)> a_OnFrameCallback) {
        m_OnFrameCallback = a_OnFrameCallback;
    }

    /*! \brief  Provide the callback method for handling of connection aborts
     * 
     *  The user should provide one callback method to be able to handle error events such as a closed TCP socket.
     * 
     *  \param  a_OnClosedCallback the callback method that is invoked on error or if the socket was closed
     */
    void SetOnClosedCallback(std::function<void()> a_OnClosedCallback) {
        m_OnClosedCallback = a_OnClosedCallback;
    }

private:
    /*! \brief  Internal helper method to trigger reading of data from the TCP socket
     * 
     *  This method contains the asynchonous reader.
     */
    void ReadNextChunk() {
        if (m_bStopped || m_bReceiving) {
            // Already stopped / a later trigger will happen
            return;
        } // if

        assert(m_ReadBufferOffset == m_BytesInReadBuffer);
        m_BytesInReadBuffer = 0;
        m_ReadBufferOffset = 0;
        assert(m_ReadBufferOffset == 0);
        m_bReceiving = true;
        auto self(shared_from_this());
        if (m_bStopped) return;
        m_TcpSocket.async_read_some(boost::asio::buffer(m_ReadBuffer, E_MAX_LENGTH),[this, self](boost::system::error_code a_ErrorCode, std::size_t a_BytesRead) {
            if (a_ErrorCode == boost::asio::error::operation_aborted) return;
            if (m_bStopped) return;
            m_bReceiving = false;
            if (a_ErrorCode) {
                std::cerr << "Read error on TCP socket: " << a_ErrorCode << ", closing" << std::endl;
                Close();
            } else {
                // Evaluate the bytes at hand
                m_BytesInReadBuffer = a_BytesRead;
                TriggerNextFrame();
            } // else
        }); // async_read_some
    }
    
    /*! \brief  Internal helper method to evaluate a received chunk of data read from the TCP socket
     * 
     *  This method delivers incoming frames to the user via the provided callback method. If the user decides
     *  not to accept subsequent frames, i.e., to stall the receiver, this method does not trigger reading.
     * 
     *  \retval true subsequent incoming frames are allowd to be read and handled
     *  \retval false no subsequent incoming frames must be read and handled currently
     *  \return bool indicates whether immediate delivery of subsequent incoming frames is allowed
     */
    bool EvaluateReadBuffer() {
        bool l_bAcceptsSubsequentFrames = true;
        assert(m_BytesInReadBuffer);
        if (!m_IncomingFrame) {
            // No frame is waiting yet... create it
            auto l_FrameFactoryIt = m_FrameFactoryMap.find(m_ReadBuffer[m_ReadBufferOffset] & m_FrameTypeMask);
            if (l_FrameFactoryIt == m_FrameFactoryMap.end()) {
                // Error, no suitable frame factory available!
                std::cerr << "Protocol violation: unknown frame type" << std::endl;
                l_bAcceptsSubsequentFrames = false;
                Close();
            } else {
                // Create new frame
                m_IncomingFrame = l_FrameFactoryIt->second();
                assert(m_IncomingFrame);
            } // else
        } // else

        if (m_IncomingFrame) {
            // Feed the waiting frame with data
            assert(m_IncomingFrame->BytesNeeded() != 0);
            size_t l_BytesAvailable = (m_BytesInReadBuffer - m_ReadBufferOffset);
            if (m_IncomingFrame->ParseBytes(m_ReadBuffer, m_ReadBufferOffset, l_BytesAvailable) == false) {
                // Parser error
                std::cerr << "Protocol violation: invalid frame content" << std::endl;
                l_bAcceptsSubsequentFrames = false;
                Close();
            } else {
                // No error while parsing
                if (m_IncomingFrame->BytesNeeded() == 0) {
                    // The frame is complete
                    if (m_OnFrameCallback) {
                        // Deliver the data packet but maybe stall the receiver
                        l_bAcceptsSubsequentFrames = m_OnFrameCallback(std::move(m_IncomingFrame));
                    } else {
                        m_IncomingFrame.reset();
                    } // else
                } else {
                    // The frame is not complete yet
                    ReadNextChunk();
                } // else
            } // else
        } // if
        
        return l_bAcceptsSubsequentFrames;
    }

    /*! \brief  Internal helper method to trigger sending of data via the TCP socket
     * 
     *  This method contains the asynchonous writer.
     */
    void DoWrite() {
        auto self(shared_from_this());
        if (m_bStopped) return;
        m_bWriteInProgress = true;
        boost::asio::async_write(m_TcpSocket, boost::asio::buffer(&(m_SendQueue.front().first.data()[m_SendBufferOffset]), (m_SendQueue.front().first.size() - m_SendBufferOffset)),
                                 [this, self](boost::system::error_code a_ErrorCode, std::size_t a_BytesSent) {
            if (a_ErrorCode == boost::asio::error::operation_aborted) return;
            if (m_bStopped) return;
            if (!a_ErrorCode) {
                m_SendBufferOffset += a_BytesSent;
                if (m_SendBufferOffset == m_SendQueue.front().first.size()) {
                    // Completed transmission. If a callback was provided, call it now to demand for a subsequent packet
                    if (m_SendQueue.front().second) {
                        m_SendQueue.front().second();
                    } // if

                    // Remove transmitted packet
                    m_SendQueue.pop_front();
                    m_SendBufferOffset = 0;
                    if (!m_SendQueue.empty()) {
                        DoWrite();
                    } else {
                        m_bWriteInProgress = false;
                        if (m_bShutdown) {
                            m_SEPState = SEPSTATE_SHUTDOWN;
                            m_TcpSocket.shutdown(boost::asio::ip::tcp::socket::shutdown_both);
                            Close();
                        } // if
                    } // else
                } else {
                    // Only a partial transmission. We are not done yet.
                    DoWrite();
                } // else
            } else {
                std::cerr << "TCP write error!" << std::endl;
                Close();
            } // else
        }); // async_write
    }

    // Members
    boost::asio::io_service& m_IOService;     //!< The boost IO service object
    boost::asio::ip::tcp::socket m_TcpSocket; //!< The TCP socket
    uint8_t m_FrameTypeMask;                  //!< The currently active frame type mask for frame factory selection
    
    std::shared_ptr<Frame> m_IncomingFrame;   //!< The pending frame that is currently assembled by reading from the TCP socket
    std::deque<std::pair<std::vector<unsigned char>, std::function<void()>>> m_SendQueue; //!< The transmission queue of waiting frames
    size_t m_SendBufferOffset;                //!< To detect and handle partial writes to the TCP socket
    bool m_bWriteInProgress;                  //!< This flag indicates that the TCP writer is currently active
    
    enum { E_MAX_LENGTH = 65535 };
    unsigned char m_ReadBuffer[E_MAX_LENGTH]; //!< The raw data read buffer
    size_t m_BytesInReadBuffer;               //!< The amount of bytes currently stored in the read buffer
    size_t m_ReadBufferOffset;                //!< The read offset within the read buffer
    
    // State
    typedef enum {
        SEPSTATE_DISCONNECTED = 0,
        SEPSTATE_CONNECTED    = 1,
        SEPSTATE_SHUTDOWN     = 2
    } E_SEPSTATE;
    E_SEPSTATE m_SEPState;                    //!< The state of this entity
    bool m_bShutdown;                         //!< This flag indicates that Shutdown() was called
    bool m_bStarted;                          //!< This flag indicates that Start() was called
    bool m_bStopped;                          //!< This flag indicates that Close() was called
    bool m_bReceiving;                        //!< This flag indicates that the TCP reader is currently active
    
    // Callbacks
    std::function<bool(std::shared_ptr<Frame>)> m_OnFrameCallback;  //!< The callback to be invoked on each incoming frame
    std::function<void()>                       m_OnClosedCallback; //!< The callback to be invoked on error or on close
    
    // The frame factories
    std::map<uint8_t, std::function<std::shared_ptr<Frame>(void)>> m_FrameFactoryMap; //!< The provided frame factory callbacks
};

#endif // FRAME_ENDPOINT_H
