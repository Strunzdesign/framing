/**
 * \file      FrameEndpoint.h
 * \brief 
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

class FrameEndpoint: public std::enable_shared_from_this<FrameEndpoint>  {
public:
    // CTOR and DTOR
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
    
    ~FrameEndpoint() {
        // Drop all callbacks and assure that close was called
        m_OnFrameCallback  = nullptr;
        m_OnClosedCallback = nullptr;
        Close();
    }
    
    void ResetFrameFactories(uint8_t a_FrameTypeMask = 0xFF) {
        // Drop all frame factories and copy the new filter mask
        m_FrameFactoryMap.clear();
        m_FrameTypeMask = a_FrameTypeMask;
    }
    
    void RegisterFrameFactory(unsigned char a_FrameType, std::function<std::shared_ptr<Frame>(void)> a_FrameFactory) {
        // Check that there is no frame factory for the specified frame type yet. If not then add it.
        assert(a_FrameFactory);
        unsigned char l_EffectiveFrameType = (a_FrameType & m_FrameTypeMask);
        assert(m_FrameFactoryMap.find(l_EffectiveFrameType) == m_FrameFactoryMap.end());
        m_FrameFactoryMap[l_EffectiveFrameType] = a_FrameFactory;
    }
    
    bool GetWasStarted() const {
        return m_bStarted;
    }
    
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
    
    void Shutdown() {
        m_bShutdown = true;
    }
    
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
    
    void SetOnFrameCallback(std::function<bool(std::shared_ptr<Frame>)> a_OnFrameCallback) {
        m_OnFrameCallback = a_OnFrameCallback;
    }
    
    void SetOnClosedCallback(std::function<void()> a_OnClosedCallback) {
        m_OnClosedCallback = a_OnClosedCallback;
    }

private:
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
    boost::asio::io_service& m_IOService;
    boost::asio::ip::tcp::socket m_TcpSocket;
    uint8_t m_FrameTypeMask;
    
    std::shared_ptr<Frame> m_IncomingFrame;
    std::deque<std::pair<std::vector<unsigned char>, std::function<void()>>> m_SendQueue; // To be transmitted
    size_t m_SendBufferOffset; //!< To detect and handle partial writes to the TCP socket
    bool m_bWriteInProgress;
    
    enum { E_MAX_LENGTH = 65535 };
    unsigned char m_ReadBuffer[E_MAX_LENGTH];
    size_t m_BytesInReadBuffer;
    size_t m_ReadBufferOffset;
    
    // State
    typedef enum {
        SEPSTATE_DISCONNECTED = 0,
        SEPSTATE_CONNECTED    = 1,
        SEPSTATE_SHUTDOWN     = 2
    } E_SEPSTATE;
    E_SEPSTATE m_SEPState;
    bool m_bShutdown;
    bool m_bStarted;
    bool m_bStopped;
    bool m_bReceiving;
    
    // Callbacks
    std::function<bool(std::shared_ptr<Frame>)> m_OnFrameCallback;
    std::function<void()>                       m_OnClosedCallback;
    
    // The frame factories
    std::map<uint8_t, std::function<std::shared_ptr<Frame>(void)>> m_FrameFactoryMap;
};

#endif // FRAME_ENDPOINT_H
