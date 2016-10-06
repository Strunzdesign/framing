/**
 * \file      Frame.h
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

#ifndef FRAME_H
#define FRAME_H

#include <stddef.h>
#include <vector>
#include <assert.h>

class Frame {
public:
    // CTOR and DTOR
    Frame(): m_BytesRemaining(0) {}
    virtual ~Frame(){}

    // Serializer and deserializer
    virtual const std::vector<unsigned char> Serialize() const = 0;
    size_t BytesNeeded() const { return m_BytesRemaining; }

    // true: no error, false: parser error
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
    // Internal helpers
    virtual bool Deserialize() = 0;

    // Members
    std::vector<unsigned char> m_Buffer;
    size_t m_BytesRemaining;
};

#endif // FRAME_H
