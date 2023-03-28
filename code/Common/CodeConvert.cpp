/*
Open Asset Import Library (assimp)
----------------------------------------------------------------------

Copyright (c) 2006-2023, assimp team

All rights reserved.

Redistribution and use of this software in source and binary forms,
with or without modification, are permitted provided that the
following conditions are met:

* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.

* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the
  following disclaimer in the documentation and/or other
  materials provided with the distribution.

* Neither the name of the assimp team, nor the names of its
  contributors may be used to endorse or promote products
  derived from this software without specific prior
  written permission of the assimp team.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------
*/
#include "CodeConvert.h"
#include<assimp/Exceptional.h>
#include <assimp/ByteSwapper.h>
#include <assimp/DefaultLogger.hpp>
#ifdef ASSIMP_USE_HUNTER
#  include <utf8.h>
#else
#  include "../contrib/utf8cpp/source/utf8.h"
#endif

namespace Assimp {

CodeConvert::CodeConvert(std::vector<char> &data) : mData(data) {
    // empty
}

std::vector<char> &CodeConvert::convertToUtf8() {
    // ConversionResult result;
    if (mData.size() < 8) {
        throw DeadlyImportError("File is too small");
    }

    // UTF 8 with BOM
    if ((uint8_t) mData[0] == 0xEF && (uint8_t) mData[1] == 0xBB && (uint8_t) mData[2] == 0xBF) {
        ASSIMP_LOG_DEBUG("Found UTF-8 BOM ...");

        std::copy(mData.begin() + 3, mData.end(), mData.begin());
        mData.resize(mData.size() - 3);
        return mData;
    }

    // UTF 32 BE with BOM
    if (*((uint32_t*) &mData.front()) == 0xFFFE0000) {

        // swap the endianness ..
        for (uint32_t *p = (uint32_t*) &mData.front(), *end = (uint32_t*) &mData.back(); p <= end; ++p) {
            AI_SWAP4P(p);
        }
    }

    // UTF 32 LE with BOM
    if (*((uint32_t*) &mData.front()) == 0x0000FFFE) {
        ASSIMP_LOG_DEBUG("Found UTF-32 BOM ...");

        std::vector<char> output;
        int *ptr = (int *)&mData[0];
        int *end = ptr + (mData.size() / sizeof(int)) + 1;
        utf8::utf32to8(ptr, end, back_inserter(output));
        return mData;
    }

    // UTF 16 BE with BOM
    if (*((uint16_t*) &mData.front()) == 0xFFFE) {
        // Check to ensure no overflow can happen
        if (mData.size() % 2 != 0) {
            return mData;
        }
        // swap the endianness ..
        for (uint16_t *p = (uint16_t *)&mData.front(), *end = (uint16_t *)&mData.back(); p <= end; ++p) {
            ByteSwap::Swap2(p);
        }
    }

    // UTF 16 LE with BOM
    if (*((uint16_t *) &mData.front()) == 0xFEFF) {
        // UTF16 to UTF8
        const uint16_t *sourceStart = (uint16_t*) mData.data();
        const size_t  targetSize= mData.size() * 3u; // enough to encode
        char *targetStart = new char[targetSize];
        std::memset(targetStart, 0, targetSize * sizeof(char));

        utf8::utf16to8(sourceStart, sourceStart + mData.size()/2, targetStart);
        for(size_t i=0; i<targetSize; ++i) {
            mData[i] = targetStart[i];
        }
        delete [] targetStart;
    }

    return mData;
}

} // Namespace Assimp
