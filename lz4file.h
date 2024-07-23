/*
   LZ4r file library
   Header File
   Copyright (C) 2022, Xiaomi Inc.
   BSD 2-Clause License (http://www.opensource.org/licenses/bsd-license.php)

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are
   met:

       * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
       * Redistributions in binary form must reproduce the above
   copyright notice, this list of conditions and the following disclaimer
   in the documentation and/or other materials provided with the
   distribution.

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

   You can contact the author at :
   - LZ4r source repository : https://github.com/lz4/lz4
   - LZ4r public forum : https://groups.google.com/forum/#!forum/lz4c
*/
#if defined (__cplusplus)
extern "C" {
#endif

#ifndef LZ4rFILE_H
#define LZ4rFILE_H

#include <stdio.h>  /* FILE* */
#include "lz4frame_static.h"

typedef struct LZ4r_readFile_s LZ4r_readFile_t;
typedef struct LZ4r_writeFile_s LZ4r_writeFile_t;

/*! LZ4rF_readOpen() :
 * Set read lz4file handle.
 * `lz4f` will set a lz4file handle.
 * `fp` must be the return value of the lz4 file opened by fopen.
 */
LZ4rFLIB_STATIC_API LZ4rF_errorCode_t LZ4rF_readOpen(LZ4r_readFile_t** lz4fRead, FILE* fp);

/*! LZ4rF_read() :
 * Read lz4file content to buffer.
 * `lz4f` must use LZ4r_readOpen to set first.
 * `buf` read data buffer.
 * `size` read data buffer size.
 */
LZ4rFLIB_STATIC_API size_t LZ4rF_read(LZ4r_readFile_t* lz4fRead, void* buf, size_t size);

/*! LZ4rF_readClose() :
 * Close lz4file handle.
 * `lz4f` must use LZ4r_readOpen to set first.
 */
LZ4rFLIB_STATIC_API LZ4rF_errorCode_t LZ4rF_readClose(LZ4r_readFile_t* lz4fRead);

/*! LZ4rF_writeOpen() :
 * Set write lz4file handle.
 * `lz4f` will set a lz4file handle.
 * `fp` must be the return value of the lz4 file opened by fopen.
 */
LZ4rFLIB_STATIC_API LZ4rF_errorCode_t LZ4rF_writeOpen(LZ4r_writeFile_t** lz4fWrite, FILE* fp, const LZ4rF_preferences_t* prefsPtr);

/*! LZ4rF_write() :
 * Write buffer to lz4file.
 * `lz4f` must use LZ4rF_writeOpen to set first.
 * `buf` write data buffer.
 * `size` write data buffer size.
 */
LZ4rFLIB_STATIC_API size_t LZ4rF_write(LZ4r_writeFile_t* lz4fWrite, const void* buf, size_t size);

/*! LZ4rF_writeClose() :
 * Close lz4file handle.
 * `lz4f` must use LZ4rF_writeOpen to set first.
 */
LZ4rFLIB_STATIC_API LZ4rF_errorCode_t LZ4rF_writeClose(LZ4r_writeFile_t* lz4fWrite);

#endif /* LZ4rFILE_H */

#if defined (__cplusplus)
}
#endif
