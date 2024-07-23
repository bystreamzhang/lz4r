/*
 * LZ4r file library
 * Copyright (C) 2022, Xiaomi Inc.
 *
 * BSD 2-Clause License (http://www.opensource.org/licenses/bsd-license.php)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following disclaimer
 *   in the documentation and/or other materials provided with the
 *   distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You can contact the author at :
 * - LZ4r homepage : http://www.lz4.org
 * - LZ4r source repository : https://github.com/lz4/lz4
 */
#include <stdlib.h>  /* malloc, free */
#include <string.h>
#include <assert.h>
#include "lz4r.h"
#include "lz4file.h"

static LZ4rF_errorCode_t returnErrorCode(LZ4rF_errorCodes code)
{
    return (LZ4rF_errorCode_t)-(ptrdiff_t)code;
}
#undef RETURN_ERROR
#define RETURN_ERROR(e) return returnErrorCode(LZ4rF_ERROR_ ## e)

/* =====   read API   ===== */

struct LZ4r_readFile_s {
  LZ4rF_dctx* dctxPtr;
  FILE* fp;
  LZ4r_byte* srcBuf;
  size_t srcBufNext;
  size_t srcBufSize;
  size_t srcBufMaxSize;
};

static void LZ4rF_freeReadFile(LZ4r_readFile_t* lz4fRead)
{
  if (lz4fRead==NULL) return;
  LZ4rF_freeDecompressionContext(lz4fRead->dctxPtr);
  free(lz4fRead->srcBuf);
  free(lz4fRead);
}

static void LZ4rF_freeAndNullReadFile(LZ4r_readFile_t** statePtr)
{
  assert(statePtr != NULL);
  LZ4rF_freeReadFile(*statePtr);
  *statePtr = NULL;
}

LZ4rF_errorCode_t LZ4rF_readOpen(LZ4r_readFile_t** lz4fRead, FILE* fp)
{
  char buf[LZ4rF_HEADER_SIZE_MAX];
  size_t consumedSize;
  LZ4rF_errorCode_t ret;

  if (fp == NULL || lz4fRead == NULL) {
    RETURN_ERROR(parameter_null);
  }

  *lz4fRead = (LZ4r_readFile_t*)calloc(1, sizeof(LZ4r_readFile_t));
  if (*lz4fRead == NULL) {
    RETURN_ERROR(allocation_failed);
  }

  ret = LZ4rF_createDecompressionContext(&(*lz4fRead)->dctxPtr, LZ4rF_VERSION);
  if (LZ4rF_isError(ret)) {
    LZ4rF_freeAndNullReadFile(lz4fRead);
    return ret;
  }

  (*lz4fRead)->fp = fp;
  consumedSize = fread(buf, 1, sizeof(buf), (*lz4fRead)->fp);
  if (consumedSize != sizeof(buf)) {
    LZ4rF_freeAndNullReadFile(lz4fRead);
    RETURN_ERROR(io_read);
  }

  { LZ4rF_frameInfo_t info;
    LZ4rF_errorCode_t const r = LZ4rF_getFrameInfo((*lz4fRead)->dctxPtr, &info, buf, &consumedSize);
    if (LZ4rF_isError(r)) {
      LZ4rF_freeAndNullReadFile(lz4fRead);
      return r;
    }

    switch (info.blockSizeID) {
      case LZ4rF_default :
      case LZ4rF_max64KB :
        (*lz4fRead)->srcBufMaxSize = 64 * 1024;
        break;
      case LZ4rF_max256KB:
        (*lz4fRead)->srcBufMaxSize = 256 * 1024;
        break;
      case LZ4rF_max1MB:
        (*lz4fRead)->srcBufMaxSize = 1 * 1024 * 1024;
        break;
      case LZ4rF_max4MB:
        (*lz4fRead)->srcBufMaxSize = 4 * 1024 * 1024;
        break;
      default:
        LZ4rF_freeAndNullReadFile(lz4fRead);
        RETURN_ERROR(maxBlockSize_invalid);
    }
  }

  (*lz4fRead)->srcBuf = (LZ4r_byte*)malloc((*lz4fRead)->srcBufMaxSize);
  if ((*lz4fRead)->srcBuf == NULL) {
    LZ4rF_freeAndNullReadFile(lz4fRead);
    RETURN_ERROR(allocation_failed);
  }

  (*lz4fRead)->srcBufSize = sizeof(buf) - consumedSize;
  memcpy((*lz4fRead)->srcBuf, buf + consumedSize, (*lz4fRead)->srcBufSize);

  return ret;
}

size_t LZ4rF_read(LZ4r_readFile_t* lz4fRead, void* buf, size_t size)
{
  LZ4r_byte* p = (LZ4r_byte*)buf;
  size_t next = 0;

  if (lz4fRead == NULL || buf == NULL)
    RETURN_ERROR(parameter_null);

  while (next < size) {
    size_t srcsize = lz4fRead->srcBufSize - lz4fRead->srcBufNext;
    size_t dstsize = size - next;
    size_t ret;

    if (srcsize == 0) {
      ret = fread(lz4fRead->srcBuf, 1, lz4fRead->srcBufMaxSize, lz4fRead->fp);
      if (ret > 0) {
        lz4fRead->srcBufSize = ret;
        srcsize = lz4fRead->srcBufSize;
        lz4fRead->srcBufNext = 0;
      } else if (ret == 0) {
        break;
      } else {
        RETURN_ERROR(io_read);
      }
    }

    ret = LZ4rF_decompress(lz4fRead->dctxPtr,
                          p, &dstsize,
                          lz4fRead->srcBuf + lz4fRead->srcBufNext,
                          &srcsize,
                          NULL);
    if (LZ4rF_isError(ret)) {
        return ret;
    }

    lz4fRead->srcBufNext += srcsize;
    next += dstsize;
    p += dstsize;
  }

  return next;
}

LZ4rF_errorCode_t LZ4rF_readClose(LZ4r_readFile_t* lz4fRead)
{
  if (lz4fRead == NULL)
    RETURN_ERROR(parameter_null);
  LZ4rF_freeReadFile(lz4fRead);
  return LZ4rF_OK_NoError;
}

/* =====   write API   ===== */

struct LZ4r_writeFile_s {
  LZ4rF_cctx* cctxPtr;
  FILE* fp;
  LZ4r_byte* dstBuf;
  size_t maxWriteSize;
  size_t dstBufMaxSize;
  LZ4rF_errorCode_t errCode;
};

static void LZ4rF_freeWriteFile(LZ4r_writeFile_t* state)
{
  if (state == NULL) return;
  LZ4rF_freeCompressionContext(state->cctxPtr);
  free(state->dstBuf);
  free(state);
}

static void LZ4rF_freeAndNullWriteFile(LZ4r_writeFile_t** statePtr)
{
  assert(statePtr != NULL);
  LZ4rF_freeWriteFile(*statePtr);
  *statePtr = NULL;
}

LZ4rF_errorCode_t LZ4rF_writeOpen(LZ4r_writeFile_t** lz4fWrite, FILE* fp, const LZ4rF_preferences_t* prefsPtr)
{
  LZ4r_byte buf[LZ4rF_HEADER_SIZE_MAX];
  size_t ret;

  if (fp == NULL || lz4fWrite == NULL)
    RETURN_ERROR(parameter_null);

  *lz4fWrite = (LZ4r_writeFile_t*)calloc(1, sizeof(LZ4r_writeFile_t));
  if (*lz4fWrite == NULL) {
    RETURN_ERROR(allocation_failed);
  }
  if (prefsPtr != NULL) {
    switch (prefsPtr->frameInfo.blockSizeID) {
      case LZ4rF_default :
      case LZ4rF_max64KB :
        (*lz4fWrite)->maxWriteSize = 64 * 1024;
        break;
      case LZ4rF_max256KB:
        (*lz4fWrite)->maxWriteSize = 256 * 1024;
        break;
      case LZ4rF_max1MB:
        (*lz4fWrite)->maxWriteSize = 1 * 1024 * 1024;
        break;
      case LZ4rF_max4MB:
        (*lz4fWrite)->maxWriteSize = 4 * 1024 * 1024;
        break;
      default:
        LZ4rF_freeAndNullWriteFile(lz4fWrite);
        RETURN_ERROR(maxBlockSize_invalid);
      }
    } else {
      (*lz4fWrite)->maxWriteSize = 64 * 1024;
    }

  (*lz4fWrite)->dstBufMaxSize = LZ4rF_compressBound((*lz4fWrite)->maxWriteSize, prefsPtr);
  (*lz4fWrite)->dstBuf = (LZ4r_byte*)malloc((*lz4fWrite)->dstBufMaxSize);
  if ((*lz4fWrite)->dstBuf == NULL) {
    LZ4rF_freeAndNullWriteFile(lz4fWrite);
    RETURN_ERROR(allocation_failed);
  }

  ret = LZ4rF_createCompressionContext(&(*lz4fWrite)->cctxPtr, LZ4rF_VERSION);
  if (LZ4rF_isError(ret)) {
      LZ4rF_freeAndNullWriteFile(lz4fWrite);
      return ret;
  }

  ret = LZ4rF_compressBegin((*lz4fWrite)->cctxPtr, buf, LZ4rF_HEADER_SIZE_MAX, prefsPtr);
  if (LZ4rF_isError(ret)) {
      LZ4rF_freeAndNullWriteFile(lz4fWrite);
      return ret;
  }

  if (ret != fwrite(buf, 1, ret, fp)) {
    LZ4rF_freeAndNullWriteFile(lz4fWrite);
    RETURN_ERROR(io_write);
  }

  (*lz4fWrite)->fp = fp;
  (*lz4fWrite)->errCode = LZ4rF_OK_NoError;
  return LZ4rF_OK_NoError;
}

size_t LZ4rF_write(LZ4r_writeFile_t* lz4fWrite, const void* buf, size_t size)
{
  const LZ4r_byte* p = (const LZ4r_byte*)buf;
  size_t remain = size;
  size_t chunk;
  size_t ret;

  if (lz4fWrite == NULL || buf == NULL)
    RETURN_ERROR(parameter_null);
  while (remain) {
    if (remain > lz4fWrite->maxWriteSize)
      chunk = lz4fWrite->maxWriteSize;
    else
      chunk = remain;

    ret = LZ4rF_compressUpdate(lz4fWrite->cctxPtr,
                              lz4fWrite->dstBuf, lz4fWrite->dstBufMaxSize,
                              p, chunk,
                              NULL);
    if (LZ4rF_isError(ret)) {
      lz4fWrite->errCode = ret;
      return ret;
    }

    if (ret != fwrite(lz4fWrite->dstBuf, 1, ret, lz4fWrite->fp)) {
      lz4fWrite->errCode = returnErrorCode(LZ4rF_ERROR_io_write);
      RETURN_ERROR(io_write);
    }

    p += chunk;
    remain -= chunk;
  }

  return size;
}

LZ4rF_errorCode_t LZ4rF_writeClose(LZ4r_writeFile_t* lz4fWrite)
{
  LZ4rF_errorCode_t ret = LZ4rF_OK_NoError;

  if (lz4fWrite == NULL) {
    RETURN_ERROR(parameter_null);
  }

  if (lz4fWrite->errCode == LZ4rF_OK_NoError) {
    ret =  LZ4rF_compressEnd(lz4fWrite->cctxPtr,
                            lz4fWrite->dstBuf, lz4fWrite->dstBufMaxSize,
                            NULL);
    if (LZ4rF_isError(ret)) {
      goto out;
    }

    if (ret != fwrite(lz4fWrite->dstBuf, 1, ret, lz4fWrite->fp)) {
      ret = returnErrorCode(LZ4rF_ERROR_io_write);
    }
  }

out:
  LZ4rF_freeWriteFile(lz4fWrite);
  return ret;
}
