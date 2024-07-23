/*
   LZ4rF - LZ4r-Frame library
   Header File
   Copyright (C) 2011-2020, Yann Collet.
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

/* LZ4rF is a stand-alone API able to create and decode LZ4r frames
 * conformant with specification v1.6.1 in doc/lz4_Frame_format.md .
 * Generated frames are compatible with `lz4` CLI.
 *
 * LZ4rF also offers streaming capabilities.
 *
 * lz4r.h is not required when using lz4frame.h,
 * except to extract common constants such as LZ4r_VERSION_NUMBER.
 * */

#ifndef LZ4rF_H_09782039843
#define LZ4rF_H_09782039843

#if defined (__cplusplus)
extern "C" {
#endif

/* ---   Dependency   --- */
#include <stddef.h>   /* size_t */


/**
 * Introduction
 *
 * lz4frame.h implements LZ4r frame specification: see doc/lz4_Frame_format.md .
 * LZ4r Frames are compatible with `lz4` CLI,
 * and designed to be interoperable with any system.
**/

/*-***************************************************************
 *  Compiler specifics
 *****************************************************************/
/*  LZ4r_DLL_EXPORT :
 *  Enable exporting of functions when building a Windows DLL
 *  LZ4rFLIB_VISIBILITY :
 *  Control library symbols visibility.
 */
#ifndef LZ4rFLIB_VISIBILITY
#  if defined(__GNUC__) && (__GNUC__ >= 4)
#    define LZ4rFLIB_VISIBILITY __attribute__ ((visibility ("default")))
#  else
#    define LZ4rFLIB_VISIBILITY
#  endif
#endif
#if defined(LZ4r_DLL_EXPORT) && (LZ4r_DLL_EXPORT==1)
#  define LZ4rFLIB_API __declspec(dllexport) LZ4rFLIB_VISIBILITY
#elif defined(LZ4r_DLL_IMPORT) && (LZ4r_DLL_IMPORT==1)
#  define LZ4rFLIB_API __declspec(dllimport) LZ4rFLIB_VISIBILITY
#else
#  define LZ4rFLIB_API LZ4rFLIB_VISIBILITY
#endif

#ifdef LZ4rF_DISABLE_DEPRECATE_WARNINGS
#  define LZ4rF_DEPRECATE(x) x
#else
#  if defined(_MSC_VER)
#    define LZ4rF_DEPRECATE(x) x   /* __declspec(deprecated) x - only works with C++ */
#  elif defined(__clang__) || (defined(__GNUC__) && (__GNUC__ >= 6))
#    define LZ4rF_DEPRECATE(x) x __attribute__((deprecated))
#  else
#    define LZ4rF_DEPRECATE(x) x   /* no deprecation warning for this compiler */
#  endif
#endif


/*-************************************
 *  Error management
 **************************************/
typedef size_t LZ4rF_errorCode_t;

LZ4rFLIB_API unsigned    LZ4rF_isError(LZ4rF_errorCode_t code);   /**< tells when a function result is an error code */
LZ4rFLIB_API const char* LZ4rF_getErrorName(LZ4rF_errorCode_t code);   /**< return error code string; for debugging */


/*-************************************
 *  Frame compression types
 ************************************* */
/* #define LZ4rF_ENABLE_OBSOLETE_ENUMS   // uncomment to enable obsolete enums */
#ifdef LZ4rF_ENABLE_OBSOLETE_ENUMS
#  define LZ4rF_OBSOLETE_ENUM(x) , LZ4rF_DEPRECATE(x) = LZ4rF_##x
#else
#  define LZ4rF_OBSOLETE_ENUM(x)
#endif

/* The larger the block size, the (slightly) better the compression ratio,
 * though there are diminishing returns.
 * Larger blocks also increase memory usage on both compression and decompression sides.
 */
typedef enum {
    LZ4rF_default=0,
    LZ4rF_max64KB=4,
    LZ4rF_max256KB=5,
    LZ4rF_max1MB=6,
    LZ4rF_max4MB=7
    LZ4rF_OBSOLETE_ENUM(max64KB)
    LZ4rF_OBSOLETE_ENUM(max256KB)
    LZ4rF_OBSOLETE_ENUM(max1MB)
    LZ4rF_OBSOLETE_ENUM(max4MB)
} LZ4rF_blockSizeID_t;

/* Linked blocks sharply reduce inefficiencies when using small blocks,
 * they compress better.
 * However, some LZ4r decoders are only compatible with independent blocks */
typedef enum {
    LZ4rF_blockLinked=0,
    LZ4rF_blockIndependent
    LZ4rF_OBSOLETE_ENUM(blockLinked)
    LZ4rF_OBSOLETE_ENUM(blockIndependent)
} LZ4rF_blockMode_t;

typedef enum {
    LZ4rF_noContentChecksum=0,
    LZ4rF_contentChecksumEnabled
    LZ4rF_OBSOLETE_ENUM(noContentChecksum)
    LZ4rF_OBSOLETE_ENUM(contentChecksumEnabled)
} LZ4rF_contentChecksum_t;

typedef enum {
    LZ4rF_noBlockChecksum=0,
    LZ4rF_blockChecksumEnabled
} LZ4rF_blockChecksum_t;

typedef enum {
    LZ4rF_frame=0,
    LZ4rF_skippableFrame
    LZ4rF_OBSOLETE_ENUM(skippableFrame)
} LZ4rF_frameType_t;

#ifdef LZ4rF_ENABLE_OBSOLETE_ENUMS
typedef LZ4rF_blockSizeID_t blockSizeID_t;
typedef LZ4rF_blockMode_t blockMode_t;
typedef LZ4rF_frameType_t frameType_t;
typedef LZ4rF_contentChecksum_t contentChecksum_t;
#endif

/*! LZ4rF_frameInfo_t :
 *  makes it possible to set or read frame parameters.
 *  Structure must be first init to 0, using memset() or LZ4rF_INIT_FRAMEINFO,
 *  setting all parameters to default.
 *  It's then possible to update selectively some parameters */
typedef struct {
  LZ4rF_blockSizeID_t     blockSizeID;         /* max64KB, max256KB, max1MB, max4MB; 0 == default (LZ4rF_max64KB) */
  LZ4rF_blockMode_t       blockMode;           /* LZ4rF_blockLinked, LZ4rF_blockIndependent; 0 == default (LZ4rF_blockLinked) */
  LZ4rF_contentChecksum_t contentChecksumFlag; /* 1: add a 32-bit checksum of frame's decompressed data; 0 == default (disabled) */
  LZ4rF_frameType_t       frameType;           /* read-only field : LZ4rF_frame or LZ4rF_skippableFrame */
  unsigned long long     contentSize;         /* Size of uncompressed content ; 0 == unknown */
  unsigned               dictID;              /* Dictionary ID, sent by compressor to help decoder select correct dictionary; 0 == no dictID provided */
  LZ4rF_blockChecksum_t   blockChecksumFlag;   /* 1: each block followed by a checksum of block's compressed data; 0 == default (disabled) */
} LZ4rF_frameInfo_t;

#define LZ4rF_INIT_FRAMEINFO   { LZ4rF_max64KB, LZ4rF_blockLinked, LZ4rF_noContentChecksum, LZ4rF_frame, 0ULL, 0U, LZ4rF_noBlockChecksum }    /* v1.8.3+ */

/*! LZ4rF_preferences_t :
 *  makes it possible to supply advanced compression instructions to streaming interface.
 *  Structure must be first init to 0, using memset() or LZ4rF_INIT_PREFERENCES,
 *  setting all parameters to default.
 *  All reserved fields must be set to zero. */
typedef struct {
  LZ4rF_frameInfo_t frameInfo;
  int      compressionLevel;    /* 0: default (fast mode); values > LZ4rHC_CLEVEL_MAX count as LZ4rHC_CLEVEL_MAX; values < 0 trigger "fast acceleration" */
  unsigned autoFlush;           /* 1: always flush; reduces usage of internal buffers */
  unsigned favorDecSpeed;       /* 1: parser favors decompression speed vs compression ratio. Only works for high compression modes (>= LZ4rHC_CLEVEL_OPT_MIN) */  /* v1.8.2+ */
  unsigned reserved[3];         /* must be zero for forward compatibility */
} LZ4rF_preferences_t;

#define LZ4rF_INIT_PREFERENCES   { LZ4rF_INIT_FRAMEINFO, 0, 0u, 0u, { 0u, 0u, 0u } }    /* v1.8.3+ */


/*-*********************************
*  Simple compression function
***********************************/

/*! LZ4rF_compressFrame() :
 *  Compress srcBuffer content into an LZ4r-compressed frame.
 *  It's a one shot operation, all input content is consumed, and all output is generated.
 *
 *  Note : it's a stateless operation (no LZ4rF_cctx state needed).
 *  In order to reduce load on the allocator, LZ4rF_compressFrame(), by default,
 *  uses the stack to allocate space for the compression state and some table.
 *  If this usage of the stack is too much for your application,
 *  consider compiling `lz4frame.c` with compile-time macro LZ4rF_HEAPMODE set to 1 instead.
 *  All state allocations will use the Heap.
 *  It also means each invocation of LZ4rF_compressFrame() will trigger several internal alloc/free invocations.
 *
 * @dstCapacity MUST be >= LZ4rF_compressFrameBound(srcSize, preferencesPtr).
 * @preferencesPtr is optional : one can provide NULL, in which case all preferences are set to default.
 * @return : number of bytes written into dstBuffer.
 *           or an error code if it fails (can be tested using LZ4rF_isError())
 */
LZ4rFLIB_API size_t LZ4rF_compressFrame(void* dstBuffer, size_t dstCapacity,
                                const void* srcBuffer, size_t srcSize,
                                const LZ4rF_preferences_t* preferencesPtr);

/*! LZ4rF_compressFrameBound() :
 *  Returns the maximum possible compressed size with LZ4rF_compressFrame() given srcSize and preferences.
 * `preferencesPtr` is optional. It can be replaced by NULL, in which case, the function will assume default preferences.
 *  Note : this result is only usable with LZ4rF_compressFrame().
 *         It may also be relevant to LZ4rF_compressUpdate() _only if_ no flush() operation is ever performed.
 */
LZ4rFLIB_API size_t LZ4rF_compressFrameBound(size_t srcSize, const LZ4rF_preferences_t* preferencesPtr);


/*! LZ4rF_compressionLevel_max() :
 * @return maximum allowed compression level (currently: 12)
 */
LZ4rFLIB_API int LZ4rF_compressionLevel_max(void);   /* v1.8.0+ */


/*-***********************************
*  Advanced compression functions
*************************************/
typedef struct LZ4rF_cctx_s LZ4rF_cctx;   /* incomplete type */
typedef LZ4rF_cctx* LZ4rF_compressionContext_t;  /* for compatibility with older APIs, prefer using LZ4rF_cctx */

typedef struct {
  unsigned stableSrc;    /* 1 == src content will remain present on future calls to LZ4rF_compress(); skip copying src content within tmp buffer */
  unsigned reserved[3];
} LZ4rF_compressOptions_t;

/*---   Resource Management   ---*/

#define LZ4rF_VERSION 100    /* This number can be used to check for an incompatible API breaking change */
LZ4rFLIB_API unsigned LZ4rF_getVersion(void);

/*! LZ4rF_createCompressionContext() :
 *  The first thing to do is to create a compressionContext object,
 *  which will keep track of operation state during streaming compression.
 *  This is achieved using LZ4rF_createCompressionContext(), which takes as argument a version,
 *  and a pointer to LZ4rF_cctx*, to write the resulting pointer into.
 *  @version provided MUST be LZ4rF_VERSION. It is intended to track potential version mismatch, notably when using DLL.
 *  The function provides a pointer to a fully allocated LZ4rF_cctx object.
 *  @cctxPtr MUST be != NULL.
 *  If @return != zero, context creation failed.
 *  A created compression context can be employed multiple times for consecutive streaming operations.
 *  Once all streaming compression jobs are completed,
 *  the state object can be released using LZ4rF_freeCompressionContext().
 *  Note1 : LZ4rF_freeCompressionContext() is always successful. Its return value can be ignored.
 *  Note2 : LZ4rF_freeCompressionContext() works fine with NULL input pointers (do nothing).
**/
LZ4rFLIB_API LZ4rF_errorCode_t LZ4rF_createCompressionContext(LZ4rF_cctx** cctxPtr, unsigned version);
LZ4rFLIB_API LZ4rF_errorCode_t LZ4rF_freeCompressionContext(LZ4rF_cctx* cctx);


/*----    Compression    ----*/

#define LZ4rF_HEADER_SIZE_MIN  7   /* LZ4r Frame header size can vary, depending on selected parameters */
#define LZ4rF_HEADER_SIZE_MAX 19

/* Size in bytes of a block header in little-endian format. Highest bit indicates if block data is uncompressed */
#define LZ4rF_BLOCK_HEADER_SIZE 4

/* Size in bytes of a block checksum footer in little-endian format. */
#define LZ4rF_BLOCK_CHECKSUM_SIZE 4

/* Size in bytes of the content checksum. */
#define LZ4rF_CONTENT_CHECKSUM_SIZE 4

/*! LZ4rF_compressBegin() :
 *  will write the frame header into dstBuffer.
 *  dstCapacity must be >= LZ4rF_HEADER_SIZE_MAX bytes.
 * `prefsPtr` is optional : NULL can be provided to set all preferences to default.
 * @return : number of bytes written into dstBuffer for the header
 *           or an error code (which can be tested using LZ4rF_isError())
 */
LZ4rFLIB_API size_t LZ4rF_compressBegin(LZ4rF_cctx* cctx,
                                      void* dstBuffer, size_t dstCapacity,
                                      const LZ4rF_preferences_t* prefsPtr);

/*! LZ4rF_compressBound() :
 *  Provides minimum dstCapacity required to guarantee success of
 *  LZ4rF_compressUpdate(), given a srcSize and preferences, for a worst case scenario.
 *  When srcSize==0, LZ4rF_compressBound() provides an upper bound for LZ4rF_flush() and LZ4rF_compressEnd() instead.
 *  Note that the result is only valid for a single invocation of LZ4rF_compressUpdate().
 *  When invoking LZ4rF_compressUpdate() multiple times,
 *  if the output buffer is gradually filled up instead of emptied and re-used from its start,
 *  one must check if there is enough remaining capacity before each invocation, using LZ4rF_compressBound().
 * @return is always the same for a srcSize and prefsPtr.
 *  prefsPtr is optional : when NULL is provided, preferences will be set to cover worst case scenario.
 *  tech details :
 * @return if automatic flushing is not enabled, includes the possibility that internal buffer might already be filled by up to (blockSize-1) bytes.
 *  It also includes frame footer (ending + checksum), since it might be generated by LZ4rF_compressEnd().
 * @return doesn't include frame header, as it was already generated by LZ4rF_compressBegin().
 */
LZ4rFLIB_API size_t LZ4rF_compressBound(size_t srcSize, const LZ4rF_preferences_t* prefsPtr);

/*! LZ4rF_compressUpdate() :
 *  LZ4rF_compressUpdate() can be called repetitively to compress as much data as necessary.
 *  Important rule: dstCapacity MUST be large enough to ensure operation success even in worst case situations.
 *  This value is provided by LZ4rF_compressBound().
 *  If this condition is not respected, LZ4rF_compress() will fail (result is an errorCode).
 *  After an error, the state is left in a UB state, and must be re-initialized or freed.
 *  If previously an uncompressed block was written, buffered data is flushed
 *  before appending compressed data is continued.
 * `cOptPtr` is optional : NULL can be provided, in which case all options are set to default.
 * @return : number of bytes written into `dstBuffer` (it can be zero, meaning input data was just buffered).
 *           or an error code if it fails (which can be tested using LZ4rF_isError())
 */
LZ4rFLIB_API size_t LZ4rF_compressUpdate(LZ4rF_cctx* cctx,
                                       void* dstBuffer, size_t dstCapacity,
                                 const void* srcBuffer, size_t srcSize,
                                 const LZ4rF_compressOptions_t* cOptPtr);

/*! LZ4rF_flush() :
 *  When data must be generated and sent immediately, without waiting for a block to be completely filled,
 *  it's possible to call LZ4r_flush(). It will immediately compress any data buffered within cctx.
 * `dstCapacity` must be large enough to ensure the operation will be successful.
 * `cOptPtr` is optional : it's possible to provide NULL, all options will be set to default.
 * @return : nb of bytes written into dstBuffer (can be zero, when there is no data stored within cctx)
 *           or an error code if it fails (which can be tested using LZ4rF_isError())
 *  Note : LZ4rF_flush() is guaranteed to be successful when dstCapacity >= LZ4rF_compressBound(0, prefsPtr).
 */
LZ4rFLIB_API size_t LZ4rF_flush(LZ4rF_cctx* cctx,
                              void* dstBuffer, size_t dstCapacity,
                        const LZ4rF_compressOptions_t* cOptPtr);

/*! LZ4rF_compressEnd() :
 *  To properly finish an LZ4r frame, invoke LZ4rF_compressEnd().
 *  It will flush whatever data remained within `cctx` (like LZ4r_flush())
 *  and properly finalize the frame, with an endMark and a checksum.
 * `cOptPtr` is optional : NULL can be provided, in which case all options will be set to default.
 * @return : nb of bytes written into dstBuffer, necessarily >= 4 (endMark),
 *           or an error code if it fails (which can be tested using LZ4rF_isError())
 *  Note : LZ4rF_compressEnd() is guaranteed to be successful when dstCapacity >= LZ4rF_compressBound(0, prefsPtr).
 *  A successful call to LZ4rF_compressEnd() makes `cctx` available again for another compression task.
 */
LZ4rFLIB_API size_t LZ4rF_compressEnd(LZ4rF_cctx* cctx,
                                    void* dstBuffer, size_t dstCapacity,
                              const LZ4rF_compressOptions_t* cOptPtr);


/*-*********************************
*  Decompression functions
***********************************/
typedef struct LZ4rF_dctx_s LZ4rF_dctx;   /* incomplete type */
typedef LZ4rF_dctx* LZ4rF_decompressionContext_t;   /* compatibility with previous API versions */

typedef struct {
  unsigned stableDst;     /* pledges that last 64KB decompressed data is present right before @dstBuffer pointer.
                           * This optimization skips internal storage operations.
                           * Once set, this pledge must remain valid up to the end of current frame. */
  unsigned skipChecksums; /* disable checksum calculation and verification, even when one is present in frame, to save CPU time.
                           * Setting this option to 1 once disables all checksums for the rest of the frame. */
  unsigned reserved1;     /* must be set to zero for forward compatibility */
  unsigned reserved0;     /* idem */
} LZ4rF_decompressOptions_t;


/* Resource management */

/*! LZ4rF_createDecompressionContext() :
 *  Create an LZ4rF_dctx object, to track all decompression operations.
 *  @version provided MUST be LZ4rF_VERSION.
 *  @dctxPtr MUST be valid.
 *  The function fills @dctxPtr with the value of a pointer to an allocated and initialized LZ4rF_dctx object.
 *  The @return is an errorCode, which can be tested using LZ4rF_isError().
 *  dctx memory can be released using LZ4rF_freeDecompressionContext();
 *  Result of LZ4rF_freeDecompressionContext() indicates current state of decompressionContext when being released.
 *  That is, it should be == 0 if decompression has been completed fully and correctly.
 */
LZ4rFLIB_API LZ4rF_errorCode_t LZ4rF_createDecompressionContext(LZ4rF_dctx** dctxPtr, unsigned version);
LZ4rFLIB_API LZ4rF_errorCode_t LZ4rF_freeDecompressionContext(LZ4rF_dctx* dctx);


/*-***********************************
*  Streaming decompression functions
*************************************/

#define LZ4rF_MAGICNUMBER 0x184D2204U
#define LZ4rF_MAGIC_SKIPPABLE_START 0x184D2A50U
#define LZ4rF_MIN_SIZE_TO_KNOW_HEADER_LENGTH 5

/*! LZ4rF_headerSize() : v1.9.0+
 *  Provide the header size of a frame starting at `src`.
 * `srcSize` must be >= LZ4rF_MIN_SIZE_TO_KNOW_HEADER_LENGTH,
 *  which is enough to decode the header length.
 * @return : size of frame header
 *           or an error code, which can be tested using LZ4rF_isError()
 *  note : Frame header size is variable, but is guaranteed to be
 *         >= LZ4rF_HEADER_SIZE_MIN bytes, and <= LZ4rF_HEADER_SIZE_MAX bytes.
 */
LZ4rFLIB_API size_t LZ4rF_headerSize(const void* src, size_t srcSize);

/*! LZ4rF_getFrameInfo() :
 *  This function extracts frame parameters (max blockSize, dictID, etc.).
 *  Its usage is optional: user can also invoke LZ4rF_decompress() directly.
 *
 *  Extracted information will fill an existing LZ4rF_frameInfo_t structure.
 *  This can be useful for allocation and dictionary identification purposes.
 *
 *  LZ4rF_getFrameInfo() can work in the following situations :
 *
 *  1) At the beginning of a new frame, before any invocation of LZ4rF_decompress().
 *     It will decode header from `srcBuffer`,
 *     consuming the header and starting the decoding process.
 *
 *     Input size must be large enough to contain the full frame header.
 *     Frame header size can be known beforehand by LZ4rF_headerSize().
 *     Frame header size is variable, but is guaranteed to be >= LZ4rF_HEADER_SIZE_MIN bytes,
 *     and not more than <= LZ4rF_HEADER_SIZE_MAX bytes.
 *     Hence, blindly providing LZ4rF_HEADER_SIZE_MAX bytes or more will always work.
 *     It's allowed to provide more input data than the header size,
 *     LZ4rF_getFrameInfo() will only consume the header.
 *
 *     If input size is not large enough,
 *     aka if it's smaller than header size,
 *     function will fail and return an error code.
 *
 *  2) After decoding has been started,
 *     it's possible to invoke LZ4rF_getFrameInfo() anytime
 *     to extract already decoded frame parameters stored within dctx.
 *
 *     Note that, if decoding has barely started,
 *     and not yet read enough information to decode the header,
 *     LZ4rF_getFrameInfo() will fail.
 *
 *  The number of bytes consumed from srcBuffer will be updated in *srcSizePtr (necessarily <= original value).
 *  LZ4rF_getFrameInfo() only consumes bytes when decoding has not yet started,
 *  and when decoding the header has been successful.
 *  Decompression must then resume from (srcBuffer + *srcSizePtr).
 *
 * @return : a hint about how many srcSize bytes LZ4rF_decompress() expects for next call,
 *           or an error code which can be tested using LZ4rF_isError().
 *  note 1 : in case of error, dctx is not modified. Decoding operation can resume from beginning safely.
 *  note 2 : frame parameters are *copied into* an already allocated LZ4rF_frameInfo_t structure.
 */
LZ4rFLIB_API size_t
LZ4rF_getFrameInfo(LZ4rF_dctx* dctx,
                  LZ4rF_frameInfo_t* frameInfoPtr,
            const void* srcBuffer, size_t* srcSizePtr);

/*! LZ4rF_decompress() :
 *  Call this function repetitively to regenerate data compressed in `srcBuffer`.
 *
 *  The function requires a valid dctx state.
 *  It will read up to *srcSizePtr bytes from srcBuffer,
 *  and decompress data into dstBuffer, of capacity *dstSizePtr.
 *
 *  The nb of bytes consumed from srcBuffer will be written into *srcSizePtr (necessarily <= original value).
 *  The nb of bytes decompressed into dstBuffer will be written into *dstSizePtr (necessarily <= original value).
 *
 *  The function does not necessarily read all input bytes, so always check value in *srcSizePtr.
 *  Unconsumed source data must be presented again in subsequent invocations.
 *
 * `dstBuffer` can freely change between each consecutive function invocation.
 * `dstBuffer` content will be overwritten.
 *
 *  Note: if `LZ4rF_getFrameInfo()` is called before `LZ4rF_decompress()`, srcBuffer must be updated to reflect
 *  the number of bytes consumed after reading the frame header. Failure to update srcBuffer before calling
 *  `LZ4rF_decompress()` will cause decompression failure or, even worse, successful but incorrect decompression.
 *  See the `LZ4rF_getFrameInfo()` docs for details.
 *
 * @return : an hint of how many `srcSize` bytes LZ4rF_decompress() expects for next call.
 *  Schematically, it's the size of the current (or remaining) compressed block + header of next block.
 *  Respecting the hint provides some small speed benefit, because it skips intermediate buffers.
 *  This is just a hint though, it's always possible to provide any srcSize.
 *
 *  When a frame is fully decoded, @return will be 0 (no more data expected).
 *  When provided with more bytes than necessary to decode a frame,
 *  LZ4rF_decompress() will stop reading exactly at end of current frame, and @return 0.
 *
 *  If decompression failed, @return is an error code, which can be tested using LZ4rF_isError().
 *  After a decompression error, the `dctx` context is not resumable.
 *  Use LZ4rF_resetDecompressionContext() to return to clean state.
 *
 *  After a frame is fully decoded, dctx can be used again to decompress another frame.
 */
LZ4rFLIB_API size_t
LZ4rF_decompress(LZ4rF_dctx* dctx,
                void* dstBuffer, size_t* dstSizePtr,
          const void* srcBuffer, size_t* srcSizePtr,
          const LZ4rF_decompressOptions_t* dOptPtr);


/*! LZ4rF_resetDecompressionContext() : added in v1.8.0
 *  In case of an error, the context is left in "undefined" state.
 *  In which case, it's necessary to reset it, before re-using it.
 *  This method can also be used to abruptly stop any unfinished decompression,
 *  and start a new one using same context resources. */
LZ4rFLIB_API void LZ4rF_resetDecompressionContext(LZ4rF_dctx* dctx);   /* always successful */



#if defined (__cplusplus)
}
#endif

#endif  /* LZ4rF_H_09782039843 */

#if defined(LZ4rF_STATIC_LINKING_ONLY) && !defined(LZ4rF_H_STATIC_09782039843)
#define LZ4rF_H_STATIC_09782039843

/* Note :
 * The below declarations are not stable and may change in the future.
 * They are therefore only safe to depend on
 * when the caller is statically linked against the library.
 * To access their declarations, define LZ4rF_STATIC_LINKING_ONLY.
 *
 * By default, these symbols aren't published into shared/dynamic libraries.
 * You can override this behavior and force them to be published
 * by defining LZ4rF_PUBLISH_STATIC_FUNCTIONS.
 * Use at your own risk.
 */

#if defined (__cplusplus)
extern "C" {
#endif

#ifdef LZ4rF_PUBLISH_STATIC_FUNCTIONS
# define LZ4rFLIB_STATIC_API LZ4rFLIB_API
#else
# define LZ4rFLIB_STATIC_API
#endif


/* ---   Error List   --- */
#define LZ4rF_LIST_ERRORS(ITEM) \
        ITEM(OK_NoError) \
        ITEM(ERROR_GENERIC) \
        ITEM(ERROR_maxBlockSize_invalid) \
        ITEM(ERROR_blockMode_invalid) \
        ITEM(ERROR_parameter_invalid) \
        ITEM(ERROR_compressionLevel_invalid) \
        ITEM(ERROR_headerVersion_wrong) \
        ITEM(ERROR_blockChecksum_invalid) \
        ITEM(ERROR_reservedFlag_set) \
        ITEM(ERROR_allocation_failed) \
        ITEM(ERROR_srcSize_tooLarge) \
        ITEM(ERROR_dstMaxSize_tooSmall) \
        ITEM(ERROR_frameHeader_incomplete) \
        ITEM(ERROR_frameType_unknown) \
        ITEM(ERROR_frameSize_wrong) \
        ITEM(ERROR_srcPtr_wrong) \
        ITEM(ERROR_decompressionFailed) \
        ITEM(ERROR_headerChecksum_invalid) \
        ITEM(ERROR_contentChecksum_invalid) \
        ITEM(ERROR_frameDecoding_alreadyStarted) \
        ITEM(ERROR_compressionState_uninitialized) \
        ITEM(ERROR_parameter_null) \
        ITEM(ERROR_io_write) \
        ITEM(ERROR_io_read) \
        ITEM(ERROR_maxCode)

#define LZ4rF_GENERATE_ENUM(ENUM) LZ4rF_##ENUM,

/* enum list is exposed, to handle specific errors */
typedef enum { LZ4rF_LIST_ERRORS(LZ4rF_GENERATE_ENUM)
              _LZ4rF_dummy_error_enum_for_c89_never_used } LZ4rF_errorCodes;

LZ4rFLIB_STATIC_API LZ4rF_errorCodes LZ4rF_getErrorCode(size_t functionResult);

/**********************************
 *  Advanced compression operations
 *********************************/

/*! LZ4rF_getBlockSize() :
 * @return, in scalar format (size_t),
 *          the maximum block size associated with @blockSizeID,
 *          or an error code (can be tested using LZ4rF_isError()) if @blockSizeID is invalid.
**/
LZ4rFLIB_STATIC_API size_t LZ4rF_getBlockSize(LZ4rF_blockSizeID_t blockSizeID);

/*! LZ4rF_uncompressedUpdate() :
 *  LZ4rF_uncompressedUpdate() can be called repetitively to add data stored as uncompressed blocks.
 *  Important rule: dstCapacity MUST be large enough to store the entire source buffer as
 *  no compression is done for this operation
 *  If this condition is not respected, LZ4rF_uncompressedUpdate() will fail (result is an errorCode).
 *  After an error, the state is left in a UB state, and must be re-initialized or freed.
 *  If previously a compressed block was written, buffered data is flushed first,
 *  before appending uncompressed data is continued.
 *  This operation is only supported when LZ4rF_blockIndependent is used.
 * `cOptPtr` is optional : NULL can be provided, in which case all options are set to default.
 * @return : number of bytes written into `dstBuffer` (it can be zero, meaning input data was just buffered).
 *           or an error code if it fails (which can be tested using LZ4rF_isError())
 */
LZ4rFLIB_STATIC_API size_t
LZ4rF_uncompressedUpdate(LZ4rF_cctx* cctx,
                        void* dstBuffer, size_t dstCapacity,
                  const void* srcBuffer, size_t srcSize,
                  const LZ4rF_compressOptions_t* cOptPtr);

/**********************************
 *  Dictionary compression API
 *********************************/

/* A Dictionary is useful for the compression of small messages (KB range).
 * It dramatically improves compression efficiency.
 *
 * LZ4r can ingest any input as dictionary, though only the last 64 KB are useful.
 * Better results are generally achieved by using Zstandard's Dictionary Builder
 * to generate a high-quality dictionary from a set of samples.
 *
 * The same dictionary will have to be used on the decompression side
 * for decoding to be successful.
 * To help identify the correct dictionary at decoding stage,
 * the frame header allows optional embedding of a dictID field.
 */

/*! LZ4rF_compressBegin_usingDict() :
 *  Inits dictionary compression streaming, and writes the frame header into dstBuffer.
 * `dstCapacity` must be >= LZ4rF_HEADER_SIZE_MAX bytes.
 * `prefsPtr` is optional : you may provide NULL as argument,
 *  however, it's the only way to provide dictID in the frame header.
 * `dictBuffer` must outlive the compression session.
 * @return : number of bytes written into dstBuffer for the header,
 *           or an error code (which can be tested using LZ4rF_isError())
 *  NOTE: this entry point doesn't fully exploit the spec,
 *        which allows each independent block to be compressed with the dictionary.
 *        Currently, only the first block uses the dictionary.
 *        This is still technically compliant, but less efficient for large inputs.
 */
LZ4rFLIB_STATIC_API size_t
LZ4rF_compressBegin_usingDict(LZ4rF_cctx* cctx,
                            void* dstBuffer, size_t dstCapacity,
                      const void* dictBuffer, size_t dictSize,
                      const LZ4rF_preferences_t* prefsPtr);

/*! LZ4rF_decompress_usingDict() :
 *  Same as LZ4rF_decompress(), using a predefined dictionary.
 *  Dictionary is used "in place", without any preprocessing.
**  It must remain accessible throughout the entire frame decoding. */
LZ4rFLIB_STATIC_API size_t
LZ4rF_decompress_usingDict(LZ4rF_dctx* dctxPtr,
                          void* dstBuffer, size_t* dstSizePtr,
                    const void* srcBuffer, size_t* srcSizePtr,
                    const void* dict, size_t dictSize,
                    const LZ4rF_decompressOptions_t* decompressOptionsPtr);

/**********************************
 *  Bulk processing dictionary API
 *********************************/

/* Loading a dictionary has a cost, since it involves construction of tables.
 * The Bulk processing dictionary API makes it possible to share this cost
 * over an arbitrary number of compression jobs, even concurrently,
 * markedly improving compression latency for these cases.
 */
typedef struct LZ4rF_CDict_s LZ4rF_CDict;

/*! LZ4r_createCDict() :
 *  When compressing multiple messages / blocks using the same dictionary, it's recommended to load it just once.
 *  LZ4r_createCDict() will create a digested dictionary, ready to start future compression operations without startup delay.
 *  LZ4r_CDict can be created once and shared by multiple threads concurrently, since its usage is read-only.
 * `dictBuffer` can be released after LZ4r_CDict creation, since its content is copied within CDict */
LZ4rFLIB_STATIC_API LZ4rF_CDict* LZ4rF_createCDict(const void* dictBuffer, size_t dictSize);
LZ4rFLIB_STATIC_API void        LZ4rF_freeCDict(LZ4rF_CDict* CDict);

/*! LZ4r_compressFrame_usingCDict() :
 *  Compress an entire srcBuffer into a valid LZ4r frame using a digested Dictionary.
 *  cctx must point to a context created by LZ4rF_createCompressionContext().
 *  If cdict==NULL, compress without a dictionary.
 *  dstBuffer MUST be >= LZ4rF_compressFrameBound(srcSize, preferencesPtr).
 *  If this condition is not respected, function will fail (@return an errorCode).
 *  The LZ4rF_preferences_t structure is optional : you may provide NULL as argument,
 *  but it's not recommended, as it's the only way to provide dictID in the frame header.
 * @return : number of bytes written into dstBuffer.
 *           or an error code if it fails (can be tested using LZ4rF_isError()) */
LZ4rFLIB_STATIC_API size_t
LZ4rF_compressFrame_usingCDict(LZ4rF_cctx* cctx,
                              void* dst, size_t dstCapacity,
                        const void* src, size_t srcSize,
                        const LZ4rF_CDict* cdict,
                        const LZ4rF_preferences_t* preferencesPtr);

/*! LZ4rF_compressBegin_usingCDict() :
 *  Inits streaming dictionary compression, and writes the frame header into dstBuffer.
 * `dstCapacity` must be >= LZ4rF_HEADER_SIZE_MAX bytes.
 * `prefsPtr` is optional : you may provide NULL as argument,
 *  however, it's the only way to provide dictID in the frame header.
 * `cdict` must outlive the compression session.
 * @return : number of bytes written into dstBuffer for the header,
 *           or an error code (which can be tested using LZ4rF_isError()) */
LZ4rFLIB_STATIC_API size_t
LZ4rF_compressBegin_usingCDict(LZ4rF_cctx* cctx,
                              void* dstBuffer, size_t dstCapacity,
                        const LZ4rF_CDict* cdict,
                        const LZ4rF_preferences_t* prefsPtr);


/**********************************
 *  Custom memory allocation
 *********************************/

/*! Custom memory allocation : v1.9.4+
 *  These prototypes make it possible to pass custom allocation/free functions.
 *  LZ4rF_customMem is provided at state creation time, using LZ4rF_create*_advanced() listed below.
 *  All allocation/free operations will be completed using these custom variants instead of regular <stdlib.h> ones.
 */
typedef void* (*LZ4rF_AllocFunction) (void* opaqueState, size_t size);
typedef void* (*LZ4rF_CallocFunction) (void* opaqueState, size_t size);
typedef void  (*LZ4rF_FreeFunction) (void* opaqueState, void* address);
typedef struct {
    LZ4rF_AllocFunction customAlloc;
    LZ4rF_CallocFunction customCalloc; /* optional; when not defined, uses customAlloc + memset */
    LZ4rF_FreeFunction customFree;
    void* opaqueState;
} LZ4rF_CustomMem;
static
#ifdef __GNUC__
__attribute__((__unused__))
#endif
LZ4rF_CustomMem const LZ4rF_defaultCMem = { NULL, NULL, NULL, NULL };  /**< this constant defers to stdlib's functions */

LZ4rFLIB_STATIC_API LZ4rF_cctx* LZ4rF_createCompressionContext_advanced(LZ4rF_CustomMem customMem, unsigned version);
LZ4rFLIB_STATIC_API LZ4rF_dctx* LZ4rF_createDecompressionContext_advanced(LZ4rF_CustomMem customMem, unsigned version);
LZ4rFLIB_STATIC_API LZ4rF_CDict* LZ4rF_createCDict_advanced(LZ4rF_CustomMem customMem, const void* dictBuffer, size_t dictSize);


#if defined (__cplusplus)
}
#endif

#endif  /* defined(LZ4rF_STATIC_LINKING_ONLY) && !defined(LZ4rF_H_STATIC_09782039843) */
