#ifndef __LZ4DEFS_H__
#define __LZ4DEFS_H__

/*
 * lz4defs.h -- common and architecture specific defines for the kernel usage

 * LZ4 - Fast LZ compression algorithm
 * Copyright (C) 2011-2016, Yann Collet.
 * BSD 2-Clause License (http://www.opensource.org/licenses/bsd-license.php)
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *	* Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following disclaimer
 * in the documentation and/or other materials provided with the
 * distribution.
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
 * You can contact the author at :
 *	- LZ4 homepage : http://www.lz4.org
 *	- LZ4 source repository : https://github.com/lz4/lz4
 *
 *	Changed for kernel usage by:
 *	Sven Schmidt <4sschmid@informatik.uni-hamburg.de>
 */

#include <asm/unaligned.h>
#include <linux/string.h>	 /* memset, memcpy */

#define FORCE_INLINE __always_inline

/*-************************************
 *	Basic Types
 **************************************/
#include <linux/types.h>

typedef	uint8_t BYTE;
typedef uint16_t U16;
typedef uint32_t U32;
typedef	int32_t S32;
typedef uint64_t U64;
typedef uintptr_t uptrval;

/*-************************************
 *	Architecture specifics
 **************************************/
#if defined(CONFIG_64BIT)
#define LZ4_ARCH64 1
#else
#define LZ4_ARCH64 0
#endif

#if defined(__LITTLE_ENDIAN)
#define LZ4_LITTLE_ENDIAN 1
#else
#define LZ4_LITTLE_ENDIAN 0
#endif

/*
 * LZ4_FORCE_SW_BITCOUNT
 * Define this parameter if your target system
 * does not support hardware bit count
 */
/* #define LZ4_FORCE_SW_BITCOUNT */

/*-************************************
 *	Constants
 **************************************/
#define MINMATCH 4

#define WILDCOPYLENGTH 8
#define LASTLITERALS 5
#define MFLIMIT (WILDCOPYLENGTH + MINMATCH)

/* Increase this value ==> compression run slower on incompressible data */
#define LZ4_SKIPTRIGGER 6

#define KB (1<<10)
#define MB (1<<20)
#define GB (1U<<30)

#define MAXD_LOG 16
#define MAX_DISTANCE ((1<<MAXD_LOG) - 1)
#define STEPSIZE sizeof(size_t)

#define ML_BITS	4
#define ML_MASK	((1U<<ML_BITS)-1)
#define RUN_BITS (8-ML_BITS)
#define RUN_MASK ((1U<<RUN_BITS)-1)

/*-************************************
 *	Reading and writing into memory
 **************************************/
typedef union {
	U16 u16;
	U32 u32;
	size_t uArch;
} __packed unalign;

static FORCE_INLINE __maybe_unused U16 LZ4_read16(const void *ptr)
{
	return ((const unalign *)ptr)->u16;
}

static FORCE_INLINE __maybe_unused U32 LZ4_read32(const void *ptr)
{
	return ((const unalign *)ptr)->u32;
}

static FORCE_INLINE __maybe_unused size_t LZ4_read_ARCH(const void *ptr)
{
	return ((const unalign *)ptr)->uArch;
}

static FORCE_INLINE __maybe_unused void LZ4_write16(void *memPtr, U16 value)
{
	((unalign *)memPtr)->u16 = value;
}

static FORCE_INLINE __maybe_unused void LZ4_write32(void *memPtr, U32 value) {
	((unalign *)memPtr)->u32 = value;
}

static FORCE_INLINE __maybe_unused U16 LZ4_readLE16(const void *memPtr)
{
#if LZ4_LITTLE_ENDIAN
	return LZ4_read16(memPtr);
#else
	const BYTE *p = (const BYTE *)memPtr;

	return (U16)((U16)p[0] + (p[1] << 8));
#endif
}

static FORCE_INLINE __maybe_unused void LZ4_writeLE16(void *memPtr, U16 value)
{
#if LZ4_LITTLE_ENDIAN
	LZ4_write16(memPtr, value);
#else
	BYTE *p = (BYTE *)memPtr;

	p[0] = (BYTE) value;
	p[1] = (BYTE)(value >> 8);
#endif
}

static FORCE_INLINE void LZ4_copy8(void *dst, const void *src)
{
	memcpy(dst, src, 8);
}

/*
 * customized variant of memcpy,
 * which can overwrite up to 7 bytes beyond dstEnd
 */
static FORCE_INLINE void LZ4_wildCopy(void *dstPtr,
	const void *srcPtr, void *dstEnd)
{
	BYTE *d = (BYTE *)dstPtr;
	const BYTE *s = (const BYTE *)srcPtr;
	BYTE *const e = (BYTE *)dstEnd;

	do {
		LZ4_copy8(d, s);
		d += 8;
		s += 8;
	} while (d < e);
}

static FORCE_INLINE unsigned int LZ4_NbCommonBytes(register size_t val)
{
#if LZ4_LITTLE_ENDIAN
#if LZ4_ARCH64 /* 64 Bits Little Endian */
#if defined(LZ4_FORCE_SW_BITCOUNT)
	static const int DeBruijnBytePos[64] = {
		0, 0, 0, 0, 0, 1, 1, 2, 0, 3, 1, 3, 1, 4, 2, 7,
		0, 2, 3, 6, 1, 5, 3, 5, 1, 3, 4, 4, 2, 5, 6, 7,
		7, 0, 1, 2, 3, 3, 4, 6, 2, 6, 5, 5, 3, 4, 5, 6,
		7, 1, 2, 4, 6, 4, 4, 5, 7, 2, 6, 5, 7, 6, 7, 7
	};

	return DeBruijnBytePos[((U64)((val & -(long long)val)
		* 0x0218A392CDABBD3FULL)) >> 58];
#else
	return (__builtin_ctzll((U64)val) >> 3);
#endif /* defined(LZ4_FORCE_SW_BITCOUNT) */
#else /* 32 Bits Little Endian */
#if defined(LZ4_FORCE_SW_BITCOUNT)
	static const int DeBruijnBytePos[32] = {
		0, 0, 3, 0, 3, 1, 3, 0, 3, 2, 2, 1, 3, 2, 0, 1,
		3, 3, 1, 2, 2, 2, 2, 0, 3, 1, 2, 0, 1, 0, 1, 1
	};

	return DeBruijnBytePos[((U32)((val & -(S32)val)
		* 0x077CB531U)) >> 27];
#else
	return (__builtin_ctz((U32)val) >> 3);
#endif /* defined(LZ4_FORCE_SW_BITCOUNT) */
#endif /* LZ4_ARCH64 */
#else /* Big Endian */
#if LZ4_ARCH64 /* 64 Bits Big Endian */
#if defined(LZ4_FORCE_SW_BITCOUNT)
	unsigned int r;

	if (!(val >> 32)) {
		r = 4;
	} else {
		r = 0;
		val >>= 32;
	}

	if (!(val >> 16)) {
		r += 2;
		val >>= 8;
	} else {
		val >>= 24;
	}

	r += (!val);

	return r;
#else
	return (__builtin_clzll((U64)val) >> 3);
#endif /* defined(LZ4_FORCE_SW_BITCOUNT) */
#else /* 32 Bits Big Endian */
#if defined(LZ4_FORCE_SW_BITCOUNT)
	unsigned int r;

	if (!(val >> 16)) {
		r = 2;
		val >>= 8;
	} else {
		r = 0;
		val >>= 24;
	}

	r += (!val);

	return r;
#else
	return (__builtin_clz((U32)val) >> 3);
#endif /* defined(LZ4_FORCE_SW_BITCOUNT) */
#endif /* LZ4_ARCH64 */
#endif /* LZ4_LITTLE_ENDIAN */
}

static FORCE_INLINE __maybe_unused unsigned int LZ4_count(
	const BYTE *pIn,
	const BYTE *pMatch,
	const BYTE *pInLimit)
{
	const BYTE *const pStart = pIn;

	while (likely(pIn < pInLimit - (STEPSIZE - 1))) {
		size_t const diff = LZ4_read_ARCH(pMatch) ^ LZ4_read_ARCH(pIn);

		if (!diff) {
			pIn += STEPSIZE;
			pMatch += STEPSIZE;
			continue;
		}

		pIn += LZ4_NbCommonBytes(diff);

		return (unsigned int)(pIn - pStart);
	}

#if LZ4_ARCH64
	if ((pIn < (pInLimit - 3))
		&& (LZ4_read32(pMatch) == LZ4_read32(pIn))) {
		pIn += 4;
		pMatch += 4;
	}
#endif

	if ((pIn < (pInLimit - 1))
		&& (LZ4_read16(pMatch) == LZ4_read16(pIn))) {
		pIn += 2;
		pMatch += 2;
	}

	if ((pIn < pInLimit) && (*pMatch == *pIn))
		pIn++;

	return (unsigned int)(pIn - pStart);
}

typedef enum { noLimit = 0, limitedOutput = 1 } limitedOutput_directive;
typedef enum { byPtr, byU32, byU16 } tableType_t;

typedef enum { noDict = 0, withPrefix64k, usingExtDict } dict_directive;
typedef enum { noDictIssue = 0, dictSmall } dictIssue_directive;

typedef enum { endOnOutputSize = 0, endOnInputSize = 1 } endCondition_directive;
typedef enum { full = 0, partial = 1 } earlyEnd_directive;

#endif
