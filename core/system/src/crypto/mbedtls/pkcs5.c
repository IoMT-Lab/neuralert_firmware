/**
 * \file pkcs5.c
 *
 * \brief PKCS#5 functions
 *
 * \author Mathias Olsson <mathias@kompetensum.com>
 *
 *  Copyright (C) 2006-2015, ARM Limited, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  This file is part of mbed TLS (https://tls.mbed.org)
 */
/*
 * PKCS#5 includes PBKDF2 and more
 *
 * http://tools.ietf.org/html/rfc2898 (Specification)
 * http://tools.ietf.org/html/rfc6070 (Test vectors)
 */

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#if defined(MBEDTLS_PKCS5_C)

#include "mbedtls/pkcs5.h"
#include "mbedtls/asn1.h"
#include "mbedtls/cipher.h"
#include "mbedtls/oid.h"

#include <string.h>

#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdio.h>
#define mbedtls_printf printf
#endif

#define PCKS5_FCI_OPTIMIZE
#ifdef	PCKS5_FCI_OPTIMIZE
#include "target.h"
#include "mbedtls/md_internal.h"
#endif	//PCKS5_FCI_OPTIMIZE

#pragma GCC diagnostic ignored "-Wsign-conversion"

static int pkcs5_parse_pbkdf2_params( const mbedtls_asn1_buf *params,
                                      mbedtls_asn1_buf *salt, int *iterations,
                                      int *keylen, mbedtls_md_type_t *md_type )
{
    int ret;
    mbedtls_asn1_buf prf_alg_oid;
    unsigned char *p = params->p;
    const unsigned char *end = params->p + params->len;

    if( params->tag != ( MBEDTLS_ASN1_CONSTRUCTED | MBEDTLS_ASN1_SEQUENCE ) )
        return( MBEDTLS_ERR_PKCS5_INVALID_FORMAT +
                MBEDTLS_ERR_ASN1_UNEXPECTED_TAG );
    /*
     *  PBKDF2-params ::= SEQUENCE {
     *    salt              OCTET STRING,
     *    iterationCount    INTEGER,
     *    keyLength         INTEGER OPTIONAL
     *    prf               AlgorithmIdentifier DEFAULT algid-hmacWithSHA1
     *  }
     *
     */
    if( ( ret = mbedtls_asn1_get_tag( &p, end, &salt->len, MBEDTLS_ASN1_OCTET_STRING ) ) != 0 )
        return( MBEDTLS_ERR_PKCS5_INVALID_FORMAT + ret );

    salt->p = p;
    p += salt->len;

    if( ( ret = mbedtls_asn1_get_int( &p, end, iterations ) ) != 0 )
        return( MBEDTLS_ERR_PKCS5_INVALID_FORMAT + ret );

    if( p == end )
        return( 0 );

    if( ( ret = mbedtls_asn1_get_int( &p, end, keylen ) ) != 0 )
    {
        if( ret != MBEDTLS_ERR_ASN1_UNEXPECTED_TAG )
            return( MBEDTLS_ERR_PKCS5_INVALID_FORMAT + ret );
    }

    if( p == end )
        return( 0 );

    if( ( ret = mbedtls_asn1_get_alg_null( &p, end, &prf_alg_oid ) ) != 0 )
        return( MBEDTLS_ERR_PKCS5_INVALID_FORMAT + ret );

    if( MBEDTLS_OID_CMP( MBEDTLS_OID_HMAC_SHA1, &prf_alg_oid ) != 0 )
        return( MBEDTLS_ERR_PKCS5_FEATURE_UNAVAILABLE );

    *md_type = MBEDTLS_MD_SHA1;

    if( p != end )
        return( MBEDTLS_ERR_PKCS5_INVALID_FORMAT +
                MBEDTLS_ERR_ASN1_LENGTH_MISMATCH );

    return( 0 );
}

int mbedtls_pkcs5_pbes2( const mbedtls_asn1_buf *pbe_params, int mode,
                 const unsigned char *pwd,  size_t pwdlen,
                 const unsigned char *data, size_t datalen,
                 unsigned char *output )
{
    int ret, iterations = 0, keylen = 0;
    unsigned char *p, *end;
    mbedtls_asn1_buf kdf_alg_oid, enc_scheme_oid, kdf_alg_params, enc_scheme_params;
    mbedtls_asn1_buf salt;
    mbedtls_md_type_t md_type = MBEDTLS_MD_SHA1;
    unsigned char key[32], iv[32];
    size_t olen = 0;
    const mbedtls_md_info_t *md_info;
    const mbedtls_cipher_info_t *cipher_info;
    mbedtls_md_context_t md_ctx;
    mbedtls_cipher_type_t cipher_alg;
    mbedtls_cipher_context_t cipher_ctx;

    p = pbe_params->p;
    end = p + pbe_params->len;

    /*
     *  PBES2-params ::= SEQUENCE {
     *    keyDerivationFunc AlgorithmIdentifier {{PBES2-KDFs}},
     *    encryptionScheme AlgorithmIdentifier {{PBES2-Encs}}
     *  }
     */
    if( pbe_params->tag != ( MBEDTLS_ASN1_CONSTRUCTED | MBEDTLS_ASN1_SEQUENCE ) )
        return( MBEDTLS_ERR_PKCS5_INVALID_FORMAT +
                MBEDTLS_ERR_ASN1_UNEXPECTED_TAG );

    if( ( ret = mbedtls_asn1_get_alg( &p, end, &kdf_alg_oid, &kdf_alg_params ) ) != 0 )
        return( MBEDTLS_ERR_PKCS5_INVALID_FORMAT + ret );

    // Only PBKDF2 supported at the moment
    //
    if( MBEDTLS_OID_CMP( MBEDTLS_OID_PKCS5_PBKDF2, &kdf_alg_oid ) != 0 )
        return( MBEDTLS_ERR_PKCS5_FEATURE_UNAVAILABLE );

    if( ( ret = pkcs5_parse_pbkdf2_params( &kdf_alg_params,
                                           &salt, &iterations, &keylen,
                                           &md_type ) ) != 0 )
    {
        return( ret );
    }

    md_info = mbedtls_md_info_from_type( md_type );
    if( md_info == NULL )
        return( MBEDTLS_ERR_PKCS5_FEATURE_UNAVAILABLE );

    if( ( ret = mbedtls_asn1_get_alg( &p, end, &enc_scheme_oid,
                              &enc_scheme_params ) ) != 0 )
    {
        return( MBEDTLS_ERR_PKCS5_INVALID_FORMAT + ret );
    }

    if( mbedtls_oid_get_cipher_alg( &enc_scheme_oid, &cipher_alg ) != 0 )
        return( MBEDTLS_ERR_PKCS5_FEATURE_UNAVAILABLE );

    cipher_info = mbedtls_cipher_info_from_type( cipher_alg );
    if( cipher_info == NULL )
        return( MBEDTLS_ERR_PKCS5_FEATURE_UNAVAILABLE );

    /*
     * The value of keylen from pkcs5_parse_pbkdf2_params() is ignored
     * since it is optional and we don't know if it was set or not
     */
    keylen = cipher_info->key_bitlen / 8;

    if( enc_scheme_params.tag != MBEDTLS_ASN1_OCTET_STRING ||
        enc_scheme_params.len != cipher_info->iv_size )
    {
        return( MBEDTLS_ERR_PKCS5_INVALID_FORMAT );
    }

    mbedtls_md_init( &md_ctx );
    mbedtls_cipher_init( &cipher_ctx );

    memcpy( iv, enc_scheme_params.p, enc_scheme_params.len );

    if( ( ret = mbedtls_md_setup( &md_ctx, md_info, 1 ) ) != 0 )
        goto exit;

    if( ( ret = mbedtls_pkcs5_pbkdf2_hmac( &md_ctx, pwd, pwdlen, salt.p, salt.len,
                                   iterations, keylen, key ) ) != 0 )
    {
        goto exit;
    }

    if( ( ret = mbedtls_cipher_setup( &cipher_ctx, cipher_info ) ) != 0 )
        goto exit;

    if( ( ret = mbedtls_cipher_setkey( &cipher_ctx, key, 8 * keylen, (mbedtls_operation_t) mode ) ) != 0 )
        goto exit;

    if( ( ret = mbedtls_cipher_crypt( &cipher_ctx, iv, enc_scheme_params.len,
                              data, datalen, output, &olen ) ) != 0 )
        ret = MBEDTLS_ERR_PKCS5_PASSWORD_MISMATCH;

exit:
    mbedtls_md_free( &md_ctx );
    mbedtls_cipher_free( &cipher_ctx );

    return( ret );
}
#define PCKS5_FCI_HW
#ifdef PCKS5_FCI_HW
#define HW_PBKDF2_CLK	0x500060b0
#define HW_PBKDF2_START	0x500e0000
#define HW_PBKDF2_IHV1	0x500e0004
#define HW_PBKDF2_IHV2	0x500e0018
#define HW_PBKDF2_DATA  0x500e002c
#define HW_PBKDF2_DIGEST  0x500e0040

#define MEM_LONG_READ(addr, data)		*data = *((volatile unsigned int *)addr)
#define MEM_LONG_WRITE(addr, data)		*((volatile unsigned int *)addr) = data
//! Byte swap int
static unsigned int swap_int32( unsigned int val )
{
    val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF );
    return (val << 16) | ((val >> 16) & 0xFFFF);
}
#endif
int mbedtls_pkcs5_pbkdf2_hmac( mbedtls_md_context_t *ctx, const unsigned char *password,
                       size_t plen, const unsigned char *salt, size_t slen,
                       unsigned int iteration_count,
                       uint32_t key_length, unsigned char *output )
{
	int ret, j;
	unsigned int i;
	unsigned char md1[MBEDTLS_MD_MAX_SIZE];
	unsigned char work[MBEDTLS_MD_MAX_SIZE];
	unsigned char md_size = mbedtls_md_get_size( ctx->md_info );
	size_t use_len;
	unsigned char *out_p = output;
	mbedtls_md_context_t init_ctx;
#ifdef	PCKS5_FCI_OPTIMIZE
	unsigned int  counter32;

	counter32 = 1;
#else	//PCKS5_FCI_OPTIMIZE
	unsigned char counter[4];

	memset( counter, 0, 4 );
	counter[3] = 1;
#endif	//PCKS5_FCI_OPTIMIZE

	if( iteration_count > 0xFFFFFFFF )
		return( MBEDTLS_ERR_PKCS5_BAD_INPUT_DATA );

	if( ( ret = mbedtls_md_hmac_starts( ctx, password, plen ) ) != 0 )
		return( ret );

	/* Copy the initial digest state to init_ctx for reuse later.  This
	* avoids the rehashing of password if it larger than the digest block
	* size and the recomputation of the digest across the HMAC input pad
	* for every iteration. */
	mbedtls_md_init( &init_ctx );

	if( ( ret = mbedtls_md_setup( &init_ctx, ctx->md_info, 0 ) ) != 0 )
		return( ret );

	if( ( ret = mbedtls_md_clone( &init_ctx, ctx ) ) != 0 )
		goto out_init_ctx;


	while( key_length )
	{
		// U1 ends up in work
		//
		if( ( ret = mbedtls_md_clone( ctx, &init_ctx ) ) != 0 )
			goto out_init_ctx;

		if( ( ret = mbedtls_md_hmac_update( ctx, salt, slen ) ) != 0 )
			goto out_init_ctx;
#ifdef	PCKS5_FCI_OPTIMIZE
		if( ( ret = mbedtls_md_hmac_update( ctx, (unsigned char *)(DA16X_SRAM32S_BASE|(unsigned int)(&counter32)), 4 ) ) != 0 )
			goto out_init_ctx;
#else	//PCKS5_FCI_OPTIMIZE
		if( ( ret = mbedtls_md_hmac_update( ctx, counter, 4 ) ) != 0 )
			goto out_init_ctx;
#endif	//PCKS5_FCI_OPTIMIZE

		if( ( ret = mbedtls_md_hmac_finish( ctx, work ) ) != 0 )
			goto out_init_ctx;

		memcpy( md1, work, md_size );

#ifndef PCKS5_FCI_HW
		for( i = 1; i < iteration_count; i++ )
		{
#ifdef	PCKS5_FCI_OPTIMIZE
			// U2 ends up in md1
			//
			ctx->md_info->clone_func( ctx->md_ctx, init_ctx.md_ctx );
			ctx->md_info->update_func( ctx->md_ctx, md1, md_size );
			{
			    unsigned char *tmp;
			    unsigned char *opad;

			    opad = (unsigned char *) ctx->hmac_ctx + ctx->md_info->block_size;
			    tmp  = (unsigned char *) ctx->hmac_ctx + (2 * ctx->md_info->block_size);

			    ctx->md_info->finish_func( ctx->md_ctx, tmp );
			    ctx->md_info->starts_func( ctx->md_ctx );
			    ctx->md_info->update_func( ctx->md_ctx, opad, (ctx->md_info->block_size + ctx->md_info->size) );
			    ctx->md_info->finish_func( ctx->md_ctx, md1 );
			}
			// U1 xor U2
			//
			{
				unsigned int *work32 = (unsigned int *) work;
				unsigned int *md1_32 = (unsigned int *) md1;
				for( j = 0; j < (md_size>>2); j++ ){
					work32[j] ^= md1_32[j];
				}
			}
#else	//PCKS5_FCI_OPTIMIZE
			// U2 ends up in md1
			//
			if( ( ret = mbedtls_md_clone( ctx, &init_ctx ) ) != 0 )
				goto out_init_ctx;

			if( ( ret = mbedtls_md_hmac_update( ctx, md1, md_size ) ) != 0 )
				goto out_init_ctx;

			if( ( ret = mbedtls_md_hmac_finish( ctx, md1 ) ) != 0 )
				goto out_init_ctx;

			// U1 xor U2
			//
			for( j = 0; j < md_size; j++ )
				work[j] ^= md1[j];
#endif	//PCKS5_FCI_OPTIMIZE
		}
#else
		// digest md1, vector1 ctx.md_ctx, vector2 calculate
		if (iteration_count > 1 && (ctx->md_info->type == MBEDTLS_MD_SHA1) )
		{
			unsigned int init_vector1[5], init_vector2[5], data[5], *pwork = (unsigned int *)work;
			char *cpy_tmp;
			// vector 2 calculate
			ctx->md_info->clone_func( ctx->md_ctx, init_ctx.md_ctx );
			ctx->md_info->update_func( ctx->md_ctx, md1, md_size );
			cpy_tmp = (char *)(ctx->md_ctx) + 8;
			memcpy(init_vector1, cpy_tmp, 20);
			//memcpy(((void *)HW_PBKDF2_IHV1), cpy_tmp, 20);

			*((volatile char *)(HW_PBKDF2_CLK)) = 0x00;
			MEM_LONG_WRITE(HW_PBKDF2_START, 0x01);

			MEM_LONG_WRITE(HW_PBKDF2_IHV1, (init_vector1[0]));
			MEM_LONG_WRITE(HW_PBKDF2_IHV1+1, (init_vector1[1]));
			MEM_LONG_WRITE(HW_PBKDF2_IHV1+2, (init_vector1[2]));
			MEM_LONG_WRITE(HW_PBKDF2_IHV1+3, (init_vector1[3]));
			MEM_LONG_WRITE(HW_PBKDF2_IHV1+4, (init_vector1[4]));

			{
			    unsigned char *tmp;
			    unsigned char *opad;

			    opad = (unsigned char *) ctx->hmac_ctx + ctx->md_info->block_size;
			    tmp  = (unsigned char *) ctx->hmac_ctx + (2 * ctx->md_info->block_size);

			    ctx->md_info->finish_func( ctx->md_ctx, tmp );
			    ctx->md_info->starts_func( ctx->md_ctx );
			    ctx->md_info->update_func( ctx->md_ctx, opad, (ctx->md_info->block_size + ctx->md_info->size) );
				cpy_tmp = (char *)(ctx->md_ctx) + 8;
				memcpy(init_vector2, cpy_tmp, 20);
				//memcpy(((void *)HW_PBKDF2_IHV2), cpy_tmp, 20);

				MEM_LONG_WRITE(HW_PBKDF2_IHV2, (init_vector2[0]));
				MEM_LONG_WRITE(HW_PBKDF2_IHV2+1, (init_vector2[1]));
				MEM_LONG_WRITE(HW_PBKDF2_IHV2+2, (init_vector2[2]));
				MEM_LONG_WRITE(HW_PBKDF2_IHV2+3, (init_vector2[3]));
				MEM_LONG_WRITE(HW_PBKDF2_IHV2+4, (init_vector2[4]));

			}
			//memcpy((void*)HW_PBKDF2_DATA, md1, 20);
			memcpy(data, md1, 20);
			MEM_LONG_WRITE(HW_PBKDF2_DATA, swap_int32(data[0]));
			MEM_LONG_WRITE(HW_PBKDF2_DATA+1, swap_int32(data[1]));
			MEM_LONG_WRITE(HW_PBKDF2_DATA+2, swap_int32(data[2]));
			MEM_LONG_WRITE(HW_PBKDF2_DATA+3, swap_int32(data[3]));
			MEM_LONG_WRITE(HW_PBKDF2_DATA+4, swap_int32(data[4]));

			MEM_LONG_WRITE(HW_PBKDF2_START, ((iteration_count - 2)<<16) | 0x03);
			asm volatile (	"nop       \n");
			asm volatile (	"nop       \n");
			asm volatile (	"nop       \n");
			asm volatile (	"nop       \n");
			asm volatile (	"nop       \n");
			while( !(*((volatile unsigned int *)HW_PBKDF2_START) & 0x100 ));
			//memcpy(work, (void*)HW_PBKDF2_DIGEST, 20);
			MEM_LONG_READ(HW_PBKDF2_DIGEST, &(data[0]));
			MEM_LONG_READ(HW_PBKDF2_DIGEST+1, &(data[1]));
			MEM_LONG_READ(HW_PBKDF2_DIGEST+2, &(data[2]));
			MEM_LONG_READ(HW_PBKDF2_DIGEST+3, &(data[3]));
			MEM_LONG_READ(HW_PBKDF2_DIGEST+4, &(data[4]));
			pwork[0] = swap_int32(data[0]);
			pwork[1] = swap_int32(data[1]);
			pwork[2] = swap_int32(data[2]);
			pwork[3] = swap_int32(data[3]);
			pwork[4] = swap_int32(data[4]);
			MEM_LONG_WRITE(HW_PBKDF2_START, 0);
			*((volatile char *)(HW_PBKDF2_CLK)) = 0x01;
		}
		else
		{
			for( i = 1; i < iteration_count; i++ )
			{
#ifdef	PCKS5_FCI_OPTIMIZE
				// U2 ends up in md1
				//
				ctx->md_info->clone_func( ctx->md_ctx, init_ctx.md_ctx );
				ctx->md_info->update_func( ctx->md_ctx, md1, md_size );
				{
					unsigned char *tmp;
					unsigned char *opad;

					opad = (unsigned char *) ctx->hmac_ctx + ctx->md_info->block_size;
					tmp  = (unsigned char *) ctx->hmac_ctx + (2 * ctx->md_info->block_size);

					ctx->md_info->finish_func( ctx->md_ctx, tmp );
					ctx->md_info->starts_func( ctx->md_ctx );
					ctx->md_info->update_func( ctx->md_ctx, opad, (ctx->md_info->block_size + ctx->md_info->size) );
					ctx->md_info->finish_func( ctx->md_ctx, md1 );
				}
				// U1 xor U2
				//
				{
					unsigned int *work32 = (unsigned int *) work;
					unsigned int *md1_32 = (unsigned int *) md1;
					for( j = 0; j < (md_size>>2); j++ ){
						work32[j] ^= md1_32[j];
					}
				}
#else	//PCKS5_FCI_OPTIMIZE
				// U2 ends up in md1
				//
				if( ( ret = mbedtls_md_clone( ctx, &init_ctx ) ) != 0 )
					goto out_init_ctx;

				if( ( ret = mbedtls_md_hmac_update( ctx, md1, md_size ) ) != 0 )
					goto out_init_ctx;

				if( ( ret = mbedtls_md_hmac_finish( ctx, md1 ) ) != 0 )
					goto out_init_ctx;

				// U1 xor U2
				//
				for( j = 0; j < md_size; j++ )
					work[j] ^= md1[j];
#endif	//PCKS5_FCI_OPTIMIZE
			}

		}
#endif

		use_len = ( key_length < md_size ) ? key_length : md_size;
		memcpy( out_p, work, use_len );

		key_length -= (uint32_t) use_len;
		out_p += use_len;

#ifdef	PCKS5_FCI_OPTIMIZE
		counter32 ++;
#else	//PCKS5_FCI_OPTIMIZE
		for( i = 4; i > 0; i-- )
			if( ++counter[i - 1] != 0 )
				break;
#endif	//PCKS5_FCI_OPTIMIZE
	}

out_init_ctx:
	mbedtls_md_free( &init_ctx );

	return( ret );
}

#if defined(MBEDTLS_SELF_TEST)

#if !defined(MBEDTLS_SHA1_C)
int mbedtls_pkcs5_self_test( int verbose )
{
    if( verbose != 0 )
        mbedtls_printf( "  PBKDF2 (SHA1): skipped\n\n" );

    return( 0 );
}
#else

#define MAX_TESTS   6

static const size_t plen[MAX_TESTS] =
    { 8, 8, 8, 24, 9 };

static const unsigned char password[MAX_TESTS][32] =
{
    "password",
    "password",
    "password",
    "passwordPASSWORDpassword",
    "pass\0word",
};

static const size_t slen[MAX_TESTS] =
    { 4, 4, 4, 36, 5 };

static const unsigned char salt[MAX_TESTS][40] =
{
    "salt",
    "salt",
    "salt",
    "saltSALTsaltSALTsaltSALTsaltSALTsalt",
    "sa\0lt",
};

static const uint32_t it_cnt[MAX_TESTS] =
    { 1, 2, 4096, 4096, 4096 };

static const uint32_t key_len[MAX_TESTS] =
    { 20, 20, 20, 25, 16 };

static const unsigned char result_key[MAX_TESTS][32] =
{
    { 0x0c, 0x60, 0xc8, 0x0f, 0x96, 0x1f, 0x0e, 0x71,
      0xf3, 0xa9, 0xb5, 0x24, 0xaf, 0x60, 0x12, 0x06,
      0x2f, 0xe0, 0x37, 0xa6 },
    { 0xea, 0x6c, 0x01, 0x4d, 0xc7, 0x2d, 0x6f, 0x8c,
      0xcd, 0x1e, 0xd9, 0x2a, 0xce, 0x1d, 0x41, 0xf0,
      0xd8, 0xde, 0x89, 0x57 },
    { 0x4b, 0x00, 0x79, 0x01, 0xb7, 0x65, 0x48, 0x9a,
      0xbe, 0xad, 0x49, 0xd9, 0x26, 0xf7, 0x21, 0xd0,
      0x65, 0xa4, 0x29, 0xc1 },
    { 0x3d, 0x2e, 0xec, 0x4f, 0xe4, 0x1c, 0x84, 0x9b,
      0x80, 0xc8, 0xd8, 0x36, 0x62, 0xc0, 0xe4, 0x4a,
      0x8b, 0x29, 0x1a, 0x96, 0x4c, 0xf2, 0xf0, 0x70,
      0x38 },
    { 0x56, 0xfa, 0x6a, 0xa7, 0x55, 0x48, 0x09, 0x9d,
      0xcc, 0x37, 0xd7, 0xf0, 0x34, 0x25, 0xe0, 0xc3 },
};

int mbedtls_pkcs5_self_test( int verbose )
{
    mbedtls_md_context_t sha1_ctx;
    const mbedtls_md_info_t *info_sha1;
    int ret, i;
    unsigned char key[64];

    mbedtls_md_init( &sha1_ctx );

    info_sha1 = mbedtls_md_info_from_type( MBEDTLS_MD_SHA1 );
    if( info_sha1 == NULL )
    {
        ret = 1;
        goto exit;
    }

    if( ( ret = mbedtls_md_setup( &sha1_ctx, info_sha1, 1 ) ) != 0 )
    {
        ret = 1;
        goto exit;
    }

    for( i = 0; i < MAX_TESTS; i++ )
    {
        if( verbose != 0 )
            mbedtls_printf( "  PBKDF2 (SHA1) #%d: ", i );

        ret = mbedtls_pkcs5_pbkdf2_hmac( &sha1_ctx, password[i], plen[i], salt[i],
                                  slen[i], it_cnt[i], key_len[i], key );
        if( ret != 0 ||
            memcmp( result_key[i], key, key_len[i] ) != 0 )
        {
            if( verbose != 0 )
                mbedtls_printf( "failed\n" );

            ret = 1;
            goto exit;
        }

        if( verbose != 0 )
            mbedtls_printf( "passed\n" );
    }

    if( verbose != 0 )
        mbedtls_printf( "\n" );

exit:
    mbedtls_md_free( &sha1_ctx );

    return( ret );
}
#endif /* MBEDTLS_SHA1_C */

#endif /* MBEDTLS_SELF_TEST */

#endif /* MBEDTLS_PKCS5_C */
