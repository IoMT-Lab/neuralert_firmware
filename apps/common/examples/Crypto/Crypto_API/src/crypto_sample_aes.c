/**
 ****************************************************************************************
 *
 * @file crypto_sample_aes.c
 *
 * @brief Sample app of AES algorithms.
 *
 * Copyright (c) 2016-2022 Renesas Electronics. All rights reserved.
 *
 * This software ("Software") is owned by Renesas Electronics.
 *
 * By using this Software you agree that Renesas Electronics retains all
 * intellectual property and proprietary rights in and to this Software and any
 * use, reproduction, disclosure or distribution of the Software without express
 * written permission or a license agreement from Renesas Electronics is
 * strictly prohibited. This Software is solely for use on or in conjunction
 * with Renesas Electronics products.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE
 * SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. EXCEPT AS OTHERWISE
 * PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * RENESAS ELECTRONICS BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF
 * USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 *
 ****************************************************************************************
 */

#include "crypto_sample.h"
#include "mbedtls/aes.h"
#include "mbedtls/ccm.h"
#include "mbedtls/gcm.h"
#include "mbedtls_cc_aes_crypt_additional.h"

#define	CRYPTO_SAMPLE_AES_LOOP_COUNT	10000

#if defined(MBEDTLS_CIPHER_MODE_CBC)
/*
 * AES test vectors from:
 *
 * http://csrc.nist.gov/archive/aes/rijndael/rijndael-vals.zip
 */
static const unsigned char crypto_sample_aes_cbc_dec[3][16] = {
    // 128 Bits
    {
        0xFA, 0xCA, 0x37, 0xE0, 0xB0, 0xC8, 0x53, 0x73,
        0xDF, 0x70, 0x6E, 0x73, 0xF7, 0xC9, 0xAF, 0x86
    },
    // 192 Bits
    {
        0x5D, 0xF6, 0x78, 0xDD, 0x17, 0xBA, 0x4E, 0x75,
        0xB6, 0x17, 0x68, 0xC6, 0xAD, 0xEF, 0x7C, 0x7B
    },
    // 256 Bits
    {
        0x48, 0x04, 0xE1, 0x81, 0x8F, 0xE6, 0x29, 0x75,
        0x19, 0xA3, 0xE8, 0x8C, 0x57, 0x31, 0x04, 0x13
    }
};

static const unsigned char crypto_sample_aes_cbc_enc[3][16] = {
    // 128 Bits
    {
        0x8A, 0x05, 0xFC, 0x5E, 0x09, 0x5A, 0xF4, 0x84,
        0x8A, 0x08, 0xD3, 0x28, 0xD3, 0x68, 0x8E, 0x3D
    },
    // 196 Bits
    {
        0x7B, 0xD9, 0x66, 0xD5, 0x3A, 0xD8, 0xC1, 0xBB,
        0x85, 0xD2, 0xAD, 0xFA, 0xE8, 0x7B, 0xB1, 0x04
    },
    // 256 Bits
    {
        0xFE, 0x3C, 0x53, 0x65, 0x3E, 0x2F, 0x45, 0xB5,
        0x6F, 0xCD, 0x88, 0xB2, 0xCC, 0x89, 0x8F, 0xF0
    }
};

int crypto_sample_aes_cbc()
{
    int ret = -1, i, j, u, v;

    unsigned char *key = NULL;
    unsigned char *buf = NULL;
    unsigned char *iv = NULL;
    unsigned char *prv = NULL;

    mbedtls_aes_context *ctx = NULL;

    ctx = (mbedtls_aes_context *)crypto_sample_calloc(1, sizeof(mbedtls_aes_context));
    if (!ctx) {
        PRINTF("[%s] Failed to allocate AES Context\r\n", __func__);
        return ret;
    }

    key = crypto_sample_calloc(32, sizeof(unsigned char));
    if (!key) {
        PRINTF("[%s] Failed to allocate Key buffer\r\n", __func__);
        goto cleanup;
    }

    buf = crypto_sample_calloc(64, sizeof(unsigned char));
    if (!buf) {
        PRINTF("[%s] Failed to allocate buffer\r\n", __func__);
        goto cleanup;
    }

    iv = crypto_sample_calloc(16, sizeof(unsigned char));
    if (!iv) {
        PRINTF("[%s] Failed to allocate IV\r\n", __func__);
        goto cleanup;
    }

    prv = crypto_sample_calloc(16, sizeof(unsigned char));
    if (!prv) {
        PRINTF("[%s] Failed to allocate prv buffer\r\n", __func__);
        goto cleanup;
    }

    // Initialize the AES context.
    mbedtls_aes_init(ctx);

    for (i = 0; i < 6; i++) {
        u = i >> 1;
        v = i  & 1;

        PRINTF("* AES-CBC-%3d (%s): ", 128 + u * 64,
               (v == MBEDTLS_AES_DECRYPT) ? "dec" : "enc");

        memset(iv, 0x00, 16);
        memset(prv, 0x00, 16);
        memset(buf, 0x00, 16);

        if (v == MBEDTLS_AES_DECRYPT) {
            // Set the decryption key.
            mbedtls_aes_setkey_dec(ctx, key, 128 + u * 64);

            // Performs an AES-CBC decryption operation on full blocks.
            for (j = 0; j < CRYPTO_SAMPLE_AES_LOOP_COUNT ; j++) {
                mbedtls_aes_crypt_cbc(ctx, v, 16, iv, buf, buf);
            }

            // Compare output with expected output.
            if ((ret = memcmp(buf, crypto_sample_aes_cbc_dec[u], 16)) != 0) {
                PRINTF("failed\r\n");
                goto cleanup;
            }
        } else {
            // Set the encryption key.
            mbedtls_aes_setkey_enc(ctx, key, 128 + u * 64);

            // Performs an AES-CBC encryption operation on full blocks.
            for (j = 0 ; j < CRYPTO_SAMPLE_AES_LOOP_COUNT ; j++) {
                unsigned char tmp[16] = {0x00,};

                mbedtls_aes_crypt_cbc(ctx, v, 16, iv, buf, buf);

                memcpy(tmp, prv, 16);
                memcpy(prv, buf, 16);
                memcpy(buf, tmp, 16);
            }

            // Compare output with expected output.
            if ((ret = memcmp(prv, crypto_sample_aes_cbc_enc[u], 16)) != 0) {
                PRINTF("failed\r\n");
                goto cleanup;
            }
        }

        PRINTF("passed\r\n");
    }

cleanup:

    // Clear the AES context.
    mbedtls_aes_free(ctx);

    if (ctx) {
        crypto_sample_free(ctx);
    }

    if (key) {
        crypto_sample_free(key);
    }

    if (buf) {
        crypto_sample_free(buf);
    }

    if (iv) {
        crypto_sample_free(iv);
    }

    if (prv) {
        crypto_sample_free(prv);
    }

    return ret;
}
#endif // (MBEDTLS_CIPHER_MODE_CBC)

#if defined(MBEDTLS_CIPHER_MODE_CFB)
/*
 * AES-CFB128 test vectors from:
 *
 * http://csrc.nist.gov/publications/nistpubs/800-38a/sp800-38a.pdf
 */
static const unsigned char crypto_sample_aes_cfb128_key[3][32] = {
    // 128 Bits
    {
        0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
        0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
    },
    // 192 Bits
    {
        0x8E, 0x73, 0xB0, 0xF7, 0xDA, 0x0E, 0x64, 0x52,
        0xC8, 0x10, 0xF3, 0x2B, 0x80, 0x90, 0x79, 0xE5,
        0x62, 0xF8, 0xEA, 0xD2, 0x52, 0x2C, 0x6B, 0x7B
    },
    // 256 Bits
    {
        0x60, 0x3D, 0xEB, 0x10, 0x15, 0xCA, 0x71, 0xBE,
        0x2B, 0x73, 0xAE, 0xF0, 0x85, 0x7D, 0x77, 0x81,
        0x1F, 0x35, 0x2C, 0x07, 0x3B, 0x61, 0x08, 0xD7,
        0x2D, 0x98, 0x10, 0xA3, 0x09, 0x14, 0xDF, 0xF4
    }
};

static const unsigned char crypto_sample_aes_cfb128_iv[16] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
};

static const unsigned char crypto_sample_aes_cfb128_pt[64] = {
    0x6B, 0xC1, 0xBE, 0xE2, 0x2E, 0x40, 0x9F, 0x96,
    0xE9, 0x3D, 0x7E, 0x11, 0x73, 0x93, 0x17, 0x2A,
    0xAE, 0x2D, 0x8A, 0x57, 0x1E, 0x03, 0xAC, 0x9C,
    0x9E, 0xB7, 0x6F, 0xAC, 0x45, 0xAF, 0x8E, 0x51,
    0x30, 0xC8, 0x1C, 0x46, 0xA3, 0x5C, 0xE4, 0x11,
    0xE5, 0xFB, 0xC1, 0x19, 0x1A, 0x0A, 0x52, 0xEF,
    0xF6, 0x9F, 0x24, 0x45, 0xDF, 0x4F, 0x9B, 0x17,
    0xAD, 0x2B, 0x41, 0x7B, 0xE6, 0x6C, 0x37, 0x10
};

static const unsigned char crypto_sample_aes_cfb128_ct[3][64] = {
    {
        0x3B, 0x3F, 0xD9, 0x2E, 0xB7, 0x2D, 0xAD, 0x20,
        0x33, 0x34, 0x49, 0xF8, 0xE8, 0x3C, 0xFB, 0x4A,
        0xC8, 0xA6, 0x45, 0x37, 0xA0, 0xB3, 0xA9, 0x3F,
        0xCD, 0xE3, 0xCD, 0xAD, 0x9F, 0x1C, 0xE5, 0x8B,
        0x26, 0x75, 0x1F, 0x67, 0xA3, 0xCB, 0xB1, 0x40,
        0xB1, 0x80, 0x8C, 0xF1, 0x87, 0xA4, 0xF4, 0xDF,
        0xC0, 0x4B, 0x05, 0x35, 0x7C, 0x5D, 0x1C, 0x0E,
        0xEA, 0xC4, 0xC6, 0x6F, 0x9F, 0xF7, 0xF2, 0xE6
    },
    {
        0xCD, 0xC8, 0x0D, 0x6F, 0xDD, 0xF1, 0x8C, 0xAB,
        0x34, 0xC2, 0x59, 0x09, 0xC9, 0x9A, 0x41, 0x74,
        0x67, 0xCE, 0x7F, 0x7F, 0x81, 0x17, 0x36, 0x21,
        0x96, 0x1A, 0x2B, 0x70, 0x17, 0x1D, 0x3D, 0x7A,
        0x2E, 0x1E, 0x8A, 0x1D, 0xD5, 0x9B, 0x88, 0xB1,
        0xC8, 0xE6, 0x0F, 0xED, 0x1E, 0xFA, 0xC4, 0xC9,
        0xC0, 0x5F, 0x9F, 0x9C, 0xA9, 0x83, 0x4F, 0xA0,
        0x42, 0xAE, 0x8F, 0xBA, 0x58, 0x4B, 0x09, 0xFF
    },
    {
        0xDC, 0x7E, 0x84, 0xBF, 0xDA, 0x79, 0x16, 0x4B,
        0x7E, 0xCD, 0x84, 0x86, 0x98, 0x5D, 0x38, 0x60,
        0x39, 0xFF, 0xED, 0x14, 0x3B, 0x28, 0xB1, 0xC8,
        0x32, 0x11, 0x3C, 0x63, 0x31, 0xE5, 0x40, 0x7B,
        0xDF, 0x10, 0x13, 0x24, 0x15, 0xE5, 0x4B, 0x92,
        0xA1, 0x3E, 0xD0, 0xA8, 0x26, 0x7A, 0xE2, 0xF9,
        0x75, 0xA3, 0x85, 0x74, 0x1A, 0xB9, 0xCE, 0xF8,
        0x20, 0x31, 0x62, 0x3D, 0x55, 0xB1, 0xE4, 0x71
    }
};

int crypto_sample_aes_cfb()
{
    int ret = -1, i, u, v;
    unsigned char *key = NULL;
    unsigned char *buf = NULL;
    unsigned char *iv = NULL;
    size_t offset = 0;

    mbedtls_aes_context *ctx = NULL;

    ctx = (mbedtls_aes_context *)crypto_sample_calloc(1, sizeof(mbedtls_aes_context));
    if (!ctx) {
        PRINTF("[%s] Failed to allocate AES Context\r\n", __func__);
        return ret;
    }

    key = crypto_sample_calloc(32, sizeof(unsigned char));
    if (!key) {
        PRINTF("[%s] Failed to allocate Key buffer\r\n", __func__);
        goto cleanup;
    }

    buf = crypto_sample_calloc(64, sizeof(unsigned char));
    if (!buf) {
        PRINTF("[%s] Failed to allocate buffer\r\n", __func__);
        goto cleanup;
    }

    iv = crypto_sample_calloc(16, sizeof(unsigned char));
    if (!iv) {
        PRINTF("[%s] Failed to allocate IV\r\n", __func__);
        goto cleanup;
    }

    // Initialize the AES context.
    mbedtls_aes_init(ctx);

    // Test CFB128 Mode
    for (i = 0; i < 6; i++) {
        u = i >> 1;
        v = i  & 1;

        PRINTF("* AES-CFB128-%3d (%s): ", 128 + u * 64,
               (v == MBEDTLS_AES_DECRYPT) ? "dec" : "enc");

        memcpy(iv,  crypto_sample_aes_cfb128_iv, 16);
        memcpy(key, crypto_sample_aes_cfb128_key[u], 16 + u * 8);

        offset = 0;

        // Set the key.
        mbedtls_aes_setkey_enc(ctx, key, 128 + u * 64);

        if (v == MBEDTLS_AES_DECRYPT) {
            memcpy(buf, crypto_sample_aes_cfb128_ct[u], 64);

            // Perform an AES-CFB128 decryption operation.
            mbedtls_aes_crypt_cfb128(ctx, v, 64, &offset, iv, buf, buf);

            // Compare output with expected output.
            if ((ret = memcmp(buf, crypto_sample_aes_cfb128_pt, 64)) != 0) {
                PRINTF("failed\r\n");
                goto cleanup;
            }
        } else {
            memcpy(buf, crypto_sample_aes_cfb128_pt, 64);

            // Perform an AES-CFB128 encryption operation.
            mbedtls_aes_crypt_cfb128(ctx, v, 64, &offset, iv, buf, buf);

            // Compare output with expected output.
            if ((ret = memcmp(buf, crypto_sample_aes_cfb128_ct[u], 64)) != 0) {
                PRINTF("failed\r\n");
                goto cleanup;
            }
        }

        PRINTF("passed\r\n");
    }

cleanup:

    // Clear the AES context.
    mbedtls_aes_free(ctx);

    if (ctx) {
        crypto_sample_free(ctx);
    }

    if (key) {
        crypto_sample_free(key);
    }

    if (buf) {
        crypto_sample_free(buf);
    }

    if (iv) {
        crypto_sample_free(iv);
    }

    return ret;
}
#endif // (MBEDTLS_CIPHER_MODE_CFB)

/*
 * AES test vectors from:
 *
 * http://csrc.nist.gov/archive/aes/rijndael/rijndael-vals.zip
 */
static const unsigned char crypto_sample_aes_ecb_dec[3][16] = {
    {
        0x44, 0x41, 0x6A, 0xC2, 0xD1, 0xF5, 0x3C, 0x58,
        0x33, 0x03, 0x91, 0x7E, 0x6B, 0xE9, 0xEB, 0xE0
    },
    {
        0x48, 0xE3, 0x1E, 0x9E, 0x25, 0x67, 0x18, 0xF2,
        0x92, 0x29, 0x31, 0x9C, 0x19, 0xF1, 0x5B, 0xA4
    },
    {
        0x05, 0x8C, 0xCF, 0xFD, 0xBB, 0xCB, 0x38, 0x2D,
        0x1F, 0x6F, 0x56, 0x58, 0x5D, 0x8A, 0x4A, 0xDE
    }
};

static const unsigned char crypto_sample_aes_ecb_enc[3][16] = {
    {
        0xC3, 0x4C, 0x05, 0x2C, 0xC0, 0xDA, 0x8D, 0x73,
        0x45, 0x1A, 0xFE, 0x5F, 0x03, 0xBE, 0x29, 0x7F
    },
    {
        0xF3, 0xF6, 0x75, 0x2A, 0xE8, 0xD7, 0x83, 0x11,
        0x38, 0xF0, 0x41, 0x56, 0x06, 0x31, 0xB1, 0x14
    },
    {
        0x8B, 0x79, 0xEE, 0xCC, 0x93, 0xA0, 0xEE, 0x5D,
        0xFF, 0x30, 0xB4, 0xEA, 0x21, 0x63, 0x6D, 0xA4
    }
};

int crypto_sample_aes_ecb()
{
    int ret = -1, i, j, u, v;
    unsigned char *key = NULL;
    unsigned char *buf = NULL;

    mbedtls_aes_context *ctx = NULL;

    ctx = (mbedtls_aes_context *)crypto_sample_calloc(1, sizeof(mbedtls_aes_context));
    if (!ctx) {
        PRINTF("[%s] Failed to allocate AES Context\r\n", __func__);
        return ret;
    }

    key = crypto_sample_calloc(32, sizeof(unsigned char));
    if (!key) {
        PRINTF("[%s] Failed to allocate Key buffer\r\n", __func__);
        goto cleanup;
    }

    buf = crypto_sample_calloc(64, sizeof(unsigned char));
    if (!buf) {
        PRINTF("[%s] Failed to allocate buffer\r\n", __func__);
        goto cleanup;
    }

    // Initialize the AES context.
    mbedtls_aes_init(ctx);

    // Test ECB Mode
    for (i = 0; i < 6; i++) {
        u = i >> 1;
        v = i  & 1;

        PRINTF("* AES-ECB-%3d (%s): ", 128 + u * 64,
               (v == MBEDTLS_AES_DECRYPT) ? "dec" : "enc");

        memset(buf, 0x00, 16);

        if (v == MBEDTLS_AES_DECRYPT) {
            // Set the decryption key.
            mbedtls_aes_setkey_dec(ctx, key, 128 + u * 64);

            // Perform an AES single-block decryption operation.
            for (j = 0 ; j < CRYPTO_SAMPLE_AES_LOOP_COUNT ; j++) {
                mbedtls_aes_crypt_ecb(ctx, v, buf, buf);
            }

            // Compare output with expected output.
            if ((ret = memcmp(buf, crypto_sample_aes_ecb_dec[u], 16)) != 0) {
                PRINTF("failed\r\n");
                goto cleanup;
            }
        } else {
            // Set the encryption key.
            mbedtls_aes_setkey_enc(ctx, key, 128 + u * 64);

            // Perform an AES single-block encryption operation.
            for (j = 0 ; j < CRYPTO_SAMPLE_AES_LOOP_COUNT ; j++) {
                mbedtls_aes_crypt_ecb(ctx, v, buf, buf);
            }

            // Compare output with expected output.
            if ((ret = memcmp(buf, crypto_sample_aes_ecb_enc[u], 16)) != 0) {
                PRINTF("failed\r\n");
                goto cleanup;
            }
        }

        PRINTF("passed\r\n");
    }

cleanup:

    // Clear the AES context.
    mbedtls_aes_free(ctx);

    if (ctx) {
        crypto_sample_free(ctx);
    }

    if (key) {
        crypto_sample_free(key);
    }

    if (buf) {
        crypto_sample_free(buf);
    }

    return ret;
}

#if defined(MBEDTLS_CIPHER_MODE_CTR)
/*
 * AES-CTR test vectors from:
 *
 * http://www.faqs.org/rfcs/rfc3686.html
 */

static const unsigned char crypto_sample_aes_ctr_key[16] = {
    //#1. Encrypting 16 octets using AES-CTR with 128-bit key
    0xAE, 0x68, 0x52, 0xF8, 0x12, 0x10, 0x67, 0xCC,
    0x4B, 0xF7, 0xA5, 0x76, 0x55, 0x77, 0xF3, 0x9E
};

static const unsigned char crypto_sample_aes_ctr_nonce_counter[16] = {
    0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
};

static const unsigned char crypto_sample_aes_ctr_pt[48] = {
    0x53, 0x69, 0x6E, 0x67, 0x6C, 0x65, 0x20, 0x62,
    0x6C, 0x6F, 0x63, 0x6B, 0x20, 0x6D, 0x73, 0x67
};

static const unsigned char crypto_sample_aes_ctr_ct[48] = {
    0xE4, 0x09, 0x5D, 0x4F, 0xB7, 0xA7, 0xB3, 0x79,
    0x2D, 0x61, 0x75, 0xA3, 0x26, 0x13, 0x11, 0xB8
};

static const int crypto_sample_aes_ctr_len = 16;

int crypto_sample_aes_ctr()
{
    int ret = -1, i, v;
    unsigned char *key = NULL;
    unsigned char *buf = NULL;
    size_t offset = 0;
    int len = 0;
    unsigned char *nonce_counter = NULL; //nonce_counter[16];
    unsigned char *stream_block = NULL; //stream_block[16];

    mbedtls_aes_context *ctx = NULL;

    ctx = (mbedtls_aes_context *)crypto_sample_calloc(1, sizeof(mbedtls_aes_context));
    if (!ctx) {
        PRINTF("[%s] Failed to allocate AES Context\r\n", __func__);
        return ret;
    }

    key = crypto_sample_calloc(32, sizeof(unsigned char));
    if (!key) {
        PRINTF("[%s] Failed to allocate Key buffer\r\n", __func__);
        goto cleanup;
    }

    buf = crypto_sample_calloc(64, sizeof(unsigned char));
    if (!buf) {
        PRINTF("[%s] Failed to allocate buffer\r\n", __func__);
        goto cleanup;
    }

    nonce_counter = crypto_sample_calloc(16, sizeof(unsigned char));
    if (!nonce_counter) {
        PRINTF("[%s] Failed to allocate Nonce buffer\r\n", __func__);
        goto cleanup;
    }

    stream_block = crypto_sample_calloc(16, sizeof(unsigned char));

    if (!stream_block) {
        PRINTF("[%s] Failed to allocate Stream Block buffer\r\n", __func__);
        goto cleanup;
    }

    // Initialize the AES context.
    mbedtls_aes_init(ctx);

    //Test CTR Mode
    for (i = 0; i < 2; i++) {
        v = i  & 1;

        PRINTF("* AES-CTR-128 (%s): ",
               (v == MBEDTLS_AES_DECRYPT) ? "dec" : "enc");

        memcpy(nonce_counter, crypto_sample_aes_ctr_nonce_counter, 16);
        memcpy(key, crypto_sample_aes_ctr_key, 16);

        offset = 0;

        // Set the key.
        mbedtls_aes_setkey_enc(ctx, key, 128);

        if (v == MBEDTLS_AES_DECRYPT) {
            len = crypto_sample_aes_ctr_len;
            memcpy(buf, crypto_sample_aes_ctr_ct, len);

            // Perform an AES-CTR decryption operation.
            mbedtls_aes_crypt_ctr(ctx, len, &offset,
                                  nonce_counter, stream_block, buf, buf);

            // Compare output with expected output.
            if ((ret = memcmp(buf, crypto_sample_aes_ctr_pt, len)) != 0) {
                PRINTF("failed\r\n");
                goto cleanup;
            }
        } else {
            len = crypto_sample_aes_ctr_len;
            memcpy(buf, crypto_sample_aes_ctr_pt, len);

            // Perform an AES-CTR encryption operation.
            mbedtls_aes_crypt_ctr(ctx, len, &offset,
                                  nonce_counter, stream_block, buf, buf);

            // Compare output with expected output.
            if ((ret = memcmp(buf, crypto_sample_aes_ctr_ct, len)) != 0) {
                PRINTF("failed\r\n");
                goto cleanup;
            }
        }

        PRINTF("passed\r\n");
    }

cleanup:

    // Clear the AES context.
    mbedtls_aes_free(ctx);

    if (ctx) {
        crypto_sample_free(ctx);
    }

    if (key) {
        crypto_sample_free(key);
    }

    if (buf) {
        crypto_sample_free(buf);
    }

    if (nonce_counter) {
        crypto_sample_free(nonce_counter);
    }

    if (stream_block) {
        crypto_sample_free(stream_block);
    }

    return ret;
}
#endif // (MBEDTLS_CIPHER_MODE_CTR)

/*
 * Examples 1 from SP800-38C Appendix C
 */

static const unsigned char crypto_sample_ccm_key[] = {
    0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f
};

static const unsigned char crypto_sample_ccm_iv[] = {
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b
};

static const unsigned char crypto_sample_ccm_ad[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13
};

static const unsigned char crypto_sample_ccm_msg[] = {
    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
    0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
};

static const size_t crypto_sample_ccm_iv_len = 7;
static const size_t crypto_sample_ccm_add_len = 8;
static const size_t crypto_sample_ccm_msg_len = 4;
static const size_t crypto_sample_ccm_tag_len = 4;

static const unsigned char crypto_sample_ccm_res[32] = {
    0x71, 0x62, 0x01, 0x5b, 0x4d, 0xac, 0x25, 0x5d
};

int crypto_sample_aes_ccm()
{
    mbedtls_ccm_context *ctx = NULL;
    unsigned char *out = NULL;
    int ret = -1;

    ctx = (mbedtls_ccm_context *)crypto_sample_calloc(1, sizeof(mbedtls_ccm_context));

    if (!ctx) {
        PRINTF("[%s] Failed to allocate CCM Context\r\n", __func__);
        return ret;
    }

    out = crypto_sample_calloc(32, sizeof(unsigned char));

    if (!out) {
        PRINTF("[%s] Failed to allocate buffer\r\n", __func__);
        goto cleanup;
    }

    // Initialize the CCM context
    mbedtls_ccm_init(ctx);

    /* Initialize the CCM context set in the ctx parameter
     * and sets the encryption key.
     */
    ret = mbedtls_ccm_setkey(ctx, MBEDTLS_CIPHER_ID_AES,
                             crypto_sample_ccm_key, 8 * sizeof(crypto_sample_ccm_key));

    if (ret) {
        PRINTF("[%s] Failed to setup CCM(0x%x)", __func__, -ret);
        goto cleanup;
    }

    PRINTF("* CCM-AES (enc): ");

    // Encrypt a buffer using CCM.
    ret = mbedtls_ccm_encrypt_and_tag(ctx, crypto_sample_ccm_msg_len,
                                      crypto_sample_ccm_iv, crypto_sample_ccm_iv_len,
                                      crypto_sample_ccm_ad, crypto_sample_ccm_add_len,
                                      crypto_sample_ccm_msg, out,
                                      out + crypto_sample_ccm_msg_len,
                                      crypto_sample_ccm_tag_len);

    // Compare output with expected output.
    if ((ret != 0)
            || ((ret = memcmp(out, crypto_sample_ccm_res,
                              crypto_sample_ccm_msg_len + crypto_sample_ccm_tag_len)) != 0)) {
        PRINTF("failed\r\n");
        crypto_sample_hex_dump("  + out: ",
                               out, (crypto_sample_ccm_msg_len + crypto_sample_ccm_tag_len));
        crypto_sample_hex_dump("  + res: ",
                               (unsigned char *)(crypto_sample_ccm_res),
                               (crypto_sample_ccm_msg_len + crypto_sample_ccm_tag_len));
        goto cleanup;
    }

    PRINTF("passed\r\n");

    PRINTF("* CCM-AES (dec): ");

    // Perform a CCM* authenticated decryption of a buffer.
    ret = mbedtls_ccm_auth_decrypt(ctx, crypto_sample_ccm_msg_len,
                                   crypto_sample_ccm_iv, crypto_sample_ccm_iv_len,
                                   crypto_sample_ccm_ad, crypto_sample_ccm_add_len,
                                   crypto_sample_ccm_res, out,
                                   crypto_sample_ccm_res + crypto_sample_ccm_msg_len,
                                   crypto_sample_ccm_tag_len);

    // Compare output with expected output.
    if ((ret != 0)
            || (ret = memcmp(out, crypto_sample_ccm_msg, crypto_sample_ccm_msg_len)) != 0) {
        PRINTF("failed\r\n");
        crypto_sample_hex_dump("  + out: ",
                               out, (crypto_sample_ccm_msg_len + crypto_sample_ccm_tag_len));
        crypto_sample_hex_dump("  + res: ",
                               (unsigned char *)(crypto_sample_ccm_res),
                               (crypto_sample_ccm_msg_len + crypto_sample_ccm_tag_len));
        goto cleanup;
    }

    PRINTF("passed\r\n");

cleanup:

    // Clear the CCM context.
    mbedtls_ccm_free(ctx);

    if (out) {
        crypto_sample_free(out);
    }

    if (ctx) {
        crypto_sample_free(ctx);
    }

    return ret;
}

/*
 * AES-GCM test vectors from:
 *
 * http://csrc.nist.gov/groups/STM/cavp/documents/mac/gcmtestvectors.zip
 */
static const unsigned char crypto_sample_gcm_key[32] = {
    0xfe, 0xff, 0xe9, 0x92, 0x86, 0x65, 0x73, 0x1c,
    0x6d, 0x6a, 0x8f, 0x94, 0x67, 0x30, 0x83, 0x08,
    0xfe, 0xff, 0xe9, 0x92, 0x86, 0x65, 0x73, 0x1c,
    0x6d, 0x6a, 0x8f, 0x94, 0x67, 0x30, 0x83, 0x08
};

static const unsigned char crypto_sample_gcm_iv[12] = {
    0xca, 0xfe, 0xba, 0xbe, 0xfa, 0xce, 0xdb, 0xad,
    0xde, 0xca, 0xf8, 0x88
};

static const unsigned char crypto_sample_gcm_additional[20] = {
    0xfe, 0xed, 0xfa, 0xce, 0xde, 0xad, 0xbe, 0xef,
    0xfe, 0xed, 0xfa, 0xce, 0xde, 0xad, 0xbe, 0xef,
    0xab, 0xad, 0xda, 0xd2
};

static const unsigned char crypto_sample_gcm_pt[60] = {
    0xd9, 0x31, 0x32, 0x25, 0xf8, 0x84, 0x06, 0xe5,
    0xa5, 0x59, 0x09, 0xc5, 0xaf, 0xf5, 0x26, 0x9a,
    0x86, 0xa7, 0xa9, 0x53, 0x15, 0x34, 0xf7, 0xda,
    0x2e, 0x4c, 0x30, 0x3d, 0x8a, 0x31, 0x8a, 0x72,
    0x1c, 0x3c, 0x0c, 0x95, 0x95, 0x68, 0x09, 0x53,
    0x2f, 0xcf, 0x0e, 0x24, 0x49, 0xa6, 0xb5, 0x25,
    0xb1, 0x6a, 0xed, 0xf5, 0xaa, 0x0d, 0xe6, 0x57,
    0xba, 0x63, 0x7b, 0x39
};

static const unsigned char crypto_sample_gcm_ct[3][60] = {
    // 128 Bits
    {
        0x42, 0x83, 0x1e, 0xc2, 0x21, 0x77, 0x74, 0x24,
        0x4b, 0x72, 0x21, 0xb7, 0x84, 0xd0, 0xd4, 0x9c,
        0xe3, 0xaa, 0x21, 0x2f, 0x2c, 0x02, 0xa4, 0xe0,
        0x35, 0xc1, 0x7e, 0x23, 0x29, 0xac, 0xa1, 0x2e,
        0x21, 0xd5, 0x14, 0xb2, 0x54, 0x66, 0x93, 0x1c,
        0x7d, 0x8f, 0x6a, 0x5a, 0xac, 0x84, 0xaa, 0x05,
        0x1b, 0xa3, 0x0b, 0x39, 0x6a, 0x0a, 0xac, 0x97,
        0x3d, 0x58, 0xe0, 0x91
    },
    // 192 Bits
    {
        0x39, 0x80, 0xca, 0x0b, 0x3c, 0x00, 0xe8, 0x41,
        0xeb, 0x06, 0xfa, 0xc4, 0x87, 0x2a, 0x27, 0x57,
        0x85, 0x9e, 0x1c, 0xea, 0xa6, 0xef, 0xd9, 0x84,
        0x62, 0x85, 0x93, 0xb4, 0x0c, 0xa1, 0xe1, 0x9c,
        0x7d, 0x77, 0x3d, 0x00, 0xc1, 0x44, 0xc5, 0x25,
        0xac, 0x61, 0x9d, 0x18, 0xc8, 0x4a, 0x3f, 0x47,
        0x18, 0xe2, 0x44, 0x8b, 0x2f, 0xe3, 0x24, 0xd9,
        0xcc, 0xda, 0x27, 0x10
    },
    // 256 Bits
    {
        0x52, 0x2d, 0xc1, 0xf0, 0x99, 0x56, 0x7d, 0x07,
        0xf4, 0x7f, 0x37, 0xa3, 0x2a, 0x84, 0x42, 0x7d,
        0x64, 0x3a, 0x8c, 0xdc, 0xbf, 0xe5, 0xc0, 0xc9,
        0x75, 0x98, 0xa2, 0xbd, 0x25, 0x55, 0xd1, 0xaa,
        0x8c, 0xb0, 0x8e, 0x48, 0x59, 0x0d, 0xbb, 0x3d,
        0xa7, 0xb0, 0x8b, 0x10, 0x56, 0x82, 0x88, 0x38,
        0xc5, 0xf6, 0x1e, 0x63, 0x93, 0xba, 0x7a, 0x0a,
        0xbc, 0xc9, 0xf6, 0x62
    }
};

static const unsigned char crypto_sample_gcm_tag[3][16] = {
    // 128 Bits
    {
        0x5b, 0xc9, 0x4f, 0xbc, 0x32, 0x21, 0xa5, 0xdb,
        0x94, 0xfa, 0xe9, 0x5a, 0xe7, 0x12, 0x1a, 0x47
    },
    // 192 Bits
    {
        0x25, 0x19, 0x49, 0x8e, 0x80, 0xf1, 0x47, 0x8f,
        0x37, 0xba, 0x55, 0xbd, 0x6d, 0x27, 0x61, 0x8c
    },
    // 256 Bits
    {
        0x76, 0xfc, 0x6e, 0xce, 0x0f, 0x4e, 0x17, 0x68,
        0xcd, 0xdf, 0x88, 0x53, 0xbb, 0x2d, 0x55, 0x1b
    }
};

int crypto_sample_aes_gcm()
{
    mbedtls_gcm_context *ctx = NULL;	// The GCM context structure.
    unsigned char *buf = NULL;
    unsigned char *tag_buf = NULL;
    int j, ret = -1;
    mbedtls_cipher_id_t cipher = MBEDTLS_CIPHER_ID_AES;

    ctx = (mbedtls_gcm_context *)crypto_sample_calloc(1, sizeof(mbedtls_gcm_context));
    if (!ctx) {
        PRINTF("[%s] Failed to allocate GCM context\r\n", __func__);
        goto cleanup;
    }

    buf = crypto_sample_calloc(64, sizeof(unsigned char));
    if (!buf) {
        PRINTF("[%s] Failed to allocate buffer\r\n", __func__);
        goto cleanup;
    }

    tag_buf = crypto_sample_calloc(16, sizeof(unsigned char));
    if (!tag_buf) {
        PRINTF("[%s] Failed to allocate TAG buffer\r\n", __func__);
        goto cleanup;
    }

    // Initialize the specified GCM context.
    mbedtls_gcm_init(ctx);

    // AES-GCM Encryption Test
    for (j = 0; j < 3; j++) {
        int key_len = 128 + 64 * j;

        PRINTF("* AES-GCM-%3d (%s): ", key_len, "enc");

        // Associate a GCM context with a cipher algorithm and a key.
        mbedtls_gcm_setkey(ctx, cipher, crypto_sample_gcm_key, key_len);

        // Perform GCM encryption of a buffer.
        ret = mbedtls_gcm_crypt_and_tag(ctx, MBEDTLS_GCM_ENCRYPT,
                                        sizeof(crypto_sample_gcm_pt),
                                        crypto_sample_gcm_iv, sizeof(crypto_sample_gcm_iv),
                                        crypto_sample_gcm_additional, sizeof(crypto_sample_gcm_additional),
                                        crypto_sample_gcm_pt, buf,
                                        16, tag_buf);
        if (ret) {
            PRINTF("failed\r\n");
            goto cleanup;
        }

        // Compare output with expected output.
        if (((ret = memcmp(buf, crypto_sample_gcm_ct[j],
                           sizeof(crypto_sample_gcm_ct[j]))) != 0)
                || ((ret = memcmp(tag_buf, crypto_sample_gcm_tag[j], 16)) != 0)) {
            PRINTF("failed\r\n");
            goto cleanup;
        }

        // Clear a GCM context and the underlying cipher sub-context.
        mbedtls_gcm_free(ctx);

        PRINTF("passed\r\n");
    }

    //AES-GCM Decryption Test
    for (j = 0; j < 3; j++) {
        int key_len = 128 + 64 * j;

        PRINTF("* AES-GCM-%3d (%s): ", key_len, "dec");

        // Associate a GCM context with a cipher algorithm and a key.
        mbedtls_gcm_setkey(ctx, cipher, crypto_sample_gcm_key, key_len);

        // Perform GCM decryption of a buffer.
        ret = mbedtls_gcm_crypt_and_tag(ctx, MBEDTLS_GCM_DECRYPT,
                                        sizeof(crypto_sample_gcm_pt),
                                        crypto_sample_gcm_iv, sizeof(crypto_sample_gcm_iv),
                                        crypto_sample_gcm_additional, sizeof(crypto_sample_gcm_additional),
                                        crypto_sample_gcm_ct[j], buf,
                                        16, tag_buf);
        if (ret) {
            PRINTF("failed\r\n");
            goto cleanup;
        }

        // Compare output with expected output.
        if ((ret = memcmp(buf, crypto_sample_gcm_pt, sizeof(crypto_sample_gcm_pt)) != 0)
                || (ret = memcmp(tag_buf, crypto_sample_gcm_tag[j], 16) != 0)) {
            PRINTF("failed\r\n");
            goto cleanup;
        }

        // Clear a GCM context and the underlying cipher sub-context.
        mbedtls_gcm_free(ctx);

        PRINTF("passed\r\n");
    }

cleanup:

    // Clear a GCM context and the underlying cipher sub-context.
    mbedtls_gcm_free(ctx);

    if (ctx) {
        crypto_sample_free(ctx);
    }

    if (buf) {
        crypto_sample_free(buf);
    }

    if (tag_buf) {
        crypto_sample_free(tag_buf);
    }

    return ret;
}

#if defined(MBEDTLS_CIPHER_MODE_OFB)
/*
 * AES-OFB test vectors from:
 *
 * https://csrc.nist.gov/publications/detail/sp/800-38a/final
 */
static const unsigned char crypto_sample_aes_ofb_key[3][32] = {
    // 128 Bits
    {
        0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
        0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
    },
    //192 Bits
    {
        0x8E, 0x73, 0xB0, 0xF7, 0xDA, 0x0E, 0x64, 0x52,
        0xC8, 0x10, 0xF3, 0x2B, 0x80, 0x90, 0x79, 0xE5,
        0x62, 0xF8, 0xEA, 0xD2, 0x52, 0x2C, 0x6B, 0x7B
    },
    //256 Bits
    {
        0x60, 0x3D, 0xEB, 0x10, 0x15, 0xCA, 0x71, 0xBE,
        0x2B, 0x73, 0xAE, 0xF0, 0x85, 0x7D, 0x77, 0x81,
        0x1F, 0x35, 0x2C, 0x07, 0x3B, 0x61, 0x08, 0xD7,
        0x2D, 0x98, 0x10, 0xA3, 0x09, 0x14, 0xDF, 0xF4
    }
};

static const unsigned char crypto_sample_aes_ofb_iv[16] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
};

static const unsigned char crypto_sample_aes_ofb_pt[64] = {
    0x6B, 0xC1, 0xBE, 0xE2, 0x2E, 0x40, 0x9F, 0x96,
    0xE9, 0x3D, 0x7E, 0x11, 0x73, 0x93, 0x17, 0x2A,
    0xAE, 0x2D, 0x8A, 0x57, 0x1E, 0x03, 0xAC, 0x9C,
    0x9E, 0xB7, 0x6F, 0xAC, 0x45, 0xAF, 0x8E, 0x51,
    0x30, 0xC8, 0x1C, 0x46, 0xA3, 0x5C, 0xE4, 0x11,
    0xE5, 0xFB, 0xC1, 0x19, 0x1A, 0x0A, 0x52, 0xEF,
    0xF6, 0x9F, 0x24, 0x45, 0xDF, 0x4F, 0x9B, 0x17,
    0xAD, 0x2B, 0x41, 0x7B, 0xE6, 0x6C, 0x37, 0x10
};

static const unsigned char crypto_sample_aes_ofb_ct[3][64] = {
    // 128 Bits
    {
        0x3B, 0x3F, 0xD9, 0x2E, 0xB7, 0x2D, 0xAD, 0x20,
        0x33, 0x34, 0x49, 0xF8, 0xE8, 0x3C, 0xFB, 0x4A,
        0x77, 0x89, 0x50, 0x8d, 0x16, 0x91, 0x8f, 0x03,
        0xf5, 0x3c, 0x52, 0xda, 0xc5, 0x4e, 0xd8, 0x25,
        0x97, 0x40, 0x05, 0x1e, 0x9c, 0x5f, 0xec, 0xf6,
        0x43, 0x44, 0xf7, 0xa8, 0x22, 0x60, 0xed, 0xcc,
        0x30, 0x4c, 0x65, 0x28, 0xf6, 0x59, 0xc7, 0x78,
        0x66, 0xa5, 0x10, 0xd9, 0xc1, 0xd6, 0xae, 0x5e
    },
    // 192 Bits
    {
        0xCD, 0xC8, 0x0D, 0x6F, 0xDD, 0xF1, 0x8C, 0xAB,
        0x34, 0xC2, 0x59, 0x09, 0xC9, 0x9A, 0x41, 0x74,
        0xfc, 0xc2, 0x8b, 0x8d, 0x4c, 0x63, 0x83, 0x7c,
        0x09, 0xe8, 0x17, 0x00, 0xc1, 0x10, 0x04, 0x01,
        0x8d, 0x9a, 0x9a, 0xea, 0xc0, 0xf6, 0x59, 0x6f,
        0x55, 0x9c, 0x6d, 0x4d, 0xaf, 0x59, 0xa5, 0xf2,
        0x6d, 0x9f, 0x20, 0x08, 0x57, 0xca, 0x6c, 0x3e,
        0x9c, 0xac, 0x52, 0x4b, 0xd9, 0xac, 0xc9, 0x2a
    },
    // 256 Bits
    {
        0xDC, 0x7E, 0x84, 0xBF, 0xDA, 0x79, 0x16, 0x4B,
        0x7E, 0xCD, 0x84, 0x86, 0x98, 0x5D, 0x38, 0x60,
        0x4f, 0xeb, 0xdc, 0x67, 0x40, 0xd2, 0x0b, 0x3a,
        0xc8, 0x8f, 0x6a, 0xd8, 0x2a, 0x4f, 0xb0, 0x8d,
        0x71, 0xab, 0x47, 0xa0, 0x86, 0xe8, 0x6e, 0xed,
        0xf3, 0x9d, 0x1c, 0x5b, 0xba, 0x97, 0xc4, 0x08,
        0x01, 0x26, 0x14, 0x1d, 0x67, 0xf3, 0x7b, 0xe8,
        0x53, 0x8f, 0x5a, 0x8b, 0xe7, 0x40, 0xe4, 0x84
    }
};

int crypto_sample_aes_ofb()
{
    int ret = 0, i, u, v;
    unsigned int keybits;
    unsigned char *key = NULL;
    unsigned char *buf = NULL;
    unsigned char *iv = NULL;
    unsigned char stream_block[16];	//Unused parameter
    const unsigned char *expected_out;
    size_t offset;

    mbedtls_aes_context *ctx = NULL;

    ctx = (mbedtls_aes_context *)crypto_sample_calloc(1, sizeof(mbedtls_aes_context));
    if (!ctx) {
        PRINTF("[%s] Failed to allocate AES Context\r\n", __func__);
        return ret;
    }

    key = crypto_sample_calloc(32, sizeof(unsigned char));
    if (!key) {
        PRINTF("[%s] Failed to allocate Key buffer\r\n", __func__);
        goto cleanup;
    }

    buf = crypto_sample_calloc(64, sizeof(unsigned char));
    if (!buf) {
        PRINTF("[%s] Failed to allocate buffer\r\n", __func__);
        goto cleanup;
    }

    iv = crypto_sample_calloc(16, sizeof(unsigned char));
    if (!iv) {
        PRINTF("[%s] Failed to allocate IV\r\n", __func__);
        goto cleanup;
    }

    // Initialize the AES context.
    mbedtls_aes_init(ctx);

    // Test OFB mode
    for (i = 0; i < 6; i++) {
        u = i >> 1;
        v = i & 1;
        keybits = 128 + u * 64;

        PRINTF("* AES-OFB-%3d (%s): ", keybits,
               (v == MBEDTLS_AES_DECRYPT) ? "dec" : "enc");

        memcpy(iv, crypto_sample_aes_ofb_iv, 16);
        memcpy(key, crypto_sample_aes_ofb_key[u], keybits / 8);

        offset = 0;

        // Set the encryption key.
        ret = mbedtls_aes_setkey_enc(ctx, key, keybits);
        if (ret) {
            PRINTF("failed\r\n");
            goto cleanup;
        }

        if (v == MBEDTLS_AES_DECRYPT) {
            memcpy(buf, crypto_sample_aes_ofb_ct[u], 64);
            expected_out = crypto_sample_aes_ofb_pt;
        } else {
            memcpy(buf, crypto_sample_aes_ofb_pt, 64);
            expected_out = crypto_sample_aes_ofb_ct[u];
        }

        // Perform an AES-OFB (Output Feedback Mode) encryption or decryption operation.
        ret = mbedtls_aes_crypt_ofb(ctx, 64, &offset, iv, stream_block, buf, buf);
        if (ret) {
            PRINTF("failed\r\n");
            goto cleanup;
        }

        // Compare output with expected output.
        if ((ret = memcmp(buf, expected_out, 64)) != 0) {
            PRINTF("failed\r\n");
            goto cleanup;
        }

        PRINTF("passed\r\n");
    }

cleanup:

    // Clear the AES context.
    mbedtls_aes_free(ctx);

    if (ctx) {
        crypto_sample_free(ctx);
    }

    if (key) {
        crypto_sample_free(key);
    }

    if (buf) {
        crypto_sample_free(buf);
    }

    if (iv) {
        crypto_sample_free(iv);
    }

    return ret;
}
#endif // (MBEDTLS_CIPHER_MODE_OFB)

/*
 * Sample Functions for AES Algorithms
 */
void crypto_sample_aes(void *param)
{
    DA16X_UNUSED_ARG(param);

#if defined(MBEDTLS_CIPHER_MODE_CBC)
    crypto_sample_aes_cbc();
#endif // (MBEDTLS_CIPHER_MODE_CBC)

#if defined(MBEDTLS_CIPHER_MODE_CFB)
    crypto_sample_aes_cfb();
#endif // (MBEDTLS_CIPHER_MODE_CFB)

    crypto_sample_aes_ecb();

#if defined(MBEDTLS_CIPHER_MODE_CTR)
    crypto_sample_aes_ctr();
#endif // (MBEDTLS_CIPHER_MODE_CTR)

    crypto_sample_aes_ccm();

    crypto_sample_aes_gcm();

#if defined(MBEDTLS_CIPHER_MODE_OFB)
    crypto_sample_aes_ofb();
#endif // (MBEDTLS_CIPHER_MODE_OFB)

    return ;
}

/* EOF */
