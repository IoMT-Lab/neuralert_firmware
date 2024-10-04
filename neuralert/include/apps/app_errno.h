/**
 ****************************************************************************************
 *
 * @file app_errno.h
 *
 * @brief Define for application error number
 *
 *
 * Modified from Renesas Electronics SDK example code with the same name
 *
 * Copyright (c) 2024, Vanderbilt University
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************************
 */

#ifndef APP_ERRNO_H
#define APP_ERRNO_H

#ifdef __cplusplus
extern "C" {
#endif


/*
 * Common error code. It seems like NX_XXX error code.
 *  0x0000 ~ -0x00FF: Common error code
 * -0x0100 ~ -0x01FF: specific app#1
 * -0x0200 ~ -0x02FF: specific app#2
 */

#define DA_APP_SUCCESS                      -0x0000
#define DA_APP_NO_PACKET                    -0x0001
#define DA_APP_UNDERFLOW                    -0x0002
#define DA_APP_OVERFLOW                     -0x0003
#define DA_APP_NO_MAPPING                   -0x0004
#define DA_APP_DELETED                      -0x0005
#define DA_APP_POOL_ERROR                   -0x0006
#define DA_APP_PTR_ERROR                    -0x0007
#define DA_APP_WAIT_ERROR                   -0x0008
#define DA_APP_SIZE_ERROR                   -0x0009
#define DA_APP_OPTION_ERROR                 -0x000A
#define DA_APP_MALLOC_ERROR                 -0x000B

#define DA_APP_DELETE_ERROR                 -0x0010
#define DA_APP_CALLER_ERROR                 -0x0011
#define DA_APP_INVALID_PACKET               -0x0012
#define DA_APP_INVALID_SOCKET               -0x0013
#define DA_APP_NOT_ENABLED                  -0x0014
#define DA_APP_ALREADY_ENABLED              -0x0015
#define DA_APP_ENTRY_NOT_FOUND              -0x0016
#define DA_APP_NO_MORE_ENTRIES              -0x0017
#define DA_APP_ARP_TIMER_ERROR              -0x0018
#define DA_APP_RESERVED_CODE0               -0x0019
#define DA_APP_WAIT_ABORTED                 -0x001A

#define DA_APP_IP_INTERNAL_ERROR            -0x0020
#define DA_APP_IP_ADDRESS_ERROR             -0x0021
#define DA_APP_ALREADY_BOUND                -0x0022
#define DA_APP_PORT_UNAVAILABLE             -0x0023
#define DA_APP_NOT_BOUND                    -0x0024
#define DA_APP_RESERVED_CODE1               -0x0025
#define DA_APP_SOCKET_UNBOUND               -0x0026
#define DA_APP_NOT_CREATED                  -0x0027
#define DA_APP_SOCKETS_BOUND                -0x0028
#define DA_APP_NO_RESPONSE                  -0x0029
#define DA_APP_SOCKET_NOT_CREATE            -0x002A
#define DA_APP_SOCKET_OPT_ERROR             -0x002B

#define DA_APP_POOL_DELETED                 -0x0030
#define DA_APP_ALREADY_RELEASED             -0x0031
#define DA_APP_RESERVED_CODE2               -0x0032
#define DA_APP_MAX_LISTEN                   -0x0033
#define DA_APP_DUPLICATE_LISTEN             -0x0034
#define DA_APP_NOT_CLOSED                   -0x0035
#define DA_APP_NOT_LISTEN_STATE             -0x0036
#define DA_APP_IN_PROGRESS                  -0x0037
#define DA_APP_NOT_CONNECTED                -0x0038
#define DA_APP_WINDOW_OVERFLOW              -0x0039

#define DA_APP_ALREADY_SUSPENDED            -0x0040
#define DA_APP_DISCONNECT_FAILED            -0x0041
#define DA_APP_STILL_BOUND                  -0x0042
#define DA_APP_NOT_SUCCESSFUL               -0x0043
#define DA_APP_UNHANDLED_COMMAND            -0x0044
#define DA_APP_NO_FREE_PORTS                -0x0045
#define DA_APP_INVALID_PORT                 -0x0046
#define DA_APP_INVALID_RELISTEN             -0x0047
#define DA_APP_CONNECTION_PENDING           -0x0048
#define DA_APP_TX_QUEUE_DEPTH               -0x0049
#define DA_APP_NOT_IMPLEMENTED              -0x004A
#define DA_APP_NOT_SUPPORTED                -0x004B
#define DA_APP_INVALID_INTERFACE            -0x004C
#define DA_APP_INVALID_PARAMETERS           -0x004D
#define DA_APP_NOT_FOUND                    -0x004E
#define DA_APP_CANNOT_START                 -0x004F

#define DA_APP_NO_INTERFACE_ADDRESS         -0x0050
#define DA_APP_INVALID_MTU_DATA             -0x0051
#define DA_APP_DUPLICATED_ENTRY             -0x0052
#define DA_APP_PACKET_OFFSET_ERROR          -0x0053
#define DA_APP_OPTION_HEADER_ERROR          -0x0054
#define DA_APP_CONTINUE                     -0x0055
#define DA_APP_PARAMETER_ERROR              -0x0056

#define DA_APP_SSL_ROLE_NOT_SUPPORT         -0x0057
#define DA_APP_SSL_CFG_CONF_ID_TYPE         -0x0058
#define DA_APP_SSL_CFG_CONF_ID_NOT_SUPPORT  -0x0059
#define DA_APP_SSL_CFG_TLS_VER              -0x005A
#define DA_APP_SSL_CFG_CA_CERT              -0x005B
#define DA_APP_SSL_CFG_CERT                 -0x005C
#define DA_APP_SSL_CFG_SET_SNI              -0x005D
#define DA_APP_SSL_SVR_VAILD_TYPE           -0x005E
#define DA_APP_SSL_SVR_VAILD_RANGE          -0x005F
#define DA_APP_SSL_RX_BUF_LENTH             -0x0060
#define DA_APP_SSL_TX_BUF_LENTH             -0x0061
#define DA_APP_SSL_CONN_INVALID_PORT_NUM    -0x0062
#define DA_APP_SSL_CONN_UNKNOWN_HOST        -0x0063
#define DA_APP_SSL_CONN_SETUP_CFG_ERR       -0x0064
#define DA_APP_SSL_CONN_TLS_CLIENT_RUN_ERR  -0x0065
#define DA_APP_SSL_CLR_CTX_NVRAM_FAIL       -0x0066
#define DA_APP_SSL_SAVE_CTX_NVRAM_FAIL      -0x0067

#ifdef __cplusplus
}
#endif

#endif // APP_ERRNO_H
