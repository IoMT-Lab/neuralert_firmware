/**
 *****************************************************************************************
 * @file    includes.h
 * @brief   wpa_supplicant/hostapd - Default include files from wpa_supplicant-2.4
 *****************************************************************************************
 */


/*
 * wpa_supplicant/hostapd - Default include files
 * Copyright (c) 2005-2006, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 *
 * This header file is included into all C files so that commonly used header
 * files can be selected with OS specific ifdef blocks in one place instead of
 * having to have OS/C library specific selection in many files.
 */

/* Include possible build time configuration before including anything else */


#include "FreeRTOS.h"
#include "event_groups.h"
#include "queue.h"
#include "semphr.h"

#include "da16x_system.h"
#include "supp_def.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <signal.h>
#include <errno.h>

#include "supp_types.h"
#include "supp_common.h"
#include "wpabuf.h"
#include "supp_debug.h"

/* EOF */
