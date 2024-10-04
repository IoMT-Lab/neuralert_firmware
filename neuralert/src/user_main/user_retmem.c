/**
 ****************************************************************************************
 *
 * @file user_retmem.c
 *
 * @brief Example for an implementation of user retention memory API.
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

#include "common_def.h"
#include "da16x_system.h"
#include "da16x_dpm.h"
#include "lwip/err.h"
#include "rtc.h"
#include "user_retmem.h"

#if defined (CFG_USE_RETMEM_WITHOUT_DPM)

extern int dpm_get_wakeup_source(void);
extern void da16x_dpm_rtm_mem_init(void *mem_ptr, size_t mem_len);
extern void da16x_dpm_rtm_mem_recovery(void *mem_ptr, size_t mem_len, void *lfree_ptr);
extern void *da16x_dpm_rtm_mem_get_free_ptr();
extern void *da16x_dpm_rtm_mem_calloc(uint16_t count, uint16_t size);
extern void da16x_dpm_rtm_mem_free(void *rmem);
extern void da16x_dpm_rtm_mem_status();

static unsigned char	user_rtm_pool_init_done	= pdFALSE;
static dpm_user_rtm_pool *user_retmem_pool = (dpm_user_rtm_pool *)RTM_USER_POOL_PTR;

static void user_retmem_pool_clear(void)
{
	memset((void *)user_retmem_pool, 0x00, sizeof(dpm_user_rtm_pool));

	return;
}

static void user_retmem_clear(void)
{
	memset((void *)RTM_USER_DATA_PTR, 0x00, USER_DATA_ALLOC_SZ);

	return;
}

static void user_retmem_add(dpm_user_rtm *data)
{
	dpm_user_rtm *cur = NULL;

	taskENTER_CRITICAL();

	if (user_retmem_pool->first_user_addr != NULL) {
		//goto end of linked-list
		for (cur = user_retmem_pool->first_user_addr;
			cur->next_user_addr != NULL ;
			cur = cur ->next_user_addr) {
		}

		cur->next_user_addr = data;
	} else {
		user_retmem_pool->first_user_addr = data;
	}

	taskEXIT_CRITICAL();

	return;
}

static dpm_user_rtm *user_retmem_remove(char *name)
{
	dpm_user_rtm    *cur = NULL;
	dpm_user_rtm    *prev = NULL;

	taskENTER_CRITICAL();
	if (user_retmem_pool->first_user_addr != NULL) {
		cur = user_retmem_pool->first_user_addr;

		if (dpm_strcmp(cur->name, name) == 0) {
			user_retmem_pool->first_user_addr = cur->next_user_addr;
			taskEXIT_CRITICAL();
			return cur;
		} else {
			do {
				prev= cur;
				cur= cur->next_user_addr;

				if (cur != NULL && dpm_strcmp(cur->name, name) == 0) {
					prev->next_user_addr = cur->next_user_addr;
					taskEXIT_CRITICAL();
					return cur;
				}
			} while (cur != NULL && cur->next_user_addr != NULL);
		}
	}
	taskEXIT_CRITICAL();
	return NULL;
}

static dpm_user_rtm *user_retmem_search(char *name)
{
	dpm_user_rtm *cur = NULL;

	taskENTER_CRITICAL();
	if (user_retmem_pool->first_user_addr != NULL) {
		for (cur = user_retmem_pool->first_user_addr; cur != NULL; cur = cur->next_user_addr) {
			if (dpm_strcmp(cur->name, name) == 0) {
				taskEXIT_CRITICAL();
				return cur;
			}
		}
	}
	taskEXIT_CRITICAL();
	return NULL;
}

void user_retmem_init(void)
{
	PRINTF("\n**Neuralert: user_retmem_init()\n"); // FRSDEBUG

	if (!user_rtm_pool_init_done) {
		if (dpm_get_wakeup_source() & WAKEUP_SOURCE_POR) {

			PRINTF("\n**Neuralert: user_retmem_init() initializing retmem\n"); // FRSDEBUG

			//to clear user rtm pool
			user_retmem_pool_clear();

			//to clear user rtm data
			user_retmem_clear();

			da16x_dpm_rtm_mem_init(RTM_USER_DATA_PTR, USER_DATA_ALLOC_SZ);

			user_retmem_pool->free_ptr = da16x_dpm_rtm_mem_get_free_ptr();

			user_retmem_pool->first_user_addr = NULL;

			PRINTF("User RETMEM initialized.\r\n");
		} else {
			da16x_dpm_rtm_mem_recovery(RTM_USER_DATA_PTR, USER_DATA_ALLOC_SZ,
					user_retmem_pool->free_ptr);
			PRINTF("User RETMEM recovered.\r\n");
		}

		// Set flag to mark User RTM Initialize done ...
		user_rtm_pool_init_done = pdTRUE;
	}
}

void user_retmem_deinit(void)
{
    //to clear user rtm pool
	user_retmem_pool_clear();

    //to clear user rtm data
	user_retmem_clear();

    PRINTF("User RETMEM removed.\r\n");
}

unsigned int user_retmmem_allocate(char *name,
								   void **memory_ptr,
								   unsigned long memory_size)
{
    dpm_user_rtm *user_memory = NULL;

    //to check parameters
    if ((name == NULL)
            || (strlen(name) >= REG_NAME_DPM_MAX_LEN)
            || (memory_size == 0)) {
        return ER_INVALID_PARAMETERS;
    }

    //to check duplicated name
    if (user_retmem_search(name) != NULL) {
        PRINTF("[%s] Already registered user rtm(%s)\n", __func__, name);
        return ER_DUPLICATED_ENTRY;
    }

    //to allocate memory
	user_memory = (dpm_user_rtm *)da16x_dpm_rtm_mem_calloc((memory_size + sizeof(dpm_user_rtm)), sizeof(unsigned char));
	if (user_memory == NULL) {
		PRINTF("[%s] Failed to allocate memory(size:%d)\n", __func__, (memory_size + sizeof(dpm_user_rtm)));
		return ER_NO_MEMORY;
	}

	//update lfree. It has to be called after allocate & free
	user_retmem_pool->free_ptr = da16x_dpm_rtm_mem_get_free_ptr();

    //to set user rtm
    strcpy(user_memory->name, name);
    user_memory->size = memory_size;
    user_memory->start_user_addr = (UCHAR *)(user_memory) + sizeof(dpm_user_rtm);
    user_memory->next_user_addr = NULL;

    //to add user rtm into chain
    user_retmem_add(user_memory);

    //to set user memory point
    *memory_ptr = user_memory->start_user_addr;

    return 0;
}

unsigned int user_retmmem_release(char *name)
{
    dpm_user_rtm *cur_mem = NULL;

    if ((name == NULL)
            || (strlen(name) >= REG_NAME_DPM_MAX_LEN)) {
        return  ER_INVALID_PARAMETERS;
    }

    cur_mem = user_retmem_remove(name);
    if (cur_mem != NULL) {
        da16x_dpm_rtm_mem_free(cur_mem);

		//update lfree. It has to be called after allocate & free
        user_retmem_pool->free_ptr = da16x_dpm_rtm_mem_get_free_ptr();

        return 0;
    }

    PRINTF("[%s] Not found user data(%s)\n", __func__, name);

    return ER_NOT_FOUND;
}

unsigned int user_retmmem_get(char *name, unsigned char **data)
{
	dpm_user_rtm *cur = NULL;

    if ((name == NULL)
			|| (strlen(name) >= REG_NAME_DPM_MAX_LEN)) {
		return  0;
	}

	cur = user_retmem_search(name);
	if (cur != NULL) {
		*data = cur->start_user_addr;
		return cur->size;
	}

	return 0;
}

void user_retmmem_info_get(void)
{
	da16x_dpm_rtm_mem_status();
}

#endif // CFG_USE_RETMEM_WITHOUT_DPM
