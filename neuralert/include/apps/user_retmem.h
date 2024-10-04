/**
 ****************************************************************************************
 *
 * @file user_retmem.h
 *
 * @brief Header file for the example for an implementation of user retention memory API.
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

#ifndef USER_MAIN_INCLUDE_USER_RETMEM_H_
#define USER_MAIN_INCLUDE_USER_RETMEM_H_

/**
 ****************************************************************************************
 * @brief Initialize User Retention Memory
 *
 * @return void.
 ****************************************************************************************
 */
void user_retmem_init(void);

/**
 ****************************************************************************************
 * @brief De-initialize User Retention Memory
 *
 * @return void.
 ****************************************************************************************
 */
void user_retmem_deinit(void);

/**
 ****************************************************************************************
 * @brief User Retention Memory Allocation
 *
 * @Param[in] name 		 	a name of the memory
 * @Param[in] memory_ptr	a pointer of the memory
 * @Param[in] memory_size 	a size of the memory
 *
 * @return unsigned int 	0 or error code
 ****************************************************************************************
 */
unsigned int user_retmmem_allocate(char *name,
								   void **memory_ptr,
								   unsigned long memory_size);

/**
 ****************************************************************************************
 * @brief User Retention Memory Release
 *
 * @Param[in] name 		 	a name of the memory
 *
 * @return unsigned int 	0 or error code
 ****************************************************************************************
 */
unsigned int user_retmmem_release(char *name);

/**
 ****************************************************************************************
 * @brief Get the pointer of the User Retention Memory
 *
 * @Param[in] name 		 	a name of the memory
 * @Param[in] memory_ptr	a pointer of the memory
 *
 * @return unsigned int 	the size of the memory
 ****************************************************************************************
 */
unsigned int user_retmmem_get(char *name, unsigned char **data);

/**
 ****************************************************************************************
 * @brief Print out the status of the User Retention Memory
 *
 * @return void.
 ****************************************************************************************
 */
void user_retmmem_info_get(void);

#endif /* USER_MAIN_INCLUDE_USER_RETMEM_H_ */
