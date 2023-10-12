################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Stclients/Inteprod/Neuralert/Source/Stage10f/SRC-9079-Stage10f/core/system/src/common/library/da16x_image.c \
C:/Stclients/Inteprod/Neuralert/Source/Stage10f/SRC-9079-Stage10f/core/system/src/common/library/da16x_oops.c \
C:/Stclients/Inteprod/Neuralert/Source/Stage10f/SRC-9079-Stage10f/core/system/src/common/library/sys_image.c \
C:/Stclients/Inteprod/Neuralert/Source/Stage10f/SRC-9079-Stage10f/core/system/src/common/library/sys_sort.c 

OBJS += \
./core/system/src/common/library/da16x_image.o \
./core/system/src/common/library/da16x_oops.o \
./core/system/src/common/library/sys_image.o \
./core/system/src/common/library/sys_sort.o 

C_DEPS += \
./core/system/src/common/library/da16x_image.d \
./core/system/src/common/library/da16x_oops.d \
./core/system/src/common/library/sys_image.d \
./core/system/src/common/library/sys_sort.d 


# Each subdirectory must supply rules for building sources it contributes
core/system/src/common/library/da16x_image.o: C:/Stclients/Inteprod/Neuralert/Source/Stage10f/SRC-9079-Stage10f/core/system/src/common/library/da16x_image.c core/system/src/common/library/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wextra  -g3 -DDEBUG -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -D__DA16200__ -D__ENABLE_SAMPLE_APP__ -D__TCP_CLIENT_SLEEP2_SAMPLE__ -I../../../include -I../../../../../common_config -I../../../../../../../../core/main/include -I../../../../../../../../core/main/config -I../../../../../../../../core/config -I../../../../../../../../core/freertos/include -I../../../../../../../../core/freertos/portable/GCC/ARM_CM4F -I../../../../../../../../core/libraries/3rdparty/lwip/src/include -I../../../../../../../../core/libraries/3rdparty/lwip_osal/include -I../../../../../../../../core/libraries/3rdparty/lwip/src/include/lwip/apps -I../../../../../../../../core/libraries/3rdparty/crypto/include -I../../../../../../../../core/libraries/3rdparty/json -I../../../../../../../../core/bsp/cmsis/include -I../../../../../../../../core/bsp/cortexm/include -I../../../../../../../../core/bsp/driver/include -I../../../../../../../../core/bsp/driver/include/DA16200 -I../../../../../../../da16200/get_started/include/apps -I../../../../../../../../core/system/include -I../../../../../../../../core/system/include/at_cmd -I../../../../../../../../core/system/include/dpm -I../../../../../../../../core/system/include/common -I../../../../../../../../core/system/include/common/library -I../../../../../../../../core/system/include/crypto -I../../../../../../../../core/system/include/crypto/cryptocell -I../../../../../../../../core/system/include/crypto/mbedtls -I../../../../../../../../core/system/include/crypto/shared -I../../../../../../../../core/system/include/crypto/shared/hw/include -I../../../../../../../../core/system/include/crypto/shared/include -I../../../../../../../../core/system/include/crypto/shared/include/crypto_api -I../../../../../../../../core/system/include/crypto/shared/include/pal -I../../../../../../../../core/system/include/crypto/shared/include/pal/freertos -I../../../../../../../../core/system/include/crypto/shared/include/proj/cc3x -I../../../../../../../../core/system/include/crypto/shared/include/trng -I../../../../../../../../core/system/include/crypto/shared/include/cc_util -I../../../../../../../../core/system/include/crypto/shared/include/sbrom -I../../../../../../../../core/system/include/crypto/shared/include/crypto_api/cc3x -I../../../../../../../../core/system/include/mqtt_client -I../../../../../../../../core/system/include/mqtt_client/lib -I../../../../../../../../core/system/include/temp -I../../../../../../../../core/system/include/provision -I../../../../../../../../core/wifistack/supplicant/src -I../../../../../../../../core/wifistack/supplicant/src/ap -I../../../../../../../../core/wifistack/supplicant/src/common -I../../../../../../../../core/wifistack/supplicant/src/crypto -I../../../../../../../../core/wifistack/supplicant/src/drivers -I../../../../../../../../core/wifistack/supplicant/src/eap_common -I../../../../../../../../core/wifistack/supplicant/src/eap_peer -I../../../../../../../../core/wifistack/supplicant/src/eap_server -I../../../../../../../../core/wifistack/supplicant/src/eapol_auth -I../../../../../../../../core/wifistack/supplicant/src/eapol_supp -I../../../../../../../../core/wifistack/supplicant/src/l2_packet -I../../../../../../../../core/wifistack/supplicant/src/p2p -I../../../../../../../../core/wifistack/supplicant/src/pae -I../../../../../../../../core/wifistack/supplicant/src/radius -I../../../../../../../../core/wifistack/supplicant/src/rsn_supp -I../../../../../../../../core/wifistack/supplicant/src/utils -I../../../../../../../../core/wifistack/supplicant/wpa_supplicant -I../../../../../../../../core/wifistack/wpa_cli -I../../../../../../../../core/system/include/coap -I../../../../../../../../version/genfiles/ -I../../../../../../../../core/system/include/ramlib -I../../../../../../../../core/segger_tools/Config -I../../../../../../../../core/segger_tools/OS -I../../../../../../../../core/segger_tools/SEGGER -I../../../../../../../../core/system/include/ota -I../../../../../../../../core/wifistack/romac/include -I../../../../../../../app_common/include/provisioning -I../../../../../../../app_common/include/util -include../../../../../../../../core/main/config/custom_config.h -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

core/system/src/common/library/da16x_oops.o: C:/Stclients/Inteprod/Neuralert/Source/Stage10f/SRC-9079-Stage10f/core/system/src/common/library/da16x_oops.c core/system/src/common/library/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wextra  -g3 -DDEBUG -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -D__DA16200__ -D__ENABLE_SAMPLE_APP__ -D__TCP_CLIENT_SLEEP2_SAMPLE__ -I../../../include -I../../../../../common_config -I../../../../../../../../core/main/include -I../../../../../../../../core/main/config -I../../../../../../../../core/config -I../../../../../../../../core/freertos/include -I../../../../../../../../core/freertos/portable/GCC/ARM_CM4F -I../../../../../../../../core/libraries/3rdparty/lwip/src/include -I../../../../../../../../core/libraries/3rdparty/lwip_osal/include -I../../../../../../../../core/libraries/3rdparty/lwip/src/include/lwip/apps -I../../../../../../../../core/libraries/3rdparty/crypto/include -I../../../../../../../../core/libraries/3rdparty/json -I../../../../../../../../core/bsp/cmsis/include -I../../../../../../../../core/bsp/cortexm/include -I../../../../../../../../core/bsp/driver/include -I../../../../../../../../core/bsp/driver/include/DA16200 -I../../../../../../../da16200/get_started/include/apps -I../../../../../../../../core/system/include -I../../../../../../../../core/system/include/at_cmd -I../../../../../../../../core/system/include/dpm -I../../../../../../../../core/system/include/common -I../../../../../../../../core/system/include/common/library -I../../../../../../../../core/system/include/crypto -I../../../../../../../../core/system/include/crypto/cryptocell -I../../../../../../../../core/system/include/crypto/mbedtls -I../../../../../../../../core/system/include/crypto/shared -I../../../../../../../../core/system/include/crypto/shared/hw/include -I../../../../../../../../core/system/include/crypto/shared/include -I../../../../../../../../core/system/include/crypto/shared/include/crypto_api -I../../../../../../../../core/system/include/crypto/shared/include/pal -I../../../../../../../../core/system/include/crypto/shared/include/pal/freertos -I../../../../../../../../core/system/include/crypto/shared/include/proj/cc3x -I../../../../../../../../core/system/include/crypto/shared/include/trng -I../../../../../../../../core/system/include/crypto/shared/include/cc_util -I../../../../../../../../core/system/include/crypto/shared/include/sbrom -I../../../../../../../../core/system/include/crypto/shared/include/crypto_api/cc3x -I../../../../../../../../core/system/include/mqtt_client -I../../../../../../../../core/system/include/mqtt_client/lib -I../../../../../../../../core/system/include/temp -I../../../../../../../../core/system/include/provision -I../../../../../../../../core/wifistack/supplicant/src -I../../../../../../../../core/wifistack/supplicant/src/ap -I../../../../../../../../core/wifistack/supplicant/src/common -I../../../../../../../../core/wifistack/supplicant/src/crypto -I../../../../../../../../core/wifistack/supplicant/src/drivers -I../../../../../../../../core/wifistack/supplicant/src/eap_common -I../../../../../../../../core/wifistack/supplicant/src/eap_peer -I../../../../../../../../core/wifistack/supplicant/src/eap_server -I../../../../../../../../core/wifistack/supplicant/src/eapol_auth -I../../../../../../../../core/wifistack/supplicant/src/eapol_supp -I../../../../../../../../core/wifistack/supplicant/src/l2_packet -I../../../../../../../../core/wifistack/supplicant/src/p2p -I../../../../../../../../core/wifistack/supplicant/src/pae -I../../../../../../../../core/wifistack/supplicant/src/radius -I../../../../../../../../core/wifistack/supplicant/src/rsn_supp -I../../../../../../../../core/wifistack/supplicant/src/utils -I../../../../../../../../core/wifistack/supplicant/wpa_supplicant -I../../../../../../../../core/wifistack/wpa_cli -I../../../../../../../../core/system/include/coap -I../../../../../../../../version/genfiles/ -I../../../../../../../../core/system/include/ramlib -I../../../../../../../../core/segger_tools/Config -I../../../../../../../../core/segger_tools/OS -I../../../../../../../../core/segger_tools/SEGGER -I../../../../../../../../core/system/include/ota -I../../../../../../../../core/wifistack/romac/include -I../../../../../../../app_common/include/provisioning -I../../../../../../../app_common/include/util -include../../../../../../../../core/main/config/custom_config.h -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

core/system/src/common/library/sys_image.o: C:/Stclients/Inteprod/Neuralert/Source/Stage10f/SRC-9079-Stage10f/core/system/src/common/library/sys_image.c core/system/src/common/library/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wextra  -g3 -DDEBUG -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -D__DA16200__ -D__ENABLE_SAMPLE_APP__ -D__TCP_CLIENT_SLEEP2_SAMPLE__ -I../../../include -I../../../../../common_config -I../../../../../../../../core/main/include -I../../../../../../../../core/main/config -I../../../../../../../../core/config -I../../../../../../../../core/freertos/include -I../../../../../../../../core/freertos/portable/GCC/ARM_CM4F -I../../../../../../../../core/libraries/3rdparty/lwip/src/include -I../../../../../../../../core/libraries/3rdparty/lwip_osal/include -I../../../../../../../../core/libraries/3rdparty/lwip/src/include/lwip/apps -I../../../../../../../../core/libraries/3rdparty/crypto/include -I../../../../../../../../core/libraries/3rdparty/json -I../../../../../../../../core/bsp/cmsis/include -I../../../../../../../../core/bsp/cortexm/include -I../../../../../../../../core/bsp/driver/include -I../../../../../../../../core/bsp/driver/include/DA16200 -I../../../../../../../da16200/get_started/include/apps -I../../../../../../../../core/system/include -I../../../../../../../../core/system/include/at_cmd -I../../../../../../../../core/system/include/dpm -I../../../../../../../../core/system/include/common -I../../../../../../../../core/system/include/common/library -I../../../../../../../../core/system/include/crypto -I../../../../../../../../core/system/include/crypto/cryptocell -I../../../../../../../../core/system/include/crypto/mbedtls -I../../../../../../../../core/system/include/crypto/shared -I../../../../../../../../core/system/include/crypto/shared/hw/include -I../../../../../../../../core/system/include/crypto/shared/include -I../../../../../../../../core/system/include/crypto/shared/include/crypto_api -I../../../../../../../../core/system/include/crypto/shared/include/pal -I../../../../../../../../core/system/include/crypto/shared/include/pal/freertos -I../../../../../../../../core/system/include/crypto/shared/include/proj/cc3x -I../../../../../../../../core/system/include/crypto/shared/include/trng -I../../../../../../../../core/system/include/crypto/shared/include/cc_util -I../../../../../../../../core/system/include/crypto/shared/include/sbrom -I../../../../../../../../core/system/include/crypto/shared/include/crypto_api/cc3x -I../../../../../../../../core/system/include/mqtt_client -I../../../../../../../../core/system/include/mqtt_client/lib -I../../../../../../../../core/system/include/temp -I../../../../../../../../core/system/include/provision -I../../../../../../../../core/wifistack/supplicant/src -I../../../../../../../../core/wifistack/supplicant/src/ap -I../../../../../../../../core/wifistack/supplicant/src/common -I../../../../../../../../core/wifistack/supplicant/src/crypto -I../../../../../../../../core/wifistack/supplicant/src/drivers -I../../../../../../../../core/wifistack/supplicant/src/eap_common -I../../../../../../../../core/wifistack/supplicant/src/eap_peer -I../../../../../../../../core/wifistack/supplicant/src/eap_server -I../../../../../../../../core/wifistack/supplicant/src/eapol_auth -I../../../../../../../../core/wifistack/supplicant/src/eapol_supp -I../../../../../../../../core/wifistack/supplicant/src/l2_packet -I../../../../../../../../core/wifistack/supplicant/src/p2p -I../../../../../../../../core/wifistack/supplicant/src/pae -I../../../../../../../../core/wifistack/supplicant/src/radius -I../../../../../../../../core/wifistack/supplicant/src/rsn_supp -I../../../../../../../../core/wifistack/supplicant/src/utils -I../../../../../../../../core/wifistack/supplicant/wpa_supplicant -I../../../../../../../../core/wifistack/wpa_cli -I../../../../../../../../core/system/include/coap -I../../../../../../../../version/genfiles/ -I../../../../../../../../core/system/include/ramlib -I../../../../../../../../core/segger_tools/Config -I../../../../../../../../core/segger_tools/OS -I../../../../../../../../core/segger_tools/SEGGER -I../../../../../../../../core/system/include/ota -I../../../../../../../../core/wifistack/romac/include -I../../../../../../../app_common/include/provisioning -I../../../../../../../app_common/include/util -include../../../../../../../../core/main/config/custom_config.h -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

core/system/src/common/library/sys_sort.o: C:/Stclients/Inteprod/Neuralert/Source/Stage10f/SRC-9079-Stage10f/core/system/src/common/library/sys_sort.c core/system/src/common/library/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wextra  -g3 -DDEBUG -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -D__DA16200__ -D__ENABLE_SAMPLE_APP__ -D__TCP_CLIENT_SLEEP2_SAMPLE__ -I../../../include -I../../../../../common_config -I../../../../../../../../core/main/include -I../../../../../../../../core/main/config -I../../../../../../../../core/config -I../../../../../../../../core/freertos/include -I../../../../../../../../core/freertos/portable/GCC/ARM_CM4F -I../../../../../../../../core/libraries/3rdparty/lwip/src/include -I../../../../../../../../core/libraries/3rdparty/lwip_osal/include -I../../../../../../../../core/libraries/3rdparty/lwip/src/include/lwip/apps -I../../../../../../../../core/libraries/3rdparty/crypto/include -I../../../../../../../../core/libraries/3rdparty/json -I../../../../../../../../core/bsp/cmsis/include -I../../../../../../../../core/bsp/cortexm/include -I../../../../../../../../core/bsp/driver/include -I../../../../../../../../core/bsp/driver/include/DA16200 -I../../../../../../../da16200/get_started/include/apps -I../../../../../../../../core/system/include -I../../../../../../../../core/system/include/at_cmd -I../../../../../../../../core/system/include/dpm -I../../../../../../../../core/system/include/common -I../../../../../../../../core/system/include/common/library -I../../../../../../../../core/system/include/crypto -I../../../../../../../../core/system/include/crypto/cryptocell -I../../../../../../../../core/system/include/crypto/mbedtls -I../../../../../../../../core/system/include/crypto/shared -I../../../../../../../../core/system/include/crypto/shared/hw/include -I../../../../../../../../core/system/include/crypto/shared/include -I../../../../../../../../core/system/include/crypto/shared/include/crypto_api -I../../../../../../../../core/system/include/crypto/shared/include/pal -I../../../../../../../../core/system/include/crypto/shared/include/pal/freertos -I../../../../../../../../core/system/include/crypto/shared/include/proj/cc3x -I../../../../../../../../core/system/include/crypto/shared/include/trng -I../../../../../../../../core/system/include/crypto/shared/include/cc_util -I../../../../../../../../core/system/include/crypto/shared/include/sbrom -I../../../../../../../../core/system/include/crypto/shared/include/crypto_api/cc3x -I../../../../../../../../core/system/include/mqtt_client -I../../../../../../../../core/system/include/mqtt_client/lib -I../../../../../../../../core/system/include/temp -I../../../../../../../../core/system/include/provision -I../../../../../../../../core/wifistack/supplicant/src -I../../../../../../../../core/wifistack/supplicant/src/ap -I../../../../../../../../core/wifistack/supplicant/src/common -I../../../../../../../../core/wifistack/supplicant/src/crypto -I../../../../../../../../core/wifistack/supplicant/src/drivers -I../../../../../../../../core/wifistack/supplicant/src/eap_common -I../../../../../../../../core/wifistack/supplicant/src/eap_peer -I../../../../../../../../core/wifistack/supplicant/src/eap_server -I../../../../../../../../core/wifistack/supplicant/src/eapol_auth -I../../../../../../../../core/wifistack/supplicant/src/eapol_supp -I../../../../../../../../core/wifistack/supplicant/src/l2_packet -I../../../../../../../../core/wifistack/supplicant/src/p2p -I../../../../../../../../core/wifistack/supplicant/src/pae -I../../../../../../../../core/wifistack/supplicant/src/radius -I../../../../../../../../core/wifistack/supplicant/src/rsn_supp -I../../../../../../../../core/wifistack/supplicant/src/utils -I../../../../../../../../core/wifistack/supplicant/wpa_supplicant -I../../../../../../../../core/wifistack/wpa_cli -I../../../../../../../../core/system/include/coap -I../../../../../../../../version/genfiles/ -I../../../../../../../../core/system/include/ramlib -I../../../../../../../../core/segger_tools/Config -I../../../../../../../../core/segger_tools/OS -I../../../../../../../../core/segger_tools/SEGGER -I../../../../../../../../core/system/include/ota -I../../../../../../../../core/wifistack/romac/include -I../../../../../../../app_common/include/provisioning -I../../../../../../../app_common/include/util -include../../../../../../../../core/main/config/custom_config.h -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


