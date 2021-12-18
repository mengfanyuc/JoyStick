CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_PROCESSOR cortex-m3)
SET(CMAKE_VERBOSE_MAKEFILE ON)

PROJECT(demo C CXX ASM)

IF(CMAKE_BUILD_TYPE MATCHES Debug)
    MESSAGE(STATUS "Build type: Debug")
    SET(DBG_FLAGS "-g3 -gdwarf-2 -O0")
ELSEIF(CMAKE_BUILD_TYPE MATCHES Release)
    MESSAGE(STATUS "Build type: Release")
    SET(DBG_FLAGS "-Os")
ENDIF()

SET(CROSS_COMPILE_PREFIX arm-none-eabi-)

SET(MCU_FLAGS "-mcpu=cortex-m3 -mthumb")
SET(SECTIONS_FLAGS "-ffunction-sections -fdata-sections")
SET(LINK_MAP_FLAGS "-Map=${PROJECT_NAME}.map")
SET(LINK_FLAGS "${MCU_FLAGS} ${SECTIONS_FLAGS} -Wl,--gc-sections,${LINK_MAP_FLAGS},-cref")
SET(LINK_SCRIPTS_FLAGS "-T ${CMAKE_SOURCE_DIR}/linkscripts/link.lds")

SET(CMAKE_C_COMPILER ${CROSS_COMPILE_PREFIX}gcc)
SET(CMAKE_ASM_COMPILER ${CROSS_COMPILE_PREFIX}gcc)
SET(CMAKE_CXX_COMPILER ${CROSS_COMPILE_PREFIX}g++)
SET(CMAKE_OBJCOPY ${CROSS_COMPILE_PREFIX}objcopy)
SET(CMAKE_SIZE ${CROSS_COMPILE_PREFIX}size)
SET(CMAKE_C_COMPILER_WORKS TRUE)
SET(CMAKE_CXX_COMPILER_WORKS TRUE)

SET(CMAKE_C_FLAGS " ${MCU_FLAGS} ${DBG_FLAGS} ${SECTIONS_FLAGS} -Dgcc")
SET(CMAKE_CXX_FLAGS " ${MCU_FLAGS} ${DBG_FLAGS} ${SECTIONS_FLAGS} -Dgcc")
SET(CMAKE_ASM_FLAGS " -c ${MCU_FLAGS} ${DBG_FLAGS} ${SECTIONS_FLAGS} -x assembler-with-cpp -Wa,-mimplicit-it=thumb")
SET(CMAKE_EXE_LINKER_FLAGS " ${LINK_FLAGS},-u,Reset_Handler ${LINK_SCRIPTS_FLAGS}")

SET(CMAKE_CXX_STANDARD 14)

INCLUDE_DIRECTORIES(
    ${CMAKE_SOURCE_DIR}/rt-thread
    ${CMAKE_SOURCE_DIR}/drivers/include
    ${CMAKE_SOURCE_DIR}/drivers/include/config
    ${CMAKE_SOURCE_DIR}/libraries/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/libraries/CMSIS/Device/ST/STM32F1xx/Include
    ${CMAKE_SOURCE_DIR}/libraries/STM32F1xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/rt-thread/components/drivers/include
    ${CMAKE_SOURCE_DIR}/rt-thread/components/finsh
    ${CMAKE_SOURCE_DIR}/rt-thread/components/libc/compilers/common
    ${CMAKE_SOURCE_DIR}/rt-thread/components/libc/compilers/newlib
    ${CMAKE_SOURCE_DIR}/rt-thread/include
    ${CMAKE_SOURCE_DIR}/rt-thread/libcpu/arm/common
    ${CMAKE_SOURCE_DIR}/rt-thread/libcpu/arm/cortex-m3
)

ADD_DEFINITIONS(
	-DSTM32F103xB
	-DUSE_HAL_DRIVER
)

SET(PROJECT_SOURCES
    ${CMAKE_SOURCE_DIR}/applications/main.c
    ${CMAKE_SOURCE_DIR}/drivers/board.c
    ${CMAKE_SOURCE_DIR}/drivers/drv_common.c
    ${CMAKE_SOURCE_DIR}/drivers/drv_gpio.c
    ${CMAKE_SOURCE_DIR}/drivers/drv_usart.c
    ${CMAKE_SOURCE_DIR}/drivers/stm32f1xx_hal_msp.c
    ${CMAKE_SOURCE_DIR}/libraries/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.c
    ${CMAKE_SOURCE_DIR}/libraries/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s
    ${CMAKE_SOURCE_DIR}/libraries/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
	${CMAKE_SOURCE_DIR}/libraries/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
    ${CMAKE_SOURCE_DIR}/libraries/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
    ${CMAKE_SOURCE_DIR}/libraries/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
    ${CMAKE_SOURCE_DIR}/libraries/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
    ${CMAKE_SOURCE_DIR}/libraries/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c
    ${CMAKE_SOURCE_DIR}/libraries/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_usart.c
    ${CMAKE_SOURCE_DIR}/libraries/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
    ${CMAKE_SOURCE_DIR}/libraries/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
    ${CMAKE_SOURCE_DIR}/libraries/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
    ${CMAKE_SOURCE_DIR}/rt-thread/libcpu/arm/common/div0.c
	${CMAKE_SOURCE_DIR}/rt-thread/libcpu/arm/common/showmem.c
	${CMAKE_SOURCE_DIR}/rt-thread/libcpu/arm/common/backtrace.c
	${CMAKE_SOURCE_DIR}/rt-thread/libcpu/arm/cortex-m3/cpuport.c
	${CMAKE_SOURCE_DIR}/rt-thread/libcpu/arm/cortex-m3/context_gcc.S
	${CMAKE_SOURCE_DIR}/rt-thread/components/drivers/misc/pin.c
	${CMAKE_SOURCE_DIR}/rt-thread/components/drivers/serial/serial.c
    ${CMAKE_SOURCE_DIR}/rt-thread/components/finsh/shell.c
	${CMAKE_SOURCE_DIR}/rt-thread/components/finsh/msh.c
	${CMAKE_SOURCE_DIR}/rt-thread/components/finsh/cmd.c
    ${CMAKE_SOURCE_DIR}/rt-thread/components/drivers/src/dataqueue.c
	${CMAKE_SOURCE_DIR}/rt-thread/components/drivers/src/ringbuffer.c
	${CMAKE_SOURCE_DIR}/rt-thread/components/drivers/src/completion.c
	${CMAKE_SOURCE_DIR}/rt-thread/components/drivers/src/pipe.c
	${CMAKE_SOURCE_DIR}/rt-thread/components/drivers/src/waitqueue.c
	${CMAKE_SOURCE_DIR}/rt-thread/components/drivers/src/workqueue.c
	${CMAKE_SOURCE_DIR}/rt-thread/components/drivers/src/ringblk_buf.c
    ${CMAKE_SOURCE_DIR}/rt-thread/src/ipc.c
	${CMAKE_SOURCE_DIR}/rt-thread/src/scheduler.c
	${CMAKE_SOURCE_DIR}/rt-thread/src/memheap.c
	${CMAKE_SOURCE_DIR}/rt-thread/src/thread.c
	${CMAKE_SOURCE_DIR}/rt-thread/src/irq.c
	${CMAKE_SOURCE_DIR}/rt-thread/src/mem.c
	${CMAKE_SOURCE_DIR}/rt-thread/src/timer.c
	${CMAKE_SOURCE_DIR}/rt-thread/src/clock.c
	${CMAKE_SOURCE_DIR}/rt-thread/src/kservice.c
	${CMAKE_SOURCE_DIR}/rt-thread/src/device.c
	${CMAKE_SOURCE_DIR}/rt-thread/src/object.c
	${CMAKE_SOURCE_DIR}/rt-thread/src/idle.c
	${CMAKE_SOURCE_DIR}/rt-thread/src/components.c
	${CMAKE_SOURCE_DIR}/rt-thread/components/libc/compilers/newlib/minilib.c
)

LINK_DIRECTORIES(
)

LINK_LIBRARIES(
)

ADD_EXECUTABLE(${CMAKE_PROJECT_NAME}.elf ${PROJECT_SOURCES})
ADD_CUSTOM_COMMAND(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD 
COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.bin COMMAND ${CMAKE_SIZE} ${CMAKE_PROJECT_NAME}.elf)
