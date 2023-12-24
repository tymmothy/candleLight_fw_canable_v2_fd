project(STM32_HAL)

include(../../cmake/stmultitarget.cmake)

set(SOURCES
    cmsis/dsp/inc/arm_math.h
    cmsis/dsp/inc/arm_common_tables.h
    cmsis/dsp/inc/arm_const_structs.h
    
    cmsis/inc/cmsis_armcc.h
    cmsis/inc/cmsis_gcc.h

    cmsis/inc/core_cm0.h
    cmsis/inc/core_cm0plus.h
    cmsis/inc/core_cm3.h
    cmsis/inc/core_cm7.h
    cmsis/inc/core_cm4.h
    cmsis/inc/cmsis_device.h
    
    cmsis/inc/device/stm32g431xx.h
    cmsis/inc/device/system_stm32g4xx.h

    hal/src/stm32g4xx_hal.c
    hal/src/stm32g4xx_hal_fdcan.c
    hal/src/stm32g4xx_hal_cortex.c
    hal/src/stm32g4xx_hal_dma.c
    hal/src/stm32g4xx_hal_flash.c
    hal/src/stm32g4xx_hal_flash_ex.c
    hal/src/stm32g4xx_hal_gpio.c
    hal/src/stm32g4xx_hal_i2c.c
    hal/src/stm32g4xx_hal_i2c_ex.c
    hal/src/stm32g4xx_hal_pcd.c
    hal/src/stm32g4xx_hal_pcd_ex.c
    hal/src/stm32g4xx_hal_pwr.c
    hal/src/stm32g4xx_hal_pwr_ex.c
    hal/src/stm32g4xx_hal_rcc.c
    hal/src/stm32g4xx_hal_rcc_ex.c
    hal/src/stm32g4xx_hal_tim.c
    hal/src/stm32g4xx_hal_tim_ex.c

    hal/src/stm32g4xx_ll_usb.c

    cmsis/src/system_stm32g4xx.c

    hal/inc/stm32g4xx_hal_conf.h
    hal/inc/stm32g4xx_hal_tim.h
    hal/inc/stm32g4xx_hal_pcd.h
    hal/inc/stm32g4xx_hal_gpio.h
    hal/inc/stm32g4xx_hal_i2c_ex.h
    hal/inc/stm32g4xx_hal_flash_ex.h
    hal/inc/stm32g4xx_hal_dma_ex.h
    hal/inc/stm32g4xx_hal_i2c.h
    hal/inc/stm32g4xx_hal_pwr.h
    hal/inc/stm32g4xx_hal_pwr_ex.h
    hal/inc/stm32g4xx_hal_dma.h
    hal/inc/stm32g4xx_hal_gpio_ex.h
    hal/inc/Legacy/stm32_hal_legacy.h
    hal/inc/stm32g4xx_hal_rcc_ex.h
    hal/inc/stm32g4xx_hal_def.h
    hal/inc/stm32g4xx_hal.h
    hal/inc/stm32g4xx_hal_rcc.h
    hal/inc/stm32g4xx_hal_cortex.h
    hal/inc/stm32g4xx_hal_flash.h
    hal/inc/stm32g4xx_hal_tim_ex.h
    hal/inc/stm32g4xx_hal_pcd_ex.h
    hal/inc/stm32g4xx_hal_fdcan.h
    
    usb_device/Core/Inc/usbd_ioreq.h
    usb_device/Core/Inc/usbd_def.h
    usb_device/Core/Inc/usbd_ctlreq.h
    usb_device/Core/Inc/usbd_core.h
    usb_device/Core/Src/usbd_ioreq.c
    usb_device/Core/Src/usbd_ctlreq.c
    usb_device/Core/Src/usbd_core.c
    usb_device/config/usbd_conf.h
)

set(INCLUDE_DIRS
    cmsis/inc
    cmsis/dsp/inc
    cmsis/inc/device
    hal/inc 
    config/
    ../../targets/
    ../../inc/
    usb_device/Core/Inc
    usb_device/config

)

stmulti_add_hal_library("STM32G4" "cortex-m4" "${SOURCES}" "${INCLUDE_DIRS}")

    
