cmake_minimum_required(VERSION 3.22)

project(ST_Car LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

add_executable(ST_Car
    Core/Src/main.c
    Core/Src/gpio.c
    Core/Src/dma.c
    Core/Src/i2c.c
    Core/Src/usart.c
    Core/Src/tim.c
    Core/Src/stm32g4xx_it.c
    Core/Src/stm32g4xx_hal_msp.c
    Core/Src/system_stm32g4xx.c
    Core/Src/motor.c
    Core/Src/encoder.c
    Core/Src/pid.c
    Core/Src/sa100.c
    Core/Src/vision_uart.c
    Core/Src/line_sensor.c
    Core/Src/control_loops.c
    Core/Src/safety_monitor.c
    Core/Src/telemetry.c
    Core/Src/bench_debug.c
    Core/Src/app.c
    Core/Src/mpu.c
    Core/Src/yaw.c
    Drivers/BSP/STM32G4xx_Nucleo/stm32g4xx_nucleo.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_usart.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_usart_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c
    Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/gcc/startup_stm32g491xx.s
)

target_compile_definitions(ST_Car PRIVATE
    USE_NUCLEO_64
    USE_HAL_DRIVER
    STM32G491xx
)

target_include_directories(ST_Car PRIVATE
    Core/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    Drivers/BSP/STM32G4xx_Nucleo
    Drivers/CMSIS/Device/ST/STM32G4xx/Include
    Drivers/CMSIS/Include
)

set(MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

target_compile_options(ST_Car PRIVATE
    ${MCU_FLAGS}
    -ffunction-sections
    -fdata-sections
    -Wall
    -Wextra
    -Werror
    $<$<CONFIG:Debug>:-Og;-g3>
    $<$<CONFIG:Release>:-Os>
)

target_link_options(ST_Car PRIVATE
    ${MCU_FLAGS}
    -T${CMAKE_SOURCE_DIR}/STM32G491RETX_FLASH.ld
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,--gc-sections
    -Wl,-u,_printf_float
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/ST_Car.map
    -Wl,--print-memory-usage
)

target_link_libraries(ST_Car PRIVATE m)

set_target_properties(ST_Car PROPERTIES
    OUTPUT_NAME ST_Car
    SUFFIX .elf
)

add_custom_command(TARGET ST_Car POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:ST_Car> ${CMAKE_CURRENT_BINARY_DIR}/ST_Car.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:ST_Car> ${CMAKE_CURRENT_BINARY_DIR}/ST_Car.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:ST_Car>
)
