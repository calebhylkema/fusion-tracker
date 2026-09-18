# firmware/ — STM32F446RE (NUCLEO-F446RE)

The STM32CubeIDE project lives here. It is **generated in Phase 1** — create a new CubeIDE project
targeting the **NUCLEO-F446RE** board and let CubeMX scaffold it into this folder.

Toolchain: **STM32CubeIDE + HAL/LL**, timer-driven superloop (no RTOS), 50 Hz fusion loop.

Planned module layout (see the spec for detail):
```
Core/      main.c, sysclk_config.c, it_handlers.c
Drivers/   hlk_ld2450, bno085, can_output, uart_pi_link
Fusion/    ekf, imm, measurement_models, frame_transform, data_association, track_manager
App/       fusion_task, system_monitor
```

Peripheral assignment (from the spec — resolves the ST-LINK VCP conflict):
- **USART1** PB6/PB7 — radar (DMA + idle IRQ), 256000 baud
- **USART2** PA2/PA3 — ST-LINK VCP printf debug
- **USART6** PC6/PC7 — Pi link, 115200 baud
- **I2C1** PB8/PB9 (+ PB5 INT, PB4 RST) — BNO085
- **CAN1** PA11/PA12 — SN65HVD230, 500 kbps
- **TIM6** — 50 Hz loop tick

Build/flash: open in CubeIDE → Build → Run (ST-LINK). arm-gcc is bundled with CubeIDE.
