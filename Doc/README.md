# Smarthand 工程文件说明

STM32H750VBTx 固件工程，由 STM32CubeMX（`DEMO-LED.ioc`）生成骨架 + CMake/Ninja 构建。
当前实现：**ADC 采集模拟量、力传感器数字输入、串口打印、LED、4 路舵机 PWM 平滑控制**。

---

## 一、硬件引脚

| 引脚 | 复用 / 模式 | 用途 |
|---|---|---|
| PA0 | 模拟（SharedAnalog） | ADC1_INP16 —— **`peripheral.c` 里配的是这一路** |
| PA1 | 模拟（SharedAnalog） | ADC1_INP17 —— **`.ioc` 配的是这一路，也是 `config.md` 标注的 "ADC AO 采样"** ⚠️ 两者不一致 |
| PA2 | GPIO 输入（上拉） | 力传感器数字信号接收 |
| PB14 | USART1_TX（AF4） | 串口发送 |
| PB15 | USART1_RX（AF4） | 串口接收 |
| PA5 | TIM2_CH1（AF1） | 舵机 1 PWM |
| PB3 | TIM2_CH2（AF1） | 舵机 2 PWM |
| PB10 | TIM2_CH3（AF1） | 舵机 3 PWM |
| PB11 | TIM2_CH4（AF1） | 舵机 4 PWM |
| PC13 | GPIO 输出 | 板载 LED |
| PA13 / PA14 | SWDIO / SWCLK | 下载调试 |

- 串口：**115200 8N1，无流控**
- 舵机：TIM2 输出 **50 Hz**（20 ms 周期），脉宽 **0.5～2.5 ms** 对应 0～180°，1 个计数 = 1 µs

---

## 二、根目录文件

| 文件 | 内容 |
|---|---|
| `CMakeLists.txt` | 顶层构建脚本。设置 C11、默认 Debug、引入工具链文件、把 `cmake/stm32cubemx` 作为子工程链接起来；用户新增的源文件/头文件路径/宏/库都往这里加 |
| `CMakePresets.json` | CMake 预设：`Debug` / `RelWithDebInfo` / `Release` / `MinSizeRel`，统一用 Ninja，产物落到 `build/<预设名>/`，并导出 `compile_commands.json` 供 clangd 用 |
| `DEMO-LED.ioc` | STM32CubeMX 工程文件（时钟树、引脚复用、外设参数都在里面）。改硬件配置就打开它重新生成 |
| `.mxproject` | CubeMX 的生成记录，记录上次生成时用了哪些文件，别手动改 |
| `startup_stm32h750xx.s` | 启动汇编文件：中断向量表、复位入口、把 `.data` 从 Flash 搬到 RAM、清零 `.bss`，最后跳 `main` |
| `STM32H750VBTx_FLASH.ld` | 链接脚本：Flash / RAM 分布与各段（`.text` `.data` `.bss` `.heap` `.stack`）的摆放 |
| `config.md` | 手写的引脚功能速记（GBK 编码）：PA1 ADC AO 采样、PA2 力传感器、PB14/PB15 串口 |
| `.gitignore` | 忽略 `build/`、`mx.scratch`，放行 `.settings` |
| `.clangd` | clangd 索引配置（指定编译数据库路径等，配合 `compile_commands.json`） |
| `.settings/` | STM32CubeIDE / CubeMX 的工程设置 |
| `.vscode/` | VS Code 工作区设置：`cube-cmake` 构建、Ninja 生成器、`starm-clangd` 指向 CubeIDE 自带的 arm-none-eabi 工具链 |
| `cmake/gcc-arm-none-eabi.cmake` | 交叉编译工具链文件：指定 `arm-none-eabi-gcc/objcopy/size`、CPU 参数（Cortex-M7、hard-float）、`-mcpu/-mfpu` 等编译选项 |
| `cmake/stm32cubemx/CMakeLists.txt` | CubeMX 生成的源文件清单：把 `Core/Src` 下的工程源文件 + 用到的 HAL 驱动 `.c` 全部列进 `stm32cubemx` 这个 INTERFACE 库，并声明头文件包含路径 |

---

## 三、Core/Inc（头文件）

| 文件 | 内容 |
|---|---|
| `main.h` | 公共头文件：引入 HAL、声明 `Error_Handler()`、声明 `HAL_TIM_MspPostInit()` |
| `peripheral.h` | 只暴露一个 `peripheral_Init()`，把外设初始化打包起来 |
| `sensor.h` | ADC 相关：`hadc1`、`tim_log` 标志、`Sensor_Read()` / `Convert_TO_Val()` / `Sensor_Log()` |
| `mot_drv.h` | **舵机驱动接口**：脉宽标定宏（0.5/1.5/2.5 ms ↔ 0/90/180°）、速度加速度默认值、`Servo_Handle_t` 结构体、全部 API（使能、设 CCR/脉宽、立即定位、平滑转动、4 路同步、速度标定等） |
| `led.h` | LED 初始化 / 翻转接口 |
| `log.h` | 串口日志：`ST_Once_log()`，并 extern 了 `huart1`、`htim1` |
| `tim.h` | 预留（暂时为空，`tim.c` 里只有一个空的配置函数） |
| `stm32h7xx_it.h` | 中断服务函数声明 |
| `stm32h7xx_hal_conf.h` | HAL 库总配置：裁剪哪些外设模块、时钟值 `HSE_VALUE`、断言开关等 |
| `h7xx_hal_msp.c` 对应的声明在 `main.h` | MSP（底层引脚/时钟/NVIC）初始化的回调声明 |

---

## 四、Core/Src（源文件）

| 文件 | 内容 |
|---|---|
| `main.c` | 程序入口。`HAL_Init()` → `peripheral_Init()` → `Servo_Init()` 并只开 CH1；主循环里跑 `Servo_Update()`、`Servo_Debug_Task()`（单舵机 0/45/90/135/180° 往返扫描，每步串口打印角度、脉宽、CCR），`tim_log` 置位时调 `Sensor_Log()` 打印 ADC |
| `peripheral.c` | **外设初始化总集**：`peripheral_Init()` 依次做 ADC1、DMA、TIM1、GPIO、USART1、TIM2 的初始化，最后 `SystemClock_Config()`。里面还包含：ADC1 连续转换 16 位（代码配的是通道 16 = PA0）、TIM1 作 **5 Hz** 的采样节拍（APB2 定时器时钟 60 MHz ÷ 1200 ÷ 10000）、USART1 115200、TIM2 四路 PWM、`__io_putchar()` 把 `printf` 重定向到串口、`Error_Handler()` 死循环 |
| `sensor.c` | ADC 采样：`Sensor_Read()` 读 ADC 数据寄存器，`Convert_TO_Val()` 按 3.3 V 基准换算电压，`Sensor_Log()` 串口打印原始值 + 电压；`HAL_TIM_PeriodElapsedCallback()` 在定时器更新中断里采样并置 `tim_log = 1`，由主循环负责打印 |
| `mot_drv.c` | **舵机驱动实现**（本工程最完整的一块）：按实际 TIM2 时钟自动精调分频到 1 µs 分辨率、50 Hz PWM；角度↔脉宽↔CCR 换算；梯形速度规划的平滑转动（速度/加速度可配、到位判定）；4 路同步转动；直接设 CCR/脉宽；限幅保护 |
| `led.c` | PC13 LED：`LED_Init()` 推挽输出并置低，`LED_Tringle()` 翻转 + 延时 300 ms |
| `log.c` | `ST_Once_log()`：用 `HAL_UART_Transmit` 阻塞式发一串字符，用于一次性打印提示 |
| `tim.c` / `tim.h` | 预留的空实现，留给后续自定义定时器功能 |
| `stm32h7xx_it.c` | 中断服务函数：NMI/HardFault/MemManage/BusFault/UsageFault/SVC/DebugMon/PendSV/SysTick，以及本工程用到的 `DMA1_Stream0_IRQHandler`、`TIM1_UP_IRQHandler` |
| `stm32h7xx_hal_msp.c` | HAL 底层初始化回调（MSP）：各外设的 GPIO 复用、时钟使能、NVIC 优先级、DMA 关联；**TIM2_CH1~CH4 的 PWM 引脚就是在这里用 `HAL_TIM_MspPostInit()` 配置成 AF1** |
| `system_stm32h7xx.c` | CMSIS 系统初始化：`SystemInit()`、向量表地址、AHB/APB 预分频的初始值（本工程时钟由 `SystemClock_Config()` 再配置） |
| `syscalls.c` | newlib 的系统调用桩：`_write`（printf 出口）、`_read`、`_sbrk`、`_close` 等，让裸机也能用标准库 |
| `sysmem.c` | 堆内存管理：`_sbrk()` 用 `_estack` 与 heap 边界实现 malloc 的动态内存增长 |

---

## 五、其它目录

| 目录 | 内容 |
|---|---|
| `Drivers/STM32H7xx_HAL_Driver/` | ST 官方 HAL 库源码 + 头文件（只编译实际用到的 `.c`，清单见 `cmake/stm32cubemx/CMakeLists.txt`） |
| `Drivers/CMSIS/` | ARM CMSIS：内核头文件、设备头文件，以及一整套 DSP 库（`.a` 静态库和示例，本工程未使用，占体积大头） |
| `App/` | 预留给应用层代码（当前为空） |
| `Doc/` | 文档目录，本文件就在这里 |
| `build/` | 构建输出（`.elf` `.bin` `.map`、对象文件、Ninja 文件），已被 `.gitignore` 忽略 |

---

## 六、怎么编译

方式一，命令行（需要 `arm-none-eabi-gcc` 与 `ninja` 在 PATH 里）：

```bash
cmake --preset Debug
cmake --build build/Debug
```

方式二，VS Code + STM32Cube 扩展：工作区已经配好 `cube-cmake`、Ninja、`starm-clangd`，直接点 Build 即可。
产物在 `build/Debug/`，用 ST-Link 烧 `.elf`（或转出的 `.bin` / `.hex`）。

---

## 七、几处需要核对的地方

1. **ADC 通道**：`config.md` 说 "PA1 ADC AO 采样"，而 `peripheral.c` 里 ADC1 配的是 `ADC_CHANNEL_16`（对应 PA0）；CubeMX 里 PA0、PA1 都设成了 SharedAnalog。两者需要确认到底接在哪一路。
2. **PA1 的最终模式**：`stm32h7xx_hal_msp.c` 先把 PA0/PA1 配成模拟输入，但 `peripheral.c` 的 `MX_GPIO_Init()` 又把 **PA1 改成推挽输出**（`GPIO_InitStruct` 被覆盖，PA2 的输入配置其实没生效）。如果 PA1 要当模拟输入用，这一处得改。
3. **PA2 上拉输入**：`MX_GPIO_Init()` 里给 PA2 赋了输入模式，但紧接着结构体被改写、`HAL_GPIO_Init` 只对 PA1 生效，PA2 实际仍是 CubeMX 生成的默认状态——力传感器的读法需要再确认。
4. `tim_log` 在 `sensor.c` 里定义、`main.c` 里使用，属于跨文件共享的全局标志，后续改动时留意别在多处写它。
