# MaixCam 视觉伺服集成指南

> 日期: 2026-07-25 | 芯片: MSPM0G3507 | 摄像头: Sipeed MaixCam (224×224)

---

## 1. 工作概览

本阶段完成了从零到可用的视觉闭环控制全部移植与开发：

| 步骤 | 内容 |
|---|---|
| ① SysConfig 配置 | `main.syscfg` 添加 UART3 实例 (PB2/PB3, 115200) |
| ② 硬件初始化 | SysConfig 生成 `ti_msp_dl_config.h/.c`，含 `SYSCFG_DL_UART_3_init()` |
| ③ MaixCam 通信 | `maixcam.c` 移植完成 (UART ISR → FreeRTOS 队列 → 帧解析) |
| ④ 视觉控制算法 | 新建 `vision_control.c/.h` (纯 C, 无 RTOS/硬件依赖) |
| ⑤ 服务适配层 | 新建 `vision_service.c/.h` (FreeRTOS 队列 + 50ms 周期) |
| ⑥ 应用层集成 | 修改 `app_tasks.c` (按键、任务、OLED、guard) |

---

## 2. SysConfig — UART3 配置

### `main.syscfg` 新增内容

```js
const UART2 = UART.addInstance();

UART2.$name                    = "UART_3";
UART2.enabledInterrupts        = [];    // 中断由 maixcam_init() 管理
UART2.targetBaudRate           = 115200;
UART2.peripheral.rxPin.$assign = "PB3";
UART2.peripheral.txPin.$assign = "PB2";
UART2.peripheral.$suggestSolution = "UART3";
```

### 生成的宏 (`ti_msp_dl_config.h` L162-177)

| 宏 | 值 | 用途 |
|---|---|---|
| `UART_3_INST` | `UART3` | 外设实例基址 |
| `UART_3_INST_INT_IRQN` | `UART3_INT_IRQn` | NVIC 中断号 |
| `UART_3_INST_IRQHandler` | `UART3_IRQHandler` | 中断服务函数名 |
| `GPIO_UART_3_TX_PIN` | `DL_GPIO_PIN_2` (PB2, PINCM15) | TX 引脚 |
| `GPIO_UART_3_RX_PIN` | `DL_GPIO_PIN_3` (PB3, PINCM16) | RX 引脚 |
| `UART_3_INST_FREQUENCY` | `80000000` | 总线时钟 80MHz |
| `UART_3_IBRD_80_MHZ_115200_BAUD` | `43` | 波特率整数分频 |
| `UART_3_FBRD_80_MHZ_115200_BAUD` | `26` | 波特率小数分频 |

### 初始化时序

```
SYSCFG_DL_init()
  └─ SYSCFG_DL_UART_3_init()   // 时钟/波特率/GPIO/UART使能, 不启中断

app_tasks_start()
  └─ maixcam_init()             // 创建队列 + 清FIFO + NVIC最低优先级 + 启RX中断
```

---

## 3. MaixCam 通信模块 (`Component/maixcam/`)

### 帧格式 (15 字节, 大端)

```
帧头 0xFE | x(u16) | y(u16) | w(u16) | h(u16) | score(f32) | count(u8) | 帧尾 0xFD
```

### 数据结构

```c
typedef struct {
    uint16_t x, y;        // 钢球中心像素坐标
    uint16_t w, h;        // 检测框宽高
    float    score;       // 置信度 0.0~1.0
    uint8_t  count;       // 帧内钢球数量
    uint32_t frame_id;    // 帧序号 (每收到一帧递增)
    bool     valid;       // 是否有有效数据
} maixcam_ball_t;
```

### API

| 函数 | 说明 |
|---|---|
| `maixcam_init()` | 创建接收队列, 清 FIFO, 设 NVIC 最低优先级, 启 RX 中断 |
| `maixcam_get_ball(&ball)` | 非阻塞拷贝快照, 返回 `ball.valid` |
| `maixcam_parse_task(arg)` | 解析任务入口 (需 `xTaskCreate` 创建) |

### 数据流

```
MaixCam UART TX → MSPM0 PB3 (UART3 RX)
  ISR: 读字节 → xQueueSendFromISR → FreeRTOS 队列
  MAIXCAM task (prio=2): xQueueReceive → 帧状态机解析
    → 大端转小端 → 更新 g_ball 全局结构体
  oled_task / vision_task: maixcam_get_ball() 读快照
```

---

## 4. 视觉控制算法 (`Component/control/vision_control.c`)

### 控制策略

```
error_x = ball.x - 112           // 画面中心 X (224/2)
steer   = pid_pos_t(error_x)     // 位置式 PID, 输出差速 RPM
base    = 25 或 10 RPM           // 根据球面积切换

left_rpm  = base - steer         // 球偏左→左轮慢→车左转
right_rpm = base + steer         // 球偏右→右轮慢→车右转
```

steer 被 clamp 到 ±base (防止车轮反转)。

### PID 参数 (整数 ×1000, 无浮点)

| 参数 | 默认值 | 含义 |
|---|---|---|
| Kp | 15 (0.015) | 每像素偏差的转向修正量 |
| Ki | 2 (0.002) | 缓慢消除稳态偏差 |
| Kd | 0 | 不用微分 |
| 输出限幅 | ±25 RPM | 最大差速 |

### 球面积 → 速度衰减

- `w × h > 2000` → base = 10 RPM (接近减速)
- `w × h ≤ 2000` → base = 25 RPM (正常前进)

### 状态机

```
IDLE ──start()──→ TRACKING ──丢帧(2帧)──→ SEARCHING ──超时(3s)──→ STOPPED
                                   ↑                        │
                                   └── 重新识别 ──────────────┘
```

| 状态 | 行为 |
|---|---|
| IDLE | 未启动, 不下发速度 |
| TRACKING | 计算差速, 下发 base±steer |
| SEARCHING | 原地旋转 (15RPM, 每800ms换向) |
| STOPPED | 搜索超时停车 |

### 可调参数速查表

所有参数在 `vision_control.c` 顶部 `#define`:

| 宏 | 默认值 | 说明 |
|---|---|---|
| `VISION_CENTER_X` | 112 | 画面水平中心 (224/2) |
| `VISION_BASE_RPM` | 25 | 追踪前进基准速度 |
| `VISION_SLOW_RPM` | 10 | 靠近减速速度 |
| `VISION_NEAR_AREA_THRESHOLD` | 2000 | 触发减速的面积阈值 |
| `VISION_PID_KP_MILLI` | 15 | 转向 P 增益 |
| `VISION_PID_KI_MILLI` | 2 | 转向 I 增益 |
| `VISION_STEER_MAX_RPM` | 25 | 最大差速 |
| `VISION_LOST_DEBOUNCE` | 2 | 连续丢帧阈值 |
| `VISION_SEARCH_TIMEOUT_MS` | 3000 | 搜索超时 |
| `VISION_SEARCH_RPM` | 15 | 搜索旋转速度 |
| `VISION_SEARCH_DIR_SWITCH_MS` | 800 | 旋转换向间隔 |

---

## 5. 服务适配层 (`Component/service/vision_service.c`)

完全参照 `distance_service` 的架构模式：

```
vision_service_init()
├── xQueueCreate(1, vision_cmd_t)    // 命令队列 (覆盖写)
├── xQueueCreate(1, vision_service_state_t)  // 状态队列 (覆盖写)
└── vision_control_init(&g_vision_ctrl)

vision_service_step(50ms)
├── xQueueReceive(command)           // 消费 START/STOP 命令
├── maixcam_get_ball(&ball)         // 读球数据快照
├── vision_control_update(&ball)    // 调用纯算法核心
├── speed_service_set_target(L,R)   // 直接下发速度目标
│   ↑ 绕过 app_tasks_set_wheel_speed_target 的 guard
└── xQueueOverwrite(state)          // 发布状态快照
```

### API

| 函数 | 说明 |
|---|---|
| `vision_service_init()` | 创建队列, 初始化核心 |
| `vision_service_start()` | 入队 START 命令 |
| `vision_service_stop()` | 入队 STOP 命令, 停车 |
| `vision_service_step(dt_ms)` | 50ms 周期入口 |
| `vision_service_get_state(&out)` | 读状态快照 |
| `vision_service_is_active_snapshot()` | 是否 active (TRACKING/SEARCHING) |

---

## 6. 应用层集成 (`Component/task/app_tasks.c`)

### 修改点

1. **新增 include**: `#include "service/vision_service.h"`

2. **Guard 扩展**: `app_tasks_set_wheel_speed_target()` 增加视觉 active 检查

3. **按键改造** (PB21):
   - **长按 >1s**: 切换视觉追踪 ON/OFF
   - **短按 <1s**: 仅在视觉关闭时循环速度档位

4. **新增 vision_task** (prio=2, 50ms):
   ```c
   xTaskCreate(vision_task, "VISION", 384, NULL, 2, NULL);
   ```

5. **OLED 双模式显示**: 视觉开→追踪详情; 视觉关→速度/档位

6. **初始化流程**:
   ```c
   speed_service_init();
   distance_service_init();
   maixcam_init();         // UART3 ISR 队列 + 使能
   vision_service_init();  // 视觉命令/状态队列 + 核心
   // → 各任务创建 → vTaskStartScheduler()
   ```

### 任务优先级表

| 任务 | 优先级 | 周期 | 角色 |
|---|---|---|---|
| SPD_LOOP | 3 | 10ms (硬件定时器) | 速度环 PID |
| MAIXCAM | 2 | 事件驱动 | UART 帧解析 |
| VISION | 2 | 50ms | 视觉控制循环 |
| DIST_KEY | 2 | 10ms | 按键检测 |
| LED | 1 | 500ms | 心跳 |
| OLED | 1 | 200ms | 显示 |
| DEBUG | 1 | 10ms | 串口输出 |

---

## 7. OLED 显示布局

### 视觉追踪模式 (视觉 ON)

```
┌──────────────────────────┐
│ [VIS] L:-30 R:-30        │  Row 0: 模式标签 + 车轮速度
│ X:112 err:+03 st: 2      │  Row 1: 球X + 偏差 + 差速
│ TRACKING A:1240          │  Row 2: 状态 + 球面积
│ Sc:0.95 N:1 RB:23        │  Row 3: 置信度 + 数量 + 基准速度
└──────────────────────────┘
```

### 手动速度模式 (视觉 OFF)

```
┌──────────────────────────┐
│ L:-30 R:-30              │  Row 0: 车轮速度 RPM
│ LT: -20 RT: -20          │  Row 1: 速度目标
│ Gear:30RPM [1]    B:112, │  Row 2: 档位 + 钢球预览
│ RUNNING                  │  Row 3: 状态
└──────────────────────────┘
```

---

## 8. 快速操作指南

| 操作 | 方法 |
|---|---|
| 开启视觉追踪 | 长按 PB21 >1秒 |
| 关闭视觉追踪 | 再次长按 PB21 >1秒 |
| 手动速度档位 | 短按 PB21 (仅视觉关闭时有效) |
| 紧急停车 | 长按 PB21 关闭视觉 (自动下发 0,0) |
| 观测追踪状态 | OLED 显示 `[VIS]` + `TRACKING/SEARCHING/STOPPED` |
| 观测球坐标 | OLED Row 1 显示 `X:nnn err:±nn` |
| 调 PID 参数 | 修改 `vision_control.c` 顶部 `VISION_PID_*_MILLI` |

---

## 9. 后续优化方向

1. **PID 标定**: 通过 DEBUG 串口输出 `error_x,steer,left_rpm,right_rpm` CSV 数据
2. **面积→速度插值**: 当前是二值切换, 可改为线性映射
3. **Y 轴利用**: 球 Y 坐标可用于判断球在画面上下位置, 辅助决策
4. **多球策略**: 当 `count > 1` 时, 选择最近的(面积最大的)球
5. **distance_task 恢复**: 距离控制和视觉控制可在 `app_tasks_set_wheel_speed_target` guard 中共存

---

## 10. 文件清单

| 文件 | 状态 | 说明 |
|---|---|---|
| `main.syscfg` | 修改 | 新增 UART3 实例 |
| `ti_msp_dl_config.h` | 自动生成 | UART_3_INST 等宏定义 |
| `ti_msp_dl_config.c` | 自动生成 | SYSCFG_DL_UART_3_init() |
| `Component/maixcam/maixcam.h` | 端口 | 球数据结构 + API 声明 |
| `Component/maixcam/maixcam.c` | 端口 | UART ISR + 帧解析 + 快照 |
| `Component/control/vision_control.h` | **新建** | 纯算法头文件 |
| `Component/control/vision_control.c` | **新建** | 纯算法实现 |
| `Component/service/vision_service.h` | **新建** | 服务层头文件 |
| `Component/service/vision_service.c` | **新建** | RTOS 适配层 |
| `Component/task/app_tasks.c` | **修改** | vision_task, 按键, guard, OLED, 初始化 |
