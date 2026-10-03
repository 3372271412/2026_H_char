/**
 * @file    app_tasks.c
 * @brief   H题六任务菜单、按键交互与控制任务调度
 */

#include "FreeRTOS.h"
#include "task.h"
#include "ti_msp_dl_config.h"
#include "task/app_tasks.h"
#include "led/led.h"
#include "led/key.h"
#include "oled/oled.h"
#include "encoder/encoder.h"
#include "gray_sensor/gray_sensor.h"
#include "tb6612/tb6612.h"
#include "service/speed_service.h"
#include "service/line_tracking_service.h"
#include "maixcam/maixcam.h"
#include "control/control2.h"
#include <stdbool.h>
#include <stdint.h>

#define APP_TASK_COUNT 6U

typedef enum {
    UI_MENU = 0,
    UI_TASK_PAGE
} ui_mode_t;

typedef enum {
    PAGE_WAIT = 0,
    PAGE_RUNNING,
    PAGE_DONE,
    PAGE_FAULT
} page_phase_t;

static TaskHandle_t g_control_task_handle = NULL;
static volatile ui_mode_t g_ui_mode = UI_MENU;
static volatile uint8_t g_selected_task = 1U;
static volatile page_phase_t g_page_phase = PAGE_WAIT;
static volatile bool g_stream_on = false;
static volatile bool g_stream_close_pending = false;
static TickType_t g_stream_close_tick = 0;
static TickType_t g_timer_start_tick = 0;
static volatile uint32_t g_timer_final_s = 0;

static uint32_t get_elapsed_s(void)
{
    if (g_timer_final_s > 0) return g_timer_final_s;
    if (g_timer_start_tick == 0) return 0;
    return (uint32_t)((xTaskGetTickCount() - g_timer_start_tick)
                      * portTICK_PERIOD_MS / 1000U);
}

static void led_task(void *arg)
{
    (void)arg;
    for (;;) {
        led_toggle();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void stream_start(void)
{
    g_stream_close_pending = false;
    maixcam_send_image_transmission_start();
    g_stream_on = true;
}

static uint32_t selected_stream_close_delay_ms(void)
{
    switch (g_selected_task) {
    case 1U: return g_task1_params.stream_close_delay_ms;
    case 2U: return g_task2_params.stream_close_delay_ms;
    case 3U: return g_task3_params.stream_close_delay_ms;
    case 4U: return g_task4_params.stream_close_delay_ms;
    case 5U: return g_task5_params.stream_close_delay_ms;
    case 6U: return g_task6_params.lap.stream_close_delay_ms;
    default: return 1000U;
    }
}

static void stream_close_later(void)
{
    if (!g_stream_on || g_stream_close_pending) {
        return;
    }
    g_stream_close_tick = xTaskGetTickCount() +
        pdMS_TO_TICKS(selected_stream_close_delay_ms());
    g_stream_close_pending = true;
}

static void stream_service(void)
{
    if (g_stream_close_pending &&
        (int32_t)(xTaskGetTickCount() - g_stream_close_tick) >= 0) {
        maixcam_send_image_transmission_end();
        g_stream_on = false;
        g_stream_close_pending = false;
    }
}

static bool page_can_restart(void)
{
    return !g_stream_on && !g_stream_close_pending &&
           (g_page_phase == PAGE_DONE || g_page_phase == PAGE_FAULT);
}

static void begin_task2(void)
{
    stream_start();
    if (task2_lap_start()) {
        g_page_phase = PAGE_RUNNING;
        g_timer_start_tick = xTaskGetTickCount();
        g_timer_final_s = 0;
    } else {
        g_page_phase = PAGE_FAULT;
        stream_close_later();
    }
}

static void begin_task3(void)
{
    stream_start();
    task3_start();
    g_page_phase = PAGE_RUNNING;
    g_timer_start_tick = xTaskGetTickCount();
    g_timer_final_s = 0;
}

static void begin_task4(void)
{
    stream_start();
    if (task4_prepare()) {
        g_page_phase = PAGE_RUNNING;
    } else {
        g_page_phase = PAGE_FAULT;
        stream_close_later();
    }
}

static void begin_task5(void)
{
    stream_start();
    if (task5_start()) {
        g_page_phase = PAGE_RUNNING;
        g_timer_start_tick = xTaskGetTickCount();
        g_timer_final_s = 0;
    } else {
        g_page_phase = PAGE_FAULT;
        stream_close_later();
    }
}

static bool begin_task6(void)
{
    /* UART球坐标持续更新；先锁定有效目标，成功后才开启图传和计时。 */
    if (!task6_start_auto()) {
        return false;
    }
    stream_start();
    g_page_phase = PAGE_RUNNING;
    g_timer_start_tick = xTaskGetTickCount();
    g_timer_final_s = 0;
    return true;
}

static void enter_selected_task(void)
{
    g_ui_mode = UI_TASK_PAGE;
    g_page_phase = PAGE_WAIT;
    g_stream_on = false;
    g_stream_close_pending = false;
    g_timer_start_tick = 0;
    g_timer_final_s = 0;

    /* Task3/6进入页面只调平管道，等待PB21，不启动位置闭环。 */
    if (g_selected_task == 3U || g_selected_task == 6U) {
        speed_service_emergency_stop();
        ball_control_prepare_level_for_menu(g_selected_task);
    }

    /* 任务4按题意进入页面后立即开始将球控制到中心。 */
    if (g_selected_task == 4U) {
        begin_task4();
    }
}

static void handle_task_button(void)
{
    switch (g_selected_task) {
    case 1U:
        if (g_page_phase == PAGE_RUNNING) {
            g_page_phase = PAGE_DONE;
            stream_close_later();
            g_timer_final_s = get_elapsed_s();
            g_timer_start_tick = 0;
        } else if ((g_page_phase == PAGE_WAIT || page_can_restart()) &&
                   !g_stream_close_pending) {
            stream_start();
            g_page_phase = PAGE_RUNNING;
            g_timer_start_tick = xTaskGetTickCount();
            g_timer_final_s = 0;
        }
        break;

    case 2U:
        if (g_page_phase == PAGE_WAIT || page_can_restart()) {
            begin_task2();
        }
        break;

    case 3U:
        if (g_page_phase == PAGE_WAIT || page_can_restart()) {
            begin_task3();
        }
        break;

    case 4U: {
        task4_status_t status = {0};
        (void)task4_get_status(&status);
        if (status.state == TASK4_READY) {
            /* READY 与按键边沿之间若球又偏出容差，保持居中等待，不报错。 */
            if (task4_start_motion()) {
                g_timer_start_tick = xTaskGetTickCount();
                g_timer_final_s = 0;
            }
        } else if (page_can_restart()) {
            begin_task4();
        }
        break;
    }

    case 5U:
        if (g_page_phase == PAGE_WAIT || page_can_restart()) {
            begin_task5();
        }
        break;

    case 6U:
        if ((g_page_phase == PAGE_WAIT || page_can_restart()) &&
            !g_stream_close_pending) {
            (void)begin_task6();
        }
        break;

    default:
        break;
    }
}

static void update_page_completion(void)
{
    if (g_ui_mode != UI_TASK_PAGE || g_page_phase != PAGE_RUNNING) {
        return;
    }

    if (g_selected_task == 2U) {
        task2_lap_status_t status = {0};
        if (task2_lap_get_status(&status)) {
            if (status.state == TASK2_LAP_COMPLETE) {
                g_page_phase = PAGE_DONE;
                stream_close_later();
                g_timer_final_s = get_elapsed_s();
                g_timer_start_tick = 0;
            } else if (status.state == TASK2_LAP_FAULT) {
                g_page_phase = PAGE_FAULT;
                stream_close_later();
                g_timer_final_s = get_elapsed_s();
                g_timer_start_tick = 0;
            }
        }
    } else if (g_selected_task == 3U) {
        task3_status_t status = {0};
        if (task3_get_status(&status) &&
            status.state == TASK3_COMPLETE) {
            g_page_phase = PAGE_DONE;
            stream_close_later();
            g_timer_final_s = get_elapsed_s();
            g_timer_start_tick = 0;
        }
    } else if (g_selected_task == 4U) {
        task4_status_t status = {0};
        if (task4_get_status(&status)) {
            if (status.state == TASK4_COMPLETE) {
                g_page_phase = PAGE_DONE;
                stream_close_later();
                g_timer_final_s = get_elapsed_s();
                g_timer_start_tick = 0;
            } else if (status.state == TASK4_FAULT) {
                g_page_phase = PAGE_FAULT;
                stream_close_later();
                g_timer_final_s = get_elapsed_s();
                g_timer_start_tick = 0;
            }
        }
    } else if (g_selected_task == 5U || g_selected_task == 6U) {
        task5_status_t status = {0};
        if (task5_get_status(&status)) {
            if (status.state == TASK5_COMPLETE) {
                g_page_phase = PAGE_DONE;
                stream_close_later();
                g_timer_final_s = get_elapsed_s();
                g_timer_start_tick = 0;
            } else if (status.state == TASK5_FAULT) {
                g_page_phase = PAGE_FAULT;
                stream_close_later();
                g_timer_final_s = get_elapsed_s();
                g_timer_start_tick = 0;
            }
        }
    }
}

static void key_task(void *arg)
{
    bool user_was = false;
    bool select_was = false;
    (void)arg;

    for (;;) {
        bool user_now = key_read_user();
        bool select_now = key_read_task_select();
        bool user_pressed = user_now && !user_was;
        bool select_pressed = select_now && !select_was;
        user_was = user_now;
        select_was = select_now;

        if (g_ui_mode == UI_MENU) {
            if (select_pressed) {
                g_selected_task = (uint8_t)(g_selected_task %
                                             APP_TASK_COUNT + 1U);
            }
            if (user_pressed) {
                enter_selected_task();
            }
        } else if (user_pressed) {
            handle_task_button();
        }

        update_page_completion();
        stream_service();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void draw_menu(void)
{
    uint8_t first = (g_selected_task <= 4U) ? 1U : 3U;
    for (uint8_t row = 0U; row < 4U; row++) {
        uint8_t task_number = (uint8_t)(first + row);
        OLED_vsprint(0, row * 16U, 16, "%c Task %u",
                     task_number == g_selected_task ? '>' : ' ',
                     (unsigned int)task_number);
    }
}

static void draw_task1(void)
{
    uint32_t secs = get_elapsed_s();
    OLED_vsprint(0, 0, 16, "TASK 1 IMAGE  %lus", (unsigned long)secs);
    OLED_vsprint(0, 16, 16, "STREAM %s", g_stream_on ? "ON" : "OFF");
    if (g_stream_close_pending) {
        OLED_vsprint(0, 32, 16, "CLOSE IN 1S");
    } else if (g_page_phase == PAGE_RUNNING) {
        OLED_vsprint(0, 32, 16, "PB21 STOP");
    } else {
        OLED_vsprint(0, 32, 16, "PB21 START");
    }
    OLED_vsprint(0, 48, 16, "%s",
                 g_page_phase == PAGE_DONE ? "DONE" : "READY");
}

static void draw_task2(void)
{
    task2_lap_status_t status = {0};
    (void)task2_lap_get_status(&status);
    uint32_t secs = get_elapsed_s();
    OLED_vsprint(0, 0, 16, "TASK 2 LAP  %lus", (unsigned long)secs);
    OLED_vsprint(0, 16, 16, "%s %lums",
                 task2_lap_state_name(status.state),
                 (unsigned long)status.total_time_ms);
    OLED_vsprint(0, 32, 16, "ARM:%c BLACK:%u",
                 status.checkpoint_armed ? 'Y' : 'N',
                 (unsigned int)status.strong_channel);
    OLED_vsprint(0, 48, 16, "%s",
                 g_page_phase == PAGE_WAIT ? "PB21 RUN" :
                 (g_stream_close_pending ? "CLOSE IN 1S" :
                  (g_page_phase == PAGE_DONE ? "DONE PB21" :
                   (g_page_phase == PAGE_FAULT ? "FAULT PB21" : "RUNNING"))));
}

static void draw_task3(void)
{
    task3_status_t task = {0};
    ball_control_status_t ball = {0};
    (void)task3_get_status(&task);
    (void)ball_control_get_status(&ball);
    uint32_t secs = get_elapsed_s();
    OLED_vsprint(0, 0, 16, "TASK 3 BALL  %lus", (unsigned long)secs);
    OLED_vsprint(0, 16, 16, "%s %lums",
                 g_page_phase == PAGE_WAIT ? "LEVEL" :
                 task3_state_name(task.state),
                 (unsigned long)task.elapsed_ms);
    OLED_vsprint(0, 32, 16, "X:%ld E:%ld",
                 (long)ball.ball_x, (long)task.error);
    OLED_vsprint(0, 48, 16, "%s",
                 g_page_phase == PAGE_WAIT ? "PB21 RUN" :
                 (g_stream_close_pending ? "CLOSE IN 1S" :
                  (g_page_phase == PAGE_DONE ? "DONE PB21" : "RUNNING")));
}

static void draw_task4(void)
{
    task4_status_t status = {0};
    (void)task4_get_status(&status);
    uint32_t secs = get_elapsed_s();
    OLED_vsprint(0, 0, 16, "TASK 4 A-B  %lus", (unsigned long)secs);
    OLED_vsprint(0, 16, 16, "%s %lums",
                 task4_state_name(status.state),
                 (unsigned long)status.elapsed_ms);
    OLED_vsprint(0, 32, 16, "B:%ld E:%ld",
                 (long)status.ball_x, (long)status.ball_error);
    if (status.state == TASK4_READY) {
        OLED_vsprint(0, 48, 16, "PB21 RUN");
    } else if (g_stream_close_pending) {
        OLED_vsprint(0, 48, 16, "CLOSE IN 1S");
    } else if (g_page_phase == PAGE_DONE) {
        OLED_vsprint(0, 48, 16, "DONE PB21");
    } else if (g_page_phase == PAGE_FAULT) {
        OLED_vsprint(0, 48, 16, "FAULT PB21");
    } else {
        OLED_vsprint(0, 48, 16, "CENTER/RUN");
    }
}

static void draw_task5(void)
{
    task5_status_t status = {0};
    (void)task5_get_status(&status);
    uint32_t secs = get_elapsed_s();
    OLED_vsprint(0, 0, 16, "TASK 5 BALL %lus", (unsigned long)secs);
    OLED_vsprint(0, 16, 16, "%s X:%ld",
                 task5_state_name(status.state),
                 (long)status.ball_x);
    OLED_vsprint(0, 32, 16, "A:%c B:%u E:%ld",
                 status.checkpoint_armed ? 'Y' : 'N',
                 (unsigned int)status.strong_channel,
                 (long)status.ball_error);
    OLED_vsprint(0, 48, 16, "%s",
                 g_page_phase == PAGE_WAIT ? "PB21 RUN" :
                 (g_stream_close_pending ? "CLOSE IN 1S" :
                  (g_page_phase == PAGE_DONE ? "DONE PB21" :
                   (g_page_phase == PAGE_FAULT ? "FAULT PB21" :
                    "CENTER/RUN"))));
}

static void draw_task6(void)
{
    task5_status_t status = {0};
    maixcam_ball_t ball = {0};
    (void)task5_get_status(&status);
    uint32_t now_ms = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
    bool fresh_ball = maixcam_get_ball(&ball) && ball.valid &&
        now_ms - ball.tick_ms <= g_task6_params.ball_fresh_max_ms;
    int32_t shown_x = g_page_phase == PAGE_WAIT ?
                      (fresh_ball ? (int32_t)ball.x : -1) :
                      status.ball_x;
    uint32_t secs = get_elapsed_s();

    OLED_vsprint(0, 0, 16, "TASK 6 SPEC %lus", (unsigned long)secs);
    if (g_page_phase == PAGE_WAIT) {
        OLED_vsprint(0, 16, 16, "LEVEL X:%ld", (long)shown_x);
        OLED_vsprint(0, 32, 16, "PLACE BALL");
        OLED_vsprint(0, 48, 16, "%s",
                     fresh_ball ? "PB21 SET&RUN" : "WAIT CAMERA");
        return;
    }

    OLED_vsprint(0, 16, 16, "%s X:%ld T:%ld",
                 task5_state_name(status.state),
                 (long)shown_x, (long)status.ball_target_x);
    OLED_vsprint(0, 32, 16, "A:%c B:%u E:%ld",
                 status.checkpoint_armed ? 'Y' : 'N',
                 (unsigned int)status.strong_channel,
                 (long)status.ball_error);
    OLED_vsprint(0, 48, 16, "%s",
                 g_stream_close_pending ? "CLOSE IN 1S" :
                 (g_page_phase == PAGE_DONE ? "DONE PB21" :
                  (g_page_phase == PAGE_FAULT ? "FAULT PB21" :
                   "RUNNING")));
}

static void oled_task(void *arg)
{
    (void)arg;
    OLED_Init();
    OLED_Clear();
    OLED_Refresh();

    for (;;) {
        OLED_Clear();
        if (g_ui_mode == UI_MENU) {
            draw_menu();
        } else if (g_selected_task == 1U) {
            draw_task1();
        } else if (g_selected_task == 2U) {
            draw_task2();
        } else if (g_selected_task == 3U) {
            draw_task3();
        } else if (g_selected_task == 4U) {
            draw_task4();
        } else if (g_selected_task == 5U) {
            draw_task5();
        } else {
            draw_task6();
        }
        OLED_Refresh();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void handle_control_overrun(void)
{
    if (task5_is_active()) {
        task5_emergency_stop();
    } else if (task4_is_active()) {
        task4_emergency_stop();
    } else if (task3_is_active()) {
        task3_stop();
        ball_control_emergency_stop();
    } else if (task2_lap_is_active()) {
        task2_lap_stop();
        /* task2_lap_stop投递ABORT；立即消费，避免重跑时service仍显示active。 */
        line_tracking_service_step_10ms();
    } else if (ball_control_is_active()) {
        ball_control_emergency_stop();
    }
    speed_service_emergency_stop();

    if (g_ui_mode == UI_TASK_PAGE && g_page_phase == PAGE_RUNNING) {
        g_page_phase = PAGE_FAULT;
        stream_close_later();
        g_timer_final_s = get_elapsed_s();
        g_timer_start_tick = 0;
    }
}

static void control_task(void *arg)
{
    (void)arg;
    NVIC_SetPriority(TIMER_0_INST_INT_IRQN, 3);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);

    for (;;) {
        uint32_t notify_count = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (notify_count != 1U) {
            /* 过时的10ms控制周期不补跑；立即停车，避免长控制空窗。 */
            handle_control_overrun();
            continue;
        }
        if (task5_is_active()) {
            (void)task5_step(10U);
        } else if (task4_is_active()) {
            (void)task4_step(10U);
        } else if (task3_is_active()) {
            (void)task3_step(10U);
        } else if (task2_lap_is_active()) {
            (void)task2_lap_step(10U);
        } else if (ball_control_is_active()) {
            (void)ball_control_step(10U);
        }
    }
}

void app_tasks_start(void)
{
    BaseType_t ok = pdPASS;

    maixcam_init();
    tb6612_init();
    encoder_init();
    gray_sensor_init();
    if (!speed_service_init() || !line_tracking_service_init()) {
        tb6612_brake();
        ball_control_emergency_stop();
        while (1) {}
    }
    speed_service_emergency_stop();
    ball_control_emergency_stop();

    if (xTaskCreate(maixcam_parse_task, "MAIXCAM", 384, NULL, 4, NULL) != pdPASS)
        ok = pdFAIL;
    if (xTaskCreate(control_task, "CONTROL", 512, NULL, 3,
                    &g_control_task_handle) != pdPASS)
        ok = pdFAIL;
    if (xTaskCreate(key_task, "KEY", 256, NULL, 2, NULL) != pdPASS)
        ok = pdFAIL;
    if (xTaskCreate(oled_task, "OLED", 512, NULL, 1, NULL) != pdPASS)
        ok = pdFAIL;
    if (xTaskCreate(led_task, "LED", 128, NULL, 1, NULL) != pdPASS)
        ok = pdFAIL;

    if (ok != pdPASS) {
        speed_service_emergency_stop();
        ball_control_emergency_stop();
        while (1) {}
    }

    vTaskStartScheduler();
    speed_service_emergency_stop();
    ball_control_emergency_stop();
    while (1) {}
}

void GROUP1_IRQHandler(void)
{
    if (encoder_right_int_is_pending()) {
        encoder_right_irq_handler();
    }
}

void TIMER_0_INST_IRQHandler(void)
{
    BaseType_t task_woken = pdFALSE;

    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO &&
        g_control_task_handle != NULL) {
        vTaskNotifyGiveFromISR(g_control_task_handle, &task_woken);
    }
    portYIELD_FROM_ISR(task_woken);
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    speed_service_emergency_stop();
    ball_control_emergency_stop();
    while (1) {}
}

void vApplicationGetIdleTaskMemory(StaticTask_t **tcb, StackType_t **stack,
                                   uint32_t *size)
{
    static StaticTask_t idle_tcb;
    static StackType_t idle_stack[configMINIMAL_STACK_SIZE];
    *tcb = &idle_tcb;
    *stack = idle_stack;
    *size = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **tcb, StackType_t **stack,
                                    uint32_t *size)
{
    static StaticTask_t timer_tcb;
    static StackType_t timer_stack[configTIMER_TASK_STACK_DEPTH];
    *tcb = &timer_tcb;
    *stack = timer_stack;
    *size = configTIMER_TASK_STACK_DEPTH;
}
