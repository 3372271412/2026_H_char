/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define CPUCLK_FREQ                                                     80000000



/* Defines for PWM_TB6612 */
#define PWM_TB6612_INST                                                    TIMG0
#define PWM_TB6612_INST_IRQHandler                              TIMG0_IRQHandler
#define PWM_TB6612_INST_INT_IRQN                                (TIMG0_INT_IRQn)
#define PWM_TB6612_INST_CLK_FREQ                                        40000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_TB6612_C0_PORT                                            GPIOA
#define GPIO_PWM_TB6612_C0_PIN                                    DL_GPIO_PIN_12
#define GPIO_PWM_TB6612_C0_IOMUX                                 (IOMUX_PINCM34)
#define GPIO_PWM_TB6612_C0_IOMUX_FUNC                IOMUX_PINCM34_PF_TIMG0_CCP0
#define GPIO_PWM_TB6612_C0_IDX                               DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_TB6612_C1_PORT                                            GPIOA
#define GPIO_PWM_TB6612_C1_PIN                                    DL_GPIO_PIN_13
#define GPIO_PWM_TB6612_C1_IOMUX                                 (IOMUX_PINCM35)
#define GPIO_PWM_TB6612_C1_IOMUX_FUNC                IOMUX_PINCM35_PF_TIMG0_CCP1
#define GPIO_PWM_TB6612_C1_IDX                               DL_TIMER_CC_1_INDEX




/* Defines for QEI_ENCODER_LEFT */
#define QEI_ENCODER_LEFT_INST                                              TIMG8
#define QEI_ENCODER_LEFT_INST_IRQHandler                        TIMG8_IRQHandler
#define QEI_ENCODER_LEFT_INST_INT_IRQN                          (TIMG8_INT_IRQn)
/* Pin configuration defines for QEI_ENCODER_LEFT PHA Pin */
#define GPIO_QEI_ENCODER_LEFT_PHA_PORT                                     GPIOA
#define GPIO_QEI_ENCODER_LEFT_PHA_PIN                             DL_GPIO_PIN_26
#define GPIO_QEI_ENCODER_LEFT_PHA_IOMUX                          (IOMUX_PINCM59)
#define GPIO_QEI_ENCODER_LEFT_PHA_IOMUX_FUNC             IOMUX_PINCM59_PF_TIMG8_CCP0
/* Pin configuration defines for QEI_ENCODER_LEFT PHB Pin */
#define GPIO_QEI_ENCODER_LEFT_PHB_PORT                                     GPIOA
#define GPIO_QEI_ENCODER_LEFT_PHB_PIN                             DL_GPIO_PIN_27
#define GPIO_QEI_ENCODER_LEFT_PHB_IOMUX                          (IOMUX_PINCM60)
#define GPIO_QEI_ENCODER_LEFT_PHB_IOMUX_FUNC             IOMUX_PINCM60_PF_TIMG8_CCP1


/* Defines for TIMER_0 */
#define TIMER_0_INST                                                    (TIMG12)
#define TIMER_0_INST_IRQHandler                                TIMG12_IRQHandler
#define TIMER_0_INST_INT_IRQN                                  (TIMG12_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                        (799999U)



/* Defines for UART_3 */
#define UART_3_INST                                                        UART3
#define UART_3_INST_FREQUENCY                                           80000000
#define UART_3_INST_IRQHandler                                  UART3_IRQHandler
#define UART_3_INST_INT_IRQN                                      UART3_INT_IRQn
#define GPIO_UART_3_RX_PORT                                                GPIOB
#define GPIO_UART_3_TX_PORT                                                GPIOB
#define GPIO_UART_3_RX_PIN                                         DL_GPIO_PIN_3
#define GPIO_UART_3_TX_PIN                                         DL_GPIO_PIN_2
#define GPIO_UART_3_IOMUX_RX                                     (IOMUX_PINCM16)
#define GPIO_UART_3_IOMUX_TX                                     (IOMUX_PINCM15)
#define GPIO_UART_3_IOMUX_RX_FUNC                      IOMUX_PINCM16_PF_UART3_RX
#define GPIO_UART_3_IOMUX_TX_FUNC                      IOMUX_PINCM15_PF_UART3_TX
#define UART_3_BAUD_RATE                                                (115200)
#define UART_3_IBRD_80_MHZ_115200_BAUD                                      (43)
#define UART_3_FBRD_80_MHZ_115200_BAUD                                      (26)
/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_FREQUENCY                                           40000000
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOB
#define GPIO_UART_1_TX_PORT                                                GPIOB
#define GPIO_UART_1_RX_PIN                                         DL_GPIO_PIN_7
#define GPIO_UART_1_TX_PIN                                         DL_GPIO_PIN_6
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM24)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM23)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM24_PF_UART1_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM23_PF_UART1_TX
#define UART_1_BAUD_RATE                                                (115200)
#define UART_1_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_1_FBRD_40_MHZ_115200_BAUD                                      (45)





/* Defines for ADC_GRAY */
#define ADC_GRAY_INST                                                       ADC0
#define ADC_GRAY_INST_IRQHandler                                 ADC0_IRQHandler
#define ADC_GRAY_INST_INT_IRQN                                   (ADC0_INT_IRQn)
#define ADC_GRAY_ADCMEM_0                                     DL_ADC12_MEM_IDX_0
#define ADC_GRAY_ADCMEM_0_REF                    DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC_GRAY_ADCMEM_0_REF_VOLTAGE_V                                      3.3
#define GPIO_ADC_GRAY_C4_PORT                                              GPIOB
#define GPIO_ADC_GRAY_C4_PIN                                      DL_GPIO_PIN_25



/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOB)

/* Defines for PIN_22: GPIOB.22 with pinCMx 50 on package pin 21 */
#define LED_PIN_22_PIN                                          (DL_GPIO_PIN_22)
#define LED_PIN_22_IOMUX                                         (IOMUX_PINCM50)
/* Port definition for Pin Group GPIO_KEY_TASK */
#define GPIO_KEY_TASK_PORT                                               (GPIOA)

/* Defines for PIN_KEY_TASK: GPIOA.18 with pinCMx 40 on package pin 11 */
#define GPIO_KEY_TASK_PIN_KEY_TASK_PIN                          (DL_GPIO_PIN_18)
#define GPIO_KEY_TASK_PIN_KEY_TASK_IOMUX                         (IOMUX_PINCM40)
/* Port definition for Pin Group GPIO_KEY_USER */
#define GPIO_KEY_USER_PORT                                               (GPIOB)

/* Defines for PIN_KEY_USER: GPIOB.21 with pinCMx 49 on package pin 20 */
#define GPIO_KEY_USER_PIN_KEY_USER_PIN                          (DL_GPIO_PIN_21)
#define GPIO_KEY_USER_PIN_KEY_USER_IOMUX                         (IOMUX_PINCM49)
/* Port definition for Pin Group GPIO_OLED_SW_I2C */
#define GPIO_OLED_SW_I2C_PORT                                            (GPIOA)

/* Defines for PIN_OLED_SDA: GPIOA.28 with pinCMx 3 on package pin 35 */
#define GPIO_OLED_SW_I2C_PIN_OLED_SDA_PIN                       (DL_GPIO_PIN_28)
#define GPIO_OLED_SW_I2C_PIN_OLED_SDA_IOMUX                       (IOMUX_PINCM3)
/* Defines for PIN_OLED_SCL: GPIOA.31 with pinCMx 6 on package pin 39 */
#define GPIO_OLED_SW_I2C_PIN_OLED_SCL_PIN                       (DL_GPIO_PIN_31)
#define GPIO_OLED_SW_I2C_PIN_OLED_SCL_IOMUX                       (IOMUX_PINCM6)
/* Defines for PIN_AIN1: GPIOB.17 with pinCMx 43 on package pin 14 */
#define GPIO_TB6612_PIN_AIN1_PORT                                        (GPIOB)
#define GPIO_TB6612_PIN_AIN1_PIN                                (DL_GPIO_PIN_17)
#define GPIO_TB6612_PIN_AIN1_IOMUX                               (IOMUX_PINCM43)
/* Defines for PIN_AIN2: GPIOB.19 with pinCMx 45 on package pin 16 */
#define GPIO_TB6612_PIN_AIN2_PORT                                        (GPIOB)
#define GPIO_TB6612_PIN_AIN2_PIN                                (DL_GPIO_PIN_19)
#define GPIO_TB6612_PIN_AIN2_IOMUX                               (IOMUX_PINCM45)
/* Defines for PIN_BIN1: GPIOA.16 with pinCMx 38 on package pin 9 */
#define GPIO_TB6612_PIN_BIN1_PORT                                        (GPIOA)
#define GPIO_TB6612_PIN_BIN1_PIN                                (DL_GPIO_PIN_16)
#define GPIO_TB6612_PIN_BIN1_IOMUX                               (IOMUX_PINCM38)
/* Defines for PIN_BIN2: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GPIO_TB6612_PIN_BIN2_PORT                                        (GPIOB)
#define GPIO_TB6612_PIN_BIN2_PIN                                (DL_GPIO_PIN_24)
#define GPIO_TB6612_PIN_BIN2_IOMUX                               (IOMUX_PINCM52)
/* Port definition for Pin Group GPIO_GRAY_ADDRESS */
#define GPIO_GRAY_ADDRESS_PORT                                           (GPIOB)

/* Defines for PIN_AD0: GPIOB.0 with pinCMx 12 on package pin 47 */
#define GPIO_GRAY_ADDRESS_PIN_AD0_PIN                            (DL_GPIO_PIN_0)
#define GPIO_GRAY_ADDRESS_PIN_AD0_IOMUX                          (IOMUX_PINCM12)
/* Defines for PIN_AD1: GPIOB.1 with pinCMx 13 on package pin 48 */
#define GPIO_GRAY_ADDRESS_PIN_AD1_PIN                            (DL_GPIO_PIN_1)
#define GPIO_GRAY_ADDRESS_PIN_AD1_IOMUX                          (IOMUX_PINCM13)
/* Defines for PIN_AD2: GPIOB.10 with pinCMx 27 on package pin 62 */
#define GPIO_GRAY_ADDRESS_PIN_AD2_PIN                           (DL_GPIO_PIN_10)
#define GPIO_GRAY_ADDRESS_PIN_AD2_IOMUX                          (IOMUX_PINCM27)
/* Port definition for Pin Group GPIO_ENCODER_RIGHT */
#define GPIO_ENCODER_RIGHT_PORT                                          (GPIOA)

/* Defines for PIN_RIGHT_A: GPIOA.25 with pinCMx 55 on package pin 26 */
// pins affected by this interrupt request:["PIN_RIGHT_A","PIN_RIGHT_B"]
#define GPIO_ENCODER_RIGHT_INT_IRQN                             (GPIOA_INT_IRQn)
#define GPIO_ENCODER_RIGHT_INT_IIDX             (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define GPIO_ENCODER_RIGHT_PIN_RIGHT_A_IIDX                 (DL_GPIO_IIDX_DIO25)
#define GPIO_ENCODER_RIGHT_PIN_RIGHT_A_PIN                      (DL_GPIO_PIN_25)
#define GPIO_ENCODER_RIGHT_PIN_RIGHT_A_IOMUX                     (IOMUX_PINCM55)
/* Defines for PIN_RIGHT_B: GPIOA.14 with pinCMx 36 on package pin 7 */
#define GPIO_ENCODER_RIGHT_PIN_RIGHT_B_IIDX                 (DL_GPIO_IIDX_DIO14)
#define GPIO_ENCODER_RIGHT_PIN_RIGHT_B_PIN                      (DL_GPIO_PIN_14)
#define GPIO_ENCODER_RIGHT_PIN_RIGHT_B_IOMUX                     (IOMUX_PINCM36)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_TB6612_init(void);
void SYSCFG_DL_QEI_ENCODER_LEFT_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_UART_3_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_ADC_GRAY_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
