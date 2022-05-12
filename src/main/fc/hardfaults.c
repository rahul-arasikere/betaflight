/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/light_led.h"
#include "drivers/time.h"
#include "drivers/transponder_ir.h"

#include "fc/init.h"

#include "flight/mixer.h"

#if defined(STM32F7) && !defined(DEBUG_HARDFAULTS)
void MemManage_Handler(void)
{
    LED2_ON;

    // fall out of the sky
    uint8_t requiredStateForMotors = SYSTEM_STATE_CONFIG_LOADED | SYSTEM_STATE_MOTORS_READY;
    if ((systemState & requiredStateForMotors) == requiredStateForMotors)
    {
        stopMotors();
    }

#ifdef USE_TRANSPONDER
    // prevent IR LEDs from burning out.
    uint8_t requiredStateForTransponder = SYSTEM_STATE_CONFIG_LOADED | SYSTEM_STATE_TRANSPONDER_ENABLED;
    if ((systemState & requiredStateForTransponder) == requiredStateForTransponder)
    {
        transponderIrDisable();
    }
#endif

    LED1_OFF;
    LED0_OFF;

    while (1)
    {
        delay(500);
        LED2_TOGGLE;
        delay(50);
        LED2_TOGGLE;
    }
}
#endif

#ifdef DEBUG_HARDFAULTS
// from: https://mcuoneclipse.com/2012/11/24/debugging-hard-faults-on-arm-cortex-m/
/**
 * hard_fault_handler_c:
 * This is called from the HardFault_HandlerAsm with a pointer the Fault stack
 * as the parameter. We can then read the values from the stack and place them
 * into local variables for ease of reading.
 * We then read the various Fault Status and Address Registers to help decode
 * cause of the fault.
 * The function ends with a BKPT instruction to force control back into the debugger
 */
void hard_fault_handler_c(unsigned long *hardfault_args)
{
    volatile unsigned long stacked_r0;
    volatile unsigned long stacked_r1;
    volatile unsigned long stacked_r2;
    volatile unsigned long stacked_r3;
    volatile unsigned long stacked_r12;
    volatile unsigned long stacked_lr;
    volatile unsigned long stacked_pc;
    volatile unsigned long stacked_psr;
    volatile unsigned long _CFSR;
    volatile unsigned long _HFSR;
    volatile unsigned long _DFSR;
    volatile unsigned long _AFSR;
    volatile unsigned long _BFAR;
    volatile unsigned long _MMAR;

    stacked_r0 = ((unsigned long)hardfault_args[0]);
    stacked_r1 = ((unsigned long)hardfault_args[1]);
    stacked_r2 = ((unsigned long)hardfault_args[2]);
    stacked_r3 = ((unsigned long)hardfault_args[3]);
    stacked_r12 = ((unsigned long)hardfault_args[4]);
    stacked_lr = ((unsigned long)hardfault_args[5]);
    stacked_pc = ((unsigned long)hardfault_args[6]);
    stacked_psr = ((unsigned long)hardfault_args[7]);

    // Configurable Fault Status Register
    // Consists of MMSR, BFSR and UFSR
    _CFSR = (*((volatile unsigned long *)(0xE000ED28)));

    // Hard Fault Status Register
    _HFSR = (*((volatile unsigned long *)(0xE000ED2C)));

    // Debug Fault Status Register
    _DFSR = (*((volatile unsigned long *)(0xE000ED30)));

    // Auxiliary Fault Status Register
    _AFSR = (*((volatile unsigned long *)(0xE000ED3C)));

    // Read the Fault Address Registers. These may not contain valid values.
    // Check BFARVALID/MMARVALID to see if they are valid values
    // MemManage Fault Address Register
    _MMAR = (*((volatile unsigned long *)(0xE000ED34)));
    // Bus Fault Address Register
    _BFAR = (*((volatile unsigned long *)(0xE000ED38)));

#ifdef PRINT_HARDFAULT_SERIAL
    UART_HandleTypeDef huart;
    char buffer[128];
    size_t len = 0;

    huart.Instance = USART3;
    huart.Init.BaudRate = 115200;
    huart.Init.WordLength = UART_WORDLENGTH_8B;
    huart.Init.StopBits = UART_STOPBITS_1;
    huart.Init.Parity = UART_PARITY_NONE;
    huart.Init.Mode = UART_MODE_TX_RX;
    huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart.Init.OverSampling = UART_OVERSAMPLING_16;
    huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_DeInit(&huart);
    if (HAL_UART_Init(&huart) != HAL_OK)
    {
        LED0_ON;
        while (1)
        {
            asm volatile("\tnop\n");
        }
    }
    while (1)
    {
        LED0_ON;
        delay(50);
        HAL_UART_Transmit(&huart, (uint8_t *)"-----------------------------------------------------\r\n", 56, 500);
        HAL_UART_Transmit(&huart, (uint8_t *)"fault handler\r\n", 19, 500);
        len = snprintf(buffer, 128, "lr: 0x%08x\r\n", stacked_lr);
        HAL_UART_Transmit(&huart, (uint8_t *)buffer, len, 500);
        len = snprintf(buffer, 128, "r0: 0x%08x\r\n", stacked_r0);
        HAL_UART_Transmit(&huart, (uint8_t *)buffer, len, 500);
        len = snprintf(buffer, 128, "r1: 0x%08x\r\n", stacked_r1);
        HAL_UART_Transmit(&huart, (uint8_t *)buffer, len, 500);
        len = snprintf(buffer, 128, "r2: 0x%08x\r\n", stacked_r2);
        HAL_UART_Transmit(&huart, (uint8_t *)buffer, len, 500);
        len = snprintf(buffer, 128, "r3: 0x%08x\r\n", stacked_r3);
        HAL_UART_Transmit(&huart, (uint8_t *)buffer, len, 500);
        len = snprintf(buffer, 128, "r12: 0x%08x\r\n", stacked_r12);
        HAL_UART_Transmit(&huart, (uint8_t *)buffer, len, 500);
        len = snprintf(buffer, 128, "ret addr: 0x%08x\r\n", stacked_pc);
        HAL_UART_Transmit(&huart, (uint8_t *)buffer, len, 500);
        len = snprintf(buffer, 128, "xpsr: 0x%08x\r\n", stacked_psr);
        HAL_UART_Transmit(&huart, (uint8_t *)buffer, len, 500);
        len = snprintf(buffer, 128, "cfsr: 0x%08x\r\n", _CFSR);
        HAL_UART_Transmit(&huart, (uint8_t *)buffer, len, 500);
        len = snprintf(buffer, 128, "hfsr: 0x%08x\r\n", _HFSR);
        HAL_UART_Transmit(&huart, (uint8_t *)buffer, len, 500);
        len = snprintf(buffer, 128, "dfsr: 0x%08x\r\n", _DFSR);
        HAL_UART_Transmit(&huart, (uint8_t *)buffer, len, 500);
        len = snprintf(buffer, 128, "afsr: 0x%08x\r\n", _AFSR);
        HAL_UART_Transmit(&huart, (uint8_t *)buffer, len, 500);
        len = snprintf(buffer, 128, "mmar: 0x%08x\r\n", _MMAR);
        HAL_UART_Transmit(&huart, (uint8_t *)buffer, len, 500);
        len = snprintf(buffer, 128, "bfar: 0x%08x\r\n", _BFAR);
        HAL_UART_Transmit(&huart, (uint8_t *)buffer, len, 500);
        LED0_OFF;
        delay(50);
    }

#endif

    __asm("BKPT #0\n"); // Break into the debugger
}

#else
void HardFault_Handler(void)
{
    LED0_ON;
    LED1_ON;
    LED2_ON;

    // fall out of the sky
    uint8_t requiredStateForMotors = SYSTEM_STATE_CONFIG_LOADED | SYSTEM_STATE_MOTORS_READY;
    if ((systemState & requiredStateForMotors) == requiredStateForMotors)
    {
        stopMotors();
    }

#ifdef USE_TRANSPONDER
    // prevent IR LEDs from burning out.
    uint8_t requiredStateForTransponder = SYSTEM_STATE_CONFIG_LOADED | SYSTEM_STATE_TRANSPONDER_ENABLED;
    if ((systemState & requiredStateForTransponder) == requiredStateForTransponder)
    {
        transponderIrDisable();
    }
#endif

    LED0_OFF;
    LED1_OFF;
    LED2_OFF;

    while (1)
    {
        delay(50);
        LED0_TOGGLE;
        LED1_TOGGLE;
        LED2_TOGGLE;
    }
}
#endif
