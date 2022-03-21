/*
 * hw.c
 *
 *  Created on: Nov 4, 2021
 *      Author: vinic
 */
/*
 * Link: 	https://os.mbed.com/platforms/ST-Nucleo-F767ZI/
 * 			file:///C:/Users/vinic/AppData/Local/Temp/stm32f767zi.pdf
 * 			file:///C:/Users/vinic/AppData/Local/Temp/nucleo-f767zi.pdf
 * 			file:///C:/Users/vinic/AppData/Local/Temp/dm00244518-stm32-nucleo144-boards-mb1137-stmicroelectronics.pdf
 * 			https://docs.google.com/document/u/0/d/1oI5DEEcAGrtcy6bprIj_35AK4aKURbOCSm3-ezbdbmk/mobilebasic
 */
/* CONECTOR FFC5:
 * Pinos usados na conexão Nucleo F7 e uC (LINHAS) - Pins Arduino
 * L1:	PA_3 (ADC1/3) - A0
 * L2:	PC_0 (ADC1/10) - A1
 * L3:	PC_3 (ADC1/13) - A2
 * L4:	PA_7 (ADC1/7) -
 *
 * Pinos usados na conexão Nucleo F7 e uC (COLUNAS)
 * COL1		COL2	COL3	COL4
 * PF_13	PE_9	PE_11	PF_14
 *
 * Pinos STM32F103
 * #define IN_C1 GPIO_PIN_15 //PORTA (PA15)
 * #define IN_C4 GPIO_PIN_10 //PORTA (PA10)
 * #define IN_C2 GPIO_PIN_7 //PORTB (PB7)
 * #define IN_C3 GPIO_PIN_6 //PORTB (PB6)
 *
 * Linha 1: PA5
 * Linha 1: PA4
 * Linha 1: PA7
 * Linha 1: PA6
 */
/*
 * ANOTAÇÕES:
 *
 * TIMER:
 * 	CLOCK EM 216Mz
 * 	Prescale: 43200 = 216000/5
 * 	Count period: 5 para gerar em 1 ms
 * 	--------------------------------------------------------------------------------------------------
 * millis = HAL_GetTick(); // igual
 *-----------------------------------------------------------------------------------------------------
 * SOBRE ADC:
 * 		COMENTAR FUNÇÃO: static void MX_ADC1_Init(void)
 * 		Do ponto: /** Configure for the selected ADC regular channel its corresponding ...
 *			 		Até o final
 *-----------------------------------------------------------------------------------------------------
 * SOBRE USB:
 * Configurar .ioc corretamente como nesse programa
 * Alterar pasta: usbd_cdc_if.c conforme tutorial nas referências acima (ST USB Device Libary)
 */

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "app.h"
#include "stm32f7xx_hal_adc.h"
#include <stdio.h>
#include "odesolver.h"
#include <locale.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;
extern ADC_ChannelConfTypeDef sConfig = {0};

void hw_adc_start(void) {
	HAL_ADC_Start(&hadc1);
	//HAL_ADC_Start_IT(&hadc1); // Se for por timer
}

void hw_Select_ch1(void) {
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	//sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

void hw_Select_ch2(void) {
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	//sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

void hw_Select_ch3(void) {
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	//sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

void hw_Select_ch4(void) {
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = ADC_REGULAR_RANK_4;
	//sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

void hw_adc_calibration(void) {
	// Essa familia (STM32F7) não possue essa função de auto calibração
	//HAL_ADCEx_Calibration_Start(&hadc1);
}

void hw_adc_read_single(uint8_t _channel, uint16_t *adc_val) {

	switch (_channel) {
	case 0:
		hw_Select_ch1();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		*adc_val = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		break;
	case 1:
		hw_Select_ch2();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		*adc_val = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		break;
	case 2:
		hw_Select_ch3();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		*adc_val = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		break;
	case 3:
		hw_Select_ch4();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		*adc_val = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		break;
	}

	/*
	 HAL_StatusTypeDef adc_hal_status;

	 while (1) {
	 adc_hal_status = HAL_ADC_PoolForConversion(&hadc1, 2);
	 if (adc_hal_status == HAL_OK) {
	 *adc_val = (uint16_t) HAL_ADC_GetValue(&hadc1);
	 return true;
	 } else if (adc_hal_status != HAL_OK) {
	 return false;
	 }
	 }
	 */
}

uint32_t hw_tick_ms_get(void) {
	return HAL_GetTick();
}

void hw_cpu_sleep(void) {
	/*
	 HAL_SuspendTick(); // Outro modo que não fica utilizando o Tick
	 HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	 HAL_ResumeTick();
	 */
	__WFI(); // Recurso do cortex para econômia de energia e acorda quando acontece interrupção
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13) {
		app_switch_interrupt();
	}
}

void hw_led_toggle(void) {
	HAL_GPIO_TogglePin(USER_LED_RED_GPIO_Port, USER_LED_RED_Pin);
}

bool hw_switch_state_get(void) {
	GPIO_PinState switch_state = HAL_GPIO_ReadPin(USER_SW_GPIO_Port,
	USER_SW_Pin);

	if (switch_state == GPIO_PIN_SET)
		return true;
	else
		return false;
}

void hw_led_state_set(bool _state) {
	GPIO_PinState led_state = _state ? GPIO_PIN_RESET : GPIO_PIN_SET;

	HAL_GPIO_WritePin(USER_LED_RED_GPIO_Port, USER_LED_RED_Pin, led_state);
}

void hw_delay_ms(uint32_t _time_ms) {
	HAL_Delay(_time_ms);
}

void hw_timer_start(void) {
	//__HAL_TIM_SET_AUTORELOAD(&htim1, 1999)
	//__HAL_TIM_SET_COUTER(&htim1, 0);
	HAL_TIM_Base_Start_IT(&htim1);
}

void hw_timer_stop(void) {
	HAL_TIM_Base_Stop_IT(&htim1);
}

void hw_timer2_stop(void)
{
	HAL_TIM_Base_Stop_IT(&htim2);
}

void hw_timer2_start(void) {
	//__HAL_TIM_SET_AUTORELOAD(&htim1, 1999)
	//__HAL_TIM_SET_COUTER(&htim1, 0);
	HAL_TIM_Base_Start_IT(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim1) {
		//hw_led_toggle();

		/*
		uint16_t adc_read_ffc5[16] = {0};
		app_leitura_matriz_FFC5(adc_read_ffc5);
		uint16_t result_average_FFC5 = app_average_matriz(adc_read_ffc5);
*/
		app_output_ode_data();//result_average_FFC5);//adc_read_ffc5);
	}

	if (htim == &htim2) {
		app_timer_tick();
	}
}

/** Pinos usados na conexão Nucleo F7 e uC (LINHAS)
 * L1:	PA_3 (ADC1/3)
 * L2:	PC_0 (ADC1/10)
 * L3:	PC_3 (ADC1/13)
 * L4:	PA_7 (ADC1/7)
 */
uint16_t hw_set_pins_row_FFC5(uint8_t _pin) {
	uint16_t adc_point_FFC5;

	hw_adc_read_single(_pin, &adc_point_FFC5);

	return adc_point_FFC5;
}

/** Pinos usados na conexão Nucleo F7 e uC (COLUNAS)
 * COL1		COL2	COL3	COL4
 * PF_13	PE_9	PE_11	PF_14
 */
void hw_get_pins_col_FFC5(uint8_t _pin) {
	// Inicia todos os pinos desligados
	HAL_GPIO_WritePin(FFC5_C1_GPIO_Port, FFC5_C1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FFC5_C2_GPIO_Port, FFC5_C1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FFC5_C3_GPIO_Port, FFC5_C1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FFC5_C4_GPIO_Port, FFC5_C1_Pin, GPIO_PIN_SET);
	// Seleciona coluna
	switch (_pin) {
	case 0:
		HAL_GPIO_WritePin(FFC5_C1_GPIO_Port, FFC5_C1_Pin, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(FFC5_C2_GPIO_Port, FFC5_C2_Pin, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(FFC5_C3_GPIO_Port, FFC5_C3_Pin, GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(FFC5_C4_GPIO_Port, FFC5_C4_Pin, GPIO_PIN_RESET);
		break;
	}
}

uint8_t hw_usb_rx(uint8_t _Buf,int32_t _Len)
{
	return CDC_Receive_FS(_Buf, _Len);
}

uint8_t hw_usb_tx(uint8_t _Buf,int32_t _Len)
{
	return CDC_Transmit_FS(_Buf, _Len);
}
