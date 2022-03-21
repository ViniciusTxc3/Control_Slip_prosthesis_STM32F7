/*
 * app.c
 *
 *  Created on: Nov 4, 2021
 *      Author: vinic
 */

#include <stdint.h>
#include <stdbool.h>
#include "app.h"
#include "main.h"
#include "hw.h" // Biblioteca de portabilidade entre harwares
#include <stdio.h>
#include "odesolver.h"
#include <locale.h>

// Variaveis modelo ode
#define N_NEURONS 16


// Cria os objetos com os repectivos modelos. Deve ser incluído no início do código, antes do loop infinito do
// microcontrolador.
t_ode_system *izhikevich_neurons[N_NEURONS]; // izhikevich
t_ode_system *alpha_neurons[N_NEURONS]; // alpha function
ode_data_type weights[N_NEURONS]; // pesos a serem utilizados com o alpha function.

ode_data_type input_currents[N_NEURONS];
// Fim das variaveis

#define APP_DEBOUCING_TIME_MS 50
volatile uint32_t led_time_ms = 100;
bool app_started = false;

#define ADC_SAMPLES_N 2
#define ADC_CHANNEL_N 16
#define ADC_SET_0 4095 // Nível ADC quando não há pressão, usada para o sinal aumentar

uint16_t adc_raw_data[ADC_SAMPLES_N] = { 0 };

void app_init(void) {
	app_init_ode_solver();
	hw_adc_calibration();
	hw_timer_start();
	app_started = true;
}

void app_loop(void) {
	hw_cpu_sleep();
}

void app_average_adc_value(uint16_t *adc_raw_val, uint16_t *adc_avg_ch1,
		uint16_t *adc_avg_ch2) {
	uint16_t cnt = 0;
	uint32_t sum_temp[ADC_CHANNEL_N] = { 0, 0 };

	while (cnt < ADC_SAMPLES_N) {
		for (uint8_t i = 0; i < ADC_CHANNEL_N; i++)
			sum_temp[i] = sum_temp[i] + adc_raw_val[cnt + 1];
		cnt += ADC_CHANNEL_N;
	}
	for (uint8_t i = 0; i < ADC_CHANNEL_N; i++)
		sum_temp[i] = sum_temp[i] / (ADC_SAMPLES_N / ADC_CHANNEL_N);

	*adc_avg_ch1 = (uint16_t) sum_temp[0];
	*adc_avg_ch2 = (uint16_t) sum_temp[1];
}

void app_tick_1ms(void) // SysTick_Handler(void) -> stm32F7xx_it.c
{

	//static uint32_t led_time_ctn_ms = 0;

	if (!app_started) // Espera inicializar o microcontrolador
		return;
	/*
	 led_time_ctn_ms++;

	 if(led_time_ctn_ms >= led_time_ms)
	 {
	 led_time_ctn_ms = 0;
	 hw_led_toggle();
	 }
	 */
}

void app_switch_interrupt(void) {
	static uint32_t deboucing_time_ms = 0;

	if (!app_started) // Espera inicializar o microcontrolador
		return;

	if ((hw_tick_ms_get() - deboucing_time_ms) >= APP_DEBOUCING_TIME_MS) {
		if (led_time_ms == 400)
			led_time_ms = 100;
		else
			led_time_ms = 400;

		deboucing_time_ms = hw_tick_ms_get();
	}
}

/** Disposição da posições do vetor na matriz:
 * [	00, 01, 02, 03,
 * 		04, 05, 06, 07,
 * 		08, 09, 10, 11,
 * 		12, 13, 14, 15	]
 */
void app_leitura_matriz_FFC5(uint16_t *adc_read_ffc5) {
	uint16_t _adc_read_ffc5 [ADC_CHANNEL_N] = {0};
	uint8_t posicao_matriz = 0;

	for (uint8_t i = 0; i < 4; i++) {
		//Verifica valor das colunas
		hw_get_pins_col_FFC5(i);

		//Alterna o estado dos pinos das linhas
		for (uint8_t j = 0; j < 4; j++) {
			// Coloca valores do ADC no vetor
			_adc_read_ffc5[posicao_matriz] = ADC_SET_0 - hw_set_pins_row_FFC5(j); // Retira o valor base para o sinal ficar crescente
			posicao_matriz++;
		}
	}
	adc_read_ffc5 = _adc_read_ffc5;
}

uint16_t app_average_matriz(uint16_t _adc_read_ffc[]) {
	uint16_t result_average = 0;
	for (uint8_t i = 0; i < 16; i++) {
		result_average += _adc_read_ffc[i];
	}

	return result_average /= 16;
}


void app_init_ode_solver(void) {

	setlocale(LC_NUMERIC, "pt_BR.UTF-8");

	// Aqui deve-se definir os pesos dos neurônios.
	// Define todos os pesos como zero
	for (int i = 0; i < N_NEURONS; i++) {
		weights[i] = 0;
	}
	weights[1] = 1; //Só ativa o segundo neurônio

	// Essa função cria a  rede neural com 16 neurônios
	ode_status_t status = fill_neuron_set(izhikevich_neurons, alpha_neurons, // Arrays com os modelos
			N_NEURONS, // Número de neurônios
			0.02f, 0.2f, -65.0f, 8.0f, // Parâmetros de Izhikevich a,b,c e d
			0.01f, weights); // tau do alpha function e pesos definidos anteriormente.

	if (status != ODE_OK) {
		// Houve uma falha na hora de alocar a memória para os neurônios.
		// Encerra o programa com um erro.
		return 1;
	}
/*
	// Essas são as correntes de entrada dos neurônios. O conversor AD deve preencher essas correntes.
	ode_data_type input_currents[N_NEURONS];


	// O código dentro desse laço é o que deve ser executado a cada 1 ms para obter as amostras do AD e calcular a saída
	    // do modelo.
	    for(int i = 0; i < 200; i++)
	    {
	        //Lê as amostras dos ads
	    	app_example_read_adc_samples(input_currents, N_NEURONS);

	        // Simula a rede neural - output é a resposta do potencial pós-sinápitico
	        ode_data_type output = process_neuron_set_step(izhikevich_neurons, alpha_neurons, input_currents, N_NEURONS, 1.0f);

	        // Coloca a resposta na saída
	        output = output + 0;
	    }
*/
}

void app_output_ode_data(void)//uint16_t _input_currents)//[])
{
	// Essas são as correntes de entrada dos neurônios. O conversor AD deve preencher essas correntes.
	//ode_data_type input_currents[N_NEURONS]; // = _input_currents;
	app_leitura_matriz_FFC5(input_currents);
	ode_data_type output = process_neuron_set_step(izhikevich_neurons, alpha_neurons, input_currents, N_NEURONS, 1.0f);
	ode_data_type SA_I = izhikevich_neurons[1]->y[0];  // Saída do 2 neuronio (v)
}


void app_example_read_adc_samples(ode_data_type * input_currents, int num_channels);
void app_example_read_adc_samples(ode_data_type * input_currents, int num_channels)
{
    for(int i = 0; i < num_channels; i++)
	{
		  input_currents[i] = (0.0f);
	}
	//Adiciona só uma corrente ao final ao canal 2. 0.5 é um ganho que vai ter que ser ajustado.
	input_currents[1] = (0.5f * 1024); // No caso o ADC leu o valor 1024
}
