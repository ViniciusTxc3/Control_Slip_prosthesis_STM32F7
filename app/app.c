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
#include "definitions.h"
#include "bebionic.h"
#include "izhikevich.h"

// Variaveis para modelo Izihikevich
uint32_t adcBuffer[MAX_ADC];
uint8_t usbBuffer[DATA_BUFFER_SIZE];
uint8_t taxelCounter = 1;
uint8_t row = 0;
uint8_t col = 0;
uint8_t start = 0;
uint8_t auxRow = 4;
uint8_t sendSpike_test = 0; // teste para enviar serial

Izhikevich neuronioSA;//[16];
Izhikevich neuronioFA;
uint8_t contN = 1;
uint8_t nSA = 0;
uint8_t nFA = 0;
double soma_input_anterior = 0.0, resp_soma_input = 0.0;
uint8_t n = 100; // Tamanho da MAV
uint16_t recebe_input[16][100]; // Vetor da MAV
uint16_t somaSinal[16];

// Variaveis modelo ode
#define N_NEURONS 16

// Teste FiltroMAV
double recebeADC = 0.0;
uint16_t arrNumbers[N_NEURONS] = {0}; // Vetor de valores anteriores
uint8_t receiveSA[18] = {0};// = adc_read_ffcX;//(4095 - adc_read_ffcX); // Test
uint8_t receiveFA[N_NEURONS + 1] = {0};
uint8_t receive_neuron[(N_NEURONS *2) + 1];// = {0};
//uint16_t pos = 0;
//uint16_t newAvg = 0;
//uint16_t sum = 0;
//uint16_t len = sizeof(arrNumbers) / sizeof(uint16_t);
//uint16_t count = sizeof(recebeADC) / sizeof(uint16_t);


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

uint8_t process_bebionic_commands(uint8_t *Buf);
uint8_t ledFlag = 0;

void app_init(void) {
	// Iniciando modelo Izihikevich
	set_model_parameters(&neuronioSA,0.02, 0.2, -65.0, 8.0, 1.0, -65.0, 0.2); // Neurônio ativação
	set_model_parameters(&neuronioFA, 0.1, 0.2, -65.0, 2.0, 1.0, -65.0, 0.2); // Neurônio transição
	hw_usb_init();
	/*
	app_init_ode_solver();
	hw_adc_calibration();
	hw_timer_start();
	app_started = true;
	*/
	hw_timer_start();
	app_started = true;

	usb_send_buffer[0] = PKG_ST;
	usb_send_buffer[1] = PKG_ET;

	  // stop the timer
	hw_timer2_stop(); // teste para controle da protese
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

	app_get_neurons(adc_read_ffc5); // Teste para gerar Spikes
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

void app_controle_bebionic(void)
{
	if (usb_command_received == 1)
	    {
	      usb_command_received = 0;                  // reset the usb command received
	      bebionic_process_commands(usb_rcv_buffer); // pass the usb buffer to be processed
	      sendSpike_test = 1;
	    }
}

void app_timer_tick(void) // function called by the timer once an interrupt is triggered
{
  bebionic_process_timer(); // calls the function that handles the actions of the bebionic
}

void app_get_neurons(uint16_t *adc_read_ffcX)
{
//	hw_led_toggle();

	if (BEBIONIC_STATE)
	{
		if(ledFlag > 5) // Roda depois de 10x
		{
			BEBIONIC_STATE = 0x01;
			ledFlag = 0;
//			receive_neuron[0] = 77;//PKG_ST;
//			receiveSA[0] = PKG_ST;//77;
//			receiveFA[0] = PKG_ST;
			//receive_neuron[((N_NEURONS + 1) * 2) - 1] = 77;//PKG_ET;
			for(uint8_t i = 0 ; i < N_NEURONS; i++)
			{
				int16_t derivate_FA = abs(arrNumbers[i]- adc_read_ffcX[i]);

//				receive_neuron[i] = integrate(&neuronioSA, adc_read_ffcX[i] / 80.0);
//				receive_neuron[N_NEURONS + i] = integrate(&neuronioFA, derivate_FA / 80.0);

				receiveSA[i] = integrate(&neuronioSA, adc_read_ffcX[i] / 80.0);
//				receiveFA[i] = integrate(&neuronioFA, derivate_FA / 80.0);

				arrNumbers[i] = adc_read_ffcX[i]; // Coloca como valor antigo
			}

			nSA = !nSA;
			receive_neuron[(N_NEURONS * 2)] = '\n';

//			receiveSA[0] = nSA;
//			receiveSA[1] = !nSA;
//			receiveSA[2] = 0;
			receiveSA[N_NEURONS] = '\n';
			CDC_Transmit_FS(receiveSA, N_NEURONS + 1);
//			hw_usb_tx(receiveSA, N_NEURONS + 1);//receive_neuron, (N_NEURONS * 2) + 1);
			hw_led_state_set(receiveSA[0]);
		}
		ledFlag++;
	}
}

void timerADC_old(void)
{
// 	if(start != 0)
// 	{
// 		//read the first four adc channels
// 		for(int i=0; i<NPATCH; i++)
// 		{
// 			//read the adc channels from the multiplexed tactile patches
// 			if((i < (NPATCH-1)) || (i == 0))
// 			{
// // NOVO
// 				if((row == 2) && (col == 2))
// 				{
// 				recebeADC = (4095 - adcBuffer[i]);// / 100.0; // Recebe valor analógico do sensor
// 				//FilterMAV(); // Aplica média movel
// 				moving_average = runningAverage(4200 - adcBuffer[i]);//recebeADC);
// 				input_fa1 = abs(moving_average - last_moving_average); // realiza a derivada
// 				last_moving_average = moving_average; // Atualiza valor para antiga média móvel

// 				usbBuffer[taxelCounter] = integrate(&neuronioFA, input_fa1);
// 				taxelCounter++;
// 				usbBuffer[taxelCounter] = integrate(&neuronioSA, recebeADC / 80.0);
// 				taxelCounter++;

// 				contN++;
// 				}
// 				//nFA = integrate(&neuronioFA[contN], resp_soma_input);
// 				/*
// 				if((NROWS == 4) && (NCOLS == 4))
// 				{
// 					soma_input_anterior = saida; // Atualiza valor anterior

// 					nSA = recebe_neuronio;
// 					//FiltroMAV(recebeADC);
// 					nFA = integrate(&neuronioFA, resp_soma_input);
// 				}
// /*
// 				if(recebe_neuronio != 0)
// 				{
// 					uint8_t pontoMatriz = col; // revela ponto da matriz ativado (concatena para informar posição)
// 					pontoMatriz += row * 4;
// 					usbBuffer[cont_neuronio] = pontoMatriz - 4;
// 					cont_neuronio++; // conta neuronio ativado
// 				}
// */
// 				//usbBuffer[taxelCounter] = adcBuffer[i] >> 8;
// 				//usbBuffer[taxelCounter+1] = adcBuffer[i] & 0xff;
// // FIM DO NOVO

// 				//usbBuffer[taxelCounter] = recebe_neuronio >> 8;
// 				//usbBuffer[taxelCounter+1] = recebe_neuronio & 0xff;
// 				//taxelCounter += 2;
// 				//taxelCounter++;
// 				//contN++;


// 			}
// 			else //read the four adc channels from the last tactile patch
// 			{

// 				//usbBuffer[taxelCounter] = adcBuffer[i+(row-1)] >> 8;
// 				//usbBuffer[taxelCounter+1] = adcBuffer[i+(row-1)] & 0xff;
// 				taxelCounter += 2;
// 			}
// 		}

// 		//send usb data after all taxels have been read
// 		if(taxelCounter >= NROWS*NCOLS*NPATCH*2)
// 		{
// // TESTE NOVO
// 		//if(contTemp % 300 == 0) // Olha se tem algum dado para enviar
// 		//{
// 			/*
// 			if(cont_neuronio > 4)
// 			{
// 				usbBuffer[0] = PKG_ST; //package header
// 				usbBuffer[1] = cont_neuronio-2; // Posição reservada para o numero total de neurônios ativados
// 				usbBuffer[2] = contTemp >> 8;  // Contagem do tempo
// 				usbBuffer[3] = contTemp  & 0xff;
// 				usbBuffer[cont_neuronio] = PKG_ET; //end of package
// 				CDC_Transmit_FS(usbBuffer,cont_neuronio+1); //send data via usb
// 				cont_neuronio = 4;
// 			}

// 			contTemp++;
// 			if(contTemp >= 65535) // Se contador de tempo chegar ao máximo é resetado
// 				contTemp = 0;
// 				*/
// // FIM TESTE

// 			//RETIRAR
// 			/*
// 			//read the slip sensor --> like an aux channel
// 			usbBuffer[taxelCounter] = adcBuffer[MAX_ADC-1] >> 8;
// 			usbBuffer[taxelCounter+1] = adcBuffer[MAX_ADC-1] & 0xff;

// 			usbBuffer[0] = PKG_ST; //package header
// 			usbBuffer[DATA_BUFFER_SIZE-1] = PKG_ET; //end of package
// 			CDC_Transmit_FS(usbBuffer,DATA_BUFFER_SIZE); //send data via usb
// 			*/

// 			// NOVO FA
// 			usbBuffer[0] = PKG_ST; //package header
// 			//usbBuffer[1] = contN;
// 			//usbBuffer[1] = nSA; // Posição reservada para o numero total de neurônios ativados
// 			//usbBuffer[2] = nFA;  // Contagem do tempo
// 			usbBuffer[taxelCounter] = PKG_ET; //end of package
// 			taxelCounter++;
// 			CDC_Transmit_FS(usbBuffer,taxelCounter); //send data via usb
// 			// FIM DO NOVO FA
// 			taxelCounter = 1; //increment taxel counter
// 			contN = 1;
// 		}
// 			 //ATÉ AQUI
// 	}
// 	else
// 		start = 1;

// 	//if all rows have been read, the column has to be changed
// 	if(row == NROWS)
// 	{
// 		//reset the rows
// 		row = 0;
// 		//increment column
// 		col++;
// 		//if all columns have been read, reset
// 		if(col >= NCOLS)
// 			col = 0;

// 		//select the new column
// 		select_column(col);
// 	}

// 	//select the row
// 	select_row(row);

// 	//trigger ADC
// 	HAL_ADC_Start_DMA(&hadc1,adcBuffer,MAX_ADC);

// 	//increment row
// 	row++;
}
