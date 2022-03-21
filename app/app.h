/*
 * app.h
 *
 *  Created on: Nov 4, 2021
 *      Author: vinic
 */

#ifndef APP_H_
#define APP_H_

void app_init(void);
void app_loop(void);
void app_tick_1ms(void);
void app_switch_interrupt(void);
void app_leitura_matriz_FFC5(uint16_t *adc_read_ffc5);
//void app_leitura_matriz_FFC5(uint16_t adc_read_ffc5[]);
uint16_t app_average_matriz(uint16_t _adc_read_ffc[]);
void app_init_ode_solver(void);
void app_output_ode_data(void);//uint16_t input_currents);//[]);


#endif /* APP_H_ */