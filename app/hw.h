/*
 * hw.h
 *
 *  Created on: Nov 4, 2021
 *      Author: vinic
 */

#ifndef HW_H_
#define HW_H_

bool hw_switch_state_get(void);
void hw_led_state_set(bool _state);
void hw_delay_ms(uint32_t _time_ms);
void hw_led_toggle(void);
void hw_cpu_sleep(void);
uint32_t hw_tick_ms_get(void);
void hw_timer_start(void);
void hw_timer_stop(void);
void hw_adc_start(void);
void hw_adc_calibration(void);
void hw_adc_read_single(uint8_t _channel, uint16_t *adc_val);
uint16_t hw_set_pins_row_FFC5(uint8_t _pin);
void hw_get_pins_col_FFC5(uint8_t _pin);
void hw_Select_ch1(void);
void hw_Select_ch2(void);
void hw_Select_ch3(void);
void hw_Select_ch4(void);
uint8_t hw_usb_rx(uint8_t _Buf,int32_t _Len);
uint8_t hw_usb_tx(uint8_t _Buf,int32_t _Len);

#endif /* HW_H_ */
