/*
 * -----------------------------------------------------------
 * FEDERAL UNIVERISTY OF UBERLANDIA - UFU
 * Faculty of Electrical Engineering - FEELT
 * Biomedical Engineering Lab - BIOLAB
 * Uberlandia, Brazil
 * -----------------------------------------------------------
 * Author: Andrei Nakagawa-Silva
 * -----------------------------------------------------------
 */
//------------------------------------------------------------
#ifndef INC_BEBIONIC_H_
#define INC_BEBIONIC_H_
//------------------------------------------------------------
//------------------------------------------------------------
#include "main.h"
#include "stm32f7xx_hal.h"
#include "definitions.h"
#include "stdint.h"

//Constants
//BeBionic Control
#define BEBIONIC_OK 0x20
#define BEBIONIC_ERROR 0x21
#define BEBIONIC_CLOSING 0x00
#define BEBIONIC_OPENING 0x01
#define BEBIONIC_STOP 0x02
#define BEBIONIC_IDLE 0x03
#define BEBIONIC_CHANGE_GRIP 0x04
#define BEBIONIC_THRESHOLD_MODE 0x40
#define BEBIONIC_PROPORTIONAL_MODE 0x41
//------------------------------------------------------------
//------------------------------------------------------------
//Variables
//------------------------------------------------------------
//------------------------------------------------------------
//Functions
//process the received usb commands
uint8_t bebionic_process_commands(uint8_t *Buf);
//bebionic control
void bebionic_process_timer(); //function to be called inside a timer: controls the duration of the action
uint8_t bebionic_change_grip(); //change grip
uint8_t bebionic_stop(); //stop movement of the hand
//Threshold Mode
uint8_t bebionic_th_open_hand(uint16_t duration); //open the hand
uint8_t bebionic_th_close_hand(uint16_t duration); //close the hand
//Proportional Mode
uint8_t bebionic_prop_open_hand(uint16_t percentage); //open the hand
uint8_t bebionic_prop_close_hand(uint16_t percentage); //close the hand
//------------------------------------------------------------
#endif /* INC_BEBIONIC_H_ */
