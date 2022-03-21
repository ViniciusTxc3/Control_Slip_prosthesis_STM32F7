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
//------------------------------------------------------------
//libraries
#include "main.h"
#include "bebionic.h"
//------------------------------------------------------------
//------------------------------------------------------------
/* Global Variables */
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim2;
//------------------------------------------------------------
//------------------------------------------------------------
volatile uint16_t timer_counter = 0;
volatile uint8_t BEBIONIC_cmd_running = 0;
//------------------------------------------------------------
//------------------------------------------------------------
void bebionic_process_timer()
{
	//if for some reason the timer is active and the bebionic is in idle state
	//stop the timer and return
	if(BEBIONIC_STATE == BEBIONIC_IDLE)
	{
		//stop the timer
		HAL_TIM_Base_Stop_IT(&htim2);
		return; //exit function
	}

	//if the counter is greater than zero
	//decrement and continue
	if(timer_counter > 0)
	{
		timer_counter--;
	}

	//if the counter has reached zero, then it is time to stop the movement
	//movement is stopped by setting the control pin back to zero
	if(timer_counter <= 0)
	{
		//check if the BeBionic was opening or closing
		//and choose the proper command
		if(BEBIONIC_STATE == BEBIONIC_CLOSING) //stop from closing
			HAL_GPIO_WritePin(GPIOD, BB_ChB_Pin, GPIO_PIN_RESET);
		else if(BEBIONIC_STATE == BEBIONIC_OPENING) //stop from opening
			HAL_GPIO_WritePin(GPIOD, BB_ChA_Pin, GPIO_PIN_RESET);

		//change the flag indicating that the command has finished
		BEBIONIC_cmd_running = 0;

		//set the BeBionic to the IDLE state.
		//In the IDLE state, it is possible to receive a new command
		BEBIONIC_STATE = BEBIONIC_IDLE;

		//turn the LED off
		//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

		//stop the timer
		HAL_TIM_Base_Stop_IT(&htim2);
	}
}

//function that reads commands received via USB and decides which action should
//be performed
uint8_t bebionic_process_commands(uint8_t *Buf)
{
	//check if first byte of the packet is the ST byte
	if(Buf[0] == PKG_ST && Buf[PKG_SIZE-1] == PKG_ET)
	{
		uint16_t duration = 0;
		//first data byte: open, close or stop
		if(Buf[2] == BEBIONIC_OPENING) //open the hand
		{
			//second data byte: duration in ms (MSB)
			//third data byte: duration in ms (LSB)
			duration = Buf[3] << 8 | Buf[4];
			//call the function to open the hand
		    bebionic_th_open_hand(duration);
		}
		else if(Buf[2] == BEBIONIC_CLOSING) //close the hand
		{
			//second data byte: duration in ms (MSB)
			//third data byte: duration in ms (LSB)
			duration = Buf[3] << 8 | Buf[4];
			//call the function to close the hand
			bebionic_th_close_hand(duration);
		}
		else if(Buf[2] == BEBIONIC_STOP) //stops the BeBionic from moving
		{
			//set both control pins to zero
			HAL_GPIO_WritePin(GPIOD, BB_ChA_Pin|BB_ChB_Pin, GPIO_PIN_RESET);
			//reset state to IDLE
			BEBIONIC_STATE = BEBIONIC_IDLE;
			timer_counter = 0; //resets the timer counter
			BEBIONIC_cmd_running = 0; //bebionic completed action
			//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); //turn LED off
		}


		//check the state of the hand
		//if the hand is not idle, i.e. it is moving
		//then decide whether the timer has to be started or not
		if(BEBIONIC_STATE != BEBIONIC_IDLE)
		{
			//if another command is not being processed, then
			//start the timer
			if(BEBIONIC_cmd_running == 0)
			{
				HAL_TIM_Base_Start_IT(&htim2); //start the timer
				BEBIONIC_cmd_running = 1; //set the command flag
			}

			//turn the LED on
			//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		}

		return PKG_OK; //return successful operation
	}
	else //package not recognized
		return PKG_ERROR; //return false
}

//function to open the hand
uint8_t bebionic_th_open_hand(uint16_t duration)
{
	if(BEBIONIC_STATE == BEBIONIC_CLOSING) //stop from closing
	{
		HAL_GPIO_WritePin(GPIOD, BB_ChB_Pin, GPIO_PIN_RESET);
		//stop the timer
		HAL_TIM_Base_Stop_IT(&htim2);
		//reset the command flag
		BEBIONIC_cmd_running = 0;
		//resets the timer counter
		timer_counter = 0;
	}

	//Change BeBionic state to "Opening"
	BEBIONIC_STATE = BEBIONIC_OPENING;
	//Set the output in to high: Threshold control
	HAL_GPIO_WritePin(GPIOD, BB_ChA_Pin, GPIO_PIN_SET);
	//increment the time duration of the action
	timer_counter += duration;

	return BEBIONIC_OK;
}

//function to close the hand according to selected grip
uint8_t bebionic_th_close_hand(uint16_t duration)
{
	if(BEBIONIC_STATE == BEBIONIC_OPENING) //stop from opening
	{
		HAL_GPIO_WritePin(GPIOD, BB_ChA_Pin, GPIO_PIN_RESET);
		//stop the timer
		HAL_TIM_Base_Stop_IT(&htim2);
		//reset the command flag
		BEBIONIC_cmd_running = 0;
		//resets the timer counter
		timer_counter = 0;
	}

	//Change BeBionic state to "Closing"
	BEBIONIC_STATE = BEBIONIC_CLOSING;
	//Set the output in to high: Threshold control
	HAL_GPIO_WritePin(GPIOD, BB_ChB_Pin, GPIO_PIN_SET);
	//increment the time duration of the action
	timer_counter += duration;

	return BEBIONIC_OK;
}

