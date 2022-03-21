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
#include "stdint.h"
//USB comm
#define PKG_ST 0x24
#define PKG_ET 0x21
#define PKG_SIZE 6
#define PKG_DATA_SIZE 4
#define PKG_OK 1
#define PKG_ERROR 0
//------------------------------------------------------------
//Variables
uint8_t usb_send_buffer[PKG_SIZE];
uint8_t usb_rcv_buffer[PKG_SIZE];

uint8_t BEBIONIC_STATE;
uint8_t usb_command_received;

//------------------------------------------------------------

//------------------------------------------------------------
