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
//tactile sensors definitions
#define NROWS 4 //number of rows
#define NCOLS 4 //number of columns
#define NPATCH 1 //number of patches
#define MAX_ADC 9 //number of adc channels
#define DATA_BUFFER_SIZE 36 //1*2*16 + 2 + 2 (sensores * n taxil * 2 (bytes) + 2 (bytes inicio/fim) + 2 (canal AD)
#define PKG_ST 0x24
#define PKG_ET 0x21
//------------------------------------------------------------
