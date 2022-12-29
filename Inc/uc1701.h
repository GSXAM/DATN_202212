// UC1701
// Library for accessing the UC1701 128x64 LCD display
// Written by Larry Bank (bitbank@pobox.com)
// Copyright (c) 2018 BitBank Software, Inc.
// Project started 2/22/2018
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "stm32f0xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

#define MODE_COMMAND GPIO_PIN_RESET
#define MODE_DATA 	 GPIO_PIN_SET

#define uc1701Set_CD_mode(mode) HAL_GPIO_WritePin(uc1701->CD_port, uc1701->CD_pin, mode)
#define uc1701Set_CS_level(level) HAL_GPIO_WritePin(uc1701->CS_port, uc1701->CS_pin, level)
#define uc1701Set_RESET(level) HAL_GPIO_WritePin(uc1701->RESET_port, uc1701->RESET_pin, level)

typedef enum __Font_length
{
	FONT_UVLO_LENGTH = 0u,
	FONT_OCP_LENGTH,
	FONT_OVP_LENGTH,
	FONT_SCP_LENGTH
}Font_length;

typedef enum __Type_param
{
	TYPE_VOLT = 0u,
	TYPE_AMPE,
	TYPE_WATT
}Type_param;

typedef struct __UC1701_HandleTypedef
{
	GPIO_TypeDef *CS_port;		// Chip select port
	unsigned short CS_pin;		// Chip select pin
	GPIO_TypeDef *CD_port;		// Command/Data port
	unsigned short CD_pin;		// Command/Data pin
	GPIO_TypeDef *RESET_port;	// Reset port
	unsigned short RESET_pin;	// Reset pin
	SPI_HandleTypeDef *p_spi;	// SPI
}UC1701_HandleTypedef;


void uc1701Init(UC1701_HandleTypedef *uctype);
void uc1701Fill(uint8_t ucPattern);
void uc1701SetContrast(uint8_t ucContrast);
void uc1701SetPosition(int x, int y);
void uc1701Write_ascii();
void uc1701Write_warning(uint8_t *font, uint32_t fontlength);
void uc1701Write_ADC_value(uint32_t *ADC_value);
void uc1701Write_skeleton();
void uc1701Write_setup_value(float uvlo, float vout, float ocp);
void uc1701Write_string(char *str, uint32_t strlen);
float uc1701_map(uint32_t x, uint32_t fromLow, uint32_t fromHigh, float toLow, float toHigh);
