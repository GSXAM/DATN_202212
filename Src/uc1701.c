// uc1701 LCD library
// Written by Larry Bank (bitbank@pobox.com)
// Project started 2/22/2018
//
// The uc1701 LCD display is controlled through the SPI interface
// and three GPIO pins to control the RST (reset), D/C (data/command),
// and LED backlight control lines. 
// The LCD controller is set to "horizontal mode". This divides the display
// into 8 128x8 "pages" or strips. Each data write advances the output
// automatically to the next address. The bytes are arranged such that the LSB
// is the topmost pixel and the MSB is the bottom.
// A copy of the display memory is maintained by this code so that single pixel
// writes can occur without having to read from the display controller.
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
//
#include "uc1701.h"

const uint8_t uc1701_font_UVLO[] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFE, 0xF0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xF0, 0xFE, 0xFF, 0xFF, 0xFF, 0x7F, 0x0F, 0x01, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0x7E, 0x7E, 0x3F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x3F, 0x3E, 0x7E, 0xFC, 0xFC, 0xF8, 0xF0, 0xE0, 0xC0, 0x00, 0x00,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFC, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xFC, 0xFF, 0xFF, 0xFF, 0x7F, 0x1F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8,
	0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0xC0, 0x00, 0xC0, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x0F,
	0x00, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x3F, 0x3F, 0x7E, 0x7E, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7E, 0x7F, 0x3F, 0x3F, 0x1F, 0x1F, 0x0F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x1F, 0x7F, 0x7F, 0x7F, 0x7E, 0x7F, 0x7F, 0x7F, 0x3F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x3F, 0x3F, 0x7E, 0x7E, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7E, 0x7E, 0x3F, 0x3F, 0x3F, 0x1F, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00
};
const uint8_t uc1701_font_SCP[] = {
	0xC0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFE, 0x3E, 0x3F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x3F, 0x3F, 0x7E, 0xFE, 0xFE, 0xFC, 0xF8, 0xF0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFC, 0x7E, 0x3E, 0x3F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x3F, 0x3F, 0x7E, 0xFE, 0xFC, 0xFC, 0xF8, 0xF0, 0xE0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x3F, 0x3F, 0x7F, 0xFE, 0xFE, 0xFC, 0xF8, 0xF0, 0xC0,
	0x07, 0x1F, 0x3F, 0x7F, 0x7F, 0xFF, 0xFE, 0xFC, 0xFC, 0xF8, 0xF8, 0xF8, 0xF8, 0xF0, 0xF0, 0xF0, 0xE0, 0xE1, 0xE1, 0xC1, 0xC1, 0x81, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xE0, 0xE0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x1F,
	0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xC0, 0x00, 0x00, 0x01, 0x01, 0x01, 0x03, 0x03, 0x03, 0x03, 0x07, 0x07, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xE0, 0xF8, 0xF8, 0xF0, 0xF0, 0xF0, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x03, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00,
	0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x3F, 0x3F, 0x7E, 0x7E, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7E, 0x3E, 0x3F, 0x3F, 0x1F, 0x1F, 0x0F, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x3F, 0x3E, 0x7E, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7E, 0x7E, 0x3F, 0x3F, 0x3F, 0x1F, 0x0F, 0x0F, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const uint8_t uc1701_font_OVP[] = {
	0x00, 0x00, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0x7E, 0x7E, 0x3F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x3F, 0x3E, 0x7E, 0xFC, 0xFC, 0xF8, 0xF0, 0xE0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFE, 0xF0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xF0, 0xFE, 0xFF, 0xFF, 0xFF, 0x7F, 0x0F, 0x01, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x3F, 0x3F, 0x7F, 0xFE, 0xFE, 0xFC, 0xF8, 0xF0, 0xC0,
	0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFC, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xFC, 0xFF, 0xFF, 0xFF, 0x7F, 0x1F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xE0, 0xE0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x1F,
	0x0F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0xC0, 0x00, 0xC0, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x03, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x3F, 0x3F, 0x7E, 0x7E, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7E, 0x7E, 0x3F, 0x3F, 0x3F, 0x1F, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x1F, 0x7F, 0x7F, 0x7F, 0x7E, 0x7F, 0x7F, 0x7F, 0x3F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const uint8_t uc1701_font_OCP[] = {
	0x00, 0x00, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0x7E, 0x7E, 0x3F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x3F, 0x3E, 0x7E, 0xFC, 0xFC, 0xF8, 0xF0, 0xE0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFC, 0x7E, 0x3E, 0x3F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x3F, 0x3F, 0x7E, 0xFE, 0xFC, 0xFC, 0xF8, 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x3F, 0x3F, 0x7F, 0xFE, 0xFE, 0xFC, 0xF8, 0xF0, 0xC0,
	0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xE0, 0xE0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x1F,
	0x0F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xE0, 0xF8, 0xF8, 0xF0, 0xF0, 0xF0, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x03, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x3F, 0x3F, 0x7E, 0x7E, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7E, 0x7E, 0x3F, 0x3F, 0x3F, 0x1F, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x3F, 0x3E, 0x7E, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7E, 0x7E, 0x3F, 0x3F, 0x3F, 0x1F, 0x0F, 0x0F, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const uint8_t V_A_W_pos[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1F, 0xFF, 0xFC, 0xE0, 0x00, 0x00, 0x00, 0xE0, 0xFC, 0xFF, 0x1F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x70, 0x70, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x3F, 0x7F, 0x78, 0x7F, 0x3F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xF8, 0xFF, 0x3F, 0x07, 0x3F, 0xFF, 0xF8, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x70, 0x70, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x7C, 0x7F, 0x1F, 0x0F, 0x0E, 0x0E, 0x0E, 0x0F, 0x1F, 0x7F, 0x7C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0xFF, 0xFC, 0xC0, 0x00, 0xC0, 0xFC, 0xFF, 0x0F, 0xFF, 0xFC, 0xC0, 0x00, 0xC0, 0xFC, 0xFF, 0x3F, 0x03,
    0x70, 0x70, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x7F, 0x7F, 0x7C, 0x7F, 0x1F, 0x00, 0x00, 0x00, 0x1F, 0x7F, 0x7C, 0x7F, 0x7F, 0x07, 0x00, 0x00
};
const uint8_t uc1701_number_16x11[][22] = {
    {0xF8, 0xFE, 0xFF, 0x07, 0x03, 0x07, 0xFF, 0xFE, 0xF8, 0x00, 0x00, 0x0F, 0x3F, 0x7F, 0x70, 0x60, 0x70, 0x7F, 0x3F, 0x0F, 0x00, 0x00}, // 0
    {0x70, 0x38, 0x38, 0x1C, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x7F, 0x7F, 0x00, 0x00, 0x00, 0x00}, // 1
    {0x0C, 0x0E, 0x0F, 0x03, 0x83, 0xC3, 0xFF, 0xFE, 0x3C, 0x00, 0x00, 0x60, 0x78, 0x7C, 0x7F, 0x6F, 0x63, 0x61, 0x60, 0x60, 0x00, 0x00}, // 2
    {0x0C, 0x0E, 0x0F, 0xC3, 0xC3, 0xE3, 0xFF, 0xFE, 0x3C, 0x00, 0x00, 0x18, 0x38, 0x78, 0x60, 0x60, 0x61, 0x7F, 0x3F, 0x1F, 0x00, 0x00}, // 3
    {0x00, 0x00, 0xC0, 0xE0, 0x38, 0x1E, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x0E, 0x0F, 0x0D, 0x0C, 0x0C, 0x0C, 0x7F, 0x7F, 0x7F, 0x0C, 0x00}, // 4
    {0xE0, 0xFC, 0xFF, 0x7F, 0x63, 0xE3, 0xE3, 0xC3, 0x83, 0x00, 0x00, 0x18, 0x38, 0x78, 0x70, 0x60, 0x70, 0x7F, 0x3F, 0x1F, 0x0E, 0x00}, // 5
    {0xF8, 0xFE, 0xFE, 0xC7, 0x63, 0xE7, 0xEF, 0xCE, 0x84, 0x00, 0x00, 0x0F, 0x3F, 0x3F, 0x70, 0x60, 0x70, 0x3F, 0x3F, 0x0F, 0x06, 0x00}, // 6
    {0x03, 0x03, 0x03, 0xC3, 0xF3, 0xFB, 0x3F, 0x0F, 0x07, 0x03, 0x00, 0x00, 0x78, 0x7F, 0x7F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // 7
    {0x1C, 0x3E, 0xFF, 0xE3, 0xC3, 0xE3, 0xFF, 0x3E, 0x1C, 0x00, 0x00, 0x1E, 0x3F, 0x7F, 0x71, 0x60, 0x71, 0x7F, 0x3F, 0x1E, 0x00, 0x00}, // 8
    {0xF8, 0xFE, 0xFE, 0x87, 0x03, 0x87, 0xFE, 0xFE, 0xF8, 0x00, 0x00, 0x10, 0x39, 0x7B, 0x73, 0x63, 0x71, 0x3F, 0x3F, 0x0F, 0x00, 0x00}, // 9
};
const uint8_t uc1701_font_6x8[][6] = {	  // offset = 32, step +1
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // Space
	{0x00, 0x00, 0x5f, 0x00, 0x00, 0x00}, // !
	{0x00, 0x07, 0x00, 0x07, 0x00, 0x00}, // "
	{0x14, 0x7f, 0x14, 0x7f, 0x14, 0x00}, // #
	{0x24, 0x2a, 0x7f, 0x2a, 0x12, 0x00}, // $
	{0x23, 0x13, 0x08, 0x64, 0x62, 0x00}, // %
	{0x36, 0x49, 0x55, 0x22, 0x50, 0x00}, // &
	{0x00, 0x00, 0x07, 0x00, 0x00, 0x00}, // '
	{0x00, 0x1c, 0x22, 0x41, 0x00, 0x00}, // (
	{0x00, 0x41, 0x22, 0x1c, 0x00, 0x00}, // )
	{0x14, 0x08, 0x3e, 0x08, 0x14, 0x00}, // *
	{0x10, 0x10, 0x7c, 0x10, 0x10, 0x00}, // +
	{0x00, 0x50, 0x30, 0x00, 0x00, 0x00}, // ,
	{0x10, 0x10, 0x10, 0x10, 0x10, 0x00}, // -
	{0x00, 0x60, 0x60, 0x00, 0x00, 0x00}, // .
	{0x00, 0x60, 0x18, 0x06, 0x01, 0x00}, // /
	{0x3e, 0x51, 0x49, 0x45, 0x3e, 0x00}, // 0
	{0x44, 0x42, 0x7f, 0x40, 0x40, 0x00}, // 1
	{0x42, 0x61, 0x51, 0x49, 0x46, 0x00}, // 2
	{0x22, 0x41, 0x49, 0x49, 0x36, 0x00}, // 3
	{0x18, 0x14, 0x12, 0x7f, 0x10, 0x00}, // 4
	{0x27, 0x45, 0x45, 0x45, 0x39, 0x00}, // 5
	{0x3c, 0x4a, 0x49, 0x49, 0x30, 0x00}, // 6
	{0x61, 0x11, 0x09, 0x05, 0x03, 0x00}, // 7
	{0x36, 0x49, 0x49, 0x49, 0x36, 0x00}, // 8
	{0x26, 0x49, 0x49, 0x49, 0x3e, 0x00}, // 9
	{0x66, 0x66, 0x00, 0x00, 0x00, 0x00}, // :
	{0x56, 0x36, 0x00, 0x00, 0x00, 0x00}, // ;
	{0x00, 0x08, 0x14, 0x22, 0x00, 0x00}, // <
	{0x00, 0x28, 0x28, 0x28, 0x00, 0x00}, // =
	{0x00, 0x22, 0x14, 0x08, 0x00, 0x00}, // >
	{0x02, 0x01, 0x51, 0x09, 0x06, 0x00}, // ?
	{0x32, 0x49, 0x79, 0x41, 0x3e, 0x00}, // @
	{0x7e, 0x11, 0x11, 0x11, 0x7e, 0x00}, // A
	{0x7f, 0x49, 0x49, 0x49, 0x36, 0x00}, // B
	{0x3e, 0x41, 0x41, 0x41, 0x22, 0x00}, // C
	{0x7f, 0x41, 0x41, 0x41, 0x3e, 0x00}, // D
	{0x7f, 0x49, 0x49, 0x49, 0x41, 0x00}, // E
	{0x7f, 0x09, 0x09, 0x09, 0x01, 0x00}, // F
	{0x3e, 0x41, 0x49, 0x49, 0x3a, 0x00}, // G
	{0x7f, 0x08, 0x08, 0x08, 0x7f, 0x00}, // H
	{0x41, 0x41, 0x7f, 0x41, 0x41, 0x00}, // I
	{0x20, 0x40, 0x41, 0x3f, 0x01, 0x00}, // J
	{0x7f, 0x08, 0x14, 0x22, 0x41, 0x00}, // K
	{0x7f, 0x40, 0x40, 0x40, 0x40, 0x00}, // L
	{0x7f, 0x02, 0x0c, 0x02, 0x7f, 0x00}, // M
	{0x7f, 0x04, 0x08, 0x10, 0x7f, 0x00}, // N
	{0x3e, 0x41, 0x41, 0x41, 0x3e, 0x00}, // O
	{0x7e, 0x09, 0x09, 0x09, 0x06, 0x00}, // P
	{0x3e, 0x41, 0x51, 0x21, 0x5e, 0x00}, // Q
	{0x7f, 0x09, 0x19, 0x29, 0x46, 0x00}, // R
	{0x26, 0x49, 0x49, 0x49, 0x32, 0x00}, // S
	{0x01, 0x01, 0x7f, 0x01, 0x01, 0x00}, // T
	{0x3f, 0x40, 0x40, 0x40, 0x3f, 0x00}, // U
	{0x0f, 0x30, 0x40, 0x30, 0x0f, 0x00}, // V
	{0x3f, 0x40, 0x30, 0x40, 0x3f, 0x00}, // W
	{0x63, 0x14, 0x08, 0x14, 0x63, 0x00}, // X
	{0x03, 0x04, 0x78, 0x04, 0x03, 0x00}, // Y
	{0x61, 0x51, 0x49, 0x45, 0x43, 0x00}, // Z
	{0x00, 0x7f, 0x41, 0x41, 0x00, 0x00}, // [
	{0x01, 0x06, 0x18, 0x60, 0x00, 0x00}, // "\"
	{0x00, 0x41, 0x41, 0x7f, 0x00, 0x00}, // ]
	{0x04, 0x02, 0x01, 0x02, 0x04, 0x00}, // ^
	{0x40, 0x40, 0x40, 0x40, 0x40, 0x00}, // _
	{0x00, 0x01, 0x02, 0x00, 0x00, 0x00}, // `
	{0x20, 0x54, 0x54, 0x54, 0x78, 0x00}, // a
	{0x7f, 0x48, 0x48, 0x48, 0x30, 0x00}, // b
	{0x38, 0x44, 0x44, 0x44, 0x20, 0x00}, // c
	{0x30, 0x48, 0x48, 0x48, 0x7f, 0x00}, // d
	{0x38, 0x54, 0x54, 0x54, 0x18, 0x00}, // e
	{0x08, 0x7e, 0x09, 0x01, 0x02, 0x00}, // f
	{0x08, 0x54, 0x54, 0x54, 0x3c, 0x00}, // g
	{0x7f, 0x10, 0x08, 0x08, 0x70, 0x00}, // h
	{0x44, 0x44, 0x7d, 0x40, 0x40, 0x00}, // i
	{0x20, 0x40, 0x44, 0x3d, 0x00, 0x00}, // j
	{0x7f, 0x10, 0x28, 0x44, 0x00, 0x00}, // k
	{0x00, 0x41, 0x7f, 0x40, 0x00, 0x00}, // l
	{0x7c, 0x04, 0x78, 0x04, 0x78, 0x00}, // m
	{0x7c, 0x08, 0x04, 0x04, 0x78, 0x00}, // n
	{0x38, 0x44, 0x44, 0x44, 0x38, 0x00}, // o
	{0x7c, 0x14, 0x14, 0x14, 0x08, 0x00}, // p
	{0x08, 0x14, 0x14, 0x14, 0x7c, 0x00}, // q
	{0x7c, 0x08, 0x04, 0x04, 0x08, 0x00}, // r
	{0x48, 0x54, 0x54, 0x54, 0x20, 0x00}, // s
	{0x04, 0x3e, 0x44, 0x40, 0x20, 0x00}, // t
	{0x3c, 0x40, 0x40, 0x20, 0x7c, 0x00}, // u
	{0x1c, 0x20, 0x40, 0x20, 0x1c, 0x00}, // v
	{0x3c, 0x40, 0x30, 0x40, 0x3c, 0x00}, // w
	{0x44, 0x28, 0x10, 0x28, 0x44, 0x00}, // x
	{0x0c, 0x50, 0x50, 0x50, 0x3c, 0x00}, // y
	{0x44, 0x64, 0x54, 0x4c, 0x44, 0x00}, // z
	{0x00, 0x41, 0x36, 0x08, 0x00, 0x00}, // }
	{0x00, 0x00, 0x7f, 0x00, 0x00, 0x00}, // |
	{0x00, 0x08, 0x36, 0x41, 0x00, 0x00}, // {
	{0x10, 0x08, 0x08, 0x10, 0x08, 0x00} // ~
};



static UC1701_HandleTypedef *uc1701;
static SPI_HandleTypeDef *ucspi;

static void uc1701SPI_send(uint8_t *value, int size);
static void uc1701WriteDataBlock(uint8_t *ucBuf, int iLen);
static void uc1701Write_number(uint32_t number, int x, int y);
static void uc1701Write_float_number(float number, uint32_t type);


/*****************************************************************************/
/*                              Private functions                            */
/*****************************************************************************/
/** @brief SPI transmit data only.
 *  @param value: pointer of data.
 *  @param size: how many byte of data need to be sent.
 *  @retval None
 */
static void uc1701SPI_send(uint8_t *value, int size)
{
	uc1701Set_CS_level(GPIO_PIN_RESET);
	HAL_SPI_Transmit(ucspi, value, size, 100);
	uc1701Set_CS_level(GPIO_PIN_SET);
}

/** @brief Write a block of data to LCD.
 *  @param ucBuf: pointer of data.
 *  @param ilen: how many byte of data need to be sent.
 *  @retval None
 */
static void uc1701WriteDataBlock(uint8_t *ucBuf, int iLen)
{
	uc1701Set_CD_mode(MODE_DATA);
	uc1701SPI_send(ucBuf, iLen);
}

/** @brief Write one character number in uc1701_number_16x11 array to LCD.
 *  @param number: input number.
 *  @param x: column position.
 *  @param y: row position.
 *  @retval None
 */
static void uc1701Write_number(uint32_t number, int x, int y)
{
	uint8_t *font = (uint8_t*)uc1701_number_16x11;
	uc1701SetPosition(x, y);
	uc1701WriteDataBlock(font + number*22 /*pointer to first line of number*/, 11  /*width of number*/);
	uc1701SetPosition(x, y+1);
	uc1701WriteDataBlock(font + number*22 + 11 /*pointer of second line of number*/, 11  /*width of number*/);	
}

/** @brief Write float number to lCD
 * 	@param number: float value
 * 	@param type: type of parameter in Type_param typedef
 * 	@retval None.
 */
static void uc1701Write_float_number(float number, uint32_t type)
{
	/* floor(): Returns the largest integer not greater than x, expressed as a double */
	/* round(): Returns the largest integer greater than x, expressed as a double */
	uint32_t i_ten = (uint32_t)floor(number/10);
	uint32_t i_unit = (uint32_t)(floor(number) - i_ten*10);
	uint32_t f_ten = (uint32_t)(floor(number*10) - i_ten*100 - i_unit*10);
	uint32_t f_unit = (uint32_t)(round(number*100) - i_ten*1000 - i_unit*100 - f_ten*10);
	/* Check number in range [0:9], make sure the number will become
	 * a right pointer in uc1701_number_16x11 array characters
	 * --> this help CPU do not refer to wrong address and
	 * HardFault_Handler() happen.
	 */
	if(i_ten>9) i_ten = 0;
	if(i_unit>9) i_unit = 0;
	if(f_ten>9) f_ten = 0;
	if(f_unit>9) f_unit = 1;
	
	switch (type)
	{
		case TYPE_VOLT:
			// integer: position (x,y) = (0,0) --> page 0
			uc1701Write_number(i_ten, 0, 0); // i_ten (x,y) = (0,0)
			uc1701Write_number(i_unit, 11, 0); // i_unit (x,y) = (0+11,0)
			// float: position (x,y) = (28,0)
			uc1701Write_number(f_ten, 28, 0); // f_ten (x,y) = (28,0)
			uc1701Write_number(f_unit, 39, 0); // f_unit (x,y) = (28+11,0)
			break;
		case TYPE_AMPE:
			// integer: position (x,y) = (0,24) --> page 3
			uc1701Write_number(i_ten, 0, 3); // i_ten (x,y) = (0,24)
			uc1701Write_number(i_unit, 11, 3); // i_unit (x,y) = (0+11,24)
			// float: position (x,y) = (28,24)
			uc1701Write_number(f_ten, 28, 3); // f_ten (x,y) = (28,24)
			uc1701Write_number(f_unit, 39, 3); // f_unit (x,y) = (28+11,24)
			break;
		case TYPE_WATT:
			// integer: position (x,y) = (0,48) --> page 6
			uc1701Write_number(i_ten, 0, 6); // i_ten (x,y) = (0,48)
			uc1701Write_number(i_unit, 11, 6); // i_unit (x,y) = (0+11,48)
			// float: position (x,y) = (28,48)
			uc1701Write_number(f_ten, 28,6); // f_ten (x,y) = (28,48)
			uc1701Write_number(f_unit, 39,6); // f_unit (x,y) = (28+11,48)
			break;
		default:
			break;
	}
}


/*****************************************************************************/
/*                              Public functions                             */
/*****************************************************************************/
/** @brief Set LCD cursor
 * 	@param x: horizontal position
 * 	@param y: vertical position
 * 	@retval None.
 */
void uc1701SetPosition(int x, int y)
{
	uc1701Set_CD_mode(MODE_COMMAND);
	uint8_t cmds[] = {0xb0 | y,			// set Y
							0x10 | (x >> 4),	// set X (high MSB)
							0x00 | (x & 0xf)};	// set X (low MSB)
	uc1701SPI_send(cmds, sizeof(cmds));
}

/** @brief Write all characters in uc1701_font_6x8.
 * 	@retval None.
 */
void uc1701Write_ascii()
{
	int yCount = 0;
	int xCount = 0;
	uc1701SetPosition(0, yCount);
	for (int i = 0; i < 96; i++)
	{
		if(xCount >= 21){
			xCount = 0;
			yCount++;
			uc1701SetPosition(0, yCount);
			uc1701WriteDataBlock((uint8_t*)uc1701_font_6x8[i], 6);
			xCount++;
		}
		else{
			uc1701WriteDataBlock((uint8_t*)uc1701_font_6x8[i], 6);
			xCount++;
		}
	}
}

/** @brief Adjust the constrast of the LCD display
 *  @param ucConstrast: value range: 0~63
 *  @retval None.
 */
void uc1701SetContrast(uint8_t ucContrast)
{
	uc1701Set_CD_mode(MODE_COMMAND);
  uint8_t cmds[] = {0x81, ucContrast};
	uc1701SPI_send(cmds, sizeof(cmds));
}

/** @brief Fill all pixels of the LCD display.
 *  @param ucData: value in uint8_t type.
 *  @retval None.
 */
void uc1701Fill(uint8_t ucData)
{
	uint8_t temp[128];			 // 128 columns
	memset(temp, ucData, 128);
	for (int y=0; y<8; y++)					 // 8 Pages
	{
		uc1701SetPosition(0,y);			 // set to (0,Y)
		uc1701WriteDataBlock(temp, 128); // fill with data byte
	}
}

/** @brief Initializes the LCD controller into "horizontal write mode",
 * 		   Prepares the font data for the orientation of the display.
 *  @param p_uc1701: pointer to a UC1701_HandleTypedef structure that contains
 * 				configuration infomation for UC1701.
 *  @retval None
 */
void uc1701Init(UC1701_HandleTypedef *p_uc1701)
{
	/**	@attention Declare UC1701 handler structure in main.c
UC1701_HandleTypedef UC1701;
	 *  @attention Define configure function in main.c, then call it in main()
static void UC1701_config()
{
	UC1701.CS_port = CS_GPIO_Port;
	UC1701.CS_pin = CS_Pin;
	UC1701.CD_port = CD_GPIO_Port;
	UC1701.CD_pin = CD_Pin;
	UC1701.RESET_port = RESET_GPIO_Port;
	UC1701.RESET_pin = RESET_Pin;
	UC1701.p_spi = &hspi1;
}
	 *  @attention Therefore, call uc1701Init() function in main()
	 */

	// Assign pointer to working pointer
	ucspi = p_uc1701->p_spi;
	uc1701 = p_uc1701;

	// Start by reseting the LCD controller (Hardware reset)
	uc1701Set_RESET(GPIO_PIN_SET);		// Make sure RESET pin at HIGH before reset LCD. Delay at least 3ms.
	HAL_Delay(50);
	uc1701Set_RESET(GPIO_PIN_RESET);	// After pull RESET pin LOW, wait at least 1us.
	HAL_Delay(5);
	uc1701Set_RESET(GPIO_PIN_SET);		// Take it out of reset.
	HAL_Delay(5);
	ucspi->Instance->CR1 |= 0x40;		// Enable SPI peripheral

	uint8_t init_cmds[] = {
		0xe2, // (15.)system reset
		0xae, // (12.)display off
		0x40, // (6.)set LCD start line to 0
		0xa0, // (13.)set SEG direction ('a1-a0' to flip horizontal)
		0xc0, // (14.)set COM direction ('c0-c8' to flip vert)
		0xa2, // (17.)set LCD bias ratio 1/9
		0x2f, // (5.)all power control circuits on
		0xf8, 0x00, // (22.)set booster ratio to 4x
		0x23, // (8.)set resistor ratio = 4 (default = 100b)
		0xfa, 0xb0, // (25.)Default value, but Frame rate is max (0b11)
		0xac, // (20.)set static indicator off
		0xa6, // (11.)disable inverse
		0xaf, // (12.)enable display
		0xa5, // (10.)Set all pixel OFF
		0x81, 0x32, // (9.)set contrast = 50
	};
	uc1701Set_CD_mode(MODE_COMMAND);
	uc1701SPI_send(init_cmds, sizeof(init_cmds));
	HAL_Delay(100);
	uint8_t nordis = 0xa4;
	uc1701SPI_send(&nordis, 1); // (10.)normal display
	HAL_Delay(50);
	uc1701Fill(0x00); // erase memory
}
/** @brief Write warning status to LCD
 * 	@param font: warning message
 * 	@param fontlength: length of font defined in Front_length typedef
 * 	@retval None.
 */
void uc1701Write_warning(uint8_t *font, uint32_t fontlength)
{
	int x = 20; /*column*/
	int y = 2; /*row*/
	switch (fontlength)
	{
	case FONT_UVLO_LENGTH:
		fontlength = (sizeof(uc1701_font_UVLO)/4);
		x = 6;
		break;
	case FONT_OCP_LENGTH:
		fontlength = (sizeof(uc1701_font_OCP)/4);
		break;
	case FONT_OVP_LENGTH:
		fontlength = (sizeof(uc1701_font_OVP)/4);
		break;
	case FONT_SCP_LENGTH:
		fontlength = (sizeof(uc1701_font_SCP)/4);
		break;
	default:
		break;
	}
	
	for(int i = y; i<(y+4); i++)
	{
		uc1701SetPosition(x,i);
		uc1701WriteDataBlock(font, fontlength);
		font += fontlength;
	}
}

/** @brief Write V-A-W, UVLO, VOUT, OCP form
 * 	@retval None.
 */
void uc1701Write_skeleton()
{
	/* Write setup value title */
	// write UVLO:
	uc1701SetPosition(98,0); // --> page 0
	uc1701Write_string("UVLO:",5);
	// write VOUT:
	uc1701SetPosition(98,3); // --> page 3
	uc1701Write_string("VOUT:",5);
	// write OCP:
	uc1701SetPosition(104,6); // --> page 6
	uc1701Write_string("OCP:",4);
	
	/* Write dot(.), V - A - W */
	uint8_t *font = (uint8_t*)V_A_W_pos;
	int len = sizeof(V_A_W_pos)/8;
	for(int i = 0; i<8; i++)
	{
		uc1701SetPosition(22,i);
		uc1701WriteDataBlock(font, len);
		font += len;
	}
}

/** @brief Write setup value to LCD
 * 	@param uvlo: uvlo_flash value
 * 	@param vout: vout_flash value
 * 	@param ocp: ocp_flash value
 * 	@retval None.
 */
void uc1701Write_setup_value(float uvlo, float vout, float ocp)
{
	char buff[6];
	/* Write UVLO setup value */
	sprintf(buff, "%0.2f", uvlo);
	strcat(buff, "V");
	uc1701SetPosition(92, 1); // --> page 1
	uc1701Write_string(buff, strlen(buff));
	/* Write VOUT setup value */
	sprintf(buff, "%0.2f", vout);
	strcat(buff, "V");
	uc1701SetPosition(92, 4); // --> page 4
	uc1701Write_string(buff, strlen(buff));
	/* Write OCP setup value */
	sprintf(buff, "%0.2f", ocp);
	strcat(buff, "A");
	uc1701SetPosition(92, 7); // --> page 7
	uc1701Write_string(buff, strlen(buff));
}

/** @brief Write ADC measured value to LCD
 * 	@param ADC_value: pointer to array of ADC measured values.
 * 	@retval None.
 */
void uc1701Write_ADC_value(uint32_t *ADC_value)
{
	// compute VOUT in range [0:10V]
	float volt =  fabs(uc1701_map(*(ADC_value+1), 0, 4095, 0, 14.2));
	// compute CURRENT in range [0:10A]
	float amp = fabs(uc1701_map(*(ADC_value+2), 1140, 4095, 0, 25.2));
	// compute WATT
	float watt = volt*amp; 
	
	uc1701Write_float_number(volt, TYPE_VOLT);
	uc1701Write_float_number(amp, TYPE_AMPE);
	uc1701Write_float_number(watt, TYPE_WATT);
}

/** @brief Write string in font 6x8 to LCD
 * 	@param str: pointer to string
 *  @param strlen: length of string
 * 	@retval None.
 */
void uc1701Write_string(char *str, uint32_t strlen)
{
	for(char *i = str; i < (str + strlen); i++)
	{
		uc1701WriteDataBlock((uint8_t*)uc1701_font_6x8[*i - 32], 6);
	}
}

/** @brief Re-maps a number from one range to another.
 * 	@param value: the number to be converted.
 * 	@param fromLow: the lower bound of the values current range.
 *	@param fromHigh: the upper bound of the values current range.
 * 	@param toLow: the lower bound of the values target range.
 *	@param toHigh: the upper bound of the values target range.
 * 	@retval None.
 */
float uc1701_map(uint32_t value, uint32_t fromLow, uint32_t fromHigh, float toLow, float toHigh)
{
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}
