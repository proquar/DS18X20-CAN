/*****************************************************************************
 *   Copyright (C) 2010 by Christian Groeger                                 *
 *   code@proquari.at                                                        *
 *                                                                           *
 *   This program is free software: you can redistribute it and/or modify    *
 *   it under the terms of the GNU General Public License as published by    *
 *   the Free Software Foundation, either version 3 of the License, or       *
 *   (at your option) any later version.                                     *
 *                                                                           *
 *   This program is distributed in the hope that it will be useful,         *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *   GNU General Public License for more details.                            *
 *                                                                           *
 *   You should have received a copy of the GNU General Public License       *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.   *
 *                                                                           *
 *****************************************************************************/


#include <avr/io.h>
#include <util/delay.h>
#include "can.h"

#include "onewire.h"
#include "ds18x20.h"

#define MAXSENSORS	20

#define OW2_DDR		DDRB
#define OW2_PORT	PORTB
#define OW2_PIN		PINB
#define OW2_BIT		PB0

#define OW1_DDR		DDRB
#define OW1_PORT	PORTB
#define OW1_PIN		PINB
#define OW1_BIT		PB7

#define OW0_DDR		DDRD
#define OW0_PORT	PORTD
#define OW0_PIN		PIND
#define OW0_BIT		PD4

#define MYID		0x20a0



typedef struct {
	uint8_t bus;
	uint8_t id[OW_ROMCODE_SIZE];
	int16_t temperature;
} ow_device;

ow_device sensors[MAXSENSORS];

/* If you want to receive both 11 and 29 bit identifiers, set your filters
 * and masks as follows:
 */
prog_uint8_t can_filter[] =
{
        // Group 0
        MCP2515_FILTER(0),                              // Filter 0
        MCP2515_FILTER(0),                              // Filter 1

        // Group 1
        MCP2515_FILTER_EXTENDED(0),             // Filter 2
        MCP2515_FILTER_EXTENDED(0),             // Filter 3
        MCP2515_FILTER_EXTENDED(0),             // Filter 4
        MCP2515_FILTER_EXTENDED(0),             // Filter 5

        MCP2515_FILTER(0),                              // Mask 0 (for group 0)
        MCP2515_FILTER_EXTENDED(0),             // Mask 1 (for group 1)
};

uint8_t can_ow_msg(can_t *msg, ow_device *sensor) {
	msg->id = 0x2020;
	msg->flags.rtr = 0;
	msg->flags.extended = 1;
	
	msg->length = 8;
	
	uint8_t i;
	for(i=1;i<=6;i++) {
		msg->data[i-1]=sensor->id[i];
	}
	msg->data[6]=sensor->temperature>>8;
	msg->data[7]=sensor->temperature;
	
	return can_send_message(msg);
}

uint8_t can_debug1(can_t *msg, const char *send, uint8_t num, uint8_t len) {
	msg->id = MYID;
	msg->flags.rtr = 0;
	msg->flags.extended = 1;
	
	msg->length = len+1;
	
	uint8_t i;
	for(i=0; i<8; i++) {
		if (i<len)			msg->data[i]=send[i];
		else if (i==len)	msg->data[i]=num+'0';
		else 				msg->data[i]=0;
	}
	
	return can_send_message(msg);
}

uint8_t can_debug0(can_t *msg, const char *send, uint8_t len) {
	msg->id = MYID;
	msg->flags.rtr = 0;
	msg->flags.extended = 1;
	
	msg->length = len;
	
	uint8_t i;
	for(i=0; i<8; i++) {
		if (i<len)			msg->data[i]=send[i];
		else 				msg->data[i]=0;
	}
	
	return can_send_message(msg);
}

void choose_bus(uint8_t num) {
	if (num==0) ow_set_bus(&OW0_PIN, &OW0_PORT, &OW0_DDR, OW0_BIT);
	else if (num==1) ow_set_bus(&OW1_PIN, &OW1_PORT, &OW1_DDR, OW1_BIT);
	else ow_set_bus(&OW2_PIN, &OW2_PORT, &OW2_DDR, OW2_BIT);
}

int main(void) {
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff;
	uint8_t sensor_number=0;
	uint8_t i,j;
	
	can_t msg;
	
	can_init(BITRATE_125_KBPS);
	
	can_static_filter(can_filter);
	
	can_debug0(&msg, "start", 5);
	
	//scan 1-wire busses
	for (i=0; i<3; i++) {
		choose_bus(i);
		for( diff = OW_SEARCH_FIRST; diff != OW_LAST_DEVICE && sensor_number < MAXSENSORS ; ) {
			DS18X20_find_sensor( &diff, id );
			
			if( diff == OW_PRESENCE_ERR ) {
				can_debug1(&msg, "PE ", i, 3);
				break;
			}
			if( diff == OW_DATA_ERR ) {
				can_debug1(&msg, "DE ", i, 3);
				break;
			}
			for (j=0;j<OW_ROMCODE_SIZE;j++)
				sensors[sensor_number].id[j]=id[j];
			sensors[sensor_number].bus=i;
			sensor_number++;
		}
	}
	
	can_debug1(&msg, "found ", sensor_number, 6);
	
	// measure, wait and read sensors
	for(;;) {
		uint8_t dowait=0;
		for(j=0; j<3; j++) {
			
			choose_bus(j);
			if (DS18X20_start_meas(DS18X20_POWER_PARASITE, NULL) == DS18X20_OK) dowait=1;
		}
		if (dowait) {
			_delay_ms(DS18B20_TCONV_12BIT);
			
			for(i=0; i<sensor_number; i++) {
				choose_bus(sensors[i].bus);
				if ( DS18X20_read_meas( sensors[i].id, &sensors[i].temperature) == DS18X20_OK ) {
					can_ow_msg(&msg, &sensors[i]);
				}
			}
		}
		_delay_ms(1000);
	}
	
	return 0;
}
