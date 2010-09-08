/*********************************************************************************
Title:    DS18X20-Functions via One-Wire-Bus
Author:   Martin Thomas <eversmith@heizung-thomas.de>   
          http://www.siwawi.arubi.uni-kl.de/avr-projects
          modified by Christian Groeger (09/2010)
Software: avr-gcc 3.4.1 / avr-libc 1.0.4 
Hardware: any AVR - tested with ATmega16/ATmega32 and 3 DS18B20

Partly based on code from Peter Dannegger and others

changelog:
20100909 - new DS18X20_meas_format() by Christian Groeger <code@proquari.at>
20041124 - Extended measurements for DS18(S)20 contributed by Carsten Foss (CFO)
200502xx - function DS18X20_read_meas_single
20050310 - DS18x20 EEPROM functions (can be disabled to save flash-memory)
           (DS18X20_EEPROMSUPPORT in ds18x20.h)

**********************************************************************************/

#include <avr/io.h>
#include <util/delay.h>

#include "ds18x20.h"
#include "onewire.h"
#include "crc8.h"

int16_t DS18X20_meas_format ( uint8_t fc, uint8_t *sp ) {
	// standard format is 16bit signed integer with fixed point precision of 6 bits
	// sign:1bit, value: 9 bits, precision: 6 bits 
	uint16_t meas;
	
	meas = sp[0];  // LSB
	meas |= ((uint16_t)sp[1])<<8; // MSB
	
	if( fc == DS18S20_ID ) { //9bit
		meas<<=5;
		
		// get up to 12bit precision from count_remain:
		meas += 0x30; // -0.25 +16/16
		meas -= (sp[6]<<2); // -count_remain/16
	}
	else if( fc == DS18B20_ID ) { //9bit
		if (sp[DS18B20_CONF_REG] & DS18B20_12_BIT)			meas<<=2;
		else if (sp[DS18B20_CONF_REG] & DS18B20_11_BIT)		meas<<=3;
		else if (sp[DS18B20_CONF_REG] & DS18B20_10_BIT)		meas<<=4;
		else if (sp[DS18B20_CONF_REG] & DS18B20_9_BIT)		meas<<=5;
		else meas=0xffff;
	}
	else {
		meas=0xffff;
	}
	
	return meas;
}

/* find DS18X20 Sensors on 1-Wire-Bus
   input/ouput: diff is the result of the last rom-search
   output: id is the rom-code of the sensor found */
void DS18X20_find_sensor(uint8_t *diff, uint8_t id[])
{
	for (;;) {
		*diff = ow_rom_search( *diff, &id[0] );
		if ( *diff==OW_PRESENCE_ERR || *diff==OW_DATA_ERR ||
		  *diff == OW_LAST_DEVICE ) return;
		if ( id[0] == DS18B20_ID || id[0] == DS18S20_ID ) return;
	}
}

/* get power status of DS18x20 
   input  : id = rom_code 
   returns: DS18X20_POWER_EXTERN or DS18X20_POWER_PARASITE */
uint8_t	DS18X20_get_power_status(uint8_t id[])
{
	uint8_t pstat;
    ow_reset();
    ow_command(DS18X20_READ_POWER_SUPPLY, id);
    pstat=ow_bit_io(1); // pstat 0=is parasite/ !=0 ext. powered
    ow_reset();
	return (pstat) ? DS18X20_POWER_EXTERN:DS18X20_POWER_PARASITE;
}

/* start measurement (CONVERT_T) for all sensors if input id==NULL 
   or for single sensor. then id is the rom-code */
uint8_t DS18X20_start_meas( uint8_t with_power_extern, uint8_t id[])
{
	ow_reset(); //**
	if( ow_input_pin_state() ) { // only send if bus is "idle" = high
		ow_command( DS18X20_CONVERT_T, id );
		if (with_power_extern != DS18X20_POWER_EXTERN)
			ow_parasite_enable();
		return DS18X20_OK;
	} 
	else { 
		#ifdef DS18X20_VERBOSE
		#endif
		return DS18X20_START_FAIL;
	}
}

/* reads temperature (scratchpad) of sensor with rom-code id
   output: subzero==1 if temp.<0, cel: full celsius, mcel: frac 
   in millicelsius*0.1
   i.e.: subzero=1, cel=18, millicel=5000 = -18,5000�C */
uint8_t DS18X20_read_meas(uint8_t id[], int16_t *temperature)
{
	uint8_t i;
	uint8_t sp[DS18X20_SP_SIZE];
	
	ow_reset(); //**
	ow_command(DS18X20_READ, id);
	for ( i=0 ; i< DS18X20_SP_SIZE; i++ ) sp[i]=ow_byte_rd();
	if ( crc8( &sp[0], DS18X20_SP_SIZE ) ) 
		return DS18X20_ERROR_CRC;
	*temperature = DS18X20_meas_format(id[0], sp);
	return DS18X20_OK;
}

/* reads temperature (scratchpad) of a single sensor (uses skip-rom)
   output: subzero==1 if temp.<0, cel: full celsius, mcel: frac 
   in millicelsius*0.1
   i.e.: subzero=1, cel=18, millicel=5000 = -18,5000�C */
uint8_t DS18X20_read_meas_single(uint8_t familycode, int16_t *temperature)
{
	uint8_t i;
	uint8_t sp[DS18X20_SP_SIZE];
	
	ow_command(DS18X20_READ, NULL);
	for ( i=0 ; i< DS18X20_SP_SIZE; i++ ) sp[i]=ow_byte_rd();
	if ( crc8( &sp[0], DS18X20_SP_SIZE ) ) 
		return DS18X20_ERROR_CRC;
	*temperature = DS18X20_meas_format(familycode, sp);
	return DS18X20_OK;
}

#ifdef DS18X20_EEPROMSUPPORT

uint8_t DS18X20_write_scratchpad( uint8_t id[], 
	uint8_t th, uint8_t tl, uint8_t conf)
{
	ow_reset(); //**
	if( ow_input_pin_state() ) { // only send if bus is "idle" = high
		ow_command( DS18X20_WRITE_SCRATCHPAD, id );
		ow_byte_wr(th);
		ow_byte_wr(tl);
		if (id[0] == DS18B20_ID) ow_byte_wr(conf); // config avail. on B20 only
		return DS18X20_OK;
	} 
	else { 
		#ifdef DS18X20_VERBOSE
		#endif
		return DS18X20_ERROR;
	}
}

uint8_t DS18X20_read_scratchpad( uint8_t id[], uint8_t sp[] )
{
	uint8_t i;
	
	ow_reset(); //**
	if( ow_input_pin_state() ) { // only send if bus is "idle" = high
		ow_command( DS18X20_READ, id );
		for ( i=0 ; i< DS18X20_SP_SIZE; i++ )	sp[i]=ow_byte_rd();
		return DS18X20_OK;
	} 
	else { 
		#ifdef DS18X20_VERBOSE
		#endif
		return DS18X20_ERROR;
	}
}

uint8_t DS18X20_copy_scratchpad( uint8_t with_power_extern, 
	uint8_t id[] )
{
	ow_reset(); //**
	if( ow_input_pin_state() ) { // only send if bus is "idle" = high
		ow_command( DS18X20_COPY_SCRATCHPAD, id );
		if (with_power_extern != DS18X20_POWER_EXTERN)
			ow_parasite_enable();
		_delay_ms(DS18X20_COPYSP_DELAY); // wait for 10 ms 
		if (with_power_extern != DS18X20_POWER_EXTERN)
			ow_parasite_disable();
		return DS18X20_OK;
	} 
	else { 
		#ifdef DS18X20_VERBOSE
		#endif
		return DS18X20_START_FAIL;
	}
}

uint8_t DS18X20_recall_E2( uint8_t id[] )
{
	ow_reset(); //**
	if( ow_input_pin_state() ) { // only send if bus is "idle" = high
		ow_command( DS18X20_RECALL_E2, id );
		// TODO: wait until status is "1" (then eeprom values
		// have been copied). here simple delay to avoid timeout 
		// handling
		_delay_ms(DS18X20_COPYSP_DELAY);
		return DS18X20_OK;
	} 
	else { 
		#ifdef DS18X20_VERBOSE
		#endif
		return DS18X20_ERROR;
	}
}
#endif
