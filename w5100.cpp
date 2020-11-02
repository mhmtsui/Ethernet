/*
 * Copyright 2018 Paul Stoffregen
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */
#include <cpudefs.h>
#include <Arduino.h>
#include <DSPI.h>
#include <sys/kmem.h>
#include "eethernet.h"
#include "w5100.h"
#include "board.h"

#ifdef defined (__PIC32MX3XX__)
	DSPI0 _spi0;
	#define SPI_BASE _DSPI0_BASE
#elif defined (__PIC32MZXX__)
	DSPI2 _spi0;
	#define SPI_BASE _DSPI2_BASE
	//#define ASYNC_ENA
	//#define INT_ENA
	//#ifdef ASYNC_ENA
	volatile uint8_t __attribute__((coherent)) txBuf_g[2050];
	volatile uint8_t __attribute__((coherent)) rxBuf_g[2050];
	//#endif
#else
	DSPI0 _spi0;
	#define SPI_BASE _DSPI0_BASE
#endif

/***************************************************/
/**            Default SS pin setting             **/
/***************************************************/

// If variant.h or other headers specifically define the
// default SS pin for ethernet, use it.
#if defined(PIN_SPI_SS_ETHERNET_LIB)
#define SS_PIN_DEFAULT  PIN_SPI_SS_ETHERNET_LIB

// MKR boards default to pin 5 for MKR ETH
// Pins 8-10 are MOSI/SCK/MISO on MRK, so don't use pin 10
#elif defined(USE_ARDUINO_MKR_PIN_LAYOUT) || defined(ARDUINO_SAMD_MKRZERO) || defined(ARDUINO_SAMD_MKR1000) || defined(ARDUINO_SAMD_MKRFox1200) || defined(ARDUINO_SAMD_MKRGSM1400) || defined(ARDUINO_SAMD_MKRWAN1300)
#define SS_PIN_DEFAULT  5

// For boards using AVR, assume shields with SS on pin 10
// will be used.  This allows for Arduino Mega (where
// SS is pin 53) and Arduino Leonardo (where SS is pin 17)
// to work by default with Arduino Ethernet Shield R2 & R3.
#elif defined(__AVR__)
#define SS_PIN_DEFAULT  10

// If variant.h or other headers define these names
// use them if none of the other cases match
#elif defined(PIN_SPI_SS)
#define SS_PIN_DEFAULT  PIN_SPI_SS
#elif defined(CORE_SS0_PIN)
#define SS_PIN_DEFAULT  CORE_SS0_PIN

// As a final fallback, use pin 10
#else
#define SS_PIN_DEFAULT  10
#endif

uint32_t W5100Class::cumWaitTime = 0;
uint32_t W5100Class::numWaitCalls = 0;

// W5100 controller instance
uint8_t  W5100Class::chip = 0;
uint8_t  W5100Class::CH_BASE_MSB;
uint8_t  W5100Class::ss_pin = SS_PIN_DEFAULT;
#ifdef ETHERNET_LARGE_BUFFERS
uint16_t W5100Class::SSIZE = 2048;
uint16_t W5100Class::SMASK = 0x07FF;
#endif
W5100Class W5100;

// pointers and bitmasks for optimized SS pin
#if defined(__AVR__)
  volatile uint8_t * W5100Class::ss_pin_reg;
  uint8_t W5100Class::ss_pin_mask;
#elif defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK66FX1M0__) || defined(__MK64FX512__)
  volatile uint8_t * W5100Class::ss_pin_reg;
#elif defined(__MKL26Z64__)
  volatile uint8_t * W5100Class::ss_pin_reg;
  uint8_t W5100Class::ss_pin_mask;
#elif defined(__SAM3X8E__) || defined(__SAM3A8C__) || defined(__SAM3A4C__)
  volatile uint32_t * W5100Class::ss_pin_reg;
  uint32_t W5100Class::ss_pin_mask;
#elif defined(__PIC32MX__) || defined(__PIC32MZ__)
  volatile uint32_t * W5100Class::ss_pin_reg;
  uint32_t W5100Class::ss_pin_mask;
#elif defined(ARDUINO_ARCH_ESP8266)
  volatile uint32_t * W5100Class::ss_pin_reg;
  uint32_t W5100Class::ss_pin_mask;
#elif defined(__SAMD21G18A__)
  volatile uint32_t * W5100Class::ss_pin_reg;
  uint32_t W5100Class::ss_pin_mask;
#endif


uint8_t W5100Class::init(void)
{
	static bool initialized = false;
	uint8_t i;

	if (initialized) return 1;

	// Many Ethernet shields have a CAT811 or similar reset chip
	// connected to W5100 or W5200 chips.  The W5200 will not work at
	// all, and may even drive its MISO pin, until given an active low
	// reset pulse!  The CAT811 has a 240 ms typical pulse length, and
	// a 400 ms worst case maximum pulse length.  MAX811 has a worst
	// case maximum 560 ms pulse length.  This delay is meant to wait
	// until the reset pulse is ended.  If your hardware has a shorter
	// reset time, this can be edited or removed.
	delay(560);
	DEBUG_PRINTLN("w5100 init");
#ifdef ASYNC_ENA
	_spi0.beginasync(ss_pin, 6, 7);
	//_spi0.begin(ss_pin);
#else
	_spi0.beginasync(ss_pin, 6, 7);
	//_spi0.begin(ss_pin);
#endif
	initSS();
	resetSS();
	//SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
	_spi0.setTransferSize(DSPI_8BIT);
	_spi0.setSpeed(33000000);
	_spi0.setMode(DSPI_MODE0);

	p32_spi *pspi = (p32_spi *) SPI_BASE;
	pspi->sxCon.clr = (1 << _SPICON_ON);
	pspi->sxCon.reg = 0;
	pspi->sxCon.set = ((0 << _SPICON_CKP) | (1 << _SPICON_CKE) | (1 << _SPICON_SMP) | (1 << _SPICON_MSTEN));
	pspi->sxCon.set = (1 << _SPICON_ON);

#ifdef INT_ENA
	_spi0.enableInterruptTransfer();
#else
	_spi0.disableInterruptTransfer();
#endif

	// Attempt W5200 detection first, because W5200 does not properly
	// reset its SPI state when CS goes high (inactive).  Communication
	// from detecting the other chips can leave the W5200 in a state
	// where it won't recover, unless given a reset pulse.
	if (isW5200()) {
		CH_BASE_MSB = 0x40;
#ifdef ETHERNET_LARGE_BUFFERS
#if MAX_SOCK_NUM <= 1
		SSIZE = 16384;
#elif MAX_SOCK_NUM <= 2
		SSIZE = 8192;
#elif MAX_SOCK_NUM <= 4
		SSIZE = 4096;
#else
		SSIZE = 2048;
#endif
		SMASK = SSIZE - 1;
#endif
		for (i=0; i<MAX_SOCK_NUM; i++) {
			writeSnRX_SIZE(i, SSIZE >> 10);
			writeSnTX_SIZE(i, SSIZE >> 10);
		}
		for (; i<8; i++) {
			writeSnRX_SIZE(i, 0);
			writeSnTX_SIZE(i, 0);
		}
	// Try W5500 next.  Wiznet finally seems to have implemented
	// SPI well with this chip.  It appears to be very resilient,
	// so try it after the fragile W5200
	} else if (isW5500()) {
		CH_BASE_MSB = 0x10;
#ifdef ETHERNET_LARGE_BUFFERS
#if MAX_SOCK_NUM <= 1
		SSIZE = 16384;
#elif MAX_SOCK_NUM <= 2
		SSIZE = 8192;
#elif MAX_SOCK_NUM <= 4
		SSIZE = 4096;
#else
		SSIZE = 2048;
#endif
		SMASK = SSIZE - 1;
		for (i=0; i<MAX_SOCK_NUM; i++) {
			writeSnRX_SIZE(i, SSIZE >> 10);
			writeSnTX_SIZE(i, SSIZE >> 10);
		}
		for (; i<8; i++) {
			writeSnRX_SIZE(i, 0);
			writeSnTX_SIZE(i, 0);
		}
#endif
	// Try W5100 last.  This simple chip uses fixed 4 byte frames
	// for every 8 bit access.  Terribly inefficient, but so simple
	// it recovers from "hearing" unsuccessful W5100 or W5200
	// communication.  W5100 is also the only chip without a VERSIONR
	// register for identification, so we check this last.
	} else if (isW5100()) {
		CH_BASE_MSB = 0x04;
#ifdef ETHERNET_LARGE_BUFFERS
#if MAX_SOCK_NUM <= 1
		SSIZE = 8192;
		writeTMSR(0x03);
		writeRMSR(0x03);
#elif MAX_SOCK_NUM <= 2
		SSIZE = 4096;
		writeTMSR(0x0A);
		writeRMSR(0x0A);
#else
		SSIZE = 2048;
		writeTMSR(0x55);
		writeRMSR(0x55);
#endif
		SMASK = SSIZE - 1;
#else
		writeTMSR(0x55);
		writeRMSR(0x55);
#endif
	// No hardware seems to be present.  Or it could be a W5200
	// that's heard other SPI communication if its chip select
	// pin wasn't high when a SD card or other SPI chip was used.
	} else {
		DEBUG_PRINTLN("no chip :-(");
		chip = 0;
		//SPI.endTransaction();
		return 0; // no known chip is responding :-(
	}
 
	// Test showed default TTL was 128. Recommended value is 64 secs, per https://www.iana.org/assignments/ip-parameters/ip-parameters.xhtml#ip-parameters-2
	for (i=0; i<MAX_SOCK_NUM; i++) {
		writeSnTTL(i, TTL_DEFAULT);
	}
	uint8_t ttl = readSnTTL(0);
	DEBUG_PRINT("\nW5100.init() - TTL(0) = ");
	DEBUG_PRINTLN(ttl); 

	//SPI.endTransaction();
	initialized = true;
	return 1; // successful init
}

// Soft reset the Wiznet chip, by writing to its MR register reset bit
uint8_t W5100Class::softReset(void)
{
	uint16_t count=0;

	DEBUG_PRINTLN("Wiznet soft reset");
	// write to reset bit
	writeMR(0x80);
	// then wait for soft reset to complete
	do {
		delay(1);
		uint8_t mr = readMR();
		DEBUG_PRINT("mr=");
		DEBUG_PRINTLNHEX(mr);
		if (mr == 0) return 1;
		//delay(1);		// moved 1/11/2019
	} while (++count < 20);
	return 0;
}

uint8_t W5100Class::isW5100(void)
{
	chip = 51;
	DEBUG_PRINTLN("w5100.cpp: detect W5100 chip");
	if (!softReset()) return 0;
	writeMR(0x10);
	if (readMR() != 0x10) return 0;
	writeMR(0x12);
	if (readMR() != 0x12) return 0;
	writeMR(0x00);
	if (readMR() != 0x00) return 0;
	DEBUG_PRINTLN("chip is W5100");
	return 1;
}

uint8_t W5100Class::isW5200(void)
{
	chip = 52;
	DEBUG_PRINTLN("w5100.cpp: detect W5200 chip");
	if (!softReset()) return 0;
	writeMR(0x08);
	if (readMR() != 0x08) return 0;
	writeMR(0x10);
	if (readMR() != 0x10) return 0;
	writeMR(0x00);
	if (readMR() != 0x00) return 0;
	int ver = readVERSIONR_W5200();
	DEBUG_PRINT("version=");
	DEBUG_PRINTLN(ver);
	if (ver != 3) return 0;
	DEBUG_PRINTLN("chip is W5200");
	return 1;
}

uint8_t W5100Class::isW5500(void)
{
	chip = 55;
	DEBUG_PRINTLN("w5100.cpp: detect W5500 chip");
	if (!softReset()) return 0;
	writeMR(0x08);
	if (readMR() != 0x08) return 0;
	writeMR(0x10);
	if (readMR() != 0x10) return 0;
	writeMR(0x00);
	if (readMR() != 0x00) return 0;
	int ver = readVERSIONR_W5500();
	DEBUG_PRINT("version=");
	DEBUG_PRINTLN(ver);

/*	uint16_t currRTRval = 0;
	currRTRval = readRTR();
	DEBUG_PRINT("\nisW5500() - RTR = ");
	DEBUG_PRINTLN(currRTRval);
	uint8_t currRCRval = 0;
	currRCRval = readRCR();
	DEBUG_PRINT("isW5500() - RCR = ");
	DEBUG_PRINTLN(currRCRval);*/

	if (ver != 4) return 0;
	DEBUG_PRINTLN("chip is W5500");
	return 1;
}

W5100Linkstatus W5100Class::getLinkStatus()
{
	uint8_t phystatus;

	if (!init()) return UNKNOWN;
	switch (chip) {
	  case 52:
		//SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
		phystatus = readPSTATUS_W5200();
		//SPI.endTransaction();
		if (phystatus & 0x20) return LINK_ON;
		return LINK_OFF;
	  case 55:
		//SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
		phystatus = readPHYCFGR_W5500();
		//SPI.endTransaction();
		if (phystatus & 0x01) return LINK_ON;
		return LINK_OFF;
	  default:
		return UNKNOWN;
	}
}

uint16_t W5100Class::write(uint16_t addr, const uint8_t *buf, uint16_t len)
{
	uint8_t cmd[8];
#ifdef ASYNC_ENA
	// uint8_t * ptr;
	// for(ptr = (uint8_t *) buf; ptr < buf+len; ptr+=16){
	// 	_cache(((1)|(5<<2)), ptr);
	// }
	// uint8_t* ptr1;//uint8_t* ptr2;
	// for(ptr1=(uint8_t*) buf; ptr1<buf+len;ptr1++){
	// 	DEBUG_PRINT(*ptr1);
	// 	DEBUG_PRINT(',');
	// 	DEBUG_PRINT(*((uint8_t*)(KVA0_TO_KVA1(ptr1))));
	// }
	// DEBUG_PRINTLN();	
#endif
	if (chip == 51) {
		for (uint16_t i=0; i<len; i++) {
			setSS();
#ifdef ASYNC_ENA
			//uint8_t temp[4];
			txBuf_g[0] = 0xF0;
			txBuf_g[1] = addr>>8;
			txBuf_g[2] = addr&0xFF;
			addr++;
			txBuf_g[3] = buf[i];
			// _cache(((1)|(5<<2)), temp);
			// _sync();
			_spi0.asyncTransfertimeout(4, (uint8_t*) txBuf_g, (uint8_t*) rxBuf_g, 30);
#elif defined(INT_ENA)
			uint8_t temp[4];
			temp[0] = 0xF0;
			temp[1] = addr>>8;
			temp[2] = addr&0xFF;
			addr++;
			temp[3] = buf[i];
			_spi0.intTransfertimeout(4, temp, 30);
#else
			_spi0.transfer(0xF0);
			_spi0.transfer(addr >> 8);
			_spi0.transfer(addr & 0xFF);
			addr++;
			_spi0.transfer(buf[i]);
#endif
			resetSS();
		}
	} else if (chip == 52) {
		setSS();
		cmd[0] = addr >> 8;
		cmd[1] = addr & 0xFF;
		cmd[2] = ((len >> 8) & 0x7F) | 0x80;
		cmd[3] = len & 0xFF;
#ifdef ASYNC_ENA
		// _cache(((1)|(5<<2)), cmd);
		// _sync();
		txBuf_g[0] = addr >> 8;
		txBuf_g[1] = addr & 0xFF;
		txBuf_g[2] = ((len >> 8) & 0x7F) | 0x80;
		txBuf_g[3] = len & 0xFF;
		_spi0.asyncTransfertimeout(4, (uint8_t*) txBuf_g, (uint8_t*) rxBuf_g, 30);
#elif defined(INT_ENA)
		_spi0.intTransfertimeout(4, cmd, 30);
#else
		_spi0.transfer(4, cmd);
#endif
#ifdef ASYNC_ENA
	// uint8_t * ptr;
	// for(ptr = (uint8_t *) buf; ptr < buf+len; ptr+=1){
	// 	_cache(((1)|(5<<2)), ptr);
	// 	_sync();
	// }
	memcpy((void*) txBuf_g, buf, len);
	_spi0.asyncTransfertimeout(len, (uint8_t*) txBuf_g, (uint8_t*) rxBuf_g, 30);
#elif defined(INT_ENA)
	_spi0.intTransfertimeout(len, (uint8_t*) buf, 30);
#else
	#ifdef SPI_HAS_TRANSFER_BUF
		_spi0.transfer(len, (uint8_t*) buf);
	#else
		// TODO: copy 8 bytes at a time to cmd[] and block transfer
		for (uint16_t i=0; i < len; i++) {
			_spi0.transfer(buf[i]);
		}
	#endif
#endif
		resetSS();
	} else { // chip == 55
		setSS();
		if (addr < 0x100) {
			// common registers 00nn
			cmd[0] = 0;
			cmd[1] = addr & 0xFF;
			cmd[2] = 0x04;
		} else if (addr < 0x8000) {
			// socket registers  10nn, 11nn, 12nn, 13nn, etc
			cmd[0] = 0;
			cmd[1] = addr & 0xFF;
			cmd[2] = ((addr >> 3) & 0xE0) | 0x0C;
		} else if (addr < 0xC000) {
			// transmit buffers  8000-87FF, 8800-8FFF, 9000-97FF, etc
			//  10## #nnn nnnn nnnn
			cmd[0] = addr >> 8;
			cmd[1] = addr & 0xFF;
			#if defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 1
			cmd[2] = 0x14;                       // 16K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 2
			cmd[2] = ((addr >> 8) & 0x20) | 0x14; // 8K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 4
			cmd[2] = ((addr >> 7) & 0x60) | 0x14; // 4K buffers
			#else
			cmd[2] = ((addr >> 6) & 0xE0) | 0x14; // 2K buffers
			#endif
		} else {
			// receive buffers
			cmd[0] = addr >> 8;
			cmd[1] = addr & 0xFF;
			#if defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 1
			cmd[2] = 0x1C;                       // 16K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 2
			cmd[2] = ((addr >> 8) & 0x20) | 0x1C; // 8K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 4
			cmd[2] = ((addr >> 7) & 0x60) | 0x1C; // 4K buffers
			#else
			cmd[2] = ((addr >> 6) & 0xE0) | 0x1C; // 2K buffers
			#endif
		}
		if (len <= 5) {
			for (uint8_t i=0; i < len; i++) {
				cmd[i + 3] = buf[i];
			}
#ifdef ASYNC_ENA
			// _cache(((1)|(5<<2)), cmd);
			// _sync();
			memcpy((void *) txBuf_g, cmd, len+3);
			_spi0.asyncTransfertimeout(len+3, (uint8_t*) txBuf_g, (uint8_t*) rxBuf_g, 30);
#elif defined(INT_ENA)
			_spi0.intTransfertimeout(len+3, cmd, 30);
#else
			_spi0.transfer(len + 3, cmd);
#endif
		} else {
#ifdef ASYNC_ENA
			// _cache(((1)|(5<<2)), cmd);
			// _sync();
			memcpy((void *) txBuf_g, cmd, 3);
			_spi0.asyncTransfertimeout(3, (uint8_t*) txBuf_g, (uint8_t*) rxBuf_g, 30);
#elif defined(INT_ENA)
			_spi0.intTransfertimeout(3, cmd, 30);
#else
			_spi0.transfer(3, cmd);
#endif
#ifdef ASYNC_ENA
			// uint8_t * ptr;
			// for(ptr = (uint8_t *) buf; ptr < buf+len; ptr+=1){
			// 	_cache(((1)|(5<<2)), ptr);
			// 	_sync();
			// }
			memcpy((void *) txBuf_g, buf, len);
			_spi0.asyncTransfertimeout(len, (uint8_t*) txBuf_g, (uint8_t*) rxBuf_g, 30);
#elif defined(INT_ENA)
			_spi0.intTransfertimeout(len, (uint8_t*) buf, 30);
#else
	//DEBUG_PRINT("WRITE");
	//DEBUG_PRINTLN(len);
	#ifdef SPI_HAS_TRANSFER_BUF
			//_spi0.transfer(len, (uint8_t*) buf);
			memcpy((void *) txBuf_g, buf, len);
			_spi0.asyncTransfertimeout(len, (uint8_t*) txBuf_g, (uint8_t*) rxBuf_g, 30);
	#else
			// TODO: copy 8 bytes at a time to cmd[] and block transfer
			for (uint16_t i=0; i < len; i++) {
				_spi0.transfer(buf[i]);
			}
	#endif
#endif
		}
		resetSS();
	}
	return len;
}

uint16_t W5100Class::read(uint16_t addr, uint8_t *buf, uint16_t len)
{
	uint8_t cmd[4];
#ifdef ASYNC_ENA
	// uint8_t * ptr;
	// for(ptr = (uint8_t *) buf; ptr < buf+len; ptr+=16){
	// 	_cache(((1)|(5<<2)), ptr);
	// }
#endif
	if (chip == 51) {
		for (uint16_t i=0; i < len; i++) {
			setSS();
#ifdef ASYNC_ENA
			txBuf_g[0] = 0x0F;
			txBuf_g[1] = addr >> 8;
			txBuf_g[2] = addr & 0xFF;
			txBuf_g[3] = 0;
			// _cache(((1)|(5<<2)), cmd);
			// _sync();
			_spi0.asyncTransfertimeout(4, (uint8_t*) txBuf_g, (uint8_t*) rxBuf_g, 30); // TODO: why doesn't this work?
			//_spi0.asyncTransfertimeout(1, (uint8_t)0x00, (uint8_t*) &txBuf_g[3], 30);
			//_cache(((1)|(4<<2)), cmd);
			//_sync();
			//uint8_t * tmp = (uint8_t*) KVA0_TO_KVA1(cmd);
			buf[i] = rxBuf_g[3];
			addr++;
#elif defined(INT_ENA)
			cmd[0] = 0x0F;
			cmd[1] = addr >> 8;
			cmd[2] = addr & 0xFF;
			cmd[3] = 0;
			//_cache(((1)|(5<<2)), cmd);
			_spi0.intTransfertimeout(4, cmd, cmd, 30); // TODO: why doesn't this work?
			//_cache(((1)|(4<<2)), cmd);
			buf[i] = cmd[3];
			addr++;
#else
			#if 1
			_spi0.transfer(0x0F);
			_spi0.transfer(addr >> 8);
			_spi0.transfer(addr & 0xFF);
			addr++;
			_spi0.transfer(0);
			#else
			cmd[0] = 0x0F;
			cmd[1] = addr >> 8;
			cmd[2] = addr & 0xFF;
			cmd[3] = 0;
			_spi0.transfer(cmd, 4); // TODO: why doesn't this work?
			buf[i] = cmd[3];
			addr++;
			#endif
#endif
			resetSS();
		}
	} else if (chip == 52) {
		setSS();
		cmd[0] = addr >> 8;
		cmd[1] = addr & 0xFF;
		cmd[2] = (len >> 8) & 0x7F;
		cmd[3] = len & 0xFF;
#ifdef ASYNC_ENA
		// _cache(((1)|(5<<2)), cmd);
		// _sync();
		txBuf_g[0] = addr >> 8;
		txBuf_g[1] = addr & 0xFF;
		txBuf_g[2] = (len >> 8) & 0x7F;
		txBuf_g[3] = len & 0xFF;
		_spi0.asyncTransfertimeout(4,(uint8_t*)  txBuf_g, (uint8_t*) rxBuf_g, 30);		
#elif defined(INT_ENA)
		_spi0.intTransfertimeout(4, cmd, 30);
#else
		_spi0.transfer(4, cmd);
#endif
		memset(buf, 0, len);
#ifdef ASYNC_ENA
		memset((void *) rxBuf_g, 0, len);
		_spi0.asyncTransfertimeout(len, (uint8_t) 0x00, (uint8_t*)  rxBuf_g, 30);
		memcpy(buf, (void *)  rxBuf_g, len);
		// uint8_t * ptr;
		// for(ptr = (uint8_t *) buf; ptr < buf+len; ptr+=1){
		// 	_cache(((1)|(4<<2)), ptr);
		// 	_sync();
		// }
		//buf = (uint8_t *) KVA0_TO_KVA1(buf);
#elif defined(INT_ENA)
		_spi0.intTransfertimeout(len, (uint8_t) 0x0, buf, 30);
#else
		_spi0.transfer(len, buf, buf);
#endif
		resetSS();
	} else { // chip == 55
		setSS();
		if (addr < 0x100) {
			// common registers 00nn
			cmd[0] = 0;
			cmd[1] = addr & 0xFF;
			cmd[2] = 0x00;
		} else if (addr < 0x8000) {
			// socket registers  10nn, 11nn, 12nn, 13nn, etc
			cmd[0] = 0;
			cmd[1] = addr & 0xFF;
			cmd[2] = ((addr >> 3) & 0xE0) | 0x08;
		} else if (addr < 0xC000) {
			// transmit buffers  8000-87FF, 8800-8FFF, 9000-97FF, etc
			//  10## #nnn nnnn nnnn
			cmd[0] = addr >> 8;
			cmd[1] = addr & 0xFF;
			#if defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 1
			cmd[2] = 0x10;                       // 16K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 2
			cmd[2] = ((addr >> 8) & 0x20) | 0x10; // 8K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 4
			cmd[2] = ((addr >> 7) & 0x60) | 0x10; // 4K buffers
			#else
			cmd[2] = ((addr >> 6) & 0xE0) | 0x10; // 2K buffers
			#endif
		} else {
			// receive buffers
			cmd[0] = addr >> 8;
			cmd[1] = addr & 0xFF;
			#if defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 1
			cmd[2] = 0x18;                       // 16K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 2
			cmd[2] = ((addr >> 8) & 0x20) | 0x18; // 8K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 4
			cmd[2] = ((addr >> 7) & 0x60) | 0x18; // 4K buffers
			#else
			cmd[2] = ((addr >> 6) & 0xE0) | 0x18; // 2K buffers
			#endif
		}
#ifdef ASYNC_ENA
		// _cache(((1)|(5<<2)), cmd);
		// _sync();
		memcpy((void *) txBuf_g, cmd, 3);
		_spi0.asyncTransfertimeout(3, (uint8_t*) txBuf_g, (uint8_t*) rxBuf_g, 30);
#elif defined(INT_ENA)
		_spi0.intTransfertimeout(3, cmd, 30);
#else
		_spi0.transfer(3, cmd);
#endif
		memset(buf, 0, len);
#ifdef ASYNC_ENA
		memset((void*) rxBuf_g, 0, len);
		_spi0.asyncTransfertimeout(len, (uint8_t) 0x00, (uint8_t*) rxBuf_g, 30);
		memcpy(buf, (void *) rxBuf_g, len);
		// uint8_t * ptr;
		// for(ptr = (uint8_t *) buf; ptr < buf+len; ptr+=1){
		// 	_cache(((1)|(4<<2)), ptr);
		// 	_sync();
		// }
		// DEBUG_PRINT("READ KVA0");
		// DEBUG_PRINTLN(buf[0]);
		// buf = (uint8_t *) KVA0_TO_KVA1(buf);
		// DEBUG_PRINT("READ KVA1");
		// DEBUG_PRINTLN(buf[0]);
#elif defined(INT_ENA)
		_spi0.intTransfertimeout(len, (uint8_t)0x00, buf, 30);
		// DEBUG_PRINT("READ");
		// DEBUG_PRINTLN(buf[0]);
#else
	if (len > 2){
		DEBUG_PRINT("READ");
		DEBUG_PRINTLN(len);
		memset((void*) rxBuf_g, 0, len);
		_spi0.asyncTransfertimeout(len, (uint8_t*) rxBuf_g, (uint8_t*) rxBuf_g, 30);
		memcpy((void *) buf, (void *) rxBuf_g, len);
	}else{
		_spi0.transfer(len, buf , buf);
	}
#endif
		resetSS();
	}
	return len;
}

void W5100Class::execCmdSn(SOCKET s, SockCMD _cmd)
{
	W5100.writeSnIR(s, SnIR::TIMEOUT | SnIR::DISCON);	// clear SnIR TIMEOUT bit +rs 18Feb2019
	delayMicroseconds(1);								// wait for register to clear

	writeSnCR(s, _cmd);									// Send command to socket
	// Wait for command to complete
	if (_cmd == Sock_CONNECT || _cmd == Sock_DISCON) {		
	  delayMicroseconds(20);							// Connect and Disconnect consistently fail without this delay
	  //delayMicroseconds(1);								// Connect and Disconnect consistently fail without this delay
	}
	
	do {
		if (_cmd == Sock_CONNECT || _cmd == Sock_DISCON)
			delayMicroseconds(20);						// Connect and Disconnect often fail without this longer delay
			//delayMicroseconds(1);						// Connect and Disconnect often fail without this longer delay
		else
			delayMicroseconds(1);
	} while (readSnCR(s));
}

// waitForCmd is called after a socket command is sent. It waits for the socket status register to indicate that the command has been successfully executed
bool W5100Class::waitForCmd(SOCKET s, uint8_t SnSR_expected)
{
	uint8_t IRstat;
	uint32_t start = micros();
	uint8_t stat;
	int cnt=0;
//	return true;

	W5100Class::numWaitCalls++;

	// Now wait for socket status register to have expected value
	//SPI.beginTransaction(SPI_ETHERNET_SETTINGS);		// begin SPI transaction

	do {		
		if (++cnt > 500){
			cnt = 0;
			delay(1);
		}else{												// W5500 sets socket status register when command is complete
			delayMicroseconds(1);						// delay to give W5x00 time to process command
		}										
		//delayMicroseconds(1);							// delay to give W5x00 time to process command
		stat = W5100.readSnSR(s);						// read W5x00 Socket n Status Register
		IRstat = W5100.readSnIR(s);						// read W5x00 Socket n Interrupt Register
		if ((IRstat & SnIR::TIMEOUT) == SnIR::TIMEOUT ||// if timeout occurs before command completes...
			(IRstat & SnIR::DISCON) == SnIR::DISCON) {	//  or client disconnects...
			//SPI.endTransaction();						//   end SPI transaction and...
			cumWaitTime+= micros() - start;
			return false;								//    return fail.
		}
	} while (stat != SnSR_expected);					// if the expected result occurs...

	//SPI.endTransaction();								//  end SPI transaction and...
	W5100Class::cumWaitTime+= micros() - start;
	return true;										//   return success.
}

// this version of waitForCmd is called when a socket command may result in one of two different socket status register values upon successful completion
bool W5100Class::waitForCmd(SOCKET s, uint8_t SnSR_expected1, uint8_t SnSR_expected2, uint8_t SnSR_expected3)
{
	uint8_t IRstat;
	uint32_t start = micros();
	uint8_t stat;
	int cnt=0;
//	return true;

	W5100Class::numWaitCalls++;
	
	// Now wait for socket status register to have expected value
	//SPI.beginTransaction(SPI_ETHERNET_SETTINGS);		// begin SPI transaction
	W5100.writeSnIR(s, SnIR::TIMEOUT);					// clear SnIR TIMEOUT bit
	do {												// W5500 sets socket status register when command is complete
		if (++cnt > 500){
			cnt = 0;
			delay(1);
		}else{												// W5500 sets socket status register when command is complete
			delayMicroseconds(1);						// delay to give W5x00 time to process command
		}								// delay to give W5x00 time to process command
		//delayMicroseconds(1);							// delay to give W5x00 time to process command
		stat = W5100.readSnSR(s);						// read W5x00 Socket n Status Register
		IRstat = W5100.readSnIR(s);						// read W5x00 Socket n Interrupt Register
		if ((IRstat & SnIR::TIMEOUT) == SnIR::TIMEOUT ||// if timeout occurs before command completes...
			(IRstat & SnIR::DISCON) == SnIR::DISCON) {	//  or client disconnects...
			//SPI.endTransaction();						//   end SPI transaction and...
			cumWaitTime+= micros() - start;
			return false;								//    return fail
		}
/*		if (millis() - start > 5000)
			return false; */
	} while (stat != SnSR_expected1 && 
			 stat != SnSR_expected2 && 
			 stat != SnSR_expected3);					// if one of the 3 expected results occurs...

	//SPI.endTransaction();								//  end SPI transaction and...
	W5100Class::cumWaitTime+= micros() - start;
	return true;										//   return true for success.
}

uint32_t W5100Class::getAvgWait(void) { 
	
	return W5100Class::cumWaitTime/W5100Class::numWaitCalls; 

}