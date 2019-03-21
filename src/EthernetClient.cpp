/* Copyright 2018 Paul Stoffregen
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the "Software"), to deal in the Software
 * without restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <Arduino.h>
#include "Ethernet.h"
#include "Dns.h"
#include "utility/w5100.h"

int EthernetClient::connect(const char * host, uint16_t port)
{
	DNSClient dns;									// Look up the host first
	IPAddress remote_addr;

	if (sockindex < MAX_SOCK_NUM) {
		uint8_t stat = Ethernet.socketStatus(sockindex);
		if (stat != SnSR::FIN_WAIT &&
			stat != SnSR::CLOSING &&
			stat != SnSR::TIME_WAIT &&
			stat != SnSR::CLOSE_WAIT &&
			stat != SnSR::LAST_ACK &&
			stat != SnSR::CLOSED) {
		//if (Ethernet.socketStatus(sockindex) != SnSR::CLOSED) {
			Ethernet.socketDisconnect(sockindex);	// TODO: should we call stop()?
		}
		sockindex = MAX_SOCK_NUM;
	}
	dns.begin(Ethernet.dnsServerIP());
	if (!dns.getHostByName(host, remote_addr)) {
		//Serial.println("EthernetClient::connect() - DNS lookup failed");	
		return 0; // TODO: use _timeout
	}
	return connect(remote_addr, port);
}

int EthernetClient::connect(IPAddress ip, uint16_t port)
{
	if (sockindex < MAX_SOCK_NUM) {
		uint8_t stat = Ethernet.socketStatus(sockindex);
		if (stat != SnSR::FIN_WAIT &&
			stat != SnSR::CLOSING &&
			stat != SnSR::TIME_WAIT &&
			stat != SnSR::CLOSE_WAIT &&
			stat != SnSR::LAST_ACK &&
			stat != SnSR::CLOSED) {
		//if (Ethernet.socketStatus(sockindex) != SnSR::CLOSED) {
			Ethernet.socketDisconnect(sockindex); // TODO: should we call stop()?
		}
		sockindex = MAX_SOCK_NUM;
	}
#if defined(ESP8266) || defined(ESP32)
	if (ip == IPAddress((uint32_t)0) || ip == IPAddress(0xFFFFFFFFul)) return 0;
#else
	if (ip == IPAddress(0ul) || ip == IPAddress(0xFFFFFFFFul)) return 0;
#endif
		
	sockindex = Ethernet.socketBegin(SnMR::TCP, 0);
	if (sockindex >= MAX_SOCK_NUM) return 0;
	Ethernet.socketConnect(sockindex, rawIPAddress(ip), port);
	//uint32_t start = millis();


	//// I see socketStatus returning 0x13 in the following loop, which suggests that the command was never sent or seen by the W5500.
	//// Consider watching for this condition and resending the connect command if socketStatus never returns 0x15 indicating SYN has been sent.

	while (1) {
		uint8_t stat = Ethernet.socketStatus(sockindex);
		uint8_t statIR = W5100.readSnIR(sockindex);
		if (stat == SnSR::ESTABLISHED) return 1;
		if (stat == SnSR::CLOSE_WAIT) return 1;
		if (stat == SnSR::CLOSED || (statIR & SnIR::TIMEOUT) == SnIR::TIMEOUT) {
			return 0;
		}
		delayMicroseconds(5);			// changed from 5ms 20 Mar 2019
	}

	Ethernet.socketClose(sockindex);
	sockindex = MAX_SOCK_NUM;
	return 0;
}

int EthernetClient::availableForWrite(void)
{
	if (sockindex >= MAX_SOCK_NUM) return 0;
	return Ethernet.socketSendAvailable(sockindex);
}

size_t EthernetClient::write(uint8_t b)
{
	return write(&b, 1);
}

size_t EthernetClient::write(const uint8_t *buf, size_t size)
{
	if (sockindex >= MAX_SOCK_NUM) return 0;
	if (Ethernet.socketSend(sockindex, buf, size)) return size;
	setWriteError();
	return 0;
}

int EthernetClient::available()
{
	if (sockindex >= MAX_SOCK_NUM) return 0;
	return Ethernet.socketRecvAvailable(sockindex);
	// TODO: do the Wiznet chips automatically retransmit TCP ACK
	// packets if they are lost by the network?  Someday this should
	// be checked by a man-in-the-middle test which discards certain
	// packets.  If ACKs aren't resent, we would need to check for
	// returning 0 here and after a timeout do another Sock_RECV
	// command to cause the Wiznet chip to resend the ACK packet.
}

int EthernetClient::read(uint8_t *buf, size_t size)
{
	if (sockindex >= MAX_SOCK_NUM) return 0;
	return Ethernet.socketRecv(sockindex, buf, size);
}

int EthernetClient::peek()
{
	if (sockindex >= MAX_SOCK_NUM) return -1;
	if (!available()) return -1;
	return Ethernet.socketPeek(sockindex);
}

int EthernetClient::read()
{
	uint8_t b;
	if (Ethernet.socketRecv(sockindex, &b, 1) > 0) return b;
	return -1;
}

void EthernetClient::flush()
{
	while (sockindex < MAX_SOCK_NUM) {
		uint8_t stat = Ethernet.socketStatus(sockindex);
		if (stat != SnSR::ESTABLISHED && stat != SnSR::CLOSE_WAIT) return;
		if (Ethernet.socketSendAvailable(sockindex) >= W5100.SSIZE) return;
	}
}

void EthernetClient::stop()
{
	if (sockindex >= MAX_SOCK_NUM) return;

	// attempt to close the connection gracefully (send a FIN to other side)
	Ethernet.socketDisconnect(sockindex);
	unsigned long start = millis();

	// The following block of code is problematic since sockets may take up to 128 sec to close, & remain in FIN_WAIT or TIME_WAIT status in the meantime.
	// This normal behavior regularly results in a timeout here. Need to find a more elegant way to handle this that doesn't break Arduino compatibility.

	// wait up to a second for the connection to close
	do {
		uint8_t stat = Ethernet.socketStatus(sockindex);
		if (stat == SnSR::FIN_WAIT ||
			stat == SnSR::CLOSING ||
			stat == SnSR::TIME_WAIT ||
			stat == SnSR::CLOSE_WAIT ||
			stat == SnSR::LAST_ACK ||
			stat == SnSR::CLOSED) {		
			sockindex = MAX_SOCK_NUM;								
			return; // exit the loop
		}
		delay(1);
	} while (millis() - start < _timeout);		// shouldn't be needed for W5500; socket status is set to SnSR::CLOSED when RTR timeout occurs (Sn_IR = 0x08)
											    // on the other hand, it should cause no harm unless RTR is set higher than 10,000

	// if it hasn't closed, close it forcefully -- this is now done in manageSockets(), which doesn't set sockindex though...
	//Ethernet.socketClose(sockindex);			// this should probably be removed
	//sockindex = MAX_SOCK_NUM;					// should this still happen?
}

uint8_t EthernetClient::connected()
{
	if (sockindex >= MAX_SOCK_NUM) return 0;

	uint8_t s = Ethernet.socketStatus(sockindex);

	if (s == SnSR::CLOSED) {
		sockindex = MAX_SOCK_NUM;
		return false;
	}

	return !(s == SnSR::LISTEN || s == SnSR::CLOSED || s == SnSR::FIN_WAIT ||
		     ((s == SnSR::CLOSE_WAIT) && !available()));
}

uint8_t EthernetClient::status()
{
	if (sockindex >= MAX_SOCK_NUM) return SnSR::CLOSED;
	return Ethernet.socketStatus(sockindex);
}

// the next function allows us to use the client returned by
// EthernetServer::available() as the condition in an if-statement.
bool EthernetClient::operator==(const EthernetClient& rhs)
{
	if (sockindex != rhs.sockindex) return false;
	if (sockindex >= MAX_SOCK_NUM) return false;
	if (rhs.sockindex >= MAX_SOCK_NUM) return false;
	return true;
}

// https://github.com/per1234/EthernetMod
// from: https://github.com/ntruchsess/Arduino-1/commit/937bce1a0bb2567f6d03b15df79525569377dabd
uint16_t EthernetClient::localPort()
{
	if (sockindex >= MAX_SOCK_NUM) return 0;
	uint16_t port;
	SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
	port = W5100.readSnPORT(sockindex);
	SPI.endTransaction();
	return port;
}

// https://github.com/per1234/EthernetMod
// returns the remote IP address: http://forum.arduino.cc/index.php?topic=82416.0
IPAddress EthernetClient::remoteIP()
{
	if (sockindex >= MAX_SOCK_NUM) return IPAddress((uint32_t)0);
	uint8_t remoteIParray[4];
	SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
	W5100.readSnDIPR(sockindex, remoteIParray);
	SPI.endTransaction();
	return IPAddress(remoteIParray);
}

// https://github.com/per1234/EthernetMod
// from: https://github.com/ntruchsess/Arduino-1/commit/ca37de4ba4ecbdb941f14ac1fe7dd40f3008af75
uint16_t EthernetClient::remotePort()
{
	if (sockindex >= MAX_SOCK_NUM) return 0;
	uint16_t port;
	SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
	port = W5100.readSnDPORT(sockindex);
	SPI.endTransaction();
	return port;
}

// return the protocol for which the specified socket is configured
//
uint8_t EthernetClient::getSocketProtocol()
{
	if (sockindex >= MAX_SOCK_NUM) return 0;
	uint8_t protocol;
	SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
	protocol = W5100.readSnMR(sockindex) & 0x0F;	// protocol is in 4 LSBs of Socket n Mode Register: 0 = closed, 1 = TCP, 2 = UDP, 4 = MACRAW
	SPI.endTransaction();
	return protocol;
}


