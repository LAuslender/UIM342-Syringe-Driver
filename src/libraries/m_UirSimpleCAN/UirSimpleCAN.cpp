
/*////////////////////////////////////////////////////////////////////////////
MIT License

Copyright (c) [2022] UIROBOT

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Disclaimer: UIROBOT shall not be held responsible for any direct or indirect
consequences resulting from the misuse of this software, including but not
limited to damages caused by unauthorized purchases, improper configurations,
or unintended usage. Users are solely responsible for ensuring the proper and
safe application of this software in their respective environments.
////////////////////////////////////////////////////////////////////////////*/

#include "UirSimpleCAN.h"
#include "Arduino.h"
#include "mcp2515.h"
#include <SPI.h>

// Define CAN frame and MCP2515 instance
struct can_frame CanMsg;
// Instantiate MCP2515 with CS (chip select) pin 10
MCP2515 mcp2515can(10); 

///=============================
//      FUNCTIONS for GET
///=============================

/**
 * @brief Reads motor configuration using CAN ID and data.
 * @param can_id CAN ID for the message.
 * @param data_len Length of the data payload.
 * @param data Pointer to the data array.
 * @return CAN frame containing the response.
 */

can_frame SimpleCAN::ReadMLConfig(unsigned char pid, unsigned char cid, unsigned char cw, unsigned char datalen, unsigned char* data)
{
	unsigned long checkid = 0;//Store the last can_id result as a comparison
	bool checkresult = false; // Flag to determine if the correct response is received

	// Prepare CAN frame for transmission
	unsigned int ProducerID = pid;	// Producer(Ardiuno) ID = 4
	unsigned int ConsumerID = cid;	// Consumer(UIM342 ) ID = 5
	unsigned int ControlWord = cw; // | 0x80 = Need ACK from UIM342
	unsigned long SID = 0;
	unsigned long EID = 0;
	unsigned long CANID = 0;

	SID = ((ConsumerID << 1) & 0x003F) | ((ProducerID << 6) & 0x7C);
	EID = ((ConsumerID << 9) & 0xC000) | ((ProducerID << 11) & 0x30000) | ControlWord;
	CANID = SID << 18 | EID;

	// Prepare CAN frame for transmission
	CanMsg.can_id = CANID | CAN_EFF_FLAG;
	CanMsg.can_dlc = datalen;
	for (int i = 0; i < datalen; i++)
	{
		CanMsg.data[i] = data[i];
	}

	// Send the CAN frame
	mcp2515can.sendMessage(MCP2515::TXB1, &CanMsg);
	delay(2);

	// Wait for the correct response
	while (!checkresult)
	{
		mcp2515can.readMessage(&CanMsg);
		checkid = CanMsg.can_id;
		mcp2515can.readMessage(&CanMsg);
		checkresult = (checkid == CanMsg.can_id);
	}

	// Print received data to the Serial Monitor
	Serial.print("CAN ID: ");
	Serial.println(CanMsg.can_id, HEX);
	Serial.print("Data Length: ");
	Serial.println(CanMsg.can_dlc);
	Serial.print("ML: ");
	for (int i = 0; i < CanMsg.can_dlc; i++)
	{
		Serial.print(CanMsg.data[i], HEX);
		Serial.print(" ");
	}
	Serial.println(" ");

	return CanMsg;
}

/**
 * @brief Reads the Serial Number (SN) configuration.
 * @param can_id CAN ID for the message.
 * @param data_len Length of the data payload.
 * @param data Pointer to the data array.
 * @return CAN frame containing the response.
 */
can_frame SimpleCAN::ReadSNConfig(unsigned char pid, unsigned char cid, unsigned char cw, unsigned char datalen, unsigned char* data)
{
	unsigned long checkid = 0;
	bool checkresult = false;

	// Prepare CAN frame for transmission
	unsigned int ProducerID = pid;	// Producer(Ardiuno) ID = 4
	unsigned int ConsumerID = cid;	// Consumer(UIM342 ) ID = 5
	unsigned int ControlWord = cw; // | 0x80 = Need ACK from UIM342
	unsigned long SID = 0;
	unsigned long EID = 0;
	unsigned long CANID = 0;

	SID = ((ConsumerID << 1) & 0x003F) | ((ProducerID << 6) & 0x7C);
	EID = ((ConsumerID << 9) & 0xC000) | ((ProducerID << 11) & 0x30000) | ControlWord;
	CANID = SID << 18 | EID;

	// Prepare CAN frame for transmission
	CanMsg.can_id = CANID | CAN_EFF_FLAG;
	CanMsg.can_dlc = datalen;
	for (int i = 0; i < datalen; i++)
	{
		CanMsg.data[i] = data[i];
	}

	// Send the CAN frame
	mcp2515can.sendMessage(MCP2515::TXB1, &CanMsg);
	delay(2);
	// Wait for Response
	while (!checkresult) {
		mcp2515can.readMessage(&CanMsg);
		checkid = CanMsg.can_id;
		mcp2515can.readMessage(&CanMsg);
		checkresult = (checkid == CanMsg.can_id);
	}

	// Print received data to the Serial Monitor
	Serial.print("CAN ID: ");
	Serial.println(CanMsg.can_id, HEX);
	Serial.print("Data Length: ");
	Serial.println(CanMsg.can_dlc);
	Serial.print("SN: ");
	for (int i = 0; i < CanMsg.can_dlc; i++)
	{
		Serial.print(CanMsg.data[i], HEX);
		Serial.print(" ");
	}
	Serial.println(" ");

	return CanMsg;
}

/**
 * @brief Reads position parameter (PP) configuration.
 * @param can_id CAN ID for the message.
 * @param data_len Length of the data payload.
 * @param data Pointer to the data array.
 * @return CAN frame containing the response.
 */
can_frame SimpleCAN::ReadPPConfig(unsigned char pid, unsigned char cid, unsigned char cw, unsigned char datalen, unsigned char* data)
{
	unsigned long checkid = 0;
	bool checkresult = false;

	// Prepare CAN frame for transmission
	unsigned int ProducerID = pid;	// Producer(Ardiuno) ID = 4
	unsigned int ConsumerID = cid;	// Consumer(UIM342 ) ID = 5
	unsigned int ControlWord = cw; // | 0x80 = Need ACK from UIM342
	unsigned long SID = 0;
	unsigned long EID = 0;
	unsigned long CANID = 0;

	SID = ((ConsumerID << 1) & 0x003F) | ((ProducerID << 6) & 0x7C);
	EID = ((ConsumerID << 9) & 0xC000) | ((ProducerID << 11) & 0x30000) | ControlWord;
	CANID = SID << 18 | EID;

	// Prepare CAN frame for transmission
	CanMsg.can_id = CANID | CAN_EFF_FLAG;
	CanMsg.can_dlc = datalen;
	for (int i = 0; i < datalen; i++)
	{
		CanMsg.data[i] = data[i];
	}

	// Send the CAN frame
	mcp2515can.sendMessage(MCP2515::TXB1, &CanMsg);
	delay(2);

	// Wait for Response
	while (!checkresult) {
		mcp2515can.readMessage(&CanMsg);
		checkid = CanMsg.can_id;
		mcp2515can.readMessage(&CanMsg);
		checkresult = (checkid == CanMsg.can_id);
	}

	// Print received data to the Serial Monitor
	Serial.print("CAN ID: ");
	Serial.println(CanMsg.can_id, HEX);
	Serial.print("Data Length: ");
	Serial.println(CanMsg.can_dlc);
	Serial.print("PP: ");
	for (int i = 0; i < CanMsg.can_dlc; i++)
	{
		Serial.print(CanMsg.data[i], HEX);
		Serial.print(" ");
	}
	Serial.println(" ");

	return CanMsg;
}

///=============================
//      FUNCTIONS for SET
///=============================

/**
 * @brief Set Parameters to UIM342 Motor
 * @param can_id CAN ID for the message.
 * @param data_len Length of the data payload.
 * @param data Pointer to the data array.
 * @return CAN frame containing the response.
 */
can_frame SimpleCAN::SetToUim342(unsigned long can_id, unsigned char data_len, unsigned char* data)
{
	// Prepare CAN frame for transmission
	CanMsg.can_id = can_id | CAN_EFF_FLAG;
	CanMsg.can_dlc = data_len;

	if (data)
	{
		for (int i = 0; i < data_len; i++) {
			CanMsg.data[i] = data[i];
		}
	}

	// Send the CAN frame
	mcp2515can.sendMessage(MCP2515::TXB1, &CanMsg);
	delay(2);

	// Read response frame
	mcp2515can.readMessage(&CanMsg);
	return CanMsg;
}

can_frame SimpleCAN::CommandSet(unsigned char pid, unsigned char cid, unsigned char cw, unsigned char datalen, unsigned char* data)
{
	// Data0 = 7; ( SubIndex = 7; PP[7] i.e. set/get NodeID) 
	// Data1 = 6; ( New NodeID = 6)
	// SID = ((5 << 1) & 0x003F) | 0x0100    = 0x010A;
	// EID = (((5 << 1) & 0x00C0) << 8) | CW = 0x0081;
	// CAN_ID = 0x010A << 18 | 0x0081 = 0x04280081;
	// DLC   = 0x02;
	// Data0 = 0x07;
	// Data1 = 0x06;

	// Set: PP[7] = 6;
	unsigned int ProducerID = pid;	// Producer(Ardiuno) ID = 4
	unsigned int ConsumerID = cid;	// Consumer(UIM342 ) ID = 5
	unsigned int ControlWord = cw; // | 0x80 = Need ACK from UIM342
	unsigned long SID = 0;
	unsigned long EID = 0;
	unsigned long CANID = 0;

	SID = ((ConsumerID << 1) & 0x003F) | ((ProducerID << 6) & 0x7C);
	EID = ((ConsumerID << 9) & 0xC000) | ((ProducerID << 11) & 0x30000) | ControlWord;
	CANID = SID << 18 | EID;

	// Prepare CAN frame for transmission
	CanMsg.can_id = CANID | CAN_EFF_FLAG;
	CanMsg.can_dlc = datalen;
	for (int i = 0; i < datalen; i++)
	{
		CanMsg.data[i] = data[i];
	}

	// Send the CAN frame
	mcp2515can.sendMessage(MCP2515::TXB1, &CanMsg);
	delay(2);

	// Read response frame
	mcp2515can.readMessage(&CanMsg);
	return CanMsg;
}
