/*------------------------------------------------------------------------------*/
/*                                                                              */
/* FILE NAME: masterModbus.cpp                                                  */
/*                                                                              */
/* PURPOSE:   funzioni classe SerialMesh lato Master.                           */
/*                                                                              */
/* AUTHORS:   Christian Corsetti                                                */
/*                                                                              */
/* VERSION:   0.00                                                              */
/*                                                                              */
/* FILE REFERENCES:                                                             */
/*                                                                              */
/* Name                     IO        Description                               */
/* --------------           --        -------------                             */
/* none                                                                         */
/*                                                                              */
/* FUNCTION REFERENCES:                                                         */
/*                                                                              */
/* Name                         Description                                     */
/* --------------               -------------                                   */
/*                                                                              */
/* EXTERNAL VARIABLES:                                                          */
/*                                                                              */
/* Source:                                                                      */
/*                                                                              */
/* Name                     Type            IO         Description              */
/* -----------              ----            ---        ------------             */
/* None                                                                         */
/*                                                                              */
/* EXTERNAL REFERENCES:                                                         */
/*                                                                              */
/* Name                                     Description                         */
/* -----------                              ------------                        */
/*                                                                              */
/*                                                                              */
/* ERROR AND WARNING MESSAGES:                                                  */
/*                                                                              */
/* NOTES:                                                                       */
/*                                                                              */
/* HISTORY:                                                                     */
/* Date         Name               Release     Description                      */
/* ----------   -------------      -------     -------------                    */
/* 05-12-2019   Christian Corsetti alfa        prima versione del progetto      */
/*------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------*/
/*    INCLUDE                                                                   */
/*------------------------------------------------------------------------------*/
#include <Arduino.h>
#include "modbus.hpp"
// #include "AutoConnect.h"
#include "tmr.h"

using namespace modbus;

extern volatile uint32_t mbus_timeout;
/*------------------------------------------------------------------------------*/
/*    VARIABILI EXTERN                                                          */
/*------------------------------------------------------------------------------*/

bool SerialMesh::serialToMeshSlave(void)
{
	CRC my_crc, pkt_crc;
	uint16_t old_len, len;
	// uint8_t *p;
	uint32_t mini_timeout;

	// Log(COMMUNICATION, "%s\n", busy ? "true" : "false");
	// busy = false = non aspetto risposte
	if (SerialMesh::busy == true) {
		old_len = Serial2.available();

		if (old_len == 0)
			return (false);

		len = old_len;
		// mini_timeout = set_ttimeout_sec_M(50);
		mini_timeout = set_ttimeout_sec_M(5);
		while (!chk_timeout(mini_timeout)) {
			len = Serial2.available();
			if (len != old_len) {
				old_len = len;
				mini_timeout = set_ttimeout_sec_M(5);
			}
		}

		if (len < sizeof(SerialMesh::data)) {
			Log(COMMUNICATION, "len dati seriali ricevuti: %d\n",
			    len);
			Serial2.readBytes((uint8_t *)&SerialMesh::data.add,
					  len);
			SerialMesh::pchar = &SerialMesh::data.add;
			memcpy((uint8_t *)&pkt_crc.i_crc,
			       SerialMesh::pchar + len - 2, 2);
			my_crc.i_crc = SerialMesh::calcCRC(SerialMesh::pchar,
							   len - 2, 0xFFFF);
			if (my_crc.i_crc == pkt_crc.i_crc) {
				SerialMesh::data.trans_ID =
					SerialMesh::rx_transID;
				SerialMesh::data.protocol_ID = 0;
				SerialMesh::data.len = len - 2;
				Log(COMMUNICATION, "data.len: %d\n", data.len);

				SerialMesh::busy = false;
				return (true);
			} else {
				Log(COMMUNICATION,
				    "---- CRC ERROR!!!!!!! ----\n");
				SerialMesh::busy = false;
			}

		} else {
			Log(COMMUNICATION, "---- MODBUS LEN ERROR!!!! ----\n");
			while (Serial2.available()) {
				Serial2.read();
			}
			SerialMesh::busy = false;
		}
	}
	return (false);
}

bool SerialMesh::bridgeMeshToSerial(void)
{
	Log(DEBUG, "Dati da inviare a seriale:\n");
	for (int i = 0; i < SerialMesh::rxlen; i++) {
		Log(DEBUG, "%02X ", SerialMesh::rxSerial[i]);
	}
	Log(DEBUG, "\n");

	SerialMesh::pchar = (uint8_t *)SerialMesh::rxSerial;
	Serial2.write(SerialMesh::pchar, SerialMesh::rxlen);
	Serial2.flush();
	SerialMesh::busy = false;
	return (false);
}

bool SerialMesh::bridgeSerialToMesh(void)
{
	uint16_t old_len, len;
	uint32_t mini_timeout;

	// Serial.printf("len dati seriali ricevuti: %d\n", SerialMesh::busy);
	if (SerialMesh::busy == false) {
		old_len = Serial2.available();
		if (old_len == 0)
			return (false);

		len = old_len;

		mini_timeout = set_ttimeout_sec_M(5);
		while (!chk_timeout(mini_timeout)) {
			len = Serial2.available();
			if (len != old_len) {
				old_len = len;
				mini_timeout = set_ttimeout_sec_M(5);
			}
		}

		if (len < sizeof(SerialMesh::txSerial)) {
			SerialMesh::txlen = len;

			Log(DEBUG, "len dati seriali ricevuti: %d\n",
			    SerialMesh::txlen);

			Serial2.readBytes(SerialMesh::txSerial, len);
			SerialMesh::txSerial[len] = '\0';

			Log(DEBUG, "Dati da inviare:\n");
			for (int i = 0; i < len; i++) {
				Log(DEBUG, "%02X ", SerialMesh::txSerial[i]);
			}
			Log(DEBUG, "\n");

			SerialMesh::comm_timeout =
				set_ttimeout_sec_M(MESH_MBUS_TIMEOUT);
			SerialMesh::busy = true;
			return (true);
		} else {
			while (Serial2.available()) {
				Serial2.read();
			}
			SerialMesh::busy = false;
		}
	}
	return (false);
}

bool SerialMesh::meshToSerialSlave(JsonObject root)
{
	rx_transID = root["transID"];
	data.protocol_ID = root["protocol"];
	data.len = root["len"];
	data.add = root["add"];
	data.func = root["func"];

	for (uint8_t i = 0; i < data.len - 2;
	     i++) // -2 perchè ho già tolto add e func.
	{
		if (root.containsKey(String(i))) {
			data.data_485.buffer_rx.cbuf_rx[i] = root[String(i)];
		}
	}

	uint16_t crc = SerialMesh::calcCRC((uint8_t *)&SerialMesh::data.add,
					   SerialMesh::data.len, 0xFFFF);

	SerialMesh::pchar = (uint8_t *)&SerialMesh::data.add;
	memcpy(SerialMesh::pchar + SerialMesh::data.len, (uint8_t *)&crc, 2);

	if (SerialMesh::half_duplex == false) {
		uint32_t mute_time = set_ttimeout(1);
		while (chk_timeout(mute_time) == 0)
			;
	}
	Serial2.write((uint8_t *)SerialMesh::pchar, SerialMesh::data.len + 2);
	Serial2.flush();
	if (SerialMesh::half_duplex == false) {
		uint32_t mute_time = set_ttimeout(1);
		while (chk_timeout(mute_time) == 0)
			;
	}
	SerialMesh::busy = true;
	mbus_timeout = set_ttimeout(MESH_MBUS_TIMEOUT);
}