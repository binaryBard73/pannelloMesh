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
/*------------------------------------------------------------------------------*/
/*    VARIABILI EXTERN                                                          */
/*------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*/
/*    VARIABILI                                                                 */
/*------------------------------------------------------------------------------*/
volatile uint32_t mbus_timeout;
//======================== VARIABILI MESH ======================================//
// Scheduler MyScheduler; // to control your personal task
// painlessMesh  mesh;
/*------------------------------------------------------------------------------*/
/*    PROTOTIPI                                                                 */
/*------------------------------------------------------------------------------*/

/********************************************************************************/
void SerialMesh::init(void)
{
	SerialMesh::busy = false;
	if (SerialMesh::my_addr.total == MASTER_ADDR) {
		SerialMesh::half_duplex = true;
		Log(COMMUNICATION, "Parto come master\n");
		//=============================================================//
		// Finchè non trovo il primo nodo, tengo stoppate le richieste //
		// modbus. Appena trovo un nodo, abbasso il pin e procedo a    //
		// inviare le richieste. NOTA: QUESTO PIN SERVE ANCHE PER LA   //
		// COMUNICAZIONE IN HALF DUPLEX (ES. SLAVE) BE CAREFUL         //
		//=============================================================//

	} else {
		SerialMesh::half_duplex = false;
		Log(COMMUNICATION, "Parto come slave\n");
	}
}

bool SerialMesh::sendReady(void)
{
	data.len = 6;
	data.add = 1;
	data.func = 0x06;
	data.data_485.buffer_rx.cbuf_rx[0] = 3;
	data.data_485.buffer_rx.cbuf_rx[1] = 130;
	data.data_485.buffer_rx.cbuf_rx[2] = 0;
	data.data_485.buffer_rx.cbuf_rx[3] = 1;

	uint16_t crc = SerialMesh::calcCRC((uint8_t *)&SerialMesh::data.add,
					   SerialMesh::data.len, 0xFFFF);

	SerialMesh::pchar = (uint8_t *)&SerialMesh::data.add;
	memcpy(SerialMesh::pchar + SerialMesh::data.len, (uint8_t *)&crc, 2);

	Serial2.write(SerialMesh::pchar, SerialMesh::data.len + 2);
	Serial2.flush();
	SerialMesh::busy = false;

	return (true);
}

bool SerialMesh::serialToMeshMaster(void)
{
	CRC my_crc, pkt_crc;
	uint16_t old_len, len;
	// uint8_t *p;
	uint32_t mini_timeout;

	if (SerialMesh::busy ==
	    false) // busy = false = non aspetto risposte come master
	{
		// Controllo se ci sono dati presenti: se ce ne sono, entro nel loop
		// e aspetto di riceverli tutti.
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

		// len = Serial2.available();
		// if(len == 0)
		//     return(false);

		if (len < sizeof(SerialMesh::data)) {
			Log(COMMUNICATION, "len dati seriali ricevuti: %d\n",
			    len);
			// Leggo i dati dalla seriale
			Serial2.readBytes((uint8_t *)&SerialMesh::data.add,
					  len);
			//=======================================================//
			// Stoppo le richieste fintanto che non ho ricevuto una  //
			// risposta o sono andato in timeout / err               //
			//=======================================================//

			SerialMesh::pchar = &SerialMesh::data.add;
			memcpy((uint8_t *)&pkt_crc.i_crc,
			       SerialMesh::pchar + len - 2, 2);
			my_crc.i_crc = SerialMesh::calcCRC(SerialMesh::pchar,
							   len - 2, 0xFFFF);
			if (my_crc.i_crc == pkt_crc.i_crc) {
				SerialMesh::data.trans_ID = SerialMesh::id_cnt;
				SerialMesh::data.protocol_ID = 0;
				SerialMesh::data.len = len - 2;
				Log(COMMUNICATION, "mbus.len: %d\n", data.len);

				SerialMesh::busy = true;
				mbus_timeout =
					set_ttimeout_sec_M(MESH_MBUS_TIMEOUT);
				SerialMesh::tx_transID = SerialMesh::id_cnt;
				SerialMesh::id_cnt++;
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
			// SerialMesh::task->enable();
			SerialMesh::busy = false;
		}
	}
	return (false);
}

bool SerialMesh::meshToSerialMaster(JsonObject root)
{
	// mbus.trans_ID = root["transID"];
	rx_transID = root["transID"];
	if (rx_transID != tx_transID) {
		Log(COMMUNICATION, "TransID ERROR: inviato; %d, ricevuto: %d\n",
		    tx_transID, rx_transID);
		SerialMesh::busy = false;

		return (false);
	}
	data.trans_ID = rx_transID;
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
		uint32_t mute_time = set_ttimeout(2);
		while (chk_timeout(mute_time) == 0)
			;
	}
	Serial2.write(SerialMesh::pchar, SerialMesh::data.len + 2);
	Serial2.flush();
	SerialMesh::busy = false;
	if (SerialMesh::half_duplex == false) {
		uint32_t mute_time = set_ttimeout(2);
		while (chk_timeout(mute_time) == 0)
			;
	}

	return (true);
}

bool SerialMesh::loop(void)
{
	if (SerialMesh::busy == true) {
		if (chk_timeout(SerialMesh::comm_timeout) == true) {
			SerialMesh::busy = false;
			// Serial.println("timeout scaduto");
			return (true);
		}
	}
	return (false);
}

uint16_t SerialMesh::calcCRC(uint8_t *lpCMD, uint16_t num_char,
			     uint16_t checksum)
{
#define POLINOMIO 0xA001
#define NUM_SHIFT 8

	uint16_t cnt;
	uint8_t shift;
	uint8_t carat;

	for (cnt = 0; cnt < num_char; cnt++) {
		carat = *lpCMD;
		checksum ^= (uint16_t)carat;
		lpCMD++;
		for (shift = 1; shift < NUM_SHIFT + 1; shift++) {
			if ((checksum & 0x0001) == 0)
				checksum = checksum >> 1;
			else
				checksum = ((checksum >> 1) ^ POLINOMIO);
		}
	}
	return (checksum);
}
