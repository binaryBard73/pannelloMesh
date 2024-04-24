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
#ifndef MODBUS_MESH
#define MODBUS_MESH
/*------------------------------------------------------------------------------*/
/*    INCLUDE                                                                   */
/*------------------------------------------------------------------------------*/
#include "painlessMesh.h"
/*------------------------------------------------------------------------------*/
/*    DEFINE                                                                    */
/*------------------------------------------------------------------------------*/
#define SERIAL_DEBUG
#define FIRMWARE_VERSION 0.00

#define TCPIDENT 6 /* Lunghezza dell'identazione del pacchetto ModbusTcp.   */
#define WAIT_TIME 5 /* Tempo di attesa prima di abbandonare la connessione   */
/* sul socket (valore in Sec).                           */
#define MINLENGTH_MODBUSTCP \
	2 + TCPIDENT /* Lunghezza minima di un pacchetto ModbusTcp.           */
#define MAXLENGTH_MODBUSTCP \
	512 + TCPIDENT /* Lunghezza massima di un pacchetto ModbusTcp.          */
#define LENGTH_FRAME \
	MAXLENGTH_MODBUSTCP /* Lunghezza dei buffer della struttura dati MODBUS_TCP. */

#define PAIRING 22 // Usato per switch tra Soft-AP e single mesh
#define SCLK 18 // Clock modbus half-duplex
//===========================================================================//
// Se nessun Jumper è collegato (address totale = 15) il dispositivo parte   //
// come master, altrimenti si setta come slave.                              //
//===========================================================================//
#define ADDR1 34 // Jumper 1
#define ADDR2 35 // Jumper 2
#define ADDR3 32 // Jumper 3
#define ADDR4 33 // Jumper 4

#define MASTER_ADDR 15
#define MESH_MBUS_TIMEOUT 490 // 70 con questo valore no CRC error

#define MAX_SRV_CLIENTS 3 // Max client in caso di partenza come soft ap

#define MESH_PREFIX "ALTAIR-RD2"
#define MESH_PASSWORD "12345678"
#define MESH_PORT 5555

/*------------------------------------------------------------------------------*/
/*    TYPEDEF                                                                   */
/*------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/* MODBUS_TCP                                                                */
/*---------------------------------------------------------------------------*/
typedef struct __attribute__((packed)) tagMODBUS_TCP {
	uint16_t trans_ID; /* Identificatore di transazione. */
	uint16_t protocol_ID; /* Identificatore di transazione. */
	uint16_t len; /* Lunghezza del pacchetto.       */
	unsigned char add; /* Indirizzo dello strumento.     */
	unsigned char func; /* Funzione dello slave.          */
	union __attribute__((packed)) dat_485 {
		/*-------------------------------------------------------------------*/
		/* Struttura dati di ricezione.                                      */
		/*-------------------------------------------------------------------*/
		union buff_rx {
			uint8_t cbuf_rx[LENGTH_FRAME];
			uint16_t ibuf_rx[LENGTH_FRAME / 2];
			uint32_t lbuf_rx[LENGTH_FRAME / 4];
			float fbuf_rx[LENGTH_FRAME / 4];

		} buffer_rx;

		/*-------------------------------------------------------------------*/
		/* Struttura dati di trasmissione.                                   */
		/*-------------------------------------------------------------------*/
		struct __attribute__((packed)) buff_tx {
			unsigned char byte_count;
			union databuff_tx {
				uint8_t cbuf_tx[LENGTH_FRAME];
				uint16_t ibuf_tx[LENGTH_FRAME / 2];
				uint32_t lbuf_tx[LENGTH_FRAME / 4];
				float fbuf_tx[LENGTH_FRAME / 4];
			} data_tx;
		} buffer_tx;
	} data_485;
	int crc; /* Crc (redundancy check). */
} MODBUS_TCP;

/*---------------------------------------------------------------------------*/
/* union per passaggio CRC da intero a byte e viceversa                      */
/*---------------------------------------------------------------------------*/
typedef union __attribute__((packed)) tagCRC {
	unsigned char crc[2];
	unsigned short i_crc;
} CRC;

typedef struct {
	uint8_t bit0 : 1;
	uint8_t bit1 : 1;
	uint8_t bit2 : 1;
	uint8_t bit3 : 1;
} byte_addr;

typedef union {
	byte_addr bits;
	uint16_t total : 4;
} addr_modulo;

namespace modbus
{
class SerialMesh {
    public:
	void init(void);
	bool sendReady(void);
	bool bridgeSerialToMesh(void);
	bool bridgeMeshToSerial(void);
	bool meshToSerialMaster(JsonObject root);
	bool meshToSerialSlave(JsonObject root);
	bool serialToMeshMaster(void);
	bool serialToMeshSlave(void);
	bool loop(void);
	addr_modulo my_addr; // somma dei valori Jumper per set dell'address
	std::list<uint32_t> mesh_map;
	MODBUS_TCP data;
	uint8_t rxSerial[LENGTH_FRAME];
	uint8_t txSerial[LENGTH_FRAME];
	uint16_t txlen;
	uint16_t rxlen;

    private:
	uint16_t calcCRC(uint8_t *lpCMD, uint16_t num_char, uint16_t checksum);
	// Task *task;
	bool half_duplex;
	bool busy;
	// uint32_t timeout;
	uint32_t tx_transID;
	uint32_t id_cnt;
	uint32_t rx_transID;
	uint8_t *pchar;
	uint32_t comm_timeout;
};
}
#endif