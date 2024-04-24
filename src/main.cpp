/*------------------------------------------------------------------------------*/
/*                                                                              */
/* FILE NAME: main.cpp                                                          */
/*                                                                              */
/* PURPOSE:   pagina principale progetto mesh_ap                                */
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

using namespace std;
using namespace modbus;
/*------------------------------------------------------------------------------*/
/*    VARIABILI EXTERN                                                          */
/*------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*/
/*    VARIABILI                                                                 */
/*------------------------------------------------------------------------------*/

uint32_t logServerId;
uint32_t npkg;

// bool (SerialMesh::*serialToMesh)(void);
// bool (SerialMesh::*meshToSerial)(JsonObject root);
//======================= VARIABILI MODBUS =====================================//
SerialMesh mbus;

//======================== VARIABILI MESH ======================================//
Scheduler userScheduler; // to control your personal task
painlessMesh mesh;
uint8_t nodes;
uint32_t first_node;
uint32_t npkg_tx;
uint32_t npkg_rx;
uint32_t timeTx;
uint32_t timeRx;
uint32_t timeTotal;
/*------------------------------------------------------------------------------*/
/*    PROTOTIPI                                                                 */
/*------------------------------------------------------------------------------*/
void MeshToSerialCallback(uint32_t from, String &msg);
void SlaveReceivedCallback(uint32_t from, String &msg);
void sendMasterMessage(void);
void sendSlaveMessage(void);

#define TASK_SEND TASK_MILLISECOND * 10
#define MASTER 1

// Task taskSendSlaveMessage( TASK_MILLISECOND * 2, TASK_FOREVER, &sendSlaveMessage);
// Task taskSendMasterMessage( TASK_MILLISECOND * 2, TASK_FOREVER, &sendMasterMessage);

void sendMessage();
// Task taskSendReady(TASK_SECOND * 5, TASK_FOREVER, &sendMessage);
Task taskSendReady(TASK_SEND, TASK_FOREVER, &sendMessage);

void sendMessage(void)
{
	std::list<uint32_t> nodes = mesh.getNodeList(false);

#ifdef MASTER
	if ((nodes.size() > 0) && (digitalRead(SCLK) == LOW)) {
		mbus.sendReady();
	}
#endif

	if (mbus.bridgeSerialToMesh() == true) {
		Log(DEBUG, "Dati da inviare formato stringa:\n");
		for (int i = 0; i < mbus.txlen; i++) {
			Log(DEBUG, "%02X ", mbus.txSerial[i]);
		}
		Log(DEBUG, "\n");

		uint8_t buffer[100];
		memcpy(buffer, mbus.txSerial, mbus.txlen);

		DynamicJsonDocument jsonBuffer(1024);
		JsonArray meshMsg = jsonBuffer.to<JsonArray>();

		for (uint8_t i = 0; i < mbus.txlen; i++) {
			meshMsg.add(buffer[i]);
		}
		String json;
		serializeJson(meshMsg, json);

		Log(DEBUG, "Dati json: %s\n", json.c_str());

		npkg_tx++;
		timeTx = millis();

		Log(DEBUG, "Pkt inviato su mesh n. %d\n", npkg_tx);
		Log(DEBUG, "==========================================\n");
		Log(DEBUG, "\n");

		mesh.sendBroadcast(json);
	}

	taskSendReady.setInterval(TASK_SEND);
}
/********************************************************************************/
void MeshToSerialCallback(uint32_t from, String &msg)
{
	String mesh_data;

	Log(DEBUG, "Len RX: %d\n", msg.length());
	Log(DEBUG, "Dati RX: ");
	for (int i = 0; i < msg.length(); i++) {
		Log(DEBUG, "%02X ", msg.c_str()[i]);
	}
	Log(DEBUG, "\n");

	DynamicJsonDocument doc(1024 + msg.length());
	DeserializationError error = deserializeJson(doc, msg);
	if (error) {
		Log(DEBUG, "DeserializationError\n");
		return;
	}

	JsonArray rxData = doc.as<JsonArray>();
	mbus.rxlen = rxData.size();

	Log(DEBUG, "Dati deserializzati in Array: ");
	uint8_t k = 0;
	for (JsonVariant v : rxData) {
		Log(DEBUG, "%d ", v.as<String>());
		mbus.rxSerial[k] = v.as<String>().toInt();
		k++;
	}

	mbus.bridgeMeshToSerial();
}

void newConnectionCallback(uint32_t nodeId)
{
	Log(DEBUG, "Trovato nuovo nodo: %u\n", nodeId);

	std::list<uint32_t> nodes = mesh.getNodeList(false);
#ifdef MASTER
	if (digitalRead(SCLK) == LOW) {
		mbus.sendReady();
	}
#endif

	mbus.mesh_map.push_back(nodeId);
}

void changedConnectionCallback()
{
	Serial.printf("Changed connections\n");

	mesh.getNodeList();
}

void nodeTimeAdjustedCallback(int32_t offset)
{
	Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),
		      offset);
}

void setup()
{
	//-- Serial init -------------------------------------------------------//
	Serial.begin(115200);
	Serial2.begin(38400);

	//-- IO init -----------------------------------------------------------//
	// pinMode(PAIRING, INPUT_PULLUP);         //board stefano
	pinMode(SCLK, INPUT);

	//-- Mesh Init --------------------------------------------------------//

	// Da usare per abilitare le funzioni di DEBUG
	//mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
	// mesh.setDebugMsgTypes(DEBUG);
	mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);

	mesh.onNewConnection(&newConnectionCallback);
	mesh.onChangedConnections(&changedConnectionCallback);
	mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

	//-- System timer interrupt init ---------------------------------------//
	init_timeout_interrupt();

	mesh.onReceive(&MeshToSerialCallback);

	userScheduler.addTask(taskSendReady);
	taskSendReady.enable();
}

void loop()
{
	// it will run the user scheduler as well
	mesh.update();
	mbus.loop();
}