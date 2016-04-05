#include <MCP2510.h>
#include <Canutil.h>
#include <Wire.h>
#include <SPI.h>

#define BLUETOOTH_BAUDRATE 9600

#define DATA_SIZE 3 
#define INIT_SIZE 1
#define CHECKSUM_SIZE 0
#define FULL_SIZE DATA_SIZE + INIT_SIZE + CHECKSUM_SIZE

#define OWN_NODE_ID 0x0010
#define SPEED_ID 0x1
#define CAMERA_ID 0x2
#define SOUND_ID 0x3
#define RASP_MSB_ID 0x20000


MCP2510  can_dev (9); // defines pb1 (arduino pin9) as the _CS pin for MCP2510
Canutil  canutil(can_dev);
//volatile uint16_t msgID;
//volatile int recSize;
//volatile uint8_t canDataReceived[8];
uint8_t txstatus;
byte data_buf[DATA_SIZE];
byte init_buf[INIT_SIZE];
byte checksum_buf[CHECKSUM_SIZE];

bool is_max_value(byte[],int);
void process_serial_data(byte[],uint32_t);

void setup() {
	Serial.begin(BLUETOOTH_BAUDRATE);
	// put your setup code here, to run once:
//	can_dev.write(CANINTE, 0x01); //disables all interrupts but RX0IE (received message in RX buffer 0)
//	can_dev.write(CANINTF, 0x00);  // Clears all interrupts flags

	canutil.setClkoutMode(0, 0); // disables CLKOUT
	canutil.setTxnrtsPinMode(0, 0, 0); // all TXnRTS pins as all-purpose digital input

	canutil.setOpMode(4); // sets configuration mode
	// IMPORTANT NOTE: configuration mode is the ONLY mode where bit timing registers (CNF1, CNF2, CNF3), acceptance
	// filters and acceptance masks can be modified

	Serial.println("waiting opmode");
	canutil.waitOpMode(4);  // waits configuration mode
	Serial.println("opmode received");

	can_dev.write(CNF1, 0x03); // SJW = 1, BRP = 3
	can_dev.write(CNF2, 0b10110001); //BLTMODE = 1, SAM = 0, PHSEG = 6, PRSEG = 1
	can_dev.write(CNF3, 0x05);  // WAKFIL = 0, PHSEG2 = 5

	// SETUP MASKS / FILTERS FOR CAN

	canutil.setOpMode(0); // sets normal mode
}

void loop() {
	while( Serial.available() >= FULL_SIZE ) {
		Serial.readBytes(init_buf,INIT_SIZE);
		if( !is_max_value(init_buf,INIT_SIZE) ) {
			Serial.println("Message is not fine.");
			Serial.flush();
			break;
		}
		Serial.readBytes(checksum_buf,CHECKSUM_SIZE);	// should check it ! but not written yet
		Serial.readBytes(data_buf,DATA_SIZE);
		switch(data_buf[0]) {
			case SPEED_ID:
				Serial.println("speed");
				process_serial_data(data_buf+1,SPEED_ID);
				break;
			case CAMERA_ID:
				Serial.println("camera");
				process_serial_data(data_buf+1,CAMERA_ID+RASP_MSB_ID);
				break;
			case SOUND_ID:
				Serial.println("sound");
				process_serial_data(data_buf+1,SOUND_ID);
		}
	}
}


bool is_max_value(byte buf[], int len) {
	for(int i = 0; i < len ; i++) {
		if(buf[i] != 0xFF)
			return false;
	}
	return true;
}	

void process_serial_data(byte data[],uint32_t id_ext) {
	uint8_t message[8];
	message[0] = data[0];
	message[1] = data[1];

	canutil.setTxBufferID(OWN_NODE_ID, id_ext , 1, 0); // sets the message ID, specifies standard message (i.e. short ID) with buffer 0
	canutil.setTxBufferDataLength(0, 2, 0);
	canutil.setTxBufferDataField(message, 0);   // fills TX buffer
	//      canutil.setTxBufferDataField(message, 0);   // fills TX buffer
	canutil.messageTransmitRequest(0, 1, 3);
	Serial.println("about to send message");
	do {
		txstatus = 0;
		txstatus = canutil.isTxError(0);  // checks tx error
		txstatus = txstatus + canutil.isArbitrationLoss(0);  // checks for arbitration loss
		txstatus = txstatus + canutil.isMessageAborted(0);  // ckecks for message abort
		txstatus = txstatus + canutil.isMessagePending(0);   // checks transmission
	}
	while (txstatus != 0);
	Serial.println("message sent");
}
