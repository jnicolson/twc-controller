#ifndef TWC_PROTOCOL_H
#define TWC_PROTOCOL_H


#define GET_SERIAL_NUMBER	0xFB19
#define GET_MODEL_NUMBER	0xFB1A
#define GET_FIRMWARE_VER 	0xFB1B
#define GET_PLUG_STATE		0xFBB4

#define MASTER_HEATBEAT		0xFBE0
#define LINKREADY2		0xFBE2

#define GET_VIN_FIRST		0xFBEE
#define GET_VIN_MIDDLE		0xFBEF
#define GET_VIN_LAST		0xFBF1

// Commands without responses (0xFC)
#define START_CHARGING		0xFCB1
#define STOP_CHARGING		0xFCB2
#define LINKREADY1		    0xFCE1

// Responses (0xFD)
#define RESP_SERIAL_NUMBER	0xFD19
#define RESP_MODEL_NUMBER	0xFD1A
#define RESP_FIRMWARE_VER	0xFD1B
#define RESP_PLUG_STATE		0xFDB4

#define SLAVE_HEARTBEAT		0xFDE0

#define RESP_LINK_READY		0xFDE2		// Sent by slave on reset
#define RESP_PWR_STATUS		0xFDEB		// Sent by master on reset
#define RESP_VIN_FIRST		0xFDEE
#define RESP_VIN_MIDDLE		0xFDEF
#define RESP_VIN_LAST		0xFDF1


#endif /* TWC_PROTOCOL_H */