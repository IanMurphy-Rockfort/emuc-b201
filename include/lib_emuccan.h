#ifndef __LIB_EMUC_CAN_H__
#define __LIB_EMUC_CAN_H__ 

#define ID_LEN          4
#define DATA_LEN        8   

enum
{
	/* channel */
	EMUC_CH_1 = 1,
	EMUC_CH_2,
	EMUC_CH_BOTH,
	
	/* baudrate */
	EMUC_CANSPEED_5K = 0,
	EMUC_CANSPEED_10K,
	EMUC_CANSPEED_20K,
	EMUC_CANSPEED_50K,
	EMUC_CANSPEED_125K,
	EMUC_CANSPEED_250K,
	EMUC_CANSPEED_500K,
	EMUC_CANSPEED_1M,
};

typedef struct
{
    int channel;
    int mod;
    int rtr;
    int dlc;

    unsigned char id[ID_LEN];
    unsigned char data[DATA_LEN];

    char com_buf[64];

} EMUC_CAN_FRAME;

extern void EMUCSetCANHex(int ch, int speed);
extern void EMUCSendHex(EMUC_CAN_FRAME *frame);
extern int  EMUCRevHex(EMUC_CAN_FRAME *frame);

#endif

