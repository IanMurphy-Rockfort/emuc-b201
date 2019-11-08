#ifndef __LIB_EMUC_H__
#define __LIB_EMUC_H__ 

/*------------------------------------------------------------------------------------------------------------*/
#define MAX_COM_PORT    16
#define MAX_BUF         512

#define ID_LEN          4
#define DATA_LEN        8   
#define VER_LEN         16

#define DIM(var, type)  (sizeof(var)) / sizeof(type)

#define BOOL    bool
#define TRUE    true
#define FALSE   false

/*------------------------------------------------------------------------------------------------------------*/
enum
{
    /* channel */
    EMUC_CH_1           = 1,
    EMUC_CH_2,
    EMUC_CH_BOTH,

    /* baudrate */
    EMUC_BAUDRATE_5K    = 0,
    EMUC_BAUDRATE_10K,
    EMUC_BAUDRATE_20K,
    EMUC_BAUDRATE_50K,
    EMUC_BAUDRATE_125K,
    EMUC_BAUDRATE_250K,
    EMUC_BAUDRATE_500K,
    EMUC_BAUDRATE_1M,
};

/*------------------------------------------------------------------------------------------------------------*/
typedef struct
{
    int           com_port;

    int           channel;
    int           mod;
    int           rtr;
    int           dlc;

    unsigned char id[ID_LEN];
    unsigned char data[DATA_LEN];

} DATA_INFO;

/*------------------------------------------------------------------------------------------------------------*/
typedef struct
{
    char fw[VER_LEN];
    char api[VER_LEN];
} VER_INFO;

/*------------------------------------------------------------------------------------------------------------*/
typedef struct
{
    int           channel;
    int           mod;
    int           rtr;
    int           dlc;

    unsigned char id[ID_LEN];
    unsigned char data[DATA_LEN];

    unsigned char com_buf[64];

} SOCKET_CAN_INFO;

/*------------------------------------------------------------------------------------------------------------*/
extern int  EMUCOpenDevice(int port);
extern void EMUCCloseDevice(int port);
extern int  EMUCShowVer(int port, VER_INFO *ver);
extern int  EMUCSetCAN(int port, int ch, int bdrate);
extern int  EMUCReset(int port);
extern int  EMUCReceive(DATA_INFO *info);
extern int  EMUCSend(DATA_INFO *info);

extern void EMUCSendHex(SOCKET_CAN_INFO *info);
extern BOOL EMUCRevHex(SOCKET_CAN_INFO *info);
extern void SetCANHex(SOCKET_CAN_INFO *info, int ch, int bdrate);

#endif