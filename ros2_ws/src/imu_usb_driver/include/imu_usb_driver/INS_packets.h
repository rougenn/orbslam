#ifndef INS_PACKETS_H
#define INS_PACKETS_H

// TYPES
#define _SINT8     signed char
#define _UINT8     unsigned char
#define _SINT16    signed short
#define _UINT16    unsigned short
#define _SINT32    signed long
#define _UINT32    unsigned long
#define _FLOAT32   float
#define _DOUBLE64  double

// SCALE FACTORS FOR DATA
#define ANGLES_SF       (_FLOAT32)0.01
#define SPEED_SF        (_FLOAT32)0.01
#define ANG_VEL_SF      (_FLOAT32)0.01
#define ACCEL_SF        (_FLOAT32)0.001
#define MAGN_INT_SF     (_FLOAT32)0.01

#define INS_PACKETS_LIST_SIZE (_UINT32)32

#pragma pack(1)
typedef struct {
  _UINT16 Header; // 0xABCD
  _UINT16 PacketIndex;
  _UINT16 Flag;               // 0x01 – Alignment, 0x02 – Moving
  _UINT16 Multiplexor_10Hz;
  _SINT16 Roll_deg, Pitch_deg,
          GyroMagnHeading_deg,
          MagnHeading_deg,
          Speed_m_s;
  _SINT16 OmegaB_deg_s[3],
          fB_m_s_s[3],
          MagnInt_MicroTesla[3];
  _UINT8  Reserved[4];
  _UINT16 CheckSum_XOR_16;
} TOutINSPacket;
#pragma pack()

#pragma pack(1)
typedef struct {
  _FLOAT32 Roll_deg, Pitch_deg,
           GyroMagnHeading_deg,
           MagnHeading_deg,
           Speed_m_s;
  _FLOAT32 OmegaB_deg_s[3],
           fB_m_s_s[3],
           MagnInt_MicroTesla[3];
} TINSData;
#pragma pack()

extern _UINT32 INSPacketsCounter;
extern TOutINSPacket INSPacketsList[INS_PACKETS_LIST_SIZE];

extern void ProcessReceivedByteFromUART(_UINT8 ch);
extern void GetINSDataFromOutINSPacket(TINSData *pINSData, TOutINSPacket *pINSPacket);

#endif  // INS_PACKETS_H
