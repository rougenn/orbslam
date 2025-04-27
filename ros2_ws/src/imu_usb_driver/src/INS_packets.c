#include "imu_usb_driver/INS_packets.h"
#include <string.h>
#include <stdint.h>   // для uint32_t

_UINT32 INSPacketsCounter = 0;
TOutINSPacket INSPacketsList[INS_PACKETS_LIST_SIZE];

TOutINSPacket FoundINSPacket;
_UINT8 TempBuffer[sizeof(TOutINSPacket)];

// -----------------------------------------------------------------------------
// CheckSum calculation
_UINT16 ML_CheckSum_Calc_XOR16(void *packet, _UINT32 length)
{
  _UINT16 x16 = 0;
  _UINT32 word_max = length/2 - 1;
  for (_UINT32 i = 0; i < word_max; ++i) {
    x16 ^= ((_UINT16*)packet)[i];
  }
  return x16;
}

// -----------------------------------------------------------------------------
// Добавляем в кольцевой буфер
void AddFoundPacketToList(TOutINSPacket *pINSPacket)
{
  uint32_t idx = INSPacketsCounter % INS_PACKETS_LIST_SIZE;
  memcpy(&INSPacketsList[idx], pINSPacket, sizeof(TOutINSPacket));
  INSPacketsCounter++;
}

// -----------------------------------------------------------------------------
// Парсим каждый байт
void ProcessReceivedByteFromUART(_UINT8 ch)
{
  // сдвигаем буфер
  for (size_t i = 1; i < sizeof(TempBuffer); ++i) {
    TempBuffer[i-1] = TempBuffer[i];
  }
  TempBuffer[sizeof(TempBuffer)-1] = ch;

  // проверяем заголовок 0xABCD (в памяти как CD AB)
  if (TempBuffer[0]==0xCD && TempBuffer[1]==0xAB) {
    _UINT16 pkt_cs = (_UINT16)TempBuffer[sizeof(TempBuffer)-1] << 8
                   | (_UINT16)TempBuffer[sizeof(TempBuffer)-2];
    _UINT16 calc_cs = ML_CheckSum_Calc_XOR16(TempBuffer, sizeof(TempBuffer));
    if (pkt_cs == calc_cs) {
      memcpy(&FoundINSPacket, TempBuffer, sizeof(FoundINSPacket));
      AddFoundPacketToList(&FoundINSPacket);
    }
  }
}

// -----------------------------------------------------------------------------
// Распаковка и масштабирование
_FLOAT32 UnPack_SINT16Value(_SINT16 v, _FLOAT32 sf) {
  return (_FLOAT32)v * sf;
}

void GetINSDataFromOutINSPacket(TINSData *d, TOutINSPacket *p)
{
  d->Roll_deg  = UnPack_SINT16Value(p->Roll_deg, ANGLES_SF);
  d->Pitch_deg = UnPack_SINT16Value(p->Pitch_deg, ANGLES_SF);
  d->GyroMagnHeading_deg = UnPack_SINT16Value(p->GyroMagnHeading_deg, ANGLES_SF) + 180.0f;
  d->MagnHeading_deg     = UnPack_SINT16Value(p->MagnHeading_deg, ANGLES_SF) + 180.0f;
  d->Speed_m_s           = UnPack_SINT16Value(p->Speed_m_s, SPEED_SF);
  for (int i = 0; i < 3; ++i) {
    d->OmegaB_deg_s[i]       = UnPack_SINT16Value(p->OmegaB_deg_s[i], ANG_VEL_SF);
    d->fB_m_s_s[i]           = UnPack_SINT16Value(p->fB_m_s_s[i], ACCEL_SF);
    d->MagnInt_MicroTesla[i] = UnPack_SINT16Value(p->MagnInt_MicroTesla[i], MAGN_INT_SF);
  }
}
