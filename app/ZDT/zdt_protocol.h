#ifndef __ZDT_PROTOCOL_H
#define __ZDT_PROTOCOL_H

#include <stdint.h>
#include "zdt_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ZDT_CMD_ENABLE   0xF3U
#define ZDT_CMD_VEL      0xF6U
#define ZDT_CMD_POS      0xFDU
#define ZDT_CMD_STOP     0xFEU
#define ZDT_CMD_ZERO     0x0AU
#define ZDT_CMD_READ_VER   0x1FU
#define ZDT_CMD_READ_POS   0x36U
#define ZDT_CMD_READ_VEL   0x35U
#define ZDT_CMD_READ_FLG   0x3AU

#define ZDT_ACK_OK       0x02U
#define ZDT_TAIL         0x6BU

typedef struct
{
  uint8_t  used;
  uint8_t  addr;
  uint8_t  dir;
  uint8_t  acc;
  uint8_t  mode;
  uint8_t  snF;
  uint16_t vel;
  int32_t  pos_clk;
} ZdtSlot;

typedef struct
{
  uint8_t addr;
  uint8_t cmd;
  uint8_t status;
} ZdtAck;

/* Emm 0x36 realtime position response (motor encoder units, 65536/rev). */
typedef struct
{
  uint8_t  addr;
  int32_t  units;
} ZdtPosRead;

uint8_t ZdtProto_EncodeEnable(uint8_t *out, uint8_t capacity, uint8_t addr, uint8_t on, uint8_t snF);
uint8_t ZdtProto_EncodeStop(uint8_t *out, uint8_t capacity, uint8_t addr, uint8_t snF);
uint8_t ZdtProto_EncodeGoPulses(uint8_t *out, uint8_t capacity, const ZdtSlot *slot, uint32_t clk);
uint8_t ZdtProto_EncodeVel(uint8_t *out, uint8_t capacity, const ZdtSlot *slot);
uint8_t ZdtProto_EncodeZero(uint8_t *out, uint8_t capacity, uint8_t addr);
uint8_t ZdtProto_EncodeRead(uint8_t *out, uint8_t capacity, uint8_t addr, uint8_t cmd);
int ZdtProto_FindAck(const uint8_t *data, uint16_t len, ZdtAck *ack);
int ZdtProto_ParseRealtimePos(const uint8_t *data, uint16_t len, ZdtPosRead *out);
int ZdtProto_ParseStatusFlags(const uint8_t *data, uint16_t len, uint8_t *addr, uint8_t *flags);
int ZdtProto_ParseRealtimeVel(const uint8_t *data, uint16_t len, int32_t *rpm);
#define ZDT_ST_ENABLE           0x01U
#define ZDT_ST_POS_REACHED      0x02U
uint32_t ZdtProto_DegToPulses(uint32_t abs_deg);
/* Emm manual: angle_deg = units * 360 / 65536 (wire/encoder space, no dir map). */
int32_t ZdtProto_UnitsToEmmDeg(int32_t units);

#ifdef __cplusplus
}
#endif

#endif /* __ZDT_PROTOCOL_H */
