#include "zdt_protocol.h"

extern "C" uint32_t ZdtProto_DegToPulses(uint32_t abs_deg)
{
  return static_cast<uint32_t>((static_cast<uint64_t>(abs_deg) * ZDT_PULSE_PER_REV) / 360U);
}

extern "C" uint8_t ZdtProto_EncodeEnable(uint8_t *out, uint8_t capacity, uint8_t addr, uint8_t on, uint8_t snF)
{
  if (out == nullptr || capacity < 6U) return 0U;
  out[0] = addr;
  out[1] = ZDT_CMD_ENABLE;
  out[2] = 0xABU;
  out[3] = on ? 1U : 0U;
  out[4] = snF;
  out[5] = ZDT_TAIL;
  return 6U;
}

extern "C" uint8_t ZdtProto_EncodeStop(uint8_t *out, uint8_t capacity, uint8_t addr, uint8_t snF)
{
  if (out == nullptr || capacity < 5U) return 0U;
  out[0] = addr;
  out[1] = ZDT_CMD_STOP;
  out[2] = 0x98U;
  out[3] = snF;
  out[4] = ZDT_TAIL;
  return 5U;
}

extern "C" uint8_t ZdtProto_EncodeGoPulses(uint8_t *out, uint8_t capacity, const ZdtSlot *slot, uint32_t clk)
{
  if (out == nullptr || slot == nullptr || capacity < 13U) return 0U;
  out[0]  = slot->addr;
  out[1]  = ZDT_CMD_POS;
  out[2]  = slot->dir;
  out[3]  = static_cast<uint8_t>(slot->vel >> 8);
  out[4]  = static_cast<uint8_t>(slot->vel);
  out[5]  = slot->acc;
  out[6]  = static_cast<uint8_t>(clk >> 24);
  out[7]  = static_cast<uint8_t>(clk >> 16);
  out[8]  = static_cast<uint8_t>(clk >> 8);
  out[9]  = static_cast<uint8_t>(clk);
  out[10] = slot->mode;
  out[11] = slot->snF;
  out[12] = ZDT_TAIL;
  return 13U;
}

extern "C" uint8_t ZdtProto_EncodeVel(uint8_t *out, uint8_t capacity, const ZdtSlot *slot)
{
  if (out == nullptr || slot == nullptr || capacity < 8U) return 0U;
  out[0] = slot->addr;
  out[1] = ZDT_CMD_VEL;
  out[2] = slot->dir;
  out[3] = static_cast<uint8_t>(slot->vel >> 8);
  out[4] = static_cast<uint8_t>(slot->vel);
  out[5] = slot->acc;
  out[6] = slot->snF;
  out[7] = ZDT_TAIL;
  return 8U;
}

extern "C" uint8_t ZdtProto_EncodeZero(uint8_t *out, uint8_t capacity, uint8_t addr)
{
  if (out == nullptr || capacity < 4U) return 0U;
  out[0] = addr;
  out[1] = ZDT_CMD_ZERO;
  out[2] = 0x6DU;
  out[3] = ZDT_TAIL;
  return 4U;
}

extern "C" uint8_t ZdtProto_EncodeRead(uint8_t *out, uint8_t capacity, uint8_t addr, uint8_t cmd)
{
  if (out == nullptr || capacity < 3U) return 0U;
  if (addr == ZDT_ADDR_BROADCAST) return 0U;
  out[0] = addr;
  out[1] = cmd;
  out[2] = ZDT_TAIL;
  return 3U;
}

extern "C" int ZdtProto_FindAck(const uint8_t *data, uint16_t len, ZdtAck *ack)
{
  if (data == nullptr || ack == nullptr || len < 4U) return 0;
  for (uint16_t i = 0U; i + 3U < len; ++i)
  {
    const uint8_t cmd = data[i + 1U];
    const uint8_t st  = data[i + 2U];
    const uint8_t tail = data[i + 3U];
    if (tail != ZDT_TAIL) continue;
    if (st != ZDT_ACK_OK) continue;
    if (cmd == ZDT_CMD_ENABLE || cmd == ZDT_CMD_VEL || cmd == ZDT_CMD_POS ||
        cmd == ZDT_CMD_STOP || cmd == ZDT_CMD_ZERO ||
        cmd == 0xAEU || cmd == 0x9AU || cmd == 0x0EU)
    {
      ack->addr = data[i];
      ack->cmd = cmd;
      ack->status = st;
      return 1;
    }
  }
  return 0;
}

extern "C" int ZdtProto_ParseReadPos(const uint8_t *data, uint16_t len, uint8_t expect_addr, int32_t *pos_out)
{
  if (data == nullptr || pos_out == nullptr || len < 8U) return 0;
  for (uint16_t i = 0U; i + 7U < len; ++i)
  {
    if (data[i] != expect_addr) continue;
    if (data[i + 1U] != ZDT_CMD_READ_POS) continue;
    if (data[i + 7U] != ZDT_TAIL) continue;
    const uint8_t sign = data[i + 2U];
    uint32_t mag = (static_cast<uint32_t>(data[i + 3U]) << 24) |
                   (static_cast<uint32_t>(data[i + 4U]) << 16) |
                   (static_cast<uint32_t>(data[i + 5U]) << 8) |
                   static_cast<uint32_t>(data[i + 6U]);
    *pos_out = (sign != 0U) ? -static_cast<int32_t>(mag) : static_cast<int32_t>(mag);
    return 1;
  }
  return 0;
}
