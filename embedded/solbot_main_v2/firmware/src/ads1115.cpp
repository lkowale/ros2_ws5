#include "ads1115.h"

#include <Wire.h>

namespace {
constexpr uint8_t REG_CONV = 0x00, REG_CFG = 0x01;
// OS=1 (start) | MUX=1xx (AINx vs GND) | PGA=000 (+-6.144 V) | MODE=1 (single shot)
// | DR=111 (860 SPS) | COMP_QUE=11 (comparator off)
constexpr uint16_t CFG_BASE = 0x8000 | (0u << 9) | (1u << 8) | (7u << 5) | 0x3;
}  // namespace

bool Ads1115::readReg(uint8_t reg, uint16_t &v) {
  Wire.beginTransmission(addr_);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(addr_, (uint8_t)2) != 2) return false;
  v = (uint16_t)Wire.read() << 8;
  v |= Wire.read();
  return true;
}

bool Ads1115::writeReg(uint8_t reg, uint16_t v) {
  Wire.beginTransmission(addr_);
  Wire.write(reg);
  Wire.write(v >> 8);
  Wire.write(v & 0xFF);
  return Wire.endTransmission() == 0;
}

bool Ads1115::probe(uint8_t addr) {
  addr_ = addr;
  uint16_t cfg;
  ok_ = readReg(REG_CFG, cfg);
  return ok_;
}

bool Ads1115::start(uint8_t ch) {
  return writeReg(REG_CFG, CFG_BASE | ((uint16_t)(4 + (ch & 3)) << 12));
}

bool Ads1115::ready(bool &rdy) {
  uint16_t cfg;
  if (!readReg(REG_CFG, cfg)) return false;
  rdy = cfg & 0x8000;  // OS bit reads 1 when no conversion is in progress
  return true;
}

bool Ads1115::result(int16_t &raw) {
  uint16_t v;
  if (!readReg(REG_CONV, v)) return false;
  raw = (int16_t)v;
  return true;
}
