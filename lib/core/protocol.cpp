#include "protocol.h"

#include "machine_config.h"

namespace pd {
namespace {

void put16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>(v >> 8);
}
uint16_t get16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }

void seal(uint8_t* buf, size_t len) {
  uint16_t x = 0;
  for (size_t i = 0; i + 2 < len; i += 2) x ^= get16(buf + i);
  put16(buf + len - 2, x);
}

}  // namespace

void encodeCommand(const Command& c, uint8_t out[kCommandLen]) {
  put16(out + 0, kStartFrame);
  put16(out + 2, static_cast<uint16_t>(c.steer));
  put16(out + 4, static_cast<uint16_t>(c.speed));
  seal(out, kCommandLen);
}

void encodeFeedback(const Feedback& f, uint8_t out[kFeedbackLen]) {
  put16(out + 0, kStartFrame);
  put16(out + 2, static_cast<uint16_t>(f.cmd1));
  put16(out + 4, static_cast<uint16_t>(f.cmd2));
  put16(out + 6, static_cast<uint16_t>(f.speedR));
  put16(out + 8, static_cast<uint16_t>(f.speedL));
  put16(out + 10, static_cast<uint16_t>(f.wheelR));
  put16(out + 12, static_cast<uint16_t>(f.wheelL));
  put16(out + 14, static_cast<uint16_t>(f.batV100));
  put16(out + 16, static_cast<uint16_t>(f.tempC10));
  put16(out + 18, f.led);
  seal(out, kFeedbackLen);
}

bool FeedbackParser::feed(uint8_t b, Feedback& out) {
  if (!p_.feed(b)) return false;
  const uint8_t* d = p_.buf();
  out.cmd1 = static_cast<int16_t>(get16(d + 2));
  out.cmd2 = static_cast<int16_t>(get16(d + 4));
  out.speedR = static_cast<int16_t>(get16(d + 6));
  out.speedL = static_cast<int16_t>(get16(d + 8));
  out.wheelR = static_cast<int16_t>(get16(d + 10));
  out.wheelL = static_cast<int16_t>(get16(d + 12));
  out.batV100 = static_cast<int16_t>(get16(d + 14));
  out.tempC10 = static_cast<int16_t>(get16(d + 16));
  out.led = get16(d + 18);
  return true;
}

bool CommandParser::feed(uint8_t b, Command& out) {
  if (!p_.feed(b)) return false;
  const uint8_t* d = p_.buf();
  out.steer = static_cast<int16_t>(get16(d + 2));
  out.speed = static_cast<int16_t>(get16(d + 4));
  return true;
}

int odomDelta(int16_t now, int16_t prev) {
  int d = (now - prev) % kOdomModulo;
  if (d < -kOdomModulo / 2) d += kOdomModulo;
  if (d >= kOdomModulo / 2) d -= kOdomModulo;
  return d;
}

}  // namespace pd
