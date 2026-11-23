// Quadros UART da placa (EFeru hoverboard-firmware-hack-FOC, VARIANT_USART,
// TANK_STEERING e ENABLE_ODOMETRY). Tudo little-endian, checksum = XOR das palavras.
#pragma once
#include <cstddef>
#include <cstdint>

namespace pd {

constexpr uint16_t kStartFrame = 0xABCD;
constexpr size_t kCommandLen = 8;
constexpr size_t kFeedbackLen = 22;
constexpr int16_t kCmdMax = 1000;

struct Command {
  int16_t steer = 0;  // com TANK_STEERING: motor esquerdo
  int16_t speed = 0;  // com TANK_STEERING: motor direito
};

struct Feedback {
  int16_t cmd1 = 0;
  int16_t cmd2 = 0;
  int16_t speedR = 0;     // [rpm]
  int16_t speedL = 0;     // [rpm]
  int16_t wheelR = 0;     // passos de Hall, 0..8999
  int16_t wheelL = 0;     // passos de Hall, 0..8999
  int16_t batV100 = 0;    // [V x 100]
  int16_t tempC10 = 0;    // [graus C x 10]
  uint16_t led = 0;
};

void encodeCommand(const Command& c, uint8_t out[kCommandLen]);
void encodeFeedback(const Feedback& f, uint8_t out[kFeedbackLen]);

// Parser de fluxo de bytes: ressincroniza no start frame e descarta checksum ruim.
template <size_t N>
class FrameParser {
 public:
  // Devolve true quando um quadro completo e valido acabou de chegar em buf().
  bool feed(uint8_t b) {
    if (n_ == 0) {
      if (b == (kStartFrame & 0xFF)) buf_[n_++] = b;
      return false;
    }
    if (n_ == 1) {
      if (b == (kStartFrame >> 8)) buf_[n_++] = b;
      else if (b != (kStartFrame & 0xFF)) n_ = 0;
      return false;
    }
    buf_[n_++] = b;
    if (n_ < N) return false;
    n_ = 0;
    uint16_t x = 0;
    for (size_t i = 0; i < N; i += 2) x ^= static_cast<uint16_t>(buf_[i] | (buf_[i + 1] << 8));
    if (x != 0) {  // XOR de tudo, incluindo o checksum, tem de dar zero
      ++badFrames_;
      return false;
    }
    return true;
  }
  const uint8_t* buf() const { return buf_; }
  uint32_t badFrames() const { return badFrames_; }

 private:
  uint8_t buf_[N] = {};
  size_t n_ = 0;
  uint32_t badFrames_ = 0;
};

class FeedbackParser {
 public:
  bool feed(uint8_t b, Feedback& out);
  uint32_t badFrames() const { return p_.badFrames(); }

 private:
  FrameParser<kFeedbackLen> p_;
};

class CommandParser {
 public:
  bool feed(uint8_t b, Command& out);
  uint32_t badFrames() const { return p_.badFrames(); }

 private:
  FrameParser<kCommandLen> p_;
};

// Diferenca entre duas leituras do contador modulo 9000, no intervalo [-4500, 4499].
int odomDelta(int16_t now, int16_t prev);

}  // namespace pd
