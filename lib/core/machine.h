// Maquina completa: liga as placas (UART) aos dois bracos e vigia as falhas.
// Portatil: recebe bytes e o relogio de fora, devolve os quadros a enviar.
#pragma once
#include <cstddef>
#include <cstdint>

#include "arm.h"
#include "machine_config.h"
#include "protocol.h"
#include "safety.h"

namespace pd {

enum class Fault : uint8_t { None, Estop, TelemetryTimeout, PositionJump, Overspeed, HomingTimeout, BoardTemp };
const char* faultName(Fault f);

constexpr int kNumArms = 2;
constexpr uint32_t kControlPeriodMs = 10;  // 100 Hz, igual a telemetria da placa

class Machine {
 public:
  explicit Machine(Variant v);
  Machine(const Machine&) = delete;
  Machine& operator=(const Machine&) = delete;

  // Bytes recebidos da placa `link` (0 no 1M; 0 = braco esquerdo, 1 = direito no 2M).
  void onRx(int link, const uint8_t* data, size_t n, uint32_t nowMs);

  // Um ciclo de controle (chamar a cada kControlPeriodMs). Depois, enviar txFrame() de cada placa.
  void step(uint32_t nowMs);
  const uint8_t* txFrame(int link) const { return links_[link].tx; }
  int numLinks() const { return mc_.numLinks; }

  void setEstop(bool pressed);
  bool clearFault();  // falso enquanto o botao de emergencia estiver apertado
  bool startHoming(); // falso com falha ativa ou sem telemetria

  Fault fault() const { return fault_; }
  bool linkAlive(int link, uint32_t nowMs) const;
  ArmController& arm(int i) { return arms_[i]; }
  const ArmController& arm(int i) const { return arms_[i]; }
  SafetyConfig& safety() { return sc_; }
  const MachineConfig& config() const { return mc_; }
  float batteryV(int link) const { return links_[link].fb.batV100 / 100.0f; }
  float boardTempC(int link) const { return links_[link].fb.tempC10 / 10.0f; }
  uint32_t badFrames(int link) const { return links_[link].parser.badFrames(); }

 private:
  struct Motor {
    int32_t steps = 0;
    int16_t lastCnt = 0;
    float rpm = 0.0f;
  };
  struct Link {
    FeedbackParser parser;
    Feedback fb;
    Motor left, right;
    uint32_t lastRxMs = 0;
    bool everRx = false;
    uint8_t tx[kCommandLen];
  };

  void raise(Fault f);
  void track(Motor& m, int16_t cnt, int16_t rpm, const MotorSigns& s, bool first);

  MachineConfig mc_;
  SafetyConfig sc_;
  ArmController arms_[kNumArms];
  Link links_[2];
  Fault fault_ = Fault::None;
  bool estop_ = false;
  uint32_t lastStepMs_ = 0;
  bool stepped_ = false;
};

}  // namespace pd
