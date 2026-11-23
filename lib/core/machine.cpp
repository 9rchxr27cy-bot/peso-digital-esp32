#include "machine.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace pd {

const char* faultName(Fault f) {
  switch (f) {
    case Fault::None: return "nenhuma";
    case Fault::Estop: return "emergencia";
    case Fault::TelemetryTimeout: return "sem telemetria";
    case Fault::PositionJump: return "salto de posicao";
    case Fault::Overspeed: return "velocidade excessiva";
    case Fault::HomingTimeout: return "homing nao terminou";
    case Fault::BoardTemp: return "placa quente";
  }
  return "?";
}

Machine::Machine(Variant v)
    : mc_(v == Variant::M1 ? config1M() : config2M()), arms_{ArmController(mc_, sc_), ArmController(mc_, sc_)} {
  for (auto& l : links_) encodeCommand(Command{}, l.tx);
}

void Machine::track(Motor& m, int16_t cnt, int16_t rpm, const MotorSigns& s, bool first) {
  if (!first) {
    int d = odomDelta(cnt, m.lastCnt);
    if (std::abs(d) > sc_.maxStepJump) raise(Fault::PositionJump);
    m.steps += s.odom * d;
  }
  m.lastCnt = cnt;
  m.rpm = static_cast<float>(s.speed * rpm);
}

void Machine::onRx(int link, const uint8_t* data, size_t n, uint32_t nowMs) {
  Link& l = links_[link];
  for (size_t i = 0; i < n; ++i) {
    if (!l.parser.feed(data[i], l.fb)) continue;
    bool first = !l.everRx || nowMs - l.lastRxMs > sc_.telemetryTimeoutMs;
    track(l.left, l.fb.wheelL, l.fb.speedL, mc_.left[link], first);
    track(l.right, l.fb.wheelR, l.fb.speedR, mc_.right[link], first);
    l.lastRxMs = nowMs;
    l.everRx = true;
  }
}

bool Machine::linkAlive(int link, uint32_t nowMs) const {
  const Link& l = links_[link];
  // diferenca com sinal: um quadro pode ter chegado depois de nowMs
  return l.everRx && static_cast<int32_t>(nowMs - l.lastRxMs) <= static_cast<int32_t>(sc_.telemetryTimeoutMs);
}

void Machine::raise(Fault f) {
  if (fault_ == Fault::None) fault_ = f;
  for (auto& a : arms_) a.shutdown();
}

void Machine::setEstop(bool pressed) {
  estop_ = pressed;
  if (pressed) raise(Fault::Estop);
}

bool Machine::clearFault() {
  if (estop_) return false;
  fault_ = Fault::None;
  return true;
}

bool Machine::startHoming() {
  if (fault_ != Fault::None || !stepped_) return false;
  for (int i = 0; i < mc_.numLinks; ++i)
    if (!linkAlive(i, lastStepMs_)) return false;
  for (auto& a : arms_) a.startHoming();
  return true;
}

void Machine::step(uint32_t nowMs) {
  float dt = stepped_ ? std::clamp((nowMs - lastStepMs_) / 1000.0f, 0.001f, 0.05f) : kControlPeriodMs / 1000.0f;
  lastStepMs_ = nowMs;
  stepped_ = true;

  bool anyRunning = false;
  for (auto& a : arms_) anyRunning |= a.state() != ArmState::Idle;

  if (estop_) raise(Fault::Estop);
  for (int i = 0; i < mc_.numLinks; ++i) {
    if (anyRunning && !linkAlive(i, nowMs)) raise(Fault::TelemetryTimeout);
    if (links_[i].everRx && boardTempC(i) > sc_.maxBoardTempC) raise(Fault::BoardTemp);
  }

  const float rpmToMps = kTwoPi * mc_.spoolRadiusM / 60.0f;
  float amps[kNumArms] = {0.0f, 0.0f};
  for (int i = 0; i < kNumArms; ++i) {
    int32_t steps;
    float rpm;
    if (mc_.variant == Variant::M1) {  // uma placa: esquerda = braco 0, direita = braco 1
      const Motor& m = i == 0 ? links_[0].left : links_[0].right;
      steps = m.steps;
      rpm = m.rpm;
    } else {  // uma placa por braco, dois motores no mesmo carretel
      steps = (links_[i].left.steps + links_[i].right.steps) / 2;
      rpm = 0.5f * (links_[i].left.rpm + links_[i].right.rpm);
    }
    float v = rpm * rpmToMps;
    if (std::fabs(v) > sc_.overspeedMps && arms_[i].state() != ArmState::Idle) raise(Fault::Overspeed);
    amps[i] = arms_[i].update(steps, v, dt);
    if (arms_[i].takeHomingFailed()) raise(Fault::HomingTimeout);
  }

  if (fault_ != Fault::None) amps[0] = amps[1] = 0.0f;  // falha em um braco zera os dois

  auto toCmd = [](float a, int8_t sign) {
    float c = std::clamp(a, 0.0f, kHardMaxCurrentA) / kBoardMaxCurrentA * kCmdMax;
    return static_cast<int16_t>(sign * std::clamp(static_cast<int>(std::lround(c)), 0, static_cast<int>(kCmdMax)));
  };
  for (int i = 0; i < mc_.numLinks; ++i) {
    Command c;
    if (mc_.variant == Variant::M1) {
      c.steer = toCmd(amps[0], mc_.left[0].cmd);
      c.speed = toCmd(amps[1], mc_.right[0].cmd);
    } else {
      c.steer = toCmd(amps[i], mc_.left[i].cmd);
      c.speed = toCmd(amps[i], mc_.right[i].cmd);
    }
    encodeCommand(c, links_[i].tx);
  }
}

}  // namespace pd
