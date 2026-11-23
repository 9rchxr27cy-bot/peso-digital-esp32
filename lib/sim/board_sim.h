// Placa de hoverboard falsa: fala os mesmos bytes do firmware EFeru
// (VARIANT_USART + TRQ_MODE + TANK_STEERING + ENABLE_ODOMETRY) e aplica
// o torque nos modelos dos bracos. Chamar tick1ms() a cada milissegundo.
#pragma once
#include <cstdint>
#include <vector>

#include "plant.h"
#include "protocol.h"

namespace pdsim {

class BoardSim {
 public:
  BoardSim(ArmPlant* left, ArmPlant* right) : left_(left), right_(right) {}

  int serialTimeoutLoops = 20;  // SERIAL_TIMEOUT do fork (lacos de 5 ms); o original e 160
  float ratePerLoop = 30.0f;    // DEFAULT_RATE 480 = 30,0 unidades por laco
  float filterCoef = 0.1f;      // DEFAULT_FILTER 6553 = 0,1

  void rx(const uint8_t* data, size_t n);
  void tick1ms(std::vector<uint8_t>& tx);  // acrescenta em tx os bytes de telemetria
  void addTorque() const;                  // soma o torque atual nos bracos

  bool timedOut() const { return timeout_; }
  float currentLeftA() const { return outL_ / pd::kCmdMax * 15.0f; }
  float currentRightA() const { return outR_ / pd::kCmdMax * 15.0f; }
  int16_t lastCmdLeft() const { return cmd_.steer; }
  int16_t lastCmdRight() const { return cmd_.speed; }
  int16_t minCmdSeen() const { return minCmd_; }
  int16_t maxCmdSeen() const { return maxCmd_; }

 private:
  static int odom(const ArmPlant* p, float x0);

  ArmPlant* left_;
  ArmPlant* right_;
  pd::CommandParser parser_;
  pd::Command cmd_;
  int ms_ = 0;
  int timeoutCnt_ = 1 << 20;  // sem comando no boot = em timeout
  bool timeout_ = true;
  float rateL_ = 0, rateR_ = 0, outL_ = 0, outR_ = 0;
  float x0L_ = -1, x0R_ = -1;
  int16_t minCmd_ = 0, maxCmd_ = 0;
};

}  // namespace pdsim
