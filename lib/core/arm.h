// Controle de um braco: estados, referencia do zero (homing), modo e seguranca.
#pragma once
#include "machine_config.h"
#include "modes.h"
#include "safety.h"

namespace pd {

enum class ArmState : uint8_t {
  Idle,    // comando zero, posicao desconhecida
  Homing,  // recolhendo devagar ate o batente encostar na torre
  Ready,   // zerado; so tensao de repouso
  Active,  // modo de treino aplicado
};

const char* armStateName(ArmState s);

class ArmController {
 public:
  ArmController(const MachineConfig& mc, const SafetyConfig& sc) : mc_(mc), sc_(sc), limiter_(mc, sc) {}

  void startHoming();
  bool start();                 // Ready -> Active; falso se ainda nao zerou
  void stop();                  // Active -> Ready
  void shutdown();              // qualquer -> Idle, comando zero na hora, perde o zero
  void setMode(const ModeParams& p) { mode_ = p; modeState_ = ModeState{}; }

  // Um ciclo de controle. rawSteps: passos acumulados (positivo = cabo saindo).
  // Devolve a corrente por motor [A]. takeHomingFailed() avisa uma vez se o homing estourar o tempo.
  float update(int32_t rawSteps, float velMps, float dt);

  ArmState state() const { return state_; }
  bool takeHomingFailed() { bool f = homingFailed_; homingFailed_ = false; return f; }
  float positionM() const { return posM_; }
  int32_t positionSteps() const { return posSteps_; }
  float velocityMps() const { return velMps_; }
  float forceN() const { return limiter_.outputN(); }
  float currentA() const { return currentA_; }
  const ModeParams& mode() const { return mode_; }
  const ModeState& modeState() const { return modeState_; }

  float homingForceN = 15.0f;
  float homingMaxMps = 0.15f;
  float homingDampNspm = 150.0f;
  float homingStillMps = 0.01f;
  float homingStillTimeS = 0.5f;
  float homingTimeoutS = 15.0f;

 private:
  const MachineConfig& mc_;
  const SafetyConfig& sc_;
  ForceLimiter limiter_;
  ModeParams mode_;
  ModeState modeState_;
  ArmState state_ = ArmState::Idle;
  int32_t zeroSteps_ = 0;
  int32_t posSteps_ = 0;
  float posM_ = 0.0f, velMps_ = 0.0f, currentA_ = 0.0f;
  float homingTimeS_ = 0.0f, stillTimeS_ = 0.0f;
  bool homingFailed_ = false;
};

}  // namespace pd
