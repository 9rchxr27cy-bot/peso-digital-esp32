#include "arm.h"

#include <algorithm>
#include <cmath>

namespace pd {

const char* armStateName(ArmState s) {
  switch (s) {
    case ArmState::Idle: return "parado";
    case ArmState::Homing: return "zerando";
    case ArmState::Ready: return "pronto";
    case ArmState::Active: return "ativo";
  }
  return "?";
}

void ArmController::startHoming() {
  state_ = ArmState::Homing;
  homingTimeS_ = stillTimeS_ = 0.0f;
  homingFailed_ = false;
}

bool ArmController::start() {
  if (state_ != ArmState::Ready && state_ != ArmState::Active) return false;
  if (state_ == ArmState::Ready) modeState_ = ModeState{};
  state_ = ArmState::Active;
  return true;
}

void ArmController::stop() {
  if (state_ == ArmState::Active) state_ = ArmState::Ready;
}

void ArmController::shutdown() {
  state_ = ArmState::Idle;
  limiter_.reset();
  currentA_ = 0.0f;
}

float ArmController::update(int32_t rawSteps, float velMps, float dt) {
  velMps_ = velMps;
  bool homed = state_ == ArmState::Ready || state_ == ArmState::Active;
  posSteps_ = rawSteps - zeroSteps_;
  posM_ = posSteps_ * mc_.metersPerStep();

  float request = 0.0f;
  switch (state_) {
    case ArmState::Idle:
      limiter_.reset();
      currentA_ = 0.0f;
      return 0.0f;

    case ArmState::Homing:
      // Recolhe devagar: acima de homingMaxMps a forca cai, para o batente nao bater na torre.
      request = std::max(0.0f, homingForceN - homingDampNspm * std::max(0.0f, -velMps - homingMaxMps));
      homingTimeS_ += dt;
      stillTimeS_ = std::fabs(velMps) < homingStillMps ? stillTimeS_ + dt : 0.0f;
      // So vale como batente se a forca de homing ja estiver toda aplicada.
      if (stillTimeS_ >= homingStillTimeS && limiter_.outputN() >= 0.95f * std::min(homingForceN, limiter_.ceilingN())) {
        zeroSteps_ = rawSteps;
        posSteps_ = 0;
        posM_ = 0.0f;
        state_ = ArmState::Ready;
        homed = true;
        request = sc_.restForceN;
      } else if (homingTimeS_ > homingTimeoutS) {
        homingFailed_ = true;
        shutdown();
        return 0.0f;
      }
      break;

    case ArmState::Ready:
      request = sc_.restForceN;
      break;

    case ArmState::Active:
      request = modeForce(mode_, modeState_, posM_, velMps, dt);
      break;
  }

  float f = limiter_.apply(request, posM_, velMps, dt, homed);
  currentA_ = std::clamp(f / mc_.newtonPerAmp(), 0.0f, kHardMaxCurrentA);
  return currentA_;
}

}  // namespace pd
