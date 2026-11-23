#include "board_sim.h"

#include <algorithm>
#include <cmath>

#include "machine_config.h"

namespace pdsim {

void BoardSim::rx(const uint8_t* data, size_t n) {
  for (size_t i = 0; i < n; ++i) {
    if (!parser_.feed(data[i], cmd_)) continue;
    timeoutCnt_ = 0;
    timeout_ = false;
    minCmd_ = std::min({minCmd_, cmd_.steer, cmd_.speed});
    maxCmd_ = std::max({maxCmd_, cmd_.steer, cmd_.speed});
  }
}

int BoardSim::odom(const ArmPlant* p, float x0) {
  float mPerStep = pd::kTwoPi * p->radiusM / pd::kStepsPerRev;
  int steps = static_cast<int>(std::floor((p->x - x0) / mPerStep));
  return ((steps % pd::kOdomModulo) + pd::kOdomModulo) % pd::kOdomModulo;
}

void BoardSim::tick1ms(std::vector<uint8_t>& tx) {
  if (x0L_ < 0) {  // o contador da placa comeca em zero onde o cabo estiver ao ligar
    x0L_ = left_->x;
    x0R_ = right_->x;
  }
  if (ms_ % 5 == 0) {  // laco principal da placa
    if (timeoutCnt_++ >= serialTimeoutLoops) timeout_ = true;
    float inL = timeout_ ? 0.0f : std::clamp<float>(cmd_.steer, -1000, 1000);
    float inR = timeout_ ? 0.0f : std::clamp<float>(cmd_.speed, -1000, 1000);
    rateL_ += std::clamp(inL - rateL_, -ratePerLoop, ratePerLoop);
    rateR_ += std::clamp(inR - rateR_, -ratePerLoop, ratePerLoop);
    outL_ += filterCoef * (rateL_ - outL_);
    outR_ += filterCoef * (rateR_ - outR_);
    if (timeout_) outL_ = outR_ = rateL_ = rateR_ = 0.0f;  // OPEN_MODE: sem torque
  }
  if (ms_ % 10 == 0) {  // telemetria
    pd::Feedback f;
    f.cmd1 = cmd_.steer;
    f.cmd2 = cmd_.speed;
    auto rpm = [](const ArmPlant* p) {
      return static_cast<int16_t>(std::lround(p->v / (pd::kTwoPi * p->radiusM) * 60.0f));
    };
    f.speedL = rpm(left_);
    f.speedR = rpm(right_);
    f.wheelL = static_cast<int16_t>(odom(left_, x0L_));
    f.wheelR = static_cast<int16_t>(odom(right_, x0R_));
    f.batV100 = 3900;
    f.tempC10 = 350;
    uint8_t buf[pd::kFeedbackLen];
    pd::encodeFeedback(f, buf);
    tx.insert(tx.end(), buf, buf + pd::kFeedbackLen);
  }
  ++ms_;
}

void BoardSim::addTorque() const {
  left_->motorTorqueNm += currentLeftA() * pd::kTorquePerAmp;
  right_->motorTorqueNm += currentRightA() * pd::kTorquePerAmp;
}

}  // namespace pdsim
