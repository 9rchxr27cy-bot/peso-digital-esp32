#include "modes.h"

#include <algorithm>
#include <cmath>

namespace pd {

float modeForce(const ModeParams& p, ModeState& s, float x, float v, float dt) {
  switch (p.type) {
    case ModeType::Constant:
      return p.baseN;

    case ModeType::Eccentric: {
      float k = std::clamp(-v / p.eccBlendMps, 0.0f, 1.0f);  // 0 subindo, 1 descendo
      return p.baseN * (1.0f + (p.eccFactor - 1.0f) * k);
    }

    case ModeType::Chain:
      return p.baseN + p.chainNpm * std::max(x, 0.0f);

    case ModeType::Elastic:
      return std::max(p.baseN, p.elasticNpm * (x - p.elasticStartM));

    case ModeType::Isokinetic: {
      float e = v - p.isoSpeedMps;
      s.isoIntegralN = std::max(0.0f, s.isoIntegralN + p.isoIntegralNpm * e * dt);  // esvazia abaixo do alvo
      return p.baseN + p.isoGainNspm * std::max(0.0f, e) + s.isoIntegralN;
    }

    case ModeType::Spotter: {
      s.topM = std::max(s.topM, x);
      bool belowTop = x < p.spotTopFraction * s.topM;
      if (x <= p.spotMinPosM || !belowTop) s.lifting = false;  // no repouso ou repeticao completa
      else if (v > p.spotStallMps) s.lifting = true;           // comecou a subir

      if (s.lifting && v < p.spotStallMps) {  // subida parou (ou esta cedendo) antes do alto
        s.stallTimerS += dt;
        if (s.stallTimerS > p.spotStallTimeS)
          s.spotFactor = std::max(p.spotMinFactor, s.spotFactor - p.spotReliefPerS * dt);
      } else {
        s.stallTimerS = 0.0f;
        if (x <= p.spotMinPosM) s.spotFactor = std::min(1.0f, s.spotFactor + p.spotRecoverPerS * dt);
      }
      return p.baseN * s.spotFactor;
    }
  }
  return 0.0f;
}

const char* modeName(ModeType t) {
  switch (t) {
    case ModeType::Constant: return "constante";
    case ModeType::Eccentric: return "excentrico";
    case ModeType::Chain: return "corrente";
    case ModeType::Elastic: return "elastico";
    case ModeType::Isokinetic: return "isocinetico";
    case ModeType::Spotter: return "spotter";
  }
  return "?";
}

}  // namespace pd
