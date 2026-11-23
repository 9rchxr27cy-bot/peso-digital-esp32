#include "safety.h"

#include <algorithm>

namespace pd {

float ForceLimiter::ceilingN() const {
  return std::min(sc_.ceilingA, kHardMaxCurrentA) * mc_.newtonPerAmp();
}

float ForceLimiter::apply(float requestN, float x, float v, float dt, bool homed) {
  float f = std::max(requestN, 0.0f);  // o motor so puxa; nunca desenrola o cabo

  if (homed) {
    // Perto do batente: so tensao de repouso, e a carga entra aos poucos.
    float cap = f;
    if (x <= sc_.landingZoneM) {
      cap = sc_.restForceN;
    } else if (x < sc_.landingZoneM + sc_.blendZoneM) {
      float k = (x - sc_.landingZoneM) / sc_.blendZoneM;
      cap = sc_.restForceN + k * (f - sc_.restForceN);
    }
    f = std::min(f, std::max(cap, 0.0f));

    // Fim de curso maximo: parede virtual que puxa de volta (limitada pelo teto).
    float wallStart = mc_.maxTravelM() - sc_.endMarginM;
    if (x > wallStart) f = std::max(f, sc_.endWallNpm * (x - wallStart));
  }

  // Retorno rapido demais (cabo solto da mao): reduz a forca.
  if (-v > sc_.maxRetractMps) f = std::max(0.0f, f - sc_.retractDampNspm * (-v - sc_.maxRetractMps));

  f = std::min(f, ceilingN());

  // Rampa: nada de degrau.
  float up = sc_.slewUpNps * dt, down = sc_.slewDownNps * dt;
  outN_ += std::clamp(f - outN_, -down, up);
  return outN_;
}

}  // namespace pd
