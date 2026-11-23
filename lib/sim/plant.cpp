#include "plant.h"

#include <algorithm>
#include <cmath>

namespace pdsim {

void ArmPlant::step(float t, float dt) {
  userForceN = user ? std::max(0.0f, user(t, x, v)) : 0.0f;
  float mass = handMassKg + motors * rotorInertia / (radiusM * radiusM);
  float drive = userForceN - tensionN();
  float fc = motors * coulombNm / radiusM;
  float net;
  if (std::fabs(v) < 1e-4f && std::fabs(drive) <= fc) {
    v = 0.0f;  // preso pelo atrito seco
    net = 0.0f;
  } else {
    float dir = std::fabs(v) >= 1e-4f ? (v > 0 ? 1.0f : -1.0f) : (drive > 0 ? 1.0f : -1.0f);
    net = drive - dir * fc - motors * viscousNms / (radiusM * radiusM) * v;
  }
  float vNew = v + net / mass * dt;
  if (v != 0.0f && vNew * v < 0.0f && std::fabs(drive) <= fc) vNew = 0.0f;  // parou e o atrito segura
  v = vNew;
  x += v * dt;
  if (x <= 0.0f) {  // batente encostado na torre
    x = 0.0f;
    v = std::max(v, 0.0f);
  }
  maxX = std::max(maxX, x);
}

}  // namespace pdsim
