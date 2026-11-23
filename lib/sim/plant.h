// Modelo fisico de um braco: carretel + cabo + mao do usuario.
// x = cabo para fora do batente [m], v > 0 = puxando.
#pragma once
#include <functional>

namespace pdsim {

struct ArmPlant {
  float radiusM = 0.020f;
  int motors = 1;
  float rotorInertia = 0.005f;   // [kg.m2] por motor (rotor do hoverboard sem pneu)
  float handMassKg = 2.0f;       // pegador + parte do braco do usuario
  float coulombNm = 0.08f;       // atrito seco por motor
  float viscousNms = 0.002f;     // atrito viscoso por motor
  float x = 0.0f, v = 0.0f;
  float motorTorqueNm = 0.0f;    // soma dos motores, positivo = recolhe
  float userForceN = 0.0f;       // ultima forca da mao (para registro)
  float maxX = 0.0f;
  std::function<float(float t, float x, float v)> user;  // forca da mao, >= 0

  float tensionN() const { return motorTorqueNm / radiusM; }
  void step(float t, float dt);
};

}  // namespace pdsim
