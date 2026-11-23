// Constantes fisicas da maquina e das duas versoes (1M e 2M). Sem dependencias.
#pragma once
#include <cstdint>

namespace pd {

constexpr float kTorquePerAmp = 0.52f;     // [N.m/A] constante de torque do motor
constexpr float kHardMaxCurrentA = 15.0f;  // [A] limite absoluto: pecas mecanicas de plastico
constexpr float kBoardMaxCurrentA = 15.0f; // [A] I_MOT_MAX gravado na placa (comando 1000)
constexpr int kStepsPerRev = 90;           // passos de Hall por volta
constexpr int kOdomModulo = 9000;          // contador da placa: 0..8999
constexpr float kGravity = 9.80665f;
constexpr float kTwoPi = 6.2831853f;

enum class Variant : uint8_t { M1, M2 };

// Sinais de montagem de um motor. A convencao do nucleo e:
//   comando positivo = recolhe o cabo; passo/velocidade positivos = cabo saindo.
struct MotorSigns {
  int8_t cmd = 1;
  int8_t odom = 1;
  int8_t speed = 1;
};

struct MachineConfig {
  Variant variant;
  int motorsPerArm;    // motores somando torque em cada braco
  int numLinks;        // placas (UARTs): 1 no 1M, 2 no 2M
  float spoolRadiusM;  // [m]
  int maxSteps;        // fim de curso maximo, em passos a partir do batente
  MotorSigns left[2];  // sinais do motor esquerdo de cada placa
  MotorSigns right[2]; // sinais do motor direito de cada placa

  float metersPerStep() const { return kTwoPi * spoolRadiusM / kStepsPerRev; }
  float maxTravelM() const { return maxSteps * metersPerStep(); }
  // Forca no cabo por ampere em CADA motor do braco.
  float newtonPerAmp() const { return motorsPerArm * kTorquePerAmp / spoolRadiusM; }
};

inline MachineConfig config1M() {
  MachineConfig c{};
  c.variant = Variant::M1;
  c.motorsPerArm = 1;
  c.numLinks = 1;
  c.spoolRadiusM = 0.020f;
  c.maxSteps = 1440;
  return c;
}

inline MachineConfig config2M() {
  MachineConfig c{};
  c.variant = Variant::M2;
  c.motorsPerArm = 2;
  c.numLinks = 2;
  c.spoolRadiusM = 0.025f;
  c.maxSteps = 1170;
  return c;
}

inline float kgToNewton(float kg) { return kg * kGravity; }
inline float newtonToKg(float n) { return n / kGravity; }

}  // namespace pd
