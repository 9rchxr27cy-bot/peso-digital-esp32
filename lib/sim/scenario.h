// Cenarios: maquina + placa(s) falsa(s) + bracos + roteiro da mao do usuario.
#pragma once
#include <functional>
#include <string>
#include <vector>

#include "machine.h"

namespace pdsim {

using UserFn = std::function<float(float t, float x, float v)>;

// Repeticoes entre lo e hi [m], com forca maxima `strength(t)` [N]. A mao so puxa.
// Como um musculo de verdade, segura ate 40% a mais quando esta cedendo (descida).
UserFn repsUser(float lo, float hi, float periodS, float t0, std::function<float(float)> strength);

struct Scenario {
  std::string name;
  pd::Variant variant = pd::Variant::M1;
  pd::ModeParams mode;
  float ceilingA = 5.0f;
  float slewUpNps = -1.0f;        // < 0 = padrao da seguranca
  float durationS = 20.0f;
  float initialX = 0.30f;         // cabo fora do batente ao ligar (testa o homing)
  UserFn user;                    // os dois bracos usam o mesmo roteiro
  float estopAtS = -1.0f;         // aperta a emergencia
  float cutTelemetryAtS = -1.0f;  // a placa para de responder
  float cutCommandAtS = -1.0f;    // o ESP32 para de mandar comando (travou)
  int serialTimeoutLoops = 20;
};

struct Sample {
  float t;
  float x, v;          // verdadeiros, braco 0
  float userN;         // forca da mao
  float tensionN;      // forca real do motor no cabo
  float cmdForceN;     // forca comandada pelo ESP32
  float cmdCurrentA;   // corrente por motor comandada
  float estX, estV;    // posicao e velocidade estimadas pelo ESP32
  int state;           // pd::ArmState
  int fault;           // pd::Fault
  float spotFactor;
};

struct Result {
  std::vector<Sample> samples;
  float homingErrorM = 0.0f;  // erro do zero em relacao ao batente verdadeiro
  float activeAtS = -1.0f;    // instante em que o modo comecou
  float maxX = 0.0f;
  int minCmd = 0, maxCmd = 0;
  pd::Fault fault = pd::Fault::None;
};

Result runScenario(const Scenario& sc);
std::vector<Scenario> standardScenarios(pd::Variant v);
bool writeCsv(const Result& r, const std::string& path);

}  // namespace pdsim
