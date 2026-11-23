// Camada de seguranca: ultima etapa antes do comando de corrente.
// Recebe a forca pedida pelo modo e devolve a forca permitida.
#pragma once
#include "machine_config.h"

namespace pd {

struct SafetyConfig {
  float ceilingA = 5.0f;          // [A por motor] teto configuravel; nunca acima de kHardMaxCurrentA
  float slewUpNps = 400.0f;       // [N/s] rampa de subida da forca
  float slewDownNps = 1500.0f;    // [N/s] rampa de descida (aliviar pode ser mais rapido)
  float restForceN = 10.0f;       // [N] tensao de repouso, so para manter o cabo esticado
  float landingZoneM = 0.05f;     // ate aqui, perto do batente, so a tensao de repouso
  float blendZoneM = 0.05f;       // faixa em que a carga entra aos poucos depois da zona acima
  float endMarginM = 0.15f;       // parede virtual comeca esta distancia antes do curso maximo
  float endWallNpm = 3000.0f;     // [N/m] rigidez da parede virtual (limitada pelo teto)
  float maxRetractMps = 0.8f;     // velocidade de retorno acima da qual a forca e reduzida
  float retractDampNspm = 150.0f; // [N.s/m] reducao de forca por excesso de velocidade de retorno
  float overspeedMps = 3.0f;      // falha: cabo solto/arrebentado ou leitura absurda
  int maxStepJump = 40;           // falha: salto de passos entre duas telemetrias
  uint32_t telemetryTimeoutMs = 100;
  float maxBoardTempC = 70.0f;

  void setCeilingA(float a) {
    ceilingA = a < 0.0f ? 0.0f : (a > kHardMaxCurrentA ? kHardMaxCurrentA : a);
  }
};

class ForceLimiter {
 public:
  ForceLimiter(const MachineConfig& mc, const SafetyConfig& sc) : mc_(mc), sc_(sc) {}

  // homed = false: posicao desconhecida, nao aplica as regras que dependem dela.
  float apply(float requestN, float x, float v, float dt, bool homed);
  void reset() { outN_ = 0.0f; }  // zera na hora (emergencia / falha)
  float outputN() const { return outN_; }
  float ceilingN() const;

 private:
  const MachineConfig& mc_;
  const SafetyConfig& sc_;
  float outN_ = 0.0f;
};

}  // namespace pd
