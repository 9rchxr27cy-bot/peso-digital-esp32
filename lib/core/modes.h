// Modos de treino: cada um e uma formula de forca [N] em funcao da posicao x [m]
// (cabo para fora do batente) e da velocidade v [m/s] (positiva = puxando).
// A saida ainda passa pela camada de seguranca; nenhum modo fala com a placa.
#pragma once
#include <cstdint>

namespace pd {

enum class ModeType : uint8_t { Constant, Eccentric, Chain, Elastic, Isokinetic, Spotter };

struct ModeParams {
  ModeType type = ModeType::Constant;
  float baseN = 50.0f;  // carga base [N]

  // Excentrico: na descida a carga vira baseN * eccFactor.
  float eccFactor = 1.3f;
  float eccBlendMps = 0.05f;  // faixa de velocidade da transicao (evita trepidacao)

  // Corrente: baseN + chainNpm * x.
  float chainNpm = 60.0f;

  // Elastico: elasticNpm * (x - elasticStartM), nunca abaixo de baseN.
  float elasticNpm = 150.0f;
  float elasticStartM = 0.10f;

  // Isocinetico: carga leve ate isoSpeedMps; acima disso a carga sobe para segurar a velocidade
  // (proporcional + integral do excesso de velocidade).
  float isoSpeedMps = 0.40f;
  float isoGainNspm = 400.0f;  // mais que isso oscila com o filtro padrao da placa (ver simulador)
  float isoIntegralNpm = 1500.0f;

  // Spotter: aprende a altura do movimento na primeira repeticao; se uma subida parar
  // antes de chegar perto dessa altura, alivia a carga aos poucos e mantem o alivio na serie.
  // Limite conhecido: nao ajuda se a subida nem chega a comecar.
  float spotStallMps = 0.03f;    // subindo mais devagar do que isso (ou cedendo) conta como travado
  float spotStallTimeS = 0.8f;   // tempo travado antes de comecar a aliviar
  float spotMinPosM = 0.15f;     // perto do batente nao conta como travado
  float spotTopFraction = 0.85f; // a repeticao vale como completa acima desta fracao da altura aprendida
  float spotReliefPerS = 0.25f;  // fracao da carga retirada por segundo
  float spotMinFactor = 0.40f;   // nunca alivia abaixo desta fracao
  float spotRecoverPerS = 0.15f; // a carga so volta com o pegador de novo perto do batente
};

struct ModeState {
  float stallTimerS = 0.0f;
  float spotFactor = 1.0f;
  float isoIntegralN = 0.0f;
  float topM = 0.0f;     // maior altura ja alcancada na serie
  bool lifting = false;  // subida em andamento, ainda sem chegar ao alto
};

float modeForce(const ModeParams& p, ModeState& s, float x, float v, float dt);
const char* modeName(ModeType t);

}  // namespace pd
