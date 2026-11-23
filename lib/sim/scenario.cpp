#include "scenario.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>

#include "board_sim.h"
#include "plant.h"

namespace pdsim {

UserFn repsUser(float lo, float hi, float periodS, float t0, std::function<float(float)> strength) {
  return [=](float t, float x, float v) {
    if (t < t0) return 0.0f;
    float ph = pd::kTwoPi * (t - t0) / periodS;
    float xr = lo + (hi - lo) * 0.5f * (1.0f - std::cos(ph));
    float vr = (hi - lo) * 0.5f * std::sin(ph) * pd::kTwoPi / periodS;
    float f = 2500.0f * (xr - x) + 250.0f * (vr - v);
    float ecc = 1.0f + 0.4f * std::clamp(-v / 0.3f, 0.0f, 1.0f);
    return std::clamp(f, 0.0f, std::max(0.0f, strength(t)) * ecc);
  };
}

Result runScenario(const Scenario& sc) {
  pd::Machine m(sc.variant);
  m.safety().setCeilingA(sc.ceilingA);
  if (sc.slewUpNps > 0) m.safety().slewUpNps = sc.slewUpNps;
  const pd::MachineConfig& mc = m.config();

  ArmPlant plants[2];
  for (auto& p : plants) {
    p.radiusM = mc.spoolRadiusM;
    p.motors = mc.motorsPerArm;
    p.x = sc.initialX;
    p.user = sc.user;
  }
  std::vector<std::unique_ptr<BoardSim>> boards;
  if (sc.variant == pd::Variant::M1) {
    boards.push_back(std::make_unique<BoardSim>(&plants[0], &plants[1]));
  } else {
    // no 2M os dois motores da placa estao no mesmo carretel
    boards.push_back(std::make_unique<BoardSim>(&plants[0], &plants[0]));
    boards.push_back(std::make_unique<BoardSim>(&plants[1], &plants[1]));
  }
  for (auto& b : boards) b->serialTimeoutLoops = sc.serialTimeoutLoops;

  Result r;
  for (int a = 0; a < pd::kNumArms; ++a) m.arm(a).setMode(sc.mode);
  bool homingAsked = false, started = false;
  std::vector<uint8_t> tx;
  const int totalMs = static_cast<int>(sc.durationS * 1000.0f);

  for (int ms = 0; ms < totalMs; ++ms) {
    float t = ms / 1000.0f;
    bool telemetryCut = sc.cutTelemetryAtS >= 0 && t >= sc.cutTelemetryAtS;
    bool commandCut = sc.cutCommandAtS >= 0 && t >= sc.cutCommandAtS;

    for (size_t i = 0; i < boards.size(); ++i) {
      tx.clear();
      boards[i]->tick1ms(tx);
      if (!telemetryCut && !tx.empty()) m.onRx(static_cast<int>(i), tx.data(), tx.size(), ms);
    }

    if (ms % pd::kControlPeriodMs == 5) {  // defasado da telemetria, como no hardware
      if (sc.estopAtS >= 0 && t >= sc.estopAtS) m.setEstop(true);
      if (!homingAsked && t > 0.2f) homingAsked = m.startHoming();
      if (!started && m.arm(0).state() == pd::ArmState::Ready && m.arm(1).state() == pd::ArmState::Ready) {
        r.homingErrorM = std::fabs(plants[0].x - m.arm(0).positionM());
        for (int a = 0; a < pd::kNumArms; ++a) m.arm(a).start();
        started = true;
        r.activeAtS = t;
      }
      if (!commandCut) {
        m.step(ms);
        for (size_t i = 0; i < boards.size(); ++i) boards[i]->rx(m.txFrame(static_cast<int>(i)), pd::kCommandLen);
      }
      const pd::ArmController& a0 = m.arm(0);
      r.samples.push_back(Sample{t, plants[0].x, plants[0].v, plants[0].userForceN, plants[0].tensionN(),
                                 a0.forceN(), a0.currentA(), a0.positionM(), a0.velocityMps(),
                                 static_cast<int>(a0.state()), static_cast<int>(m.fault()),
                                 a0.modeState().spotFactor});
    }

    for (auto& p : plants) p.motorTorqueNm = 0.0f;
    for (auto& b : boards) b->addTorque();
    for (auto& p : plants) p.step(t, 0.001f);
  }

  r.maxX = plants[0].maxX;
  r.fault = m.fault();
  for (auto& b : boards) {
    r.minCmd = std::min<int>(r.minCmd, b->minCmdSeen());
    r.maxCmd = std::max<int>(r.maxCmd, b->maxCmdSeen());
  }
  return r;
}

std::vector<Scenario> standardScenarios(pd::Variant v) {
  const float t0 = 6.0f;  // o usuario comeca depois do homing
  auto strong = [](float) { return 400.0f; };
  auto reps = repsUser(0.20f, 0.90f, 4.0f, t0, strong);
  // Cansaco: no meio da terceira subida a forca maxima cai abaixo da carga (80 N).
  auto tired = [=](float t) { return t < t0 + 8.7f ? 130.0f : 65.0f; };

  auto base = [&](const char* name, pd::ModeType type) {
    Scenario s;
    s.name = name;
    s.variant = v;
    s.mode.type = type;
    s.mode.baseN = 80.0f;
    s.user = reps;
    s.durationS = 22.0f;
    return s;
  };

  std::vector<Scenario> out;
  out.push_back(base("constante", pd::ModeType::Constant));
  out.push_back(base("excentrico", pd::ModeType::Eccentric));
  out.push_back(base("corrente", pd::ModeType::Chain));
  {
    Scenario s = base("elastico", pd::ModeType::Elastic);
    s.mode.baseN = 20.0f;
    out.push_back(s);
  }
  {
    Scenario s = base("isocinetico", pd::ModeType::Isokinetic);
    s.mode.baseN = 30.0f;
    s.ceilingA = 8.0f;
    s.slewUpNps = 1500.0f;  // o isocinetico precisa reagir rapido; ainda e rampa, nao degrau
    // Usuario tenta puxar bem mais rapido do que a velocidade alvo (0,4 m/s).
    s.user = repsUser(0.20f, 0.90f, 2.0f, t0, [](float) { return 150.0f; });
    out.push_back(s);
  }
  {
    Scenario s = base("spotter", pd::ModeType::Spotter);
    s.user = repsUser(0.30f, 0.90f, 4.0f, t0, tired);
    s.durationS = 30.0f;
    out.push_back(s);
  }
  {
    // Mesmo usuario cansado, sem spotter, para comparar.
    Scenario s = base("cansado_sem_spotter", pd::ModeType::Constant);
    s.user = repsUser(0.30f, 0.90f, 4.0f, t0, tired);
    s.durationS = 30.0f;
    out.push_back(s);
  }
  {
    // Solta o pegador no alto da repeticao.
    Scenario s = base("soltar_pegador", pd::ModeType::Constant);
    s.user = repsUser(0.20f, 0.90f, 4.0f, t0, [=](float t) { return t < t0 + 2.0f ? 400.0f : 0.0f; });
    s.durationS = 14.0f;
    out.push_back(s);
  }
  {
    // Puxa alem do fim de curso com muito mais forca do que o teto.
    Scenario s = base("puxar_alem_do_curso", pd::ModeType::Constant);
    s.user = repsUser(0.20f, 2.40f, 8.0f, t0, [](float) { return 100.0f; });
    out.push_back(s);
  }
  {
    Scenario s = base("emergencia", pd::ModeType::Constant);
    s.estopAtS = t0 + 1.0f;
    s.durationS = 10.0f;
    out.push_back(s);
  }
  {
    Scenario s = base("sem_telemetria", pd::ModeType::Constant);
    s.cutTelemetryAtS = t0 + 1.0f;
    s.durationS = 10.0f;
    out.push_back(s);
  }
  {
    Scenario s = base("esp32_travado", pd::ModeType::Constant);
    s.cutCommandAtS = t0 + 1.0f;
    s.durationS = 10.0f;
    out.push_back(s);
  }
  return out;
}

bool writeCsv(const Result& r, const std::string& path) {
  FILE* f = std::fopen(path.c_str(), "w");
  if (!f) return false;
  std::fprintf(f, "t,x,v,forca_mao_N,forca_motor_N,forca_cmd_N,corrente_cmd_A,x_est,v_est,estado,falha,fator_spotter\n");
  for (const Sample& s : r.samples)
    std::fprintf(f, "%.3f,%.4f,%.4f,%.2f,%.2f,%.2f,%.3f,%.4f,%.4f,%d,%d,%.3f\n", s.t, s.x, s.v, s.userN, s.tensionN,
                 s.cmdForceN, s.cmdCurrentA, s.estX, s.estV, s.state, s.fault, s.spotFactor);
  std::fclose(f);
  return true;
}

}  // namespace pdsim
