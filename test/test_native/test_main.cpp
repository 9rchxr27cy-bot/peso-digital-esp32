// Testes em PC (pio test -e native): protocolo, seguranca e modos, usando o simulador.
#include <unity.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <string>

#include "machine.h"
#include "protocol.h"
#include "scenario.h"

using pdsim::Result;
using pdsim::Sample;
using pdsim::Scenario;

void setUp() {}
void tearDown() {}

static const pd::Variant kVariants[] = {pd::Variant::M1, pd::Variant::M2};
static const float kT0 = 6.0f;  // inicio do usuario nos cenarios padrao

static Scenario find(pd::Variant v, const std::string& name) {
  for (const Scenario& s : pdsim::standardScenarios(v))
    if (s.name == name) return s;
  TEST_FAIL_MESSAGE(("cenario inexistente: " + name).c_str());
  return Scenario{};
}

static Result run(pd::Variant v, const std::string& name) {
  static std::map<std::string, Result> cache;
  std::string key = std::string(v == pd::Variant::M1 ? "1M_" : "2M_") + name;
  auto it = cache.find(key);
  if (it == cache.end()) it = cache.emplace(key, pdsim::runScenario(find(v, name))).first;
  return it->second;
}

static pd::MachineConfig cfg(pd::Variant v) { return v == pd::Variant::M1 ? pd::config1M() : pd::config2M(); }

// ---------------------------------------------------------------- protocolo

static void test_command_roundtrip() {
  uint8_t buf[pd::kCommandLen];
  pd::encodeCommand(pd::Command{-123, 456}, buf);
  TEST_ASSERT_EQUAL_HEX8(0xCD, buf[0]);
  TEST_ASSERT_EQUAL_HEX8(0xAB, buf[1]);
  pd::CommandParser p;
  pd::Command c;
  bool ok = false;
  for (uint8_t b : buf) ok = p.feed(b, c);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_EQUAL_INT16(-123, c.steer);
  TEST_ASSERT_EQUAL_INT16(456, c.speed);
}

static void test_feedback_resync_and_bad_checksum() {
  pd::Feedback f;
  f.speedL = -77;
  f.wheelL = 8999;
  f.wheelR = 12;
  f.batV100 = 3900;
  uint8_t good[pd::kFeedbackLen], bad[pd::kFeedbackLen];
  pd::encodeFeedback(f, good);
  pd::encodeFeedback(f, bad);
  bad[7] ^= 0x10;  // um bit trocado

  pd::FeedbackParser p;
  pd::Feedback out;
  int frames = 0;
  const uint8_t junk[] = {0x00, 0xCD, 0x11, 0xCD};  // lixo com meio start frame
  for (uint8_t b : junk) frames += p.feed(b, out);
  for (uint8_t b : bad) frames += p.feed(b, out);
  TEST_ASSERT_EQUAL(0, frames);
  for (int k = 0; k < 2; ++k)  // o primeiro quadro bom pode ser gasto para ressincronizar
    for (uint8_t b : good) frames += p.feed(b, out);
  TEST_ASSERT_GREATER_OR_EQUAL(1, frames);
  TEST_ASSERT_EQUAL_INT16(-77, out.speedL);
  TEST_ASSERT_EQUAL_INT16(8999, out.wheelL);
  TEST_ASSERT_EQUAL_INT16(3900, out.batV100);
  TEST_ASSERT_GREATER_OR_EQUAL(1, p.badFrames());
}

static void test_odom_wraps() {
  TEST_ASSERT_EQUAL(3, pd::odomDelta(1, 8998));
  TEST_ASSERT_EQUAL(-3, pd::odomDelta(8998, 1));
  TEST_ASSERT_EQUAL(10, pd::odomDelta(110, 100));
}

static void test_force_math() {
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 390.0f, pd::config1M().newtonPerAmp() * 15.0f);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 624.0f, pd::config2M().newtonPerAmp() * 15.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.40f, pd::config1M().metersPerStep() * 1000.0f);
}

// ---------------------------------------------------------------- seguranca

static void test_ceiling_never_above_15A() {
  pd::SafetyConfig sc;
  TEST_ASSERT_EQUAL_FLOAT(5.0f, sc.ceilingA);  // comeca em 5 A
  sc.setCeilingA(40.0f);
  TEST_ASSERT_EQUAL_FLOAT(15.0f, sc.ceilingA);
  sc.setCeilingA(-3.0f);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, sc.ceilingA);
}

// Em todos os cenarios: comando nunca negativo, nunca acima do teto, nunca em degrau.
static void test_invariants_all_scenarios() {
  pd::SafetyConfig def;
  for (pd::Variant v : kVariants) {
    for (const Scenario& sc : pdsim::standardScenarios(v)) {
      Result r = run(v, sc.name);
      const char* n = sc.name.c_str();
      float ceilN = sc.ceilingA * cfg(v).newtonPerAmp();
      float up = (sc.slewUpNps > 0 ? sc.slewUpNps : def.slewUpNps) * 0.01f;
      TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(0, r.minCmd, n);  // nunca desenrola o cabo
      TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(static_cast<int>(sc.ceilingA / 15.0f * 1000.0f) + 1, r.maxCmd, n);
      TEST_ASSERT_TRUE_MESSAGE(r.activeAtS > 0, n);       // o homing terminou
      TEST_ASSERT_FLOAT_WITHIN_MESSAGE(2 * cfg(v).metersPerStep(), 0.0f, r.homingErrorM, n);
      float prev = 0.0f;
      for (const Sample& s : r.samples) {
        TEST_ASSERT_TRUE_MESSAGE(s.cmdCurrentA <= sc.ceilingA + 1e-3f, n);
        TEST_ASSERT_TRUE_MESSAGE(s.cmdForceN <= ceilN + 1e-2f, n);
        TEST_ASSERT_TRUE_MESSAGE(s.tensionN <= ceilN * 1.02f + 0.5f, n);
        TEST_ASSERT_TRUE_MESSAGE(s.cmdForceN - prev <= up + 1e-2f, n);  // rampa
        // perto do batente, so tensao de repouso
        if (s.state == static_cast<int>(pd::ArmState::Active) && s.estX <= def.landingZoneM)
          TEST_ASSERT_TRUE_MESSAGE(s.cmdForceN <= std::max(prev, def.restForceN) + 1e-2f, n);
        prev = s.cmdForceN;
      }
    }
  }
}

static void test_estop_zeroes_command() {
  for (pd::Variant v : kVariants) {
    Result r = run(v, "emergencia");
    TEST_ASSERT_EQUAL(static_cast<int>(pd::Fault::Estop), static_cast<int>(r.fault));
    float tE = kT0 + 1.0f;
    bool loaded = false;
    for (const Sample& s : r.samples) {
      if (s.t < tE && s.tensionN > 50.0f) loaded = true;
      if (s.t >= tE + 0.011f) TEST_ASSERT_EQUAL_FLOAT(0.0f, s.cmdForceN);  // no ciclo seguinte
      if (s.t >= tE + 0.20f) TEST_ASSERT_TRUE(s.tensionN < 4.0f);   // o filtro da placa atrasa o zero
      if (s.t >= tE + 0.50f) TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, s.tensionN);
    }
    TEST_ASSERT_TRUE(loaded);
  }
}

static void test_telemetry_loss_zeroes_command() {
  for (pd::Variant v : kVariants) {
    Result r = run(v, "sem_telemetria");
    TEST_ASSERT_EQUAL(static_cast<int>(pd::Fault::TelemetryTimeout), static_cast<int>(r.fault));
    for (const Sample& s : r.samples)
      if (s.t >= kT0 + 1.0f + 0.13f) TEST_ASSERT_EQUAL_FLOAT(0.0f, s.cmdForceN);
  }
}

static void test_board_watchdog_when_esp32_hangs() {
  for (pd::Variant v : kVariants) {
    Result r = run(v, "esp32_travado");
    for (const Sample& s : r.samples)
      if (s.t >= kT0 + 1.0f + 0.15f) TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, s.tensionN);

    // Com o SERIAL_TIMEOUT original (160) a placa segura a carga por ~0,8 s: por isso o ajuste no fork.
    Scenario slow = find(v, "esp32_travado");
    slow.serialTimeoutLoops = 160;
    Result r2 = pdsim::runScenario(slow);
    bool stillLoaded = false;
    for (const Sample& s : r2.samples)
      if (s.t > kT0 + 1.5f && s.t < kT0 + 1.7f && s.tensionN > 50.0f) stillLoaded = true;
    TEST_ASSERT_TRUE(stillLoaded);
  }
}

static void test_fault_requires_clear_and_rehoming() {
  pd::Machine m(pd::Variant::M1);
  pd::Feedback f;
  uint8_t buf[pd::kFeedbackLen];
  pd::encodeFeedback(f, buf);
  m.onRx(0, buf, sizeof buf, 0);
  m.step(5);
  TEST_ASSERT_TRUE(m.startHoming());
  m.setEstop(true);
  m.step(15);
  TEST_ASSERT_EQUAL(static_cast<int>(pd::Fault::Estop), static_cast<int>(m.fault()));
  TEST_ASSERT_FALSE(m.clearFault());   // botao ainda apertado
  TEST_ASSERT_FALSE(m.startHoming());
  m.setEstop(false);
  TEST_ASSERT_FALSE(m.arm(0).start()); // perdeu o zero: precisa de novo homing
  TEST_ASSERT_TRUE(m.clearFault());
  TEST_ASSERT_EQUAL(static_cast<int>(pd::ArmState::Idle), static_cast<int>(m.arm(0).state()));
}

static void test_release_handle_is_bounded() {
  for (pd::Variant v : kVariants) {
    Result r = run(v, "soltar_pegador");
    float vMin = 0.0f;
    for (const Sample& s : r.samples) vMin = std::min(vMin, s.v);
    TEST_ASSERT_TRUE(vMin > -1.6f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, r.samples.back().x);  // voltou ao batente
    TEST_ASSERT_EQUAL(static_cast<int>(pd::Fault::None), static_cast<int>(r.fault));
  }
}

static void test_end_of_travel_wall() {
  for (pd::Variant v : kVariants) {
    Scenario sc = find(v, "puxar_alem_do_curso");
    Result r = run(v, sc.name);
    float ceilN = sc.ceilingA * cfg(v).newtonPerAmp(), fAtEnd = 0.0f;
    for (const Sample& s : r.samples)
      if (s.estX > cfg(v).maxTravelM() - 0.03f) fAtEnd = std::max(fAtEnd, s.cmdForceN);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, ceilN, fAtEnd);  // no fim do curso a forca vai ao teto
  }
}

// ---------------------------------------------------------------- modos

// Media da forca real do motor nas amostras que passam no filtro, depois da 1a repeticao.
template <typename Pred>
static float meanTension(const Result& r, Pred pred) {
  double sum = 0;
  int n = 0;
  for (const Sample& s : r.samples)
    if (s.t > kT0 + 4.0f && pred(s)) sum += s.tensionN, ++n;
  TEST_ASSERT_GREATER_THAN(10, n);
  return static_cast<float>(sum / n);
}

static void test_mode_constant() {
  for (pd::Variant v : kVariants) {
    float f = meanTension(run(v, "constante"), [](const Sample& s) { return s.x > 0.25f; });
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 80.0f, f);
  }
}

static void test_mode_eccentric() {
  for (pd::Variant v : kVariants) {
    Result r = run(v, "excentrico");
    float up = meanTension(r, [](const Sample& s) { return s.v > 0.3f && s.x > 0.3f; });
    float down = meanTension(r, [](const Sample& s) { return s.v < -0.3f && s.x > 0.3f; });
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 80.0f, up);
    TEST_ASSERT_FLOAT_WITHIN(4.0f, 104.0f, down);  // 30% a mais na descida
  }
}

static void test_mode_chain() {
  for (pd::Variant v : kVariants) {
    Result r = run(v, "corrente");
    float low = meanTension(r, [](const Sample& s) { return s.x > 0.28f && s.x < 0.32f; });
    float high = meanTension(r, [](const Sample& s) { return s.x > 0.78f && s.x < 0.82f; });
    TEST_ASSERT_FLOAT_WITHIN(4.0f, 80.0f + 60.0f * 0.30f, low);
    TEST_ASSERT_FLOAT_WITHIN(4.0f, 80.0f + 60.0f * 0.80f, high);
  }
}

static void test_mode_elastic() {
  for (pd::Variant v : kVariants) {
    Result r = run(v, "elastico");
    float mid = meanTension(r, [](const Sample& s) { return s.x > 0.48f && s.x < 0.52f; });
    float high = meanTension(r, [](const Sample& s) { return s.x > 0.78f && s.x < 0.82f; });
    TEST_ASSERT_FLOAT_WITHIN(6.0f, 150.0f * 0.40f, mid);
    TEST_ASSERT_FLOAT_WITHIN(6.0f, 150.0f * 0.70f, high);
  }
}

static void test_mode_isokinetic_limits_speed() {
  for (pd::Variant v : kVariants) {
    Scenario iso = find(v, "isocinetico");
    Result r = run(v, iso.name);
    Scenario free = iso;  // mesmo usuario com carga leve constante, para comparar
    free.mode.type = pd::ModeType::Constant;
    Result rf = pdsim::runScenario(free);
    // Velocidade media de subida no meio do curso, depois da primeira repeticao.
    auto meanUp = [](const Result& res) {
      double sum = 0;
      int n = 0;
      for (const Sample& s : res.samples)
        if (s.t > kT0 + 2.0f && s.v > 0.05f && s.x > 0.45f && s.x < 0.75f) sum += s.v, ++n;
      return n ? static_cast<float>(sum / n) : 0.0f;
    };
    float vIso = meanUp(r), vFree = meanUp(rf);
    TEST_ASSERT_TRUE(vFree > 2.0f * iso.mode.isoSpeedMps);            // sem o modo, o usuario vai bem mais rapido
    TEST_ASSERT_FLOAT_WITHIN(0.05f, iso.mode.isoSpeedMps, vIso);      // com o modo, fica no alvo
    float peak = 0.0f;  // a arrancada passa do alvo (atraso do filtro da placa), mas tem limite
    for (const Sample& s : r.samples)
      if (s.t > kT0 + 2.0f) peak = std::max(peak, s.v);
    TEST_ASSERT_TRUE(peak < 2.5f * iso.mode.isoSpeedMps);
  }
}

static void test_mode_spotter_helps_tired_user() {
  for (pd::Variant v : kVariants) {
    Result with = run(v, "spotter"), without = run(v, "cansado_sem_spotter");
    float minFactor = 1.0f, topWith = 0.0f, topWithout = 0.0f;
    for (const Sample& s : with.samples) {
      minFactor = std::min(minFactor, s.spotFactor);
      if (s.t > 22.0f) topWith = std::max(topWith, s.x);
    }
    for (const Sample& s : without.samples)
      if (s.t > 22.0f) topWithout = std::max(topWithout, s.x);
    TEST_ASSERT_TRUE(minFactor < 0.9f);              // aliviou
    TEST_ASSERT_TRUE(minFactor >= 0.4f - 1e-3f);     // mas nunca abaixo do minimo
    TEST_ASSERT_TRUE(topWithout < 0.70f);            // sem ajuda nao completa a repeticao
    TEST_ASSERT_TRUE(topWith > 0.80f);               // com ajuda completa
    // Sem cansaco o spotter nao mexe na carga.
    Scenario fresh = find(v, "constante");
    fresh.mode.type = pd::ModeType::Spotter;
    Result rf = pdsim::runScenario(fresh);
    for (const Sample& s : rf.samples) TEST_ASSERT_TRUE(s.spotFactor > 0.95f);
  }
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_command_roundtrip);
  RUN_TEST(test_feedback_resync_and_bad_checksum);
  RUN_TEST(test_odom_wraps);
  RUN_TEST(test_force_math);
  RUN_TEST(test_ceiling_never_above_15A);
  RUN_TEST(test_invariants_all_scenarios);
  RUN_TEST(test_estop_zeroes_command);
  RUN_TEST(test_telemetry_loss_zeroes_command);
  RUN_TEST(test_board_watchdog_when_esp32_hangs);
  RUN_TEST(test_fault_requires_clear_and_rehoming);
  RUN_TEST(test_release_handle_is_bounded);
  RUN_TEST(test_end_of_travel_wall);
  RUN_TEST(test_mode_constant);
  RUN_TEST(test_mode_eccentric);
  RUN_TEST(test_mode_chain);
  RUN_TEST(test_mode_elastic);
  RUN_TEST(test_mode_isokinetic_limits_speed);
  RUN_TEST(test_mode_spotter_helps_tired_user);
  return UNITY_END();
}
