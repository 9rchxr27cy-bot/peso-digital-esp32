// Simulador em PC: roda todos os cenarios nas duas versoes e grava um CSV por cenario.
//   pio run -e native -t exec          (ou: .pio/build/native/program [pasta_de_saida])
#include <algorithm>
#include <cstdio>
#include <string>
#include <sys/stat.h>

#include "scenario.h"

int main(int argc, char** argv) {
  std::string outDir = argc > 1 ? argv[1] : "out";
  mkdir(outDir.c_str(), 0755);

  std::printf("%-4s %-22s %8s %8s %8s %8s %8s  %s\n", "ver", "cenario", "Fmax[kg]", "Imax[A]", "xmax[m]", "vmax", "vmin",
              "falha");
  for (pd::Variant v : {pd::Variant::M1, pd::Variant::M2}) {
    const char* tag = v == pd::Variant::M1 ? "1M" : "2M";
    for (const pdsim::Scenario& sc : pdsim::standardScenarios(v)) {
      pdsim::Result r = pdsim::runScenario(sc);
      float fMax = 0, iMax = 0, vMax = 0, vMin = 0;
      for (const auto& s : r.samples) {
        fMax = std::max(fMax, s.tensionN);
        iMax = std::max(iMax, s.cmdCurrentA);
        vMax = std::max(vMax, s.v);
        vMin = std::min(vMin, s.v);
      }
      std::printf("%-4s %-22s %8.1f %8.2f %8.2f %8.2f %8.2f  %s\n", tag, sc.name.c_str(), pd::newtonToKg(fMax), iMax,
                  r.maxX, vMax, vMin, pd::faultName(r.fault));
      pdsim::writeCsv(r, outDir + "/" + tag + "_" + sc.name + ".csv");
    }
  }
  std::printf("CSVs em %s/. Graficos: python3 tools/plot.py %s\n", outDir.c_str(), outDir.c_str());
  return 0;
}
