#!/usr/bin/env python3
"""Gera um PNG por CSV do simulador: posicao, velocidade e forcas ao longo do tempo.

Uso: python3 tools/plot.py [pasta_dos_csv]      (padrao: out)
Precisa de matplotlib (pip install matplotlib).
"""
import csv
import glob
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

folder = sys.argv[1] if len(sys.argv) > 1 else "out"
files = sorted(glob.glob(os.path.join(folder, "*.csv")))
if not files:
    sys.exit(f"Nenhum CSV em {folder}/. Rode o simulador antes.")

for path in files:
    with open(path) as f:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
    t = [r["t"] for r in rows]
    fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    a1.plot(t, [r["x"] for r in rows], label="real")
    a1.plot(t, [r["x_est"] for r in rows], "--", label="estimada pelo ESP32")
    a1.set_ylabel("posicao [m]")
    a1.legend(loc="upper right")
    a2.plot(t, [r["v"] for r in rows])
    a2.axhline(0, color="gray", lw=0.5)
    a2.set_ylabel("velocidade [m/s]")
    a3.plot(t, [r["forca_mao_N"] / 9.80665 for r in rows], color="gray", lw=0.8, label="mao")
    a3.plot(t, [r["forca_cmd_N"] / 9.80665 for r in rows], "--", label="comandada")
    a3.plot(t, [r["forca_motor_N"] / 9.80665 for r in rows], label="motor (real)")
    a3.set_ylabel("forca [kg]")
    a3.set_xlabel("tempo [s]")
    a3.legend(loc="upper right")
    fault = [r["t"] for r in rows if r["falha"] > 0]
    if fault:
        for a in (a1, a2, a3):
            a.axvline(fault[0], color="red", lw=1)
    fig.suptitle(os.path.splitext(os.path.basename(path))[0])
    fig.tight_layout()
    out = path[:-4] + ".png"
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(out)
