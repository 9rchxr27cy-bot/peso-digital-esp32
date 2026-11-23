# peso-digital-esp32

Firmware de um ESP32-S3 que comanda uma máquina de musculação caseira com peso digital,
feita com motores de hoverboard. O ESP32 fala por UART com a placa do hoverboard
(firmware EFeru FOC em modo torque), calcula a força de cada braço e aplica a segurança.

**Estado:** o núcleo (protocolo, modos, segurança) e o simulador em PC estão prontos e testados.
O código específico do ESP32 (UART, botão, console) ainda não foi escrito.

## Estrutura

| Pasta | Conteúdo |
|---|---|
| `lib/core/` | Núcleo portátil, sem dependência de Arduino: `protocol`, `modes`, `safety`, `arm`, `machine` |
| `lib/sim/` | Simulador: placa falsa (`board_sim`), motor + carretel + mão (`plant`), cenários (`scenario`) |
| `src/sim_main.cpp` | Programa do simulador: roda todos os cenários e grava CSVs |
| `test/test_native/` | Testes automáticos de protocolo, segurança e modos |
| `tools/` | `plot.py` (gráficos) e `test_local.sh` (testes sem PlatformIO) |
| `docs/fork-peso-digital.patch` | Ajustes necessários no fork do firmware da placa |

## Como rodar no PC

Com PlatformIO:

```bash
pio run -e native -t exec
```

```bash
pio test -e native
```

Sem PlatformIO (só `g++` e `git`):

```bash
tools/test_local.sh
```

Gráficos dos cenários (precisa de `matplotlib`):

```bash
python3 tools/plot.py out
```

## Firmware da placa

Aplicar no fork `hoverboard-firmware-hack-FOC`:

```bash
git apply docs/fork-peso-digital.patch
```

O patch liga `VARIANT_USART`, modo torque, `TANK_STEERING` e `ENABLE_ODOMETRY`, baixa o
`SERIAL_TIMEOUT` para 0,1 s e o `N_MOT_MAX` para 700 rpm, e troca a `FLASH_WRITE_KEY`
para a placa não reaproveitar limites antigos gravados na flash.
