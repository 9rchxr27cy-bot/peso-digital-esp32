# peso-digital-esp32

Firmware de um ESP32-S3 (N16R8) que é o cérebro de uma máquina de musculação caseira
com peso digital (tipo Speediance), feita com motores de hoverboard.

**Como responder ao dono do projeto:** em português, curto, conclusão primeiro.

## Sistema

- O ESP32 fala por UART com a placa original do hoverboard, regravada com o firmware
  EFeru `hoverboard-firmware-hack-FOC` em modo torque (fork: `9rchxr27cy-bot/hoverboard-firmware-hack-FOC`).
- O ESP32 calcula a força pedida em cada braço, manda o comando de torque e lê a telemetria.

## Contas

- Força [N] = torque / raio do carretel. Torque = 0,52 N·m/A × corrente.
- Posição: 90 passos de Hall por volta (6 estados × 15 pares de polos).

| Versão | Motores por braço | Raio | Força a 15 A | Cabo por passo | Curso máximo | Placas |
|---|---|---|---|---|---|---|
| 1M | 1 | 20 mm | ~390 N (~40 kg) | 1,40 mm | 1440 passos (~2,0 m) | 1 placa, um braço em cada saída |
| 2M | 2 (somam torque) | 25 mm | ~624 N (~64 kg) | 1,75 mm | 1170 passos (~2,0 m) | 1 placa por braço |

## Modos de treino

Cada modo é uma fórmula de força em função de posição e velocidade:
peso constante, excêntrico (mais carga na descida), corrente (carga cresce com a altura),
elástico, isocinético (velocidade fixa) e spotter (alivia quando o movimento trava).

## Segurança (obrigatória, nunca remover nem contornar)

- Teto de força configurável; começar em 5 A (~13 kg no 1M). **Nunca passar de 15 A**: as peças mecânicas são de plástico.
- Rampa de força: nada de degrau.
- Fim de curso por software. Zero = batente encostado na torre. Máximo = 1440 passos (1M) / 1170 (2M).
  Nunca soltar as voltas mortas do carretel.
- Botão físico de emergência que zera o comando.
- Comando periódico: a placa zera o torque sozinha se ficar sem comando.

A camada de segurança é a última etapa antes do UART: nenhum modo escreve o comando direto.

## Protocolo com a placa (conferido no fork, commit 4f141cb = upstream sem alterações)

- UART 115200 8N1, 3,3 V, little-endian. USART2 = cabo esquerdo da sideboard, USART3 = cabo direito (este é tolerante a 5 V).
- Comando ESP32 → placa, 8 bytes: `start=0xABCD (u16)`, `steer (i16)`, `speed (i16)`, `checksum (u16) = start ^ steer ^ speed`.
- Faixa do comando: −1000..+1000 = −I_MOT_MAX..+I_MOT_MAX em `TRQ_MODE` (com I_MOT_MAX = 15 A, 1 unidade = 15 mA).
- Com `TANK_STEERING`: `steer` → motor esquerdo, `speed` → motor direito, sem mistura.
- Telemetria placa → ESP32 a cada 10 ms (100 Hz), XOR de todos os campos no checksum:
  `start, cmd1, cmd2, speedR_meas, speedL_meas, [wheelR_cnt, wheelL_cnt], batVoltage, boardTemp, cmdLed, checksum`
  (i16/u16; 18 bytes, ou 22 com `ENABLE_ODOMETRY`). Velocidade em rpm, bateria em V×100, temperatura em °C×10.
- `wheelX_cnt` = contador de passos de Hall módulo 9000 (0..8999), só existe com `ENABLE_ODOMETRY`.
  O ESP32 desembrulha o contador. Não há corrente medida na telemetria.
- Timeout da placa: `SERIAL_TIMEOUT 160` ≈ 0,8 s sem quadro válido → `OPEN_MODE` (torque zero) e bipes.
- A placa aplica limitador de taxa + passa-baixa no comando (`DEFAULT_RATE 480`, `DEFAULT_FILTER 6553`), laço principal de 5 ms.

O fork ainda está sem os ajustes: aplicar `docs/fork-peso-digital.patch` (a sessão não teve acesso de
escrita ao fork). O patch liga `VARIANT_USART`, `TRQ_MODE`, `TANK_STEERING`, `ENABLE_ODOMETRY`,
`SERIAL_TIMEOUT 20` (0,1 s), `N_MOT_MAX 700` e troca a `FLASH_WRITE_KEY` (senão a placa reusa
`i_max`/`n_max` antigos da flash). A placa desliga sozinha após 8 min sem comando acima de ~50
(`INACTIVITY_TIMEOUT`); a tensão de repouso fica abaixo disso.

## Código

- `lib/core/`: núcleo portátil em C++17, sem Arduino. `protocol` (quadros), `modes` (fórmulas de força),
  `safety` (teto, rampa, fim de curso, retorno rápido), `arm` (estados Idle → Homing → Ready → Active),
  `machine` (liga placas e braços, falhas, gera os quadros). Laço de 100 Hz.
- `lib/sim/`: placa falsa que fala os mesmos bytes, modelo do motor/carretel/mão e cenários.
- `test/test_native/`: 18 testes; todo cenário verifica comando ≥ 0, ≤ teto e sem degrau.
- Convenção de sinais: força e comando positivos = recolher o cabo; posição e velocidade positivas = cabo saindo.
  Os sinais reais de cada motor ficam em `MachineConfig` (`MotorSigns`) e precisam ser conferidos no hardware.

Comandos:

- `pio test -e native` e `pio run -e native -t exec` (simulador, grava CSVs em `out/`).
- Sem acesso ao registro do PlatformIO (caso das sessões na nuvem): `tools/test_local.sh`.
- Rodar os testes depois de qualquer mudança em `lib/`.

## O que o simulador já mostrou

- O filtro de comando da placa atrasa tudo em ~100–150 ms: o isocinético oscila com ganho proporcional
  acima de ~400 N·s/m (por isso usa proporcional + integral), e o zero da emergência leva ~0,2 s para chegar ao motor.
- Pegador solto volta e bate no batente a ~1,2 m/s: o motor só puxa, não freia. Em aberto: permitir frenagem.
- O fim de curso máximo por software não segura um usuário mais forte que o teto; a garantia das voltas mortas é mecânica.
- Spotter não ajuda se a subida nem começa (só detecta subida que parou no meio).
- Os parâmetros do modelo (inércia, atrito) são estimativas; calibrar com o hardware.

## Estado

- Feito: núcleo, simulador e testes. Arquitetura aprovada pelo dono (PlatformIO, framework Arduino no ESP32).
- Falta: ambiente `esp32s3` (UART, botão de emergência por interrupção, console serial) e depois a interface
  (Bluetooth ou Wi-Fi com página web, a definir).
