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

Configuração que o fork ainda precisa (em `Inc/config.h`): `VARIANT_USART`, `CTRL_MOD_REQ TRQ_MODE`,
`TANK_STEERING`, `ENABLE_ODOMETRY`, `SERIAL_TIMEOUT` menor, conferir `I_MOT_MAX 15` e baixar `N_MOT_MAX`.

## Estado

- Interface: a definir (Bluetooth ou Wi-Fi com página web). Começar por um console serial.
- Firmware ainda não escrito. Antes de escrever, a arquitetura proposta precisa ser aprovada pelo dono.
