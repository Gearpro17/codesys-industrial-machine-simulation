# Smoke Test: CODESYS Soft PLC to Modbus TCP

**Purpose:** prove the riskiest integration link before writing the plant simulator: a CODESYS soft PLC acting as Modbus TCP client (master) reading a discrete input and writing a coil on a Modbus TCP server (field-device stand-in).

**Result:** PASSED. Input and coil exchange worked in both directions with 0 errors.

## Setup

| Item | Value |
|---|---|
| IDE | CODESYS V3.5 SP22 Patch 2 |
| Soft PLC | CODESYS Control Win V3 x64, 3.5.22.20 |
| Server (stand-in) | ModRSsim2 v8.21.2.9, MODBUS/TCP, port 502 |
| Network | localhost (`127.0.0.1`) |

### Device tree

```
Device (CODESYS Control Win V3 x64)
├─ PLC Logic / Application
└─ Ethernet (Ethernet)
   └─ Modbus_TCP_Client          (Modbus TCP Client = master)
      └─ Modbus_TCP_Server       (ModbusTCP Server Device = slave)
```

### Server device configuration

| Setting | Value |
|---|---|
| IP address | `127.0.0.1` |
| Port | `502` |
| **Unit ID** | **`1`** (see gotcha 1) |
| Channel 1 | FC02 Read Discrete Inputs, offset 0x0000, length 1 |
| Channel 2 | FC05 Write Single Coil, offset 0x0000, length 1 |

### I/O mapping

Only the individual bits were mapped. The parent array rows stay empty.

| Modbus | Address | Variable |
|---|---|---|
| Discrete input 0 | `%IX0.0` | `GVL.xInput` |
| Coil 0 | `%QX0.0` | `GVL.xOutput` |

### Test logic

Temporary `SmokeTest` program, called from `MainTask`, then removed after the test:

```iecst
GVL.xOutput := GVL.xInput;
```

## Procedure

1. Start CODESYS Control Win V3 x64 (Start PLC from the tray icon) and confirm the CODESYS Gateway is running.
2. In the IDE: Device, Communication Settings, Scan Network, select the PLC, set it active.
3. Make sure Simulation is **off**.
4. Build, Generate Code (0 errors), then Login, then Start (F5). Status bar shows RUN.
5. Start ModRSsim2 as MODBUS/TCP on port 502.
6. Open `Modbus_TCP_Server`, Status tab, and check the connection.
7. Toggle discrete input 0 in ModRSsim2 and check that `GVL.xInput` and coil 0 follow.

## Observations

- Status tab: `ComState: TCP connection established`, request counter in the thousands and increasing, error counter 0.
- ModRSsim2: `Connected (1/10)`, with received/sent counters increasing.
- Toggling DI 0 changed `GVL.xInput` online, and coil 0 followed it (`01 05 00 00 FF 00` for ON, `01 05 00 00 00 00` for OFF).

### Wire format (ModRSsim2 log)

| Frame | Meaning |
|---|---|
| `RX 0A BC 00 00 00 06 01 02 00 00 00 01` | Unit 1, FC02 read discrete inputs, start 0, quantity 1 |
| `TX 0A BC 00 00 00 04 01 02 01 01` | Response: byte count 1, data `0x01` (DI 0 = TRUE) |
| `RX 0A BD 00 00 00 06 01 05 00 00 FF 00` | Unit 1, FC05 write single coil, address 0, value `FF00` = ON |
| `TX 0A BD ...` | Echo of the request (FC05 response) |

Structure: 2 bytes transaction ID, 2 bytes protocol ID (`00 00`), 2 bytes length, then unit ID, function code and data (MBAP header plus PDU). The C++ runtime server must parse and answer exactly these frames.

### Polling rate

Timestamps show each channel exchanged about every **100 ms** (default channel cycle), not every 10 ms. The C++ loop can run at 10 ms, but the PLC only sees plant values at its polling rate. The cycle time is configurable in the Modbus Server Channel tab. **Do not claim 10 ms end-to-end I/O in the README.**

## Gotchas

1. **Unit ID.** The device defaulted to `255`, which caused `ComState: SOCKET_ERROR`, `ModbusTCPServer: Not running` and no traffic in the simulator. Setting it to `1` fixed it. The C++ server must answer to the same Unit ID, or ignore it deliberately and document that.
2. **Terminology.** In this CODESYS version, "Modbus TCP Master" is **Modbus TCP Client** and "Slave" is **ModbusTCP Server Device**. The roles are unchanged: the PLC polls, the field device answers.
3. **Add Device context.** Fieldbus devices can only be added under the PLC device (Ethernet, then Client, then Server), not under the project root.
4. **Simulation mode.** With Simulation on, Modbus never reaches the network. Turn it off.
5. **Mapping bits, not arrays.** The array row and its child bit cannot both be mapped ("Multiple mapping to an output is not allowed"). Map the bit only.
6. **POU scoping (C0037).** `PLC_PRG.xOutput := ...` from another program fails with `'xOutput' is no input of 'PLC_PRG'`. Only `VAR_INPUT` can be written from outside. Fix: put the I/O image in the GVL (`GVL.xInput`, `GVL.xOutput`).
7. **Runtime version mismatch.** A Control Win runtime that doesn't match the IDE version caused connection problems. Install the matching version (3.5.22.20) into its own empty folder, not the IDE folder.

## Decisions for the real project

- Keep all Modbus-facing signals in the **GVL**, the I/O image. The state machine reads and writes `GVL.*` only.
- The C++ runtime implements a Modbus TCP server (Unit ID 1, port 502) supporting FC01, FC02, FC03, FC04, FC05, FC06, FC15 and FC16, replacing ModRSsim2.
- Consider lowering the channel cycle time (for example 20 to 50 ms) when the real I/O map is configured, and document the effective latency.

## Artifacts

- CODESYS project export (PLCopenXML) saved in `docs/`.
