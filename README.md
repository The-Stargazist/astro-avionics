# Astro Avionics - Flight Computer

Flight computer system for rocket missions.

## Project Structure

- **`firmware/`** - Current flight computer firmware for the VADA board
  (two Raspberry Pi Pico 2). One PlatformIO project with an environment per
  board: `pico_a` (deployment) and `pico_b` (sensors, telemetry, logging).
  See [`firmware/README.md`](firmware/README.md).
- **`vada/`** - VADA PCB (KiCad schematic, layout, STEP model).
- **`dosa-2025/`** - Previous flight computer code (archived)
  - Arduino Nano ESP32 + Raspberry Pi based system, kept for reference only

## Getting Started

Open `firmware/` in VS Code with the PlatformIO extension, or use the CLI:

```bash
cd firmware
```

```bash
pio run
```

```bash
pio test -e native
```

Before flying, read the board setup and hardware notes in
[`firmware/README.md`](firmware/README.md).

## License

See [`LICENSE`](LICENSE) for license information.
