# Changelog

All notable changes to this project are documented in this file.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/);
versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

### Added
- Nuevas trazas de validacion para documentacion y replay:
	- logs/PruebaLEO
	- logs/PruebaTierraAjustada
	- logs/PruebaTierraSinAjustar
- Monitor UART con capacidades de laboratorio ampliadas:
	- opcion --log para guardar UART en vivo
	- opcion --replay para reproducir logs offline
	- opcion --speed para acelerar o desacelerar replay

### Changed
- Configuracion activa de adquisicion orientada a alta dinamica (perfil LEO):
	- CFG_N_INT: 5 -> 2
	- CFG_COARSE_N_INT: 3 -> 1
	- CFG_HYST_ACQ_SWEEPS: 2 -> 1 (zero-delay)
	- CFG_HYST_REL_SWEEPS: 3 -> 2
- Compensacion de sesgo Doppler por oscilador aplicada por configuracion:
	- CFG_REAL_DOPPLER_BIAS_BINS: 0 -> -23
- Actualizacion completa de documentacion tecnica para reflejar:
	- comparativa Tierra vs LEO
	- interpretacion de deriva de fase (code Doppler)
	- criterios de trade-off sensibilidad vs velocidad
- Coherencia pre-release:
	- `scripts/analyze_capbin.py` ya no depende de un fichero por defecto inexistente; ahora autodetecta `.capbin` en `captures/` o muestra ayuda explicita.
	- Defaults de `acquisition_controller.vhd` alineados con el perfil activo (N_INT=2, COARSE_N_INT=1, HYST_ACQ=1).
	- Limpieza de comentarios desactualizados en `gps_config_pkg.vhd`.
	- Simplificacion de reglas TFM en `.gitignore` y añadido `*.dmp`.
	- Añadidos `captures/.gitkeep` y `build_logs/.gitkeep` para clon limpio.

## [1.1.0] — 2026-04-28 — Fase 12 & Skydel Ready

### Added
- Timeout adaptativo (`Dynamic Peak Timeout`) para modos coherentes runtime-seleccionables (1/2/4/8 ms) en la máquina de estados.
- Scripts de gobernanza y reporte de release firmado (`build_logs/latest_build.json`).
- Ventana 3 en el monitor de Python (`Espectro Doppler & Mapa de flags`) implementada y completamente funcional.

### Changed
- Deuda técnica cero: Eliminación de código zombi, FSM capture/replay redundantes y BOMs invisibles.

## [1.0.0] — 2026-04-13 — Pre-Skydel baseline

### Added
- FFT-based GPS L1 C/A acquisition receiver, full pipeline on Basys-3 (Artix-7 XC7A35T)
- `acquisition_controller.vhd`: 11-state FSM with coarse sweep, CFAR, dominance check, lock hysteresis
- `fft_controller.vhd`: FFT resource sharing (xfft_0 forward, xfft_1 configurable); 4 ms coherent accumulation
- `doppler_mixer.vhd`: 24-bit NCO + sign-magnitude→bipolar + I/Q mixing
- `gps_ca_generator.vhd`: BRAM ROM for all 32 PRN C/A codes
- `peak_detector.vhd`: argmax + noise floor estimation
- `multi_sat_rx_gen.vhd`: synthetic 3-satellite BPSK generator (sw[5] mux)
- `uart_reporter.vhd`: ASCII UART output (SAT / TOTAL / SWEEP lines at 115200 baud)
- `gps_config_pkg.vhd`: single source of truth for all tunable parameters
- 4 testbenches: peak_detector, acquisition_controller, synthetic_l1, fft_controller_smoke
- `scripts/gps_monitor.py`: real-time UART monitor
- `scripts/fpga_config.py`: parses `gps_config_pkg.vhd` constants for the monitor
- Full build/sim/CDC/timing automation scripts (PowerShell + Tcl)
- Comprehensive documentation in `docs/`

### Fixed
- FFT RX/CA synchronisation: atomic combined send (commit e420e53)
- Acquisition time improvements (commits 0363b15, 4a77d7f)

---

[Unreleased]: https://github.com/user/GPS_Acquisition_FPGA/compare/v1.1.0...HEAD
[1.1.0]: https://github.com/user/GPS_Acquisition_FPGA/releases/tag/v1.1.0
[1.0.0]: https://github.com/user/GPS_Acquisition_FPGA/releases/tag/v1.0.0
