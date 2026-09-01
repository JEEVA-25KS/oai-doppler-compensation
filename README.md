# Pre-Doppler Estimation and Compensation Framework for OAI 5G gNB

A real-time Doppler estimation and compensation pipeline integrated into the OAI
5G SA gNB: per-UE Doppler is estimated from successive SRS channel-phase
measurements in the MAC layer, smoothed, exposed to the PHY/RU layer through a
minimal API, and applied as frequency-domain (UL) or time-domain (DL)
pre-compensation.

## Objective
Develop and integrate a real-time Doppler estimation and compensation algorithm in
the OAI gNB.

## Architecture

```
MAC (SRS handler)                    PHY / RU
─────────────────                    ─────────
average SRS channel IQ
   → phase (atan2)
   → estimate_doppler_from_srs_phase()
   → doppler_shift / doppler_shift_filtered
   (stored per-UE in NR_UE_sched_ctrl_t)
                                      nr_mac_get_doppler_hz()  ◄── minimal API,
                                            │                      avoids pulling in
                                            ▼                      full MAC headers
                                      nr_doppler_get_for_slot()
                                            │
                              ┌─────────────┴─────────────┐
                              ▼                             ▼
                  UL: apply_ul_doppler_compensation_freq   DL: apply_pre_doppler_compensation_time
                  (frequency domain, post-FFT)              (time domain, post-OFDM-mod)
```

## Files created / modified

**Created:**
- `openair1/PHY/TOOLS/nr_doppler_compensation.h`
- `openair1/PHY/TOOLS/nr_doppler_compensation.c`
- `openair2/LAYER2/NR_MAC_gNB/nr_doppler_api.h`

**Modified:**
- `openair2/LAYER2/NR_MAC_gNB/nr_mac_gNB.h`
- `openair2/LAYER2/NR_MAC_gNB/gNB_scheduler_ulsch.c`
- `openair1/SCHED_NR/nr_ru_procedures.c`
- `openairinterface5g/CMakeLists.txt
`

## Contents

| Doc | Covers |
|-----|--------|
| [01-mac-side-estimation.md](docs/01-mac-side-estimation.md) | SRS-phase-based Doppler estimator, smoothing, and the minimal MAC→PHY API |
| [02-phy-side-compensation.md](docs/02-phy-side-compensation.md) | UL frequency-domain and DL time-domain compensation functions, and their RU call sites |
| [03-results.md](docs/03-results.md) | Live gNB log capture and interpretation of raw vs. filtered Doppler |

## Result summary

Validated on a live OAI 5G SA testbed. Raw per-SRS Doppler estimates swung roughly
between −3 Hz and +2 Hz (measurement noise + real channel variation); the two-stage
EMA-filtered output smoothed this into gradual updates between roughly −1.4 Hz and
+0.1 Hz. The filtered value — the one actually used for UL/DL compensation — lags
sudden jumps in the raw measurement by design, trading responsiveness for stability
against noisy/outlier SRS readings. Compensation only engages above a 0.1 Hz
threshold on both the UL (frequency-domain, post-FFT) and DL (time-domain,
post-OFDM-modulation) paths.
