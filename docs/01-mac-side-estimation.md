# 1. MAC-Side Doppler Estimation

## `estimate_doppler_from_srs_phase()` — `gNB_scheduler_ulsch.c`

Converts successive SRS channel-phase measurements for a UE into a smoothed Doppler
frequency estimate (Hz), logs it, and updates per-UE history for the next estimate.

```c
static void estimate_doppler_from_srs_phase(NR_UE_info_t *UE,
                                              int frame,
                                              int slot,
                                              double current_phase)
{
  if (!UE) return;
  NR_UE_sched_ctrl_t *sched_ctrl = &UE->UE_sched_ctrl;
  double time_diff_s = 0.0;
  if (sched_ctrl->last_srs_frame >= 0) {
    time_diff_s = ((double)(frame - sched_ctrl->last_srs_frame) * 0.01) +
                  ((double)(slot - sched_ctrl->last_srs_slot) * 0.0005);
  }
  if (time_diff_s > 0.0) {
    double phase_diff = current_phase - sched_ctrl->last_srs_phase;
    while (phase_diff > M_PI) phase_diff -= 2.0 * M_PI;
    while (phase_diff < -M_PI) phase_diff += 2.0 * M_PI;
    const double doppler_estimate = phase_diff / (2.0 * M_PI * time_diff_s);
    if (sched_ctrl->doppler_shift == 0.0) {
      sched_ctrl->doppler_shift = doppler_estimate;
      sched_ctrl->doppler_shift_filtered = doppler_estimate;
    } else {
      sched_ctrl->doppler_shift = 0.7 * sched_ctrl->doppler_shift + 0.3 * doppler_estimate;
      sched_ctrl->doppler_shift_filtered = 0.8 * sched_ctrl->doppler_shift_filtered + 0.2 * sched_ctrl->doppler_shift;
    }
    LOG_I(NR_MAC, "UE %04x Doppler estimate: raw %.2f Hz, filtered %.2f Hz at %d.%d (phase %.3f rad)\n",
          UE->rnti, sched_ctrl->doppler_shift, sched_ctrl->doppler_shift_filtered, frame, slot, current_phase);
  }
  sched_ctrl->last_srs_frame = frame;
  sched_ctrl->last_srs_slot = slot;
  sched_ctrl->last_srs_phase = current_phase;
}
```

### Walkthrough

- **Inputs**: `UE` (per-UE MAC struct), `frame`/`slot` (time of current SRS),
  `current_phase` (average SRS channel phase in radians, computed upstream from
  normalized SRS IQ)
- **Elapsed time Δt**: `(frame - last_srs_frame) × 0.01` (10 ms/frame) +
  `(slot - last_srs_slot) × 0.0005` (0.5 ms/slot, consistent with the numerology
  used here)
- **Phase difference Δφ**: `current_phase - last_srs_phase`, normalized to
  `[−π, π]` since angles wrap every 2π — this yields the minimal signed rotation
  between successive phases
- **Instantaneous Doppler**: `doppler_estimate = Δφ / (2π·Δt)` — Δφ/Δt is angular
  velocity (rad/s), dividing by 2π converts to Hz
- **Two-stage smoothing (EMA)**:
  - First-level (less aggressive): `doppler_shift = 0.7×old + 0.3×new`
  - Second-level (more stable): `doppler_shift_filtered = 0.8×old_filtered + 0.2×doppler_shift`
  - Reduces jitter/outliers so PHY compensation stays stable; coefficients are tunable
- **Sign convention**: positive = forward phase rotation over time. UL compensation
  uses `−fd` (de-rotate received signal); DL pre-compensation uses `+fd`.
- History (`last_srs_frame`/`last_srs_slot`/`last_srs_phase`) is updated for the next
  Δφ/Δt computation.

## Feeding the estimator — averaging SRS channel IQ

In the SRS "codebook" usage handler (`case NR_SRS_ResourceSet__usage_codebook`),
the MAC receives a full normalized channel IQ matrix. This is averaged across all
UE SRS ports × gNB antenna elements × PRGs to get a single representative phase per
SRS report:

```c
double sum_re = 0.0, sum_im = 0.0;
const int nports = nr_srs_channel_iq_matrix.num_ue_srs_ports;
const int nants = nr_srs_channel_iq_matrix.num_gnb_antenna_elements;
const int nprgs = nr_srs_channel_iq_matrix.num_prgs;
const int total = nports * nants * nprgs;
if (total > 0) {
  if (nr_srs_channel_iq_matrix.normalized_iq_representation == 0) {
    c8_t *m = (c8_t *)nr_srs_channel_iq_matrix.channel_matrix;
    for (int i = 0; i < total; i++) { sum_re += (double)m[i].r; sum_im += (double)m[i].i; }
  } else {
    c16_t *m = (c16_t *)nr_srs_channel_iq_matrix.channel_matrix;
    for (int i = 0; i < total; i++) { sum_re += (double)m[i].r; sum_im += (double)m[i].i; }
  }
  const double avg_re = sum_re / (double)total;
  const double avg_im = sum_im / (double)total;
  const double avg_phase = atan2(avg_im, avg_re);
  estimate_doppler_from_srs_phase(UE, frame, slot, avg_phase);
}
```
Handles both normalized IQ formats (int8 `c8_t` when `normalized_iq_representation
== 0`, else int16 `c16_t`). `atan2(avg_im, avg_re)` yields a single channel phase
value for the whole matrix — a minimal estimator that could be refined further (e.g.
per-antenna or per-PRG estimates) if needed.

## Per-UE state — `nr_mac_gNB.h`

Added to `NR_UE_sched_ctrl_t`:
```c
/* Doppler compensation fields */
double doppler_shift;            /* Estimated Doppler shift in Hz */
double doppler_shift_filtered;   /* Filtered estimate for stability */
double last_srs_phase;           /* Last average SRS channel phase (rad) */
int last_srs_frame;              /* Last frame when SRS was received */
int last_srs_slot;               /* Last slot when SRS was received */
} NR_UE_sched_ctrl_t;
```
- `doppler_shift` — most recent raw estimate (Hz), before smoothing
- `doppler_shift_filtered` — EMA-smoothed value; **this is what RU compensation
  actually uses**, since it's less jittery
- `last_srs_phase`/`last_srs_frame`/`last_srs_slot` — needed to compute Δφ/Δt for
  the next SRS report

## The minimal MAC→PHY API — `nr_doppler_api.h`

The RU (PHY side) runs in its own code path and deliberately avoids including the
heavy MAC header (to avoid dragging in ASN.1 include cascades). A single lightweight
function is exposed instead:

```c
#ifndef NR_DOPPLER_API_H
#define NR_DOPPLER_API_H
#include <stdint.h>
#include "common/platform_types.h"

/* Return filtered Doppler [Hz] for UE RNTI if available.
 * has_value is set to 1 if a valid value is available, 0 otherwise. */
double nr_mac_get_doppler_hz(module_id_t mod_id, uint16_t rnti, int *has_value);
#endif
```

**Implementation** (in `gNB_scheduler_ulsch.c`):
```c
double nr_mac_get_doppler_hz(module_id_t mod_id, uint16_t rnti, int *has_value)
{
  if (has_value) *has_value = 0;
  gNB_MAC_INST *mac = RC.nrmac[mod_id];
  if (!mac) return 0.0;
  NR_UEs_t *UEs = &mac->UE_info;
  if (!UEs) return 0.0;
  for (NR_UE_info_t **it = UEs->connected_ue_list; it && *it; ++it) {
    NR_UE_info_t *ue = *it;
    if (!ue) continue;
    if (ue->rnti == rnti) {
      if (has_value) *has_value = 1;
      return ue->UE_sched_ctrl.doppler_shift_filtered;
    }
  }
  return 0.0;
}
```
Scans `connected_ue_list` for a matching RNTI; if found, sets `*has_value = 1` and
returns that UE's `doppler_shift_filtered`. Otherwise returns 0.0 Hz with
`has_value` left at 0 (no MAC instance, no UE list, or no match).

This lets the PHY/RU layer ask the MAC for the current filtered Doppler of a
specific UE without coupling the RU build to full MAC internals.
