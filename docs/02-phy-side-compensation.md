# 2. PHY-Side Compensation

## RU helper — finding the active UE for a slot/beam

`nr_doppler_get_for_slot()` (`nr_ru_procedures.c`) scans the gNB's UL scheduling
table to find the UE active in a given frame/slot/beam, then queries its filtered
Doppler:
```c
static double nr_doppler_get_for_slot(PHY_VARS_gNB *gNB, int beam, int frame, int slot, uint16_t *out_rnti)
{
  if (out_rnti) *out_rnti = 0;
  if (!gNB || !gNB->ulsch) return 0.0;
  for (int i = 0; i < gNB->max_nb_pusch; i++) {
    NR_gNB_ULSCH_t *ulsch = &gNB->ulsch[i];
    if (!ulsch->active) continue;
    if ((int)ulsch->frame != frame || (int)ulsch->slot != slot) continue;
    if ((beam >= 0) && (ulsch->beam_nb >= 0) && (ulsch->beam_nb != beam)) continue;
    int has_value = 0;
    double doppler = nr_mac_get_doppler_hz(gNB->Mod_id, ulsch->rnti, &has_value);
    if (has_value) {
      if (out_rnti) *out_rnti = ulsch->rnti;
      return doppler;
    }
  }
  return 0.0;
}
```
Iterates `gNB->ulsch[]` (all possible PUSCH scheduling entries), skips inactive
entries, filters by matching frame/slot (and beam, if specified), then calls
`nr_mac_get_doppler_hz()` for the matching UE's RNTI.

## DL: time-domain pre-compensation (post-OFDM-modulation)

Applied at the end of `nr_feptx0()`, right after OFDM modulation produces
time-domain TX samples:
```c
PHY_VARS_gNB *gNB = ru->gNB_list[0];
if (gNB) {
  uint16_t rnti = 0;
  double doppler = nr_doppler_get_for_slot(gNB, -1, ru->proc.frame_tx, slot, &rnti);
  if (fabs(doppler) > 0.1) {
    double sampling_rate = (double)fp->samples_per_subframe * 1000.0;
    apply_pre_doppler_compensation_time(&ru->common.txdata[aa][slot_offset],
                                         fp->ofdm_symbol_size, num_symbols,
                                         fp->nb_prefix_samples, doppler, sampling_rate);
    LOG_D(PHY, "DL Doppler pre-compensation: frame %d slot %d antenna %d UE %04x shift %.2f Hz\n",
          ru->proc.frame_tx, slot, aa, rnti, doppler);
  }
}
```
- `-1` beam filter means "any beam" for DL
- **0.1 Hz threshold** avoids needless rotation on near-zero/noisy estimates
- `sampling_rate` derived from `samples_per_subframe × 1000` (samples/ms → samples/s)
- Positive Doppler means pre-rotate samples so the airlink cancels the phase drift at the UE

## UL: frequency-domain compensation (post-FFT)

Applied in `nr_fep()`, right after the FFT on the uplink:
```c
PHY_VARS_gNB *gNB = ru->gNB_list[0];
if (gNB) {
  uint16_t rnti = 0;
  double doppler = nr_doppler_get_for_slot(gNB, beam, ru->proc.frame_rx, slot, &rnti);
  if (fabs(doppler) > 0.1) {
    double scs = fp->subcarrier_spacing ? (double)fp->subcarrier_spacing : (15000.0 * (1 << fp->numerology_index));
    c16_t *rxdataF = (c16_t *)&ru->common.rxdataF[idx][offset];
    apply_ul_doppler_compensation_freq(rxdataF, fp->ofdm_symbol_size, startSymbol, endSymbol, -doppler, scs);
    LOG_D(PHY, "UL Doppler compensation: frame %d slot %d beam %d UE %04x shift %.2f Hz\n",
          ru->proc.frame_rx, slot, beam, rnti, doppler);
  }
}
```
- `-doppler` passed so the rotation cancels the channel-induced shift
- `scs` (subcarrier spacing) falls back to `15000 × 2^numerology` if not explicitly set

## The compensation primitives — `nr_doppler_compensation.c`

### `apply_ul_doppler_compensation_freq()` — frequency domain, post-FFT

```c
void apply_ul_doppler_compensation_freq(c16_t *rxdataF, int ofdm_symbol_size,
                                          int start_symbol, int end_symbol,
                                          double doppler_shift, double subcarrier_spacing)
{
  if (!rxdataF) return;
  if (fabs(doppler_shift) < 0.1) return;
  if (ofdm_symbol_size <= 0) return;
  if (end_symbol < start_symbol) return;

  const double bin_shift = doppler_shift / subcarrier_spacing;
  const int bin_shift_int = (int)lrint(bin_shift);
  const double bin_shift_frac = bin_shift - bin_shift_int;

  for (int sym = start_symbol; sym <= end_symbol; sym++) {
    const int sym_offset = sym * ofdm_symbol_size;
    if (bin_shift_int != 0) {
      c16_t temp[ofdm_symbol_size];
      memcpy(temp, &rxdataF[sym_offset], (size_t)ofdm_symbol_size * sizeof(c16_t));
      for (int k = 0; k < ofdm_symbol_size; k++) {
        const int shifted_idx = (k - bin_shift_int + ofdm_symbol_size) % ofdm_symbol_size;
        rxdataF[sym_offset + k] = temp[shifted_idx];
      }
    }
    if (fabs(bin_shift_frac) > 1e-3) {
      for (int k = 0; k < ofdm_symbol_size; k++) {
        const double phase = -2.0 * M_PI * bin_shift_frac * (double)k / (double)ofdm_symbol_size;
        const double cos_p = cos(phase), sin_p = sin(phase);
        const int16_t re = rxdataF[sym_offset + k].r, im = rxdataF[sym_offset + k].i;
        rxdataF[sym_offset + k].r = (int16_t)(re * cos_p - im * sin_p);
        rxdataF[sym_offset + k].i = (int16_t)(re * sin_p + im * cos_p);
      }
    }
  }
}
```
- Splits the Doppler shift into **integer bin shift** (`bin_shift_int`) and
  **fractional shift** (`bin_shift_frac`) relative to subcarrier spacing
- Integer part: circular shift of the whole spectrum by that many subcarrier bins
  (`dest[k] = temp[(k - bin_shift_int) mod N]`)
- Fractional part: per-subcarrier phase rotation (`phase = -2π × frac × k / N`)
- Both cancel the carrier offset in the FFT output; the `-fd` sign was applied by
  the caller so this de-rotates the channel-induced rotation

### `apply_pre_doppler_compensation_time()` — time domain, post-OFDM-mod

```c
void apply_pre_doppler_compensation_time(int *txdata, int ofdm_symbol_size,
                                           int num_symbols, int nb_prefix_samples,
                                           double doppler_shift, double sampling_rate)
{
  if (!txdata) return;
  if (fabs(doppler_shift) < 0.1) return;
  if (ofdm_symbol_size <= 0 || num_symbols <= 0) return;
  if (sampling_rate <= 0.0) return;

  const double phase_increment = 2.0 * M_PI * doppler_shift / sampling_rate;
  double phase = 0.0;
  for (int sym = 0; sym < num_symbols; sym++) {
    const int symbol_start = sym * (ofdm_symbol_size + nb_prefix_samples);
    const int symbol_samples = ofdm_symbol_size + nb_prefix_samples;
    for (int i = 0; i < symbol_samples; i++) {
      const int idx = (symbol_start + i) * 2;
      int16_t re = ((int16_t*)txdata)[idx];
      int16_t im = ((int16_t*)txdata)[idx + 1];
      const double cos_p = cos(phase), sin_p = sin(phase);
      ((int16_t*)txdata)[idx] = (int16_t)(re * cos_p - im * sin_p);
      ((int16_t*)txdata)[idx + 1] = (int16_t)(re * sin_p + im * cos_p);
      phase += phase_increment;
      if (phase > 2.0 * M_PI) phase -= 2.0 * M_PI;
    }
  }
}
```
- Computes a constant `phase_increment = 2π·fd/fs` (radians per sample)
- Walks every I/Q sample across all symbols (including the cyclic prefix), rotating
  by an accumulating phase and wrapping at 2π to stay bounded
- Positive Doppler pre-rotates the samples so the airlink introduces the opposite
  rotation, cancelling Doppler at the receiver

## Why frequency domain for UL, time domain for DL

- **UL**: after FFT you already have per-subcarrier bins, so integer + fractional
  bin-shift correction is natural and efficient there
- **DL**: time-domain rotation is simple and generic across all transmitted signals
  right after OFDM modulation — no need to go back into the frequency domain

Both directions early-exit for `|fd| < 0.1 Hz` to avoid reacting to jittery
micro-rotations from noise.

## Build integration
Added to `CMakeLists.txt`'s NR PHY source list:
```cmake
${OPENAIR1_DIR}/PHY/TOOLS/nr_doppler_compensation.c
```
