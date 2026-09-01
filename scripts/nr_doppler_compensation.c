/*
 * Doppler compensation helpers for NR
 */
#include "nr_doppler_compensation.h"
#include <math.h>
#include <string.h>

void apply_ul_doppler_compensation_freq(c16_t *rxdataF,
                                        int ofdm_symbol_size,
                                        int start_symbol,
                                        int end_symbol,
                                        double doppler_shift,
                                        double subcarrier_spacing)
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
        const double cos_p = cos(phase);
        const double sin_p = sin(phase);
        const int16_t re = rxdataF[sym_offset + k].r;
        const int16_t im = rxdataF[sym_offset + k].i;
        rxdataF[sym_offset + k].r = (int16_t)(re * cos_p - im * sin_p);
        rxdataF[sym_offset + k].i = (int16_t)(re * sin_p + im * cos_p);
      }
    }
  }
}

void apply_pre_doppler_compensation_time(int *txdata,
                                         int ofdm_symbol_size,
                                         int num_symbols,
                                         int nb_prefix_samples,
                                         double doppler_shift,
                                         double sampling_rate)
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
      const double cos_p = cos(phase);
      const double sin_p = sin(phase);
      ((int16_t*)txdata)[idx]     = (int16_t)(re * cos_p - im * sin_p);
      ((int16_t*)txdata)[idx + 1] = (int16_t)(re * sin_p + im * cos_p);
      phase += phase_increment;
      if (phase > 2.0 * M_PI) phase -= 2.0 * M_PI;
    }
  }
}


