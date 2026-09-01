/*
 * Doppler compensation helpers for NR
 */
#ifndef NR_DOPPLER_COMPENSATION_H
#define NR_DOPPLER_COMPENSATION_H

#include "PHY/defs_gNB.h"
#include "PHY/defs_nr_common.h"

/* Apply UL Doppler compensation in frequency domain (after FFT) */
void apply_ul_doppler_compensation_freq(c16_t *rxdataF,
                                        int ofdm_symbol_size,
                                        int start_symbol,
                                        int end_symbol,
                                        double doppler_shift,
                                        double subcarrier_spacing);

/* Apply DL pre-Doppler compensation in time domain (after IFFT, before TX) */
void apply_pre_doppler_compensation_time(int *txdata,
                                         int ofdm_symbol_size,
                                         int num_symbols,
                                         int nb_prefix_samples,
                                         double doppler_shift,
                                         double sampling_rate);

#endif /* NR_DOPPLER_COMPENSATION_H */


