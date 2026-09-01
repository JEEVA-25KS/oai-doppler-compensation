/*
 * Minimal MAC API for Doppler access from PHY
 */
#ifndef NR_DOPPLER_API_H
#define NR_DOPPLER_API_H

#include <stdint.h>
#include "common/platform_types.h"

/* Return filtered Doppler [Hz] for UE RNTI if available.
 * has_value is set to 1 if a valid value is available, 0 otherwise.
 */
double nr_mac_get_doppler_hz(module_id_t mod_id, uint16_t rnti, int *has_value);

#endif


