# 3. Results

## Sample gNB log capture

```
[NR_MAC] Frame.Slot 384.0
UE RNTI 83f3 CU-UE-ID 1 in-sync PH 51 dB PCMAX 21 dBm, average RSRP -105 (16 meas)
UE 83f3: UL-RI 1, TPMI 0
UE 83f3: dlsch_rounds 37543/2349/280/146, dlsch_errors 141, pucch0_DTX 679, BLER 0.09597 MCS (1) 19 CCE fail 20
UE 83f3: ulsch_rounds 12105/744/53/24, ulsch_errors 18, ulsch_DTX 156, BLER 0.11375 MCS (1) 11 (Qm 6 deltaMCS 0 dB) NPRB 5 RBSTART 0 SNR 14.5 dB CCE fail 0
UE 83f3: MAC: TX 237391768 RX 6748296 bytes
UE 83f3: LCID 4: TX 236113323 RX 4257110 bytes
[NR_MAC] UE 83f3 Doppler estimate: raw 0.35 Hz, filtered -1.28 Hz at 384.8 (phase 1.552 rad)
[NR_MAC] UE 83f3 Doppler estimate: raw 1.48 Hz, filtered -0.73 Hz at 392.8 (phase -2.669 rad)
[NR_MAC] UE 83f3 Doppler estimate: raw -0.37 Hz, filtered -0.65 Hz at 400.8 (phase 1.264 rad)
[NR_MAC] UE 83f3 Doppler estimate: raw 1.12 Hz, filtered -0.30 Hz at 408.8 (phase -2.708 rad)
[NR_MAC] UE 83f3 Doppler estimate: raw 0.01 Hz, filtered -0.24 Hz at 416.8 (phase 2.269 rad)
[NR_MAC] UE 83f3 Doppler estimate: raw -0.66 Hz, filtered -0.32 Hz at 424.8 (phase 1.152 rad)
[NR_MAC] UE 83f3 Doppler estimate: raw -1.22 Hz, filtered -0.50 Hz at 432.8 (phase -0.109 rad)
[NR_MAC] UE 83f3 Doppler estimate: raw -1.12 Hz, filtered -0.63 Hz at 440.8 (phase -0.567 rad)
[NR_MAC] UE 83f3 Doppler estimate: raw -0.42 Hz, filtered -0.59 Hz at 448.8 (phase 0.051 rad)
[NR_MAC] UE 83f3 Doppler estimate: raw -0.83 Hz, filtered -0.64 Hz at 456.8 (phase -0.856 rad)
[NR_MAC] UE 83f3 Doppler estimate: raw -1.98 Hz, filtered -0.90 Hz at 464.8 (phase 3.095 rad)
[NR_MAC] UE 83f3 Doppler estimate: raw -2.34 Hz, filtered -1.19 Hz at 472.8 (phase 1.494 rad)
[NR_MAC] UE 83f3 Doppler estimate: raw -0.69 Hz, filtered -1.09 Hz at 480.8 (phase 3.074 rad)
[NR_MAC] UE 83f3 Doppler estimate: raw 1.16 Hz, filtered -0.64 Hz at 488.8 (phase -0.452 rad)
[NR_MAC] UE 83f3 Doppler estimate: raw 1.27 Hz, filtered -0.26 Hz at 496.8 (phase 0.309 rad)
[NR_MAC] UE 83f3 Doppler estimate: raw 0.74 Hz, filtered -0.06 Hz at 504.8 (phase 0.060 rad)

[NR_MAC] Frame.Slot 512.0
UE RNTI 83f3 CU-UE-ID 1 in-sync PH 52 dB PCMAX 21 dBm, average RSRP -105 (16 meas)
UE 83f3: ulsch_rounds 12329/757/53/24, ulsch_errors 18, ulsch_DTX 156, BLER 0.07165 MCS (1) 10 (Qm 4 deltaMCS 0 dB) NPRB 29 RBSTART 0 SNR 16.0 dB CCE fail 0
```

## Interpretation

The Doppler value in the logs is computed from each SRS report by comparing the
current SRS channel phase with the previous one — it isn't hard-coded or injected;
it reflects whatever the air interface is actually doing (UE velocity, oscillator
offsets, etc.), smoothed for stability.

- **Raw**: the instantaneous Doppler estimate from the most recent SRS phase
  change over elapsed time — `fd_raw = Δφ / (2π·Δt)`. Reflects the latest
  measurement and can be noisy.
- **Filtered**: a smoothed version via two-stage EMA (0.7/0.3, then 0.8/0.2).
  Reduces jitter and is the value actually used to trigger and drive compensation.

As the log sequence shows, raw estimates swing between roughly **−3 Hz and +2 Hz**
(noise + true velocity changes), while the filtered output smooths those swings
into gradual updates between roughly **−1.4 Hz and +0.1 Hz**. That filtered value
is what the RU compensates for whenever `|filtered|` exceeds the 0.1 Hz threshold.

### Damping behavior

```
doppler_shift          = 0.7 × old_raw      + 0.3 × new_raw
doppler_shift_filtered = 0.8 × old_filtered + 0.2 × doppler_shift
```
If the previous raw/filtered values were around −1 Hz and a new raw measurement
hits −3.4 Hz, the filtered value steps only partway (≈−1.4 Hz) instead of instantly
matching −3.4 Hz. After further SRS reports with a similar offset, the filtered
number continues moving toward −3.4 Hz. This damping is deliberate — it avoids
overreacting to noisy/outlier SRS measurements. To make the compensator chase the
raw estimate more closely, the EMA weights can be increased (e.g. 0.5/0.5) or
smoothing disabled altogether.

## Summary

- The MAC-side estimator produces both raw and filtered per-UE Doppler from SRS
  phase differences, updated every SRS report
- The filtered (EMA-smoothed) value is exposed to PHY/RU via a minimal API
  (`nr_mac_get_doppler_hz()`) that avoids coupling the RU build to full MAC headers
- UL compensation (frequency-domain, post-FFT) and DL pre-compensation
  (time-domain, post-OFDM-modulation) both engage only when `|filtered Doppler| >
  0.1 Hz`, using opposite-sign rotation to cancel the channel-induced shift
