# Data

## Log_File.csv

Field measurements collected on the ESP32 + SX1278 hardware testbed
described in Section 4.1 of the accompanying paper. Data is real —
recorded from live transmitter/receiver operation, not simulated —
with column names and formatting normalized for readability.

### Columns

| Column          | Description                                            |
|-----------------|----------------------------------------------------------|
| Sequence        | Packet sequence number                                   |
| Timestamp_ms    | Time since test start (ms)                                |
| Distance_m      | TX–RX separation, from onboard GPS                        |
| SNR_dB          | Raw per-packet SNR (`packetSnr()`)                         |
| SNR_EMA_dB      | EMA-smoothed SNR (α = 0.3, Eq. 6)                          |
| PacketLost      | 1 if this sequence number was inferred lost, else 0        |
| PerRadio_EMA    | EMA-smoothed radio-level PER                               |
| AuthFail        | 1 if GCM authentication failed on this packet, else 0      |
| PerAuth_EMA     | EMA-smoothed authentication-failure rate                   |
| Battery         | Normalized battery level [0,1] (Eq. 9)                     |
| SelectedMode    | AES mode chosen by the controller for this packet          |
| ModeSwitched    | 1 if a mode change occurred this cycle, else 0              |
| Cost_CTR/CBC/GCM| Computed cost for each mode at this decision point (Eq. 10) |
| Latency_ms      | Measured end-to-end latency for this packet                |
| Latitude/Longitude | GPS position at time of transmission                    |

### Notes

- No values are synthesized or interpolated; all fields are direct
  measurements or deterministic derivations (EMA, cost function) computed
  from measurements at collection time.
- Column names/units were normalized post-collection for clarity
  (e.g. consistent `_dB`/`_ms`/`_EMA` suffixes); this did not alter
  any underlying values.
