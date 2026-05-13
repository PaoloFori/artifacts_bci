# artifacts_bci

Real-time artifact detection node for the BCI-VR pipeline. Identifies physiological artifacts (EOG eye movements and signal peaks) in the raw EEG signal and gates the classifier pipeline accordingly.

---

## 1. Input / Output

| Direction | Topic | Message type |
|-----------|-------|-------------|
| Input  | `/neurodata` | `rosneuro_msgs/NeuroFrame` |
| Output | `/artifact_presence` | `artifacts_bci/artifact_presence` |

The output message contains:
- `has_artifact` (bool) — `true` if any artifact (EOG or peak) is detected.
- `seq` (uint32) — sequence number copied from `NeuroFrame.neuroheader.seq`, used by the integrator for synchronisation.

---

## 2. Auto-configuration from NeuroFrame

`nchannels`, `chunkSize`, and `sampleRate` are **no longer set in the YAML**. They are extracted automatically from the first `NeuroFrame` received:

| NeuroFrame field | Used as |
|-----------------|---------|
| `eeg.info.nchannels` | `nchannels` |
| `eeg.info.nsamples`  | `chunkSize` |
| `sr`                 | `sampleRate` |

EOG channel indices are also resolved at this point by matching `EOG_ch_names` against `eeg.info.labels` (case-insensitive). If any name is not found the node logs an error and stops.

---

## 3. Configuration (YAML)

### `artifact.yaml`

```yaml
ArtifactCfg:
  name: artifact
  params:
    run_mode: online          # 'online' | 'offline'
    signal_type: eeg          # 'eeg'   | 'eeg_eog'
    th_hEOG: 75               # µV – horizontal EOG threshold
    th_vEOG: 75               # µV – vertical EOG threshold
    th_peaks: 150             # µV – peak amplitude threshold
    EOG_ch_names: ['Fp1', 'Fp2']  # channel names (resolved from NeuroFrame labels)
    freq_low_EOG: 10          # Hz – low-pass cutoff of EOG bandpass
    freq_high_EOG: 1          # Hz – high-pass cutoff of EOG bandpass
    freq_high_peaks: 1        # Hz – high-pass cutoff for peak detection
    filterOrder_EOG: 4
    filterOrder_peaks: 4
    # nchannels, chunkSize, sampleRate: derived automatically from NeuroFrame
```

### `ringbuffer.yaml`

```yaml
RingBufferCfg:
  name: ringbuffer_artifact
  type: RingBufferFloat
  params:
    size: 250   # 0.5 s at 500 Hz  (use artifact_ringbuffer.yaml in evaluation.launch)
```

> **Threshold guidance:**  
> - `th_hEOG` / `th_vEOG`: 75–100 µV. Blinks produce ~150–300 µV after the 1–10 Hz bandpass; saccades ~50–100 µV.  
> - `th_peaks`: 120–150 µV. Calibrate empirically as ~2–3× the typical peak value during a clean resting session.

---

## 4. Processing pipeline

```
NeuroFrame.eeg  [channels × chunkSize]
    │
    ▼ configure_signal()  ← called once on first message
    │   resolve EOG_ch_names → indices from eeg.info.labels
    │   init Butterworth filters + ring buffers
    │
    ▼ CAR  (mean of non-EOG channels subtracted from all channels)
    │
    ├─ EOG path:  LP(10 Hz) → HP(1 Hz)  →  ring buffer (0.5 s)
    │              max|hEOG| > th_hEOG  OR  max|vEOG| > th_vEOG  → artifact
    │
    └─ Peaks path: HP(1 Hz)  →  ring buffer (0.5 s)
                   max|EEG channels| > th_peaks  → artifact

    → publish /artifact_presence  {has_artifact, seq}
```

**hEOG / vEOG formulas (2 or 3 EOG channels):**

| Channels | hEOG | vEOG |
|---------|------|------|
| 2 (`[Fp1, Fp2]`) | `Fp1 − Fp2` | `(Fp1 + Fp2) / 2` |
| 3 (`[Fp1, Fp2, EOG]`) | `Fp1 − Fp2` | `(Fp1 + Fp2) / 2 − EOG` |

---

## 5. Launch files

### Production

```bash
# Included in evaluation.launch via:
#   <rosparam command="load" file=".../artifact_ringbuffer.yaml"/>
#   <rosparam command="load" file=".../artifact.yaml" subst_value="true"/>
#   <node name="artifactDetector_node" .../>
roslaunch launchers_bci evaluation.launch paradigm:=hybrid
```

### Standalone example

```bash
roslaunch artifacts_bci example_node_artifact.launch
```

---

## 6. Testing

### 6a. CSV-based test (quick sanity check)

Uses `rawdata.csv` published chunk by chunk via the test publisher.

```bash
roslaunch artifacts_bci test_node_artifact.launch
```

Produces `test/artifacts.csv`. Compare with MATLAB:

```matlab
input_mode = 'csv';
test_artifacts   % in MATLAB, from workspace root
```

### 6b. GDF-based test (realistic end-to-end validation)

Replays `test/prova32ch.gdf` (512 Hz, ~33 channels, ~382 s) through the real `rosneuro_acquisition` node using the eegdev `datafile` plugin. This exercises:
- Automatic `nchannels` / `chunkSize` / `sampleRate` discovery from the NeuroFrame
- Channel name resolution from the GDF labels
- Long-session dynamic logger (no fixed-size limit)

```bash
roslaunch artifacts_bci test_node_artifact_gdf.launch \
    gdf_file:=$(rospack find artifacts_bci)/test/prova32ch.gdf \
    samplerate:=512 \
    framerate:=16
# Wait for the file to finish, then Ctrl+C.
```

Produces in `test/`:
- `artifacts_gdf_output.csv` — artifact flags indexed by seq
- `artifacts_gdf_output_first_seq.txt` — first seq received (lost frames at startup)

Compare with MATLAB:

```matlab
input_mode = 'gdf';
test_artifacts   % in MATLAB, from workspace root
```

### Alignment details

Two independent timing offsets must be corrected before comparing MATLAB and ROS:

**1. `first_seq` — startup frame loss**

`rosneuro_acquisition` takes a few milliseconds to initialise before it starts receiving data. The artifact detector therefore misses the first `first_seq` frames (typically 1–2). The logger records the first seq it receives in `*_first_seq.txt`.

MATLAB **starts its processing loop at `seq = first_seq`** (not 0) so that both pipelines have identical zero-state IIR filters from the same starting sample. If MATLAB started from seq 0, its filter history would be `first_seq` frames longer than ROS's, making MATLAB consistently detect artifacts earlier.

**2. Acquisition pipeline delay (GDF only)**

When using `rosneuro_acquisition` with the eegdev `datafile` plugin, the acquisition node internally buffers one frame before publishing. As a result, ROS frame N carries samples that MATLAB would assign to frame N−1 — a systematic 1-frame delay in the ROS pipeline relative to direct file reading.

The script measures this with a cross-correlation (`xcorr`) and corrects automatically:

- `xcorr(ros, matlab)` peak at lag +k → MATLAB is k frames ahead of ROS
- Correction: compare `ROS[k+1..N]` with `MATLAB[1..N-k]`

Two figures are produced:
- **RAW** — unaligned comparison (shows the lag visually)
- **ALIGNED** — lag-corrected comparison (state changes overlap)

The console prints mismatches for both cases. With the CSV publisher (no acquisition pipeline) `measured_lag = 0` and both figures are identical.

### Logger details

The logger node (`test/logger.cpp`) dynamically grows its output vector as seq numbers arrive — no fixed upper limit. At shutdown it writes the full seq-indexed vector to CSV and saves `first_seq` in a companion `.txt` file.

---

## 7. Dependencies

| Library | Used for |
|---------|---------|
| `rosneuro_msgs` | `NeuroFrame` input message |
| `rosneuro_filters_butterworth` | IIR Butterworth LP/HP filters |
| `rosneuro_filters_car` | Common Average Reference spatial filter |
| `rosneuro_buffers_ringbuffer` | Shift-register ring buffer (NaN-initialised, `isfull()` ↔ no NaN) |
| `Eigen` | Linear algebra, matrix operations |
| `yaml-cpp` | YAML parameter loading (via ROS param server) |
