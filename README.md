# artifacts_cvsa

This directory contains the artifact detection node. This node is responsible for the real-time identification of physiological artifacts (EOG and signal peaks) in the raw EEG signal.

---

### 1. Input

* **Topic:** `/neurodata`
* **Message Type:** `rosneuro_msgs/NeuroFrame`
* **Data:** The node subscribes to this topic, expecting to receive raw EEG signal samples in real-time.

---

### 2. Output

* **Topic:** `/artifact_presence`
* **Message Type:** Custom (defined in this package)
* **Data:** The node publishes a custom message containing two fields:
    * `has_artifact` (bool): `true` if an artifact (EOG or peak) is detected, `false` otherwise.
    * `seq` (uint32): The sequence number (sample index) of the analyzed sample, used to synchronize this output with other processing nodes (e.g., `cvsa_processing`).

---

### 3. Configuration

This node **requires a YAML configuration file** that defines all the filters, channels, and thresholds necessary for detection.

The YAML file must contain the following fields:

* `nchannels`: Total number of channels acquired (e.g., 32 or 39).
* `chunkSize`: The size of the sample chunks coming from the acquisition (e.g., usually 25 for framerate=20, sample_rate=500).
* `run_mode`: Define if the protocol is running `online` or `offline`. This helps map the channels properly.
* `signal_type`: Can be `eeg` or `eeg_eog`. Used in conjunction with `run_mode` to fetch EOG channels correctly if they are mapped in the `exg` blocks (e.g., LSL playback vs live acquisition).
* `sampleRate`: The sampling frequency of the input data (e.g., 500 Hz).
* `th_hEOG`: Threshold (in µV) for horizontal eye movement.
* `th_vEOG`: Threshold (in µV) for vertical eye movement.
* `th_peaks`: Threshold (in µV) for signal peaks (e.g., muscle artifacts or saturation).
* `EOG_ch`: A list specifying the 1-based indices of the EOG-associated channels (e.g., `[12, 16]` or `[1, 2, 19]`). Internally they are mapped to 0-based indexing.
* `freq_high_EOG` / `freq_low_EOG`: Cutoff frequencies (in Hz) for the EOG band-pass filter (e.g., 1-10 Hz).
* `freq_high_peaks`: Cutoff frequency (in Hz) for the peaks high-pass filter (e.g., 2 Hz).
* `filterOrder_EOG` / `filterOrder_peaks`: The order of the respective IIR (Butterworth) filters.

#### Example `artifact.yaml`

```yaml
ArtifactCfg:
  name: artifact
  params: 
    nchannels: 32
    chunkSize: 25
    run_mode: online
    signal_type: eeg
    sampleRate: 500
    th_hEOG: 100
    th_vEOG: 100
    th_peaks: 140
    EOG_ch: [12, 16]
    freq_high_EOG: 1
    freq_low_EOG: 10
    freq_high_peaks: 1
    filterOrder_EOG: 4
    filterOrder_peaks: 4
```

---

### 4. Workflow

1.  **Format Input Data:** The node receives data from `rosneuro_msgs` taking `run_mode` and `signal_type` into account to dynamically reconstruct the matrix (mapping standard `eeg` data array or hybrid `exg` placements used in LSL standard recordings).
2.  **CAR Filter:** Applies a **CAR (Common Average Reference)** spatial filter across the channels, strictly excluding the designated EOG channels (provided in `EOG_ch`) to prevent eye movement artifacts from polluting the EEG channels prior to detection.
3.  **Buffer Filtering:** The filtered sequences run through specific IIR filters (High-pass for peaks, Band-pass for EOG) and are fed sequentially into fixed RingBuffers (usually 1 full second of samples based on the sampling rate).
4.  **EOG Monitoring:**
    * If EOG channels are correctly specified, horizontal movement is calculated ($hEOG = \text{col}_0 - \text{col}_1$).
    * Vertical movement applies an average combination against a third baseline (if available).
    * If `abs(hEOG) > th_hEOG` OR `abs(vEOG) > th_vEOG`, an EOG artifact flag is triggered.
5.  **Peak Monitoring:**
    * The node evaluates the absolute values of the remaining pure EEG channels in the peak buffer (EOG channels are deliberately skipped).
    * If *any* channel exceeds the target threshold (`abs(sample) > th_peaks`), the peak-type artifact flag is triggered.
6.  **Publish:**
    * If the EOG check OR the Peak check is positive, the node sets `has_artifact = true`.
    * Otherwise, it sets `has_artifact = false`.
    * The node sequentially publishes the message on `/artifact_presence`, including the current chunk's `seq` number for exact signal alignment.

---

### 5. Dependencies

This package requires several libraries for compilation and execution:

* **rosneuro_msgs:** Required for the `NeuroFrame` input message.
* **rosneuro_filters:** Used for the band-pass and high-pass filter implementations.
* **Eigen:** Required for linear algebra operations and vector management.
* **rtf (Ring-Time-Framework):** Used for the efficient real-time ring buffer implementation.
* **yaml-cpp:** Required for parsing the `.yaml` configuration file.