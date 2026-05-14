%% ArtifactDetector MATLAB simulation
% Replicates the ROS ArtifactDetector node chunk by chunk with causal IIR
% filters and a ring buffer, mirroring configure_signal() + apply().
%
% KEY alignment principle:
%   ROS starts processing at seq = first_seq (first frame actually received).
%   MATLAB must also start at first_seq – same zero initial filter state –
%   otherwise the IIR filter history diverges and MATLAB detects artifacts
%   systematically earlier than ROS.
%
% Workflow:
%   1. roslaunch artifacts_bci test_node_artifact_gdf.launch
%   2. Wait for the file to finish, Ctrl+C → artifacts_gdf_output.csv +
%      artifacts_gdf_output_first_seq.txt are written.
%   3. Run this script.

clear all; clc; close all;

%% --- input mode ---
% 'gdf'  → reads prova32ch.gdf  (same file used in ROS test)
% 'csv'  → reads rawdata.csv    (legacy CSV test, publisher sends seq from 0)
input_mode = 'gdf';

%% --- paths ---
data_dir        = './test_node_data/';
out_dir         = './test_node_data/artifacts_bci/';
artifact_yaml   = './src/artifacts_bci/cfg/artifact.yaml';
ringbuffer_yaml = './src/artifacts_bci/cfg/ringbuffer.yaml';

if strcmp(input_mode, 'gdf')
    input_file = [data_dir 'prova32ch.gdf'];
    ros_file   = [out_dir  'artifacts_gdf_output.csv'];
    framerate  = 16;   % must match test_node_artifact_gdf.launch  (512/16 = 32 samples/chunk)
else
    input_file = [data_dir 'raw_eeg_32ch.csv'];
    ros_file   = [out_dir  'artifacts.csv'];
    framerate  = 20;
end

%% --- read first_seq BEFORE the processing loop ---
% ROS starts at this seq because it lost the earlier frames at startup.
% MATLAB must start at the same seq to have identical filter initial conditions.
first_seq_file = strrep(ros_file, '.csv', '_first_seq.txt');
first_seq = 0;
if isfile(first_seq_file)
    first_seq = readmatrix(first_seq_file);
    fprintf('ROS first_seq = %d (lost %d frame(s) at startup)\n', first_seq, first_seq);
else
    fprintf('first_seq file not found – assuming first_seq = 0.\n');
end

%% --- load YAML configs ---
art_cfg    = yaml.ReadYaml(artifact_yaml);
p          = art_cfg.ArtifactCfg.params;
rb_cfg     = yaml.ReadYaml(ringbuffer_yaml);
bufferSize = rb_cfg.RingBufferCfg.params.size;

th_hEOG         = p.th_hEOG;
th_vEOG         = p.th_vEOG;
th_peaks        = p.th_peaks;
eog_ch_names    = p.EOG_ch_names;
filterOrder_EOG   = p.filterOrder_EOG;
filterOrder_peaks = p.filterOrder_peaks;
freq_low_EOG    = p.freq_low_EOG;
freq_high_EOG   = p.freq_high_EOG;
freq_high_peaks = p.freq_high_peaks;

fprintf('--- Artifact config from YAML ---\n');
fprintf('  bufferSize   : %d samples\n', bufferSize);
fprintf('  th_hEOG/vEOG : %g / %g uV\n', th_hEOG, th_vEOG);
fprintf('  th_peaks     : %g uV\n', th_peaks);
fprintf('  EOG_ch_names : [%s]\n', strjoin(eog_ch_names, ', '));

%% --- load data ---
[~, ~, ext] = fileparts(input_file);
if strcmpi(ext, '.gdf')
    [data_raw, hdr] = sload(input_file);   % BIOSIG required
    sampleRate = hdr.SampleRate;
    ch_names   = cellstr(hdr.Label);
    n_eeg = sum(~cellfun(@(c) contains(lower(c), {'status','trigger','mkr'}), ch_names));
    data     = data_raw(:, 1:n_eeg);
    ch_names = ch_names(1:n_eeg);
    fprintf('GDF: %d samples x %d EEG channels @ %.0f Hz\n', size(data,1), n_eeg, sampleRate);
else
    data       = readmatrix(input_file);
    sampleRate = 500;
    ch_names   = {'Fp1','Fp2','F3','Fz','F4','FC1','FC2','C3','Cz','C4', ...
                  'CP1','CP2','P3','Pz','P4','POz','O1','O2','EOG','F1', ...
                  'F2','FC3','FCz','FC4','C1','C2','CP3','CP4','P5','P1','P2','P6'};
    ch_names   = ch_names(1:size(data,2));
    fprintf('CSV: %d samples x %d channels @ %.0f Hz\n', size(data,1), size(data,2), sampleRate);
end

nchannels = size(data, 2);
chunkSize = sampleRate / framerate;
if mod(chunkSize, 1) ~= 0
    error('sampleRate(%g) / framerate(%d) = %g is not an integer.', sampleRate, framerate, chunkSize);
end
chunkSize = int32(chunkSize);

%% --- resolve EOG channel names → 1-based indices ---
EOG_ch = zeros(1, numel(eog_ch_names));
for k = 1:numel(eog_ch_names)
    m = find(strcmpi(ch_names, eog_ch_names{k}), 1);
    if isempty(m)
        error('EOG channel "%s" not found.\nAvailable: %s', eog_ch_names{k}, strjoin(ch_names, ', '));
    end
    EOG_ch(k) = m;
end
non_eog_ch = setdiff(1:nchannels, EOG_ch);
fprintf('EOG channels: [%s] → [%s]\n', strjoin(eog_ch_names, ', '), num2str(EOG_ch));

%% --- design causal IIR filters ---
nyq = sampleRate / 2;
[b_lp,  a_lp]  = butter(filterOrder_EOG,   freq_low_EOG    / nyq, 'low');
[b_hp,  a_hp]  = butter(filterOrder_EOG,   freq_high_EOG   / nyq, 'high');
[b_hpk, a_hpk] = butter(filterOrder_peaks, freq_high_peaks / nyq, 'high');

%% --- initialize filter and buffer state (zero, same as ROS configure_signal) ---
zi_lp  = zeros(max(length(a_lp),  length(b_lp))  - 1, nchannels);
zi_hp  = zeros(max(length(a_hp),  length(b_hp))  - 1, nchannels);
zi_hpk = zeros(max(length(a_hpk), length(b_hpk)) - 1, nchannels);

buf_eog   = zeros(bufferSize, nchannels);
buf_peaks = zeros(bufferSize, nchannels);
buf_ptr   = 0;

%% --- chunk-by-chunk processing (starts at first_seq, same as ROS) ---
total_samples  = size(data, 1);
n_frames       = floor(total_samples / double(chunkSize));
artifact_flags = zeros(n_frames, 1);   % indexed 1-based: flag(seq+1) = result for seq

for seq = first_seq : n_frames - 1

    f   = seq + 1;   % 1-based
    idx = seq * double(chunkSize) + 1 : (seq + 1) * double(chunkSize);
    chunk = data(idx, :);   % [chunkSize x nchannels]

    % CAR: subtract mean of non-EOG channels
    car_mean  = mean(chunk(:, non_eog_ch), 2);
    chunk_car = chunk - car_mean;

    % EOG path: LP → HP  (bandpass)
    [eog_lp, zi_lp] = filter(b_lp, a_lp, chunk_car, zi_lp, 1);
    [eog_bp, zi_hp] = filter(b_hp, a_hp, eog_lp,   zi_hp, 1);

    % Peaks path: HP
    [pks_hp, zi_hpk] = filter(b_hpk, a_hpk, chunk_car, zi_hpk, 1);

    % Update ring buffers (circular)
    for s = 1:chunkSize
        bi = mod(buf_ptr, bufferSize) + 1;
        buf_eog(bi, :)   = eog_bp(s, :);
        buf_peaks(bi, :) = pks_hp(s, :);
        buf_ptr = buf_ptr + 1;
    end

    % C++ buffer is NaN-initialised: isfull() ↔ no NaN ↔ bufferSize samples written
    if buf_ptr < bufferSize
        continue;
    end

    has_artifact = false;

    % EOG check
    heog = buf_eog(:, EOG_ch(1)) - buf_eog(:, EOG_ch(2));
    if numel(EOG_ch) >= 3
        veog = (buf_eog(:, EOG_ch(1)) + buf_eog(:, EOG_ch(2))) / 2 - buf_eog(:, EOG_ch(3));
    else
        veog = (buf_eog(:, EOG_ch(1)) + buf_eog(:, EOG_ch(2))) / 2;
    end
    if max(abs(heog)) > th_hEOG || max(abs(veog)) > th_vEOG
        has_artifact = true;
    end

    % Peaks check (non-EOG channels only)
    if max(abs(buf_peaks(:, non_eog_ch)), [], 'all') > th_peaks
        has_artifact = true;
    end

    artifact_flags(f) = has_artifact;
end

%% --- compare with ROS output ---
if ~isfile(ros_file)
    warning('ROS output not found: %s', ros_file);
    frame_rate  = sampleRate / double(chunkSize);
    warmup      = ceil(bufferSize / double(chunkSize));
    t = (0:n_frames-1) / frame_rate;
    s = first_seq + warmup + 1;
    figure;
    stairs(t(s:end), artifact_flags(s:end), 'r');
    xlabel('time [s]'); ylabel('flag'); title('MATLAB only (no ROS ref)'); grid on;
    return;
end

ros_data = readmatrix(ros_file);   % [n_seq x 1], ros_data(k) = detection at seq=k-1

n_compare   = min(n_frames, length(ros_data));
ros_flags   = ros_data(1:n_compare);
mat_flags   = artifact_flags(1:n_compare);

frame_rate    = sampleRate / double(chunkSize);
warmup_frames = ceil(bufferSize / double(chunkSize));

% Both ROS and MATLAB start at first_seq with zero filter state.
% The buffer fills after warmup_frames chunks → first valid detection at first_seq+warmup_frames.
start_frame = first_seq + warmup_frames + 1;   % 1-based index into arrays

t = (0:n_compare-1) / frame_rate;

fprintf('\nAlignment: first_seq=%d, warmup=%d → comparing from seq %d (frame %d)\n', ...
        first_seq, warmup_frames, start_frame-1, start_frame);

%% --- cross-correlation to measure residual acquisition lag ---
% When using the eegdev datafile plugin (GDF), rosneuro_acquisition buffers
% one frame internally before publishing, so ROS frame N carries the samples
% MATLAB reads as frame N-1.  The cross-correlation detects this lag and we
% apply it automatically so the final comparison is properly aligned.
MAX_LAG_SEARCH = 5;   % frames – search window for xcorr
valid = start_frame : n_compare;
r_v   = ros_flags(valid);
m_v   = mat_flags(valid);

[xcf, lags] = xcorr(double(r_v) - mean(r_v), double(m_v) - mean(m_v), ...
                    MAX_LAG_SEARCH, 'normalized');
[~, peak_idx] = max(xcf);
measured_lag  = lags(peak_idx);   % positive → ROS lags MATLAB by that many frames

fprintf('Cross-correlation peak lag: %+d frame(s)  ', measured_lag);
if measured_lag == 0
    fprintf('[✓ no residual lag]\n');
elseif measured_lag > 0
    fprintf('[ROS lags MATLAB by %d frame(s) – eegdev acquisition pipeline delay]\n', measured_lag);
else
    fprintf('[MATLAB lags ROS by %d frame(s)]\n', -measured_lag);
end

% Apply lag correction: shift the two series so they are temporally aligned.
% If measured_lag > 0: drop the first measured_lag elements of mat (MATLAB
% "arrived early") and the last measured_lag of ros, keeping same length.
if measured_lag > 0
    % MATLAB is measured_lag frames ahead of ROS.
    % Align by pairing MATLAB[k] with ROS[k + measured_lag].
    r_aligned = r_v(1 + measured_lag : end);             % ROS   shifted forward
    m_aligned = m_v(1               : end - measured_lag); % MATLAB kept at start
    t_aligned = t(valid(1 : end - measured_lag));
elseif measured_lag < 0
    % ROS is |measured_lag| frames ahead of MATLAB.
    shift     = -measured_lag;
    r_aligned = r_v(1         : end - shift);
    m_aligned = m_v(1 + shift : end        );
    t_aligned = t(valid(1 : end - shift));
else
    r_aligned = r_v;
    m_aligned = m_v;
    t_aligned = t(valid);
end

n_mismatch_raw      = sum(abs(r_v       - m_v));
n_mismatch_aligned  = sum(abs(r_aligned - m_aligned));
n_valid_raw         = numel(r_v);
n_valid_aligned     = numel(r_aligned);

fprintf('Mismatches  (raw)     : %d / %d  (%.2f%%)\n', ...
        n_mismatch_raw,     n_valid_raw,     100*n_mismatch_raw    /n_valid_raw);
fprintf('Mismatches  (aligned) : %d / %d  (%.2f%%)\n', ...
        n_mismatch_aligned, n_valid_aligned, 100*n_mismatch_aligned/n_valid_aligned);

%% --- plot: raw (unaligned) ---
figure;
subplot(2,1,1);
hold on;
stairs(t(valid), r_v, 'b',   'LineWidth', 1.5);
stairs(t(valid), m_v, 'r--', 'LineWidth', 1);
legend('ROS node', 'MATLAB simulation');
ylabel('artifact flag');
title(sprintf('[RAW] %s | bufferSize=%d | hEOG=%g vEOG=%g peaks=%g | first\\_seq=%d', ...
    upper(input_mode), bufferSize, th_hEOG, th_vEOG, th_peaks, first_seq));
ylim([-0.1 1.1]); grid on; hold off;
subplot(2,1,2);
bar(t(valid), abs(r_v - m_v));
xlabel('time [s]'); ylabel('|diff|'); title(sprintf('Differences (lag=%+d)', measured_lag)); grid on;

%% --- plot: lag-corrected ---
figure;
subplot(2,1,1);
hold on;
stairs(t_aligned, r_aligned, 'b',   'LineWidth', 1.5);
stairs(t_aligned, m_aligned, 'r--', 'LineWidth', 1);
legend('ROS node', 'MATLAB simulation');
ylabel('artifact flag');
title(sprintf('[ALIGNED lag=%+d] %s | hEOG=%g vEOG=%g peaks=%g', ...
    measured_lag, upper(input_mode), th_hEOG, th_vEOG, th_peaks));
ylim([-0.1 1.1]); grid on; hold off;
subplot(2,1,2);
bar(t_aligned, abs(r_aligned - m_aligned));
xlabel('time [s]'); ylabel('|diff|'); title('Differences after alignment'); grid on;
