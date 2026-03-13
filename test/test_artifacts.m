%% process throught matlab
% not modification needed for these informations
datapath = './src/artifacts_bci/test/';
filein = [datapath ,'rawdata.csv'];
data = readmatrix(filein);
filterOrder = 4;
bufferSize = 500;
sampleRate = 500;
frameSize = 25;
nsamples = size(data, 1);
nchannels = size(data, 2);

%% apply the processing
header.Label = [{'FP1'}    {'FP2'}    {'F3'}    {'FZ'}    {'F4'}    {'FC1'}    {'FC2'}    {'C3'}    {'CZ'}    {'C4'}    {'CP1'}    {'CP2'}  ...
    {'P3'}    {'PZ'}    {'P4'}    {'POZ'}    {'O1'}    {'O2'}    {'EOG'}    {'F1'}    {'F2'} ...
    {'FC3'}    {'FCZ'}    {'FC4'}    {'C1'}    {'C2'}    {'CP3'}    {'CP4'}    {'P5'}    {'P1'}    {'P2'}    {'P6'}];
header.SampleRate = 500;
chunkSize = 25;
eog.filterOrder = 4;
eog.band = [1 10];
eog.label = {'FP1', 'FP2', 'EOG'};
eog.h_threshold = 60;
eog.v_threshold = 60;
muscle.filterOrder = 4;
muscle.freq = 1; % remove antneuro problems
muscle.threshold = 100;
artifact = artifact_rejection(data, header, nchannels, bufferSize, chunkSize, eog, muscle);

%% Load file of rosneuro
SampleRate =  sampleRate/frameSize;
start = 2 * SampleRate;

file = [datapath '/artifacts.csv'];

disp(['Loading file: ' file])
ros_data = readmatrix(file);
matlab_data = artifact;
c_title = "processed with ros node simulation";
nsamples = size(matlab_data,1);
t = 0:1/SampleRate:nsamples/SampleRate - 1/SampleRate;


figure;
subplot(2, 1, 1);
hold on;
plot(t(start:end), ros_data(start:size(t,2)), 'b', 'LineWidth', 1);
plot(t(start:end), matlab_data(start:size(t, 2)), 'r');
legend('rosneuro', 'matlab');
hold off;
grid on;

subplot(2,1,2)
bar(t(start:end), abs(ros_data(start:size(t,2))- matlab_data(start:size(t,2))));
grid on;
xlabel('time [s]');
ylabel('amplitude [uV]');
title('Difference')

sgtitle(['Evaluation' c_title])
