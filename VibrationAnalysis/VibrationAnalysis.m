clear;
clc;
close all;

vibration_test = 'fort_side2.wav';

% Read the .wav file: y = audio data, fs = sampling frequency
[y, fs] = audioread(vibration_test);

% Use only channel 1
y = y(:,1);

% See max value of y to check for clipping
max_y = max(abs(y));

% Perform spectral analysis
N = length(y);        % Signal length
Nsec = N / fs;        % Duration in seconds
t = (0:N-1) / fs;     % Time vector

% Spectrogram windowing
Win = 4 * 512;
Noverlap = fix(Win / 2);

% Reduce datapoints for faster runtime
fsRed = 0:10:fs/2;

% Remove DC and normalize
y = y - mean(y);       
y = y / max(abs(y));   

% Apply window
window = hann(N);
y_win = y .* window;

% FFT
Y = fft(y_win);
P = abs(Y / N);
P = P(1:N/2+1);
P(2:end-1) = 2*P(2:end-1);

% Frequency vector
f = fs*(0:(N/2))/N;

% Spectrogram analysis
[yOG, fOG, tOG, pOG] = spectrogram(y, Win, Noverlap, Win, fs, 'yaxis');

% Plot the single-sided amplitude spectrum
figure(1)
plot(f, (P/max(P)))
title('Single-Sided Amplitude Spectrum', 'FontSize', 14)
xlabel('Frequency (Hz)', 'FontSize', 12)
ylabel('Normalized Amplitude', 'FontSize', 12)
xlim([0 4000])  % Adjust view to interesting frequency range

% Plot the time-domain signal and the spectrogram
figure(2)

subplot(2,1,1)
xaxisP1 = linspace(0, Nsec, N);
plot(xaxisP1, y)
title('Time-Domain Signal', 'FontSize', 14)
xlabel('Time (s)', 'FontSize', 12)
ylabel('Amplitude', 'FontSize', 12)
grid on

subplot(2,1,2)
surf(tOG, fOG/1000, 20*log10(abs(pOG)), 'EdgeColor', 'none');
axis xy; axis tight;
colormap(jet);
view(0, 90);
title('Spectral Analysis', 'FontSize', 14)
xlabel('Time (s)', 'FontSize', 12)
ylabel('Frequency (kHz)', 'FontSize', 12)

%% Save figures (optional)
% saveas(figure(1), 'AmplitudeSpectrum_Pos1_0-30.jpg')
% saveas(figure(2), 'SpectralAnalysis_Pos1_0-30.jpg')
