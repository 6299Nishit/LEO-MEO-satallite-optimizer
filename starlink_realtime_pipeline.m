%% ============================================================
%  STARLINK-STYLE REAL-TIME SATELLITE COMMUNICATION OPTIMIZER
%
%  Satellite RF → Antenna → RF Front-End → SDR
%  → MATLAB DSP (SNR / Doppler / Delay)
%  → ML Predictor (Baseline / Kalman / Optional LSTM)
%  → Best Satellite Decision
%  → SDR Re-tune (Handover)
%  → Continuous Connectivity
%% ============================================================

clc; clear; close all;

%% ---------------- USER CONFIGURATION ----------------
satNames = {'SAT-1','SAT-2','SAT-3'};
satFreq  = [137e6, 145e6, 435e6];     % VHF/UHF test bands
orbitKm  = [550, 600, 700];           % LEO altitudes (km)
velocity = [7.6, 7.5, 7.4]*1e3;       % m/s

N  = numel(satFreq);
fs = 1e6;                             % Sample rate
spf = 4096;                           % Samples per frame
BW = 10e6;                            % Channel bandwidth
c  = 3e8;                             % Speed of light

T = 200;                              % Time frames
lockDuration = 5;                     % Handover lock
hysteresis = 2.0;                     % Ping-pong protection

USE_LSTM = false;                     % 🔁 SET TRUE TO ENABLE LSTM

%% ---------------- SIMULATED TX SIGNAL ----------------
rng(42);
txSig = exp(1j*2*pi*(0:spf-1)'/100);

%% ---------------- SDR INITIALIZATION ----------------
USE_SDR = false;
try
    rx = comm.SDRRTLReceiver( ...
        'CenterFrequency', satFreq(1), ...
        'SampleRate', fs, ...
        'SamplesPerFrame', spf, ...
        'OutputDataType','double');
    USE_SDR = true;
    fprintf('📡 RTL-SDR connected → LIVE RF enabled\n');
catch
    fprintf('⚠️ RTL-SDR not detected → Using simulated signal\n');
end

%% ---------------- LSTM INITIALIZATION ----------------
if USE_LSTM
    load lstm_satellite_model          % loads variable "net"
    featureHist = [];                  % 4 × time × N
    fprintf('🤖 LSTM ML ENABLED\n');
else
    fprintf('📐 Kalman ML ENABLED\n');
end

%% ---------------- LOGGING ----------------
logSNR   = zeros(T,N);
logDopp  = zeros(T,N);
logDelay = zeros(T,N);
logScore = zeros(T,N);
logActive = zeros(1,T);

scoreHist = zeros(3,N);

currentSat = 1;
lockTimer = 0;

fprintf('\n🚀 Starting satellite optimizer...\n');

%% ===================== MAIN LOOP =====================
for t = 1:T

    % -------- Metric buffers --------
    SNR = zeros(1,N);
    Doppler = zeros(1,N);
    Delay = zeros(1,N);

    % -------- Per-satellite processing --------
    for i = 1:N

        % ===== SIGNAL ACQUISITION =====
        if USE_SDR
            rx.CenterFrequency = satFreq(i);
            sig = rx();                        % LIVE IQ
            sig = sig / rms(sig + eps);        % Software AGC
        else
            dopShift = randn()*velocity(i)/c * satFreq(i);
            tvec = (0:spf-1)'/fs;
            sig = txSig .* exp(1j*2*pi*dopShift*tvec);
            sig = awgn(sig, 20 + randn()*5, 'measured');
        end

        % ===== DSP METRICS =====
        % SNR
        sigP = mean(abs(sig).^2);
        noiseP = var(sig - mean(sig));
        SNR(i) = 10*log10(sigP/(noiseP+eps));

        % Doppler
        ph = unwrap(angle(sig));
        instFreq = diff(ph)/(2*pi)*fs;
        Doppler(i) = mean(instFreq);

        % Delay (slant range)
        elevation = 0.3 + 0.4*rand;
        slantRange = (orbitKm(i)*1e3)/sin(elevation);
        Delay(i) = (slantRange/c)*1e3;   % ms
    end

    % -------- Throughput --------
    Throughput = BW * log2(1 + 10.^(SNR/10)) / 1e6;

    % -------- LSTM FEATURE BUILD (ONLY AFTER METRICS EXIST) --------
    if USE_LSTM
        features = [SNR; Doppler; Delay; Throughput];
        for i = 1:N
            featureHist(:,:,i) = features;
        end
    end

    % -------- SCORE --------
    score = 0.3*SNR + 0.4*Throughput ...
            - 0.2*Delay - 0.1*abs(Doppler);

    scoreHist = [scoreHist(2:end,:); score];

    % -------- ML SELECTION --------
    if t < 3
        predictedScore = baseline_predictor(scoreHist);
    elseif USE_LSTM
        predictedScore = lstm_predictor(net, featureHist);
    else
        predictedScore = kalman_predictor(scoreHist);
    end

    % -------- HANDOVER --------
    [~, bestSat] = max(predictedScore);

    if lockTimer == 0 && bestSat ~= currentSat && ...
       (predictedScore(bestSat) - predictedScore(currentSat)) > hysteresis

        fprintf('⚡ t=%03d Handover: %s → %s | ΔScore=%.2f\n', ...
            t, satNames{currentSat}, satNames{bestSat}, ...
            predictedScore(bestSat)-predictedScore(currentSat));

        currentSat = bestSat;
        lockTimer = lockDuration;
    else
        lockTimer = max(lockTimer-1,0);
    end

    % -------- LOG --------
    logSNR(t,:)   = SNR;
    logDopp(t,:)  = Doppler;
    logDelay(t,:) = Delay;
    logScore(t,:) = score;
    logActive(t)  = currentSat;

    if mod(t,20)==0
        fprintf('t=%03d | Active=%s | SNR=[%.1f %.1f %.1f]\n', ...
            t, satNames{currentSat}, SNR);
    end
end

%% ===================== RESULTS =====================
figure('Position',[100 100 1200 600],'Color','w');

subplot(2,1,1);
plot(logScore,'LineWidth',1.5);
grid on;
legend(satNames,'Location','best');
xlabel('Time Frame');
ylabel('Link Score');
title('ML-Based Link Quality');

subplot(2,1,2);
stairs(logActive,'k','LineWidth',3);
yticks(1:N);
yticklabels(satNames);
xlabel('Time Frame');
ylabel('Active Satellite');
title('Seamless Predictive Handover');
grid on;

handoverCount = sum(diff([0 logActive])~=0);
fprintf('\n✅ Simulation complete → %d handovers executed.\n', handoverCount);

%% ---------------- CLEANUP ----------------
if USE_SDR
    release(rx);
end
