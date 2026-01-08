%% ============================================================
%  STARLINK-STYLE REAL-TIME SATELLITE GROUND STATION SIMULATOR
%  DSP → SNR/Doppler/Delay → AI → Handover → Frequency Retune
%% ============================================================
clc; clear; close all;

%% ---------------- USER CONFIGURATION ----------------
satNames = {'SAT-1','SAT-2','SAT-3'};
satFreq  = [137e6, 145e6, 435e6];
orbitKm  = [550, 600, 700];
velocity = [7.6, 7.5, 7.4]*1e3;
N = numel(satFreq);
fs  = 1e6; spf = 4096; BW = 10e6; c = 3e8;
lockDuration = 5; hysteresis = 2.0; T = 200;

%% ---------------- SIMULATED SIGNAL ----------------
rng(42);
txSig = exp(1j*2*pi*(0:spf-1)'/100);

%% ---------------- DSP BLOCKS ----------------
rrc = rcosdesign(0.35,6,4);
carrierSync = comm.CarrierSynchronizer('Modulation','QPSK','SamplesPerSymbol',4);
timingSync = comm.SymbolSynchronizer( ...
    'TimingErrorDetector','Gardner (non-data-aided)', ...
    'SamplesPerSymbol',2, 'DampingFactor',1.0, ...
    'NormalizedLoopBandwidth',0.01);

%% ---------------- STATE ----------------
currentSat = 1; lockTimer = 0;
scoreHist  = zeros(3,N);
logActive  = zeros(1,T);  % ✅ FIXED: Row vector (1xT)
logScore   = zeros(T,N);

%% ===================== MAIN LOOP ======================
fprintf('🚀 Starting Starlink-style satellite tracking...\n');
for t = 1:T
    SNR = zeros(1,N); Doppler = zeros(1,N); Delay = zeros(1,N);
    
    for i = 1:N
        rxFreq = satFreq(i);
        dopplerShift = randn()*velocity(i)/c * rxFreq;
        timeVec = (0:spf-1)'/fs;
        sig = txSig .* exp(1j*2*pi*dopplerShift*timeVec);
        snr_db = 20 + randn(1)*5;
        sig = awgn(sig, snr_db, 'measured');
        
        %% DSP
        filtSig = filter(rrc,1,sig);
        symSig  = timingSync(filtSig);
        carrSig = carrierSync(symSig);
        
        %% METRICS
        sigPower = mean(abs(carrSig).^2);
        noisePower = var(carrSig - mean(carrSig));
        SNR(i) = 10*log10(sigPower/(noisePower+eps));
        
        X = fftshift(fft(sig)); [~,idx] = max(abs(X));
        freqAxis = linspace(-fs/2,fs/2,spf);
        Doppler(i) = freqAxis(idx);
        
        elevation = 0.3 + 0.4*rand;
        slantRange = (orbitKm(i)*1e3)/sin(elevation);
        Delay(i) = (slantRange/c)*1e3;
    end
    
    %% AI SCORING
    Throughput = BW*log2(1+10.^(SNR/10))/1e6;
    score = 0.3*SNR + 0.4*Throughput - 0.2*Delay - 0.1*abs(Doppler);
    
    scoreHist = [scoreHist(2:end,:); score];
    if t < 3
        predictedScore = mean(scoreHist,1);
    else
        alpha = 0.7;
        predictedScore = alpha*score + (1-alpha)*mean(scoreHist(end-2:end,:),1);
    end
    
    %% HANDOVER
    [~, bestSat] = max(predictedScore);
    
    if lockTimer == 0 && bestSat ~= currentSat && ...
       (predictedScore(bestSat)-predictedScore(currentSat)) > hysteresis
        fprintf('⚡ t=%03d: Handover %s → %s (Δscore=%.1f)\n', ...
            t, satNames{currentSat}, satNames{bestSat}, ...
            predictedScore(bestSat)-predictedScore(currentSat));
        currentSat = bestSat;
        lockTimer = lockDuration;
    else
        lockTimer = max(lockTimer-1,0);
    end
    
    logActive(t) = currentSat;  % ✅ Row vector assignment
    logScore(t,:) = score;
    if mod(t,20)==0
        fprintf('t=%03d | Active=%s | SNR=[%.1f %.1f %.1f]\n', ...
            t, satNames{currentSat}, SNR);
    end
end

%% ===================== PLOTS ======================
figure('Position',[100 100 1200 600],'Color','w');
subplot(2,1,1);
plot(1:T,logScore(:,1),'b-',1:T,logScore(:,2),'r-',1:T,logScore(:,3),'g-','LineWidth',1.5);
grid on; legend(satNames,'Location','best');
xlabel('Time Frame'); ylabel('Link Score');
title('Starlink-Style Link Quality');

subplot(2,1,2);
stairs(1:T,logActive,'k','LineWidth',3);
yticks(1:N); yticklabels(satNames);
xlabel('Time Frame'); ylabel('Active Satellite');
title('AI-Driven Handover');
grid on;

%% ✅ FIXED HANDOVER COUNT
logActive = logActive(:)';  % Ensure 1xT row vector
handoverCount = sum(diff([0 logActive])~=0);
fprintf('\n✅ Simulation complete. %d handovers executed.\n', handoverCount);
