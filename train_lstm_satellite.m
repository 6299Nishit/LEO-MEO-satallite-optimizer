clc; clear;

numSteps = 300;

% Simulated training data (replace with real logs later)
SNR = 20 + randn(1,numSteps)*3;
Doppler = randn(1,numSteps)*150;
Delay = 20 + rand(1,numSteps)*10;
Throughput = 10*log2(1+10.^(SNR/10));

X = [SNR; Doppler; Delay; Throughput];
Y = 0.3*SNR + 0.4*Throughput - 0.2*Delay - 0.1*abs(Doppler);

XTrain = {X};
YTrain = {Y};

layers = [
    sequenceInputLayer(4)
    lstmLayer(32,'OutputMode','sequence')
    fullyConnectedLayer(1)
    regressionLayer];

options = trainingOptions('adam', ...
    'MaxEpochs',150, ...
    'MiniBatchSize',1, ...
    'Verbose',false);

net = trainNetwork(XTrain,YTrain,layers,options);
save lstm_satellite_model net

disp('✅ LSTM model trained and saved');
