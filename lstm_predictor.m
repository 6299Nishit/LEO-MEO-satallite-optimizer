function predictedScore = lstm_predictor(net, featureHist)
% LSTM-based future link score prediction

N = size(featureHist,3);
predictedScore = zeros(1,N);

for i = 1:N
    X = featureHist(:,:,i);
    Ypred = predict(net,{X});
    predictedScore(i) = Ypred(end);
end
end
