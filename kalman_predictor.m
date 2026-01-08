function predictedScore = kalman_predictor(scoreHist)
% --------------------------------------------------
% KALMAN FILTER BASED LINK QUALITY PREDICTOR
% Real-time, training-free ML
% --------------------------------------------------

N = size(scoreHist,2);

Q = 0.01;   % Process noise
R = 0.5;    % Measurement noise

predictedScore = zeros(1,N);

for i = 1:N
    x = scoreHist(end,i);   % State
    P = 1;                 % Covariance

    % Prediction
    x_pred = x;
    P_pred = P + Q;

    % Update
    K = P_pred / (P_pred + R);
    x_update = x_pred + K*(x - x_pred);

    predictedScore(i) = x_update;
end
end
