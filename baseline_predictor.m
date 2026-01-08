function predictedScore = baseline_predictor(scoreHist)
% --------------------------------------------------
% BASELINE ML PREDICTOR (Simple weighted average)
% Used for startup & fallback
% --------------------------------------------------

alpha = 0.7;
predictedScore = alpha*scoreHist(end,:) + ...
                 (1-alpha)*mean(scoreHist,1);
end
