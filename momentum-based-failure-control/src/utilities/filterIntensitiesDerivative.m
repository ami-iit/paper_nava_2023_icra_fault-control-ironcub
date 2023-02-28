function jetIntensitiesDerivativeDes = filterIntensitiesDerivative(jetIntensitiesEst, jetIntensitiesDerivativeDes, Config)
% The logic here is:
% If the thust is lower than the idle one -> 0
% If the desired thrust derivative is negative -> 0
% 4 possible cases:
% lower thrust and neg des derivative = 0 + 0 -> 0 des derivative
% lower thrust and pos des derivative = 0 + 1 -> pos des derivative
% higher thrust and neg des derivative = 1 + 0 -> neg des derivative
% higher thrust and pos des derivative = 1 + 1 -> positive des derivative
% (The last case - without the sign operator - would multiply the desired
% derivative by 2)
idleJetIntensities = Config.saturation.idleJetsInt;
slack = 3;
check = sign((jetIntensitiesEst > idleJetIntensities + slack) + (jetIntensitiesDerivativeDes > 0));
jetIntensitiesDerivativeDes = jetIntensitiesDerivativeDes .* check;
end
