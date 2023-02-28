function [turbineStatus, T_err_avg, T_err] = turbinesFailureDetector(jetsIntensities, jetsIntensities_star, time, Config)

% turbineStatus
%
% - default (no fail): 0
%
% - drop in the RPM: 1
%
% - partial failure: 2
%
% - complete failure: 3
%
persistent timeFailStarted possibleFailure T_err_window

if isempty(timeFailStarted) || isempty(possibleFailure)
    
    timeFailStarted = zeros(length(jetsIntensities),1);
    possibleFailure = zeros(length(jetsIntensities),1);
    T_err_window    = zeros(length(jetsIntensities),Config.failureDetection.size_window_T_err);
end
 
T_err         = abs(jetsIntensities - jetsIntensities_star);
T_err_avg     = zeros(length(T_err),1);
turbineStatus = zeros(length(T_err),1);

% cycle on all turbines to detect failures
for k = 1:length(T_err)

    T_err_avg(k) = sum(T_err_window(k,:))/Config.failureDetection.size_window_T_err;

    if T_err_avg(k) > Config.failureDetection.maxThrustError
        
        if ~possibleFailure(k)
            
            % save failure instant (for each turbine)
            timeFailStarted(k) = time;
            possibleFailure(k) = true;
        end
        
        if time > timeFailStarted(k) + Config.failureDetection.timeThr_drop && time <= timeFailStarted(k) + Config.failureDetection.timeThr_partFail && possibleFailure(k)
        
            % temporary drop in the rpm -> T_err ≥ threshold for time > threshold_drop
            turbineStatus(k) = 1;
        end
        if time > timeFailStarted(k) + Config.failureDetection.timeThr_partFail && possibleFailure(k)
            
            % turbine cannot reach the requested thrust -> T_err ≥ threshold for time > threshold_partialFailure
            turbineStatus(k) = 2;
            
            if jetsIntensities(k) <= Config.inequalitiesQP.idleJetsInt
                
                % turbine shuts off -> T_err ≥ threshold and T ≤ T_idle
                turbineStatus(k) = 3;
            end
        end
    else
        possibleFailure(k) = false;
    end
end

% update error window to compute the average error in the next iteration
T_err_window(:,1:end-1) = T_err_window(:,2:end); 
T_err_window(:,end)     = T_err;
