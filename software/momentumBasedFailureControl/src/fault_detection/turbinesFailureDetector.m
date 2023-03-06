function [turbineStatus, rpm_err_avg, rpm_err] = turbinesFailureDetector(rpm_meas, rpm_star, time, Config)

% turbineStatus
%
% - default (no fail): 0
%
% - partial failure: 1
%
% - complete failure: 2
%
persistent timeFailStarted possibleFailure rpm_err_window

if isempty(timeFailStarted) || isempty(possibleFailure)
    
    timeFailStarted = zeros(length(rpm_meas),1);
    possibleFailure = zeros(length(rpm_meas),1);
    rpm_err_window  = zeros(length(rpm_meas),Config.failureDetection.size_window_rpm_err);
end
 
rpm_err       = abs(rpm_meas - rpm_star);
rpm_err_avg   = zeros(length(rpm_err),1);
turbineStatus = zeros(length(rpm_err),1);

% cycle on all turbines to detect failures
for k = 1:length(rpm_err)

    rpm_err_avg(k) = sum(rpm_err_window(k,:))/Config.failureDetection.size_window_rpm_err;

    if rpm_err_avg(k) > Config.failureDetection.maxRPMError
        
        if ~possibleFailure(k)
            
            % save failure instant (for each turbine)
            timeFailStarted(k) = time;
            possibleFailure(k) = true;
        end
        
        if time > timeFailStarted(k) + Config.failureDetection.timeThr_partFail && possibleFailure(k)
            
            % turbine cannot reach the requested rpm -> T_err ≥ threshold for time > threshold_partialFailure
            turbineStatus(k) = 1;
            
            if rpm_meas(k) <= 0
                
                % turbine shuts off -> T_err ≥ threshold and T ≤ T_idle
                turbineStatus(k) = 2;
            end
        end
    else
        possibleFailure(k) = false;
    end
end

% update error window to compute the average error in the next iteration
rpm_err_window(:,1:end-1) = rpm_err_window(:,2:end); 
rpm_err_window(:,end)     = rpm_err;
