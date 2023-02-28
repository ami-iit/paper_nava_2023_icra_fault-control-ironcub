%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RUN THIS SCRIPT TO REMOVE LOCAL PATHS ADDED WHEN RUNNING THE CONTROLLER.
%
% In the Simulink model, this script is run every time the user presses
% the terminate button.

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% close the native control GUI (if present) and reset the GUI input to zero

% remove local paths
rmpath(genpath('./src/'))

% Try to remove chache files and folders
try
    rmdir('./slprj','s')
    delete('momentumBasedFlightSim.slxc')
catch ME
    warning(ME.message)
end

% Create a folder for collecting data
if Config.SAVE_WORKSPACE
    
    if (~exist(['experiments',date],'dir'))
        
        mkdir(['experiments',date]);
    end
    
    matFileList = dir(['./experiments',date,'/*.mat']);
    c           = clock;
    
    if (~exist(['experiments',date, '/exp_',num2str(c(4)),'-',num2str(c(5))],'dir'))
        
        mkdir(['experiments',date, '/exp_',num2str(c(4)),'-',num2str(c(5))]);
    end
    
    save(['./experiments',date,'/exp_',num2str(c(4)),'-',num2str(c(5)),'/workspace.mat'])
end
