%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RUN THIS SCRIPT TO REMOVE LOCAL PATHS ADDED WHEN RUNNING THE CONTROLLER.
%
% In the Simulink model, this script is run every time the user presses
% the terminate button.

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% close the native control GUI (if present) and reset the GUI input to zero
try
    close(ironcubControlGui);
    set_param('momentumBasedFlight/FLIGHT CONTROLLER (CORE) V3.0/Input_GUI','Value','0')  
catch ME    
    warning(ME.message)
end
try
    close(flightGui);
catch ME 
    warning(ME.message)
end
    
% remove local paths
rmpath(genpath('./src/'))
rmpath('../controlAndDataGui/');

% Try to remove chache files and folders
try
    rmdir('./slprj','s')
    delete('momentumBasedFlight.slxc')
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
   
    save(['./experiments',date,'/exp_',num2str(c(4)),'-',num2str(c(5)),'.mat'])
end
