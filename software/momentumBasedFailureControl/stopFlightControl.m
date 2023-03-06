%% Stop Flight Control
%
% Just stops the Simulink model
%

MODEL_NAME = 'momentumBasedFlight.mdl';

set_param(bdroot,'simulationcommand','stop')

save_system(MODEL_NAME);
close_system(MODEL_NAME);
