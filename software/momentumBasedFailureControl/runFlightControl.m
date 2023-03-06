%% Run Flight Control
%
% Just runs the Simulink model
%

MODEL_NAME = 'momentumBasedFlight.mdl';

open_system(MODEL_NAME,'loadonly');

set_param(bdroot,'simulationcommand','start')
