% Update the GUI with run-time data from the simulation

% update altitude
altitudeBlock = 'momentumBasedFlight/Visualizer/CoM Kinematics/';
dataAltitude  = get_param(altitudeBlock,'RuntimeObject');
altitudePort  = dataAltitude.InputPort(1);
altitude      = altitudePort.Data;

flightGui.Children(4).Children(1).String = num2str(round(altitude(3),1));

% update pitching
pitchingBlock    = 'momentumBasedFlight/Visualizer/Task and Base Frame Kinematics/';
dataPitching     = get_param(pitchingBlock,'RuntimeObject');
pitchingPort     = dataPitching.InputPort(4);
fullRotation     = pitchingPort.Data;
fullRotation_rpy = wbc.rollPitchYawFromRotation(fullRotation);
pitching         = fullRotation_rpy(2)*180/pi;

flightGui.Children(3).Children(2).String = num2str(round(pitching,1));

% update speed
speedBlock = 'momentumBasedFlight/Visualizer/CoM Kinematics/';
dataSpeed  = get_param(speedBlock,'RuntimeObject');
speedPort  = dataSpeed.InputPort(3);
speed      = speedPort.Data;

flightGui.Children(2).Children(9).String = num2str(abs(round(speed(1),1)));
flightGui.Children(2).Children(8).String = num2str(abs(round(speed(2),1)));
flightGui.Children(2).Children(7).String = num2str(abs(round(speed(3),1)));

% update thusts
thrustsBlock = 'momentumBasedFlight/Visualizer/Jets Forces/';
dataThrusts  = get_param(thrustsBlock,'RuntimeObject');
thrustsPort  = dataThrusts.InputPort(2);
thrusts      = thrustsPort.Data;

flightGui.Children(1).Children(15).String = num2str(round(thrusts(1),1));
flightGui.Children(1).Children(13).String = num2str(round(thrusts(2),1));
flightGui.Children(1).Children(14).String = num2str(round(thrusts(3),1));
flightGui.Children(1).Children(12).String = num2str(round(thrusts(4),1));

% update fuel consumption
fuelBlock    = 'momentumBasedFlight/Visualizer/Jets Forces/';
dataFuel     = get_param(fuelBlock,'RuntimeObject');
fuelPort     = dataFuel.InputPort(3);
fuelConsump  = fuelPort.Data;

flightGui.Children(1).Children(4).String = num2str(round(fuelConsump,1));
