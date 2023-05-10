## Offline joints postion and attitude optimization

The script implements a nonliner optimization to find the optimal joints position, attitude and thrust in case of turbine failure. The optimizer accounts for:

- self collision avoidance
- jet ehaust cones constraints
- enhanced maneuverability

## How to configure the optimizer

configuration parameters are all stored in [initJetFailureOptimization_MK1_1](init/initJetFailureOptimization_MK1_1.m).

## How to run the optimizer

simply run the `runJetFailureOptimization` script.
