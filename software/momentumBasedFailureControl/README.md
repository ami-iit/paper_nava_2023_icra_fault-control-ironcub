## Momentum-based flight controller for Gazebo simulations

**This controller is maintained for the robots:**

- `iRonCub-Mk1_1_Gazebo_v1`

## Tested simulation types:

#### Baseline controller

Jets dynamic model is just the numerical integration of the desired thrusts rate of changes. The controller uses the thrusts from numerical integration. To enable this mode, one must select the correct Gazebo jet plugin from [ironcub_software](https://github.com/ami-iit/ironcub_software). When running the `cmake` configuration of iRonCub software, be sure to set the following options:

```
USE_NONLINEAR_JET_DYNAMICS_PLUGIN = FALSE
USE_FIRST_ORDER_JET_DYNAMICS_PLUGIN = TRUE
```

also, deactivate the turbine fault by modifying the manual switch inside `momentumBasedFlight.mdl`:


Optional: to deactivate the autopilot mode and control the robot via the native GUI, set:

https://github.com/ami-iit/paper_nava_2023_icra_fault-control-ironcub/blob/main/software/momentumBasedFailureControl/app/robots/iRonCub-Mk1_1_Gazebo_v1/gainsAndParameters.m#L212

to FALSE, and 

https://github.com/ami-iit/paper_nava_2023_icra_fault-control-ironcub/blob/main/software/momentumBasedFailureControl/initMomentumBasedFlight.m#L50

to TRUE.

#### Fault control

It uses the exact same configuration as the baseline controller. The user can select on which turbine the failure occurs by using the manual switch inside `momentumBasedFlight.mdl`.

## How to run simulations

follow the [README](https://github.com/ami-iit/ironcub_software/blob/main/flight-controllers-stable/README.md) of `ironcub_software` repo.
