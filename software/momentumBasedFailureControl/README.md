## Momentum-based flight controller for Gazebo simulations

**This controller is maintained for the robots:**

- `iRonCub-Mk1_1_Gazebo`

## Tested simulation types:

#### Baseline controller

Jets dynamic model is just the numerical integration of the desired thrusts rate of changes. The controller uses the thrusts from numerical integration. To enable this mode, one must select the correct Gazebo jet plugin from [ironcub-mk1-software](https://github.com/ami-iit/ironcub-mk1-software). When running the `cmake` configuration of iRonCub software, be sure to set the following options:

```
USE_NONLINEAR_JET_DYNAMICS_PLUGIN = FALSE
USE_FIRST_ORDER_JET_DYNAMICS_PLUGIN = TRUE
```

also, deactivate the turbine fault by modifying the manual switch inside `momentumBasedFlight.mdl`:

![image](https://github.com/ami-iit/paper_nava_2023_icra_fault-control-ironcub/assets/12396934/f8efbbe5-fda8-4254-8d3b-523e79923a34)

Optional: to deactivate the autopilot mode and control the robot via the native GUI, set:

[Config.AUTOPILOT_ON = false](app/robots/iRonCub-Mk1_1_Gazebo/gainsAndParameters.m#L212)

[Config.USE_NATIVE_GUI = true](initMomentumBasedFlight.m#L50)

#### Fault control

It uses the exact same configuration as the baseline controller. The user can select on which turbine the failure occurs by using the manual switch inside `momentumBasedFlight.mdl`.

## How to run simulations

follow the [README](https://github.com/ami-iit/ironcub-mk1-software/blob/main/flight-controllers-stable/README.md) of `ironcub-mk1-software` repo.
