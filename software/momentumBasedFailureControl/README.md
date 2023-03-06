## Momentum-based flight controller for Gazebo simulations

**This controller is maintained for the robots:**

- `iRonCub-Mk1_1_Gazebo`
- `iRonCub-Mk1_Gazebo`

## Simulation type:

#### Without jet dynamics and estimated thrusts
Jets dynamic model is just the numerical integration of the desired thrusts rate of changes. The controller uses the thrusts from numerical integration.

#### With jet dynamics
Jet dynamic model is designed based on experiments with the real turbines. The thrust rate of change is stabilized towards the reference values by means of a low-level jets control that sets desired throttle values. The controller uses the thrusts from Gazebo.

#### With jet dynamics and estimated thrusts
Jet dynamic model is designed based on experiments with the real turbines. The thrust rate of change is stabilized towards the reference values by means of a low-level jets control that sets desired throttle values. The controller uses the estimated thrust from a EKF-based thrusts estimation algorithm.

### Available simulations:

**Robot:** `iRonCub-Mk1_Gazebo`

| SIMULATION TYPE | TAKE OFF | HOVERING | SLOW FLIGHT MANEUVERS | FAST FLIGHT MANEUVERS | LANDING |
|:-------:|:------:|:--------:|:--------:|:--------------------:|:--------------------:|
|Without jet dynamics and estimated thrusts | YES |  YES |  YES |  YES |  YES |
|With jet dynamics | YES |  YES |  YES |  NO |  YES |
|With jet dynamics and estimated thrusts | NO |  NO |  NO |  NO |  NO |

**Robot:** `iRonCub-Mk1_1_Gazebo`

| SIMULATION TYPE | TAKE OFF | HOVERING | SLOW FLIGHT MANEUVERS | FAST FLIGHT MANEUVERS | LANDING |
|:-------:|:------:|:--------:|:--------:|:--------------------:|:--------------------:|
|Without jet dynamics and estimated thrusts | YES |  YES |  YES |  YES |  YES |
|With jet dynamics | YES |  YES |  YES |  NO |  NO |
|With jet dynamics and estimated thrusts | NO |  NO |  NO |  NO |  NO |
