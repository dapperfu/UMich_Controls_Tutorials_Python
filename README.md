# Controls Tutorials for Python

Based entirely on University of Michigan's [Controls Tutorials for MATLAB & Simulink](https://ctms.engin.umich.edu/CTMS/index.php?aux=Home) using [Python Control Systems Library](https://python-control.readthedocs.io/) and [Jupyter Notebooks](https://jupyter.org/)

- Interactive online notebooks is provided by [Binder](https://mybinder.org/).
- Static rendered notebooks is provided by [nbviewer](https://nbviewer.jupyter.org/)

## Installation

Install the package in editable mode with all dependencies:

```bash
pip install -e .
```

Or install with optional dependencies:

```bash
# With Modelica support
pip install -e ".[modelica]"

# With development tools
pip install -e ".[dev]"

# With both
pip install -e ".[modelica,dev]"
```

The project uses modern Python packaging with `pyproject.toml`. All dependencies are specified in the project configuration.

# Introduction

This section introduces fundamental control theory concepts including system modeling, analysis techniques, and various control design methods (PID, root locus, frequency response, state-space, and digital control). Examples include mass-spring-damper systems and magnetically suspended ball systems.

| Section | mybinder.org| nbviewer.org|
| - | - | - |
| System Modeling | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=Introduction%2FIntroduction_SystemModeling.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/Introduction/Introduction_SystemModeling.ipynb) |
| System Analysis | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=Introduction%2FIntroduction_SystemAnalysis.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/Introduction/Introduction_SystemAnalysis.ipynb) |
| Control: PID | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=Introduction%2FIntroduction_ControlPID.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/Introduction/Introduction_ControlPID.ipynb) |
| Control: Root Locus | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=Introduction%2FIntroduction_ControlRootLocus.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/Introduction/Introduction_ControlRootLocus.ipynb) |
| Control: Frequency | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=Introduction%2FIntroduction_ControlFrequency.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/Introduction/Introduction_ControlFrequency.ipynb) |
| Control: State Space | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=Introduction%2FIntroduction_ControlStateSpace.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/Introduction/Introduction_ControlStateSpace.ipynb) |
| Control: Digital | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=Introduction%2FIntroduction_ControlDigital.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/Introduction/Introduction_ControlDigital.ipynb) |

# Cruise Control

An automatic vehicle speed control system that maintains a constant speed despite external disturbances such as changes in wind or road grade. This example demonstrates a first-order system with a single state variable (velocity) and provides a practical introduction to feedback control.

| Section | mybinder.org| nbviewer.org|
| - | - | - |
| System Modeling | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=CruiseControl%2FCruiseControl_SystemModeling.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/CruiseControl/CruiseControl_SystemModeling.ipynb) |
| System Analysis | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=CruiseControl%2FCruiseControl_SystemAnalysis.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/CruiseControl/CruiseControl_SystemAnalysis.ipynb) |
| Control: PID | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=CruiseControl%2FCruiseControl_ControlPID.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/CruiseControl/CruiseControl_ControlPID.ipynb) |
| Control: Root Locus | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=CruiseControl%2FCruiseControl_ControlRootLocus.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/CruiseControl/CruiseControl_ControlRootLocus.ipynb) |
| Control: Frequency | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=CruiseControl%2FCruiseControl_ControlFrequency.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/CruiseControl/CruiseControl_ControlFrequency.ipynb) |
| Control: State Space | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=CruiseControl%2FCruiseControl_ControlStateSpace.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/CruiseControl/CruiseControl_ControlStateSpace.ipynb) |
| Control: Digital | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=CruiseControl%2FCruiseControl_ControlDigital.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/CruiseControl/CruiseControl_ControlDigital.ipynb) |

# Motor Speed

A DC motor speed control system that regulates the rotational speed of a motor shaft. This example combines electrical dynamics (armature circuit) with mechanical dynamics (rotor inertia and friction) to create a second-order system, demonstrating control of electromechanical systems.

| Section | mybinder.org| nbviewer.org|
| - | - | - |
| System Modeling | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=MotorSpeed%2FMotorSpeed_SystemModeling.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/MotorSpeed/MotorSpeed_SystemModeling.ipynb) |
| System Analysis | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=MotorSpeed%2FMotorSpeed_SystemAnalysis.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/MotorSpeed/MotorSpeed_SystemAnalysis.ipynb) |
| Control: PID | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=MotorSpeed%2FMotorSpeed_ControlPID.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/MotorSpeed/MotorSpeed_ControlPID.ipynb) |
| Control: Root Locus | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=MotorSpeed%2FMotorSpeed_ControlRootLocus.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/MotorSpeed/MotorSpeed_ControlRootLocus.ipynb) |
| Control: Frequency | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=MotorSpeed%2FMotorSpeed_ControlFrequency.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/MotorSpeed/MotorSpeed_ControlFrequency.ipynb) |
| Control: State Space | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=MotorSpeed%2FMotorSpeed_ControlStateSpace.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/MotorSpeed/MotorSpeed_ControlStateSpace.ipynb) |
| Control: Digital | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=MotorSpeed%2FMotorSpeed_ControlDigital.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/MotorSpeed/MotorSpeed_ControlDigital.ipynb) |

# Motor Position

A DC motor position control system that regulates the angular position of a motor shaft. Similar to the motor speed example but with position as the output, this system requires an integrator and demonstrates control of systems with higher-order dynamics.

| Section | mybinder.org| nbviewer.org|
| - | - | - |
| System Modeling | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=MotorPosition%2FMotorPosition_SystemModeling.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/MotorPosition/MotorPosition_SystemModeling.ipynb) |
| System Analysis | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=MotorPosition%2FMotorPosition_SystemAnalysis.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/MotorPosition/MotorPosition_SystemAnalysis.ipynb) |
| Control: PID | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=MotorPosition%2FMotorPosition_ControlPID.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/MotorPosition/MotorPosition_ControlPID.ipynb) |
| Control: Root Locus | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=MotorPosition%2FMotorPosition_ControlRootLocus.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/MotorPosition/MotorPosition_ControlRootLocus.ipynb) |
| Control: Frequency | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=MotorPosition%2FMotorPosition_ControlFrequency.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/MotorPosition/MotorPosition_ControlFrequency.ipynb) |
| Control: State Space | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=MotorPosition%2FMotorPosition_ControlStateSpace.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/MotorPosition/MotorPosition_ControlStateSpace.ipynb) |
| Control: Digital | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=MotorPosition%2FMotorPosition_ControlDigital.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/MotorPosition/MotorPosition_ControlDigital.ipynb) |

# Suspension

An automotive active suspension system using a quarter-car (1/4 bus) model. This example demonstrates control of a multi-mass, multi-spring-damper system to minimize body motion when encountering road disturbances, showcasing control design for multi-input, multi-output (MIMO) systems.

| Section | mybinder.org| nbviewer.org|
| - | - | - |
| System Modeling | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=Suspension%2FSuspension_SystemModeling.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/Suspension/Suspension_SystemModeling.ipynb) |
| System Analysis | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=Suspension%2FSuspension_SystemAnalysis.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/Suspension/Suspension_SystemAnalysis.ipynb) |
| Control: PID | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=Suspension%2FSuspension_ControlPID.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/Suspension/Suspension_ControlPID.ipynb) |
| Control: Root Locus | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=Suspension%2FSuspension_ControlRootLocus.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/Suspension/Suspension_ControlRootLocus.ipynb) |
| Control: Frequency | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=Suspension%2FSuspension_ControlFrequency.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/Suspension/Suspension_ControlFrequency.ipynb) |
| Control: State Space | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=Suspension%2FSuspension_ControlStateSpace.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/Suspension/Suspension_ControlStateSpace.ipynb) |
| Control: Digital | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=Suspension%2FSuspension_ControlDigital.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/Suspension/Suspension_ControlDigital.ipynb) |

# Inverted Pendulum

A classic control problem involving an inverted pendulum mounted on a motorized cart. This unstable, nonlinear system requires active control to maintain the pendulum in an upright position. The problem is analogous to attitude control of a booster rocket at takeoff and demonstrates control of unstable systems.

| Section | mybinder.org| nbviewer.org|
| - | - | - |
| System Modeling | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=InvertedPendulum%2FInvertedPendulum_SystemModeling.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/InvertedPendulum/InvertedPendulum_SystemModeling.ipynb) |
| System Analysis | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=InvertedPendulum%2FInvertedPendulum_SystemAnalysis.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/InvertedPendulum/InvertedPendulum_SystemAnalysis.ipynb) |
| Control: PID | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=InvertedPendulum%2FInvertedPendulum_ControlPID.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/InvertedPendulum/InvertedPendulum_ControlPID.ipynb) |
| Control: Root Locus | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=InvertedPendulum%2FInvertedPendulum_ControlRootLocus.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/InvertedPendulum/InvertedPendulum_ControlRootLocus.ipynb) |
| Control: Frequency | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=InvertedPendulum%2FInvertedPendulum_ControlFrequency.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/InvertedPendulum/InvertedPendulum_ControlFrequency.ipynb) |
| Control: State Space | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=InvertedPendulum%2FInvertedPendulum_ControlStateSpace.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/InvertedPendulum/InvertedPendulum_ControlStateSpace.ipynb) |
| Control: Digital | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=InvertedPendulum%2FInvertedPendulum_ControlDigital.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/InvertedPendulum/InvertedPendulum_ControlDigital.ipynb) |

# Aircraft Pitch

An aircraft autopilot system that controls the pitch angle of an aircraft using elevator deflection. Based on linearized longitudinal dynamics from Boeing commercial aircraft data, this example demonstrates control design for aerospace applications with multiple state variables.

| Section | mybinder.org| nbviewer.org|
| - | - | - |
| System Modeling | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=AircraftPitch%2FAircraftPitch_SystemModeling.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/AircraftPitch/AircraftPitch_SystemModeling.ipynb) |
| System Analysis | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=AircraftPitch%2FAircraftPitch_SystemAnalysis.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/AircraftPitch/AircraftPitch_SystemAnalysis.ipynb) |
| Control: PID | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=AircraftPitch%2FAircraftPitch_ControlPID.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/AircraftPitch/AircraftPitch_ControlPID.ipynb) |
| Control: Root Locus | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=AircraftPitch%2FAircraftPitch_ControlRootLocus.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/AircraftPitch/AircraftPitch_ControlRootLocus.ipynb) |
| Control: Frequency | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=AircraftPitch%2FAircraftPitch_ControlFrequency.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/AircraftPitch/AircraftPitch_ControlFrequency.ipynb) |
| Control: State Space | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=AircraftPitch%2FAircraftPitch_ControlStateSpace.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/AircraftPitch/AircraftPitch_ControlStateSpace.ipynb) |
| Control: Digital | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=AircraftPitch%2FAircraftPitch_ControlDigital.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/AircraftPitch/AircraftPitch_ControlDigital.ipynb) |

# Ball & Beam

A ball position control system where a ball rolls along a beam, and the beam angle is controlled by a servo motor. This system features a double integrator plant (marginally stable) and demonstrates control of systems with challenging dynamics, requiring careful controller design to achieve stability.

| Section | mybinder.org| nbviewer.org|
| - | - | - |
| System Modeling | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=BallBeam%2FBallBeam_SystemModeling.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/BallBeam/BallBeam_SystemModeling.ipynb) |
| System Analysis | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=BallBeam%2FBallBeam_SystemAnalysis.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/BallBeam/BallBeam_SystemAnalysis.ipynb) |
| Control: PID | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=BallBeam%2FBallBeam_ControlPID.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/BallBeam/BallBeam_ControlPID.ipynb) |
| Control: Root Locus | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=BallBeam%2FBallBeam_ControlRootLocus.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/BallBeam/BallBeam_ControlRootLocus.ipynb) |
| Control: Frequency | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=BallBeam%2FBallBeam_ControlFrequency.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/BallBeam/BallBeam_ControlFrequency.ipynb) |
| Control: State Space | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=BallBeam%2FBallBeam_ControlStateSpace.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/BallBeam/BallBeam_ControlStateSpace.ipynb) |
| Control: Digital | [![Binder](binder_badge.svg)](https://mybinder.org/v2/gh/dapperfu/UMich_Controls_Tutorials_Python/vibe_updates?filepath=BallBeam%2FBallBeam_ControlDigital.ipynb) | [![nbviewer](nbviewer_badge.svg)](https://nbviewer.org/github/dapperfu/UMich_Controls_Tutorials_Python/blob/vibe_updates/BallBeam/BallBeam_ControlDigital.ipynb) |

# Modelica Models

This repository now includes Modelica models converted from the original Simulink examples. The Modelica models provide an alternative implementation using the Modelica language and can be simulated using OpenModelica.

## Quick Start

1. **Install OpenModelica** (Linux):
   ```bash
   bash Modelica/scripts/install_modelica.sh
   ```

2. **Install Python dependencies**:
   ```bash
   pip install OMPython
   ```

3. **Use in Python**:
   ```python
   from OMPython import ModelicaSystem
   model = ModelicaSystem("Modelica/UMichControls/CruiseControl/CruiseControl_System.mo", "CruiseControl_System")
   model.simulate()
   ```

For detailed documentation, see [Modelica/README.md](Modelica/README.md).

## Available Modelica Models

All 8 example systems have been converted to Modelica:

- **Introduction**: Mass-spring-damper, magnetic suspension
- **CruiseControl**: Vehicle speed control
- **MotorSpeed**: DC motor speed control
- **MotorPosition**: DC motor position control
- **Suspension**: Quarter-car active suspension
- **InvertedPendulum**: Cart-pendulum system
- **AircraftPitch**: Aircraft pitch control
- **BallBeam**: Ball and beam position control

The models are organized in the `Modelica/UMichControls/` package following Modelica Standard Library conventions.

# Copyright

The Control Tutorials for Python are licensed under a [Creative Commons Attribution-ShareAlike 4.0 International License](http://creativecommons.org/licenses/by-sa/4.0/). This means that the contents of this website may be copied and adapted for other uses as long as the user provides credit to the original authors (Attribution) and distributes their materials under the same license as we use here (ShareAlike).

The Controls Tutorial for Python is based on the [Controls Tutorials for MATLAB & Simulink](https://ctms.engin.umich.edu/CTMS/index.php?aux=Home).
