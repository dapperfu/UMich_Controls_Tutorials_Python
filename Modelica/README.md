# Modelica Models for UMich Controls Tutorials

This directory contains Modelica models converted from the original Simulink examples in the University of Michigan Controls Tutorials.

## Installation

### Linux (Ubuntu 24.04 / Linux Mint 22)

Run the installation script:

```bash
bash Modelica/scripts/install_modelica.sh
```

This script will:
- Add the OpenModelica repository
- Install OpenModelica and dependencies
- Verify the installation

### Manual Installation

For other Linux distributions or manual installation:

1. Add OpenModelica repository (see `scripts/install_modelica.sh` for details)
2. Install OpenModelica: `sudo apt-get install openmodelica`
3. Verify: `omc --version`

### Python Integration

Install Python dependencies for Modelica integration:

```bash
pip install OMPython
```

For image parsing (optional):

```bash
pip install opencv-python pytesseract Pillow
```

## Package Structure

The Modelica models are organized in the `UMichControls` package:

```
UMichControls/
├── Introduction/          # Mass-spring-damper, magnetic suspension
├── CruiseControl/         # Vehicle speed control
├── MotorSpeed/            # DC motor speed control
├── MotorPosition/         # DC motor position control
├── Suspension/            # Quarter-car active suspension
├── InvertedPendulum/      # Cart-pendulum system
├── AircraftPitch/         # Aircraft pitch control
└── BallBeam/              # Ball and beam position control
```

## Usage

### Using OpenModelica Command Line

1. Start OpenModelica:

```bash
omc
```

2. Load the package:

```modelica
loadModel(UMichControls)
```

3. Simulate a model:

```modelica
simulate(UMichControls.CruiseControl.CruiseControl_System)
```

### Using Python with OMPython

```python
from OMPython import ModelicaSystem

# Create model instance
model = ModelicaSystem(
    "Modelica/UMichControls/CruiseControl/CruiseControl_System.mo",
    "CruiseControl_System"
)

# Set parameters
model.setParameters("m=1000", "b=50")

# Simulate
model.simulate()

# Get results
results = model.getSolutions()
time = results['time']
velocity = results['v']
```

### Example: Cruise Control System

```python
from OMPython import ModelicaSystem
import matplotlib.pyplot as plt

# Load model
model = ModelicaSystem(
    "Modelica/UMichControls/CruiseControl/CruiseControl_System.mo",
    "CruiseControl_System"
)

# Set simulation parameters
model.setSimulationOptions("startTime=0", "stopTime=10", "stepSize=0.01")

# Set input (step input of 500 N)
model.setInputs("u=500")

# Simulate
model.simulate()

# Get results
results = model.getSolutions()
time = results['time']
velocity = results['v']

# Plot
plt.plot(time, velocity)
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Cruise Control System Response')
plt.grid(True)
plt.show()
```

## Model Descriptions

### Introduction Examples

- **MassSpringDamper**: Simple mechanical system with mass, spring, and damper
- **MagneticSuspension**: Magnetically suspended ball system (linearized)

### Cruise Control

- **CruiseControl_System**: First-order vehicle speed control system

### Motor Control

- **MotorSpeed_System**: DC motor with electrical and mechanical dynamics
- **MotorPosition_System**: DC motor position control (includes integrator)

### Suspension

- **Suspension_System**: Quarter-car active suspension (MIMO system)

### Inverted Pendulum

- **InvertedPendulum_System**: Unstable cart-pendulum system

### Aircraft Pitch

- **AircraftPitch_System**: Linearized aircraft longitudinal dynamics

### Ball and Beam

- **BallBeam_System**: Double integrator plant for ball position control

## Comparison with Python Control Systems Library

The Modelica models provide an alternative implementation to the Python Control Systems Library used in the Jupyter notebooks. Both approaches can be used to:

- Validate system models
- Compare simulation results
- Learn different modeling paradigms
- Understand physical system dynamics

## Parsing Simulink Models

The `scripts/parse_simulink.py` utility can parse Simulink models from various sources:

```bash
# Parse .mdl file
python Modelica/scripts/parse_simulink.py model.mdl

# Parse .slx file
python Modelica/scripts/parse_simulink.py model.slx

# Parse block diagram image
python Modelica/scripts/parse_simulink.py diagram.png

# Parse Jupyter notebook for equations
python Modelica/scripts/parse_simulink.py notebook.ipynb
```

## Integration with Jupyter Notebooks

Modelica code cells have been integrated into the Jupyter notebooks. See the individual notebooks for examples of:

- Loading Modelica models
- Running simulations
- Comparing results with Python Control Systems Library
- Visualizing system responses

## References

- [OpenModelica Documentation](https://openmodelica.org/doc/)
- [Modelica Language Specification](https://specification.modelica.org/)
- [OMPython Documentation](https://openmodelica.org/doc/OpenModelicaUsersGuide/latest/ompython.html)
- [University of Michigan Controls Tutorials](https://ctms.engin.umich.edu/CTMS/index.php?aux=Home)

## License

Same as the main project: Creative Commons Attribution-ShareAlike 4.0 International License

