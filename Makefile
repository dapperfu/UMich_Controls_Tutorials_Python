# Config
VENV := _controls_venv
PROJ := python-controls
# Environments to setup for this project
# Available options: host python arduino mpc57xx renesas pi matlab simulink
ENVS:=python git

## make_sandwich includes
include make_sandwich/env.mk
