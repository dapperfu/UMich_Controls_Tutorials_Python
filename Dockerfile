#FROM jupyter/base-notebook:lab
FROM jupyter/scipy-notebook:latest

USER root

RUN apt-get update \
    && apt-get install -yq --no-install-recommends graphviz git \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

USER $NB_USER

# Conda Install
RUN conda install --yes \
    -c conda-forge \
    slycot \
    control

# requirements.txt
COPY requirements.txt /tmp/
RUN pip install -r /tmp/requirements.txt
