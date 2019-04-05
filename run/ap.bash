#!/bin/bash

# Load the parameters onto the parameter server
rosparam load params.yaml mediation_layer

# Run the executable. Should be installed to the current directory.
../bin/exe/ap_blue
