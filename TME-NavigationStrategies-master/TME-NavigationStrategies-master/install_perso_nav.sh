#!/bin/sh

# Alex Coninx & Benoît Girard
# ISIR - Sorbonne Universite / CNRS
# 27/10/2020

BASE_DIR=$PWD

IPYNB_PATCH_FILE="ipynb-utils.patch"

mkdir -p $HOME/src

# 1) Install pybind11
echo
echo "====================================="
echo "===== (1/4) Installing pybind11 ====="
echo "====================================="
echo
cd $HOME/src
git clone https://github.com/pybind/pybind11.git
cd pybind11
# Install the pybind11 python module
pip3 install .
# Where we can find pybind11 (especially its includes)
PYBIND11_DIR=$HOME/src/pybind11


# 2) Install and patch fastsim
echo
echo "===================================================="
echo "===== (2/4) Patching and installing libfastsim ====="
echo "===================================================="
echo
cd $HOME/src
git clone https://github.com/jbmouret/libfastsim.git
# We need to clone the pyfastsim repository now to get the patch
git clone https://github.com/alexendy/pyfastsim.git
cd libfastsim
# Patch libfastsim
patch -p1 < ../pyfastsim/fastsim-boost2std-fixdisplay.patch
# Build and install
python2.7 ./waf configure --prefix=./install
python2.7 ./waf build
python2.7 ./waf install
# Where we installed fastsim
FASTSIM_DIR=$HOME/src/libfastsim/install

# 3) Install pyfastsim
echo
echo "======================================"
echo "===== (3/4) Installing pyfastsim ====="
echo "======================================"
echo
cd $HOME/src/pyfastsim
CPPFLAGS="-I${PYBIND11_DIR}/include -I${FASTSIM_DIR}/include" LDFLAGS="-L${FASTSIM_DIR}/lib" pip3 install --user .

# 4) install the exercises
echo
echo "=================================================="
echo "===== (4/4) Getting the navigation exercises ====="
echo "=================================================="
echo
cd $HOME/src
git clone https://github.com/benoit-girard/TME-NavigationStrategies.git
cd TME-NavigationStrategies
echo
echo
echo "********************************************************************************"
echo "Navigation exercises are installed in: ${PWD}"
echo "Go there and test the installation with, for example : % python3 wallFollower.py"
echo "********************************************************************************"
