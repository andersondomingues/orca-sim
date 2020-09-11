# ORCA-SIM

[![Documentation](https://codedocs.xyz/andersondomingues/orca-sim.svg)](https://codedocs.xyz/andersondomingues/orca-sim/) ![C++ build](https://github.com/andersondomingues/orca-sim/workflows/C++%20build/badge.svg) ![cpplint](https://github.com/andersondomingues/orca-sim/workflows/cpplint/badge.svg) [![CodeFactor](https://www.codefactor.io/repository/github/andersondomingues/orca-sim/badge)](https://www.codefactor.io/repository/github/andersondomingues/orca-sim)

ORCA-SIM is a framework for generating simulation tools. Generated simulators rely on event-driven queues to organize and run hardware events, which could be either cycles (for a cycle-accurate simualtion) or instruction (for an instruction-accurate simulation). 

## Features 
- No external library or runtime required
- Hardware is modeled using included C++ classes, no special syntax or language extension
- Runs in any unix-based system that runs GCC (tested in a variety of Ubuntu and Debian systems)
- Supports remote debugging with GDB via UDP
- Trade-off speed-accuracy can be worked out in hardware models, real-time simulation is achievable
- Peripheral access is supported via memory-mapped I/O

## Requirements
- GNU's Make 
- GNU's GCC, recommended the most recent stable version
- Cross-compilers and toolchains for the target platform (for Risc-V, see https://www.sifive.com/boards)

## Included hardware models 
- Processor model for emulating the HF-RiscV processor (supports both cycle- and instruction-accurate simulations)
- A 5-port router model for the Hermes network-on-chip
- Memory and FIFO buffers
- NoC-to-UDP virtual adapter

## Included example platforms
- *orca-mpsoc*, a manycore platform based on HF-RiscV processor and Hermes network-on-chip (https://github.com/andersondomingues/orca-mpsoc)
- *single-core*, a platform based on the HF-RiscV processor (Risc-V ISA, RV32im, 32-bit, no cache support).

## Software
- A set of application, operating system, and libraries to run on the example platforms is available at https://github.com/andersondomingues/orca-software-tools

## Quick Start
1) Clone this repository. Use ``git clone https://github.com/andersondomingues/orca-sim``
2) Edit ``Configuration.mk`` and select the desired platform by changin the ``ORCA_PLATFORM`` variable
2) Go to the root folder and type ``make`` in the terminal (requires GCC version >= 7.x and Make)
3) Run the generated tool as ``./bin/single-core.exe <software-image>``, where *software-image* is the binary compiled for the target architecture. Both examples use the HF-RiscV processor (see [Third Part Work]).

## Project Organization
- ``/bin`` : compiled binaries and compilation artifacts
- ``/docs`` : tutorials and guidance resources
- ``/logs`` : output logs
- ``/models`` : hardware models (processor core, memory cores, etc.)
- ``/platforms`` : top-level platform models 
- ``/ursa`` : simultion engine
- ``/gdbrsp``: a standalone gdbserver implementation
- ``Makefile`` : compilation script for Make tool
- ``Configuration.mk`` : project-wide configuration

## Project Status

- We have succefully simulated a fully-functional MPSoC platform comprising of a mesh-based NoC architecture interconnecting up to 16x16 processing elements (256 cores, table test).
- We have extended the framework to communicate with ROS, to run robotics application in the simulated platforms. See http://dx.doi.org/10.1109/LARS-SBR-WRE48964.2019.00043
- We added support for energy estimation via hardware characterization for the included models. Work to appear in ISCAS 2020.  

## Project Roadmap

Current work can be seen in ``Projects`` tab at the top of this page. An informal list of things that we are likely to work on in the next months is provided below.

- Automatically generate and export Doxygen documentation to a github of the project, programmatically.
- Provide hardware models as static libraries (enable model reuse for other projects)
- A website containing technical and non-technical information on the project.
- Benchmarks! 
- Visualization tools (including, NoC, Task Schedule, memory and other inspections)
- Decent tutorials 

## Generating API Documentation

- Make sure you have installed ``doxygen`` and ``graphviz`` to your system.
- In the root directory, type ``make documentation``. Documentation will be deployed to ``docs/doxygen`` folder.
- You can access docs by opening ``docs/doxygen/html/index.html`` in any web browser (e.g., ``firefox``).
- LaTeX documentation can be generate by  typeing ``make -C docs/doxygen/latex/``. The output file ``docs/doxygen/latex/refman.pdf`` contains the generated documentation. Please note that tabu.sty may be buggy. In that case, replace the generated file by one provided in CTAN's page (https://ctan.org/pkg/tabu?lang=en).

## Licensing

This is free software! See ``LICENSE.MD`` for details. 

## Contact

Feel free to contact me ([andersondomingues](https://github.com/andersondomingues)), the maintainer of this project: mailto:ti.andersondomingues@gmail.com.

## Third-Party Work and Acknowledgement 

- HF-RISCV. The hf-riscv core is maintained by Sergio Johann (sjohann81). More information on his work can be found at [his repository](https://github.com/sjohann81). I would like to thank Mr. Johann for the time spent explaining me the depths of the HF-RiscV architecture.

- HERMES. The GAPH group maintains the HERMES network-on-chip. More information on their work can be found at [their website](http://www.inf.pucrs.br/hemps/getting_started.html). Provided network-on-chip router model is based on the RTL models available at [their repository](https://github.com/GaphGroup/hemps). I would like to thank the GAPH group for giving me so many insights on Hermes' architecture. 

- I would to thank Mr. Alexandre Amory (https://github.com/amamory) for the advising during the development of this work. I also thank for the fancy compilation scripts, they rock!
