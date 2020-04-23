# ORCA-SIM

ORCA-SIM is a simulation tool to simulate/emulate the ORCA-MPSoC many core. This tool run on top of URSA, an API for developing event-driven simulators. Although event-driven, ORCA-SIM is capable of delivering simulation results with cycle accuracy. More information on the individual projects can be find in the respective repositories.

- URSA: https://github.com/andersondomingues/ursa
- ORCA-MPSOC: https://github.com/andersondomingues/orca-mpsoc

Software that can be used with in both RTL and ORCA-SIM simulations can be found in the following repository.

- ORCA-SOFTWARE-TOOLS: https://github.com/andersondomingues/orca-software-tools

## Quick Start

1) Clone this repository. Use ``git clone https://github.com/andersondomingues/orca-sim --recurse-submodules``.
2) Go to the root folder and type ``make`` in the terminal (requires GCC version >= 7.x and Make)
3) Run the simulator as ``./bin/single-core.exe <software-image>``, where softwa-image is the binary compiled for the HF-RiscV architecture.

## Project organization

- ``/bin`` : compiled binaries and compilation artifacts
- ``/docs`` : tutorials and guidance resources
- ``/logs`` : output logs for each individual processor core
- ``/models`` : hardware models (processor core, memory cores, etc.)
- ``/platforms`` : top-level platform models (for both many-core and single-core platforms)
- ``/simulator`` : URSA's core
- ``Makefile`` : Make's script. Do not modifiy it unless you know make language well.
- ``Configuration.mk`` : Change this script to reflect the intended project configuration (comments on the parameters are inside the file)

## Project Status

- We have succefully simulated a fully-functional MPSoC platform comprising of a mesh-based NoC architecture interconnecting up to 16x16 processing elements (256 cores). The platform contains an ORCA-MPSoC (links for external projects are provided at the top of this readme).
## Project Roadmap

Things that we are likely to work on in the next months:

- Automatically generate and export Doxygen documentation to a github of the project, programmatically.
- Provide hardware models as static libraries
- Remote debugging using GDB and GDB-server
- Remove eliminate pthreads from models by using asynchonous calls
- A website containing technical and non-technical information on the project.i
- Benchmarks! Probably something based on PARSEC but for NORMA archs.
- Replace multitail with some visualization tools (including, NoC, Task Schedule, memory and other inspections)
- Deal with endianess (enables heterogeneous multicore)
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
