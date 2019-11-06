# &#181; Rapid-Simulation API (URSA), and ORCA MPSoC

URSA is a lightweigth API for the rapid simulation of computing systems. The goal of the project is to provide an alternative to the cubersome, expensive, and buggy on-the-shelf tools. 

ORCA MPSoC is manycore processor based on RiscV architecture. 

## An Overview on the project(s)...

URSA comprises a discrete event simulator that enables the cycle-accurate simulation of hardware models. We describe such models using C++ language. For the sake of simplicity, no libraries other than those provided with your C++ compiler are required. The project is entirely object-oriented, well organized, and properly documented. 

![Components of URSA and their interaction.](https://raw.githubusercontent.com/andersondomingues/ursa/stable/docs/URSA.png?raw=true)

- The simulation API provides the primitives for modeling hardware as C++ classes;
- Models are compiled into a single class library and serve as basis for different simulators;

## Project organisation

- ``/bin`` : compiled binaries and external libraries
- ``/docs`` : contains a tutorial and images used in this MD file, also serving as output folder for doxygen
- ``/logs`` : output from the hardware models, as well as other implementation-specific outputs and debugging
- ``/models`` : general purpose hardware models (independent modules)
- ``/platforms`` : platform-specific hardware models (top-level modules)
- ``/simulator`` : URSA's core
- ``/software`` : software to be deployed to emulated platforms
- ``/tools`` : several scripts and helpers

## Project Status

- We have succefully emulated a fully-functional MPSoC platform comprising of a mesh-based NoC architecture interconnecting up to 16x16 processing elements (256 cores). The platform is ORCA MPSoC (see https://github.com/andersondomingues/orca-mpsoc). URSA and ORCA started as the same project, and as the project progressed we decided to split them into two different projects, one for URSA, which is still a simulation API, and another one for ORCA, a fully-functional MPSoC platform. We are still cleaning the repository and setting things up. In meanwhile, I apologize for any "orbitating artifact".

## Project Roadmap

Things we are likely to work on in the next months:

- Automatically generate and export Doxygen documentation to a github of the project, programmatically.
- A VHDL version of the platform. See (https://github.com/andersondomingues/orca-mpsoc)
- Provide an API to use for energy evaluation of applications
- Unit tests on software (mainly for HellfireOS, see (https://github.com/andersondomingues/hellfireos)
- Provide hardware models as static libraries
- Rework task mapping and wiring (code can be cumbersome when dealing with a large number of signals)
- Provide some IDE-specific projects (hopefully they won't inject any dirt in the project)
- Remote debugging using GDB
- Remove eliminate pthreads from models
- Support for C++ apps 
- Stub code generator 
- A website containing technical and non-technical information on the project.
- Benchmarks! Probably something based on PARSEC but for NORMA archs.
- Memory hierarchy and some memory protection
- Update HellfireOS to match a more stable and recent branch (see (https://github.com/sjohann81/hellfireos)
- Replace multitail with some visualization tools (including, NoC, Task Schedule, memory and other inspections)
- Provide a robust infrastructure so that anyone can create their own models
- Automatic model generation (based on some process algebra)
- Deal with endianess (enables heterogeneous multicore)
- DOOM (see https://github.com/lcq2/risc-666)
- Decent tutorials 

## Generating API Docuemtnation

- Make sure you have installed ``doxygen`` and ``rviz`` to your system.
- In the root directory, type ``make documentation``. Documentation will be deployed to ``docs/doxygen`` folder.
- You can access docs by opening ``docs/doxygen/html/index.html`` in any web browser (e.g., ``firefox``).
- LaTeX documentation can be generate by  typeing ``make -C docs/doxygen/latex/``. The output file ``docs/doxygen/latex/refman.pdf`` contains the generated documentation. 

## Third-Party Work

- HF-RISCV. The hf-riscv core is maintained by Sergio Johann (sjohann81). More information on his work can be found at [his repository](https://github.com/sjohann81). Also, our model of hf-riscv core is very based on the one provided by him. 

- HEMPS (and HERMES). The GAPH group maintains the HEMPS project. More information on their work can be found at [their website](http://www.inf.pucrs.br/hemps/getting_started.html). Provided network-on-chip router model is based on the RTL models available at [their repository](https://github.com/GaphGroup/hemps). 

- HELLFIREOS. We use [sjohann81's HellfireOS operating system](https://github.com/sjohann81) within the processing tiles of the ORCA platform. 

## Licensing

See ``LICENSE.MD`` for details. 

## Contact

Feel free to contact me ([andersondomingues](https://github.com/andersondomingues)), the maintainer of this project: mailto:ti.andersondomingues@gmail.com.
