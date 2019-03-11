# &#181; Rapid-Simulation API (URSA)

URSA is a lightweigth API for the rapid simulation of computing systems. The goal of the project is to provide an alternative to the cubersome, expensive, and buggy on-the-shelf tools. 

## An Overview on the project...

URSA comprises a discrete event simulator that enables the cycle-accurate simulation of hardware models. We describe such models using C++ language. For the sake of simplicity, no libraries other than those provided with your C++ compiler are required. The project is entirely object-oriented, well organized, and properly documented. 

![Components of URSA and their interaction.](https://raw.githubusercontent.com/andersondomingues/ursa/stable/docs/URSA.png?raw=true)

- The simulation API provides the primitives for modeling hardware as C++ classes;
- Models are compiled into a single class library and serve as basis for different simulators;

## Project organisation

- ``/bin`` : compiled binaries and external libraries
- ``/docs`` : contains a tutorial and images used in this MD file
- ``/logs`` : output from the hardware models, as well as other implementation-specific outputs
- ``/models`` : general purpose hardware models
- ``/platforms`` : platform-specific hardware models (mostly top-level modules)
- ``/simulator`` : URSA's core
- ``/software`` : software to be deployed to emulated platforms
- ``/tools`` : several scripts and helpers

## Project Status

- We have succefully emulated a fully-functional MPSoC platform comprising of a mesh-based NoC architecture interconnecting up to 16x16 processing elements (256 cores). We currently support only the ORCA platform (see ``/platform/orca-generic`` and ``/models`` folders), although other platforms can be emulated as well.

## Project Roadmap

- Develop software components to provide self-adaptive traits to ORCA platform. We rely on a fork of the [HellfireOS](https://github.com/andersondomingues/hellfireos) operating system. 
- Develop benchmarks, so that we can accurately measure the performance of the simulation
- Study other models to speed-up the simulation
- Debugging and visualization tools

## Getting Started

- A tutorial is included at ``docs/URSA_Sulphane - The Lazy Manual``. 

## Third-Party Work

- HF-RISCV. The hf-riscv core is maintained by Sergio Johann (sjohann81). More information on his work can be found at [his repository](https://github.com/sjohann81). Also, our model of hf-riscv core is very based on the one provided by him. 

- HEMPS (and HERMES). The GAPH group maintains the HEMPS project. More information on their work can be found at [their website](http://www.inf.pucrs.br/hemps/getting_started.html). Provided network-on-chip router model is based on the RTL models available at [their repository](https://github.com/GaphGroup/hemps). 

- HELLFIREOS. We use [sjohann81's HellfireOS operating system](https://github.com/sjohann81) within the processing elements of the ORCA platform. 

## Licensing

See ``LICENSE.MD`` for details. 

## Contact

For now, I ([andersondomingues](https://github.com/andersondomingues)) am the only contributor to this project. Feel free to [mail me](mailto:ti.andersondomingues@gmail.com).
