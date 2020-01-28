# &#181; Rapid-Simulation API (URSA)

URSA is a lightweigth API for the rapid simulation of computing systems. The goal of the project is to provide an alternative to the cubersome, expensive, and buggy on-the-shelf tools. 

## An Overview on the project...

URSA comprises a discrete event simulator that enables the cycle-accurate simulation of hardware models. We describe such models using C++ language using URSA API. No libraries other than those provided with your C++ compiler are required. 

## Project organization

- ``/lib`` : stores the compiled library after the compilation process
- ``/docs`` : API documentation and tutorials
- ``/logs`` : output from the hardware models, as well as other implementation-specific outputs and debugging
- ``/models`` : general purpose hardware models (independent modules)
- ``/platforms`` : top-level modules for platform-specific hardware
- ``/core`` : URSA's simulation engine

## Project Status

- We have succefully emulated a fully-functional MPSoC platform comprising of a mesh-based NoC architecture interconnecting up to 16x16 processing elements (256 cores). 
- We provide generic models that can be used to make new platforms, including buffers and memory modules. See [URSA models repository](https://github.com/andersondomingues/ursa-models) for more information.
- Additional We provide models for the ORCA platform separatelly. See [ORCA models repository](https://github.com/andersondomingues/orca-models) for more information.

## Project Roadmap

- Development of benchmarks [URSA benchmark repository](https://github.com/andersondomingues/ursa-benchmark) 
- Study other models for the trade-off simulation speed-up vs. accuracy
- Debugging and visualization tools, including a GDB RSV implementation

## Getting Started (tutorial)

- See ``/docs/tutorial.md``

## Generating API Documentation

- Make sure you have installed ``doxygen`` and ``rviz`` to your system.
- In the root directory, type ``make documentation``. Documentation will be deployed to ``docs/doxygen`` folder.
- You can access docs by opening ``docs/doxygen/html/index.html`` in any web browser (e.g., ``firefox``).
- LaTeX documentation can be generate by  typeing ``make -C docs/doxygen/latex/``. The output file ``docs/doxygen/latex/refman.pdf`` contains the generated documentation. 

## Licensing

See ``LICENSE.MD`` for details. 

## Contact

For now, I ([andersondomingues](https://github.com/andersondomingues)) am the only contributor to this project. Feel free to [mail me](mailto:ti.andersondomingues@gmail.com).
