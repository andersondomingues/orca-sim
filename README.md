# &#181; Rapid-Simulation API (URSA)

URSA is a lightweigth API for the rapid simulation of computing systems. The goal of the project is to provide an alternative to the cubersome, expensive, and buggy on-the-shelf tools. 

## An Overview on the project...

URSA comprises a discrete event simulator that enables the cycle-accurate simulation of hardware models. We describe such models using C++ language. For the sake of simplicity, no libraries other than those provided with your C++ compiler are required. The project is entirely object-oriented, well organized, and properly documented. Although a very young project, URSA is a nice alternative to personal, college and industrial projects.

![Components of URSA and their interaction.](https://raw.githubusercontent.com/andersondomingues/ursa/stable/docs/URSA.png?raw=true)

## Getting Started

- First of all, make sure you have make and gcc installed in your system. URSA is being developed using ``GNU Make 4.1``, ``GCC version 6.3.0``,  in a ``Debian 6.3.0-18+deb9u1`` machine and is guaranteed to run using these software. However, you can try another compiler or building system at your own risk. 

- Examples of platforms reside under ``platforms`` folder. Select a platform to run and type ``make clean; make`` from the inside the platform directory. For instance, type ``cd ursa/platforms/sulphane-generic; make clean; make`` and results for your simulation will be writen to ``ursa/platforms/sulphane-generic/logs`` folder.

- Note that in case you are simulating any CPU other than the one in your system, additional cross-compilers may be necessary depending on the target architecture.

## Project Status and Roadmap

- We are currently working on the Sulphane platform, which is the example provided within URSA. We hope to finish the platform as soon as the end of November/2018. However, URSA can be considered as stable and the development of Sulphane should not bring any modifications to URSA source code.

- Here are a tiny list of things to be implemented in the near future
    - Distributed and parallel simulation (because it is faster than singlecore)
    - Functional models (to gain simulation time and to speed-up software development for simulated platforms)
    - Debugging and visualization tools 
    - Drive detection
    - Breakpoints and snapshots
    - Simulation history, rollback and forward of simulation steps

## Provided Models

Some of the provided models comes from existing hardware. Documentation for these hardware is available at providers' website or repository. 

- HF-RISCV. The hf-riscv core is maintained by Sergio Johann (sjohann81). More information on his work can be found at [his repository](https://github.com/sjohann81). Also, our model of hf-riscv core is very based on the one provided by him. 

- HEMPS (and HERMES). The GAPH group maintains the HEMPS project. More information on their work can be found at [their website](http://www.inf.pucrs.br/hemps/getting_started.html). Provided network-on-chip router model is based on the RTL models available at [their repository](https://github.com/GaphGroup/hemps). 

## Licensing

See ``LICENSE.MD`` for details. 

## Contact

For now, I ([andersondomingues](https://github.com/andersondomingues)) am the only contributor to this project. Feel free to [mail me](mailto:ti.andersondomingues@gmail.com).
