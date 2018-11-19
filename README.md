# &#181; Rapid-Simulation API (URSA)

URSA is a lightweigth API for the rapid simulation of computing systems. The goal of the project is to provide an alternative to the cubersome, expensive, and buggy on-the-shelf tools. 

## An Overview on the project...

URSA comprises a discrete event simulator that enables the cycle-accurate simulation of hardware models. We describe such models using C++ language. For the sake of simplicity, no libraries other than those provided with your C++ compiler are required. The project is entirely object-oriented, well organized, and properly documented. 

![Components of URSA and their interaction.](https://raw.githubusercontent.com/andersondomingues/ursa/stable/docs/URSA.png?raw=true)

## Getting Started

- First of all, make sure you have make and gcc installed in your system. URSA is being developed using ``GNU Make 4.1``, ``GCC version 6.3.0``. We run it on a ``Debian 6.3.0-18+deb9u1`` machine, but it should work with other distros as well.

- Examples of platforms reside under ``platforms`` folder. Select a platform to run and type ``make clean; make`` from the inside the platform directory. For instance, you can navigate to sulphane-generic plataform dir using ``cd <path_to_ursa>/platforms/sulphane-generic``. Once there, type ``make clean; make`` to build and run a simulator for the platform. The results of the simulation will be written to ``<path_to_ursa/platforms/sulphane-generic/logs`` folder. To stop the simulation, input ``CRTL + c`` to your terminal.

## Project Status and Roadmap

- We just finished sulphane-generic platform for good. As next step, we hope to deliver one or more of the following features. Sugestions are welcome as well. 

    - Distributed and parallel simulation (because it is faster than single-core simulation)
    - Functional, instruction accurate models (to gain simulation time and to speed-up software development for simulated platforms)
    - Debugging and visualization tools 
    - Drive detection
    - Breakpoints and snapshots
    - Simulation history, rollback and forward of simulation steps
    - More hardware models, including a file-based memory module

## Provided Models

In-House models

- NETIF. A simple network interface that can be used to send/receive packets from/to an on-chip network. 

- MEMORY. An memory-based memory module, that is, we store data from the simulated platform in host machine's memory. No file on disk is used.

Some of the provided models comes from existing hardware. Documentation for these hardware is available at providers' website or repository. 

- HF-RISCV. The hf-riscv core is maintained by Sergio Johann (sjohann81). More information on his work can be found at [his repository](https://github.com/sjohann81). Also, our model of hf-riscv core is very based on the one provided by him. 

- HEMPS (and HERMES). The GAPH group maintains the HEMPS project. More information on their work can be found at [their website](http://www.inf.pucrs.br/hemps/getting_started.html). Provided network-on-chip router model is based on the RTL models available at [their repository](https://github.com/GaphGroup/hemps). 

## Licensing

See ``LICENSE.MD`` for details. 

## Contact

For now, I ([andersondomingues](https://github.com/andersondomingues)) am the only contributor to this project. Feel free to [mail me](mailto:ti.andersondomingues@gmail.com).
