# &#181; Rapid-Simulation API (URSA)

URSA is a lightweigth API for the rapid simulation of computing systems. The goal of the project is to provide an alternative to the cubersome, expensive, and buggy on-the-shelf tools. 

## An Overview on the project...

URSA comprises a discrete event simulator that enables the cycle-accurate simulation of hardware models. We describe such models using C++ language. For the sake of simplicity, no libraries other than those provided with your C++ compiler are required. The project is entirely object-oriented, well organized, and properly documented. Although a very young project, URSA is a nice alternative to personal, college and industrial projects.

![Components of URSA and their interaction.](https://raw.githubusercontent.com/andersondomingues/ursa/stable/docs/URSA.png?raw=true)

## Getting Started

- First of all, make sure you have make and gcc installed in your system. URSA is being developed using ``GNU Make 4.1``, ``GCC version 6.3.0``,  in a ``Debian 6.3.0-18+deb9u1`` machine and is guaranteed to run using these software. However, you can try another compiler or building system at your own risk. 

- An example of platform can be found ``sdfdfsd`` We provide files for compiling Sulphane, is provided within URSA. The ``platforms`` directory. 

## Project Status and Roadmap

- We are currently working on integrating the HF-RISCV processor with peripherals from the HEMPS project onto a single platform. we hope to finish this milestone near the end of August of 2018. 

## Provided Models

Most of the available models mimic behavior from existing hardware. Documentation for this hardware is available at providers' website or repository. 

- HF-RISCV. The hf-riscv processor is maintained by Sergio Johann (sjohann81). More information on his work can be found at [his repository](https://github.com/sjohann81). Also, most of the provided model for the HF-RISCV processor comes from Johann's simulator, which can also be found in his repository.

- HEMPS (and HERMES). The GAPH group maintains the HEMPS project. More information on their work can be found at [their website](http://www.inf.pucrs.br/hemps/getting_started.html). Provided network-on-chip router and DMNI models are based on the hardware available in [their repository](https://github.com/GaphGroup/hemps). Note that HERMES is part of the HEMPS projects and should be available at the same repository.

## Licensing

See ``LICENSE.MD`` for details. 

## Contact

Any of the contributors of this repository. For now, I ([andersondomingues](https://github.com/andersondomingues)) am the only contributor to this project. Feel free to [mail me](mailto:ti.andersondomingues@gmail.com).
