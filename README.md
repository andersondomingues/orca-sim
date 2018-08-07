# URSA

Welcome to the repository of the URSA project. URSA is an environment for creating, building and simulating multiprocessed, computing systems platforms.

The following sections may serve you as a tour on the project. Note that the project is in constant evolution! So, feel free to contact us regarding the development of URSA.

![Components of URSA and their interaction.](https://github.com/andersondomingues/ursa/blob/master/web-client/figs/URSA.png?raw=true)

## The Anatomy of URSA

URSA project is made of smaller components, described as following.

- MODELS: Models are functional implementations of hardware's behavior, that is, its C++ code that emulates the hardware. Still, it is cycle-accurate and handy for fast-programming when targeting embedded platforms. Platforms are composed of several interconnected hardware models. Models are used to make platforms, which are then simulated by URSA's simulator. 

- GENERICS: Generic hardware is used to model parts of platforms do not need to be observed at runtime or which the abstraction does not require cycle precision. We provide two generic models: Buffer and Memory.

- PLATFORMS: Platforms are made of peripherals, busses, processors and other hardware. We are currently working on integrating both the HF-RISCV processor and peripherals from the HEMPS project onto a single platform. More information on HF-RISCV and HEMPS projects can be found in the [external contribution](#external-contribution) section. 

- SIMULATOR: The discrete event simulator component is the heart of simulation. It instantiates a queue in which events from hardware are scheduled and executed in-order. URSA provides cycle-precise simulation.

- WEB-CLIENT: Interactive monitoring interface that allows the visualization of models through a web browser. For instance, one may want to watch registers changing their values during systems' execution, just for fun. Also, it is possible to control the simulation through the interface by mapping IO to web controls, simulation the input of real data to the target platform.

## Requirements and Running an Example

- We tested URSA with ``GNU Make 4.1``, ``GCC version 6.3.0``,  and ``Debian 6.3.0-18+deb9u1``. Additional compilers may be required for compiling software for simulated CPUs. 

- Examples are available at ``platforms`` directory. 

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
