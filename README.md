# adptsysc

`adptsysc` is a C++20 / SystemC-based framework for adaptive filter algorithm development, targeting digital signal processing (DSP) applications with an emphasis on:

- VLSI-oriented modeling
- hardware/software co-design
- architecture-aware template instantiation
- reusable simulation and algorithm exploration infrastructure

The project is intended for experimenting with adaptive filtering algorithms and related DSP building blocks in a SystemC-friendly environment.

---

## Features

- SystemC-based architectural modeling
- C++20 template-based design flow
- target-specific instantiation for different architectures
- support for reusable DSP modules
- configurable build flow using CMake
- optional SystemC-AMS integration
- unit test support via GoogleTest

---

## Repository Layout

```text
src/adptsysc/
  arch.hh                # architecture trait definitions
  adptsysc-main.cc       # runtime dispatch
  sysc-mem-tlm.hh/.cc        # memory-related template module
  sysc-r2sdffft-tlm.hh/.cc   # R2SDF FFT template module
  sysc-cmplxmul-tlm.hh       # complex multiplier
  sysc-shiftreg-tlm.hh       # shift register
  design-lib.hh          # DSP/helper utilities
  object.hh              # base object interface


Requirements

Typical dependencies include:

C++20 compiler
CMake >= 3.27
SystemC
optional: SystemC-AMS
optional: GoogleTest
optional: Eigen
Docker, if using the containerized flow
Build in Docker Container

If you build inside a Docker container and the repository is mounted from the host, Git may reject the repo due to ownership mismatch.

Before building, run:

git config --global --add safe.directory /sysc/.git

This avoids errors like:

fatal: detected dubious ownership in repository at '/sysc/.git'
To add an exception for this directory, call:

    git config --global --add safe.directory /sysc/.git
fatal: Could not read from remote repository.

Please make sure you have the correct access rights