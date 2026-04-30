# CMake Template-Instantiation Trick

![CMake template instantiation workflow](./cmaketemplate.png)

## Goal

This project uses many template-based `.cc` sources whose implementation depends on an architecture type such as `SyscMemArch` or `R2SdfFFTTLMArch`.

If we compile every template source for every target, we will eventually hit invalid source × target combinations.  
A typical example is:

- `sysc-r2sdffft.cc` should only be compiled for FFT-related targets
- it should **not** be instantiated for `SyscMemArch`

The CMake trick in this project is:

1. register a template source once
2. expand it into one tiny generated wrapper `.cc` per valid target
3. compile only the valid source × target combinations

This keeps the build scalable and avoids trait-mismatch errors.

---

## Core Idea

Instead of compiling a template source file directly, CMake first generates a small wrapper file like this:

```cpp
#define ADPT_TARGET R2SdfFFTTLMArch
#define ADPT_ENABLE_R2SDF 1
#include "/sysc/src/adptsysc/sysc-r2sdffft.cc"
```


## How to add a new design in both CMake and source code

Take **LMS** as an example.

In this project, adding a new design usually means updating **three layers** together:

1. **architecture definition** in `arch.hh`
2. **enable the config** in `config.hh`
3. **runtime dispatch** in `adptsysc-main.cc`
4. **build-time registration** in `CMakeLists.txt`

---

## Important note about `config.h.in` and `config.hh`

In this project:

- `config.h.in` is the **template file maintained by developers**
- `config.hh` is the **generated file produced by CMake**

So the correct workflow is:

- edit **`config.h.in`** if the config template itself needs to change
- edit **`CMakeLists.txt`** to control which `HAVE_*` macros are generated
- do **not** manually edit **`config.hh`** in normal development

### Correct mental model

```text
config.h.in  --(configure_file by CMake)-->  config.hh