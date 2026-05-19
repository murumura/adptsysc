# ADPT Command-Line Options

This document summarizes the command-line options supported by `adptsysc`.

Basic form:

```bash
./build/bin/adptsysc [options] file...
```

---

## General Options

### `--help`

Show usage information and exit.

Example:

```bash
./build/bin/adptsysc --help
```

---

### `--verbose`

Enable detailed diagnostic output.

Example:

```bash
./build/bin/adptsysc --verbose
```

---

### `--directory=<dir>`, `-C <dir>`

Change working directory to `<dir>` before running.

Examples:

```bash
./build/bin/adptsysc --directory=work
```

```bash
./build/bin/adptsysc -C work
```

---

### `--chroot=<dir>`

Treat `<dir>` as the virtual root directory when resolving paths.

Example:

```bash
./build/bin/adptsysc --chroot=/tmp/adpt_root
```

---

## Color & Diagnostics

### `--color-diagnostics`

Enable automatic colored diagnostics when stderr is a terminal.

Equivalent to:

```bash
--color-diagnostics=auto
```

---

### `--color-diagnostics=auto`

Use colored diagnostics only when stderr is a terminal.

Example:

```bash
./build/bin/adptsysc --color-diagnostics=auto
```

---

### `--color-diagnostics=always`

Always enable colored diagnostics.

Example:

```bash
./build/bin/adptsysc --color-diagnostics=always
```

---

### `--color-diagnostics=never`

Disable colored diagnostics.

Example:

```bash
./build/bin/adptsysc --color-diagnostics=never
```

---

## Execution & Threading

### `--quick-exit`

Use quick-exit behavior for faster shutdown.

Example:

```bash
./build/bin/adptsysc --quick-exit
```

---

### `--no-quick-exit`

Disable quick-exit behavior and use normal C++ teardown.

Example:

```bash
./build/bin/adptsysc --no-quick-exit
```

---

### `--thread-count=<n>`

Set the number of worker threads to `<n>`.

Example:

```bash
./build/bin/adptsysc --thread-count=4
```

---

### `--threads`

Enable multi-threading with the default thread count.

Example:

```bash
./build/bin/adptsysc --threads
```

---

### `--threads=<n>`

Set the number of worker threads to `<n>`.

Example:

```bash
./build/bin/adptsysc --threads=4
```

---

### `--no-threads`

Force single-threaded execution.

Example:

```bash
./build/bin/adptsysc --no-threads
```

---

## Input / Output Options

### `-o <file>`, `--output=<file>`

Set output filename.

Examples:

```bash
./build/bin/adptsysc -o out.bin
```

```bash
./build/bin/adptsysc --output=out.bin
```

---

### `--text-output=<file>`

Set text output filename for memory dumps or text-form output.

Example:

```bash
./build/bin/adptsysc --text-output=out.txt
```

---

### `--dependency-file=<file>`

Write Makefile-style dependency rules to `<file>`.

Example:

```bash
./build/bin/adptsysc --dependency-file=deps.d
```

---

### `--out-shared`

Allow shared output behavior.

Example:

```bash
./build/bin/adptsysc --out-shared
```

---

## File I/O Options

### `--load-file=<file>`

Load memory contents from a binary file.

Example:

```bash
./build/bin/adptsysc --load-file=init.bin
```

---

### `--textload-file=<file>`

Load memory contents from a text-format file.

Example:

```bash
./build/bin/adptsysc --textload-file=init.txt
```

---

### `--load-offset=<offset>`

Load file contents starting at byte offset `<offset>`.

Examples:

```bash
./build/bin/adptsysc --load-offset=256
```

```bash
./build/bin/adptsysc --load-offset=0x100
```

---

### `--oformat=<binary|hex>`

Set memory dump output format.

Supported values:

- `binary`
- `hex`

Examples:

```bash
./build/bin/adptsysc --oformat=binary
```

```bash
./build/bin/adptsysc --oformat=hex
```

---

## Target / Architecture Options

### `-e <target>`, `--emulation=<target>`

Select emulation target.

Supported targets:

| Target | Description |
|---|---|
| `syscmem` | SystemC memory model |
| `r2sdf_fft_tlm` | Radix-2 SDF FFT/IFFT TLM model |

Examples:

```bash
./build/bin/adptsysc -e syscmem
```

```bash
./build/bin/adptsysc --emulation=r2sdf_fft_tlm
```

---

## FFT / IFFT Options

### `--ifft`

Run the R2SDF FFT TLM testbench in IFFT mode.

This option is mainly used with:

```bash
-e r2sdf_fft_tlm --run-testbench
```

Example:

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm --ifft --verbose
```

Behavior:

```text
Without --ifft:
  test_fft_cplx()
  test_fft_real()

With --ifft:
  test_ifft_cplx()
```

The same `r2sdf_fft_tlm` architecture is used for both FFT and IFFT.  
IFFT is selected as a runtime/testbench mode, not as a separate architecture.

---

## Filter & Algorithm Options

### `--behavior-filter`

Enable behavioral filter model.

Example:

```bash
./build/bin/adptsysc --behavior-filter
```

---

### `--no-behavior-filter`

Disable behavioral filter model.

Example:

```bash
./build/bin/adptsysc --no-behavior-filter
```

---

### `--fixedpoint-eval`

Enable fixed-point evaluation.

Example:

```bash
./build/bin/adptsysc --fixedpoint-eval
```

---

### `--no-fixedpoint-eval`

Disable fixed-point evaluation.

Example:

```bash
./build/bin/adptsysc --no-fixedpoint-eval
```

---

### `--polyphase`

Enable polyphase filter mode.

Example:

```bash
./build/bin/adptsysc --polyphase
```

---

### `--no-polyphase`

Disable polyphase filter mode.

Example:

```bash
./build/bin/adptsysc --no-polyphase
```

---

## Simulation & Debug Options

### `--run-testbench`

Run the SystemC testbench for the selected emulation target.

Example:

```bash
./build/bin/adptsysc --run-testbench -e syscmem
```

---

### `--trace`

Enable trace generation.

Example:

```bash
./build/bin/adptsysc --trace
```

---

## Memory Timing Options

### `--mem-read-delay-cycles=<n>`

Set memory read delay in cycles.

`<n>` must be non-negative.

Example:

```bash
./build/bin/adptsysc --mem-read-delay-cycles=1
```

---

### `--mem-write-delay-cycles=<n>`

Set memory write delay in cycles.

`<n>` must be non-negative.

Example:

```bash
./build/bin/adptsysc --mem-write-delay-cycles=1
```

---

## Examples

### Show help

```bash
./build/bin/adptsysc --help
```

---

### Run SystemC memory testbench

```bash
./build/bin/adptsysc --run-testbench -e syscmem --verbose
```

---

### Run R2SDF FFT TLM testbench

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm --verbose
```

---

### Run R2SDF IFFT TLM testbench

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm --ifft --verbose
```

---

### Run R2SDF IFFT TLM testbench with memory delays

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm --ifft --verbose \
  --mem-write-delay-cycles=1 \
  --mem-read-delay-cycles=1
```

---

### Run R2SDF FFT TLM testbench with trace enabled

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm --trace --verbose
```

---

### Load binary file and dump output in hex format

```bash
./build/bin/adptsysc \
  --load-file=init.bin \
  --load-offset=0x100 \
  --oformat=hex
```

---

### Enable polyphase and fixed-point evaluation

```bash
./build/bin/adptsysc --polyphase --fixedpoint-eval
```

---

## Notes

### FFT and IFFT share the same architecture

The R2SDF TLM model uses one architecture name:

```text
r2sdf_fft_tlm
```

Both FFT and IFFT are handled inside the same model.

FFT mode:

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm
```

IFFT mode:

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm --ifft
```

---

### IFFT scaling convention

The current R2SDF FFT TLM architecture uses:

```cpp
static constexpr bool scale_each_stage = false;
```

Therefore IFFT uses final output scaling:

```text
IFFT output = inverse transform result / N
```

The FFT path remains unscaled.

---

### Memory delay options

The memory delay options only affect the timing model:

```bash
--mem-read-delay-cycles=<n>
--mem-write-delay-cycles=<n>
```

They do not change FFT/IFFT numerical behavior.
