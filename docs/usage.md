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

### `--no-color-diagnostics`

Disable colored diagnostics.

Equivalent to:

```bash
--color-diagnostics=never
```

---

### `--fatal-warnings`

Treat warnings as errors.

Example:

```bash
./build/bin/adptsysc --fatal-warnings
```

---

### `--no-fatal-warnings`

Do not treat warnings as errors.

Example:

```bash
./build/bin/adptsysc --no-fatal-warnings
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

Set binary/raw output filename.

Examples:

```bash
./build/bin/adptsysc -o out.bin
```

```bash
./build/bin/adptsysc --output=out.bin
```

---

### `--text-output=<file>`

Set text output filename for memory dumps or SV-readable text output.

Example:

```bash
./build/bin/adptsysc --text-output=out.mem
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

Load memory contents from a raw binary file.

Example:

```bash
./build/bin/adptsysc --load-file=init.bin
```

---

### `--text-load-file=<file>`

Load memory contents from a text-format file.

Example:

```bash
./build/bin/adptsysc --text-load-file=init.mem
```

Older aliases may also be supported depending on the parser:

```bash
--textload-file=<file>
--text-loadfile=<file>
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

Set memory dump or text-load interpretation format.

Supported values:

- `binary`
- `hex`

For raw binary output, `binary` means raw bytes.

For text output with SystemC fixed-point export:

- `hex` writes `$readmemh`-compatible words
- `binary` writes `$readmemb`-compatible words

Examples:

```bash
./build/bin/adptsysc --oformat=hex
```

```bash
./build/bin/adptsysc --oformat=binary
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

### `--fft`

Run the R2SDF FFT TLM testbench in FFT mode.

This is the default mode when `--ifft` is not specified.

Example:

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm --fft --verbose
```

---

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
  verify_cmplxfft()
  verify_realfft()

With --ifft:
  verify_cmplxifft()
```

The same `r2sdf_fft_tlm` architecture is used for both FFT and IFFT.  
IFFT is selected as a runtime/testbench mode, not as a separate architecture.

---

## Numeric Options

### `--fixedpoint-eval`

Enable fixed-point-style evaluation.

When enabled, testbench input and golden output are quantized through the architecture fixed-point type, for example:

```cpp
using Fxpt_T = sc_dt::sc_fixed<16, 12>;
```

Example:

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm --fixedpoint-eval --verbose
```

---

### `--fixedpoint-tol=<value>`

Set comparison tolerance when fixed-point evaluation is enabled.

Example:

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm \
  --fixedpoint-eval \
  --fixedpoint-tol=1e-2 \
  --verbose
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

Enable internal trace/debug log generation.

Example:

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm --trace --verbose
```

---

### `--signal-trace`

Enable numerical signal trace output.

Example:

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm --signal-trace --verbose
```

---

### `--signal-trace-file=<file>`

Write numerical signal trace output to `<file>`.

Example:

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm \
  --signal-trace-file=r2sdf_fft.csv \
  --verbose
```

---

### `--waveform`

Enable waveform dumping.

Example:

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm --waveform --verbose
```

---

### `--waveform-file=<file>`

Write waveform output to `<file>`.

Example:

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm \
  --waveform-file=r2sdf_fft.vcd \
  --verbose
```

---

### `--sv-trace-dir=<dir>`

Write SV/VCS-loadable trace files to `<dir>`.

This is intended for SystemVerilog regression comparison using `$readmemh` or `$readmemb`.

Example:

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm \
  --sv-trace-dir=trace/r2sdf_fft \
  --verbose
```

Expected output style:

```text
trace/r2sdf_fft/
  input_re.mem
  input_im.mem
  golden_re.mem
  golden_im.mem
  dut_re.mem
  dut_im.mem
  meta.json
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

### Run R2SDF FFT TLM testbench with fixed-point-style comparison

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm \
  --fixedpoint-eval \
  --fixedpoint-tol=1e-2 \
  --verbose
```

---

### Run R2SDF IFFT TLM testbench with fixed-point-style comparison

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm \
  --ifft \
  --fixedpoint-eval \
  --fixedpoint-tol=1e-2 \
  --verbose
```

---

### Run R2SDF IFFT TLM testbench with memory delays

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm --ifft --verbose \
  --mem-write-delay-cycles=1 \
  --mem-read-delay-cycles=1
```

---

### Run R2SDF FFT TLM testbench with internal trace enabled

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm --trace --verbose
```

---

### Run R2SDF FFT TLM testbench with CSV signal trace

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm \
  --signal-trace-file=r2sdf_fft.csv \
  --verbose
```

---

### Run R2SDF FFT TLM testbench with VCD waveform

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm \
  --waveform-file=r2sdf_fft.vcd \
  --verbose
```

---

### Run R2SDF FFT TLM testbench and export SV/VCS regression traces

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm \
  --sv-trace-dir=trace/r2sdf_fft \
  --verbose
```

---

### Run R2SDF IFFT TLM testbench with fixed-point compare, CSV, waveform, and SV trace

```bash
./build/bin/adptsysc --run-testbench -e r2sdf_fft_tlm \
  --ifft \
  --fixedpoint-eval \
  --fixedpoint-tol=1e-2 \
  --signal-trace-file=r2sdf_ifft.csv \
  --waveform-file=r2sdf_ifft.vcd \
  --sv-trace-dir=trace/r2sdf_ifft \
  --verbose
```

---

### Load raw binary memory at offset

```bash
./build/bin/adptsysc \
  --load-file=init.bin \
  --load-offset=0x100
```

---

### Save SystemC fixed-point memory as `$readmemh` text

```bash
./build/bin/adptsysc \
  -e syscmem \
  --text-output=fixed_dump.mem \
  --oformat=hex
```

SystemVerilog:

```systemverilog
$readmemh("fixed_dump.mem", mem);
```

---

### Save SystemC fixed-point memory as `$readmemb` text

```bash
./build/bin/adptsysc \
  -e syscmem \
  --text-output=fixed_dump.mem \
  --oformat=binary
```

SystemVerilog:

```systemverilog
$readmemb("fixed_dump.mem", mem);
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

### Fixed-point export convention

The architecture defines the fixed-point type and word format:

```cpp
using Fxpt_T = sc_dt::sc_fixed<16, 12>;

static constexpr int fx_word_bits = 16;
static constexpr int fx_integer_bits = 12;
static constexpr int fx_frac_bits = fx_word_bits - fx_integer_bits;
static constexpr bool fx_signed = true;
```

For SV/VCS text export:

```text
--oformat=hex
  writes hex words for $readmemh

--oformat=binary
  writes binary words for $readmemb
```

Do not use raw C++ object bytes for `sc_fixed` regression comparison.

---

### Memory delay options

The memory delay options only affect the timing model:

```bash
--mem-read-delay-cycles=<n>
--mem-write-delay-cycles=<n>
```

They do not change FFT/IFFT numerical behavior.