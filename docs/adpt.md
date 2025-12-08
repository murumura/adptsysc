# ADPT Command-Line Options

## General Options
- `--help`  
  Show usage information and exit.

- `--directory=<dir>`, `-C <dir>`  
  Change working directory to `<dir>` before doing anything.

- `--chroot=<dir>`  
  Treat `<dir>` as the virtual root directory when resolving paths.

- `--verbose`  
  Enable detailed diagnostic output.

---

## Color & Diagnostics
- `--color-diagnostics[=<auto|always|never>]`  
  Control ANSI-colored diagnostics. Default: `auto`.

- `--color-diagnostics`  
  Alias for `--color-diagnostics=auto`.

- `--no-color-diagnostics`  
  Alias for `--color-diagnostics=never`.

- `--fatal-warnings`  
  Treat warnings as errors.

- `--no-fatal-warnings`  
  Do not treat warnings as errors (default).

---

## Execution & Threading
- `--quick-exit`  
  Use `quick_exit()` for faster shutdown (default).

- `--no-quick-exit`  
  Disable `quick_exit()` and use normal C++ teardown.

- `--thread-count=<n>`  
  Force number of worker threads to `<n>`.

- `--threads=<n>`  
  Same as `--thread-count=<n>`.

- `--threads`  
  Enable multi-threading with default thread count.

- `--no-threads`  
  Force single-threaded execution.

---

## Input / Output Options
- `-o <file>`, `--output=<file>`  
  Set output filename.

- `--text-output=<file>`  
  Set text output filename for memory dumps.

- `--dependency-file=<file>`  
  Write Makefile-style dependency rules to `<file>`.

- `--out-shared`  
  Allow overwriting existing output files.

---

## File I/O Options
- `--load-file=<file>`  
  Load memory contents from a binary file.

- `--textload-file=<file>`  
  Load memory contents from a text-format file.

- `--load-offset=<offset>`  
  Load file contents starting at byte offset `<offset>`.

- `--oformat=<binary|hex>`  
  Set memory dump output format.

---

## Target / Architecture Options
- `-e <target>`  
  Select emulation target. Supported targets:
  - `mem_rw` — SystemC memory R/W architecture  
  - `lms` — LMS adaptive filtering architecture

- `--emulation=<target>`  
  Long form of `-e`.

---

## Filter & Algorithm Options
- `--filter-type=<type>`  
  Set filter implementation type (default: `LMSArch`).

- `--behavior-filter`  
  Enable behavioral filter model (default).

- `--no-behavior-filter`  
  Disable behavioral filter model.

- `--fixedpoint-eval`  
  Enable fixed-point evaluation.

- `--no-fixedpoint-eval`  
  Disable fixed-point evaluation (default).

- `--polyphase`  
  Enable polyphase filter mode.

- `--no-polyphase`  
  Disable polyphase filter mode (default).

---

## Simulation & Debug Options
- `--run-testbench`  
  Run SystemC testbench simulation.

- `--trace`  
  Enable VCD trace file generation.

---

## Examples

Run SystemC testbench:
```bash
adptsysc --run-testbench -e memrw
```

Run LMS architecture:
```bash
adptsysc -e lms --verbose
```

Load binary at offset and dump in hex:
```bash
adptsysc --load-file init.bin --load-offset=0x100 --oformat=hex
```

Enable polyphase and fixed-point mode:

```bash
adptsysc --polyphase --fixedpoint-eval
```