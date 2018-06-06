# Kernel 3
## Downloading the Executable

The executable file for Kernel 1 is `bin/kernel.elf`.

## Building

`make rebuild` will build the project. The program will be in `bin/kernel.elf` after this operation.

## Running

`make install` builds the project and installs `bin/kernel.elf` into `/u/cs452/tftp/ARM/$(WHOAMI)/kernel.elf`. Then, the command `load -b 0x000218000 -h 10.16.167.5 "ARM/[path]/kernel.elf` loads the program into redboot, where `[path]` represents the results of `$(WHOAMI)` from when `make install` was run.
