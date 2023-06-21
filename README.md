Commodore 64 for MEGA65
=======================

This is a Git submodule of the Commodore 64 core for the MEGA65. It is a fork of the MiSTer FPGA Commodore 64 core, which is itself based on FPGA64 by Peter Wendrich with heavy later modifications by different people.

**Go to https://github.com/MJoergen/C64MEGA65 to learn more.**


The MEGA65 port is based on the [MiSTer2MEGA65 framework](https://github.com/sy2002/MiSTer2MEGA65). We forked the [MiSTer core](https://github.com/MiSTer-devel/C64_MiSTer), so that we can easily track upstream changes and merge them into our MEGA65 port as needed. The following structure is being used:

* [master](https://github.com/MJoergen/C64_MiSTerMEGA65) branch: Original MiSTer fork with the only exception that we changed this README.md
* [C64MEGA65](https://github.com/MJoergen/C64_MiSTerMEGA65/tree/C64MEGA65) branch: Contains our stable modifications of the upstream MiSTer core
* [develop](https://github.com/MJoergen/C64_MiSTerMEGA65/tree/develop) branch: Is our kind-of stable work-in-progress (WIP) development branch

On modifications:

* Our strategy is to reduce the modifications to the upstream core to the bare minimum.
* We made sure that the code is actually synthesizing using Vivado (used for the MEGA65), which is stricter and more unforgiving than Quartus (used for the MiSTer).
* We made the ROM loading and keyboard handling compatible with the MEGA65 core.
* We made the MiSTer core [more compatible](https://github.com/MJoergen/C64_MiSTerMEGA65/blob/1d3a8e635237d205d18ae8855f67b3590c8583ee/rtl/fpga64_buslogic.vhd#L21) to the [PLA](https://github.com/MJoergen/C64MEGA65/blob/master/doc/PLA.md) of the C64 to support hardware cartridges
