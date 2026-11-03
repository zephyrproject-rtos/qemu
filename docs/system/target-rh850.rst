.. SPDX-License-Identifier: GPL-2.0-or-later

.. _RH850-System-emulator:

RH850 System emulator
---------------------

Use the executable ``qemu-system-rh850`` to emulate the Renesas RH850/U2C
microcontroller (RH850/U2C8, R7F702600AFABB) with its G4KH CPU core. Product
information: https://www.renesas.com/en/products/rh850-u2c

The ``rh850-u2c`` machine emulates:

-  G4KH CPU (CPU0 only): basic and cache instructions, system registers,
   EI/FE level exceptions, the register bank and the single and double
   precision FPU
-  Code flash (8 MiB at 0x00000000), local RAM (128 KiB at 0xfdc00000,
   with its self view at 0xfde00000), cluster RAM (0xfe000000) and retention
   RAM (0x12000000)
-  Interrupt controller (INTC1 of CPU0, INTC2, FENMI and FEINT)
-  OS timers OSTM0 and OSTM1
-  RLIN3 UART units (UART mode)
-  A clock controller and standby controller stub, with the software system
   reset (SWSRESA) of the reset controller

Not emulated: other peripherals (their address ranges are unimplemented
devices), the second CPU (PE1), MPU protection (its registers exist, accesses
are not checked), caches and cycle-accurate timing.

Running
=======

The machine loads an ELF image with ``-kernel``; execution starts at the
reset vector. The first serial port is RLIN311::

  qemu-system-rh850 -M rh850-u2c -nographic -kernel <elf>

Machine properties:

``clk-hsb=<Hz>``
  CLKC_HSB frequency, the OSTM clock (default 20000000).

``test-exit=on|off``
  Map an exit device at 0x40000000 for test programs (default off). The
  address is reserved on silicon.
