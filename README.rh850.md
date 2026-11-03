# QEMU for Renesas RH850

QEMU 11.1.1 with the RH850 target (qemu-system-rh850, machine
rh850-u2c). Used by the Zephyr RH850 port for CI and as a Zephyr SDK
host tool.

Branch zephyr-qemu-rh850-v11.1.1: the v11.1.1 release plus the RH850
target, devices, machine, tests and documentation
(docs/system/target-rh850.rst).

Build:

    ./configure --target-list=rh850-softmmu
    make

Release archive (qemu-rh850-linux-x86_64.tar.gz with bin/qemu-system-rh850,
unpacked to /opt/qemu-rh850):

    scripts/rh850-release.sh <build-dir> <output-dir>

Run a Zephyr image:

    qemu-system-rh850 -M rh850-u2c -nographic -kernel zephyr.elf
