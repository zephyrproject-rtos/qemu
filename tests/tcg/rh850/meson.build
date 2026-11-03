# SPDX-License-Identifier: GPL-2.0-or-later

tests = []

link_script = files('u2c.ld')[0]
asmflags = ['-mv850e3v5', '-mrh850-abi', '-nostdlib', '-nostartfiles',
            '-Wa,-I' + meson.current_source_dir(),
            '-Wl,-T', link_script]
machine = ['-M', 'rh850-u2c,test-exit=on', '-display', 'none',
           '-monitor', 'none']
qemu_args = machine + ['-serial', 'none', '-kernel']

setup = {'cflags': asmflags, 'qemu_args': qemu_args}

foreach t: ['t001', 't010', 't012', 't020', 't030', 't150', 't151', 't163',
            't167', 't171', 't172', 't176', 't178', 't180', 't182', 't186',
            't190', 't193', 't194', 't295', 't300', 't303', 't306', 't330']
  tests += {t + '.s': setup}
endforeach

tests += {
  't141.s': {'cflags': asmflags + ['-Wa,-mextension'], 'qemu_args': qemu_args},
  't320.s': {'cflags': asmflags,
             'qemu_args': machine + ['-serial', 'none',
                                     '-icount', 'shift=0,sleep=off', '-kernel']},
  't322.s': {'cflags': asmflags,
             'qemu_args': machine + ['-serial', 'stdio', '-kernel'],
             'expected_output': 't322.out'},
}

if 'qemu-system-rh850' in emulators
  tcg_tests += {
    'rh850-softmmu': {
      'cc': 'v850-elf-gcc',
      'folder': 'rh850',
      'gdb_arch': 'v850',
      'qemu': emulators['qemu-system-rh850'],
      'tests': tests,
    }
  }
endif
