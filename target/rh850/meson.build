# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 Linumiz

gen = [
  decodetree.process('insns16.decode',
                     extra_args: ['--static-decode=decode_insn16', '--insnwidth=16']),
  decodetree.process('insns32.decode',
                     extra_args: '--static-decode=decode_insn32'),
]

rh850_ss = ss.source_set()
rh850_ss.add(gen)
rh850_ss.add(files('cpu.c', 'fpu_helper.c', 'helper.c', 'op_helper.c', 'sysreg.c', 'translate.c',
                       'gdbstub.c'))

target_arch += {'rh850': rh850_ss}
