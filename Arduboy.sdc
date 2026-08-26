derive_pll_clocks
derive_clock_uncertainty

# core specific constraints

# spm_write_cdc.v crosses clk_avr (16 MHz) <-> clk_sys (40 MHz) via a standard 2-flop toggle
# synchronizer, added for SPM's program-memory write (see mega-regfile-timing-fix/PROJECT.md's
# "Attempt 1-4" history for why this crossing exists at all -- Quartus rejected every attempt at
# a second same-clock write port into rom[]). clk_avr and clk_sys are both outputs of the same
# PLL, so sys_top.sdc's clock grouping analyzes real timing paths between them by default --
# fine for existing crossings that go through rom[]'s block RAM, but a plain-flip-flop
# synchronizer isn't meant to be timing-closed at all; its correctness comes from the
# double-flop's own metastability resolution, not from meeting a numeric setup/hold bound.
# False-path only the two crossing edges (each direction's toggle register -> its own first
# synchronizer stage), not every register in the module, so later stages still get normal
# single-clock-domain timing checks.
set_false_path -from [get_registers {*spm_write_cdc*req_toggle}] -to [get_registers {*spm_write_cdc*req_sync1}]
set_false_path -from [get_registers {*spm_write_cdc*ack_toggle}] -to [get_registers {*spm_write_cdc*ack_sync1}]
