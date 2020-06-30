
#!/usr/bin/env python3

# Based on versa-ecp5 example https://github.com/enjoy-digital/liteiclink
# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# This file is Copyright (c) 2020 Radovan Blažek <blazra@gmail.com>
# License: BSD

import sys

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.build.generic_platform import *

from litex_boards.platforms import versa_ecp5

from litex.soc.cores import uart, spi
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from tdr.clock import *
from tdr.serdes_ecp5 import SerDesECP5PLL, SerDesECP5
from tdr.daq import Daq

# IOs ----------------------------------------------------------------------------------------------

_spidac_io = [
    ("spidac_io", 0,
        Subsignal("clk",  Pins("A13")),
        Subsignal("cs_n", Pins("C13")),
        Subsignal("mosi", Pins("E13")),
        Subsignal("miso", Pins("C14")),
        IOStandard("LVCMOS33"),
     ),
]

# CRG ----------------------------------------------------------------------------------------------


class CRG(Module):
    def __init__(self, platform, sys_clk_freq, refclk_from_pll, refclk_freq):
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_por = ClockDomain(reset_less=True)
        self.clock_domains.cd_ref = ClockDomain(reset_less=True)

        self.phasesel = CSRStorage(2)
        self.phasedir = CSRStorage()
        self.phasestep = CSRStorage()
        self.phaseloadreg = CSRStorage()

        # # #

        # clk / rst
        clk100 = platform.request("clk100")
        rst_n = platform.request("rst_n")
        platform.add_period_constraint(clk100, 1e9/100e6)

        # power on reset
        por_count = Signal(16, reset=2**16-1)
        por_done = Signal()
        self.comb += self.cd_por.clk.eq(ClockSignal())
        self.comb += por_done.eq(por_count == 0)
        self.sync.por += If(~por_done, por_count.eq(por_count - 1))

        # pll
        self.submodules.pll = pll = ECP5PLL()
        pll.register_clkin(clk100, 100e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        if refclk_from_pll:
            pll.create_clkout(self.cd_ref, refclk_freq)
        self.specials += AsyncResetSynchronizer(
            self.cd_sys, ~por_done | ~pll.locked | ~rst_n)
        self.comb += [
            pll.phasesel.eq(self.phasesel.storage),
            pll.phasedir.eq(self.phasedir.storage),
            pll.phasestep.eq(self.phasestep.storage),
            pll.phaseloadreg.eq(self.phaseloadreg.storage),
        ]


class TdrSoC(SoCMini, AutoCSR):
    mem_map = {
        "daq_memory": 0x30000000,  # this just needs to be a unique block
    }
    mem_map.update(SoCMini.mem_map)

    def __init__(self, platform, connector="sma"):
        assert connector in ["sma", "pcie"]
        sys_clk_freq = int(100e6)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq,
                         ident="TdrSoc", ident_version=True)

        # CRG --------------------------------------------------------------------------------------
        linerate = 2500e6
        refclk_from_pll = False
        refclk_freq = 156.25e6

        crg = CRG(platform, sys_clk_freq, refclk_from_pll, refclk_freq)
        self.submodules.crg = crg

        # SerDes RefClk ----------------------------------------------------------------------------
        if refclk_from_pll:
            refclk = self.crg.cd_ref.clk
        else:
            refclk_pads = platform.request("refclk", 1)
            self.comb += platform.request("refclk_en").eq(1)
            self.comb += platform.request("refclk_rst_n").eq(1)
            refclk = Signal()
            self.specials.extref0 = Instance("EXTREFB",
                                             i_REFCLKP=refclk_pads.p,
                                             i_REFCLKN=refclk_pads.n,
                                             o_REFCLKO=refclk,
                                             p_REFCK_PWDNB="0b1",
                                             p_REFCK_RTERM="0b1",  # 100 Ohm
                                             )
            self.extref0.attr.add(("LOC", "EXTREF0"))

        # SerDes PLL -------------------------------------------------------------------------------
        serdes_pll = SerDesECP5PLL(
            refclk, refclk_freq=refclk_freq, linerate=linerate)
        self.submodules += serdes_pll

        # SerDes -----------------------------------------------------------------------------------
        tx_pads = platform.request(connector + "_tx")
        rx_pads = platform.request(connector + "_rx")
        channel = 1 if connector == "sma" else 0
        serdes = SerDesECP5(serdes_pll, tx_pads, rx_pads,
                            channel=channel,
                            data_width=20)
        serdes.add_base_control()
        self.submodules.serdes = serdes
        self.add_csr("serdes")
        platform.add_period_constraint(serdes.txoutclk, 1e9/serdes.tx_clk_freq)
        platform.add_period_constraint(serdes.rxoutclk, 1e9/serdes.rx_clk_freq)
        self.clock_domains.cd_tx = ClockDomain(reset_less=True)
        self.comb += self.cd_tx.clk.eq(serdes.txoutclk)
        self.clock_domains.cd_rx = ClockDomain(reset_less=True)
        self.comb += self.cd_rx.clk.eq(serdes.rxoutclk)

        # DAC
        spi_pads = platform.request("spidac_io")
        spi_master = spi.SPIMaster(spi_pads, 16, sys_clk_freq, 1e6)
        self.submodules.spidac = spi_master
        self.add_csr("spidac")

        # DAQ --------------------------------------------------------------------------------------
        mem_depth = 256
        data_width = 20
        daq = Daq(tx_clk=serdes.rxoutclk, tx_data=serdes.tx_data, rx_clk=serdes.rxoutclk,
                  rx_data=serdes.rx_data, data_width=data_width, mem_depth=mem_depth)
        self.submodules.daq = daq
        self.comb += serdes.ldr_tx_data.eq(daq.tx)
        # define the memory region size/location
        # memdepth is the depth of the memory inferred in your logic block
        # the length is in bytes, so for example if the data_width of your memory
        # block is 32 bits, the total length is memdepth * 4
        self.add_memory_region(
            "daq_memory", self.mem_map["daq_memory"], mem_depth * data_width//8)
        # add the wishbone slave mapping to the mem_map entry made above
        self.add_wb_slave(self.mem_map["daq_memory"], self.daq.bus)
        self.add_csr("daq")

        # Leds -------------------------------------------------------------------------------------
        sys_counter = Signal(32)
        self.sync.sys += sys_counter.eq(sys_counter + 1)
        tx_counter = Signal(32)
        self.sync.tx += tx_counter.eq(tx_counter + 1)
        rx_counter = Signal(32)
        self.sync.rx += rx_counter.eq(rx_counter + 1)
        refclk_counter = Signal(32)
        self.comb += [
            platform.request("user_led", 0).eq(~sys_counter[26]),
            platform.request("user_led", 1).eq(~serdes.tx_enable),
            platform.request("user_led", 2).eq(~serdes.tx_ready),
            platform.request("user_led", 3).eq(~serdes.rx_ready),
            platform.request("user_led", 4).eq(~serdes.rx_data[0]),
            platform.request("user_led", 5).eq(~serdes.rx_lol),
            platform.request("user_led", 6).eq(~tx_counter[26]),
            platform.request("user_led", 7).eq(~rx_counter[26]),
        ]
        # UART bridge
        uart_bridge = uart.UARTWishboneBridge(
            pads=platform.request("serial"),
            clk_freq=self.clk_freq,
            baudrate=115200)
        self.submodules += uart_bridge
        self.add_wb_master(uart_bridge.wishbone)

        analyzer_groups = {}
        analyzer_groups[0] = [
            serdes.analyzer_signals,
            daq.analyzer_signals,
        ]
        from litescope import LiteScopeAnalyzer
        self.submodules.analyzer = LiteScopeAnalyzer(serdes.analyzer_signals+daq.analyzer_signals,
                                                     depth=256,
                                                     clock_domain="rx",
                                                     csr_csv="analyzer.csv")
        self.add_csr("analyzer")


def load():
    import os
    os.system("ecpprog -S build/gateware/versa_ecp5.bit")
    exit()


def main():
    platform = versa_ecp5.Platform(toolchain="trellis", device="LFE5UM")
    platform.add_extension(_spidac_io)
    soc = TdrSoC(platform)
    builder = Builder(soc, output_dir="build", csr_csv="csr.csv")
    builder.build(build_name="versa_ecp5")
    if "load" in sys.argv[1:]:
        import os
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(
            builder.gateware_dir, soc.build_name + ".svf"))
        # load()


if __name__ == "__main__":
    main()
