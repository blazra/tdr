from migen import *
from migen.genlib.fsm import *
from migen.genlib.cdc import MultiReg
from litex.soc.interconnect.csr import *
from litex.soc.interconnect import wishbone

class Daq(Module, AutoCSR):
    def __init__(self, tx_clk, tx_data, rx_clk, rx_data, data_width=20, mem_depth=256):
        assert (data_width == 20)

        self.clock_domains.cd_tx = ClockDomain(reset_less=True)
        self.comb += self.cd_tx.clk.eq(tx_clk)
        self.clock_domains.cd_rx = ClockDomain(reset_less=True)
        self.comb += self.cd_rx.clk.eq(rx_clk)

        self.trigger = CSRStorage()
        trigger = Signal()
        self.specials += MultiReg(self.trigger.storage, trigger, "tx")

        sample_counter = Signal(max=mem_depth)
        write_enable = Signal()
        self.tx = Signal()

        fsm = ClockDomainsRenamer("rx")( FSM(reset_state="IDLE") )
        self.submodules += fsm

        fsm.act("IDLE",
            write_enable.eq(0),
            NextValue(sample_counter, 0),
            If(trigger,
                NextState("MEASURING")
            )
        )
        fsm.act("MEASURING",
            write_enable.eq(1),
            NextValue(sample_counter, sample_counter + 1),
            If(sample_counter == mem_depth-1,
                NextState("DONE")
            )
        )
        fsm.act("DONE",
            write_enable.eq(0),
            NextValue(sample_counter, 0),
            If(trigger == 0,
                NextState("IDLE")
            )
        )

        self.specials += MultiReg(~fsm.ongoing("MEASURING"), self.tx, "tx")
        self.comb += If(self.tx, 
            tx_data.eq(0xfffff)
        ).Else(
            tx_data.eq(0x00000)
        )

        mem = Memory(data_width, mem_depth)
        self.specials.port = mem.get_port(write_capable=True, clock_domain="rx")
        
        # connect the write port to local logic
        self.comb += [
        self.port.adr.eq(sample_counter),
        self.port.dat_w.eq(rx_data),
        self.port.we.eq(write_enable)
        ]
        
        # attach a wishbone interface to the Memory() object, with a read-only port
        self.submodules.wb_sram_if = wishbone.SRAM(mem, read_only=True)
        
        # get the wishbone Interface accessor object
        self.bus = wishbone.Interface()
        
        # build an address filter object. This is a migen expression returns 1
        # when the address "a" matches the RAM address space. Useful for precisely
        # targeting the RAM relative to other wishbone-connected objects within
        # the logic, e.g. other RAM blocks or registers.
        # 
        # This filter means the RAM object will occupy its entire own CSR block,
        # with aliased copies of the RAM filling the entire block
        def slave_filter(a):
            return 1
        # This filter constrains the RAM to just its own address space
        decoder_offset = log2_int(mem_depth, need_pow2=False)
        def slave_filter_noalias(a):
            return a[decoder_offset:32 - decoder_offset] == 0
        
        # connect the Wishbone bus to the Memory wishbone port, using the address filter
        # to help decode the address.
        # The decdoder address filter as a list with entries as follows:
        #   (filter_function, wishbone_interface.bus)
        # This is useful if you need to attach multiple local logic memories into the
        # same wishbone address bank.
        wb_con = wishbone.Decoder(self.bus, [(slave_filter, self.wb_sram_if.bus)], register=True)
        # add the decoder as a submodule so it gets pulled into the finalize sweep
        self.submodules += wb_con

        
        
        self.analyzer_signals = [
                self.tx,
                self.port.adr,
                self.port.dat_w,
                self.port.we,
            ]