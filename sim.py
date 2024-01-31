#!/usr/bin/env python3

# Copyright (c) 2023-2024 Victor Suarez Rovere <suarezvictor@gmail.com>
# SPDX-License-Identifier: AGPL-3.0-only
#
# Copyright (c) 2015-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2020 Antmicro <www.antmicro.com>
# Copyright (c) 2017 Pierre-Olivier Vauboin <po@lambdaconcept>

"""
#DONE:
-added compatibility with risc64 compiler
-added rgb888 support (instead of rgb565)
-LiteX integration (replaced original FIFOs and DMA with LiteX's)
-Automatic CSR mapping (register addresses to control the core)
"""


import sys
import argparse

from migen import *

from litex.build.generic_platform import *
from litex.build.sim import SimPlatform
from litex.build.sim.config import SimConfig
from litex.build.sim.verilator import verilator_build_args, verilator_build_argdict

from litex.soc.interconnect.csr import *
from litex.soc.integration.common import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.integration.soc import *
from litex.soc.cores.cpu import CPUS

from litedram import modules as litedram_modules
from litedram.modules import parse_spd_hexdump
from litedram.phy.model import sdram_module_nphases, get_sdram_phy_settings
from litedram.phy.model import SDRAMPHYModel

from litex.soc.cores.video import video_data_layout

"""
#define CSR_BASE 0xf0000000L

with DMA:

#define CSR_JPEG_CORE_READER_BASE_ADDR (CSR_BASE + 0x1000L)
#define CSR_JPEG_CORE_READER_LENGTH_ADDR (CSR_BASE + 0x1004L)
#define CSR_JPEG_CORE_READER_ENABLE_ADDR (CSR_BASE + 0x1008L)
#define CSR_JPEG_CORE_READER_DONE_ADDR (CSR_BASE + 0x100cL)
#define CSR_JPEG_CORE_READER_LOOP_ADDR (CSR_BASE + 0x1010L)
#define CSR_JPEG_CORE_READER_OFFSET_ADDR (CSR_BASE + 0x1014L)
#define CSR_JPEG_CORE_WRITER_BASE_ADDR (CSR_BASE + 0x1018L)
#define CSR_JPEG_CORE_IDLE_STATUS_ADDR (CSR_BASE + 0x101cL)
#define CSR_JPEG_CORE_OUTPORT_WIDTH_ADDR (CSR_BASE + 0x1020L)
#define CSR_JPEG_CORE_OUTPORT_HEIGHT_ADDR (CSR_BASE + 0x1024L)


litex> mem_read 0x40000000 64 // this dumps the JPEG header
litex> mem_write 0x40C00000 0xFF8040 921600 4 //this fills the framebuffer
litex> mem_read 0xf0001000 0x28 //this shows the core registers

litex> mem_write 0xf0001000 0x40000000 //reader address
litex> mem_write 0xf0001004 174632 // 174629 bytes rounded up to dword
litex> mem_write 0xf0001018 0x40c00000 //writer address
litex> mem_write 0xf0001008 1 // enable DMA
litex> mem_read 0xf000101c //poll idle status
"""

class JPEGCore(Module, AutoCSR):
    def __init__(self, rdport=None, wrport=None, debug=False):
        self.inport_accept = Signal()
        self.inport_ready = Signal()
        self.idle = Signal()

        if rdport is not None:
            from litedram.frontend.dma import LiteDRAMDMAReader
            self.submodules.reader = reader = LiteDRAMDMAReader(rdport, with_csr=True)

        if wrport is not None:
            assert(wrport.data_width == 32)
            from litedram.frontend.dma import LiteDRAMDMAWriter
            self.submodules.writer = writer = LiteDRAMDMAWriter(wrport, with_csr=False)

        self.add_csrs(inputcsr=rdport is None, outputcsr=wrport is None)

        self.params = dict(
            i_clk_i = ClockSignal("sys"),
            i_rst_i = ResetSignal("sys"),
            o_outport_width_o = self.outport_width.status,
            o_outport_height_o = self.outport_height.status,
            o_inport_accept_o = self.inport_accept, #input port is ready to accept JPEG data (asserted each 4 bytes written)
            o_idle_o = self.idle # useful to know when it's done
        )

        if rdport is None:
            self.params.update(dict(
                i_inport_valid_i = self.inport_valid.re, #will be asserted for 1 cycle when written
                i_inport_data_i = self.inport_data.storage,
                i_inport_last_i = self.inport_last.storage,
                i_inport_strb_i = self.inport_strb.storage
            ))
            if debug:
                self.sync += If(self.inport_valid.re,
                    Display("new input data %x (inport_accept.status was %d)", self.inport_data.storage, self.inport_accept.status))
        else:
            assert(rdport.data_width == 32)

            source = self.reader.source
            self.params.update(dict(
                i_inport_valid_i = source.valid,
                i_inport_data_i = source.data,
                i_inport_last_i = source.last & self.inport_accept,
                i_inport_strb_i = Constant(0xF)
            ))

        if wrport is not None:
            assert(wrport.data_width == 32)
            self.outport_valid = Signal()
            self.outport_pixel_x = Signal(16)
            self.outport_pixel_y = Signal(16)
            self.outport_pixel_r = Signal(8)
            self.outport_pixel_g = Signal(8)
            self.outport_pixel_b = Signal(8)

            self.params.update(dict(
                i_outport_accept_i = writer.sink.ready,
                o_outport_valid_o = self.outport_valid,
                o_outport_pixel_x_o = self.outport_pixel_x,
                o_outport_pixel_y_o = self.outport_pixel_y,
                o_outport_pixel_r_o = self.outport_pixel_r,
                o_outport_pixel_g_o = self.outport_pixel_g,
                o_outport_pixel_b_o = self.outport_pixel_b,
            ))

            self.pixel_offset = Signal(wrport.address_width)

            self.comb += [
                writer.sink.valid.eq(self.outport_valid), #write
                self.pixel_offset.eq(self.outport_pixel_x + self.outport_pixel_y * self.outport_width.status),
                writer.sink.address.eq(self.writer_base.storage[2:] + self.pixel_offset),
                writer.sink.data.eq(Cat(self.outport_pixel_b, self.outport_pixel_g, self.outport_pixel_r))
            ]
            if debug:
                self.sync += If(writer.sink.valid, Display("DMA write at offset %d, address %x, ready %d", self.pixel_offset, writer.sink.address, writer.sink.ready))
        else:
            self.params.update(dict(
                i_outport_accept_i = self.outport_accept.storage, #output port is ready to accept pixels
                o_outport_valid_o = self.outport_valid.status,
                o_outport_pixel_x_o = self.outport_pixel_x.status,
                o_outport_pixel_y_o = self.outport_pixel_y.status,
                o_outport_pixel_r_o = self.outport_pixel_r.status,
                o_outport_pixel_g_o = self.outport_pixel_g.status,
                o_outport_pixel_b_o = self.outport_pixel_b.status,
            ))
                  

        if rdport is None:
            self.comb += self.inport_ready.eq(self.inport_accept | self.idle_status.status)
            self.comb += self.inport_accept_status.status.eq(self.inport_ready) #if idle always accept data
        else:
            self.comb += self.inport_ready.eq(self.inport_accept | (self.idle & (reader._offset.status == Constant(0)) ))
            self.comb += source.ready.eq(self.inport_ready)
            if debug:
                self.sync += If(source.valid,
                    Display("new dma input data %x (inport_accept was %d, ready %d, idle %d, offset)",
                        source.data, self.inport_accept, self.inport_ready, self.idle, reader._offset.status))

                self.sync += If(self.outport_valid.status,
                    Display("new dma output x %d y %d",
                        self.outport_pixel_x.status, self.outport_pixel_y.status))

        self.specials += Instance("jpeg_core", **self.params)
        self.comb += self.idle_status.status.eq(self.idle)


    def add_csrs(self, inputcsr=False, outputcsr=False):
        if not outputcsr:
            self.writer_base = CSRStorage(32)

        self.idle_status	 = CSRStatus()
        self.outport_width	 = CSRStatus(16)
        self.outport_height	 = CSRStatus(16)

        if inputcsr:
            self.inport_valid	 = CSRStorage(reset=0)
            self.inport_data	 = CSRStorage(32, reset=0)
            self.inport_strb	 = CSRStorage(4, reset=0xF)
            self.inport_last	 = CSRStorage(reset=0)
            self.inport_accept_status	 = CSRStatus()
        
        if outputcsr:
            self.outport_accept	 = CSRStorage(reset=1)
            self.outport_valid	 = CSRStatus()
            self.outport_pixel_x = CSRStatus(16)
            self.outport_pixel_y = CSRStatus(16)
            self.outport_pixel_r = CSRStatus(8)
            self.outport_pixel_g = CSRStatus(8)
            self.outport_pixel_b = CSRStatus(8)


SYS_CLK_FREQ = 1e6

# IOs ----------------------------------------------------------------------------------------------

_io = [
    # Rst. (clk is set by the clocker)
    ("sys_rst", 0, Pins(1)),

    # Serial.
    ("serial", 0,
        Subsignal("source_valid", Pins(1)),
        Subsignal("source_ready", Pins(1)),
        Subsignal("source_data",  Pins(8)),

        Subsignal("sink_valid",   Pins(1)),
        Subsignal("sink_ready",   Pins(1)),
        Subsignal("sink_data",    Pins(8)),
    ),
    # Video
    ("vga", 0,
        Subsignal("clk",   Pins(1)), #use pixel clock
        Subsignal("hsync", Pins(1)),
        Subsignal("vsync", Pins(1)),
        Subsignal("de",    Pins(1)),
        Subsignal("r",     Pins(8)),
        Subsignal("g",     Pins(8)),
        Subsignal("b",     Pins(8)),
        Subsignal("valid", Pins(1)), #handles backpressure
    )
]

# Platform -----------------------------------------------------------------------------------------

class Platform(SimPlatform):
    def __init__(self):
        SimPlatform.__init__(self, "SIM", _io)

# Video
class VideoPHYModel(Module, AutoCSR):
    def __init__(self, pads, clock_domain="sys"):
        self.sink = sink = stream.Endpoint(video_data_layout)

        # # #

        # Always ack Sink, no backpressure.
        self.comb += sink.ready.eq(1)

        # Drive Clk.
        if hasattr(pads, "clk"):
            self.comb += pads.clk.eq(ClockSignal(clock_domain))

        # Drive Controls.
        self.comb += pads.valid.eq(1) #may be overriden with underflow from the framebuffer
        self.comb += pads.de.eq(sink.de)
        self.comb += pads.hsync.eq(sink.hsync)
        self.comb += pads.vsync.eq(sink.vsync)

        # Drive Datas.
        cbits  = len(pads.r)
        cshift = (8 - cbits)
        for i in range(cbits):
            self.comb += pads.r[i].eq(sink.r[cshift + i] & sink.de)
            self.comb += pads.g[i].eq(sink.g[cshift + i] & sink.de)
            self.comb += pads.b[i].eq(sink.b[cshift + i] & sink.de)


# Clocks -------------------------------------------------------------------------------------------

class Clocks(dict):
    # FORMAT: {name: {"freq_hz": _, "phase_deg": _}, ...}
    def names(self):
        return list(self.keys())

    def add_io(self, io):
        for name in self.names():
            io.append((name + "_clk", 0, Pins(1)))

    def add_clockers(self, sim_config):
        for name, desc in self.items():
            sim_config.add_clocker(name + "_clk", **desc)

class _CRG(Module):
    def __init__(self, platform, domains=None):
        if domains is None:
            domains = ["sys"]
        # request() before clreating domains to avoid signal renaming problem
        domains = {name: platform.request(name + "_clk") for name in domains}

        self.clock_domains.cd_por = ClockDomain(reset_less=True)
        for name in domains.keys():
            setattr(self.clock_domains, "cd_" + name, ClockDomain(name=name))

        int_rst = Signal(reset=1)
        self.sync.por += int_rst.eq(0)
        self.comb += self.cd_por.clk.eq(self.cd_sys.clk)

        for name, clk in domains.items():
            cd = getattr(self, "cd_" + name)
            self.comb += cd.clk.eq(clk)
            self.comb += cd.rst.eq(int_rst)


class SimSoC(SoCCore):
    def __init__(self, clocks,
        with_sdram            = False,
        sdram_module          = "MT48LC16M16",
        sdram_init            = [],
        sdram_data_width      = 128, #wide bus convenient for framebuffer
        with_video_framebuffer = False,
        no_compile_gateware   = False,
        **kwargs):
        platform     = Platform()
        sys_clk_freq = int(SYS_CLK_FREQ)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, clocks.names())


        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq,
            ident = "LiteX Simulation",
            **kwargs)


        # SDRAM ------------------------------------------------------------------------------------
        if not self.integrated_main_ram_size and with_sdram:
            sdram_clk_freq = int(100e6) # FIXME: use 100MHz timings
            if True:
                sdram_module_cls = getattr(litedram_modules, sdram_module)
                sdram_rate       = "1:{}".format(sdram_module_nphases[sdram_module_cls.memtype])
                sdram_module     = sdram_module_cls(sdram_clk_freq, sdram_rate)
            self.submodules.sdrphy = SDRAMPHYModel(
                module     = sdram_module,
                data_width = sdram_data_width,
                clk_freq   = sdram_clk_freq,
                verbosity  = False,
                init       = sdram_init)
            self.add_sdram("sdram",
                phy                     = self.sdrphy,
                module                  = sdram_module,
                l2_cache_size           = kwargs.get("l2_size", 8192),
                l2_cache_min_data_width = kwargs.get("min_l2_data_width", 128),
                l2_cache_reverse        = False
            )
            if sdram_init != []:
                # Skip SDRAM test to avoid corrupting pre-initialized contents.
                self.add_constant("SDRAM_TEST_DISABLE")
            else:
                # Reduce memtest size for simulation speedup
                self.add_constant("MEMTEST_DATA_SIZE", 8*1024)
                self.add_constant("MEMTEST_ADDR_SIZE", 8*1024)


        # Video --------------------------------------------------------------------------------------
        if with_video_framebuffer:
            video_pads = platform.request("vga")
            self.submodules.videophy = VideoPHYModel(video_pads, clock_domain="pix")
            self.add_video_framebuffer(phy=self.videophy, timings="1280x720@60Hz", format="rgb888", clock_domain="pix")
            self.videophy.comb += video_pads.valid.eq(~self.video_framebuffer.underflow)

            rdport = self.sdram.crossbar.get_port(mode="read", data_width=32)
            wrport = self.sdram.crossbar.get_port(mode="write", data_width=32)
            self.submodules.jpeg_core = JPEGCore(rdport=rdport, wrport=wrport) #add JPEG module


# Build --------------------------------------------------------------------------------------------

def sim_args(parser):
    builder_args(parser)
    soc_core_args(parser)
    verilator_build_args(parser)
    parser.add_argument("--sdram-init",           default=None,            help="SDRAM init file (.bin or .json).")


def main():
    from litex.soc.integration.soc import LiteXSoCArgumentParser
    parser = LiteXSoCArgumentParser(description="LiteX SoC Simulation utility")
    sim_args(parser)
    args = parser.parse_args()

    soc_kwargs             = soc_core_argdict(args)
    builder_kwargs         = builder_argdict(args)
    verilator_build_kwargs = verilator_build_argdict(args)

    sys_clk_freq = int(SYS_CLK_FREQ)
    
    clocks = Clocks({
        "sys":         dict(freq_hz=sys_clk_freq),
        "pix":         dict(freq_hz=int(sys_clk_freq/4)), #pixel clock is slower to save RAM bandwidth
    })
    clocks.add_io(_io)
        
    sim_config   = SimConfig()
    clocks.add_clockers(sim_config)

    # Configuration --------------------------------------------------------------------------------

    cpu            = CPUS.get(soc_kwargs.get("cpu_type", "vexriscv"))
    bus_data_width = int(soc_kwargs["bus_data_width"])

    # UART.
    if soc_kwargs["uart_name"] == "serial":
        soc_kwargs["uart_name"] = "sim"
        sim_config.add_module("serial2console", "serial")

    # Create config SoC that will be used to prepare/configure real one.
    conf_soc = SimSoC(clocks, **soc_kwargs)


    # RAM / SDRAM.
    ram_boot_address = None
    soc_kwargs["integrated_main_ram_size"] = args.integrated_main_ram_size
    soc_kwargs["sdram_data_width"] = 128
    if args.sdram_init is not None:
        soc_kwargs["sdram_init"] = get_mem_data(args.sdram_init,
            data_width = bus_data_width,
            endianness = cpu.endianness,
            offset     = conf_soc.mem_map["main_ram"]
        )
        #ram_boot_address         = get_boot_address(args.sdram_init) #this is to execute at DRAM

	
    # Video.
    sim_config.add_module("video", "vga")

    # SoC ------------------------------------------------------------------------------------------
    soc = SimSoC(clocks,
        with_sdram             = True,
        with_video_framebuffer = True,
        no_compile_gateware    = args.no_compile_gateware,
        **soc_kwargs)
    if ram_boot_address is not None:
        if ram_boot_address == 0:
            ram_boot_address = conf_soc.mem_map["main_ram"]
        soc.add_constant("ROM_BOOT_ADDRESS", ram_boot_address)

    soc.add_constant("LITEX_SIMULATION") #this is to make the software know if it's simulation
    soc.platform.add_source_dir("src_v") #for jpeg_core.v and dependencies

    builder = Builder(soc, **builder_kwargs)
    builder.build(
        sim_config       = sim_config,
        **verilator_build_kwargs,
    )

#./sim.py --sdram-init autorun.jpg

if __name__ == "__main__":
    main()
