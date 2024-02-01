# Copyright (c) 2024 Victor Suarez Rovere <suarezvictor@gmail.com>

"""
#DONE:
-added compatibility with riscv64 compiler
-added rgb888 support (instead of rgb565)
-LiteX integration (replaced original FIFOs and DMA with LiteX's)
-Automatic CSR mapping (register addresses to control the core)
-Added support for optimized (custom) huffman tables
"""

from migen import *
from litex.soc.integration.soc import AutoCSR, CSRStorage, CSRStatus
from litedram.frontend.dma import LiteDRAMDMAWriter
from litedram.frontend.dma import LiteDRAMDMAReader


"""
Typical CSR assigment and usage (1280x720 framebuffer):

#define CSR_BASE 0xf0000000L
#define CSR_JPEG_CORE_READER_BASE_ADDR (CSR_BASE + 0x1000L)
#define CSR_JPEG_CORE_READER_LENGTH_ADDR (CSR_BASE + 0x1004L)
#define CSR_JPEG_CORE_READER_ENABLE_ADDR (CSR_BASE + 0x1008L)
#define CSR_JPEG_CORE_READER_DONE_ADDR (CSR_BASE + 0x100cL)
#define CSR_JPEG_CORE_READER_LOOP_ADDR (CSR_BASE + 0x1010L)
#define CSR_JPEG_CORE_READER_OFFSET_ADDR (CSR_BASE + 0x1014L)
#define CSR_JPEG_CORE_WRITER_BASE_ADDR (CSR_BASE + 0x1018L)
#define CSR_JPEG_CORE_WRITER_STRIDE_ADDR (CSR_BASE + 0x101cL)
#define CSR_JPEG_CORE_IDLE_STATUS_ADDR (CSR_BASE + 0x1020L)
#define CSR_JPEG_CORE_OUTPORT_WIDTH_ADDR (CSR_BASE + 0x1024L)
#define CSR_JPEG_CORE_OUTPORT_HEIGHT_ADDR (CSR_BASE + 0x1028L)

litex> mem_write 0xf0001000 0x40000000 //reader address
litex> mem_write 0xf0001004 174632 // 174629 bytes rounded up to dword (90304 for sample480p.jpg)
litex> mem_write 0xf0001018 0x40c00000 //writer address
litex> mem_write 0xf000101c 5120 //writer stride (1280*4)
litex> mem_write 0xf0001008 1 // enable DMA
litex> mem_read 0xf0001020 //poll idle status
litex> mem_write 0xf0001008 0 // disable DMA

litex> mem_read 0x40000000 64 // this dumps the JPEG header
litex> mem_write 0x40C00000 0xFF8040 921600 4 //this fills the framebuffer
litex> mem_read 0xf0001000 0x2c //this shows the core registers
"""

class JPEGDecoder(Module, AutoCSR):
    def __init__(self, rdport, wrport, DHT=True, output_alpha=0xFF, debug=False):
        assert(rdport.data_width == 32)
        assert(wrport.data_width == 32)

        self.submodules.reader = reader = LiteDRAMDMAReader(rdport, with_csr=True)
        self.submodules.writer = writer = LiteDRAMDMAWriter(wrport, with_csr=False)
        source = self.reader.source

        self.writer_base = CSRStorage(wrport.address_width)
        self.writer_stride = CSRStorage(16)
        self.idle_status = CSRStatus()
        self.outport_width = CSRStatus(16)
        self.outport_height = CSRStatus(16)

        inport_accept = Signal()
        inport_ready = Signal()
        idle = Signal()
        outport_valid = Signal()
        outport_pixel_x = Signal(16)
        outport_pixel_y = Signal(16)
        outport_pixel_r = Signal(8)
        outport_pixel_g = Signal(8)
        outport_pixel_b = Signal(8)
        pixel_offset = Signal(wrport.address_width)
        alpha_value = Constant(output_alpha)

        self.params = dict(
            p_SUPPORT_WRITABLE_DHT = DHT,
            i_clk_i = ClockSignal("sys"),
            i_rst_i = ResetSignal("sys"),
            o_outport_width_o = self.outport_width.status,
            o_outport_height_o = self.outport_height.status,
            o_inport_accept_o = inport_accept, #ready to accept data (asserted once each 4 bytes)
            o_idle_o = idle, # useful to know when it's done
            i_inport_valid_i = source.valid,
            i_inport_data_i = source.data,
            i_inport_last_i = source.last & inport_accept,
            i_inport_strb_i = Constant(0xF),
            i_outport_accept_i = writer.sink.ready,
            o_outport_valid_o = outport_valid,
            o_outport_pixel_x_o = outport_pixel_x,
            o_outport_pixel_y_o = outport_pixel_y,
            o_outport_pixel_r_o = outport_pixel_r,
            o_outport_pixel_g_o = outport_pixel_g,
            o_outport_pixel_b_o = outport_pixel_b,
        )

        self.comb += [
            writer.sink.valid.eq(outport_valid), #write strobe
            pixel_offset.eq(outport_pixel_x + outport_pixel_y * self.writer_stride.storage[2:]),
            writer.sink.address.eq(self.writer_base.storage[2:] + pixel_offset),
            writer.sink.data.eq(Cat(outport_pixel_b, outport_pixel_g, outport_pixel_r, alpha_value)),
            inport_ready.eq(inport_accept | (idle & (reader._offset.status == Constant(0)) )),
            source.ready.eq(inport_ready),
            self.idle_status.status.eq(idle),
        ]

        self.specials += Instance("jpeg_core", **self.params)

