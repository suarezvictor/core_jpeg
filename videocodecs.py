# Copyright (c) 2024 Victor Suarez Rovere <suarezvictor@gmail.com>


"""
#DONE:
-added compatibility with risc64 compiler
-added rgb888 support (instead of rgb565)
-LiteX integration (replaced original FIFOs and DMA with LiteX's)
-Automatic CSR mapping (register addresses to control the core)
"""


from migen import *
from litex.soc.integration.soc import AutoCSR, CSRStorage, CSRStatus

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
    def __init__(self, rdport=None, wrport=None, DHT=True, debug=False):
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
            p_SUPPORT_WRITABLE_DHT = DHT,
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
                self.pixel_offset.eq(self.outport_pixel_x + self.outport_pixel_y * self.writer_stride.storage[2:]),
                writer.sink.address.eq(self.writer_base.storage[2:] + self.pixel_offset),
                writer.sink.data.eq(Cat(self.outport_pixel_b, self.outport_pixel_g, self.outport_pixel_r, 0xFF))
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
            self.writer_stride = CSRStorage(16)

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


