
//////////////////////////////////////////////////////////////////////////////////
// systolic_csr.v
// Hold all the configuration registers of the top.sv (systolic array)
// CPU will write to these registers AFTER the DMA finishes loading the SPADs 
//
// Register Map, Byte Addressable:
// 0x00 conv_mode  [0] (PW = 0, DW = 1)
// 0x04 p_mode     [1:0]
// 0x08 i_size
// 0x0C i_c_size
// 0x10 o_c_size
// 0x14 o_size
// 0x18 stride
// 0x1C depth_mult
// 0x20 i_start_addr
// 0x24 i_addr_end
// 0x28 w_start_addr
// 0x2C w_addr_end
// 0x30 CTRL -> [0] (route_en/START), [1] (reg_clear)
// 0x34 STATUS -> [0] read_only, o_done
// 0x38 quant_sh -> quantization right shift amount
// 0x3C quant_mult
// https://developer.arm.com/documentation/ihi0022/k/ 
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
module systolic_csr #(
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 32
)(
    input wire clk, nrst,
    
    // --- AXI4-Lite Slave ---
    // this is where the CPU will write the config parameters
    input  wire [7:0]  s_axil_awaddr,
    input  wire        s_axil_awvalid,
    output reg         s_axil_awready,
    input  wire [31:0] s_axil_wdata,
    input  wire        s_axil_wvalid,
    output reg         s_axil_wready,
    output reg  [1:0]  s_axil_bresp,
    output reg         s_axil_bvalid,
    input  wire        s_axil_bready,

    input  wire [7:0]  s_axil_araddr,
    input  wire        s_axil_arvalid,
    output reg         s_axil_arready,
    output reg  [31:0] s_axil_rdata,
    output reg  [1:0]  s_axil_rresp,
    output reg         s_axil_rvalid,
    input  wire        s_axil_rready,

    // --- Status input bit from the top.sv of the systolic array --- 
    input wire         i_done, // o_done signal, this is what the CPU will poll

    // --- Config Outputs, wire these to the top.sv ports ---
    output reg                    o_conv_mode,
    output reg  [1:0]             o_p_mode,

    output reg  [ADDR_WIDTH-1:0]  o_i_size,
    output reg  [ADDR_WIDTH-1:0]  o_i_c_size,
    output reg  [ADDR_WIDTH-1:0]  o_o_c_size,
    output reg  [ADDR_WIDTH-1:0]  o_o_size,

    output reg  [ADDR_WIDTH-1:0]  o_stride,
    output reg  [ADDR_WIDTH-1:0]  o_depth_mult,

    output reg  [ADDR_WIDTH-1:0]  o_i_start_addr,
    output reg  [ADDR_WIDTH-1:0]  o_i_addr_end,
    output reg  [ADDR_WIDTH-1:0]  o_w_start_addr,
    output reg  [ADDR_WIDTH-1:0]  o_w_addr_end,

    output reg                    o_route_en,    // i_route_en (START)
    output reg                    o_reg_clear,   // i_reg_clear (flush)

    // --- Quantization Outputs to top.sv systolic ---
    output reg  [DATA_WIDTH-1:0]   o_quant_sh,
    output reg  [2*DATA_WIDTH-1:0] o_quant_mult
    );

    // --- AXI Write ---
    // Cycle 1: AW accepted, store it 
    // Cycle 2: W accepted, slave has the address and data now, you can write the data to the reg
    reg [7:0] aw_lat;  // store the address from the AW channel until W says to write
    reg       aw_pend; // pending (Flag) "I have an address I'm holding right now, wait for W before you accept data"

    always @(posedge clk or negedge nrst)begin
        if (!nrst) begin
            s_axil_awready <= 0; s_axil_wready <= 0;
            s_axil_bvalid  <= 0; s_axil_bresp  <= 0;
            aw_pend <= 0; aw_lat <= 0;

            // Defaults, don't run anything UNLESS the CPU tells to do so
            o_conv_mode    <= 0; o_p_mode      <= 0;
            o_i_size       <= 0; o_i_c_size    <= 0;
            o_o_c_size     <= 0; o_o_size      <= 0;
            o_stride       <= 1; o_depth_mult  <= 1;
            o_i_start_addr <= 0; o_i_addr_end  <= 0;
            o_w_start_addr <= 0; o_w_addr_end  <= 0;
            o_route_en     <= 0; o_reg_clear   <= 0;
            o_quant_sh <= 8'h05; o_quant_mult  <= 16'h9c8c; // make the hardcoded top.sv systolic values the default 
        
        end else begin
            s_axil_awready <= 0;
            s_axil_wready  <= 0;

            // software does not need to write back 0 to deassert these two signals:
            o_route_en     <= 0; // auto clear every cycle
            o_reg_clear    <= 0; // auto clear every cycle
            
            // CPU request will check awvalid, and then latch the addr and aw_pend = 1 to wait for W
            if(s_axil_awvalid && !aw_pend) begin
                s_axil_awready <= 1;
                aw_lat <= s_axil_awaddr; // latch the address
                aw_pend <= 1; // wait for W
            end

            if(s_axil_wvalid && aw_pend)begin
                s_axil_wready <=1;
                aw_pend <= 0; // clear, complete transaction
                case(aw_lat[7:2])
                    6'h00: o_conv_mode    <= s_axil_wdata[0];
                    6'h01: o_p_mode       <= s_axil_wdata[1:0];
                    6'h02: o_i_size       <= s_axil_wdata[ADDR_WIDTH-1:0];
                    6'h03: o_i_c_size     <= s_axil_wdata[ADDR_WIDTH-1:0];
                    6'h04: o_o_c_size     <= s_axil_wdata[ADDR_WIDTH-1:0];
                    6'h05: o_o_size       <= s_axil_wdata[ADDR_WIDTH-1:0];
                    6'h06: o_stride       <= s_axil_wdata[ADDR_WIDTH-1:0];
                    6'h07: o_depth_mult   <= s_axil_wdata[ADDR_WIDTH-1:0];
                    6'h08: o_i_start_addr <= s_axil_wdata[ADDR_WIDTH-1:0];
                    6'h09: o_i_addr_end   <= s_axil_wdata[ADDR_WIDTH-1:0];
                    6'h0A: o_w_start_addr <= s_axil_wdata[ADDR_WIDTH-1:0];
                    6'h0B: o_w_addr_end   <= s_axil_wdata[ADDR_WIDTH-1:0];
                    6'h0C: begin
                        // CTRL, command register and both bits are a 1-cycle pulse
                        o_route_en  <= s_axil_wdata[0]; // START the systolic array
                        o_reg_clear <= s_axil_wdata[1]; // flush the routers 
                    end
                    // 6'h0D: STATUS, read only so ignore here for the writes
                    6'h0E: begin
                        // 0x38 where the CPU writes the shift amount (integer)
                        // only the DATA_WIDTH bits are used, and the upper bits are ignored
                        o_quant_sh        <= s_axil_wdata[DATA_WIDTH-1:0];     
                    end
                    6'h0F: begin
                        // 0x3C where 16 bit fixed point multipler for DATA_WIDTH
                        // CPU writes the full value in wdata[15:0]
                        // Value is pre-computed, stored in the DRAM
                        // Write this before the route_en bit
                        o_quant_mult      <= s_axil_wdata[2*DATA_WIDTH-1:0];
                    end
                    default: ; // no specific default, unmapped
                endcase
                s_axil_bvalid <= 1;
                s_axil_bresp  <= 2'b00; // OK
            end

            if(s_axil_bvalid && s_axil_bready)
                s_axil_bvalid <= 0; // reset 
        end
    end

    // --- AXI Read Path, CPU reading STATUS to check for o_done --- 
    // CPU reads STATUS REG (0X34), to poll o_done after writing route_en bit
    // Can also read back any of the configuration registers to verify
    always @(posedge clk or negedge nrst)begin : p_axil_rd
        if(!nrst)begin
            s_axil_arready <= 0; s_axil_rvalid <= 0;
            s_axil_rdata  <= 0; s_axil_rresp <= 0; 
        end else begin
            s_axil_arready <= 0;
            // Do not accept a new AR while still waiting for the response
            // RVALID should hold until the RREADY is seen
            if(s_axil_arvalid && !s_axil_rvalid) begin
                s_axil_arready <= 1;
                case(s_axil_araddr[7:2])
                    6'h00: s_axil_rdata <= {31'd0, o_conv_mode};
                    6'h01: s_axil_rdata <= {30'd0, o_p_mode};
                    6'h02: s_axil_rdata <= {{(32-ADDR_WIDTH){1'b0}}, o_i_size};
                    6'h03: s_axil_rdata <= {{(32-ADDR_WIDTH){1'b0}}, o_i_c_size};
                    6'h04: s_axil_rdata <= {{(32-ADDR_WIDTH){1'b0}}, o_o_c_size};
                    6'h05: s_axil_rdata <= {{(32-ADDR_WIDTH){1'b0}}, o_o_size};
                    6'h06: s_axil_rdata <= {{(32-ADDR_WIDTH){1'b0}}, o_stride};
                    6'h07: s_axil_rdata <= {{(32-ADDR_WIDTH){1'b0}}, o_depth_mult};
                    6'h08: s_axil_rdata <= {{(32-ADDR_WIDTH){1'b0}}, o_i_start_addr};
                    6'h09: s_axil_rdata <= {{(32-ADDR_WIDTH){1'b0}}, o_i_addr_end};
                    6'h0A: s_axil_rdata <= {{(32-ADDR_WIDTH){1'b0}}, o_w_start_addr};
                    6'h0B: s_axil_rdata <= {{(32-ADDR_WIDTH){1'b0}}, o_w_addr_end};
                    6'h0C: s_axil_rdata <= 32'd0;           // CTRL is write-only
                    6'h0D: s_axil_rdata <= {31'd0, i_done}; // STATUS — CPU polls here
                    6'h0E: s_axil_rdata <= {{(32-DATA_WIDTH){1'b0}}, o_quant_sh};
                    6'h0F: s_axil_rdata <= {{(32 - 2*DATA_WIDTH){1'b0}}, o_quant_mult};
    
                    default: s_axil_rdata <= 32'hB0BACAFE;  // unmapped, debug sentinel
                endcase
                s_axil_rresp  <= 2'b00;
                s_axil_rvalid <= 1;
            end 
            if(s_axil_rvalid && s_axil_rready)
                s_axil_rvalid <= 0;
        end 
    end 

endmodule
