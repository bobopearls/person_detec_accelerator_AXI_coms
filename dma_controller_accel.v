// DMA Controller for ML Accelerators
// AXI4-Lite Slave, with an AXI4 Master and then different channels for communication
// You can search different youtube tutorials that talk about the basics of AXI, and the reads n write protocols

//     `define DATA_WIDTH 8
//     `define SPAD_DATA_WIDTH 32
//     `define SPAD_N 4
//     `define ADDR_WIDTH 16
//     `define ROWS 8
//     `define COLUMNS 8
//     `define MISO_DEPTH 16
//     `define MPP_DEPTH 9

`timescale 1 ns / 1 ps
module DMA_Controller #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 64,   // find the AXI Master Data Width
    parameter NUM_CHANNEL = 4,   // need to tell which of the blocks are being spoken to 
    parameter MAX_AXI_BURST = 32 // double check the ARLen and AWLen max bits
)(
    input wire clk, nrst,

    // --- AXI4-Lite Slave, CPU communication ---
    // Note: lower case signals are for axi lite, the upper case are all interface
    // then all the ready signals are registers that will communicate with the RISC-V processor 
    input wire [15:0] s_axil_awaddr, // valid addresses and valid for writing operations
    input wire        s_axil_awvalid, 
    output reg        s_axil_awready,
    input wire [31:0] s_axil_wdata,  // actual data writing and to see if if ready to receive valid data
    input wire        s_axil_wvalid,
    output reg        s_axil_wready,
    output reg [1:0]  s_axil_bresp,  // to see if the write was successful or not in the burst
    output reg        s_axil_bvalid, 
    input wire        s_axil_bready,

    // --- AXI4 Master Interface, where we move the data around in the bus ---
    // Read Channels: from the memory to the rest
    output reg [ADDR_WIDTH-1 : 0] M_AXI_ARADDR, // source address that the DMA will read from
    output reg [7:0]              M_AXI_ARLEN,  // burst length, how many beats to send 
    output reg                    M_AXI_ARVALID,
    input wire                    M_AXI_ARREADY,
    input wire [DATA_WIDTH-1 : 0] M_AXI_RDATA,  // actual data from the memory that will be sent to the bus
    input wire                    M_AXI_RVALID,
    input wire                    M_AXI_RLAST,  // indicate the last data in the AXI burst
    output reg                    M_AXI_RREADY,

    // Write Channels: sending data to the memory
    output reg [ADDR_WIDTH-1 : 0] M_AXI_AWADDR, // destination address to save the data in 
    output reg [7:0]              M_AXI_AWLEN,
    output reg                    M_AXI_AWVALID,
    input wire                    M_AXI_AWREADY,
    output reg [DATA_WIDTH-1 : 0] M_AXI_WDATA,  // data that we will write to the destination address
    output reg                    M_AXI_WVALID,
    output reg                    M_AXI_WLAST,  // what the last piece of data we will write to is in the burst
    input wire                    M_AXI_WREADY,
    input wire [1:0]              M_AXI_BRESP, 
    input wire                    M_AXI_BVALID, // confirm that the data in the burst is valid and ready
    output reg                    M_AXI_BREADY

    // --- CPU Interrupt Signal ---
    // after the DMA is done, 
    output reg [NUM_CHANNEL-1 : 0] interrupt_req
);

    // --- Internal Registers, will hold the 


endmodule