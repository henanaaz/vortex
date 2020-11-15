`include "VX_platform.vh"

module VX_avs_wrapper #(
    parameter AVS_DATAW     = 1, 
    parameter AVS_ADDRW     = 1,    
    parameter AVS_BURSTW    = 1,
    parameter AVS_BANKS     = 1,
    parameter REQ_TAGW      = 1,
    parameter RD_QUEUE_SIZE = 1,
        
    parameter AVS_BYTEENW   = (AVS_DATAW / 8),
    parameter RD_QUEUE_ADDRW= $clog2(RD_QUEUE_SIZE+1),
    parameter AVS_BANKS_BITS= $clog2(AVS_BANKS)
) (
    input wire                      clk,
    input wire                      reset,

    // AVS bus
    output  wire [AVS_DATAW-1:0]    avs_writedata,
    input   wire [AVS_DATAW-1:0]    avs_readdata,
    output  wire [AVS_ADDRW-1:0]    avs_address,
    input   wire                    avs_waitrequest,
    output  wire                    avs_write,
    output  wire                    avs_read,
    output  wire [AVS_BYTEENW-1:0]  avs_byteenable,
    output  wire [AVS_BURSTW-1:0]   avs_burstcount,
    input                           avs_readdatavalid,
    output wire [AVS_BANKS_BITS-1:0] avs_bankselect,

    // DRAM request
    input wire                      dram_req_valid,
    input wire                      dram_req_rw,    
    input wire [AVS_BYTEENW-1:0]    dram_req_byteen,    
    input wire [AVS_ADDRW-1:0]      dram_req_addr,
    input wire [AVS_DATAW-1:0]      dram_req_data,
    input wire [REQ_TAGW-1:0]       dram_req_tag,
    output  wire                    dram_req_ready,

    // DRAM response    
    output wire                     dram_rsp_valid,        
    output wire [AVS_DATAW-1:0]     dram_rsp_data,
    output wire [REQ_TAGW-1:0]      dram_rsp_tag,
    input wire                      dram_rsp_ready
);
    reg [AVS_BANKS_BITS-1:0] avs_bankselect_r;
    reg [AVS_BURSTW-1:0]     avs_burstcount_r;

    wire avs_rtq_push = !dram_req_rw && dram_req_valid && dram_req_ready;
    wire avs_rtq_pop  = dram_rsp_valid && dram_rsp_ready;

    wire avs_rdq_push = avs_readdatavalid;
    wire avs_rdq_pop  = avs_rtq_pop;
    wire avs_rdq_empty;

    reg [RD_QUEUE_ADDRW-1:0]  avs_pending_reads;
    wire [RD_QUEUE_ADDRW-1:0] avs_pending_reads_n;    

    assign avs_pending_reads_n = avs_pending_reads 
                               + RD_QUEUE_ADDRW'((avs_rtq_push && !avs_rdq_pop) ? 1 :
                                                 (avs_rdq_pop && !avs_rtq_push) ? -1 : 0);

    always @(posedge clk) begin
        if (reset) begin                
            avs_burstcount_r  <= 1;
            avs_bankselect_r  <= 0;
            avs_pending_reads <= 0;
        end else begin
            avs_pending_reads <= avs_pending_reads_n; 
        end
    end
    
    VX_generic_queue #(
        .DATAW (REQ_TAGW),
        .SIZE  (RD_QUEUE_SIZE)
    ) rd_req_queue (
        .clk      (clk),
        .reset    (reset),
        .push     (avs_rtq_push),
        .data_in  (dram_req_tag),
        .pop      (avs_rtq_pop),
        .data_out (dram_rsp_tag),
        `UNUSED_PIN (empty),
        `UNUSED_PIN (full),
        `UNUSED_PIN (size)
    );

    VX_generic_queue #(
        .DATAW (AVS_DATAW),
        .SIZE  (RD_QUEUE_SIZE)
    ) rd_rsp_queue (
        .clk      (clk),
        .reset    (reset),
        .push     (avs_rdq_push),
        .data_in  (avs_readdata),
        .pop      (avs_rdq_pop),
        .data_out (dram_rsp_data),
        .empty    (avs_rdq_empty),
        `UNUSED_PIN (full),
        `UNUSED_PIN (size)
    );

    assign avs_read       = dram_req_valid && !dram_req_rw;
    assign avs_write      = dram_req_valid && dram_req_rw;
    assign avs_address    = dram_req_addr;
    assign avs_byteenable = dram_req_byteen;
    assign avs_writedata  = dram_req_data;
    assign dram_req_ready = !avs_waitrequest 
                         && (avs_pending_reads < RD_QUEUE_SIZE);
    assign avs_burstcount = avs_burstcount_r;
    assign avs_bankselect = avs_bankselect_r;

    assign dram_rsp_valid = !avs_rdq_empty;   

`ifdef DBG_PRINT_AVS
    always @(posedge clk) begin
        if (dram_req_valid && dram_req_ready) begin
            if (dram_req_rw) 
                $display("%t: AVS Wr Req: addr=%0h, byteen=%0h, tag=%0h, data=%0h", $time, `DRAM_TO_BYTE_ADDR(avs_address), avs_byteenable, dram_req_tag, avs_writedata);                
            else    
                $display("%t: AVS Rd Req: addr=%0h, byteen=%0h, tag=%0h, pending=%0d", $time, `DRAM_TO_BYTE_ADDR(avs_address), avs_byteenable, dram_req_tag, avs_pending_reads_n);
        end   
        if (dram_rsp_valid && dram_rsp_ready) begin
            $display("%t: AVS Rd Rsp: data=%0h, pending=%0d", $time, avs_readdata, avs_pending_reads_n);
        end
    end
`endif

endmodule