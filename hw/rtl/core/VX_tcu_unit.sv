// Copyright © 2019-2023
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

`include "VX_define.vh"

module VX_tcu_unit import VX_gpu_pkg::*; #(
    parameter CORE_ID = 0
) (
    input wire clk,
    input wire reset,

    VX_dispatch_if.slave    dispatch_if [`ISSUE_WIDTH],
    VX_tcu_to_csr_if.master tcu_to_csr_if,
    VX_tcu_to_lsu_if.master tcu_to_lsu_if,  //Check size

    VX_commit_if.master     commit_if [`ISSUE_WIDTH]
);
    `UNUSED_PARAM (CORE_ID)
    localparam BLOCK_SIZE = `ISSUE_WIDTH;
    localparam NUM_LANES  = `NUM_THREADS;
    
    localparam PID_BITS   = `CLOG2(`NUM_THREADS / NUM_LANES);
    localparam PID_WIDTH  = `UP(PID_BITS);
    //localparam TAG_WIDTH  = `LOG2UP(`NUM_LANES);
    localparam PARTIAL_BW = (BLOCK_SIZE != `ISSUE_WIDTH) || (NUM_LANES != `NUM_THREADS);

    reg [1:0] count;

    VX_execute_if #(
        .NUM_LANES (NUM_LANES)
    ) execute_if[BLOCK_SIZE]();

    `RESET_RELAY (dispatch_reset, reset);

    VX_dispatch_unit #(
        .BLOCK_SIZE (BLOCK_SIZE),
        .NUM_LANES  (NUM_LANES),
        .OUT_REG    (PARTIAL_BW ? 1 : 0)
    ) dispatch_unit (
        .clk        (clk),
        .reset      (dispatch_reset),
        .dispatch_if(dispatch_if),
        .execute_if (execute_if)
    );

    VX_commit_if #(
        .NUM_LANES (NUM_LANES)
    ) commit_block_if[BLOCK_SIZE]();

    for (genvar block_idx = 0; block_idx < BLOCK_SIZE; ++block_idx) begin
        `UNUSED_VAR (execute_if[block_idx].data.tid)
        `UNUSED_VAR (execute_if[block_idx].data.wb)
        `UNUSED_VAR (execute_if[block_idx].data.use_PC)
        `UNUSED_VAR (execute_if[block_idx].data.use_imm)

        // Store request info
        wire tcu_req_valid, tcu_req_ready;
        wire tcu_rsp_valid, tcu_rsp_ready;    
        wire [NUM_LANES-1:0][`XLEN-1:0] tcu_rsp_result;
        //fflags_t tcu_rsp_fflags;
        //wire tcu_rsp_has_fflags;

        wire [PID_WIDTH-1:0] tcu_req_tag, tcu_rsp_tag;    
        wire mdata_full;

        wire [`INST_FMT_BITS-1:0] tcu_fmt = execute_if[block_idx].data.imm[`INST_FMT_BITS-1:0];
        wire [`INST_FRM_BITS-1:0] tcu_frm = execute_if[block_idx].data.op_mod[`INST_FRM_BITS-1:0];

        wire execute_fire = execute_if[block_idx].valid && execute_if[block_idx].ready;
        wire tcu_rsp_fire = tcu_rsp_valid && tcu_rsp_ready;

        /*
        VX_index_buffer #(
            .DATAW  (`UUID_WIDTH + `NW_WIDTH + NUM_LANES + `XLEN + `NR_BITS + PID_WIDTH + 1 + 1),
            .SIZE   (`FPUQ_SIZE)
        ) tag_store (
            .clk          (clk),
            .reset        (reset),
            .acquire_en   (execute_fire), 
            .write_addr   (tcu_req_tag), 
            .write_data   ({execute_if[block_idx].data.uuid, execute_if[block_idx].data.wid, execute_if[block_idx].data.tmask, execute_if[block_idx].data.PC, execute_if[block_idx].data.rd, execute_if[block_idx].data.pid, execute_if[block_idx].data.sop, execute_if[block_idx].data.eop}),
            .read_data    ({tcu_rsp_uuid, tcu_rsp_wid, tcu_rsp_tmask, tcu_rsp_PC, tcu_rsp_rd, tcu_rsp_pid, tcu_rsp_sop, tcu_rsp_eop}),
            .read_addr    (tcu_rsp_tag),
            .release_en   (tcu_rsp_fire), 
            .full         (mdata_full),
            `UNUSED_PIN (empty)
        );
        */

        // resolve dynamic FRM from CSR   
        wire [`INST_FRM_BITS-1:0] tcu_req_frm; 
        //`ASSIGN_BLOCKED_WID (tcu_to_csr_if[block_idx].read_wid, execute_if[block_idx].data.wid, block_idx, `NUM_LANES)
        //assign tcu_req_frm = (execute_if[block_idx].data.op_type != `INST_FPU_MISC 
        //                   && tcu_frm == `INST_FRM_DYN) ? tcu_to_csr_if[block_idx].read_frm : tcu_frm;

        // submit FPU request

        assign tcu_req_valid = execute_if[block_idx].valid && ~mdata_full;
        assign execute_if[block_idx].ready = tcu_req_ready && ~mdata_full;
  

    `RESET_RELAY (tcu_reset, reset);
    reg [2:0] state;
    reg [2:0] next_state;
    reg [1:0] index_ctr;

    reg tensor_load_done;
    reg tensor_store_start;
    reg tensor_store_done;
    reg mult_done;
    reg [31:0] data_in_a;
    reg [31:0] data_in_b;
    reg [31:0] data_out_c;
    
    always @(posedge clk)
    begin
        if (~tcu_reset) 
        begin
            state        = 3'b0;
            next_state   = 3'b0;
        end
        else
        begin
            next_state <= state;  
        end
    end

    always @(posedge clk) 
    begin
        if (~reset) 
        begin
            //reset LSU/gather signals
            tensor_load_start = 1'b0;
            data_in_a = 32'b0;
            data_in_b = 32'b0;
            state = 3'b0;
            next_state = 3'b0;
        end
        else
        begin
            case (state)
                3'b000 : //data from dcache to GPR
                    begin
                        if(execute_if.valid) 
                        begin
                            //valid to lsu
                            tcu_to_lsu_if.ready = 1'b1;
                            tcu_to_lsu_if.load  = 1'b1;
                            //tcu_to_lsu_if.addr = execute_if.rs1_data; //Check this 

                            //Resp from lsu
                            if(tcu_to_lsu_if.valid) 
                            begin
                                tcu_to_lsu_if.ready = 1'b0;
                                //next_state = 3'b1;
                                next_state = 3'b010;
                                tensor_load_start = 1'b1; 
                                
                            end
                        end
                    end
                3'b001 : //data from GPR to CSR - UNUSED
                    begin
                        //...
                        //if GPR to CSR is done
                        next_state = 3'b010;
                        tensor_load_start = 1'b1; 
                    end
                3'b010 : //data from CSR to TCU
                    begin 
                        //move data from csr to tcu
                        tcu_to_csr_if[block_idx].read_enable = 1'b1;
                        
                        //Get correct
                        data_in_a = tcu_to_csr_if[block_idx].read_data_a;
                        data_in_b = tcu_to_csr_if[block_idx].read_data_b;

                        //if CSR to TCU is done
                        if (tensor_load_done) 
                        begin
                            next_state = 3'b011;
                        end
                    end
                3'b011 : //Wait for mult
                    begin 
                        if (mult_done) 
                            next_state = 3'b100;
                    end
                3'b100 : //data from TCU to CSR
                    begin
                        if (tensor_store_done != 1)
                        begin
                            //...
                            tcu_to_csr_if[block_idx].write_enable = 1'b1;
                            tcu_to_csr_if[block_idx].write_data   = data_out_c;
                        end
                        if (index_ctr == 2'b11)
                            next_state = 3'b110;
                    end
                3'b101 : //data from CSR to Reg - UNUSED
                    begin
                        //...
                        //if CSR to Reg is done
                        next_state = 3'b110;
                    end
                3'b110 : //data from reg to DCache
                    begin
                        //...
                        tcu_to_lsu_if.ready = 1'b1;
                        tcu_to_lsu_if.load  = 1'b0;                 //Store
                        //tcu_to_lsu_if.addr = execute_if.rs1_data; //Check this 

                        //if LSU is done
                        if(tcu_to_lsu_if.valid)
                        begin
                            tcu_to_lsu_if.ready = 1'b0;
                            next_state = 3'b000;
                        end

                        //Gather valid is set high
                    end
            endcase
        end
    end

    VX_tensor_unit #( 
        .NUM_LANES      (NUM_LANES)
    ) tcu_unit 
    (
        .clk                        (clk),
        .reset                      (tcu_reset),
        .tensor_load_start          (tensor_load_start),
        .tensor_load_done           (tensor_load_done),
        .tensor_store_start         (tensor_store_start),
        .tensor_store_done          (tensor_store_done),
        .tensor_execute_done        (mult_done),
        .data_in_a                  (data_in_a),
        .data_in_b                  (data_in_b),
        .data_out_c                 (data_out_c)
    );
            

        // handle FPU response
        /*
        fflags_t tcu_rsp_fflags_q;

        if (PID_WIDTH != 0) begin
            fflags_t tcu_rsp_fflags_r;
            always @(posedge clk) begin
                if (reset) begin
                    tcu_rsp_fflags_r <= '0;
                end else if (tcu_rsp_fire) begin
                    tcu_rsp_fflags_r <= tcu_rsp_eop ? '0 : (tcu_rsp_fflags_r | tcu_rsp_fflags_r);
                end
            end
            assign tcu_rsp_fflags_q = tcu_rsp_fflags_r | tcu_rsp_fflags_r;
        end else begin
            assign tcu_rsp_fflags_q = tcu_rsp_fflags_q;
        end
        
        assign tcu_to_csr_if[block_idx].write_enable = tcu_rsp_fire && tcu_rsp_eop && tcu_rsp_fflags_q;
        `ASSIGN_BLOCKED_WID (tcu_to_csr_if[block_idx].write_wid, tcu_rsp_wid, block_idx, `NUM_LANES)
        assign tcu_to_csr_if[block_idx].write_fflags = tcu_rsp_fflags_q;

        // send response
        */

        VX_elastic_buffer #(
            .DATAW (`UUID_WIDTH + `NW_WIDTH + NUM_LANES + `XLEN + `NR_BITS + (NUM_LANES * `XLEN) + PID_WIDTH + 1 + 1),
            .SIZE  (0)
        ) rsp_buf (
            .clk       (clk),
            .reset     (reset),
            .valid_in  (tcu_rsp_valid),
            .ready_in  (tcu_rsp_ready),
            .data_in   ({tcu_rsp_uuid, tcu_rsp_wid, tcu_rsp_tmask, tcu_rsp_PC, tcu_rsp_rd, tcu_rsp_result, tcu_rsp_pid, tcu_rsp_sop, tcu_rsp_eop}),
            .data_out  ({commit_block_if[block_idx].data.uuid, commit_block_if[block_idx].data.wid, commit_block_if[block_idx].data.tmask, commit_block_if[block_idx].data.PC, commit_block_if[block_idx].data.rd, commit_block_if[block_idx].data.data, commit_block_if[block_idx].data.pid, commit_block_if[block_idx].data.sop, commit_block_if[block_idx].data.eop}),
            .valid_out (commit_block_if[block_idx].valid),
            .ready_out (commit_block_if[block_idx].ready)
        );
        assign commit_block_if[block_idx].data.wb = 1'b1;
    end

    `RESET_RELAY (commit_reset, reset);

    VX_gather_unit #(
        .BLOCK_SIZE (BLOCK_SIZE),
        .NUM_LANES  (NUM_LANES),
        .OUT_REG    (PARTIAL_BW ? 3 : 0)
    ) gather_unit (
        .clk           (clk),
        .reset         (commit_reset),
        .commit_in_if  (commit_block_if),
        .commit_out_if (commit_if)
    );

endmodule
