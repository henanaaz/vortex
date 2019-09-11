
`include "VX_define.v"

module VX_fetch (
	input  wire           clk,
	input  wire           reset,
	input  wire           in_memory_delay,
	input  wire           in_branch_stall,
	input  wire           in_fwd_stall,
	input  wire           in_branch_stall_exe,
	input  wire           in_gpr_stall,
	VX_icache_response_inter icache_response,
	VX_icache_request_inter icache_request,

	output wire           out_delay,
	output wire           out_ebreak,
	output wire[`NW_M1:0] out_which_wspawn,
	VX_jal_response_inter    VX_jal_rsp,
	VX_branch_response_inter VX_branch_rsp,
	VX_inst_meta_inter       fe_inst_meta_fd,
	VX_warp_ctl_inter        VX_warp_ctl
);

		wire           in_change_mask     = VX_warp_ctl.change_mask;
		wire           in_wspawn          = VX_warp_ctl.wspawn;
		wire[31:0]     in_wspawn_pc       = VX_warp_ctl.wspawn_pc;
		wire           in_ebreak          = VX_warp_ctl.ebreak;
		wire[`NW_M1:0] in_decode_warp_num = VX_warp_ctl.warp_num;


		wire in_freeze = out_delay || in_memory_delay;


		// wire in_thread_mask[`NT_M1:0];

		// genvar ind;
		// for (ind = 0; ind <= `NT_M1; ind = ind + 1) assign in_thread_mask[ind] = VX_warp_ctl.thread_mask[ind];



		reg       stall;
		reg[31:0] out_PC;

		reg[`NW_M1:0] warp_num;
		reg[`NW_M1:0] warp_state;
		reg[`NW_M1:0] warp_count;

		// reg[31:0] num_ecalls;

		initial begin
			warp_num   = 0;
			warp_state = 0;
			// num_ecalls = 0;
			warp_count = 1;
		end


		// always @(posedge clk) begin
		// 	if (in_ebreak) begin
		// 		num_ecalls <= num_ecalls + 1;
		// 		$display("--------> New num_ecalls = %h", num_ecalls+1);
		// 	end
		// end

		wire add_warp    = in_wspawn && !in_ebreak && !in_gpr_stall;
		wire remove_warp = in_ebreak && !in_wspawn && !in_gpr_stall;

		always @(posedge clk or posedge reset) begin
			if (reset || (warp_num >= warp_state) || remove_warp || add_warp) begin
				warp_num   <= 0;
			end else begin
				warp_num   <= warp_num + 1;
			end

			if (add_warp) begin
				warp_state <= warp_state + 1;
				warp_count <= warp_count + 1;
				// $display("Adding a new warp %h", warp_state+1);
			end else if (remove_warp) begin // No removing, just invalidating
				warp_count <= warp_count - 1;
				// $display("Removing a warp %h %h", in_decode_warp_num, warp_count);
				if (warp_count == 2) begin
					// $display("&&&&&&&&&&&&& STATE 0");
					warp_state <= 0;
				end
			end
		end

		assign out_ebreak = (in_decode_warp_num == 0) && in_ebreak;


		assign stall = in_gpr_stall || in_branch_stall || in_fwd_stall || in_branch_stall_exe || in_freeze;

		assign out_which_wspawn = (warp_state+1);

		`ifdef ONLY

		`else 

			wire[`NW-1:0][31:0] warp_glob_pc;
			wire[`NW-1:0][`NT_M1:0] warp_glob_valid;
			genvar cur_warp;
			generate
				for (cur_warp = 0; cur_warp < `NW; cur_warp = cur_warp + 1)
				begin
					wire       warp_zero_change_mask = in_change_mask && (in_decode_warp_num == cur_warp);
					wire       warp_zero_jal         = VX_jal_rsp.jal            && (VX_jal_rsp.jal_warp_num       == cur_warp);
					wire       warp_zero_branch      = VX_branch_rsp.branch_dir  && (VX_branch_rsp.branch_warp_num == cur_warp);
					wire       warp_zero_stall       = stall          || (warp_num != cur_warp);
					wire       warp_zero_wspawn      = (cur_warp == 0) ? 0 : (in_wspawn && ((warp_state+1) == cur_warp));
					wire[31:0] warp_zero_wspawn_pc   = in_wspawn_pc;
					wire       warp_zero_remove      = remove_warp && (in_decode_warp_num == cur_warp);

					VX_warp VX_Warp(
						.clk           (clk),
						.reset         (reset),
						.stall         (warp_zero_stall),
						.remove        (warp_zero_remove),
						.in_thread_mask(VX_warp_ctl.thread_mask),
						.in_change_mask(warp_zero_change_mask),
						.in_jal        (warp_zero_jal),
						.in_jal_dest   (VX_jal_rsp.jal_dest),
						.in_branch_dir (warp_zero_branch),
						.in_branch_dest(VX_branch_rsp.branch_dest),
						.in_wspawn     (warp_zero_wspawn),
						.in_wspawn_pc  (warp_zero_wspawn_pc),
						.out_PC        (warp_glob_pc[cur_warp]),
						.out_valid     (warp_glob_valid[cur_warp])
						);
				end
			endgenerate



			reg[31:0] out_PC_var;
			reg[`NT_M1:0]  out_valid_var;

			always @(*) begin : help
				integer g;
				integer h;
				for (g = 0; g < `NW; g = g + 1)
				begin
					if (warp_num == g[`NW_M1:0])
					begin
						 out_PC_var    = warp_glob_pc[g][31:0];
						 for (h = 0; h < `NT; h = h + 1) out_valid_var[h] = warp_glob_valid[g][h];
					end
				
				end
			end

			assign out_PC    = out_PC_var;

		`endif

	


		assign icache_request.pc_address = out_PC;

		assign out_delay       = 0;

		assign fe_inst_meta_fd.warp_num = warp_num;

		genvar index;
		for (index = 0; index <= `NT_M1; index = index + 1) assign fe_inst_meta_fd.valid[index] = out_valid_var[index];

		assign fe_inst_meta_fd.instruction = (stall) ? 32'b0 : icache_response.instruction;;
		assign fe_inst_meta_fd.inst_pc     = out_PC;

		// always @(*) begin
		// 	$display("fetch: icache_request: %x", out_PC);
		// end

endmodule