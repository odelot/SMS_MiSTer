// DDRAM Arbiter for SMS RetroAchievements
//
// Sits between an optional primary master and the RA mirror on the DDRAM bus.
// For SMS, the primary master is unused (tied off), so RA gets exclusive access.
// RA mirror uses toggle req/ack protocol, converted to Avalon-like single beats.
//
// Reused from MegaDrive arbiter — primary master (MDP) side is tied idle by SMS.sv.

module ddram_arb_sms (
	input         clk,            // clk_sys

	// Physical DDRAM interface (directly to top-level DDRAM ports)
	input         PHY_BUSY,
	output  [7:0] PHY_BURSTCNT,
	output [28:0] PHY_ADDR,
	input  [63:0] PHY_DOUT,
	input         PHY_DOUT_READY,
	output        PHY_RD,
	output [63:0] PHY_DIN,
	output  [7:0] PHY_BE,
	output        PHY_WE,

	// Primary master DDRAM interface (Avalon, unused in SMS — tied off)
	output        MDP_BUSY,
	input   [7:0] MDP_BURSTCNT,
	input  [28:0] MDP_ADDR,
	output [63:0] MDP_DOUT,
	output        MDP_DOUT_READY,
	input         MDP_RD,
	input  [63:0] MDP_DIN,
	input   [7:0] MDP_BE,
	input         MDP_WE,

	// RetroAchievements write channel (toggle req/ack)
	input  [28:0] ra_wr_addr,
	input  [63:0] ra_wr_din,
	input   [7:0] ra_wr_be,
	input         ra_wr_req,
	output reg    ra_wr_ack,

	// RetroAchievements read channel (toggle req/ack)
	input  [28:0] ra_rd_addr,
	input         ra_rd_req,
	output reg    ra_rd_ack,
	output reg [63:0] ra_rd_dout
);

// Synchronize RA toggle signals (may come from different clock domain)
reg ra_wr_req_s1, ra_wr_req_s2;
reg ra_rd_req_s1, ra_rd_req_s2;
always @(posedge clk) begin
	ra_wr_req_s1 <= ra_wr_req; ra_wr_req_s2 <= ra_wr_req_s1;
	ra_rd_req_s1 <= ra_rd_req; ra_rd_req_s2 <= ra_rd_req_s1;
end

// State machine
localparam S_PASSTHRU = 2'd0;
localparam S_RA_WR    = 2'd1;
localparam S_RA_RD    = 2'd2;
localparam S_RA_WAIT  = 2'd3;

reg [1:0] state = S_PASSTHRU;

// Track pending primary master read bursts
reg        mdp_rd_active = 0;
reg  [7:0] mdp_burst_cnt = 0;

// Combinational mux
assign PHY_BURSTCNT = (state == S_PASSTHRU) ? MDP_BURSTCNT : 8'd1;
assign PHY_ADDR     = (state == S_PASSTHRU) ? MDP_ADDR     :
                      (state == S_RA_WR)    ? ra_wr_addr   : ra_rd_addr;
assign PHY_DIN      = (state == S_PASSTHRU) ? MDP_DIN      : ra_wr_din;
assign PHY_BE       = (state == S_PASSTHRU) ? MDP_BE       :
                      (state == S_RA_WR)    ? ra_wr_be     : 8'hFF;
assign PHY_WE       = (state == S_RA_WR)    ? 1'b1         :
                      (state == S_PASSTHRU) ? MDP_WE       : 1'b0;
assign PHY_RD       = (state == S_RA_RD)    ? 1'b1         :
                      (state == S_PASSTHRU) ? MDP_RD       : 1'b0;

assign MDP_BUSY       = (state != S_PASSTHRU) ? 1'b1 : PHY_BUSY;
assign MDP_DOUT       = PHY_DOUT;
assign MDP_DOUT_READY = (state == S_PASSTHRU) ? PHY_DOUT_READY : 1'b0;

always @(posedge clk) begin
	// Track pending primary master read bursts
	if (state == S_PASSTHRU) begin
		if (MDP_RD && !PHY_BUSY) begin
			mdp_rd_active <= 1'b1;
			mdp_burst_cnt <= MDP_BURSTCNT;
		end
		if (mdp_rd_active && PHY_DOUT_READY) begin
			if (mdp_burst_cnt <= 8'd1)
				mdp_rd_active <= 1'b0;
			else
				mdp_burst_cnt <= mdp_burst_cnt - 8'd1;
		end
	end

	case (state)
	S_PASSTHRU: begin
		// Only steal bus when primary master is idle
		if (!MDP_WE && !MDP_RD && !PHY_BUSY && !mdp_rd_active) begin
			if (ra_wr_req_s2 != ra_wr_ack)
				state <= S_RA_WR;
			else if (ra_rd_req_s2 != ra_rd_ack)
				state <= S_RA_RD;
		end
	end

	S_RA_WR: begin
		if (!PHY_BUSY) begin
			ra_wr_ack <= ra_wr_req_s2;
			state <= S_PASSTHRU;
		end
	end

	S_RA_RD: begin
		if (!PHY_BUSY) begin
			state <= S_RA_WAIT;
		end
	end

	S_RA_WAIT: begin
		if (PHY_DOUT_READY) begin
			ra_rd_dout <= PHY_DOUT;
			ra_rd_ack <= ra_rd_req_s2;
			state <= S_PASSTHRU;
		end
	end
	endcase
end

endmodule
