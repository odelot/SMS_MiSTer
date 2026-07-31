// RetroAchievements RAM Mirror for Sega Master System / Game Gear — Option C
//
// Each VBlank, reads a list of specific addresses from DDRAM (written by ARM),
// fetches byte values from System RAM or NVRAM, and writes them back to DDRAM.
//
// SMS/GG memory map for rcheevos:
//   0x0000–0x1FFF : System RAM (8KB, Z80 $C000-$DFFF)
//   0x2000–0x9FFF : Cartridge RAM / NVRAM (up to 32KB)
//   0xA000+       : returns 0
//
// Z80 is 8-bit little-endian — no byte swap is needed (unlike 68K/MegaDrive).
//
// DDRAM Layout (at DDRAM_BASE, ARM phys 0x3D000000):
//   [0x00000] Header:   magic(32) + 0(8) + flags(8) + 0(16)
//   [0x00008] Frame:    frame_counter(32) + 0(32)
//
//   [0x40000] AddrReq:  addr_count(32) + request_id(32)       (ARM → FPGA)
//   [0x40008] Addrs:    addr[0](32) + addr[1](32), ...        (2 per 64-bit word)
//
//   [0x48000] ValResp:  response_id(32) + response_frame(32)  (FPGA → ARM)
//   [0x48008] Values:   val[0..7](8b each), val[8..15], ...   (8 per 64-bit word)

module ra_ram_mirror_sms #(
	parameter [28:0] DDRAM_BASE = 29'h07A00000  // ARM phys 0x3D000000 >> 3
)(
	input             clk,
	input             reset,
	input             vblank,

	// System RAM read interface (dpram port B, 8-bit, 14-bit addr)
	output reg [13:0] sysram_addr,
	input       [7:0] sysram_dout,

	// NVRAM read interface (dpram port B, 8-bit, 15-bit addr)
	output reg [14:0] nvram_addr,
	input       [7:0] nvram_dout,

	// DDRAM write interface (toggle req/ack)
	output reg [28:0] ddram_wr_addr,
	output reg [63:0] ddram_wr_din,
	output reg  [7:0] ddram_wr_be,
	output reg        ddram_wr_req,
	input             ddram_wr_ack,

	// DDRAM read interface (toggle req/ack)
	output reg [28:0] ddram_rd_addr,
	output reg        ddram_rd_req,
	input             ddram_rd_ack,
	input      [63:0] ddram_rd_dout,

	// Status
	output reg        active,
	output reg [31:0] dbg_frame_counter
);

// ======================================================================
// Constants
// ======================================================================
localparam [28:0] ADDRLIST_BASE = DDRAM_BASE + 29'h8000;  // byte offset 0x40000 / 8
localparam [28:0] VALCACHE_BASE = DDRAM_BASE + 29'h9000;  // byte offset 0x48000 / 8
localparam [31:0] SYSRAM_LIMIT  = 32'h2000;               // 8KB System RAM boundary
localparam [31:0] NVRAM_LIMIT   = 32'hA000;               // 8KB sysram + 32KB nvram
localparam [12:0] MAX_ADDRS     = 13'd4096;

// Realtime query mailbox (Tier 1 smart cache). Word offsets = byte offset / 8
// (DDRAM_BASE is already the ARM phys address >> 3).
localparam [28:0] QUERY_CTRL_ADDR = DDRAM_BASE + 29'hA000;  // byte offset 0x50000
localparam [28:0] QUERY_REQ_BASE  = DDRAM_BASE + 29'hA001;  // byte offset 0x50008
localparam [28:0] QUERY_RESP_BASE = DDRAM_BASE + 29'hA011;  // byte offset 0x50088
localparam [28:0] ARM_CFG_ADDR    = DDRAM_BASE + 29'd8;     // byte offset 0x40: ARM config
localparam [3:0]  MAX_RT_QUERIES  = 4'd16;

// ======================================================================
// Clock domain crossing synchronizers for DDRAM ack
// ======================================================================
reg dwr_ack_s1, dwr_ack_s2;
reg drd_ack_s1, drd_ack_s2;
always @(posedge clk) begin
	dwr_ack_s1 <= ddram_wr_ack; dwr_ack_s2 <= dwr_ack_s1;
	drd_ack_s1 <= ddram_rd_ack; drd_ack_s2 <= drd_ack_s1;
end

// ======================================================================
// VBlank edge detection
// ======================================================================
reg vblank_prev;
wire vblank_rising = vblank & ~vblank_prev;
always @(posedge clk) vblank_prev <= vblank;

// Sticky vblank flag: set on the rising edge, cleared once S_IDLE consumes it.
// Prevents a VBlank from being missed while the machine is busy servicing an
// inter-VBlank realtime query.
reg vblank_pending;
always @(posedge clk) begin
	if (reset)
		vblank_pending <= 1'b0;
	else if (vblank_rising)
		vblank_pending <= 1'b1;
	else if (state == S_IDLE && vblank_pending)
		vblank_pending <= 1'b0;
end

// ======================================================================
// State machine
// ======================================================================
localparam S_IDLE        = 6'd0;
localparam S_DD_WR_WAIT  = 6'd1;
localparam S_DD_RD_WAIT  = 6'd2;
localparam S_READ_HDR    = 6'd3;
localparam S_PARSE_HDR   = 6'd4;
localparam S_READ_PAIR   = 6'd5;
localparam S_PARSE_ADDR  = 6'd6;
localparam S_DISPATCH    = 6'd7;
localparam S_BRAM_WAIT   = 6'd8;
localparam S_BRAM_WAIT2  = 6'd9;
localparam S_STORE_VAL   = 6'd10;
localparam S_FLUSH_BUF   = 6'd11;
localparam S_WRITE_RESP  = 6'd12;
localparam S_WR_HDR0     = 6'd13;
localparam S_WR_HDR1     = 6'd14;
localparam S_WR_DBG      = 6'd15;
// ARM config read (latches rtquery_armed once per VBlank)
localparam S_RD_ARMCFG   = 6'd16;
localparam S_PARSE_ARMCFG = 6'd17;
// Realtime query mailbox states
localparam S_QRY_PARSE   = 6'd18;
localparam S_QRY_RD_REQ  = 6'd19;
localparam S_QRY_FETCH   = 6'd20;
localparam S_QRY_DISPATCH = 6'd21;
localparam S_QRY_BRAM_W1 = 6'd22;
localparam S_QRY_BRAM_W2 = 6'd23;
localparam S_QRY_WR_RESP = 6'd24;
localparam S_QRY_WR_CTRL = 6'd25;

reg [5:0]  state;
reg [5:0]  return_state;

reg [31:0] frame_counter;
always @(posedge clk) dbg_frame_counter <= frame_counter;

reg [63:0] rd_data;
reg [31:0] req_count;
reg [31:0] req_id;
reg [12:0] addr_idx;
reg [63:0] addr_word;
reg [31:0] cur_addr;
reg [63:0] collect_buf;
reg  [3:0] collect_cnt;
reg [12:0] val_word_idx;
reg  [7:0] fetch_byte;
reg        use_nvram;      // 1 = reading from NVRAM, 0 = reading from sysram

// Debug counters
reg [15:0] dbg_ok_cnt;
reg [15:0] dbg_oob_cnt;

// Realtime query registers
reg  [7:0] qry_request_seq;
reg  [7:0] qry_last_seen_seq;
reg  [7:0] qry_num;
reg  [3:0] qry_idx;
reg [31:0] qry_addr;
reg  [7:0] qry_num_bytes;
reg [31:0] qry_value;
reg  [2:0] qry_byte_idx;
reg        qry_oob;         // 1 = current query byte is out of range → 0
reg [10:0] qry_poll_timer;
reg        rtquery_armed = 1'b0;  // set by ARM via RA_ARM_CFG_RTQUERY bit

// ======================================================================
// Main state machine
// ======================================================================
always @(posedge clk) begin
	if (reset) begin
		state        <= S_IDLE;
		active       <= 1'b0;
		frame_counter <= 32'd0;
		ddram_wr_req <= dwr_ack_s2;
		ddram_rd_req <= drd_ack_s2;
		qry_last_seen_seq <= 8'd0;
		qry_poll_timer <= 11'd0;
	end
	else begin
		case (state)

		S_IDLE: begin
			active <= 1'b0;
			if (vblank_pending) begin
				active <= 1'b1;
				qry_poll_timer <= 11'd0;
				dbg_ok_cnt  <= 16'd0;
				dbg_oob_cnt <= 16'd0;
				// Write header with busy=1
				ddram_wr_addr <= DDRAM_BASE;
				ddram_wr_din  <= {16'h0100, 8'h01, 8'd0, 32'h52414348};
				ddram_wr_be   <= 8'hFF;
				ddram_wr_req  <= ~ddram_wr_req;
				return_state  <= S_READ_HDR;
				state         <= S_DD_WR_WAIT;
			end
			else if (qry_poll_timer < 11'd2000) begin
				// Space out inter-VBlank mailbox polls (~40us at 50MHz)
				qry_poll_timer <= qry_poll_timer + 11'd1;
			end
			else if (rtquery_armed) begin
				// Poll the realtime query control word for a new batch
				qry_poll_timer <= 11'd0;
				ddram_rd_addr <= QUERY_CTRL_ADDR;
				ddram_rd_req  <= ~ddram_rd_req;
				return_state  <= S_QRY_PARSE;
				state         <= S_DD_RD_WAIT;
			end
			else begin
				qry_poll_timer <= 11'd0;  // rtquery not armed — skip polling
			end
		end

		S_DD_WR_WAIT: begin
			if (ddram_wr_req == dwr_ack_s2)
				state <= return_state;
		end

		S_DD_RD_WAIT: begin
			if (ddram_rd_req == drd_ack_s2) begin
				rd_data <= ddram_rd_dout;
				state   <= return_state;
			end
		end

		S_READ_HDR: begin
			ddram_rd_addr <= ADDRLIST_BASE;
			ddram_rd_req  <= ~ddram_rd_req;
			return_state  <= S_PARSE_HDR;
			state         <= S_DD_RD_WAIT;
		end

		S_PARSE_HDR: begin
			req_id <= rd_data[63:32];
			if (rd_data[31:0] == 32'd0) begin
				req_count <= 32'd0;
				state     <= S_WRITE_RESP;
			end else begin
				req_count    <= (rd_data[31:0] > {19'd0, MAX_ADDRS}) ?
				                {19'd0, MAX_ADDRS} : rd_data[31:0];
				addr_idx     <= 13'd0;
				collect_cnt  <= 4'd0;
				collect_buf  <= 64'd0;
				val_word_idx <= 13'd0;
				state        <= S_READ_PAIR;
			end
		end

		S_READ_PAIR: begin
			ddram_rd_addr <= ADDRLIST_BASE + 29'd1 + {16'd0, addr_idx[12:1]};
			ddram_rd_req  <= ~ddram_rd_req;
			return_state  <= S_PARSE_ADDR;
			state         <= S_DD_RD_WAIT;
		end

		S_PARSE_ADDR: begin
			// SMS/GG: Z80 is 8-bit, addresses are byte-granular.
			// No byte swap needed (unlike 68K MegaDrive).
			if (!addr_idx[0]) begin
				addr_word <= rd_data;
				cur_addr  <= rd_data[31:0];
			end else begin
				cur_addr  <= addr_word[63:32];
			end
			state <= S_DISPATCH;
		end

		S_DISPATCH: begin
			if (cur_addr < SYSRAM_LIMIT) begin
				// System RAM: 8KB at rcheevos 0x0000-0x1FFF
				// SMS uses 13-bit addr (8KB); pad bit 13 to 0
				sysram_addr <= {1'b0, cur_addr[12:0]};
				use_nvram   <= 1'b0;
				dbg_ok_cnt  <= dbg_ok_cnt + 16'd1;
				state       <= S_BRAM_WAIT;
			end
			else if (cur_addr < NVRAM_LIMIT) begin
				// NVRAM / Cartridge RAM: up to 32KB at rcheevos 0x2000-0x9FFF
				nvram_addr  <= cur_addr[14:0] - 15'h2000;
				use_nvram   <= 1'b1;
				dbg_ok_cnt  <= dbg_ok_cnt + 16'd1;
				state       <= S_BRAM_WAIT;
			end
			else begin
				// Out of bounds — return 0
				fetch_byte  <= 8'd0;
				dbg_oob_cnt <= dbg_oob_cnt + 16'd1;
				state       <= S_STORE_VAL;
			end
		end

		S_BRAM_WAIT: begin
			// BRAM latency cycle 1
			state <= S_BRAM_WAIT2;
		end

		S_BRAM_WAIT2: begin
			// SMS/GG: 8-bit buses, no byte selection needed
			fetch_byte <= use_nvram ? nvram_dout : sysram_dout;
			state      <= S_STORE_VAL;
		end

		S_STORE_VAL: begin
			case (collect_cnt[2:0])
				3'd0: collect_buf[ 7: 0] <= fetch_byte;
				3'd1: collect_buf[15: 8] <= fetch_byte;
				3'd2: collect_buf[23:16] <= fetch_byte;
				3'd3: collect_buf[31:24] <= fetch_byte;
				3'd4: collect_buf[39:32] <= fetch_byte;
				3'd5: collect_buf[47:40] <= fetch_byte;
				3'd6: collect_buf[55:48] <= fetch_byte;
				3'd7: collect_buf[63:56] <= fetch_byte;
			endcase
			collect_cnt <= collect_cnt + 4'd1;
			addr_idx    <= addr_idx + 13'd1;

			if (collect_cnt == 4'd7 || (addr_idx + 13'd1 >= req_count[12:0])) begin
				state <= S_FLUSH_BUF;
			end
			else if (addr_idx[0]) begin
				state <= S_READ_PAIR;
			end else begin
				state <= S_PARSE_ADDR;
			end
		end

		S_FLUSH_BUF: begin
			ddram_wr_addr <= VALCACHE_BASE + 29'd1 + {16'd0, val_word_idx};
			ddram_wr_din  <= collect_buf;
			ddram_wr_be   <= (collect_cnt == 4'd8) ? 8'hFF
			                 : ((8'd1 << collect_cnt[2:0]) - 8'd1);
			ddram_wr_req  <= ~ddram_wr_req;
			val_word_idx  <= val_word_idx + 13'd1;
			collect_cnt   <= 4'd0;
			collect_buf   <= 64'd0;

			if (addr_idx >= req_count[12:0]) begin
				return_state <= S_WRITE_RESP;
			end else if (!addr_idx[0]) begin
				return_state <= S_READ_PAIR;
			end else begin
				return_state <= S_PARSE_ADDR;
			end
			state <= S_DD_WR_WAIT;
		end

		S_WRITE_RESP: begin
			ddram_wr_addr <= VALCACHE_BASE;
			ddram_wr_din  <= {frame_counter + 32'd1, req_id};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_WR_HDR0;
			state         <= S_DD_WR_WAIT;
		end

		S_WR_HDR0: begin
			ddram_wr_addr <= DDRAM_BASE;
			ddram_wr_din  <= {16'h0100, 8'h00, 8'd0, 32'h52414348};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_WR_HDR1;
			state         <= S_DD_WR_WAIT;
		end

		S_WR_HDR1: begin
			ddram_wr_addr <= DDRAM_BASE + 29'd1;
			ddram_wr_din  <= {32'd0, frame_counter + 32'd1};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			frame_counter <= frame_counter + 32'd1;
			return_state  <= S_WR_DBG;
			state         <= S_DD_WR_WAIT;
		end

		// Debug word @ DDRAM_BASE+2 (offset 0x10):
		// {version(8), 0(8), ok_cnt(16), oob_cnt(16), 0(16)}
		// Version bumped 0x01 -> 0x02: signals rtquery mailbox support
		// (ra_rtquery_supported() reads byte[0x17] and requires >= 0x02).
		S_WR_DBG: begin
			ddram_wr_addr <= DDRAM_BASE + 29'd2;
			ddram_wr_din  <= {8'h02, 8'd0, dbg_ok_cnt, dbg_oob_cnt, 16'd0};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_RD_ARMCFG;
			state         <= S_DD_WR_WAIT;
		end

		// Read ARM-written config byte once per VBlank. ARM sets
		// RA_ARM_CFG_RTQUERY (bit 0) when rtquery is active; the FPGA latches
		// it to gate inter-VBlank query mailbox polling.
		S_RD_ARMCFG: begin
			ddram_rd_addr <= ARM_CFG_ADDR;
			ddram_rd_req  <= ~ddram_rd_req;
			return_state  <= S_PARSE_ARMCFG;
			state         <= S_DD_RD_WAIT;
		end

		S_PARSE_ARMCFG: begin
			rtquery_armed <= rd_data[0];
			state <= S_IDLE;
		end

		// =============================================================
		// Realtime Query Mailbox (Tier 1 smart cache)
		// ARM writes addresses + a sequence number; FPGA reads System RAM /
		// NVRAM live (inter-VBlank), writes values back, echoes the seq.
		// =============================================================
		S_QRY_PARSE: begin
			// rd_data = control word. byte[0]=request_seq, byte[1]=num_queries
			if (rd_data[7:0] != qry_last_seen_seq && rd_data[15:8] != 8'd0) begin
				qry_request_seq <= rd_data[7:0];
				qry_num         <= (rd_data[15:8] > {4'd0, MAX_RT_QUERIES}) ?
				                   {4'd0, MAX_RT_QUERIES} : rd_data[15:8];
				qry_idx         <= 4'd0;
				state           <= S_QRY_RD_REQ;
			end else begin
				state <= S_IDLE;
			end
		end

		S_QRY_RD_REQ: begin
			ddram_rd_addr <= QUERY_REQ_BASE + {25'd0, qry_idx};
			ddram_rd_req  <= ~ddram_rd_req;
			return_state  <= S_QRY_FETCH;
			state         <= S_DD_RD_WAIT;
		end

		S_QRY_FETCH: begin
			// rd_data = request slot. [31:0]=address, byte[4]=num_bytes (1..4)
			qry_addr      <= rd_data[31:0];
			qry_num_bytes <= (rd_data[39:32] == 8'd0) ? 8'd1 : rd_data[39:32];
			qry_value     <= 32'd0;
			qry_byte_idx  <= 3'd0;
			state         <= S_QRY_DISPATCH;
		end

		S_QRY_DISPATCH: begin
			// Same region routing as the batch path (SMS is byte-granular).
			if (qry_addr < SYSRAM_LIMIT) begin
				sysram_addr <= {1'b0, qry_addr[12:0]};
				use_nvram   <= 1'b0;
				qry_oob     <= 1'b0;
				state       <= S_QRY_BRAM_W1;
			end
			else if (qry_addr < NVRAM_LIMIT) begin
				nvram_addr  <= qry_addr[14:0] - 15'h2000;
				use_nvram   <= 1'b1;
				qry_oob     <= 1'b0;
				state       <= S_QRY_BRAM_W1;
			end
			else begin
				qry_oob <= 1'b1;
				state   <= S_QRY_BRAM_W2;  // out of range → contributes 0
			end
		end

		S_QRY_BRAM_W1: begin
			state <= S_QRY_BRAM_W2;  // BRAM latency cycle
		end

		S_QRY_BRAM_W2: begin
			// Little-endian assembly: byte N occupies bits [N*8 +: 8]
			qry_value <= qry_value |
				({24'd0, (qry_oob ? 8'd0 : (use_nvram ? nvram_dout : sysram_dout))}
				 << (qry_byte_idx * 4'd8));
			qry_byte_idx <= qry_byte_idx + 3'd1;
			if (qry_byte_idx + 3'd1 >= qry_num_bytes[2:0]) begin
				state <= S_QRY_WR_RESP;
			end else begin
				qry_addr <= qry_addr + 32'd1;
				state    <= S_QRY_DISPATCH;
			end
		end

		S_QRY_WR_RESP: begin
			ddram_wr_addr <= QUERY_RESP_BASE + {25'd0, qry_idx};
			ddram_wr_din  <= {32'd0, qry_value};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			qry_idx       <= qry_idx + 4'd1;
			if (qry_idx + 4'd1 >= qry_num[3:0]) begin
				return_state <= S_QRY_WR_CTRL;
			end else begin
				return_state <= S_QRY_RD_REQ;
			end
			state <= S_DD_WR_WAIT;
		end

		S_QRY_WR_CTRL: begin
			qry_last_seen_seq <= qry_request_seq;
			ddram_wr_addr     <= QUERY_CTRL_ADDR;
			// byte[0]=echo seq, byte[1]=num, byte[4]=response_seq (ARM waits on it)
			ddram_wr_din      <= {24'd0, qry_request_seq, 16'd0, qry_num[7:0], qry_request_seq};
			ddram_wr_be       <= 8'hFF;
			ddram_wr_req      <= ~ddram_wr_req;
			return_state      <= S_IDLE;
			state             <= S_DD_WR_WAIT;
		end

		default: state <= S_IDLE;
		endcase
	end
end

endmodule
