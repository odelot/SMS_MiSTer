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

// ======================================================================
// State machine
// ======================================================================
localparam S_IDLE        = 4'd0;
localparam S_DD_WR_WAIT  = 4'd1;
localparam S_DD_RD_WAIT  = 4'd2;
localparam S_READ_HDR    = 4'd3;
localparam S_PARSE_HDR   = 4'd4;
localparam S_READ_PAIR   = 4'd5;
localparam S_PARSE_ADDR  = 4'd6;
localparam S_DISPATCH    = 4'd7;
localparam S_BRAM_WAIT   = 4'd8;
localparam S_BRAM_WAIT2  = 4'd9;
localparam S_STORE_VAL   = 4'd10;
localparam S_FLUSH_BUF   = 4'd11;
localparam S_WRITE_RESP  = 4'd12;
localparam S_WR_HDR0     = 4'd13;
localparam S_WR_HDR1     = 4'd14;
localparam S_WR_DBG      = 4'd15;

reg [3:0]  state;
reg [3:0]  return_state;

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
	end
	else begin
		case (state)

		S_IDLE: begin
			active <= 1'b0;
			if (vblank_rising) begin
				active <= 1'b1;
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
		S_WR_DBG: begin
			ddram_wr_addr <= DDRAM_BASE + 29'd2;
			ddram_wr_din  <= {8'h01, 8'd0, dbg_ok_cnt, dbg_oob_cnt, 16'd0};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_IDLE;
			state         <= S_DD_WR_WAIT;
		end

		default: state <= S_IDLE;
		endcase
	end
end

endmodule
