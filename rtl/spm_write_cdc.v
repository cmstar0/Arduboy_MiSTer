`timescale 1ns / 1ps

// Crosses SPM Page Erase/Write's single-word write request from clk_avr (16 MHz) into clk_sys
// (40 MHz) so it can be applied through the top-level ROM array's existing clk_sys write port
// instead of adding a second write path on clk_avr -- see mega-regfile-timing-fix/PROJECT.md's
// "Attempt 1-4" history for why a second clk_avr write path doesn't compile (Quartus RAM
// inference rejects it three separate ways).
//
// Standard 4-phase toggle handshake, 2-flop synchronizer each direction. Safe for any relative
// clock frequency because the requester (mega-core.v's SPM state machine, gated by
// spm_word_pending) is required to wait for req_ack before issuing the next req_write -- this
// module does not defend against a second req_write arriving before the first is acknowledged,
// and does not need to, since only one word is ever in flight at a time by construction.
module spm_write_cdc #(
	parameter ADDR_WIDTH = 14,
	parameter DATA_WIDTH = 16
)
(
	// clk_avr domain -- request side, driven by mega-core.v's spm_pgm_write/addr/data
	input                       clk_avr,
	input                       req_write,
	input      [ADDR_WIDTH-1:0] req_addr,
	input      [DATA_WIDTH-1:0] req_data,
	output reg                  req_ack,

	// clk_sys domain -- apply side, feeds the top-level ROM array's existing write port
	input                       clk_sys,
	output reg                  wr_write,
	output reg [ADDR_WIDTH-1:0] wr_addr,
	output reg [DATA_WIDTH-1:0] wr_data
);

reg                  req_toggle = 1'b0;
reg [ADDR_WIDTH-1:0] req_addr_hold;
reg [DATA_WIDTH-1:0] req_data_hold;
reg                  ack_toggle = 1'b0;

reg req_sync1, req_sync2, req_sync3;
reg ack_sync1, ack_sync2, ack_sync3;

// ---- clk_avr domain: latch the request, toggle it across, watch for the returning ack ----
always @ (posedge clk_avr) begin
	if (req_write) begin
		req_addr_hold <= req_addr;
		req_data_hold <= req_data;
		req_toggle    <= ~req_toggle;
	end

	ack_sync1 <= ack_toggle;
	ack_sync2 <= ack_sync1;
	ack_sync3 <= ack_sync2;
	req_ack   <= ack_sync2 ^ ack_sync3;
end

// ---- clk_sys domain: catch the toggle, apply the write, toggle the ack back ----
always @ (posedge clk_sys) begin
	req_sync1 <= req_toggle;
	req_sync2 <= req_sync1;
	req_sync3 <= req_sync2;

	wr_write <= req_sync2 ^ req_sync3;
	wr_addr  <= req_addr_hold;
	wr_data  <= req_data_hold;

	if (req_sync2 ^ req_sync3)
		ack_toggle <= ~ack_toggle;
end

endmodule
