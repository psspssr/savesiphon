//
// gb_cart_bus.v - Game Boy / Game Boy Color cartridge bus engine
//
// Pocket connector in GB mode:
//   AD[15:0]  A0..A15, always driven by the Pocket while enabled
//   HI[7:0]   D0..D7, bidirectional
//   ctl       {PHI, /WR, /RD, /CS}; /CS selects cartridge RAM (A000-BFFF) only
//   pin 30    /RESET, held high while enabled (driven in core_top)
//
// Operations (start pulse, done pulse):
//   op[0] = 0  read one byte at addr. /CS is asserted only for A000-BFFF.
//   op[0] = 1  write one byte at addr. Refused (no strobe) unless addr is in
//              mapper register space 0000-7FFF. /CS is never asserted on a
//              write, so cartridge RAM can never be written by this engine.
//
// Timing is close to a real Game Boy machine cycle (~1 us per access).
//

`default_nettype none

module gb_cart_bus #(
    parameter integer TSCALE = 1   // simulation only: divide every timing
) (
    input  wire         clk,            // clk_74a
    input  wire         enable,

    input  wire         start,
    input  wire [2:0]   op,
    input  wire [15:0]  addr,
    input  wire [7:0]   wdata,
    output reg  [7:0]   rdata,
    output reg          done,

    output reg  [15:0]  ad_out,
    output reg          ad_oe,
    output reg  [7:0]   hi_out,
    output reg          hi_oe,
    input  wire [7:0]   hi_in,
    output reg          cs_n,
    output reg          rd_n,
    output reg          wr_n,
    output reg          phi
);

localparam [7:0] T_SETUP = (16 / TSCALE > 1) ? 16 / TSCALE : 8'd1;    // address/data before the strobe
localparam [7:0] T_STROBE = (40 / TSCALE > 1) ? 40 / TSCALE : 8'd1;    // /RD or /WR low (~540 ns)
localparam [7:0] T_HOLD = (16 / TSCALE > 1) ? 16 / TSCALE : 8'd1;    // after the strobe rises
localparam [5:0] T_PHI    = (35 / TSCALE > 1) ? 35 / TSCALE : 6'd1;    // half period -> ~1.06 MHz

localparam [2:0] S_IDLE = 3'd0, S_SETUP = 3'd1, S_STROBE = 3'd2,
                 S_HOLD = 3'd3, S_DONE = 3'd4, S_ABORT = 3'd5;

reg [2:0] state;
reg [7:0] cnt;
reg       is_wr;
reg [5:0] phi_cnt;
reg [7:0] hi_in_r;

always @(posedge clk) hi_in_r <= hi_in;

// PHI free-runs while enabled
always @(posedge clk) begin
    if (!enable) begin
        phi     <= 1'b0;
        phi_cnt <= 6'd0;
    end else if (phi_cnt == T_PHI - 6'd1) begin
        phi_cnt <= 6'd0;
        phi     <= ~phi;
    end else begin
        phi_cnt <= phi_cnt + 6'd1;
    end
end

always @(posedge clk) begin
    done <= 1'b0;
    if (!enable && state == S_STROBE && is_wr) begin
        // finish an interrupted write cleanly: /WR up, data held, then release
        wr_n  <= 1'b1;
        cnt   <= T_HOLD;
        state <= S_ABORT;
    end else if (!enable && state != S_ABORT) begin
        state  <= S_IDLE;
        cnt    <= 8'd0;
        ad_out <= 16'h0000;
        ad_oe  <= 1'b0;
        hi_out <= 8'h00;
        hi_oe  <= 1'b0;
        cs_n   <= 1'b1;
        rd_n   <= 1'b1;
        wr_n   <= 1'b1;
        rdata  <= 8'h00;
        is_wr  <= 1'b0;
    end else begin
        if (cnt != 8'd0) cnt <= cnt - 8'd1;
        case (state)
        S_IDLE: begin
            ad_oe <= 1'b1;          // GB address lines are always outputs
            rd_n  <= 1'b1;
            wr_n  <= 1'b1;
            cs_n  <= 1'b1;
            hi_oe <= 1'b0;
            if (start) begin
                if (op[0] && addr[15]) begin
                    state <= S_DONE;            // write outside 0000-7FFF: refused
                end else begin
                    ad_out <= addr;
                    is_wr  <= op[0];
                    hi_out <= wdata;
                    hi_oe  <= op[0];
                    cs_n   <= !(!op[0] && addr[15:13] == 3'b101);   // A000-BFFF reads
                    cnt    <= T_SETUP;
                    state  <= S_SETUP;
                end
            end
        end
        S_SETUP: if (cnt == 8'd0) begin
            if (is_wr) wr_n <= 1'b0; else rd_n <= 1'b0;
            cnt   <= T_STROBE;
            state <= S_STROBE;
        end
        S_STROBE: if (cnt == 8'd0) begin
            if (!is_wr) rdata <= hi_in_r;
            rd_n  <= 1'b1;
            wr_n  <= 1'b1;
            cnt   <= T_HOLD;
            state <= S_HOLD;
        end
        S_HOLD: if (cnt == 8'd0) begin
            hi_oe <= 1'b0;
            cs_n  <= 1'b1;
            is_wr <= 1'b0;
            state <= S_DONE;
        end
        S_DONE: begin
            done  <= 1'b1;
            state <= S_IDLE;
        end
        S_ABORT: if (cnt == 8'd0) begin
            hi_oe <= 1'b0;
            cs_n  <= 1'b1;
            ad_oe <= 1'b0;
            is_wr <= 1'b0;
            state <= S_IDLE;
        end
        default: state <= S_IDLE;
        endcase
    end
end

endmodule

`default_nettype wire
