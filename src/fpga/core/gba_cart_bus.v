//
// gba_cart_bus.v - GBA cartridge bus engine for SaveSiphon
//
// One operation at a time, issued with a one-cycle `start` pulse and finished
// with a one-cycle `done` pulse. All timings are in clk_74a cycles (13.47 ns)
// and deliberately slower than a real GBA: a backup has no reason to race.
//
// GBA connector, as wired on the Pocket (see core_top.v):
//   AD[15:0]   ROM: multiplexed address A1..A16 / data D0..D15
//              save (CS2): plain address A0..A15
//   HI[7:0]    ROM: address A17..A24, save (CS2): data D0..D7
//   CS1#       ROM / EEPROM chip select      RD#, WR#   strobes
//   CS2#       SRAM / Flash chip select (connector pin 30)
//
// Operations
//   OP_ROM_LATCH  drive {HI,AD} = addr, drop CS1#, release AD. CS1# stays low.
//   OP_ROM_RD     one RD# pulse, returns AD. The cart auto-increments.
//   OP_EE_RD      as OP_ROM_RD with EEPROM timing; caller uses bit 0.
//   OP_EE_WR      one WR# pulse with AD = {15'b0, wdata[0]}.
//                 Refused unless the latched address is in the top 16 MiB
//                 (HI[7] set) - the EEPROM window - so it can never hit ROM.
//   OP_ROM_END    raise CS1#, release HI.
//   OP_SAVE_RD    CS2# read of one byte at addr[15:0].
//   OP_SAVE_WR    CS2# write of one byte. Only siphon_ctrl's fixed Flash
//                 bank-select sequence ever issues this.
//
// The only states in which WR# can be low are S_EEWR_PULSE and S_SWR_PULSE.
//

`default_nettype none

module gba_cart_bus #(
    parameter integer TSCALE = 1   // simulation only: divide every timing
) (
    input  wire         clk,
    input  wire         enable,     // low: every pin idle, FSM held in reset

    input  wire         start,
    input  wire [2:0]   op,
    input  wire [23:0]  addr,
    input  wire [7:0]   wdata,
    output reg  [15:0]  rdata,
    output reg          done,
    output wire         busy,

    // pin side
    output reg  [15:0]  ad_out,
    output reg          ad_oe,
    input  wire [15:0]  ad_in,
    output reg  [7:0]   hi_out,
    output reg          hi_oe,
    input  wire [7:0]   hi_in,
    output reg          cs1_n,
    output reg          cs2_n,
    output reg          rd_n,
    output reg          wr_n
);

localparam [2:0] OP_ROM_LATCH = 3'd0;
localparam [2:0] OP_ROM_RD    = 3'd1;
localparam [2:0] OP_EE_RD     = 3'd2;
localparam [2:0] OP_EE_WR     = 3'd3;
localparam [2:0] OP_ROM_END   = 3'd4;
localparam [2:0] OP_SAVE_RD   = 3'd5;
localparam [2:0] OP_SAVE_WR   = 3'd6;

// timings, clk_74a cycles
localparam [7:0] T_ADDR = (8 / TSCALE > 1) ? 8 / TSCALE : 8'd1;    // address valid before CS1# falls
localparam [7:0] T_LATCH = (8 / TSCALE > 1) ? 8 / TSCALE : 8'd1;    // CS1# low with address still driven
localparam [7:0] T_TURN = (4 / TSCALE > 1) ? 4 / TSCALE : 8'd1;    // AD released before RD# falls
localparam [7:0] T_ROM_RD = (14 / TSCALE > 1) ? 14 / TSCALE : 8'd1;   // RD# low, ROM   (~190 ns)
localparam [7:0] T_ROM_RH = (6 / TSCALE > 1) ? 6 / TSCALE : 8'd1;    // RD# high after a ROM read
localparam [7:0] T_EE_STB = (40 / TSCALE > 1) ? 40 / TSCALE : 8'd1;   // RD#/WR# low, EEPROM (~540 ns)
localparam [7:0] T_EE_GAP = (20 / TSCALE > 1) ? 20 / TSCALE : 8'd1;   // strobe high, EEPROM
localparam [7:0] T_S_SETUP = (10 / TSCALE > 1) ? 10 / TSCALE : 8'd1;   // save: address before CS2#/strobe
localparam [7:0] T_S_STB = (48 / TSCALE > 1) ? 48 / TSCALE : 8'd1;   // save: RD#/WR# low (~650 ns)
localparam [7:0] T_S_HOLD = (10 / TSCALE > 1) ? 10 / TSCALE : 8'd1;   // save: hold after strobe rises
localparam [7:0] T_IDLE = (8 / TSCALE > 1) ? 8 / TSCALE : 8'd1;    // after CS1# rises

localparam [3:0] S_IDLE       = 4'd0;
localparam [3:0] S_LAT_ADDR   = 4'd1;
localparam [3:0] S_LAT_CS     = 4'd2;
localparam [3:0] S_LAT_TURN   = 4'd3;
localparam [3:0] S_RD_PULSE   = 4'd4;
localparam [3:0] S_RD_GAP     = 4'd5;
localparam [3:0] S_EEWR_SETUP = 4'd6;
localparam [3:0] S_EEWR_PULSE = 4'd7;
localparam [3:0] S_EEWR_HOLD  = 4'd8;
localparam [3:0] S_END        = 4'd9;
localparam [3:0] S_S_SETUP    = 4'd10;
localparam [3:0] S_SRD_PULSE  = 4'd11;
localparam [3:0] S_SWR_PULSE  = 4'd12;
localparam [3:0] S_S_HOLD     = 4'd13;
localparam [3:0] S_DONE       = 4'd14;
localparam [3:0] S_ABORT      = 4'd15;   // enable dropped mid-strobe

reg [3:0]  state;
reg [7:0]  cnt;
reg [7:0]  strobe_len;      // RD# low time for the current read
reg [7:0]  gap_len;         // RD# high time after it
reg        latched_ee;      // CS1# transaction is in the EEPROM window
reg        save_wr;

// one register stage on the inputs; data is sampled long after it settled
reg [15:0] ad_in_r;
reg [7:0]  hi_in_r;
always @(posedge clk) begin
    ad_in_r <= ad_in;
    hi_in_r <= hi_in;
end

assign busy = (state != S_IDLE);

// WR# is low only in these two states. If enable drops inside one, the strobe
// is finished properly (WR# up, data held, then released) instead of yanking
// WR# and the data at the same instant, which could latch a floating value.
wire in_wr_pulse = (state == S_EEWR_PULSE) || (state == S_SWR_PULSE);

always @(posedge clk) begin
    done <= 1'b0;

    if (!enable && in_wr_pulse) begin
        wr_n  <= 1'b1;
        cnt   <= T_S_HOLD;
        state <= S_ABORT;
    end else if (!enable && state != S_ABORT) begin
        state   <= S_IDLE;
        cnt     <= 8'd0;
        ad_out  <= 16'h0000;
        ad_oe   <= 1'b0;
        hi_out  <= 8'h00;
        hi_oe   <= 1'b0;
        cs1_n   <= 1'b1;
        cs2_n   <= 1'b1;
        rd_n    <= 1'b1;
        wr_n    <= 1'b1;
        rdata   <= 16'h0000;
        latched_ee <= 1'b0;
        save_wr <= 1'b0;
        strobe_len <= T_ROM_RD;
        gap_len <= T_ROM_RH;
    end else begin
        if (cnt != 8'd0) cnt <= cnt - 8'd1;

        case (state)
        S_IDLE: begin
            rd_n <= 1'b1;
            wr_n <= 1'b1;
            if (start) begin
                case (op)
                OP_ROM_LATCH: begin
                    // never start a ROM transaction on top of another one
                    cs1_n   <= 1'b1;
                    cs2_n   <= 1'b1;
                    ad_out  <= addr[15:0];
                    ad_oe   <= 1'b1;
                    hi_out  <= addr[23:16];
                    hi_oe   <= 1'b1;
                    latched_ee <= addr[23];
                    cnt     <= T_ADDR;
                    state   <= S_LAT_ADDR;
                end
                OP_ROM_RD, OP_EE_RD: begin
                    ad_oe      <= 1'b0;
                    strobe_len <= (op == OP_EE_RD) ? T_EE_STB : T_ROM_RD;
                    gap_len    <= (op == OP_EE_RD) ? T_EE_GAP : T_ROM_RH;
                    cnt        <= T_TURN;
                    state      <= S_LAT_TURN;
                end
                OP_EE_WR: begin
                    if (!cs1_n && latched_ee) begin
                        ad_out <= {15'd0, wdata[0]};
                        ad_oe  <= 1'b1;
                        cnt    <= T_TURN;
                        state  <= S_EEWR_SETUP;
                    end else begin
                        state  <= S_DONE;   // refused, silently
                    end
                end
                OP_ROM_END: begin
                    ad_oe <= 1'b0;
                    cs1_n <= 1'b1;
                    cnt   <= T_IDLE;
                    state <= S_END;
                end
                OP_SAVE_RD, OP_SAVE_WR: begin
                    cs1_n   <= 1'b1;
                    ad_out  <= addr[15:0];
                    ad_oe   <= 1'b1;
                    save_wr <= (op == OP_SAVE_WR);
                    hi_out  <= wdata;
                    hi_oe   <= (op == OP_SAVE_WR);
                    cnt     <= T_S_SETUP;
                    state   <= S_S_SETUP;
                end
                default: state <= S_DONE;
                endcase
            end
        end

        // ---------------- ROM / EEPROM address latch ----------------
        S_LAT_ADDR: if (cnt == 8'd0) begin
            cs1_n <= 1'b0;
            cnt   <= T_LATCH;
            state <= S_LAT_CS;
        end
        S_LAT_CS: if (cnt == 8'd0) begin
            // latch is complete; AD goes back to the cart. HI stays driven.
            ad_oe <= 1'b0;
            cnt   <= T_TURN;
            state <= S_END;     // reuse the wait-then-done state
        end

        // ---------------- reads (CS1# already low) ----------------
        S_LAT_TURN: if (cnt == 8'd0) begin
            rd_n  <= 1'b0;
            cnt   <= strobe_len;
            state <= S_RD_PULSE;
        end
        S_RD_PULSE: if (cnt == 8'd0) begin
            rdata <= ad_in_r;
            rd_n  <= 1'b1;
            cnt   <= gap_len;
            state <= S_RD_GAP;
        end
        S_RD_GAP: if (cnt == 8'd0) state <= S_DONE;

        // ---------------- EEPROM serial bit write ----------------
        S_EEWR_SETUP: if (cnt == 8'd0) begin
            wr_n  <= 1'b0;
            cnt   <= T_EE_STB;
            state <= S_EEWR_PULSE;
        end
        S_EEWR_PULSE: if (cnt == 8'd0) begin
            wr_n  <= 1'b1;
            cnt   <= T_EE_GAP;
            state <= S_EEWR_HOLD;
        end
        S_EEWR_HOLD: if (cnt == 8'd0) begin
            ad_oe <= 1'b0;
            state <= S_DONE;
        end

        // ---------------- end of a CS1# transaction ----------------
        S_END: if (cnt == 8'd0) begin
            if (cs1_n) hi_oe <= 1'b0;
            state <= S_DONE;
        end

        // ---------------- save space (CS2#) ----------------
        S_S_SETUP: if (cnt == 8'd0) begin
            cs2_n <= 1'b0;
            if (save_wr) wr_n <= 1'b0;
            else         rd_n <= 1'b0;
            cnt   <= T_S_STB;
            state <= save_wr ? S_SWR_PULSE : S_SRD_PULSE;
        end
        S_SRD_PULSE: if (cnt == 8'd0) begin
            rdata <= {8'd0, hi_in_r};
            rd_n  <= 1'b1;
            cnt   <= T_S_HOLD;
            state <= S_S_HOLD;
        end
        S_SWR_PULSE: if (cnt == 8'd0) begin
            wr_n  <= 1'b1;
            cnt   <= T_S_HOLD;
            state <= S_S_HOLD;
        end
        S_S_HOLD: if (cnt == 8'd0) begin
            cs2_n   <= 1'b1;
            ad_oe   <= 1'b0;
            hi_oe   <= 1'b0;
            save_wr <= 1'b0;
            cnt     <= T_IDLE;
            state   <= S_END;
        end

        S_DONE: begin
            done  <= 1'b1;
            state <= S_IDLE;
        end

        S_ABORT: begin
            rd_n <= 1'b1;
            wr_n <= 1'b1;
            if (cnt == 8'd0) begin
                ad_oe <= 1'b0;
                hi_oe <= 1'b0;
                cs1_n <= 1'b1;
                cs2_n <= 1'b1;
                state <= S_IDLE;
            end
        end

        default: state <= S_IDLE;
        endcase
    end
end

endmodule

`default_nettype wire
