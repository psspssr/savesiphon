//
// cart_write_guard.v - independent last line of defence on the cartridge WR# pin
//
// Sits between gba_cart_bus and the connector. It does not trust the
// sequencer: every WR# strobe the bus engine requests is checked against the
// only two kinds of write SaveSiphon is ever allowed to make, using nothing but
// the values on the pins. A strobe that fails the check never reaches the pin,
// and the guard latches a permanent fault (WR# held high, bus shut down by the
// sequencer) until the core is reset.
//
// Allowed writes:
//   CS2# (save space), only while permit_bank is high, only in this order:
//       5555<-AA, 2AAA<-55, 5555<-B0, 0000<-{00|01}     (Flash 128K bank select)
//   CS1# (EEPROM), only while permit_ee is high, only inside a transaction
//   that was latched at address 0xFFFF80 (EEPROM window) with HI still 0xFF,
//   AD carrying a single bit (AD[15:1] = 0), at most 17 bits, and either
//       1 1 <address bits> 0     (read request: second bit must be 1)
//       0 0 0 ...                (flush: every bit 0)
//   so an EEPROM write command (1 0 ...) can never be formed.
//
// Also faults, at any time:
//   CS1# and CS2# low together; RD# and WR# low together; any change of
//   address, data or chip select while WR# is low.
//
// WR# reaches the pin one clock after the bus engine asks for it, on both
// edges. Every write in gba_cart_bus holds data for many clocks after WR#
// rises, so the extra cycle is covered.
//

`default_nettype none

module cart_write_guard (
    input  wire         clk,
    input  wire         reset_n,

    input  wire         permit_bank,
    input  wire         permit_ee,

    // from gba_cart_bus
    input  wire         cs1_n,
    input  wire         cs2_n,
    input  wire         rd_n,
    input  wire         wr_n,
    input  wire [15:0]  ad_out,
    input  wire         ad_oe,
    input  wire [7:0]   hi_out,
    input  wire         hi_oe,

    // to the connector
    output reg          wr_n_pin,

    output reg          fault,
    output reg  [2:0]   fault_code      // 1 cs2 write, 2 ee write, 3 bus, 4 unstable
);

// expected Flash bank-select step
reg  [1:0]  bank_step;
reg  [15:0] exp_addr;
reg         data_ok;
always @(*) begin
    case (bank_step)
    2'd0: begin exp_addr = 16'h5555; data_ok = (hi_out == 8'hAA); end
    2'd1: begin exp_addr = 16'h2AAA; data_ok = (hi_out == 8'h55); end
    2'd2: begin exp_addr = 16'h5555; data_ok = (hi_out == 8'hB0); end
    default: begin exp_addr = 16'h0000; data_ok = (hi_out == 8'h00) || (hi_out == 8'h01); end
    endcase
end

// EEPROM transaction tracking (reset whenever CS1# is high)
reg         cs1_n_q;
reg  [23:0] cs1_latch;
reg  [4:0]  ee_idx;
reg         ee_first;

wire bank_ok = permit_bank && !cs2_n && cs1_n && rd_n &&
               ad_oe && hi_oe && (ad_out == exp_addr) && data_ok;

wire ee_bit  = ad_out[0];
wire ee_ok   = permit_ee && !cs1_n && cs2_n && rd_n &&
               ad_oe && hi_oe && (hi_out == 8'hFF) && (ad_out[15:1] == 15'd0) &&
               (cs1_latch == 24'hFFFF80) && (ee_idx < 5'd17) &&
               ( (ee_idx == 5'd0) ||
                 (ee_first  && (ee_idx != 5'd1 || ee_bit)) ||   // request: 2nd bit is 1
                 (!ee_first && !ee_bit) );                       // flush: all zero

wire write_ok = bank_ok || ee_ok;

// snapshot of everything that must not move while WR# is low
reg         active;
reg  [33:0] snap;
wire [33:0] now = {cs1_n, cs2_n, ad_out, hi_out, ad_oe, hi_oe, rd_n, 5'd0};

always @(posedge clk) begin
    cs1_n_q <= cs1_n;
    if (!reset_n) begin
        wr_n_pin   <= 1'b1;
        fault      <= 1'b0;
        fault_code <= 3'd0;
        active     <= 1'b0;
        bank_step  <= 2'd0;
        ee_idx     <= 5'd0;
        ee_first   <= 1'b0;
        cs1_latch  <= 24'd0;
        snap       <= 34'd0;
    end else begin
        // EEPROM transaction boundaries
        if (cs1_n) begin
            ee_idx <= 5'd0;
        end else if (cs1_n_q) begin
            cs1_latch <= {hi_out, ad_out};      // address at the CS1# falling edge
            ee_idx    <= 5'd0;
        end
        if (!permit_bank) bank_step <= 2'd0;

        // bus-level faults, any time
        if (!fault && ((!cs1_n && !cs2_n) || (!rd_n && !wr_n))) begin
            fault <= 1'b1; fault_code <= 3'd3;
        end

        if (fault) begin
            wr_n_pin <= 1'b1;
            active   <= 1'b0;
        end else if (!wr_n && !active) begin
            // a new strobe: allow it or kill it before the pin moves
            if (write_ok) begin
                wr_n_pin <= 1'b0;
                active   <= 1'b1;
                snap     <= now;
            end else begin
                wr_n_pin   <= 1'b1;
                fault      <= 1'b1;
                fault_code <= !cs2_n ? 3'd1 : 3'd2;
            end
        end else if (!wr_n && active) begin
            if (now != snap) begin
                wr_n_pin   <= 1'b1;
                active     <= 1'b0;
                fault      <= 1'b1;
                fault_code <= 3'd4;
            end
        end else if (wr_n && active) begin
            // strobe finished: advance the allowed sequence
            wr_n_pin <= 1'b1;
            active   <= 1'b0;
            if (!snap[32]) begin                // was a CS2# write
                bank_step <= bank_step + 2'd1;
            end else begin                      // CS1# EEPROM bit
                if (ee_idx == 5'd0) ee_first <= snap[16];   // ad_out[0] in snapshot
                ee_idx <= ee_idx + 5'd1;
            end
        end else begin
            wr_n_pin <= 1'b1;
        end
    end
end

endmodule

`default_nettype wire
