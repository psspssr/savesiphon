// tb_siphon.v - simulation of siphon_ctrl + gba_cart_bus against cart models
//
//   iverilog -g2005 -DCART_TYPE=3 -o tb sim/tb_siphon.v src/fpga/core/siphon_ctrl.v src/fpga/core/gba_cart_bus.v
//
// CART_TYPE: 0 bad header, 1 SRAM 32K, 2 Flash 64K, 3 Flash 128K,
//            4 EEPROM 512B, 5 EEPROM 8K
//   safety:  6 Flash 128K with a corrupted bank-select byte injected into the
//              sequencer (guard must block it, nothing reaches the cart)
//            7 header changes after the save is read (cart swapped)
//            8 SRAM with a byte that changes on every read (unstable contacts)
//            9 EEPROM, user presses B at the confirm prompt
//           10 EEPROM write command injected (guard must block it)
`timescale 1ns/1ps
`ifndef CART_TYPE
`define CART_TYPE 3
`endif
// byte order the mock APF reads core memory in: 1 = byte 0 in [7:0] (what the
// real Pocket does), 0 = byte 0 in [31:24]
`ifndef APF_LE
`define APF_LE 1
`endif

module tb;

reg clk = 0;
always #6.734 clk = ~clk;      // 74.25 MHz

reg reset_n = 0;
reg cart_power = 0;
reg key_a = 0;
reg key_b = 0;

// ---------------------------------------------------------------- DUT
wire        bus_enable, bus_start, bus_done, bus_busy;
wire [2:0]  bus_op;
wire [23:0] bus_addr;
wire [7:0]  bus_wdata;
wire [15:0] bus_rdata;
wire [15:0] ad_out; wire ad_oe;
wire [7:0]  hi_out; wire hi_oe;
wire        cs1_n, cs2_n, rd_n, bus_wr_n, wr_n;
wire        permit_bank, permit_ee, guard_fault;
wire [2:0]  guard_code;
reg  [15:0] ad_in;
reg  [7:0]  hi_in;

wire        buf_we;
wire [14:0] buf_waddr;
wire [31:0] buf_wdata;
reg  [31:0] save_buf [0:32767];
always @(posedge clk) if (buf_we) save_buf[buf_waddr] <= buf_wdata;

wire        tgt_openfile, tgt_write;
wire [15:0] tgt_id;
wire [31:0] tgt_slotoffset, tgt_bridgeaddr, tgt_length;
reg         tgt_ack = 0, tgt_done = 0;
reg  [2:0]  tgt_err = 0;
reg  [7:0]  struct_word_addr = 0;
wire [31:0] struct_q;
wire        le_mode;

wire [4:0]  status;
wire [2:0]  save_type;
wire [7:0]  progress;
wire [2:0]  err_code;
wire [95:0] title_flat;
wire [31:0] code_flat;
wire [511:0] path_flat;

gba_cart_bus bus (
    .clk(clk), .enable(bus_enable), .start(bus_start), .op(bus_op),
    .addr(bus_addr), .wdata(bus_wdata), .rdata(bus_rdata), .done(bus_done),
    .busy(bus_busy), .ad_out(ad_out), .ad_oe(ad_oe), .ad_in(ad_in),
    .hi_out(hi_out), .hi_oe(hi_oe), .hi_in(hi_in), .cs1_n(cs1_n),
    .cs2_n(cs2_n), .rd_n(rd_n), .wr_n(bus_wr_n));

// wr_n below is the connector pin: only what the guard lets through
cart_write_guard guard (
    .clk(clk), .reset_n(reset_n), .permit_bank(permit_bank), .permit_ee(permit_ee),
    .cs1_n(cs1_n), .cs2_n(cs2_n), .rd_n(rd_n), .wr_n(bus_wr_n),
    .ad_out(ad_out), .ad_oe(ad_oe), .hi_out(hi_out), .hi_oe(hi_oe),
    .wr_n_pin(wr_n), .fault(guard_fault), .fault_code(guard_code));

siphon_ctrl #(
    .T_POWER_WAIT(28'd1000), .T_SETTLE(28'd100), .T_CMD_TIMEOUT(28'd100000),
    .SCAN_PAGES_LO(9'd2), .SCAN_PAGES_HI(9'd4)
) ctrl (
    .clk(clk), .reset_n(reset_n), .cart_power(cart_power), .key_a(key_a),
    .key_b(key_b), .permit_bank(permit_bank), .permit_ee(permit_ee),
    .guard_fault(guard_fault),
    .bus_enable(bus_enable), .bus_start(bus_start), .bus_op(bus_op),
    .bus_addr(bus_addr), .bus_wdata(bus_wdata), .bus_rdata(bus_rdata),
    .bus_done(bus_done),
    .buf_we(buf_we), .buf_waddr(buf_waddr), .buf_wdata(buf_wdata),
    .tgt_openfile(tgt_openfile), .tgt_write(tgt_write), .tgt_id(tgt_id),
    .tgt_slotoffset(tgt_slotoffset), .tgt_bridgeaddr(tgt_bridgeaddr),
    .tgt_length(tgt_length), .tgt_ack(tgt_ack), .tgt_done(tgt_done),
    .tgt_err(tgt_err),
    .struct_word_addr(struct_word_addr), .struct_q(struct_q), .le_mode(le_mode),
    .status(status), .save_type(save_type), .progress(progress),
    .err_code(err_code), .title_flat(title_flat), .code_flat(code_flat),
    .path_flat(path_flat));

// ---------------------------------------------------------------- cart model
localparam CT = `CART_TYPE;
integer errors = 0;

reg [15:0] rom [0:131071];          // 256 KiB, mirrored
reg [7:0]  sav [0:131071];
reg [7:0]  eep [0:8191];
integer    eep_size;                // bytes, 0 = none
integer    sav_size;

// ROM: latch on CS1# fall, data while RD# low, increment on RD# rise
reg [23:0] rom_addr;
always @(negedge cs1_n) rom_addr = {hi_out, ad_out};
always @(posedge rd_n) if (!cs1_n) rom_addr[15:0] = rom_addr[15:0] + 16'd1;

// EEPROM (selected by A23 = hi_out[7] while CS1# low)
integer ee_state = 0;       // 0 idle, 1 cmd bit2, 2 addr, 3 stop, 4 readout
integer ee_nbits, ee_cnt, ee_out_cnt;
reg [13:0] ee_a;
reg [63:0] ee_shift;
reg        ee_bit_out;
wire       ee_sel = !cs1_n && hi_out[7] && (eep_size != 0);
always @(posedge wr_n) if (ee_sel) begin
    case (ee_state)
    0: if (ad_out[0]) ee_state = 1;                 // start bit
    1: begin
        if (!ad_out[0]) begin
            $display("ERROR: EEPROM write command issued");
            errors = errors + 1;
        end
        ee_state = 2; ee_cnt = 0; ee_a = 0;
    end
    2: begin
        ee_a = {ee_a[12:0], ad_out[0]};
        ee_cnt = ee_cnt + 1;
        if (ee_cnt == ee_nbits) ee_state = 3;
    end
    3: begin
        // stop bit: enter read-out (real chips do not check its value)
        ee_shift = 0;
        begin : load
            integer b;
            for (b = 0; b < 8; b = b + 1)
                ee_shift = {ee_shift[55:0], eep[((ee_a & ((eep_size/8)-1)) * 8 + b) % eep_size]};
        end
        ee_out_cnt = 0;
        ee_state = 4;
    end
    4: ; // writes during read-out are ignored
    endcase
end
always @(posedge rd_n) if (ee_sel && ee_state == 4) begin
    if (ee_out_cnt >= 4) ee_shift = {ee_shift[62:0], 1'b0};
    ee_out_cnt = ee_out_cnt + 1;
    if (ee_out_cnt == 68) ee_state = 0;
end
always @(*) ee_bit_out = (ee_state == 4) ? ((ee_out_cnt < 4) ? 1'b0 : ee_shift[63]) : 1'b1;

// Flash command decoder / SRAM write guard
integer fl_state = 0;
reg     fl_bank = 0;
always @(posedge wr_n) if (!cs2_n) begin
    if (CT == 1 || CT == 8 || sav_size == 0) begin
        $display("ERROR: write to SRAM addr %h data %h", ad_out, hi_out);
        errors = errors + 1;
    end else begin
        case (fl_state)
        0: if (ad_out == 16'h5555 && hi_out == 8'hAA) fl_state = 1;
           else begin $display("ERROR: flash stray write %h=%h", ad_out, hi_out); errors = errors + 1; end
        1: if (ad_out == 16'h2AAA && hi_out == 8'h55) fl_state = 2;
           else begin $display("ERROR: flash bad seq1"); errors = errors + 1; fl_state = 0; end
        2: if (ad_out == 16'h5555 && hi_out == 8'hB0 && (CT == 3 || CT == 6)) fl_state = 3;
           else begin $display("ERROR: flash command %h", hi_out); errors = errors + 1; fl_state = 0; end
        3: begin
            if (ad_out != 16'h0000 || hi_out > 8'h01) begin
                $display("ERROR: flash bad bank write %h=%h", ad_out, hi_out); errors = errors + 1;
            end
            fl_bank = hi_out[0];
            fl_state = 0;
        end
        endcase
    end
end

// bus contention check: Pocket drives AD while the cart drives it
wire cart_drives_ad = !cs1_n && !rd_n;
always @(posedge clk) if (cart_drives_ad && ad_oe) begin
    $display("ERROR: AD contention at %t", $time); errors = errors + 1;
end
wire cart_drives_hi = !cs2_n && !rd_n;
always @(posedge clk) if (cart_drives_hi && hi_oe) begin
    $display("ERROR: HI contention at %t", $time); errors = errors + 1;
end
always @(posedge clk) if (!wr_n && !rd_n) begin
    $display("ERROR: RD and WR both low"); errors = errors + 1;
end

wire [15:0] rom_q = rom[rom_addr[16:0]];
wire [16:0] sav_idx = {((CT == 3 || CT == 6) ? fl_bank : 1'b0), ad_out};
wire [7:0]  sav_raw = sav[sav_idx % (sav_size == 0 ? 1 : sav_size)];
reg  [7:0]  flaky = 0;      // CT 8: changes on every read pass
wire [7:0]  sav_q = (CT == 8 && sav_idx == 17'd100) ? sav_raw ^ flaky : sav_raw;
always @(*) begin
    ad_in = 16'hzzzz;
    hi_in = 8'hzz;
    if (!cs1_n && !rd_n) begin
        if (ee_sel) ad_in = {15'h7FFF, ee_bit_out};
        else        ad_in = rom_q;
    end
    if (!cs2_n && !rd_n && sav_size != 0)
        hi_in = sav_q;
end

// ---------------------------------------------------------------- APF mock
integer existing = 1;       // the first name probed already exists
integer opens = 0;
reg [8*64-1:0] path_str;
task apf_reply(input [2:0] e);
    begin
        tgt_done = 0;
        repeat (20) @(posedge clk);
        tgt_ack = 1;
        repeat (50) @(posedge clk);
        tgt_ack = 0;
        tgt_err = e;
        tgt_done = 1;
    end
endtask

task read_struct;
    integer w;
    begin
        for (w = 0; w < 16; w = w + 1) begin
            struct_word_addr = w;
            @(posedge clk); @(posedge clk); #1;
            path_str[511-32*w -: 32] = `APF_LE ?
                {struct_q[7:0], struct_q[15:8], struct_q[23:16], struct_q[31:24]} : struct_q;
        end
    end
endtask

reg [31:0] flags_w, size_w;
reg [8*64-1:0] final_path;
reg wrote = 0;
reg [31:0] wrote_len;

always @(posedge clk) begin
    if (tgt_openfile) begin
        fork : f_open
        begin
            read_struct;
            // u32 fields are little-endian inside the byte image
            struct_word_addr = 8'h40; @(posedge clk); @(posedge clk); #1;
            flags_w = `APF_LE ? struct_q : {struct_q[7:0], struct_q[15:8], struct_q[23:16], struct_q[31:24]};
            struct_word_addr = 8'h41; @(posedge clk); @(posedge clk); #1;
            size_w  = `APF_LE ? struct_q : {struct_q[7:0], struct_q[15:8], struct_q[23:16], struct_q[31:24]};
            opens = opens + 1;
            $display("  open #%0d flags=%h size=%h path=\"%0s\"", opens, flags_w, size_w, path_str);
            if (path_str[511 -: 64] != "/Assets/") begin
                apf_reply(3'd4);            // malformed path
            end else if (flags_w == 0) begin
                if (existing > 0) begin existing = existing - 1; apf_reply(3'd0); end
                else apf_reply(3'd3);
            end else begin
                final_path = path_str;
                apf_reply(3'd1);
            end
        end
        join
    end
    if (tgt_write) begin
        wrote = 1; wrote_len = tgt_length;
        if (tgt_id != 16'd1 || tgt_bridgeaddr != 32'h40000000 || tgt_slotoffset != 0) begin
            $display("ERROR: bad write params"); errors = errors + 1;
        end
        apf_reply(3'd0);
    end
end

// ---------------------------------------------------------------- setup
task put_byte(input integer a, input [7:0] v);
    begin
        if (a[0]) rom[a >> 1][15:8] = v; else rom[a >> 1][7:0] = v;
    end
endtask
task put_str(input integer a, input [8*16-1:0] s, input integer n);
    integer i;
    begin
        for (i = 0; i < n; i = i + 1) put_byte(a + i, s[8*(n-1-i) +: 8]);
    end
endtask

integer i, sum, exp_size;
reg [8*12-1:0] sig;
integer sig_len;

initial begin
    for (i = 0; i < 131072; i = i + 1) rom[i] = $random;
    for (i = 0; i < 131072; i = i + 1) sav[i] = $random;
    for (i = 0; i < 8192; i = i + 1)   eep[i] = $random;
    // header
    put_byte(4, 8'h24); put_byte(5, 8'hFF); put_byte(6, 8'hAE); put_byte(7, 8'h51);
    put_str(16'hA0, "POKEMON EMER", 12);
    put_str(16'hAC, "BPEE", 4);
    put_str(16'hB0, "01", 2);
    put_byte(16'hB2, 8'h96);
    for (i = 16'hB3; i < 16'hBD; i = i + 1) put_byte(i, 8'h00);
    sum = 0;
    for (i = 16'hA0; i < 16'hBD; i = i + 1)
        sum = sum + (i[0] ? rom[i >> 1][15:8] : rom[i >> 1][7:0]);
    put_byte(16'hBD, (-sum - 8'h19) & 8'hFF);
    if (CT == 0) put_byte(16'hB2, 8'h00);

    eep_size = 0; sav_size = 0;
    case (CT)
    1, 8: begin sig = "SRAM_V113"; sig_len = 9;  sav_size = 32768;  exp_size = 32768; end
    6: begin sig = "FLASH1M_V103"; sig_len = 12; sav_size = 131072; exp_size = 131072; end
    7: begin sig = "FLASH512_V1"; sig_len = 11; sav_size = 65536; exp_size = 65536; end
    9, 10: begin sig = "EEPROM_V124"; sig_len = 11; eep_size = 8192; exp_size = 8192; ee_nbits = 14; end
    2: begin sig = "FLASH512_V1"; sig_len = 11; sav_size = 65536; exp_size = 65536; end
    3: begin sig = "FLASH1M_V103"; sig_len = 12; sav_size = 131072; exp_size = 131072; end
    4: begin sig = "EEPROM_V124"; sig_len = 11; eep_size = 512;  exp_size = 512;  ee_nbits = 6; end
    5: begin sig = "EEPROM_V124"; sig_len = 11; eep_size = 8192; exp_size = 8192; ee_nbits = 14; end
    default: begin sig = "NOTHING"; sig_len = 7; exp_size = 0; end
    endcase
    // place the signature in the second 128 KiB page, straddling a word
    put_str(32'h2_1235, sig, sig_len);

    repeat (20) @(posedge clk);
    reset_n = 1;
    repeat (50) @(posedge clk);
    cart_power = 1;
end

// ---------------------------------------------------------------- scenarios
// confirm prompt: press A (or B for CT 9)
always @(posedge clk) begin
    if (status == 5'd13 && !key_a && !key_b) begin
        repeat (200) @(posedge clk);
        if (CT == 9) key_b = 1; else key_a = 1;
        repeat (50) @(posedge clk);
        key_a = 0; key_b = 0;
    end
end
// CT 6: corrupt the 3rd bank-select byte into a Flash erase prefix (0x80)
// CT 10: corrupt the 2nd EEPROM request bit into 0 (a write command)
initial begin
    if (CT == 6) begin
        wait (ctrl.state == 6'd16 && ctrl.bank_seq == 2'd2);
        force ctrl.bus_wdata = 8'h80;
    end
    if (CT == 10) begin
        wait (ctrl.state == 6'd19 && ctrl.ee_reqn == 5'd1);
        force ctrl.bus_wdata = 8'h00;
    end
end
// CT 7: swap the title once the save has been read
initial if (CT == 7) begin
    wait (ctrl.state == 6'd42);
    put_byte(16'hA0, "X");
end
// CT 8: a different byte value on every pass
always @(ctrl.pass or ctrl.retries) flaky = flaky + 8'h11;

// any WR# strobe at the pin with the guard faulted is a guard failure
always @(negedge wr_n) if (guard_fault) begin
    $display("ERROR: WR# reached the pin after a guard fault"); errors = errors + 1;
end

// ---------------------------------------------------------------- checks
reg [3:0] last_status = 4'hx;
always @(posedge clk) if (status !== last_status) begin
    $display("[%0t ns] status=%0d save_type=%0d", $time, status, save_type);
    last_status = status;
end

reg [7:0] got;
reg [31:0] bw;
initial begin
    wait (reset_n);
    wait (ctrl.state == 6'd38);     // S_HALT
    repeat (10) @(posedge clk);
    if (CT >= 6) begin
        case (CT)
        6, 10: if (status !== 5'd12 || !guard_fault) begin $display("ERROR: expected SAFETY STOP, got %0d", status); errors = errors + 1; end
        7:     if (status !== 5'd15) begin $display("ERROR: expected CHANGED, got %0d", status); errors = errors + 1; end
        8:     if (status !== 5'd14) begin $display("ERROR: expected UNSTABLE, got %0d", status); errors = errors + 1; end
        9:     if (status !== 5'd16) begin $display("ERROR: expected CANCELLED, got %0d", status); errors = errors + 1; end
        endcase
        if (CT == 6 || CT == 10) $display("  guard fault code %0d", guard_code);
        if (wrote) begin $display("ERROR: a file was written"); errors = errors + 1; end
        if (bus_enable) begin $display("ERROR: bus still enabled after halt"); errors = errors + 1; end
    end else if (CT == 0) begin
        if (status !== 5'd2) begin $display("ERROR: expected NOCART"); errors = errors + 1; end
    end else begin
        if (status !== 5'd8) begin $display("ERROR: expected DONE, got %0d", status); errors = errors + 1; end
        if (!wrote || wrote_len != exp_size) begin
            $display("ERROR: write len %0d expected %0d", wrote_len, exp_size); errors = errors + 1;
        end
        if (size_w != exp_size) begin $display("ERROR: open size %h", size_w); errors = errors + 1; end
        if (final_path[511 -: 8*49] != "/Assets/savesiphon/common/POKEMON_EMER_BPEE_1.sav") begin
            $display("ERROR: path prefix"); errors = errors + 1;
        end
        $display("  final path: \"%0s\"", final_path);
        for (i = 0; i < exp_size; i = i + 1) begin
            // what APF would put in the file: bridge word (as core_top
            // presents it) taken apart in APF's byte order
            bw  = le_mode ? {save_buf[i >> 2][7:0], save_buf[i >> 2][15:8],
                             save_buf[i >> 2][23:16], save_buf[i >> 2][31:24]} : save_buf[i >> 2];
            got = `APF_LE ? bw[8*(i & 3) +: 8] : bw[31 - 8*(i & 3) -: 8];
            if (got !== ((CT >= 4) ? eep[i] : sav[i])) begin
                if (errors < 20) $display("ERROR: byte %0d got %h expected %h", i, got,
                                          (CT >= 4) ? eep[i] : sav[i]);
                errors = errors + 1;
            end
        end
        if (bus_enable) begin $display("ERROR: bus still enabled after halt"); errors = errors + 1; end
        if (CT == 3 && fl_bank != 0) begin $display("ERROR: flash left on bank 1"); errors = errors + 1; end
    end
    if (errors == 0) $display("PASS cart_type=%0d apf_le=%0d", CT, `APF_LE);
    else             $display("FAIL cart_type=%0d apf_le=%0d errors=%0d", CT, `APF_LE, errors);
    $finish;
end

initial begin
    #2_000_000_000;
    $display("FAIL timeout, state=%0d", ctrl.state);
    $finish;
end

endmodule
