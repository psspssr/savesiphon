// tb_siphon.v - SaveSiphon system simulation against cartridge models
//
// Builds the same engine/mux/guard/sequencer arrangement as core_top.v, plus:
//   * a GBA cartridge (ROM, SRAM / Flash 64K / Flash 128K / EEPROM)
//   * a GB/GBC cartridge (ROM-only, MBC1, MBC2, MBC3, MBC5 with RAM)
//   * a bus keeper: undriven lines hold the last value the Pocket drove,
//     which is what real hardware does (GBA open bus = its own address)
//   * an APF model that assembles every file from its chunk writes
//   * a "user" that drives the menu with the D-pad and A/B
//
// Scenario selected with -DCT=n (see the case statements below).
// APF byte order with -DAPF_LE=0/1 (real hardware: 1).
`timescale 1ns/1ps
`ifndef CT
`define CT 1
`endif
`ifndef APF_LE
`define APF_LE 1
`endif

module tb;

localparam CT = `CT;
integer errors = 0;

reg clk = 0;
always #6.734 clk = ~clk;

reg reset_n = 0;
reg cart_power = 0;
reg key_a = 0, key_b = 0, key_up = 0, key_down = 0;

// ------------------------------------------------------------------ DUT (as core_top)
wire        bus_enable, bus_mode, bus_start;
wire [2:0]  bus_op;
wire [23:0] bus_addr;
wire [7:0]  bus_wdata;
wire [15:0] bus_rdata;
wire        bus_done;
wire        permit_bank, permit_ee, permit_mbc, guard_fault;
wire [2:0]  guard_code;

reg  [15:0] ad_in;
reg  [7:0]  hi_in;

wire [15:0] gba_rdata, gba_ad_out; wire gba_done, gba_busy, gba_ad_oe, gba_hi_oe;
wire [7:0]  gba_hi_out; wire gba_cs1_n, gba_cs2_n, gba_rd_n, gba_wr_n;
gba_cart_bus #(.TSCALE(4)) gba (
    .clk(clk), .enable(bus_enable & ~bus_mode), .start(bus_start & ~bus_mode), .op(bus_op),
    .addr(bus_addr), .wdata(bus_wdata), .rdata(gba_rdata), .done(gba_done), .busy(gba_busy),
    .ad_out(gba_ad_out), .ad_oe(gba_ad_oe), .ad_in(ad_in), .hi_out(gba_hi_out), .hi_oe(gba_hi_oe),
    .hi_in(hi_in), .cs1_n(gba_cs1_n), .cs2_n(gba_cs2_n), .rd_n(gba_rd_n), .wr_n(gba_wr_n));

wire [7:0]  gb_rdata, gb_hi_out; wire [15:0] gb_ad_out;
wire        gb_done, gb_ad_oe, gb_hi_oe, gb_cs_n, gb_rd_n, gb_wr_n, gb_phi;
gb_cart_bus #(.TSCALE(4)) gbb (
    .clk(clk), .enable(bus_enable & bus_mode), .start(bus_start & bus_mode), .op(bus_op),
    .addr(bus_addr[15:0]), .wdata(bus_wdata), .rdata(gb_rdata), .done(gb_done),
    .ad_out(gb_ad_out), .ad_oe(gb_ad_oe), .hi_out(gb_hi_out), .hi_oe(gb_hi_oe), .hi_in(hi_in),
    .cs_n(gb_cs_n), .rd_n(gb_rd_n), .wr_n(gb_wr_n), .phi(gb_phi));

assign bus_rdata = bus_mode ? {8'd0, gb_rdata} : gba_rdata;
assign bus_done  = bus_mode ? gb_done : gba_done;
wire [15:0] ad_out = bus_mode ? gb_ad_out : gba_ad_out;
wire        ad_oe  = bus_enable & (bus_mode ? gb_ad_oe : gba_ad_oe);
wire [7:0]  hi_out = bus_mode ? gb_hi_out : gba_hi_out;
wire        hi_oe  = bus_enable & (bus_mode ? gb_hi_oe : gba_hi_oe);
wire        cs1_n  = bus_mode ? gb_cs_n : gba_cs1_n;
wire        cs2_n  = bus_mode ? 1'b1 : gba_cs2_n;
wire        rd_n   = bus_mode ? gb_rd_n : gba_rd_n;
wire        bus_wr_n = bus_mode ? gb_wr_n : gba_wr_n;
wire        wr_n;   // connector pin, after the guard

cart_write_guard guard (
    .clk(clk), .reset_n(reset_n), .permit_bank(permit_bank), .permit_ee(permit_ee),
    .gb_mode(bus_mode), .permit_mbc(permit_mbc),
    .cs1_n(cs1_n), .cs2_n(cs2_n), .rd_n(rd_n), .wr_n(bus_wr_n),
    .ad_out(ad_out), .ad_oe(ad_oe), .hi_out(hi_out), .hi_oe(hi_oe),
    .wr_n_pin(wr_n), .fault(guard_fault), .fault_code(guard_code));

wire        buf_we; wire [14:0] buf_waddr; wire [31:0] buf_wdata;
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

wire [2:0]  ui_mode; wire [1:0] menu_sel; wire [4:0] status; wire [7:0] progress;
wire [2:0]  err_code; wire [1:0] cart_kind; wire [127:0] name_flat; wire [4:0] name_len;
wire [31:0] code_flat; wire code_len; wire [3:0] save_label; wire [4:0] rom_log2;
wire [2:0]  mapper; wire [1:0] done_what; wire [511:0] path_flat;

siphon_ctrl #(
    .T_POWER_WAIT(28'd1000), .T_SETTLE(28'd100), .T_CMD_TIMEOUT(28'd200000),
    .SCAN_PAGES_LO(9'd2), .SCAN_PAGES_HI(9'd4)
) ctrl (
    .clk(clk), .reset_n(reset_n), .cart_power(cart_power),
    .key_a(key_a), .key_b(key_b), .key_up(key_up), .key_down(key_down),
    .permit_bank(permit_bank), .permit_ee(permit_ee), .permit_mbc(permit_mbc),
    .guard_fault(guard_fault),
    .bus_enable(bus_enable), .bus_mode(bus_mode), .bus_start(bus_start), .bus_op(bus_op),
    .bus_addr(bus_addr), .bus_wdata(bus_wdata), .bus_rdata(bus_rdata), .bus_done(bus_done),
    .buf_we(buf_we), .buf_waddr(buf_waddr), .buf_wdata(buf_wdata),
    .tgt_openfile(tgt_openfile), .tgt_write(tgt_write), .tgt_id(tgt_id),
    .tgt_slotoffset(tgt_slotoffset), .tgt_bridgeaddr(tgt_bridgeaddr),
    .tgt_length(tgt_length), .tgt_ack(tgt_ack), .tgt_done(tgt_done), .tgt_err(tgt_err),
    .struct_word_addr(struct_word_addr), .struct_q(struct_q), .le_mode(le_mode),
    .ui_mode(ui_mode), .menu_sel(menu_sel), .status(status), .progress(progress),
    .err_code(err_code), .cart_kind(cart_kind), .name_flat(name_flat), .name_len(name_len),
    .code_flat(code_flat), .code_len(code_len), .save_label(save_label), .rom_log2(rom_log2),
    .mapper(mapper), .done_what(done_what), .path_flat(path_flat));

// ------------------------------------------------------------------ cartridge setup
reg        is_gba_cart, is_gb_cart;
integer    gba_rom_bytes;   // ROM size
reg        gba_open;        // past the end: 1 open bus, 0 mirror
integer    sav_size;        // GBA SRAM/Flash bytes
reg        flash128;
integer    eep_size;        // GBA EEPROM bytes
integer    ee_nbits;
integer    gb_rom_bytes, gb_ram_bytes;
integer    gb_mbc;          // 0 ROM, 1 MBC1, 2 MBC2, 3 MBC3, 5 MBC5

reg [15:0] rom [0:524287];          // GBA ROM, up to 1 MiB
reg [7:0]  sav [0:131071];
reg [7:0]  eep [0:8191];
reg [7:0]  gbrom [0:1048575];       // GB ROM, up to 1 MiB
reg [7:0]  gbram [0:131071];

// ------------------------------------------------------------------ bus keeper
reg [15:0] ad_keep = 0;
reg [7:0]  hi_keep = 0;
always @(posedge clk) begin
    if (ad_oe) ad_keep <= ad_out;
    if (hi_oe) hi_keep <= hi_out;
end

// ------------------------------------------------------------------ GBA cartridge model
reg [23:0] rom_addr;
always @(negedge cs1_n) if (!bus_mode) rom_addr = {hi_out, ad_out};
always @(posedge rd_n) if (!cs1_n && !bus_mode) rom_addr[15:0] = rom_addr[15:0] + 16'd1;

integer ee_state = 0, ee_cnt, ee_out_cnt;
reg [13:0] ee_a;
reg [63:0] ee_shift;
reg        ee_bit_out;
wire       ee_sel = is_gba_cart && !bus_mode && !cs1_n && hi_out[7] && (eep_size != 0);
always @(posedge wr_n) if (ee_sel) begin
    case (ee_state)
    0: if (ad_out[0]) ee_state = 1;
    1: begin
        if (!ad_out[0]) begin $display("ERROR: EEPROM write command reached the cart"); errors = errors + 1; end
        ee_state = 2; ee_cnt = 0; ee_a = 0;
    end
    2: begin
        ee_a = {ee_a[12:0], ad_out[0]};
        ee_cnt = ee_cnt + 1;
        if (ee_cnt == ee_nbits) ee_state = 3;
    end
    3: begin
        ee_shift = 0;
        begin : load
            integer b;
            for (b = 0; b < 8; b = b + 1)
                ee_shift = {ee_shift[55:0], eep[((ee_a & ((eep_size/8)-1)) * 8 + b) % eep_size]};
        end
        ee_out_cnt = 0;
        ee_state = 4;
    end
    4: ;
    endcase
end
always @(posedge rd_n) if (ee_sel && ee_state == 4) begin
    if (ee_out_cnt >= 4) ee_shift = {ee_shift[62:0], 1'b0};
    ee_out_cnt = ee_out_cnt + 1;
    if (ee_out_cnt == 68) ee_state = 0;
end
always @(*) ee_bit_out = (ee_state == 4) ? ((ee_out_cnt < 4) ? 1'b0 : ee_shift[63]) : 1'b1;

integer fl_state = 0;
reg     fl_bank = 0;
always @(posedge wr_n) if (is_gba_cart && !bus_mode && !cs2_n) begin
    if (!flash128) begin
        $display("ERROR: write reached SRAM/Flash64 %h=%h", ad_out, hi_out); errors = errors + 1;
    end else case (fl_state)
        0: if (ad_out == 16'h5555 && hi_out == 8'hAA) fl_state = 1;
           else begin $display("ERROR: flash stray write %h=%h", ad_out, hi_out); errors = errors + 1; end
        1: if (ad_out == 16'h2AAA && hi_out == 8'h55) fl_state = 2;
           else begin $display("ERROR: flash bad seq1"); errors = errors + 1; fl_state = 0; end
        2: if (ad_out == 16'h5555 && hi_out == 8'hB0) fl_state = 3;
           else begin $display("ERROR: flash command %h reached the cart", hi_out); errors = errors + 1; fl_state = 0; end
        3: begin
            if (ad_out != 16'h0000 || hi_out > 8'h01) begin $display("ERROR: flash bad bank write"); errors = errors + 1; end
            fl_bank = hi_out[0];
            fl_state = 0;
        end
    endcase
end

// CT 8: one byte changes on every read pass
reg [7:0] flaky = 0;
always @(ctrl.pass or ctrl.retries) flaky = flaky + 8'h11;

wire        rom_inside = ({rom_addr, 1'b0} < gba_rom_bytes);
wire [15:0] rom_q = rom[rom_addr % (gba_rom_bytes / 2)];
wire [16:0] sav_idx = {(flash128 ? fl_bank : 1'b0), ad_out};
wire [7:0]  sav_raw = sav[sav_idx % (sav_size == 0 ? 1 : sav_size)];
wire [7:0]  sav_q = (CT == 8 && sav_idx == 17'd100) ? sav_raw ^ flaky : sav_raw;

// ------------------------------------------------------------------ GB cartridge model
reg [8:0] gb_bank_n = 1;
reg [3:0] gb_ram_bank = 0;
reg       gb_ram_en = 0, gb_mode1 = 0;
reg [1:0] gb_upper = 0;
reg [4:0] gb_low5 = 1;

// read path as plain wires (no arrays inside always @*: keeps Icarus fast)
wire [8:0]  gb_bank0  = (gb_mbc == 1 && gb_mode1) ? {2'b00, gb_upper, 5'd0} : 9'd0;
wire [8:0]  gb_bankx  = (gb_mbc == 1) ? {2'b00, gb_upper, (gb_low5 == 0) ? 5'd1 : gb_low5} :
                        (gb_mbc == 2) ? ((gb_bank_n[3:0] == 0) ? 9'd1 : {5'd0, gb_bank_n[3:0]}) :
                        (gb_mbc == 3) ? ((gb_bank_n[6:0] == 0) ? 9'd1 : {2'd0, gb_bank_n[6:0]}) :
                        (gb_mbc == 5) ? gb_bank_n : 9'd1;
wire [31:0] gb_ridx   = (((ad_out < 16'h4000) ? gb_bank0 : gb_bankx) * 16384 + (ad_out & 16'h3FFF)) % gb_rom_bytes;
wire [7:0]  gb_rom_q  = gbrom[gb_ridx];
wire [3:0]  gb_rbank  = (gb_mbc == 1) ? (gb_mode1 ? {2'b00, gb_upper} : 4'd0) : gb_ram_bank;
wire [31:0] gb_raidx  = (gb_mbc == 2) ? (ad_out & 16'h01FF) :
                        ((gb_rbank * 8192 + (ad_out - 16'hA000)) % (gb_ram_bytes == 0 ? 1 : gb_ram_bytes));
wire [7:0]  gb_ram_raw = gbram[gb_raidx];
wire [7:0]  gb_ram_q  = (!gb_ram_en && gb_mbc != 0) ? 8'hFF : (gb_mbc == 2) ? {4'hF, gb_ram_raw[3:0]} : gb_ram_raw;

always @(posedge wr_n) if (is_gb_cart && bus_mode) begin
    if (!cs1_n || ad_out[15]) begin
        $display("ERROR: write to GB cartridge RAM/space %h=%h reached the cart", ad_out, hi_out); errors = errors + 1;
    end else case (gb_mbc)
        1: begin
            if (ad_out < 16'h2000)      gb_ram_en = (hi_out[3:0] == 4'hA);
            else if (ad_out < 16'h4000) gb_low5 = hi_out[4:0];
            else if (ad_out < 16'h6000) gb_upper = hi_out[1:0];
            else                        gb_mode1 = hi_out[0];
        end
        2: if (ad_out < 16'h4000) begin
               if (!ad_out[8]) gb_ram_en = (hi_out[3:0] == 4'hA); else gb_bank_n = hi_out[3:0];
           end
        3: begin
            if (ad_out < 16'h2000)      gb_ram_en = (hi_out[3:0] == 4'hA);
            else if (ad_out < 16'h4000) gb_bank_n = hi_out[6:0];
            else if (ad_out < 16'h6000) gb_ram_bank = hi_out[3:0];
        end
        5: begin
            if (ad_out < 16'h2000)      gb_ram_en = (hi_out[3:0] == 4'hA);
            else if (ad_out < 16'h3000) gb_bank_n[7:0] = hi_out;
            else if (ad_out < 16'h4000) gb_bank_n[8] = hi_out[0];
            else if (ad_out < 16'h6000) gb_ram_bank = hi_out[3:0];
        end
        default: ;
    endcase
end

// ------------------------------------------------------------------ pin-level checks
always @(posedge clk) begin
    if (!bus_mode && is_gba_cart && !cs1_n && !rd_n && ad_oe && rom_inside) begin
        $display("ERROR: AD contention at %t", $time); errors = errors + 1;
    end
    if (!wr_n && !rd_n) begin $display("ERROR: RD and WR both low"); errors = errors + 1; end
end
always @(negedge wr_n) if (guard_fault) begin
    $display("ERROR: WR# reached the pin after a guard fault"); errors = errors + 1;
end

// cartridge drives
always @(*) begin
    ad_in = ad_keep;
    hi_in = hi_keep;
    if (is_gba_cart && !bus_mode) begin
        if (!cs1_n && !rd_n) begin
            if (ee_sel)                       ad_in = {15'h7FFF, ee_bit_out};
            else if (rom_inside || !gba_open) ad_in = rom_q;
        end
        if (!cs2_n && !rd_n && sav_size != 0) hi_in = sav_q;
    end
    if (is_gb_cart && bus_mode && !rd_n) begin
        if (ad_out < 16'h8000)                       hi_in = gb_rom_q;
        else if (!cs1_n && ad_out[15:13] == 3'b101)  hi_in = gb_ram_q;
    end
end

// ------------------------------------------------------------------ APF model
integer      existing = 0;
integer      opens = 0, nfiles = 0;
reg [8*64-1:0] path_str;
reg [8*64-1:0] fpath [0:3];
integer      fsize [0:3];
integer      fwritten [0:3];
reg [7:0]    fdata0 [0:1048575];
reg [7:0]    fdata1 [0:1048575];
reg [31:0]   flags_w, size_w, bw;

task apf_reply(input [2:0] e);
    begin
        tgt_done = 0;
        repeat (10) @(posedge clk);
        tgt_ack = 1;
        repeat (20) @(posedge clk);
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
        struct_word_addr = 8'h40; @(posedge clk); @(posedge clk); #1;
        flags_w = `APF_LE ? struct_q : {struct_q[7:0], struct_q[15:8], struct_q[23:16], struct_q[31:24]};
        struct_word_addr = 8'h41; @(posedge clk); @(posedge clk); #1;
        size_w  = `APF_LE ? struct_q : {struct_q[7:0], struct_q[15:8], struct_q[23:16], struct_q[31:24]};
    end
endtask

integer i, wi;
reg [7:0] fb;
always @(posedge clk) begin
    if (tgt_openfile) begin
        read_struct;
        opens = opens + 1;
        if (path_str[511 -: 64] != "/Assets/") begin
            apf_reply(3'd4);
        end else if (flags_w == 0) begin
            if (existing > 0) begin existing = existing - 1; apf_reply(3'd0); end
            else apf_reply(3'd3);
        end else begin
            $display("  file %0d: \"%0s\" size %0d", nfiles, path_str, size_w);
            fpath[nfiles] = path_str; fsize[nfiles] = size_w; fwritten[nfiles] = 0;
            nfiles = nfiles + 1;
            apf_reply(3'd1);
        end
    end
    if (tgt_write) begin
        if (tgt_id != 16'd1 || tgt_bridgeaddr != 32'h40000000) begin
            $display("ERROR: bad write params"); errors = errors + 1;
        end
        if (tgt_slotoffset + tgt_length > fsize[nfiles-1]) begin
            $display("ERROR: write past end of file"); errors = errors + 1;
        end
        for (wi = 0; wi < tgt_length; wi = wi + 1) begin
            bw = le_mode ? {save_buf[wi >> 2][7:0], save_buf[wi >> 2][15:8],
                            save_buf[wi >> 2][23:16], save_buf[wi >> 2][31:24]} : save_buf[wi >> 2];
            fb = `APF_LE ? bw[8*(wi & 3) +: 8] : bw[31 - 8*(wi & 3) -: 8];
            if (nfiles == 1) fdata0[tgt_slotoffset + wi] = fb; else fdata1[tgt_slotoffset + wi] = fb;
        end
        fwritten[nfiles-1] = fwritten[nfiles-1] + tgt_length;
        apf_reply(3'd0);
    end
end

// ------------------------------------------------------------------ user
task press(input integer which);   // 0 A, 1 B, 2 up, 3 down
    begin
        repeat (30) @(posedge clk);
        case (which) 0: key_a = 1; 1: key_b = 1; 2: key_up = 1; default: key_down = 1; endcase
        repeat (30) @(posedge clk);
        key_a = 0; key_b = 0; key_up = 0; key_down = 0;
        repeat (30) @(posedge clk);
    end
endtask

task choose(input integer item);    // from the menu: move to item and press A
    begin
        wait (ui_mode == 3'd1);
        while (menu_sel != item) press(3);
        press(0);
    end
endtask

// press A at a confirm prompt (or B for CT 9)
always @(posedge clk) if (ui_mode == 3'd3 && !key_a && !key_b) begin
    if (CT == 9) press(1); else press(0);
end

// ------------------------------------------------------------------ helpers
task put_byte(input integer a, input [7:0] v);
    begin
        if (a[0]) rom[a >> 1][15:8] = v; else rom[a >> 1][7:0] = v;
    end
endtask
task put_str(input integer a, input [8*16-1:0] s, input integer n);
    integer j;
    begin
        for (j = 0; j < n; j = j + 1) put_byte(a + j, s[8*(n-1-j) +: 8]);
    end
endtask
task gb_str(input integer a, input [8*16-1:0] s, input integer n);
    integer j;
    begin
        for (j = 0; j < n; j = j + 1) gbrom[a + j] = s[8*(n-1-j) +: 8];
    end
endtask

task check_file(input integer f, input integer n, input integer kind); // 0 GBA ROM, 1 GBA save, 2 eep, 3 GB ROM, 4 GB RAM
    integer j; reg [7:0] got, exp;
    begin
        if (fsize[f] != n)    begin $display("ERROR: file %0d size %0d expected %0d", f, fsize[f], n); errors = errors + 1; end
        if (fwritten[f] != n) begin $display("ERROR: file %0d wrote %0d of %0d bytes", f, fwritten[f], n); errors = errors + 1; end
        for (j = 0; j < n; j = j + 1) begin
            got = (f == 0) ? fdata0[j] : fdata1[j];
            case (kind)
            0: exp = j[0] ? rom[j >> 1][15:8] : rom[j >> 1][7:0];
            1: exp = sav[j];
            2: exp = eep[j];
            3: exp = gbrom[j];
            default: exp = (gb_mbc == 2) ? {4'hF, gbram[j][3:0]} : gbram[j];
            endcase
            if (got !== exp) begin
                if (errors < 10) $display("ERROR: file %0d byte %0d got %h expected %h", f, j, got, exp);
                errors = errors + 1;
            end
        end
    end
endtask

// ------------------------------------------------------------------ scenarios
integer sum, sig_len;
reg [8*12-1:0] sig;
reg [7:0] x;

initial begin
    for (i = 0; i < 524288; i = i + 1)  rom[i] = $random;
    for (i = 0; i < 131072; i = i + 1)  sav[i] = $random;
    for (i = 0; i < 8192; i = i + 1)    eep[i] = $random;
    for (i = 0; i < 1048576; i = i + 1) gbrom[i] = $random;
    for (i = 0; i < 131072; i = i + 1)  gbram[i] = $random;

    is_gba_cart = 1; is_gb_cart = 0;
    gba_rom_bytes = 262144; gba_open = 0;
    sav_size = 0; flash128 = 0; eep_size = 0; ee_nbits = 6;
    gb_rom_bytes = 32768; gb_ram_bytes = 0; gb_mbc = 0;
    sig = "NOTHING"; sig_len = 7;

    case (CT)
    0:  begin is_gba_cart = 0; end                                          // empty slot
    1:  begin sig = "SRAM_V113";    sig_len = 9;  sav_size = 32768; existing = 1; end
    2:  begin sig = "FLASH512_V1";  sig_len = 11; sav_size = 65536; end
    3:  begin sig = "FLASH1M_V103"; sig_len = 12; sav_size = 131072; flash128 = 1; end
    4:  begin sig = "EEPROM_V124";  sig_len = 11; eep_size = 512;  ee_nbits = 6; end
    5:  begin sig = "EEPROM_V124";  sig_len = 11; eep_size = 8192; ee_nbits = 14; end
    6:  begin sig = "FLASH1M_V103"; sig_len = 12; sav_size = 131072; flash128 = 1; end
    7:  begin sig = "FLASH512_V1";  sig_len = 11; sav_size = 65536; end
    8:  begin sig = "SRAM_V113";    sig_len = 9;  sav_size = 32768; end
    9:  begin sig = "EEPROM_V124";  sig_len = 11; eep_size = 8192; ee_nbits = 14; end
    10: begin sig = "EEPROM_V124";  sig_len = 11; eep_size = 8192; ee_nbits = 14; end
    11: begin sig = "SRAM_V113";    sig_len = 9;  sav_size = 32768; gba_rom_bytes = 524288; gba_open = 1; end
    12: begin sig = "SRAM_V113";    sig_len = 9;  sav_size = 32768; gba_rom_bytes = 262144; gba_open = 1; end
    13: begin is_gba_cart = 0; is_gb_cart = 1; gb_mbc = 5; gb_rom_bytes = 131072; gb_ram_bytes = 32768; end
    14: begin is_gba_cart = 0; is_gb_cart = 1; gb_mbc = 1; gb_rom_bytes = 1048576; gb_ram_bytes = 32768; end
    15: begin is_gba_cart = 0; is_gb_cart = 1; gb_mbc = 2; gb_rom_bytes = 65536; gb_ram_bytes = 512; end
    16: begin is_gba_cart = 0; is_gb_cart = 1; gb_mbc = 3; gb_rom_bytes = 65536; gb_ram_bytes = 8192; end
    17: begin sig = "SRAM_V113";    sig_len = 9;  sav_size = 32768; end     // run with APF_LE=0
    endcase

    // GBA header
    put_byte(4, 8'h24); put_byte(5, 8'hFF); put_byte(6, 8'hAE); put_byte(7, 8'h51);
    put_str(16'hA0, "POKEMON EMER", 12);
    put_str(16'hAC, "BPEE", 4);
    put_str(16'hB0, "01", 2);
    put_byte(16'hB2, 8'h96);
    for (i = 16'hB3; i < 16'hBD; i = i + 1) put_byte(i, 8'h00);
    sum = 0;
    for (i = 16'hA0; i < 16'hBD; i = i + 1) sum = sum + (i[0] ? rom[i >> 1][15:8] : rom[i >> 1][7:0]);
    put_byte(16'hBD, (-sum - 8'h19) & 8'hFF);
    put_str(32'h2_1235, sig, sig_len);

    // GB header
    gbrom[16'h104] = 8'hCE; gbrom[16'h105] = 8'hED; gbrom[16'h106] = 8'h66; gbrom[16'h107] = 8'h66;
    gb_str(16'h134, "POKEMON_SLV", 11);
    for (i = 16'h13F; i < 16'h143; i = i + 1) gbrom[i] = "A";       // manufacturer code
    gbrom[16'h143] = (gb_mbc == 5) ? 8'hC0 : 8'h00;
    case (gb_mbc)
    1: gbrom[16'h147] = 8'h03;
    2: gbrom[16'h147] = 8'h06;
    3: gbrom[16'h147] = 8'h13;
    5: gbrom[16'h147] = 8'h1B;
    default: gbrom[16'h147] = 8'h00;
    endcase
    gbrom[16'h148] = (gb_rom_bytes == 32768) ? 0 : (gb_rom_bytes == 65536) ? 1 : (gb_rom_bytes == 131072) ? 2 :
                     (gb_rom_bytes == 1048576) ? 5 : 0;
    gbrom[16'h149] = (gb_ram_bytes == 32768) ? 3 : (gb_ram_bytes == 8192) ? 2 : 0;
    x = 0;
    for (i = 16'h134; i <= 16'h14C; i = i + 1) x = x - gbrom[i] - 1;
    gbrom[16'h14D] = x;

    repeat (20) @(posedge clk);
    reset_n = 1;
    repeat (50) @(posedge clk);
    cart_power = 1;
end

// injected faults
initial begin
    if (CT == 6) begin   // 3rd bank-select byte -> 0x80 (Flash erase prefix)
        wait (ctrl.state == 7'd16 && ctrl.bank_seq == 2'd2);
        force ctrl.bus_wdata = 8'h80;
    end
    if (CT == 10) begin  // 2nd EEPROM request bit -> 0 (write command)
        wait (ctrl.state == 7'd19 && ctrl.ee_reqn == 5'd1);
        force ctrl.bus_wdata = 8'h00;
    end
    if (CT == 16) begin  // engine bug: RAM /CS asserted during a mapper write
        // (the GB engine itself refuses writes at 8000+, so this models a
        // fault past it, which only the guard can catch)
        wait (ctrl.permit_mbc && !gbb.wr_n);
        force gbb.cs_n = 1'b0;
    end
end
initial if (CT == 7) begin
    wait (ctrl.state == 7'd42);
    put_byte(16'hA0, "X");
end

// status trace
reg [4:0] last_status = 5'h1f;
always @(posedge clk) if (status !== last_status) begin
    $display("[%0t ns] status=%0d ui=%0d kind=%0d save=%0d rom=%0d", $time, status, ui_mode, cart_kind, save_label, rom_log2);
    last_status = status;
end

// ------------------------------------------------------------------ run + check
task expect_status(input [4:0] s);
    begin
        if (status !== s) begin $display("ERROR: expected status %0d got %0d", s, status); errors = errors + 1; end
    end
endtask
task wait_result;
    begin
        wait (ui_mode == 3'd2);
        repeat (20) @(posedge clk);
        if (bus_enable) begin $display("ERROR: bus still enabled on the result screen"); errors = errors + 1; end
    end
endtask

initial begin
    wait (reset_n);
    case (CT)
    0: begin wait_result; expect_status(5'd2); end
    1, 2, 3, 4, 5, 17: begin
        choose(0); wait_result; expect_status(5'd8);
        check_file(0, (CT == 4) ? 512 : (CT == 5) ? 8192 : sav_size, (CT == 4 || CT == 5) ? 2 : 1);
        if (fpath[0][511 -: 8*43] != "/Assets/savesiphon/common/POKEMON_EMER_BPEE")
            begin $display("ERROR: path %0s", fpath[0]); errors = errors + 1; end
        if (CT == 1 && fpath[0][511-8*43 -: 8*6] != "_1.sav")
            begin $display("ERROR: suffix"); errors = errors + 1; end
        if (CT == 3 && fl_bank != 0) begin $display("ERROR: flash left on bank 1"); errors = errors + 1; end
    end
    6, 10: begin
        choose(0); wait_result; expect_status(5'd12);
        if (!guard_fault) begin $display("ERROR: guard did not fault"); errors = errors + 1; end
        $display("  guard code %0d", guard_code);
        if (nfiles != 0) begin $display("ERROR: a file was written"); errors = errors + 1; end
    end
    7: begin choose(0); wait_result; expect_status(5'd15); if (nfiles != 0) begin $display("ERROR: file written"); errors = errors + 1; end end
    8: begin choose(0); wait_result; expect_status(5'd14); if (nfiles != 0) begin $display("ERROR: file written"); errors = errors + 1; end end
    9: begin
        choose(0);
        wait (ui_mode == 3'd3); wait (ui_mode == 3'd1);      // B at the prompt: back to menu
        repeat (200) @(posedge clk);
        if (nfiles != 0 || bus_enable) begin $display("ERROR: cancel did something"); errors = errors + 1; end
    end
    11: begin
        wait (ui_mode == 3'd1);
        if (rom_log2 !== 5'd19) begin $display("ERROR: ROM size 2^%0d expected 2^19", rom_log2); errors = errors + 1; end
        choose(1); wait_result; expect_status(5'd8);
        check_file(0, 524288, 0);
        if (fpath[0][511 -: 8*47] != "/Assets/savesiphon/common/POKEMON_EMER_BPEE.gba")
            begin $display("ERROR: path %0s", fpath[0]); errors = errors + 1; end
    end
    12: begin
        choose(2); wait_result; expect_status(5'd8);
        if (done_what != 2'd3) begin $display("ERROR: done_what %0d", done_what); errors = errors + 1; end
        check_file(0, 32768, 1);
        check_file(1, 262144, 0);
    end
    13, 14, 15: begin
        wait (ui_mode == 3'd1);
        if (cart_kind != ((CT == 13) ? 2'd3 : 2'd2)) begin $display("ERROR: cart kind %0d", cart_kind); errors = errors + 1; end
        choose(2); wait_result; expect_status(5'd8);
        check_file(0, gb_ram_bytes, 4);
        check_file(1, gb_rom_bytes, 3);
        $display("  files: \"%0s\" / \"%0s\"", fpath[0], fpath[1]);
        if (gb_ram_en) begin $display("ERROR: cartridge RAM left enabled"); errors = errors + 1; end
        if (CT == 13 && fpath[1][511-8*26 -: 8*15] != "POKEMON_SLV.gbc") begin $display("ERROR: gbc name"); errors = errors + 1; end
    end
    16: begin
        choose(0); wait_result; expect_status(5'd12);
        $display("  guard code %0d", guard_code);
        if (guard_code != 3'd5) begin $display("ERROR: expected guard code 5"); errors = errors + 1; end
    end
    endcase
    if (errors == 0) $display("PASS ct=%0d apf_le=%0d", CT, `APF_LE);
    else             $display("FAIL ct=%0d apf_le=%0d errors=%0d", CT, `APF_LE, errors);
    $finish;
end

initial begin
    #3_000_000_000;
    $display("FAIL timeout ct=%0d state=%0d status=%0d", CT, ctrl.state, status);
    $finish;
end

endmodule
