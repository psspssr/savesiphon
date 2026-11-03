//
// siphon_ctrl.v - SaveSiphon sequencer
//
//   1. wait for cart power, let it settle
//   2. read the GBA header, refuse anything that is not a valid GBA cart
//   3. scan the first 16 MiB of ROM (then the next 16 MiB if needed) for the
//      save-library strings Nintendo's SDK leaves in every retail image:
//        EEPROM_V  SRAM_V  SRAM_F_V  FLASH_V  FLASH512_V  FLASH1M_V
//   4. read the save into the buffer
//        SRAM 32K / Flash 64K   plain CS2# reads
//        Flash 128K             64K, bank-select to bank 1, 64K, back to bank 0
//        EEPROM                 serial read requests; 512 B vs 8 KiB settled by
//                               the block-alias probe (64 wide reads)
//   5. read it a second time; both CRC32s must match (3 attempts)
//   6. re-read the header: it must be the same cart
//   7. pick a free file name, open it with create+resize, write the buffer
//
// Safety:
//   * Save types that need any command on the bus (Flash 128K bank select,
//     EEPROM read requests) wait for the user to press A (B cancels).
//   * The bus is released (enable low) while idle, waiting or halted.
//   * cart_write_guard.v independently checks every WR# strobe at the pins;
//     a refused strobe latches a fault and this sequencer stops for good.
//   * Loss of slot power halts immediately.
//
// Cartridge writes, exhaustively:
//   * Flash 128K bank select: AA->5555, 55->2AAA, B0->5555, {00|01}->0000.
//     These four (address, data) pairs are constants below; there is no path
//     that can emit any other save-space write (in particular no 0x80 erase
//     prefix and no 0xA0 program command).
//   * EEPROM read requests: bit pattern 1 1 <address> 0. The second bit is a
//     constant 1; a write command (1 0 ...) cannot be formed. The flush pattern
//     is all zeros, which is not a command.
//

`default_nettype none

module siphon_ctrl #(
    parameter [27:0] T_POWER_WAIT = 28'd111_375_000,   // 1.5 s fallback
    parameter [27:0] T_SETTLE     = 28'd18_562_500,    // 250 ms
    parameter [27:0] T_CMD_TIMEOUT= 28'd222_750_000,   // 3 s per APF command
    parameter [8:0]  SCAN_PAGES_LO = 9'd128,           // 16 MiB / 128 KiB
    parameter [8:0]  SCAN_PAGES_HI = 9'd256
) (
    input  wire         clk,            // clk_74a
    input  wire         reset_n,
    input  wire         cart_power,     // from host command 0x00B1
    input  wire         key_a,
    input  wire         key_b,

    // write guard (cart_write_guard.v)
    output reg          permit_bank,    // Flash bank-select window
    output reg          permit_ee,      // EEPROM read-request window
    input  wire         guard_fault,

    // cart bus engine
    output reg          bus_enable,
    output reg          bus_start,
    output reg  [2:0]   bus_op,
    output reg  [23:0]  bus_addr,
    output reg  [7:0]   bus_wdata,
    input  wire [15:0]  bus_rdata,
    input  wire         bus_done,

    // save buffer write port (word address, big-endian bytes)
    output reg          buf_we,
    output reg  [14:0]  buf_waddr,
    output reg  [31:0]  buf_wdata,

    // APF target commands
    output reg          tgt_openfile,
    output reg          tgt_write,
    output wire [15:0]  tgt_id,
    output wire [31:0]  tgt_slotoffset,
    output wire [31:0]  tgt_bridgeaddr,
    output reg  [31:0]  tgt_length,
    input  wire         tgt_ack,
    input  wire         tgt_done,
    input  wire [2:0]   tgt_err,

    // open_dataslot_file_t, read over the bridge
    input  wire [7:0]   struct_word_addr,
    output reg  [31:0]  struct_q,
    // byte order APF reads core memory in: 1 = byte 0 in [7:0] (measured on
    // hardware by openfpga-carttools), 0 = byte 0 in [31:24]. Applies to the
    // struct and to the save buffer, so path and data always agree.
    output reg          le_mode,

    // status for the screen
    output reg  [4:0]   status,
    output reg  [2:0]   save_type,
    output reg  [7:0]   progress,
    output reg  [2:0]   err_code,
    output wire [95:0]  title_flat,     // 12 header bytes, byte 0 in [95:88]
    output wire [31:0]  code_flat,
    output wire [511:0] path_flat       // 64 path bytes, byte 0 in [511:504]
);

// ------------------------------------------------------------------ constants
localparam [2:0] OP_ROM_LATCH = 3'd0, OP_ROM_RD = 3'd1, OP_EE_RD = 3'd2,
                 OP_EE_WR = 3'd3, OP_ROM_END = 3'd4, OP_SAVE_RD = 3'd5,
                 OP_SAVE_WR = 3'd6;

localparam [15:0] SLOT_ID   = 16'd1;
localparam [31:0] BUF_BASE  = 32'h4000_0000;
assign tgt_id         = SLOT_ID;
assign tgt_slotoffset = 32'd0;
assign tgt_bridgeaddr = BUF_BASE;

// status codes (status_screen.v has the strings)
localparam [4:0] ST_S_POWER   = 5'd0,  ST_S_HEADER  = 5'd1,  ST_S_NOCART = 5'd2,
                 ST_S_SCAN    = 5'd3,  ST_S_NOSAVE  = 5'd4,  ST_S_READ   = 5'd5,
                 ST_S_BLANK   = 5'd6,  ST_S_WRITE   = 5'd7,  ST_S_DONE   = 5'd8,
                 ST_S_FILEERR = 5'd9,  ST_S_TOOMANY = 5'd10, ST_S_TIMEOUT= 5'd11,
                 ST_S_SAFETY  = 5'd12, ST_S_CONFIRM = 5'd13, ST_S_UNSTABLE = 5'd14,
                 ST_S_CHANGED = 5'd15, ST_S_CANCEL  = 5'd16, ST_S_VERIFY = 5'd17,
                 ST_S_PWRLOST = 5'd18;

// save types
localparam [2:0] SV_NONE = 3'd0, SV_SRAM = 3'd1, SV_FLASH64 = 3'd2,
                 SV_FLASH128 = 3'd3, SV_EEPROM512 = 3'd4, SV_EEPROM8K = 3'd5,
                 SV_EEPROM = 3'd6;   // EEPROM, size not known yet

localparam [8*26-1:0] PATH_PREFIX = "/Assets/savesiphon/common/";

// EEPROM window: top of the 32 MiB ROM space, valid for every cart size
localparam [23:0] EE_ADDR = 24'hFFFF80;

// ------------------------------------------------------------------ state
localparam [5:0]
    S_RESET       = 6'd0,
    S_WAIT_POWER  = 6'd1,
    S_SETTLE      = 6'd2,
    S_BUSWAIT     = 6'd3,
    S_HDR_START   = 6'd4,
    S_HDR_RD      = 6'd5,
    S_HDR_GOT     = 6'd6,
    S_HDR_CHECK   = 6'd7,
    S_SCAN_PAGE   = 6'd8,
    S_SCAN_RD     = 6'd9,
    S_SCAN_B0     = 6'd10,
    S_SCAN_B1     = 6'd11,
    S_SCAN_NEXT   = 6'd12,
    S_DECIDE      = 6'd13,
    S_SAVE_RD     = 6'd14,
    S_SAVE_GOT    = 6'd15,
    S_BANK        = 6'd16,
    S_BANK_NEXT   = 6'd17,
    S_EE_BLK      = 6'd18,
    S_EE_REQ      = 6'd19,
    S_EE_REQ_NEXT = 6'd20,
    S_EE_RLATCH   = 6'd21,
    S_EE_RD       = 6'd22,
    S_EE_RGOT     = 6'd23,
    S_EE_PROBE    = 6'd24,
    S_EE_PROBE_GOT= 6'd25,
    S_EE_FLUSH_DONE = 6'd26,
    S_EE_DUMP     = 6'd27,
    S_EE_PUSH     = 6'd28,
    S_SAVE_END    = 6'd29,
    S_BUILD       = 6'd30,
    S_OPEN_PROBE  = 6'd31,
    S_OPEN_PROBE_R= 6'd32,
    S_OPEN_CREATE = 6'd33,
    S_OPEN_CREATE_R=6'd34,
    S_WRITE_FILE  = 6'd35,
    S_WRITE_FILE_R= 6'd36,
    S_TGT_WAIT    = 6'd37,
    S_HALT        = 6'd38,
    S_CONFIRM     = 6'd39,
    S_READ_BEGIN  = 6'd40,
    S_PASS_END    = 6'd41,
    S_HDRV_START  = 6'd42;

reg [5:0]  state, ret;
reg [27:0] timer;

// header
reg [7:0]  hdr [0:31];          // bytes 0xA0..0xBF
reg [31:0] logo;                // bytes 0x04..0x07
reg [6:0]  hidx;

// scan
reg [8:0]  page;
reg [15:0] widx;
reg [79:0] win;                 // last 10 ROM bytes, newest in [7:0]
reg        f_eeprom, f_sram, f_flash64, f_flash1m;
reg [15:0] rom_half;

// save read
reg [16:0] saddr;
reg [17:0] save_size;
reg [4:0]  size_log2;
reg [16:0] byte_idx;            // bytes pushed into the buffer
reg [23:0] pack;
reg [1:0]  bank_seq;
reg        bank_target;
reg [5:0]  bank_ret;
reg        seen_non_ff, seen_non_00;

// eeprom
reg [3:0]  ee_bits;             // 6 or 14
reg        ee_flush;
reg [9:0]  ee_blk;
reg [10:0] ee_nblk;
reg [16:0] ee_req;
reg [4:0]  ee_reqn;
reg [6:0]  ee_rcnt;
reg [63:0] ee_data;
reg [63:0] ee_ref;
reg        ee_diff;
reg [5:0]  ee_ret;
reg [2:0]  ee_k;

// file
reg [7:0]  path [0:63];
reg [5:0]  pidx;
reg [5:0]  bstep;
reg [3:0]  sfx_tens, sfx_ones;
reg [31:0] open_flags;
reg [31:0] open_size;
reg        tgt_seen;
reg [5:0]  tgt_ret;

reg        key_a_1, key_b_1;

// safety
reg [5:0]  settle_ret;
reg [5:0]  confirm_next;
reg        pass;                // 0: read into buffer, 1: verify read
reg [1:0]  retries;
reg [31:0] crc, crc1;
reg        hdr_verify, hdr_mismatch;
reg        power_seen;
reg        le_tried_be;

function [31:0] crc32_byte;
    input [31:0] c;
    input [7:0]  b;
    integer n;
    begin
        crc32_byte = c ^ {24'd0, b};
        for (n = 0; n < 8; n = n + 1)
            crc32_byte = crc32_byte[0] ? ((crc32_byte >> 1) ^ 32'hEDB88320) : (crc32_byte >> 1);
    end
endfunction

// ------------------------------------------------------------------ helpers
genvar gi;
generate
    for (gi = 0; gi < 12; gi = gi + 1) begin : g_title
        assign title_flat[95-8*gi -: 8] = hdr[gi];
    end
    for (gi = 0; gi < 4; gi = gi + 1) begin : g_code
        assign code_flat[31-8*gi -: 8] = hdr[12+gi];
    end
    for (gi = 0; gi < 64; gi = gi + 1) begin : g_path
        assign path_flat[511-8*gi -: 8] = path[gi];
    end
endgenerate

// header checksum over 0xA0..0xBC, accumulated while the header is read
reg [7:0] hsum;
wire [7:0] hchk     = 8'd0 - hsum - 8'h19;
wire       hdr_ok   = (hdr[18] == 8'h96) && (hchk == hdr[29]) && (logo == 32'h24FFAE51);

// title length: index after the last non-pad byte
reg [3:0] title_len;
integer ti;
always @(*) begin
    title_len = 4'd0;
    for (ti = 0; ti < 12; ti = ti + 1)
        if (hdr[ti] != 8'h00 && hdr[ti] != 8'h20) title_len = ti[3:0] + 4'd1;
end

function [7:0] fname_char;
    input [7:0] c;
    begin
        // only A-Z 0-9 _ : the set proven to be accepted on real cards
        if ((c >= "A" && c <= "Z") || (c >= "0" && c <= "9"))
            fname_char = c;
        else if (c >= "a" && c <= "z")
            fname_char = c - 8'h20;
        else
            fname_char = "_";
    end
endfunction

// signature match on the window after shifting in byte b
function [3:0] sig_match;   // {eeprom, sram, flash64, flash1m}
    input [79:0] w;
    begin
        sig_match[3] = (w[63:0] == "EEPROM_V");
        sig_match[2] = (w[47:0] == "SRAM_V") || (w[63:0] == "SRAM_F_V");
        sig_match[1] = (w[55:0] == "FLASH_V") || (w[79:0] == "FLASH512_V");
        sig_match[0] = (w[71:0] == "FLASH1M_V");
    end
endfunction

wire [79:0] win_b0 = {win[71:0], rom_half[7:0]};

// Flash 128K bank-select sequence: the only save-space writes in the design
reg [15:0] bank_addr;
reg [7:0]  bank_data;
always @(*) begin
    case (bank_seq)
    2'd0: begin bank_addr = 16'h5555; bank_data = 8'hAA; end
    2'd1: begin bank_addr = 16'h2AAA; bank_data = 8'h55; end
    2'd2: begin bank_addr = 16'h5555; bank_data = 8'hB0; end
    default: begin bank_addr = 16'h0000; bank_data = {7'd0, bank_target}; end
    endcase
end

// path builder: character for the current step, and whether to emit it
reg [7:0] b_char;
reg       b_emit;
reg       b_last;
always @(*) begin
    b_char = 8'h00;
    b_emit = 1'b1;
    b_last = 1'b0;
    if (bstep < 6'd26) begin
        b_char = PATH_PREFIX[8*(25-bstep) +: 8];
    end else if (bstep < 6'd38) begin
        if (title_len == 4'd0) begin
            b_emit = (bstep < 6'd29);
            b_char = (bstep == 6'd26) ? "G" : (bstep == 6'd27) ? "B" : "A";
        end else begin
            b_emit = ((bstep - 6'd26) < {2'b0, title_len});
            b_char = fname_char(hdr[bstep - 6'd26]);
        end
    end else if (bstep == 6'd38) begin
        b_char = "_";
    end else if (bstep < 6'd43) begin
        b_char = fname_char(hdr[12 + bstep - 6'd39]);
    end else if (bstep == 6'd43) begin
        b_emit = (sfx_tens != 4'd0) || (sfx_ones != 4'd0);
        b_char = "_";
    end else if (bstep == 6'd44) begin
        b_emit = (sfx_tens != 4'd0);
        b_char = "0" + sfx_tens;
    end else if (bstep == 6'd45) begin
        b_emit = (sfx_tens != 4'd0) || (sfx_ones != 4'd0);
        b_char = "0" + sfx_ones;
    end else if (bstep == 6'd46) begin
        b_char = ".";
    end else if (bstep == 6'd47) begin
        b_char = "s";
    end else if (bstep == 6'd48) begin
        b_char = "a";
    end else if (bstep == 6'd49) begin
        b_char = "v";
    end else begin
        b_char = 8'h00;
        b_last = 1'b1;
    end
end

// struct readback: path bytes, then flags at 0x100 and size at 0x104
always @(posedge clk) begin
    // the struct is a little-endian byte image: in le_mode byte 0 sits in
    // [7:0] and the u32 fields are plain; otherwise everything is mirrored
    if (struct_word_addr < 8'd16)
        struct_q <= le_mode ?
            {path[{struct_word_addr[3:0], 2'd3}], path[{struct_word_addr[3:0], 2'd2}],
             path[{struct_word_addr[3:0], 2'd1}], path[{struct_word_addr[3:0], 2'd0}]} :
            {path[{struct_word_addr[3:0], 2'd0}], path[{struct_word_addr[3:0], 2'd1}],
             path[{struct_word_addr[3:0], 2'd2}], path[{struct_word_addr[3:0], 2'd3}]};
    else if (struct_word_addr == 8'h40)
        struct_q <= le_mode ? open_flags :
            {open_flags[7:0], open_flags[15:8], open_flags[23:16], open_flags[31:24]};
    else if (struct_word_addr == 8'h41)
        struct_q <= le_mode ? open_size :
            {open_size[7:0], open_size[15:8], open_size[23:16], open_size[31:24]};
    else
        struct_q <= 32'd0;
end

// ------------------------------------------------------------------ tasks
task issue;
    input [2:0]  op;
    input [23:0] a;
    input [7:0]  d;
    input [5:0]  next;
    begin
        bus_start <= 1'b1;
        bus_op    <= op;
        bus_addr  <= a;
        bus_wdata <= d;
        ret       <= next;
        state     <= S_BUSWAIT;
    end
endtask

task push_byte;
    input [7:0] b;
    begin
        pack     <= {pack[15:0], b};
        crc      <= crc32_byte(crc, b);
        if (byte_idx[1:0] == 2'd3 && !pass) begin
            buf_we    <= 1'b1;
            buf_waddr <= byte_idx[16:2];
            buf_wdata <= {pack, b};
        end
        byte_idx <= byte_idx + 17'd1;
        if (b != 8'hFF && !pass) seen_non_ff <= 1'b1;
        if (b != 8'h00 && !pass) seen_non_00 <= 1'b1;
        progress <= {byte_idx, 8'd0} >> size_log2;
    end
endtask

task fail;
    input [4:0] st;
    begin
        status <= st;
        state  <= S_HALT;
    end
endtask

integer k;

// ------------------------------------------------------------------ FSM
always @(posedge clk) begin
    bus_start    <= 1'b0;
    buf_we       <= 1'b0;
    tgt_openfile <= 1'b0;
    tgt_write    <= 1'b0;
    key_a_1      <= key_a;
    key_b_1      <= key_b;
    if (timer != 28'd0) timer <= timer - 28'd1;

    if (!reset_n) begin
        state      <= S_RESET;
        bus_enable <= 1'b0;
        status     <= ST_S_POWER;
        save_type  <= SV_NONE;
        progress   <= 8'd0;
        err_code   <= 3'd0;
        timer      <= T_POWER_WAIT;
        open_flags <= 32'd0;
        open_size  <= 32'd0;
        tgt_length <= 32'd0;
        permit_bank <= 1'b0;
        permit_ee  <= 1'b0;
        power_seen <= 1'b0;
        le_mode    <= 1'b1;
        le_tried_be <= 1'b0;
        pass       <= 1'b0;
        hdr_verify <= 1'b0;
        settle_ret <= S_HDR_START;
        for (k = 0; k < 64; k = k + 1) path[k] <= 8'h00;
        for (k = 0; k < 32; k = k + 1) hdr[k] <= 8'h00;
    end else if (guard_fault && state != S_HALT) begin
        // the write guard refused a strobe: stop everything, for good
        bus_enable  <= 1'b0;
        permit_bank <= 1'b0;
        permit_ee   <= 1'b0;
        status      <= ST_S_SAFETY;
        state       <= S_HALT;
    end else if (power_seen && !cart_power && state != S_HALT) begin
        // the slot lost power (or the cart was pulled): stop driving pins
        bus_enable  <= 1'b0;
        permit_bank <= 1'b0;
        permit_ee   <= 1'b0;
        status      <= ST_S_PWRLOST;
        state       <= S_HALT;
    end else begin
        if (cart_power) power_seen <= 1'b1;
        case (state)
        S_RESET: begin
            timer <= T_POWER_WAIT;
            state <= S_WAIT_POWER;
        end

        S_WAIT_POWER: begin
            // cartridge_adapter bit 30 keeps the slot powered; 0x00B1 confirms
            // it on firmware that sends it, the timer covers firmware that does not
            if (cart_power || timer == 28'd0) begin
                bus_enable <= 1'b1;
                timer      <= T_SETTLE;
                settle_ret <= S_HDR_START;
                state      <= S_SETTLE;
            end
        end

        S_SETTLE: if (timer == 28'd0) state <= settle_ret;

        S_BUSWAIT: if (bus_done) state <= ret;

        // ---------------------------------------------------- header
        S_HDR_START: begin
            status    <= ST_S_HEADER;
            save_type <= SV_NONE;
            progress  <= 8'd0;
            err_code  <= 3'd0;
            hidx      <= 7'd0;
            logo      <= 32'd0;
            hsum      <= 8'd0;
            hdr_verify <= 1'b0;
            pass      <= 1'b0;
            for (k = 0; k < 64; k = k + 1) path[k] <= 8'h00;
            issue(OP_ROM_LATCH, 24'd0, 8'd0, S_HDR_RD);
        end
        S_HDR_RD: issue(OP_ROM_RD, 24'd0, 8'd0, S_HDR_GOT);
        S_HDR_GOT: begin
            // halfword hidx = bytes 2*hidx (low) and 2*hidx+1 (high)
            if (hdr_verify) begin
                // second header read after the dump: must match the first
                if (hidx == 7'd2 && {bus_rdata[7:0], bus_rdata[15:8]} != logo[31:16]) hdr_mismatch <= 1'b1;
                if (hidx == 7'd3 && {bus_rdata[7:0], bus_rdata[15:8]} != logo[15:0])  hdr_mismatch <= 1'b1;
                if (hidx >= 7'h50 && (hdr[{hidx[3:0], 1'b0}] != bus_rdata[7:0] ||
                                      hdr[{hidx[3:0], 1'b1}] != bus_rdata[15:8]))   hdr_mismatch <= 1'b1;
            end else begin
            if (hidx == 7'd2) logo[31:16] <= {bus_rdata[7:0], bus_rdata[15:8]};
            if (hidx == 7'd3) logo[15:0]  <= {bus_rdata[7:0], bus_rdata[15:8]};
            if (hidx >= 7'h50) begin
                hdr[{hidx[3:0], 1'b0}] <= bus_rdata[7:0];
                hdr[{hidx[3:0], 1'b1}] <= bus_rdata[15:8];
                // 0xA0..0xBC: halfwords 0x50..0x5D both bytes, 0x5E low byte only
                if (hidx < 7'h5E)
                    hsum <= hsum + bus_rdata[7:0] + bus_rdata[15:8];
                else if (hidx == 7'h5E)
                    hsum <= hsum + bus_rdata[7:0];
            end
            end
            if (hidx == 7'h5F) begin
                issue(OP_ROM_END, 24'd0, 8'd0, S_HDR_CHECK);
            end else begin
                hidx  <= hidx + 7'd1;
                state <= S_HDR_RD;
            end
        end
        S_HDR_CHECK: begin
            if (hdr_verify) begin
                if (hdr_mismatch) fail(ST_S_CHANGED);
                else              state <= S_SAVE_END;
            end else if (!hdr_ok) begin
                fail(ST_S_NOCART);
            end else begin
                status    <= ST_S_SCAN;
                page      <= 9'd0;
                win       <= 80'd0;
                f_eeprom  <= 1'b0;
                f_sram    <= 1'b0;
                f_flash64 <= 1'b0;
                f_flash1m <= 1'b0;
                state     <= S_SCAN_PAGE;
            end
        end

        // ---------------------------------------------------- ROM scan
        // A sequential burst cannot cross a 64K-halfword page, so re-latch
        // the address at every page.
        S_SCAN_PAGE: begin
            widx <= 16'd0;
            issue(OP_ROM_LATCH, {page[7:0], 16'h0000}, 8'd0, S_SCAN_RD);
        end
        S_SCAN_RD: issue(OP_ROM_RD, 24'd0, 8'd0, S_SCAN_B0);
        S_SCAN_B0: begin
            rom_half <= bus_rdata;
            state    <= S_SCAN_B1;
        end
        S_SCAN_B1: begin
            // two bytes, low address first
            win <= {win[63:0], rom_half[7:0], rom_half[15:8]};
            {f_eeprom, f_sram, f_flash64, f_flash1m} <=
                {f_eeprom, f_sram, f_flash64, f_flash1m} |
                sig_match(win_b0) | sig_match({win[63:0], rom_half[7:0], rom_half[15:8]});
            if (widx == 16'hFFFF) begin
                issue(OP_ROM_END, 24'd0, 8'd0, S_SCAN_NEXT);
            end else begin
                widx  <= widx + 16'd1;
                state <= S_SCAN_RD;
            end
        end
        S_SCAN_NEXT: begin
            page     <= page + 9'd1;
            progress <= (page[6:0] + 7'd1) << 1;
            if ((page + 9'd1 == SCAN_PAGES_LO &&
                    (f_eeprom | f_sram | f_flash64 | f_flash1m)) ||
                (page + 9'd1 == SCAN_PAGES_HI))
                state <= S_DECIDE;
            else
                state <= S_SCAN_PAGE;
        end

        // ---------------------------------------------------- decide
        S_DECIDE: begin
            progress    <= 8'd0;
            byte_idx    <= 17'd0;
            saddr       <= 17'd0;
            seen_non_ff <= 1'b0;
            seen_non_00 <= 1'b0;
            status      <= ST_S_READ;
            pass        <= 1'b0;
            retries     <= 2'd0;
            if (f_flash1m) begin
                // needs bank-select commands: ask first
                save_type <= SV_FLASH128; save_size <= 18'h20000; size_log2 <= 5'd17;
                confirm_next <= S_READ_BEGIN;
                state <= S_CONFIRM;
            end else if (f_flash64) begin
                save_type <= SV_FLASH64;  save_size <= 18'h10000; size_log2 <= 5'd16;
                state <= S_READ_BEGIN;
            end else if (f_sram) begin
                save_type <= SV_SRAM;     save_size <= 18'h08000; size_log2 <= 5'd15;
                state <= S_READ_BEGIN;
            end else if (f_eeprom) begin
                // needs serial read requests: ask first
                save_type <= SV_EEPROM;
                confirm_next <= S_EE_PROBE;
                state <= S_CONFIRM;
            end else begin
                fail(ST_S_NOSAVE);
            end
        end

        // ---------------------------------------------------- SRAM / Flash
        S_SAVE_RD: issue(OP_SAVE_RD, {8'd0, saddr[15:0]}, 8'd0, S_SAVE_GOT);
        S_SAVE_GOT: begin
            push_byte(bus_rdata[7:0]);
            saddr <= saddr + 17'd1;
            if ({1'b0, saddr} + 18'd1 == save_size) begin
                if (save_type == SV_FLASH128) begin
                    // leave the chip on bank 0, as a GBA would find it
                    bank_seq <= 2'd0; bank_target <= 1'b0; bank_ret <= S_PASS_END;
                    permit_bank <= 1'b1;
                    state <= S_BANK;
                end else begin
                    state <= S_PASS_END;
                end
            end else if (save_type == SV_FLASH128 && saddr == 17'h0FFFF) begin
                bank_seq <= 2'd0; bank_target <= 1'b1; bank_ret <= S_SAVE_RD;
                permit_bank <= 1'b1;
                state <= S_BANK;
            end else begin
                state <= S_SAVE_RD;
            end
        end
        S_BANK: issue(OP_SAVE_WR, {8'd0, bank_addr}, bank_data, S_BANK_NEXT);
        S_BANK_NEXT: begin
            bank_seq <= bank_seq + 2'd1;
            if (bank_seq == 2'd3) begin
                permit_bank <= 1'b0;
                state       <= bank_ret;
            end else begin
                state       <= S_BANK;
            end
        end

        // ---------------------------------------------------- EEPROM block read
        // in: ee_bits, ee_blk, ee_flush, ee_ret   out: ee_data
        S_EE_BLK: begin
            if (ee_flush)
                ee_req <= 17'd0;
            else if (ee_bits == 4'd14)
                ee_req <= {2'b11, 4'b0000, ee_blk[9:0], 1'b0};
            else
                ee_req <= {2'b11, ee_blk[5:0], 1'b0, 8'd0};
            ee_reqn <= 5'd0;
            permit_ee <= 1'b1;
            issue(OP_ROM_LATCH, EE_ADDR, 8'd0, S_EE_REQ);
        end
        S_EE_REQ: issue(OP_EE_WR, EE_ADDR, {7'd0, ee_req[16]}, S_EE_REQ_NEXT);
        S_EE_REQ_NEXT: begin
            ee_req  <= {ee_req[15:0], 1'b0};
            ee_reqn <= ee_reqn + 5'd1;
            if (ee_reqn + 5'd1 == ((ee_flush || ee_bits == 4'd14) ? 5'd17 : 5'd9)) begin
                permit_ee <= 1'b0;
                issue(OP_ROM_END, 24'd0, 8'd0, S_EE_RLATCH);
            end
            else
                state <= S_EE_REQ;
        end
        S_EE_RLATCH: begin
            ee_rcnt <= 7'd0;
            issue(OP_ROM_LATCH, EE_ADDR, 8'd0, S_EE_RD);
        end
        S_EE_RD: issue(OP_EE_RD, 24'd0, 8'd0, S_EE_RGOT);
        S_EE_RGOT: begin
            if (ee_rcnt >= 7'd4) ee_data <= {ee_data[62:0], bus_rdata[0]};
            ee_rcnt <= ee_rcnt + 7'd1;
            if (ee_rcnt == 7'd67)
                issue(OP_ROM_END, 24'd0, 8'd0, ee_ret);
            else
                state <= S_EE_RD;
        end

        // ---------------------------------------------------- EEPROM size probe
        // 64 reads at 14 address bits. A 512 B chip only decodes the first six
        // of them (all zero for blocks 0..63) and answers block 0 every time.
        S_EE_PROBE: begin
            ee_bits  <= 4'd14;
            ee_flush <= 1'b0;
            ee_blk   <= 10'd0;
            ee_diff  <= 1'b0;
            ee_ret   <= S_EE_PROBE_GOT;
            state    <= S_EE_BLK;
        end
        S_EE_PROBE_GOT: begin
            if (ee_blk == 10'd0)        ee_ref  <= ee_data;
            else if (ee_data != ee_ref) ee_diff <= 1'b1;
            progress <= {ee_blk[5:0], 2'b00};
            if (ee_blk == 10'd63) begin
                if (ee_diff || (ee_data != ee_ref)) begin
                    save_type <= SV_EEPROM8K; save_size <= 18'h02000; size_log2 <= 5'd13;
                    ee_bits <= 4'd14; ee_nblk <= 11'd1024;
                    state   <= S_READ_BEGIN;
                end else begin
                    // narrow chip, possibly mid-command after the wide reads:
                    // flush with zeros and drain one data phase first
                    save_type <= SV_EEPROM512; save_size <= 18'h00200; size_log2 <= 5'd9;
                    ee_bits <= 4'd6; ee_nblk <= 11'd64;
                    ee_flush <= 1'b1;
                    ee_ret  <= S_EE_FLUSH_DONE;
                    state   <= S_EE_BLK;
                end
            end else begin
                ee_blk <= ee_blk + 10'd1;
                state  <= S_EE_BLK;
            end
        end
        S_EE_FLUSH_DONE: begin
            ee_flush <= 1'b0;
            state    <= S_READ_BEGIN;
        end
        S_EE_DUMP: begin
            ee_blk   <= 10'd0;
            ee_ret   <= S_EE_PUSH;
            ee_k     <= 3'd0;
            state    <= S_EE_BLK;
        end
        S_EE_PUSH: begin
            push_byte(ee_data[63:56]);
            ee_data <= {ee_data[55:0], 8'd0};
            ee_k    <= ee_k + 3'd1;
            if (ee_k == 3'd7) begin
                ee_k <= 3'd0;
                if ({1'b0, ee_blk} + 11'd1 == ee_nblk) begin
                    state <= S_PASS_END;
                end else begin
                    ee_blk <= ee_blk + 10'd1;
                    state  <= S_EE_BLK;
                end
            end
        end

        // ---------------------------------------------------- safety
        // Flash 128K / EEPROM: the bus is released while waiting for the user
        S_CONFIRM: begin
            bus_enable <= 1'b0;
            status     <= ST_S_CONFIRM;
            if (key_a && !key_a_1) begin
                status     <= ST_S_READ;
                bus_enable <= 1'b1;
                timer      <= T_SETTLE;
                settle_ret <= confirm_next;
                state      <= S_SETTLE;
            end else if (key_b && !key_b_1) begin
                fail(ST_S_CANCEL);
            end
        end

        // every save is read twice; both CRC32s must match
        S_READ_BEGIN: begin
            byte_idx <= 17'd0;
            saddr    <= 17'd0;
            crc      <= 32'hFFFF_FFFF;
            progress <= 8'd0;
            status   <= pass ? ST_S_VERIFY : ST_S_READ;
            if (!pass) begin
                seen_non_ff <= 1'b0;
                seen_non_00 <= 1'b0;
            end
            if (save_type == SV_EEPROM512 || save_type == SV_EEPROM8K)
                state <= S_EE_DUMP;
            else
                state <= S_SAVE_RD;
        end
        S_PASS_END: begin
            if (!pass) begin
                crc1  <= crc;
                pass  <= 1'b1;
                state <= S_READ_BEGIN;
            end else if (crc == crc1) begin
                state <= S_HDRV_START;
            end else if (retries != 2'd2) begin
                retries <= retries + 2'd1;
                pass    <= 1'b0;
                state   <= S_READ_BEGIN;
            end else begin
                fail(ST_S_UNSTABLE);
            end
        end

        // the cart must still be the same cart
        S_HDRV_START: begin
            hdr_verify   <= 1'b1;
            hdr_mismatch <= 1'b0;
            hidx         <= 7'd0;
            issue(OP_ROM_LATCH, 24'd0, 8'd0, S_HDR_RD);
        end

        // ---------------------------------------------------- file
        S_SAVE_END: begin
            progress <= 8'd255;
            if (save_type == SV_EEPROM512 && !(seen_non_ff && seen_non_00)) begin
                // all 0x00 or all 0xFF from a narrow EEPROM read is
                // indistinguishable from no chip; refuse rather than guess
                fail(ST_S_BLANK);
            end else begin
                status   <= ST_S_WRITE;
                sfx_tens <= 4'd0;
                sfx_ones <= 4'd0;
                pidx     <= 6'd0;
                bstep    <= 6'd0;
                state    <= S_BUILD;
            end
        end

        S_BUILD: begin
            if (b_emit) begin
                path[pidx] <= b_char;
                pidx <= pidx + 6'd1;
            end
            if (b_last) state <= S_OPEN_PROBE;
            else        bstep <= bstep + 6'd1;
        end

        // probe: open without create; "file not found" means the name is free
        S_OPEN_PROBE: begin
            open_flags   <= 32'd0;
            open_size    <= 32'd0;
            tgt_openfile <= 1'b1;
            tgt_ret      <= S_OPEN_PROBE_R;
            tgt_seen     <= 1'b0;
            timer        <= T_CMD_TIMEOUT;
            state        <= S_TGT_WAIT;
        end
        S_OPEN_PROBE_R: begin
            if (tgt_err == 3'd3) begin
                state <= S_OPEN_CREATE;
            end else if (tgt_err == 3'd4 && !le_tried_be) begin
                // "malformed path": APF read the struct in the other byte
                // order. Flip once (data follows) and ask again.
                le_mode     <= ~le_mode;
                le_tried_be <= 1'b1;
                state       <= S_OPEN_PROBE;
            end else if (tgt_err == 3'd0) begin
                // taken: next suffix
                if (sfx_tens == 4'd9 && sfx_ones == 4'd9) begin
                    fail(ST_S_TOOMANY);
                end else begin
                    if (sfx_ones == 4'd9) begin
                        sfx_ones <= 4'd0;
                        sfx_tens <= sfx_tens + 4'd1;
                    end else begin
                        sfx_ones <= sfx_ones + 4'd1;
                    end
                    for (k = 0; k < 64; k = k + 1) path[k] <= 8'h00;
                    pidx  <= 6'd0;
                    bstep <= 6'd0;
                    state <= S_BUILD;
                end
            end else begin
                err_code <= tgt_err;
                fail(ST_S_FILEERR);
            end
        end

        S_OPEN_CREATE: begin
            open_flags   <= 32'h0000_0003;          // create | resize
            open_size    <= {14'd0, save_size};
            tgt_openfile <= 1'b1;
            tgt_ret      <= S_OPEN_CREATE_R;
            tgt_seen     <= 1'b0;
            timer        <= T_CMD_TIMEOUT;
            state        <= S_TGT_WAIT;
        end
        S_OPEN_CREATE_R: begin
            if (tgt_err == 3'd0 || tgt_err == 3'd1) begin
                state <= S_WRITE_FILE;
            end else begin
                err_code <= tgt_err;
                fail(ST_S_FILEERR);
            end
        end

        S_WRITE_FILE: begin
            tgt_length <= {14'd0, save_size};
            tgt_write  <= 1'b1;
            tgt_ret    <= S_WRITE_FILE_R;
            tgt_seen   <= 1'b0;
            timer      <= T_CMD_TIMEOUT;
            state      <= S_TGT_WAIT;
        end
        S_WRITE_FILE_R: begin
            if (tgt_err == 3'd0) begin
                status <= ST_S_DONE;
                state  <= S_HALT;
            end else begin
                err_code <= tgt_err;
                fail(ST_S_FILEERR);
            end
        end

        // done stays high from the previous command until core_bridge_cmd
        // starts this one, so wait to see it drop (or the ack) first
        S_TGT_WAIT: begin
            if (tgt_ack || !tgt_done) tgt_seen <= 1'b1;
            if (tgt_seen && tgt_done) state <= tgt_ret;
            else if (timer == 28'd0)  fail(ST_S_TIMEOUT);
        end

        // ---------------------------------------------------- terminal
        S_HALT: begin
            // pins released while idle. A runs everything again (same cart;
            // swap carts only with the Pocket powered off). A safety stop or
            // power loss needs a core restart.
            bus_enable  <= 1'b0;
            permit_bank <= 1'b0;
            permit_ee   <= 1'b0;
            if (key_a && !key_a_1 && status != ST_S_SAFETY && status != ST_S_PWRLOST) begin
                bus_enable <= 1'b1;
                timer      <= T_SETTLE;
                settle_ret <= S_HDR_START;
                state      <= S_SETTLE;
            end
        end

        default: state <= S_RESET;
        endcase
    end
end

endmodule

`default_nettype wire
