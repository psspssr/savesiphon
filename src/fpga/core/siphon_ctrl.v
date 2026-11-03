//
// siphon_ctrl.v - SaveSiphon sequencer
//
// Detect:   GBA header (logo, fixed byte, checksum) -> ROM scan for the SDK
//           save-library string -> ROM size probe. Otherwise GB/GBC header
//           (logo, checksum) -> mapper / ROM / RAM from the header.
// Menu:     Back up save / Back up ROM / Back up save + ROM / Rescan.
// Actions:  every save and every ROM chunk is read twice and the CRC32s must
//           match (3 attempts), the header is re-read afterwards and must be
//           unchanged, then the data goes to /Assets/savesiphon/common/.
//
// Cartridge writes, exhaustively:
//   GBA  Flash 128K bank select  AA->5555, 55->2AAA, B0->5555, {00|01}->0000
//   GBA  EEPROM read requests    1 1 <address> 0   (or an all-zero flush)
//   GB   mapper registers        0000-7FFF only, RAM /CS never asserted;
//                                0000-1FFF only 0A (RAM enable) / 00 (disable)
// Anything needing those asks the user first (A = go, B = back), and
// cart_write_guard.v checks every WR# strobe at the pins independently.
// Cartridge save data is never written.
//

`default_nettype none

module siphon_ctrl #(
    parameter [27:0] T_POWER_WAIT = 28'd111_375_000,   // 1.5 s fallback
    parameter [27:0] T_SETTLE     = 28'd18_562_500,    // 250 ms
    parameter [27:0] T_CMD_TIMEOUT= 28'd222_750_000,   // 3 s per APF command
    parameter [8:0]  SCAN_PAGES_LO = 9'd128,           // 16 MiB / 128 KiB
    parameter [8:0]  SCAN_PAGES_HI = 9'd256,
    parameter [4:0]  ROM_PROBE_MIN = 5'd18,            // 256 KiB
    parameter [4:0]  ROM_PROBE_MAX = 5'd24             // 16 MiB (32 MiB by exhaustion)
) (
    input  wire         clk,            // clk_74a
    input  wire         reset_n,
    input  wire         cart_power,     // host command 0x00B1
    input  wire         key_a,
    input  wire         key_b,
    input  wire         key_up,
    input  wire         key_down,

    // write guard
    output reg          permit_bank,
    output reg          permit_ee,
    output reg          permit_mbc,
    input  wire         guard_fault,

    // cart bus (routed to the GBA or GB engine by bus_mode)
    output reg          bus_enable,
    output reg          bus_mode,       // 0 GBA, 1 GB
    output reg          bus_start,
    output reg  [2:0]   bus_op,
    output reg  [23:0]  bus_addr,
    output reg  [7:0]   bus_wdata,
    input  wire [15:0]  bus_rdata,
    input  wire         bus_done,

    // buffer write port (word address, byte 0 in [31:24])
    output reg          buf_we,
    output reg  [14:0]  buf_waddr,
    output reg  [31:0]  buf_wdata,

    // APF target commands
    output reg          tgt_openfile,
    output reg          tgt_write,
    output wire [15:0]  tgt_id,
    output reg  [31:0]  tgt_slotoffset,
    output wire [31:0]  tgt_bridgeaddr,
    output reg  [31:0]  tgt_length,
    input  wire         tgt_ack,
    input  wire         tgt_done,
    input  wire [2:0]   tgt_err,

    input  wire [7:0]   struct_word_addr,
    output reg  [31:0]  struct_q,
    output reg          le_mode,

    // UI
    output reg  [2:0]   ui_mode,        // 0 busy, 1 menu, 2 result, 3 confirm
    output reg  [1:0]   menu_sel,
    output reg  [4:0]   status,
    output reg  [7:0]   progress,
    output reg  [2:0]   err_code,
    output reg  [1:0]   cart_kind,      // 0 none, 1 GBA, 2 GB, 3 GBC
    output wire [127:0] name_flat,      // 16 chars, display form
    output reg  [4:0]   name_len,
    output wire [31:0]  code_flat,
    output wire         code_len,       // 1: 4-char game code present
    output reg  [3:0]   save_label,
    output reg  [4:0]   rom_log2,
    output reg  [2:0]   mapper,
    output reg  [1:0]   done_what,      // 1 save, 2 ROM, 3 both
    output wire [511:0] path_flat
);

// ------------------------------------------------------------------ constants
localparam [2:0] OP_ROM_LATCH = 3'd0, OP_ROM_RD = 3'd1, OP_EE_RD = 3'd2,
                 OP_EE_WR = 3'd3, OP_ROM_END = 3'd4, OP_SAVE_RD = 3'd5,
                 OP_SAVE_WR = 3'd6;
localparam [2:0] OP_GB_RD = 3'd0, OP_GB_WR = 3'd1;

assign tgt_id         = 16'd1;
assign tgt_bridgeaddr = 32'h4000_0000;

localparam [4:0] ST_S_POWER   = 5'd0,  ST_S_HEADER  = 5'd1,  ST_S_NOCART = 5'd2,
                 ST_S_SCAN    = 5'd3,  ST_S_NOSAVE  = 5'd4,  ST_S_READ   = 5'd5,
                 ST_S_BLANK   = 5'd6,  ST_S_WRITE   = 5'd7,  ST_S_DONE   = 5'd8,
                 ST_S_FILEERR = 5'd9,  ST_S_TOOMANY = 5'd10, ST_S_TIMEOUT= 5'd11,
                 ST_S_SAFETY  = 5'd12, ST_S_CONFIRM = 5'd13, ST_S_UNSTABLE = 5'd14,
                 ST_S_CHANGED = 5'd15, ST_S_CANCEL  = 5'd16, ST_S_VERIFY = 5'd17,
                 ST_S_PWRLOST = 5'd18, ST_S_ROMREAD = 5'd19, ST_S_UNSUP  = 5'd20,
                 ST_S_READY   = 5'd21, ST_S_SIZING  = 5'd22;

localparam [2:0] UI_BUSY = 3'd0, UI_MENU = 3'd1, UI_RESULT = 3'd2, UI_CONFIRM = 3'd3;

// save labels (status_screen.v has the strings)
localparam [3:0] SV_NONE = 4'd0, SV_SRAM = 4'd1, SV_FLASH64 = 4'd2,
                 SV_FLASH128 = 4'd3, SV_EEPROM512 = 4'd4, SV_EEPROM8K = 4'd5,
                 SV_EEPROM = 4'd6, SV_GB2K = 4'd7, SV_GB8K = 4'd8,
                 SV_GB32K = 4'd9, SV_GB64K = 4'd10, SV_GB128K = 4'd11,
                 SV_MBC2 = 4'd12;

localparam [2:0] M_ROM = 3'd0, M_MBC1 = 3'd1, M_MBC2 = 3'd2, M_MBC3 = 3'd3,
                 M_MBC5 = 3'd5, M_UNSUP = 3'd7;

localparam [1:0] K_NONE = 2'd0, K_GBA = 2'd1, K_GB = 2'd2, K_GBC = 2'd3;

localparam [8*26-1:0] PATH_PREFIX = "/Assets/savesiphon/common/";
localparam [23:0] EE_ADDR = 24'hFFFF80;

// ------------------------------------------------------------------ states
localparam [6:0]
    S_RESET = 7'd0, S_WAIT_POWER = 7'd1, S_SETTLE = 7'd2, S_BUSWAIT = 7'd3,
    S_DETECT = 7'd4, S_HDR_RD = 7'd5, S_HDR_GOT = 7'd6, S_HDR_CHECK = 7'd7,
    S_SCAN_PAGE = 7'd8, S_SCAN_RD = 7'd9, S_SCAN_B0 = 7'd10, S_SCAN_B1 = 7'd11,
    S_SCAN_NEXT = 7'd12, S_DECIDE = 7'd13, S_SAVE_RD = 7'd14, S_SAVE_GOT = 7'd15,
    S_BANK = 7'd16, S_BANK_NEXT = 7'd17, S_EE_BLK = 7'd18, S_EE_REQ = 7'd19,
    S_EE_REQ_NEXT = 7'd20, S_EE_RLATCH = 7'd21, S_EE_RD = 7'd22, S_EE_RGOT = 7'd23,
    S_EE_PROBE = 7'd24, S_EE_PROBE_GOT = 7'd25, S_EE_FLUSH_DONE = 7'd26,
    S_EE_DUMP = 7'd27, S_EE_PUSH = 7'd28, S_BUILD = 7'd30,
    S_OPEN_PROBE = 7'd31, S_OPEN_PROBE_R = 7'd32, S_OPEN_CREATE = 7'd33,
    S_OPEN_CREATE_R = 7'd34, S_CHUNK = 7'd35, S_CHUNK_R = 7'd36,
    S_TGT_WAIT = 7'd37, S_HALT = 7'd38, S_CONFIRM = 7'd39, S_READ_BEGIN = 7'd40,
    S_PASS_END = 7'd41, S_HDRV_START = 7'd42,
    // ROM size probe (GBA)
    S_RSZ_START = 7'd43, S_RSZ_PT = 7'd44, S_RSZ_H0 = 7'd45, S_RSZ_H1 = 7'd46,
    S_RSZ_HE = 7'd47, S_RSZ_M = 7'd48, S_RSZ_M0 = 7'd49, S_RSZ_M1 = 7'd50,
    S_RSZ_ME = 7'd51, S_RSZ_EVAL = 7'd52,
    // GB
    S_GB_DETECT = 7'd53, S_GB_HDR_RD = 7'd54, S_GB_HDR_GOT = 7'd55, S_GB_HDR_CHECK = 7'd56,
    S_WQ = 7'd57, S_WQ_NEXT = 7'd58,
    S_GB_SAVE_BEGIN = 7'd59, S_GB_RAM_BANK = 7'd60, S_GB_RAM_RD = 7'd61,
    S_GB_RAM_GOT = 7'd62, S_GB_RAM_DONE = 7'd63,
    // menu / actions
    S_MENU = 7'd64, S_ACT_START = 7'd65, S_ACT_SAVE = 7'd66, S_SAVE_FILE = 7'd67,
    S_ACT_ROM = 7'd68, S_ROM_CHUNK = 7'd69, S_ROM_CHUNK_RD = 7'd70,
    S_ROM_CHUNK_GOT = 7'd71, S_ROM_CHUNK_B1 = 7'd72, S_ROM_PASS_END = 7'd73,
    S_ROM_NEXT = 7'd74, S_ROM_FINISH = 7'd75, S_ACT_DONE = 7'd76,
    S_GB_ROM_SEL = 7'd77, S_GB_ROM_RD = 7'd78, S_GB_ROM_GOT = 7'd79,
    S_MODE_SWITCH = 7'd80, S_GB_ROM_RESTORE = 7'd81, S_FILE_OPEN = 7'd82,
    S_ROM_OPENED = 7'd83, S_HDRV_DONE = 7'd84, S_GB_RAM_END = 7'd85;

reg [6:0]  state, ret;
reg [27:0] timer;

// ------------------------------------------------------------------ registers
// header bytes: GBA 0xA0..0xBF, GB 0x130..0x14F
reg [7:0]  hdr [0:31];
reg [31:0] logo;
reg [6:0]  hidx;
reg [7:0]  hsum;
reg        hdr_verify, hdr_mismatch;

// GBA scan
reg [8:0]  page;
reg [15:0] widx;
reg [79:0] win;
reg        f_eeprom, f_sram, f_flash64, f_flash1m;
reg [15:0] rom_half;

// save read
reg [16:0] saddr;
reg [17:0] save_size;
reg [4:0]  size_log2;
reg [16:0] byte_idx;
reg [23:0] pack;
reg [1:0]  bank_seq;
reg        bank_target;
reg [6:0]  bank_ret;
reg        seen_non_ff, seen_non_00;

// eeprom
reg [3:0]  ee_bits;
reg        ee_flush;
reg [9:0]  ee_blk;
reg [10:0] ee_nblk;
reg [16:0] ee_req;
reg [4:0]  ee_reqn;
reg [6:0]  ee_rcnt;
reg [63:0] ee_data;
reg [63:0] ee_ref;
reg        ee_diff;
reg [6:0]  ee_ret;
reg [2:0]  ee_k;

// ROM size probe
reg [4:0]  cand;            // log2 of candidate size
reg [1:0]  pt;              // sample point
reg        all_end;
reg [15:0] h0, h1, m0, m1;

// GB
reg        gb_has_ram;
reg [4:0]  gb_ram_banks;    // number of RAM banks
reg [13:0] gb_bank_bytes;   // bytes per RAM bank (512, 2048, 8192)
reg [4:0]  gb_bank;         // current RAM bank
reg [13:0] gb_off;          // offset within bank
reg [8:0]  rom_bank;        // GB ROM bank
reg [15:0] gb_base;

// write queue for GB mapper registers
reg [15:0] wq_a [0:3];
reg [7:0]  wq_d [0:3];
reg [2:0]  wq_n, wq_i;
reg [6:0]  wq_ret;

// actions
reg        act_save, act_rom;
reg [9:0]  chunk;           // ROM chunk index
reg [9:0]  nchunks;
reg [17:0] chunk_bytes;     // bytes per chunk (save up to 128K, GBA ROM 64K, GB 16K)
reg [17:0] chunk_pos;       // position inside the chunk
reg [6:0]  open_ret;            // after the file is opened
reg [6:0]  chunk_ret;           // after a chunk is written
reg [6:0]  hdrv_ret;            // after the header re-check
reg [1:0]  file_ext;        // 0 sav, 1 gba, 2 gb, 3 gbc
reg [31:0] file_size;
reg [6:0]  confirm_next;

// file
reg [7:0]  path [0:63];
reg [5:0]  pidx;
reg [5:0]  bstep;
reg [3:0]  sfx_tens, sfx_ones;
reg [31:0] open_flags;
reg [31:0] open_size;
reg        tgt_seen;
reg [6:0]  tgt_ret;
reg        le_tried_be;

// safety
reg [6:0]  settle_ret;
reg        pass;
reg [1:0]  retries;
reg [31:0] crc, crc1;
reg        power_seen;
reg        fatal;               // safety stop / power lost: no menu any more
reg        want_mode;

reg        key_a_1, key_b_1, key_up_1, key_down_1;
wire       a_edge    = key_a && !key_a_1;
wire       b_edge    = key_b && !key_b_1;
wire       up_edge   = key_up && !key_up_1;
wire       down_edge = key_down && !key_down_1;

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

// ------------------------------------------------------------------ name
// GBA: title 0xA0..0xAB (12), code 0xAC..0xAF.  GB: title 0x134.. (15, or 11
// on CGB carts where the tail is a manufacturer code), no game code.
wire       is_gba   = (cart_kind == K_GBA);
wire [3:0] name_max = is_gba ? 4'd12 : (hdr[19][7] ? 4'd11 : 4'd15);
assign code_len = is_gba;

reg [7:0] nm [0:15];
integer ni;
always @(*) begin
    for (ni = 0; ni < 16; ni = ni + 1) begin
        if (ni >= name_max) nm[ni] = 8'h00;
        else if (is_gba)    nm[ni] = hdr[ni];
        else                nm[ni] = hdr[4 + ni];
    end
end

genvar gi;
generate
    for (gi = 0; gi < 16; gi = gi + 1) begin : g_name
        assign name_flat[127-8*gi -: 8] = nm[gi];
    end
    for (gi = 0; gi < 4; gi = gi + 1) begin : g_code
        assign code_flat[31-8*gi -: 8] = hdr[12+gi];
    end
    for (gi = 0; gi < 64; gi = gi + 1) begin : g_path
        assign path_flat[511-8*gi -: 8] = path[gi];
    end
endgenerate

integer ti;
reg [4:0] nlen_c;
reg       gb_nul;
always @(*) begin
    nlen_c = 5'd0;
    gb_nul = 1'b0;
    for (ti = 0; ti < 16; ti = ti + 1) begin
        if (nm[ti] == 8'h00 && !is_gba) gb_nul = 1'b1;   // GB titles end at NUL
        if (!gb_nul && nm[ti] != 8'h00 && nm[ti] != 8'h20) nlen_c = ti[4:0] + 5'd1;
    end
end
always @(posedge clk) name_len <= nlen_c;

function [7:0] fname_char;
    input [7:0] c;
    begin
        if ((c >= "A" && c <= "Z") || (c >= "0" && c <= "9"))
            fname_char = c;
        else if (c >= "a" && c <= "z")
            fname_char = c - 8'h20;
        else
            fname_char = "_";
    end
endfunction

// ------------------------------------------------------------------ GBA header check
wire [7:0] hchk   = 8'd0 - hsum - 8'h19;
wire       gba_ok = (hdr[18] == 8'h96) && (hchk == hdr[29]) && (logo == 32'h24FFAE51);
// GB: x = 0; for 0x134..0x14C: x = x - b - 1  (hsum holds the running x)
wire       gb_ok  = (hsum == hdr[29]) && (logo == 32'hCEED6666);

// ------------------------------------------------------------------ signatures
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

// Flash 128K bank select: the only GBA save-space writes in the design
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

// ------------------------------------------------------------------ ROM probe points
// point k of candidate S: S + k*S/4 + k*0x246 (bytes); its image wrapped by S
wire [25:0] cand_s   = 26'd1 << cand;
wire [25:0] pt_off   = ({24'd0, pt} * (cand_s >> 2)) + ({24'd0, pt} * 26'h246);
wire [25:0] pt_addr  = cand_s + pt_off;
wire [25:0] pt_wrap  = pt_addr & (cand_s - 26'd1);
wire [15:0] pt_a16   = pt_addr[16:1];
wire [15:0] pt_w16   = pt_wrap[16:1];
wire        pt_open  = (h0 == pt_a16) && (h1 != pt_a16 + 16'd1);
wire        ref_open = (m0 == pt_w16) && (m1 != pt_w16 + 16'd1);
wire        pt_mirror= (h0 == m0) && (h1 == m1) && (h0 != h1) && !ref_open;

// ------------------------------------------------------------------ path builder
reg [7:0] b_char;
reg       b_emit;
reg       b_last;
reg [3:0] b_i;
always @(*) begin
    b_char = 8'h00;
    b_emit = 1'b1;
    b_last = 1'b0;
    b_i    = bstep[3:0] - 4'd10;      // bstep 26..41 -> 0..15
    if (bstep < 6'd26) begin
        b_char = PATH_PREFIX[8*(25-bstep) +: 8];
    end else if (bstep < 6'd42) begin
        if (name_len == 5'd0) begin
            b_emit = (bstep < 6'd30);
            b_char = (bstep == 6'd26) ? "C" : (bstep == 6'd27) ? "A" :
                     (bstep == 6'd28) ? "R" : "T";
        end else begin
            b_emit = ({1'b0, b_i} < name_len);
            b_char = fname_char(nm[b_i]);
        end
    end else if (bstep == 6'd42) begin
        b_emit = code_len;
        b_char = "_";
    end else if (bstep < 6'd47) begin
        b_emit = code_len;
        b_char = fname_char(hdr[12 + bstep - 6'd43]);
    end else if (bstep == 6'd47) begin
        b_emit = (sfx_tens != 4'd0) || (sfx_ones != 4'd0);
        b_char = "_";
    end else if (bstep == 6'd48) begin
        b_emit = (sfx_tens != 4'd0);
        b_char = "0" + sfx_tens;
    end else if (bstep == 6'd49) begin
        b_emit = (sfx_tens != 4'd0) || (sfx_ones != 4'd0);
        b_char = "0" + sfx_ones;
    end else if (bstep == 6'd50) begin
        b_char = ".";
    end else if (bstep < 6'd54) begin
        case (file_ext)
        2'd0: b_char = (bstep == 6'd51) ? "s" : (bstep == 6'd52) ? "a" : "v";
        2'd1: b_char = (bstep == 6'd51) ? "g" : (bstep == 6'd52) ? "b" : "a";
        2'd2: begin b_char = (bstep == 6'd51) ? "g" : "b"; b_emit = (bstep != 6'd53); end
        default: b_char = (bstep == 6'd51) ? "g" : (bstep == 6'd52) ? "b" : "c";
        endcase
    end else begin
        b_char = 8'h00;
        b_last = 1'b1;
    end
end

// struct readback (little-endian byte image in le_mode)
always @(posedge clk) begin
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
    input [6:0]  next;
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
        pack <= {pack[15:0], b};
        crc  <= crc32_byte(crc, b);
        if (byte_idx[1:0] == 2'd3 && !pass) begin
            buf_we    <= 1'b1;
            buf_waddr <= byte_idx[16:2];
            buf_wdata <= {pack, b};
        end
        byte_idx <= byte_idx + 17'd1;
        if (b != 8'hFF && !pass) seen_non_ff <= 1'b1;
        if (b != 8'h00 && !pass) seen_non_00 <= 1'b1;
    end
endtask

task fail;
    input [4:0] st;
    begin
        status      <= st;
        ui_mode     <= UI_RESULT;
        bus_enable  <= 1'b0;
        permit_bank <= 1'b0;
        permit_ee   <= 1'b0;
        permit_mbc  <= 1'b0;
        state       <= S_HALT;
    end
endtask

// queue GB mapper writes (up to 4) and run them, then go to `next`
task wq1;
    input [15:0] a0; input [7:0] d0; input [6:0] next;
    begin
        wq_a[0] <= a0; wq_d[0] <= d0; wq_n <= 3'd1; wq_i <= 3'd0; wq_ret <= next; state <= S_WQ;
    end
endtask
task wq2;
    input [15:0] a0; input [7:0] d0; input [15:0] a1; input [7:0] d1; input [6:0] next;
    begin
        wq_a[0] <= a0; wq_d[0] <= d0; wq_a[1] <= a1; wq_d[1] <= d1;
        wq_n <= 3'd2; wq_i <= 3'd0; wq_ret <= next; state <= S_WQ;
    end
endtask
task wq3;
    input [15:0] a0; input [7:0] d0; input [15:0] a1; input [7:0] d1;
    input [15:0] a2; input [7:0] d2; input [6:0] next;
    begin
        wq_a[0] <= a0; wq_d[0] <= d0; wq_a[1] <= a1; wq_d[1] <= d1;
        wq_a[2] <= a2; wq_d[2] <= d2;
        wq_n <= 3'd3; wq_i <= 3'd0; wq_ret <= next; state <= S_WQ;
    end
endtask

// enable the bus in mode m, settle, then continue at `next`
task bus_up;
    input       m;
    input [6:0] next;
    begin
        if (bus_enable && bus_mode == m) begin
            state <= next;
        end else begin
            bus_enable <= 1'b0;
            want_mode  <= m;
            settle_ret <= next;
            state      <= S_MODE_SWITCH;
        end
    end
endtask

// open a new file (free name, create, resize) then continue at `next`
task file_open;
    input [1:0]  ext;
    input [31:0] size;
    input [6:0]  next;
    begin
        file_ext  <= ext;
        file_size <= size;
        open_ret  <= next;
        state     <= S_FILE_OPEN;
    end
endtask

// does this save / ROM need any command on the bus?
wire save_needs_cmd = is_gba ? (save_label == SV_FLASH128 || save_label == SV_EEPROM ||
                                save_label == SV_EEPROM512 || save_label == SV_EEPROM8K)
                             : (mapper != M_ROM);
wire rom_needs_cmd  = !is_gba && (rom_log2 > 5'd15);
wire has_save       = (save_label != SV_NONE);

integer k;

// ------------------------------------------------------------------ FSM
always @(posedge clk) begin
    bus_start    <= 1'b0;
    buf_we       <= 1'b0;
    tgt_openfile <= 1'b0;
    tgt_write    <= 1'b0;
    key_a_1      <= key_a;
    key_b_1      <= key_b;
    key_up_1     <= key_up;
    key_down_1   <= key_down;
    if (timer != 28'd0) timer <= timer - 28'd1;

    if (!reset_n) begin
        state       <= S_RESET;
        bus_enable  <= 1'b0;
        bus_mode    <= 1'b0;
        want_mode   <= 1'b0;
        status      <= ST_S_POWER;
        ui_mode     <= UI_BUSY;
        menu_sel    <= 2'd0;
        cart_kind   <= K_NONE;
        save_label  <= SV_NONE;
        rom_log2    <= 5'd0;
        mapper      <= M_ROM;
        done_what   <= 2'd0;
        progress    <= 8'd0;
        err_code    <= 3'd0;
        timer       <= T_POWER_WAIT;
        open_flags  <= 32'd0;
        open_size   <= 32'd0;
        tgt_length  <= 32'd0;
        tgt_slotoffset <= 32'd0;
        permit_bank <= 1'b0;
        permit_ee   <= 1'b0;
        permit_mbc  <= 1'b0;
        power_seen  <= 1'b0;
        fatal       <= 1'b0;
        le_mode     <= 1'b1;
        le_tried_be <= 1'b0;
        pass        <= 1'b0;
        hdr_verify  <= 1'b0;
        settle_ret  <= S_DETECT;
        for (k = 0; k < 64; k = k + 1) path[k] <= 8'h00;
        for (k = 0; k < 32; k = k + 1) hdr[k] <= 8'h00;
    end else if (guard_fault && !fatal) begin
        // the write guard refused a strobe: stop everything, for good
        fatal <= 1'b1;
        fail(ST_S_SAFETY);
    end else if (power_seen && !cart_power && !fatal) begin
        fatal <= 1'b1;
        fail(ST_S_PWRLOST);
    end else begin
        if (cart_power) power_seen <= 1'b1;
        case (state)
        S_RESET: begin
            timer <= T_POWER_WAIT;
            state <= S_WAIT_POWER;
        end
        S_WAIT_POWER: if (cart_power || timer == 28'd0) begin
            want_mode  <= 1'b0;
            settle_ret <= S_DETECT;
            state      <= S_MODE_SWITCH;
        end

        // bus is off here; select the mode, power the engine, settle
        S_MODE_SWITCH: begin
            bus_mode   <= want_mode;
            bus_enable <= 1'b1;
            timer      <= T_SETTLE;
            state      <= S_SETTLE;
        end
        S_SETTLE: if (timer == 28'd0) state <= settle_ret;

        S_BUSWAIT: if (bus_done) state <= ret;

        // ============================================================ detect
        S_DETECT: begin
            if (bus_mode != 1'b0 || !bus_enable) begin
                bus_up(1'b0, S_DETECT);
            end else begin
                ui_mode    <= UI_BUSY;
                status     <= ST_S_HEADER;
                cart_kind  <= K_NONE;
                save_label <= SV_NONE;
                rom_log2   <= 5'd0;
                progress   <= 8'd0;
                err_code   <= 3'd0;
                hidx       <= 7'd0;
                logo       <= 32'd0;
                hsum       <= 8'd0;
                hdr_verify <= 1'b0;
                for (k = 0; k < 64; k = k + 1) path[k] <= 8'h00;
                issue(OP_ROM_LATCH, 24'd0, 8'd0, S_HDR_RD);
            end
        end

        // ---------------------------------------------------- GBA header
        S_HDR_RD: issue(OP_ROM_RD, 24'd0, 8'd0, S_HDR_GOT);
        S_HDR_GOT: begin
            if (hdr_verify) begin
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
                    if (hidx < 7'h5E)       hsum <= hsum + bus_rdata[7:0] + bus_rdata[15:8];
                    else if (hidx == 7'h5E) hsum <= hsum + bus_rdata[7:0];
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
                state <= S_HDRV_DONE;
            end else if (!gba_ok) begin
                state <= S_GB_DETECT;            // not a GBA cart: try Game Boy
            end else begin
                cart_kind <= K_GBA;
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

        // ---------------------------------------------------- GBA save-type scan
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
            if ((page + 9'd1 == SCAN_PAGES_LO && (f_eeprom | f_sram | f_flash64 | f_flash1m)) ||
                (page + 9'd1 == SCAN_PAGES_HI))
                state <= S_DECIDE;
            else
                state <= S_SCAN_PAGE;
        end
        S_DECIDE: begin
            if      (f_flash1m) begin save_label <= SV_FLASH128; save_size <= 18'h20000; size_log2 <= 5'd17; end
            else if (f_flash64) begin save_label <= SV_FLASH64;  save_size <= 18'h10000; size_log2 <= 5'd16; end
            else if (f_sram)    begin save_label <= SV_SRAM;     save_size <= 18'h08000; size_log2 <= 5'd15; end
            else if (f_eeprom)  begin save_label <= SV_EEPROM; end
            else                begin save_label <= SV_NONE; end
            state <= S_RSZ_START;
        end

        // ---------------------------------------------------- GBA ROM size probe
        // Past the end of ROM the bus floats at the address just driven, so a
        // read returns its own halfword address (and the same value again on
        // the sequential read). Smallest candidate whose four sample points
        // are all open bus or a mirror of the start is the size.
        S_RSZ_START: begin
            status <= ST_S_SIZING;
            cand   <= ROM_PROBE_MIN;
            pt     <= 2'd0;
            all_end<= 1'b1;
            state  <= S_RSZ_PT;
        end
        S_RSZ_PT: begin
            // EEPROM carts answer at 16 MiB and up; they are at most 16 MiB
            if (cand == 5'd24 && f_eeprom && !(f_flash1m | f_flash64 | f_sram)) begin
                rom_log2 <= 5'd24;
                state    <= S_MENU;
            end else begin
                issue(OP_ROM_LATCH, pt_addr[24:1], 8'd0, S_RSZ_H0);
            end
        end
        S_RSZ_H0: issue(OP_ROM_RD, 24'd0, 8'd0, S_RSZ_H1);
        S_RSZ_H1: begin
            h0 <= bus_rdata;
            issue(OP_ROM_RD, 24'd0, 8'd0, S_RSZ_HE);
        end
        S_RSZ_HE: begin
            h1 <= bus_rdata;
            issue(OP_ROM_END, 24'd0, 8'd0, S_RSZ_M);
        end
        S_RSZ_M:  issue(OP_ROM_LATCH, pt_wrap[24:1], 8'd0, S_RSZ_M0);
        S_RSZ_M0: issue(OP_ROM_RD, 24'd0, 8'd0, S_RSZ_M1);
        S_RSZ_M1: begin
            m0 <= bus_rdata;
            issue(OP_ROM_RD, 24'd0, 8'd0, S_RSZ_ME);
        end
        S_RSZ_ME: begin
            m1 <= bus_rdata;
            issue(OP_ROM_END, 24'd0, 8'd0, S_RSZ_EVAL);
        end
        S_RSZ_EVAL: begin
            if (pt != 2'd3) begin
                all_end <= all_end && (pt_open || pt_mirror);
                pt      <= pt + 2'd1;
                state   <= S_RSZ_PT;
            end else if (all_end && (pt_open || pt_mirror)) begin
                rom_log2 <= cand;
                state    <= S_MENU;
            end else if (cand == ROM_PROBE_MAX) begin
                rom_log2 <= 5'd25;              // 32 MiB: the bus can't address more
                state    <= S_MENU;
            end else begin
                cand    <= cand + 5'd1;
                pt      <= 2'd0;
                all_end <= 1'b1;
                state   <= S_RSZ_PT;
            end
        end

        // ---------------------------------------------------- GB header
        S_GB_DETECT: begin
            if (bus_mode != 1'b1) begin
                bus_up(1'b1, S_GB_DETECT);
            end else begin
                hidx  <= 7'd0;
                logo  <= 32'd0;
                hsum  <= 8'd0;
                if (!hdr_verify) for (k = 0; k < 32; k = k + 1) hdr[k] <= 8'h00;
                issue(OP_GB_RD, 24'h000104, 8'd0, S_GB_HDR_GOT);
            end
        end
        // hidx 0..3: logo 0x104..0x107, then 4..35: 0x130..0x14F
        S_GB_HDR_RD: issue(OP_GB_RD, (hidx < 7'd4) ? {16'h0001, 6'd1, hidx[1:0]}
                                                   : (24'h000130 + {17'd0, hidx} - 24'd4),
                           8'd0, S_GB_HDR_GOT);
        S_GB_HDR_GOT: begin
            if (hidx < 7'd4) begin
                logo <= {logo[23:0], bus_rdata[7:0]};
            end else begin
                if (hdr_verify) begin
                    if (hdr[hidx - 7'd4] != bus_rdata[7:0]) hdr_mismatch <= 1'b1;
                end else begin
                    hdr[hidx - 7'd4] <= bus_rdata[7:0];
                end
                if (hidx >= 7'd8 && hidx <= 7'd32)          // 0x134..0x14C
                    hsum <= hsum - bus_rdata[7:0] - 8'd1;
            end
            if (hidx == 7'd35) begin
                if (hdr_verify && logo != 32'hCEED6666) hdr_mismatch <= 1'b1;
                state <= hdr_verify ? S_HDRV_DONE : S_GB_HDR_CHECK;
            end else begin
                hidx  <= hidx + 7'd1;
                state <= S_GB_HDR_RD;
            end
        end
        S_GB_HDR_CHECK: begin
            if (!gb_ok) begin
                fail(ST_S_NOCART);
            end else begin
                cart_kind <= hdr[19][7] ? K_GBC : K_GB;
                // mapper (0x147)
                case (hdr[23])
                8'h00, 8'h08, 8'h09:                      mapper <= M_ROM;
                8'h01, 8'h02, 8'h03:                      mapper <= M_MBC1;
                8'h05, 8'h06:                             mapper <= M_MBC2;
                8'h0F, 8'h10, 8'h11, 8'h12, 8'h13:        mapper <= M_MBC3;
                8'h19, 8'h1A, 8'h1B, 8'h1C, 8'h1D, 8'h1E: mapper <= M_MBC5;
                default:                                  mapper <= M_UNSUP;
                endcase
                case (hdr[23])
                8'h02, 8'h03, 8'h05, 8'h06, 8'h08, 8'h09, 8'h10, 8'h12, 8'h13,
                8'h1A, 8'h1B, 8'h1D, 8'h1E: gb_has_ram <= 1'b1;
                default:                    gb_has_ram <= 1'b0;
                endcase
                // ROM size (0x148): 32 KiB << n
                rom_log2 <= (hdr[24] <= 8'h08) ? (5'd15 + hdr[24][4:0]) : 5'd0;
                state    <= S_GB_RAM_DONE;      // decode RAM size next
            end
        end
        // RAM size (0x149) -> label, banks, bytes per bank
        S_GB_RAM_DONE: begin
            gb_bank_bytes <= 14'd8192;
            gb_ram_banks  <= 5'd1;
            save_label    <= SV_NONE;
            save_size     <= 18'd0;
            if (mapper == M_MBC2) begin
                save_label <= SV_MBC2; save_size <= 18'd512; size_log2 <= 5'd9;
                gb_bank_bytes <= 14'd512;
            end else if (gb_has_ram) begin
                case (hdr[25])
                8'h01: begin save_label <= SV_GB2K;   save_size <= 18'd2048;   size_log2 <= 5'd11; gb_bank_bytes <= 14'd2048; end
                8'h02: begin save_label <= SV_GB8K;   save_size <= 18'd8192;   size_log2 <= 5'd13; end
                8'h03: begin save_label <= SV_GB32K;  save_size <= 18'd32768;  size_log2 <= 5'd15; gb_ram_banks <= 5'd4; end
                8'h05: begin save_label <= SV_GB64K;  save_size <= 18'd65536;  size_log2 <= 5'd16; gb_ram_banks <= 5'd8; end
                8'h04: begin save_label <= SV_GB128K; save_size <= 18'd131072; size_log2 <= 5'd17; gb_ram_banks <= 5'd16; end
                default: ;
                endcase
            end
            if (mapper == M_UNSUP || rom_log2 == 5'd0) fail(ST_S_UNSUP);
            else                                         state <= S_MENU;
        end

        // ---------------------------------------------------- GB mapper writes
        S_WQ: begin
            if (wq_i == wq_n) begin
                permit_mbc <= 1'b0;
                state      <= wq_ret;
            end else begin
                permit_mbc <= 1'b1;
                issue(OP_GB_WR, {8'd0, wq_a[wq_i[1:0]]}, wq_d[wq_i[1:0]], S_WQ_NEXT);
            end
        end
        S_WQ_NEXT: begin
            wq_i  <= wq_i + 3'd1;
            state <= S_WQ;
        end

        // ============================================================ menu
        // the cartridge bus is released while the menu is up
        S_MENU: begin
            bus_enable  <= 1'b0;
            permit_bank <= 1'b0;
            permit_ee   <= 1'b0;
            permit_mbc  <= 1'b0;
            ui_mode     <= UI_MENU;
            status      <= ST_S_READY;
            if (up_edge)   menu_sel <= menu_sel - 2'd1;
            if (down_edge) menu_sel <= menu_sel + 2'd1;
            if (a_edge) begin
                case (menu_sel)
                2'd0: begin act_save <= 1'b1; act_rom <= 1'b0; state <= S_ACT_START; end
                2'd1: begin act_save <= 1'b0; act_rom <= 1'b1; state <= S_ACT_START; end
                2'd2: begin act_save <= has_save; act_rom <= 1'b1; state <= S_ACT_START; end
                default: begin ui_mode <= UI_BUSY; state <= S_DETECT; end
                endcase
            end
        end

        S_ACT_START: begin
            done_what <= 2'd0;
            progress  <= 8'd0;
            if (act_save && !has_save) begin
                fail(ST_S_NOSAVE);
            end else if ((act_save && save_needs_cmd) || (act_rom && rom_needs_cmd)) begin
                ui_mode <= UI_CONFIRM;
                status  <= ST_S_CONFIRM;
                state   <= S_CONFIRM;
            end else begin
                ui_mode <= UI_BUSY;
                state   <= act_save ? S_ACT_SAVE : S_ACT_ROM;
            end
        end
        S_CONFIRM: begin
            if (a_edge) begin
                ui_mode <= UI_BUSY;
                state   <= act_save ? S_ACT_SAVE : S_ACT_ROM;
            end else if (b_edge) begin
                state   <= S_MENU;
            end
        end

        // ============================================================ save
        S_ACT_SAVE: begin
            pass    <= 1'b0;
            retries <= 2'd0;
            status  <= ST_S_READ;
            if (!is_gba)
                bus_up(1'b1, S_READ_BEGIN);
            else if (save_label == SV_EEPROM)
                bus_up(1'b0, S_EE_PROBE);       // size first
            else
                bus_up(1'b0, S_READ_BEGIN);
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
            if (!is_gba)
                state <= S_GB_SAVE_BEGIN;
            else if (save_label == SV_EEPROM512 || save_label == SV_EEPROM8K)
                state <= S_EE_DUMP;
            else
                state <= S_SAVE_RD;
        end

        // ---------------------------------------------------- GBA SRAM / Flash
        S_SAVE_RD: issue(OP_SAVE_RD, {8'd0, saddr[15:0]}, 8'd0, S_SAVE_GOT);
        S_SAVE_GOT: begin
            push_byte(bus_rdata[7:0]);
            progress <= {byte_idx, 8'd0} >> size_log2;
            saddr <= saddr + 17'd1;
            if ({1'b0, saddr} + 18'd1 == save_size) begin
                if (save_label == SV_FLASH128) begin
                    bank_seq <= 2'd0; bank_target <= 1'b0; bank_ret <= S_PASS_END;
                    permit_bank <= 1'b1;
                    state <= S_BANK;
                end else begin
                    state <= S_PASS_END;
                end
            end else if (save_label == SV_FLASH128 && saddr == 17'h0FFFF) begin
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

        // ---------------------------------------------------- GBA EEPROM
        S_EE_BLK: begin
            if (ee_flush)               ee_req <= 17'd0;
            else if (ee_bits == 4'd14)  ee_req <= {2'b11, 4'b0000, ee_blk[9:0], 1'b0};
            else                        ee_req <= {2'b11, ee_blk[5:0], 1'b0, 8'd0};
            ee_reqn   <= 5'd0;
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
            end else begin
                state <= S_EE_REQ;
            end
        end
        S_EE_RLATCH: begin
            ee_rcnt <= 7'd0;
            issue(OP_ROM_LATCH, EE_ADDR, 8'd0, S_EE_RD);
        end
        S_EE_RD: issue(OP_EE_RD, 24'd0, 8'd0, S_EE_RGOT);
        S_EE_RGOT: begin
            if (ee_rcnt >= 7'd4) ee_data <= {ee_data[62:0], bus_rdata[0]};
            ee_rcnt <= ee_rcnt + 7'd1;
            if (ee_rcnt == 7'd67) issue(OP_ROM_END, 24'd0, 8'd0, ee_ret);
            else                  state <= S_EE_RD;
        end
        // 512 B vs 8 KiB: 64 reads with 14 address bits; a 512 B chip decodes
        // only the first six (all zero here) and answers block 0 every time
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
                    save_label <= SV_EEPROM8K; save_size <= 18'h02000; size_log2 <= 5'd13;
                    ee_bits <= 4'd14; ee_nblk <= 11'd1024;
                    state   <= S_READ_BEGIN;
                end else begin
                    save_label <= SV_EEPROM512; save_size <= 18'h00200; size_log2 <= 5'd9;
                    ee_bits  <= 4'd6; ee_nblk <= 11'd64;
                    ee_flush <= 1'b1;
                    ee_ret   <= S_EE_FLUSH_DONE;
                    state    <= S_EE_BLK;
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
            ee_blk <= 10'd0;
            ee_ret <= S_EE_PUSH;
            ee_k   <= 3'd0;
            state  <= S_EE_BLK;
        end
        S_EE_PUSH: begin
            push_byte(ee_data[63:56]);
            progress <= {byte_idx, 8'd0} >> size_log2;
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

        // ---------------------------------------------------- GB cartridge RAM
        S_GB_SAVE_BEGIN: begin
            gb_bank <= 5'd0;
            gb_off  <= 14'd0;
            // enable RAM (and RAM banking mode on multi-bank MBC1)
            if (mapper == M_ROM)
                state <= S_GB_RAM_BANK;
            else if (mapper == M_MBC1 && gb_ram_banks > 5'd1)
                wq2(16'h0000, 8'h0A, 16'h6000, 8'h01, S_GB_RAM_BANK);
            else
                wq1(16'h0000, 8'h0A, S_GB_RAM_BANK);
        end
        S_GB_RAM_BANK: begin
            gb_off <= 14'd0;
            if (gb_ram_banks > 5'd1)
                wq1(16'h4000, {3'd0, gb_bank}, S_GB_RAM_RD);
            else
                state <= S_GB_RAM_RD;
        end
        S_GB_RAM_RD: issue(OP_GB_RD, {8'd0, 16'hA000 + {2'b00, gb_off}}, 8'd0, S_GB_RAM_GOT);
        S_GB_RAM_GOT: begin
            // MBC2 RAM is 4 bits wide; the upper nibble floats
            push_byte(mapper == M_MBC2 ? {4'hF, bus_rdata[3:0]} : bus_rdata[7:0]);
            progress <= {byte_idx, 8'd0} >> size_log2;
            if (gb_off + 14'd1 == gb_bank_bytes) begin
                if (gb_bank + 5'd1 == gb_ram_banks) begin
                    state <= S_GB_RAM_END;
                end else begin
                    gb_bank <= gb_bank + 5'd1;
                    state   <= S_GB_RAM_BANK;
                end
            end else begin
                gb_off <= gb_off + 14'd1;
                state  <= S_GB_RAM_RD;
            end
        end
        // disable RAM again, back to bank 0 / ROM banking mode
        S_GB_RAM_END: begin
            if (mapper == M_ROM)
                state <= S_PASS_END;
            else if (mapper == M_MBC1 && gb_ram_banks > 5'd1)
                wq3(16'h0000, 8'h00, 16'h4000, 8'h00, 16'h6000, 8'h00, S_PASS_END);
            else if (gb_ram_banks > 5'd1)
                wq2(16'h0000, 8'h00, 16'h4000, 8'h00, S_PASS_END);
            else
                wq1(16'h0000, 8'h00, S_PASS_END);
        end

        S_PASS_END: begin
            if (!pass) begin
                crc1  <= crc;
                pass  <= 1'b1;
                state <= S_READ_BEGIN;
            end else if (crc == crc1) begin
                hdrv_ret <= S_SAVE_FILE;        // header check, then the file
                state    <= S_HDRV_START;
            end else if (retries != 2'd2) begin
                retries <= retries + 2'd1;
                pass    <= 1'b0;
                state   <= S_READ_BEGIN;
            end else begin
                fail(ST_S_UNSTABLE);
            end
        end

        S_SAVE_FILE: begin
            pass <= 1'b0;
            if (save_label == SV_EEPROM512 && !(seen_non_ff && seen_non_00))
                fail(ST_S_BLANK);               // indistinguishable from no chip
            else
                file_open(2'd0, {14'd0, save_size}, S_CHUNK);
            chunk_bytes    <= save_size;
            tgt_slotoffset <= 32'd0;
            chunk_ret      <= S_ACT_ROM;        // after the chunk is written
        end

        // ============================================================ ROM
        S_ACT_ROM: begin
            if (act_save) done_what <= 2'd1;
            if (!act_rom) begin
                state <= S_ACT_DONE;
            end else begin
                status  <= ST_S_ROMREAD;
                chunk   <= 10'd0;
                // GBA: 64 KiB chunks; GB: one 16 KiB bank per chunk
                chunk_bytes <= is_gba ? 18'h10000 : 18'h04000;
                nchunks     <= is_gba ? (10'd1 << (rom_log2 - 5'd16)) : (10'd1 << (rom_log2 - 5'd14));
                file_open(is_gba ? 2'd1 : (cart_kind == K_GBC ? 2'd3 : 2'd2),
                          32'd1 << rom_log2, S_ROM_OPENED);
            end
        end
        S_ROM_OPENED: begin
            status <= ST_S_ROMREAD;
            bus_up(is_gba ? 1'b0 : 1'b1, S_ROM_CHUNK);
        end
        S_ROM_CHUNK: begin
            pass    <= 1'b0;
            retries <= 2'd0;
            crc     <= 32'hFFFF_FFFF;
            byte_idx<= 17'd0;
            chunk_pos <= 18'd0;
            if (is_gba)
                issue(OP_ROM_LATCH, {chunk[8:0], 15'd0}, 8'd0, S_ROM_CHUNK_RD);
            else
                state <= S_GB_ROM_SEL;
        end
        // GBA: sequential halfwords inside one 64 KiB chunk
        S_ROM_CHUNK_RD: issue(OP_ROM_RD, 24'd0, 8'd0, S_ROM_CHUNK_GOT);
        S_ROM_CHUNK_GOT: begin
            rom_half <= bus_rdata;
            push_byte(bus_rdata[7:0]);
            state    <= S_ROM_CHUNK_B1;
        end
        S_ROM_CHUNK_B1: begin
            push_byte(rom_half[15:8]);
            chunk_pos <= chunk_pos + 18'd2;
            if (chunk_pos + 18'd2 == chunk_bytes)
                issue(OP_ROM_END, 24'd0, 8'd0, S_ROM_PASS_END);
            else
                state <= S_ROM_CHUNK_RD;
        end
        // GB: select the bank, then read 16 KiB
        S_GB_ROM_SEL: begin
            rom_bank <= chunk[8:0];
            gb_base  <= 16'h4000;
            if (chunk == 10'd0) begin
                gb_base <= 16'h0000;
                state   <= S_GB_ROM_RD;
            end else case (mapper)
                M_MBC1: if (chunk[4:0] == 5'd0) begin
                            // banks 20/40/60 only appear at 0000 in mode 1
                            gb_base <= 16'h0000;
                            wq2(16'h6000, 8'h01, 16'h4000, {6'd0, chunk[6:5]}, S_GB_ROM_RD);
                        end else begin
                            wq3(16'h6000, 8'h00, 16'h4000, {6'd0, chunk[6:5]},
                                16'h2000, {3'd0, chunk[4:0]}, S_GB_ROM_RD);
                        end
                M_MBC2: wq1(16'h2100, {4'd0, chunk[3:0]}, S_GB_ROM_RD);
                M_MBC3: wq1(16'h2000, {1'b0, chunk[6:0]}, S_GB_ROM_RD);
                M_MBC5: wq2(16'h2000, chunk[7:0], 16'h3000, {7'd0, chunk[8]}, S_GB_ROM_RD);
                default: state <= S_GB_ROM_RD;          // plain 32 KiB ROM
            endcase
        end
        S_GB_ROM_RD: issue(OP_GB_RD, {8'd0, gb_base + chunk_pos[15:0]}, 8'd0, S_GB_ROM_GOT);
        S_GB_ROM_GOT: begin
            push_byte(bus_rdata[7:0]);
            chunk_pos <= chunk_pos + 18'd1;
            if (chunk_pos + 18'd1 == chunk_bytes) state <= S_ROM_PASS_END;
            else                                  state <= S_GB_ROM_RD;
        end
        // two passes per chunk; CRCs must match; then write it
        S_ROM_PASS_END: begin
            if (!pass) begin
                crc1      <= crc;
                pass      <= 1'b1;
                crc       <= 32'hFFFF_FFFF;
                byte_idx  <= 17'd0;
                chunk_pos <= 18'd0;
                if (is_gba) issue(OP_ROM_LATCH, {chunk[8:0], 15'd0}, 8'd0, S_ROM_CHUNK_RD);
                else        state <= S_GB_ROM_RD;      // bank still selected
            end else if (crc == crc1) begin
                pass           <= 1'b0;
                tgt_slotoffset <= is_gba ? {6'd0, chunk, 16'd0} : {8'd0, chunk, 14'd0};
                chunk_ret      <= S_ROM_NEXT;
                state          <= S_CHUNK;
            end else if (retries != 2'd2) begin
                retries   <= retries + 2'd1;
                pass      <= 1'b0;
                crc       <= 32'hFFFF_FFFF;
                byte_idx  <= 17'd0;
                chunk_pos <= 18'd0;
                if (is_gba) issue(OP_ROM_LATCH, {chunk[8:0], 15'd0}, 8'd0, S_ROM_CHUNK_RD);
                else        state <= S_GB_ROM_SEL;
            end else begin
                fail(ST_S_UNSTABLE);
            end
        end
        S_ROM_NEXT: begin
            progress <= ({chunk, 8'd0} + 18'd256) >> (is_gba ? (rom_log2 - 5'd16) : (rom_log2 - 5'd14));
            status   <= ST_S_ROMREAD;
            if (chunk + 10'd1 == nchunks) begin
                state <= S_ROM_FINISH;
            end else begin
                chunk <= chunk + 10'd1;
                bus_up(is_gba ? 1'b0 : 1'b1, S_ROM_CHUNK);
            end
        end
        S_ROM_FINISH: begin
            // put the mapper back where a Game Boy would find it
            hdrv_ret <= S_ACT_DONE;
            if (is_gba) begin
                state    <= S_HDRV_START;
            end else begin
                case (mapper)
                M_MBC1: wq3(16'h6000, 8'h00, 16'h4000, 8'h00, 16'h2000, 8'h01, S_HDRV_START);
                M_MBC2: wq1(16'h2100, 8'h01, S_HDRV_START);
                M_MBC3: wq1(16'h2000, 8'h01, S_HDRV_START);
                M_MBC5: wq2(16'h2000, 8'h01, 16'h3000, 8'h00, S_HDRV_START);
                default: state <= S_HDRV_START;
                endcase
            end
        end

        S_ACT_DONE: begin
            done_what <= {act_rom, act_save};
            progress  <= 8'd255;
            status    <= ST_S_DONE;
            ui_mode   <= UI_RESULT;
            bus_enable<= 1'b0;
            state     <= S_HALT;
        end

        // ============================================================ header re-check
        S_HDRV_START: begin
            hdr_verify   <= 1'b1;
            hdr_mismatch <= 1'b0;
            hidx         <= 7'd0;
            if (is_gba) issue(OP_ROM_LATCH, 24'd0, 8'd0, S_HDR_RD);
            else        state <= S_GB_DETECT;
        end
        S_HDRV_DONE: begin
            hdr_verify <= 1'b0;
            if (hdr_mismatch) fail(ST_S_CHANGED);
            else              state <= hdrv_ret;
        end

        // ============================================================ files
        S_FILE_OPEN: begin
            status   <= ST_S_WRITE;
            sfx_tens <= 4'd0;
            sfx_ones <= 4'd0;
            pidx     <= 6'd0;
            bstep    <= 6'd0;
            for (k = 0; k < 64; k = k + 1) path[k] <= 8'h00;
            state    <= S_BUILD;
        end
        S_BUILD: begin
            if (b_emit) begin
                path[pidx] <= b_char;
                pidx <= pidx + 6'd1;
            end
            if (b_last) state <= S_OPEN_PROBE;
            else        bstep <= bstep + 6'd1;
        end
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
                le_mode     <= ~le_mode;        // APF reads the other byte order
                le_tried_be <= 1'b1;
                state       <= S_OPEN_PROBE;
            end else if (tgt_err == 3'd0) begin
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
            open_size    <= file_size;
            tgt_openfile <= 1'b1;
            tgt_ret      <= S_OPEN_CREATE_R;
            tgt_seen     <= 1'b0;
            timer        <= T_CMD_TIMEOUT;
            state        <= S_TGT_WAIT;
        end
        S_OPEN_CREATE_R: begin
            if (tgt_err == 3'd0 || tgt_err == 3'd1) begin
                state <= open_ret;
            end else begin
                err_code <= tgt_err;
                fail(ST_S_FILEERR);
            end
        end
        // write chunk_bytes from the buffer at tgt_slotoffset, then file_ret
        S_CHUNK: begin
            tgt_length <= {14'd0, chunk_bytes};
            tgt_write  <= 1'b1;
            tgt_ret    <= S_CHUNK_R;
            tgt_seen   <= 1'b0;
            timer      <= T_CMD_TIMEOUT;
            state      <= S_TGT_WAIT;
        end
        S_CHUNK_R: begin
            if (tgt_err == 3'd0) begin
                state <= chunk_ret;
            end else begin
                err_code <= tgt_err;
                fail(ST_S_FILEERR);
            end
        end
        S_TGT_WAIT: begin
            if (tgt_ack || !tgt_done) tgt_seen <= 1'b1;
            if (tgt_seen && tgt_done) state <= tgt_ret;
            else if (timer == 28'd0)  fail(ST_S_TIMEOUT);
        end

        // ============================================================ result
        S_HALT: begin
            bus_enable  <= 1'b0;
            permit_bank <= 1'b0;
            permit_ee   <= 1'b0;
            permit_mbc  <= 1'b0;
            if (!fatal && (a_edge || b_edge)) begin
                if (cart_kind == K_NONE || status == ST_S_UNSUP) begin
                    ui_mode <= UI_BUSY;
                    state   <= S_DETECT;            // try reading the cart again
                end else begin
                    state   <= S_MENU;
                end
            end
        end

        default: state <= S_RESET;
        endcase
    end
end

endmodule

`default_nettype wire
