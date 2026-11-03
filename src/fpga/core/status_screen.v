//
// status_screen.v - 320x240 text status display for SaveSiphon
//
// 40x30 character cells of 8x8 pixels. Inputs come from the clk_74a domain;
// they change a handful of times per second, so a torn frame is the worst a
// missed synchroniser can do (the clocks are declared asynchronous in the SDC).
//
// Pipeline: counters -> cell -> character -> font row -> pixel. Sync and DE
// are delayed to match.
//

`default_nettype none

module status_screen (
    input  wire         clk,            // ~12.288 MHz pixel clock

    input  wire [3:0]   status,
    input  wire [2:0]   save_type,
    input  wire [7:0]   progress,
    input  wire [2:0]   err_code,
    input  wire [95:0]  title_flat,
    input  wire [31:0]  code_flat,
    input  wire [511:0] path_flat,

    output reg  [23:0]  rgb,
    output reg          de,
    output reg          hs,
    output reg          vs
);

localparam [9:0] H_BP = 10'd10, H_ACT = 10'd320, H_TOT = 10'd400;
localparam [9:0] V_BP = 10'd10, V_ACT = 10'd240, V_TOT = 10'd512;

localparam [23:0] C_BG     = 24'h0E1420;
localparam [23:0] C_FG     = 24'hE6E6E6;
localparam [23:0] C_DIM    = 24'h7A8696;
localparam [23:0] C_ACCENT = 24'h3FC1FF;
localparam [23:0] C_OK     = 24'h4BE07A;
localparam [23:0] C_BAD    = 24'hFF5A5A;
localparam [23:0] C_BUSY   = 24'hFFD24A;
localparam [23:0] C_BAR_BG = 24'h28323F;

// ------------------------------------------------------------ font ROM
reg [7:0] font [0:1023];
initial $readmemh("core/font.hex", font);

// ------------------------------------------------------------ timing
reg [9:0] x = 10'd0, y = 10'd0;
always @(posedge clk) begin
    if (x == H_TOT - 10'd1) begin
        x <= 10'd0;
        y <= (y == V_TOT - 10'd1) ? 10'd0 : y + 10'd1;
    end else begin
        x <= x + 10'd1;
    end
end

wire       active = (x >= H_BP) && (x < H_BP + H_ACT) && (y >= V_BP) && (y < V_BP + V_ACT);
wire [9:0] vx = x - H_BP;
wire [9:0] vy = y - V_BP;

// stage A registers (cell coordinates)
reg [5:0] cell_col;
reg [4:0] cell_row;
reg [2:0] fy_a, fx_a;
reg       act_a;

// ------------------------------------------------------------ text helpers
function [7:0] printable;
    input [7:0] c;
    printable = (c >= 8'h20 && c < 8'h7F) ? c : 8'h20;
endfunction

reg [8*24-1:0] status_str;
always @(*) begin
    case (status)
    4'd0:  status_str = "WAITING FOR CART POWER  ";
    4'd1:  status_str = "READING HEADER...       ";
    4'd2:  status_str = "NO GBA CARTRIDGE FOUND  ";
    4'd3:  status_str = "SCANNING ROM...         ";
    4'd4:  status_str = "NO SAVE TYPE DETECTED   ";
    4'd5:  status_str = "READING SAVE...         ";
    4'd6:  status_str = "EEPROM BLANK / NO REPLY ";
    4'd7:  status_str = "WRITING FILE...         ";
    4'd8:  status_str = "DONE - SAVE DUMPED      ";
    4'd9:  status_str = "SD CARD FILE ERROR      ";
    4'd10: status_str = "TOO MANY DUMPS (MAX 99) ";
    4'd11: status_str = "SD CARD TIMEOUT         ";
    default: status_str = "                        ";
    endcase
end

reg [8*12-1:0] type_str;
always @(*) begin
    case (save_type)
    3'd1: type_str = "SRAM 32K    ";
    3'd2: type_str = "FLASH 64K   ";
    3'd3: type_str = "FLASH 128K  ";
    3'd4: type_str = "EEPROM 512B ";
    3'd5: type_str = "EEPROM 8K   ";
    3'd6: type_str = "EEPROM ?    ";
    default: type_str = "-           ";
    endcase
end

wire st_ok   = (status == 4'd8);
wire st_bad  = (status == 4'd2) || (status == 4'd4) || (status == 4'd6) ||
               (status >= 4'd9);

reg [8*12-1:0] title_p;
reg [8*4-1:0]  code_p;
reg [8*36-1:0] name_p;
integer i;
always @(*) begin
    for (i = 0; i < 12; i = i + 1) title_p[95-8*i -: 8] = printable(title_flat[95-8*i -: 8]);
    for (i = 0; i < 4;  i = i + 1) code_p[31-8*i -: 8]  = printable(code_flat[31-8*i -: 8]);
    // file name starts after "/Assets/savesiphon/common/" (26 bytes)
    for (i = 0; i < 36; i = i + 1) name_p[287-8*i -: 8] = printable(path_flat[511-8*(26+i) -: 8]);
end

// one 40-character line per row
reg [8*40-1:0] line;
reg [23:0]     line_color;
reg            line_is_bar;
always @(*) begin
    line        = {40{8'h20}};
    line_color  = C_FG;
    line_is_bar = 1'b0;
    case (cell_row)
    5'd2:  begin line = {"  SAVESIPHON", {28{8'h20}}};                 line_color = C_ACCENT; end
    5'd3:  begin line = {"  GBA cartridge save dumper", {13{8'h20}}};  line_color = C_DIM; end
    5'd6:  line = {"  GAME  ", title_p, {20{8'h20}}};
    5'd7:  line = {"  CODE  ", code_p,  {28{8'h20}}};
    5'd8:  line = {"  SAVE  ", type_str, {20{8'h20}}};
    5'd11: begin line = {"  ", status_str, {14{8'h20}}};
                 line_color = st_ok ? C_OK : st_bad ? C_BAD : C_BUSY; end
    5'd13: line_is_bar = 1'b1;
    5'd16: begin line = {"  FILE", {34{8'h20}}};                       line_color = C_DIM; end
    5'd17: line = {"  ", name_p, {2{8'h20}}};
    5'd19: if (status == 4'd9) begin
                 line = {"  APF ERROR CODE ", "0" + {5'd0, err_code}, {22{8'h20}}};
                 line_color = C_BAD; end
    5'd25: begin line = {"  A: run again", {26{8'h20}}};               line_color = C_DIM; end
    5'd26: begin line = {"  Power off before swapping carts", {7{8'h20}}}; line_color = C_DIM; end
    5'd27: begin line = {"  Dumps: /Assets/savesiphon/common/", {5{8'h20}}}; line_color = C_DIM; end
    default: ;
    endcase
end

// ------------------------------------------------------------ pipeline
// stage A: cell coordinates (declared above)
always @(posedge clk) begin
    cell_col <= vx[8:3];
    cell_row <= vy[7:3];
    fy_a     <= vy[2:0];
    fx_a     <= vx[2:0];
    act_a    <= active;
end

// stage B: character and colour
reg [7:0]  ch_b;
reg [23:0] col_b;
reg [2:0]  fy_b, fx_b;
reg        act_b, bar_b, bar_fill_b;
wire [5:0] bar_cell = cell_col - 6'd3;
always @(posedge clk) begin
    ch_b       <= (cell_col < 6'd40) ? line[8*(39-cell_col) +: 8] : 8'h20;
    col_b      <= line_color;
    bar_b      <= line_is_bar && (cell_col >= 6'd3) && (cell_col < 6'd35);
    bar_fill_b <= (bar_cell[4:0] < progress[7:3]) || (progress == 8'd255);
    fy_b       <= fy_a;
    fx_b       <= fx_a;
    act_b      <= act_a;
end

// stage C: font row
reg [7:0]  glyph_c;
reg [23:0] col_c;
reg [2:0]  fy_c, fx_c;
reg        act_c, bar_c, bar_fill_c;
always @(posedge clk) begin
    glyph_c    <= font[{ch_b[6:0], fy_b}];
    col_c      <= col_b;
    fy_c       <= fy_b;
    fx_c       <= fx_b;
    act_c      <= act_b;
    bar_c      <= bar_b;
    bar_fill_c <= bar_fill_b;
end

// stage D: pixel
reg [3:0] de_d, hs_d, vs_d;
always @(posedge clk) begin
    if (!act_c)
        rgb <= 24'h000000;
    else if (bar_c)
        rgb <= (fy_c == 3'd0 || fy_c == 3'd7 || fx_c == 3'd7) ? C_BG :
               bar_fill_c ? C_ACCENT : C_BAR_BG;
    else
        rgb <= glyph_c[3'd7 - fx_c] ? col_c : C_BG;

    // rgb lands four edges after the counters; so do these
    de_d <= {de_d[2:0], active};
    hs_d <= {hs_d[2:0], (x == 10'd3)};
    vs_d <= {vs_d[2:0], (x == 10'd0 && y == 10'd0)};
    de   <= de_d[2];
    hs   <= hs_d[2];
    vs   <= vs_d[2];
end

endmodule

`default_nettype wire
