// renders one frame of status_screen to screen.ppm; pick the scene with -DSCENE=n
`timescale 1ns/1ps
`ifndef SCENE
`define SCENE 0
`endif
module tb_screen;
reg clk = 0; always #40 clk = ~clk;
wire [23:0] rgb; wire de, hs, vs;
reg [511:0] path;
reg [2:0] ui; reg [4:0] st; reg [1:0] kind; reg [3:0] sv; reg [4:0] rl; reg [7:0] prog;
reg [127:0] nm; reg [4:0] nl; reg [1:0] sel; reg [1:0] dw;
initial begin
    path = {"/Assets/savesiphon/common/POKEMON_EMER_BPEE.sav", {17{8'h00}}};
    nm = {"POKEMON EMER", 32'h0}; nl = 12; kind = 1; sv = 3; rl = 24; sel = 0; prog = 0; dw = 0;
    case (`SCENE)
    0: begin ui = 1; st = 21; end                               // GBA menu
    1: begin ui = 3; st = 13; end                               // confirm
    2: begin ui = 0; st = 19; prog = 150; end                   // reading ROM
    3: begin ui = 2; st = 8; dw = 1; prog = 255; end            // done
    4: begin ui = 1; st = 21; kind = 3; nm = {"POKEMON_SLV", 40'h0}; nl = 11; sv = 9; sel = 1; end   // GBC menu
    5: begin ui = 2; st = 12; end                               // safety stop
    endcase
end
status_screen dut (.clk(clk), .ui_mode(ui), .menu_sel(sel), .status(st), .progress(prog),
    .err_code(3'd0), .fault_code(3'd2), .cart_kind(kind), .name_flat(nm), .name_len(nl),
    .code_flat("BPEE"), .code_len(kind == 1), .save_label(sv), .rom_log2(rl), .mapper(3'd3),
    .done_what(dw), .path_flat(path), .rgb(rgb), .de(de), .hs(hs), .vs(vs));
integer f, n;
initial begin
    f = $fopen("screen.ppm", "w");
    $fwrite(f, "P3\n320 240\n255\n");
    n = 0;
    @(posedge vs); @(posedge clk);
    while (n < 320*240) begin
        @(posedge clk);
        if (de) begin $fwrite(f, "%0d %0d %0d\n", rgb[23:16], rgb[15:8], rgb[7:0]); n = n + 1; end
    end
    $fclose(f); $finish;
end
endmodule
