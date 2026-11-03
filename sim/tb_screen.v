// renders one frame of status_screen to a PPM (sim/screen.ppm)
`timescale 1ns/1ps
module tb_screen;
reg clk = 0; always #40 clk = ~clk;
wire [23:0] rgb; wire de, hs, vs;
reg [511:0] path;
initial path = {"/Assets/savesiphon/common/POKEMON_EMER-BPEE_1.sav", {15{8'h00}}};
status_screen dut (.clk(clk), .status(4'd8), .save_type(3'd3), .progress(8'd255),
    .err_code(3'd0), .title_flat("POKEMON EMER"), .code_flat("BPEE"), .path_flat(path),
    .rgb(rgb), .de(de), .hs(hs), .vs(vs));
integer f, n, vsn;
initial begin
    f = $fopen("screen.ppm", "w");
    $fwrite(f, "P3\n320 240\n255\n");
    n = 0; vsn = 0;
    @(posedge vs); @(posedge clk);
    while (n < 320*240) begin
        @(posedge clk);
        if (de) begin $fwrite(f, "%0d %0d %0d\n", rgb[23:16], rgb[15:8], rgb[7:0]); n = n + 1; end
    end
    $fclose(f); $finish;
end
endmodule
