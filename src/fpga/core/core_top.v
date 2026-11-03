//
// User core top-level
//
// Instantiated by the real top-level: apf_top
//

`default_nettype none

module core_top (

//
// physical connections
//

///////////////////////////////////////////////////
// clock inputs 74.25mhz. not phase aligned, so treat these domains as asynchronous

input   wire            clk_74a, // mainclk1
input   wire            clk_74b, // mainclk1 

///////////////////////////////////////////////////
// cartridge interface
// switches between 3.3v and 5v mechanically
// output enable for multibit translators controlled by pic32

// GBA AD[15:8]
inout   wire    [7:0]   cart_tran_bank2,
output  wire            cart_tran_bank2_dir,

// GBA AD[7:0]
inout   wire    [7:0]   cart_tran_bank3,
output  wire            cart_tran_bank3_dir,

// GBA A[23:16]
inout   wire    [7:0]   cart_tran_bank1,
output  wire            cart_tran_bank1_dir,

// GBA [7] PHI#
// GBA [6] WR#
// GBA [5] RD#
// GBA [4] CS1#/CS#
//     [3:0] unwired
inout   wire    [7:4]   cart_tran_bank0,
output  wire            cart_tran_bank0_dir,

// GBA CS2#/RES#
inout   wire            cart_tran_pin30,
output  wire            cart_tran_pin30_dir,
// when GBC cart is inserted, this signal when low or weak will pull GBC /RES low with a special circuit
// the goal is that when unconfigured, the FPGA weak pullups won't interfere.
// thus, if GBC cart is inserted, FPGA must drive this high in order to let the level translators
// and general IO drive this pin.
output  wire            cart_pin30_pwroff_reset,

// GBA IRQ/DRQ
inout   wire            cart_tran_pin31,
output  wire            cart_tran_pin31_dir,

// infrared
input   wire            port_ir_rx,
output  wire            port_ir_tx,
output  wire            port_ir_rx_disable, 

// GBA link port
inout   wire            port_tran_si,
output  wire            port_tran_si_dir,
inout   wire            port_tran_so,
output  wire            port_tran_so_dir,
inout   wire            port_tran_sck,
output  wire            port_tran_sck_dir,
inout   wire            port_tran_sd,
output  wire            port_tran_sd_dir,
 
///////////////////////////////////////////////////
// cellular psram 0 and 1, two chips (64mbit x2 dual die per chip)

output  wire    [21:16] cram0_a,
inout   wire    [15:0]  cram0_dq,
input   wire            cram0_wait,
output  wire            cram0_clk,
output  wire            cram0_adv_n,
output  wire            cram0_cre,
output  wire            cram0_ce0_n,
output  wire            cram0_ce1_n,
output  wire            cram0_oe_n,
output  wire            cram0_we_n,
output  wire            cram0_ub_n,
output  wire            cram0_lb_n,

output  wire    [21:16] cram1_a,
inout   wire    [15:0]  cram1_dq,
input   wire            cram1_wait,
output  wire            cram1_clk,
output  wire            cram1_adv_n,
output  wire            cram1_cre,
output  wire            cram1_ce0_n,
output  wire            cram1_ce1_n,
output  wire            cram1_oe_n,
output  wire            cram1_we_n,
output  wire            cram1_ub_n,
output  wire            cram1_lb_n,

///////////////////////////////////////////////////
// sdram, 512mbit 16bit

output  wire    [12:0]  dram_a,
output  wire    [1:0]   dram_ba,
inout   wire    [15:0]  dram_dq,
output  wire    [1:0]   dram_dqm,
output  wire            dram_clk,
output  wire            dram_cke,
output  wire            dram_ras_n,
output  wire            dram_cas_n,
output  wire            dram_we_n,

///////////////////////////////////////////////////
// sram, 1mbit 16bit

output  wire    [16:0]  sram_a,
inout   wire    [15:0]  sram_dq,
output  wire            sram_oe_n,
output  wire            sram_we_n,
output  wire            sram_ub_n,
output  wire            sram_lb_n,

///////////////////////////////////////////////////
// vblank driven by dock for sync in a certain mode

input   wire            vblank,

///////////////////////////////////////////////////
// i/o to 6515D breakout usb uart

output  wire            dbg_tx,
input   wire            dbg_rx,

///////////////////////////////////////////////////
// i/o pads near jtag connector user can solder to

output  wire            user1,
input   wire            user2,

///////////////////////////////////////////////////
// RFU internal i2c bus 

inout   wire            aux_sda,
output  wire            aux_scl,

///////////////////////////////////////////////////
// RFU, do not use
output  wire            vpll_feed,


//
// logical connections
//

///////////////////////////////////////////////////
// video, audio output to scaler
output  wire    [23:0]  video_rgb,
output  wire            video_rgb_clock,
output  wire            video_rgb_clock_90,
output  wire            video_de,
output  wire            video_skip,
output  wire            video_vs,
output  wire            video_hs,
    
output  wire            audio_mclk,
input   wire            audio_adc,
output  wire            audio_dac,
output  wire            audio_lrck,

///////////////////////////////////////////////////
// bridge bus connection
// synchronous to clk_74a
output  wire            bridge_endian_little,
input   wire    [31:0]  bridge_addr,
input   wire            bridge_rd,
output  reg     [31:0]  bridge_rd_data,
input   wire            bridge_wr,
input   wire    [31:0]  bridge_wr_data,

///////////////////////////////////////////////////
// controller data
// 
// key bitmap:
//   [0]    dpad_up
//   [1]    dpad_down
//   [2]    dpad_left
//   [3]    dpad_right
//   [4]    face_a
//   [5]    face_b
//   [6]    face_x
//   [7]    face_y
//   [8]    trig_l1
//   [9]    trig_r1
//   [10]   trig_l2
//   [11]   trig_r2
//   [12]   trig_l3
//   [13]   trig_r3
//   [14]   face_select
//   [15]   face_start
//   [31:28] type
// joy values - unsigned
//   [ 7: 0] lstick_x
//   [15: 8] lstick_y
//   [23:16] rstick_x
//   [31:24] rstick_y
// trigger values - unsigned
//   [ 7: 0] ltrig
//   [15: 8] rtrig
//
input   wire    [31:0]  cont1_key,
input   wire    [31:0]  cont2_key,
input   wire    [31:0]  cont3_key,
input   wire    [31:0]  cont4_key,
input   wire    [31:0]  cont1_joy,
input   wire    [31:0]  cont2_joy,
input   wire    [31:0]  cont3_joy,
input   wire    [31:0]  cont4_joy,
input   wire    [15:0]  cont1_trig,
input   wire    [15:0]  cont2_trig,
input   wire    [15:0]  cont3_trig,
input   wire    [15:0]  cont4_trig
    
);

// SaveSiphon nets used by the pin assignments and bridge mux below
    wire            cart_power;

// cart bus engine
    wire            bus_enable;
    wire            bus_start;
    wire    [2:0]   bus_op;
    wire    [23:0]  bus_addr;
    wire    [7:0]   bus_wdata;
    wire    [15:0]  bus_rdata;
    wire            bus_done;
    wire            bus_busy;
    wire    [15:0]  bus_ad_out;
    wire            bus_ad_oe;
    wire    [7:0]   bus_hi_out;
    wire            bus_hi_oe;
    wire            bus_cs1_n, bus_cs2_n, bus_rd_n, bus_wr_n, bus_phi;
    wire            bus_mode;           // 0 GBA, 1 GB/GBC
    wire            guard_wr_n;         // WR# after the write guard
    wire            guard_fault;
    wire    [2:0]   guard_fault_code;
    wire            permit_bank, permit_ee, permit_mbc;
    reg     [31:0]  buf_q;
    wire    [31:0]  struct_q;
    wire            le_mode;

// not using the IR port, so turn off both the LED, and
// disable the receive circuit to save power
assign port_ir_tx = 0;
assign port_ir_rx_disable = 1;

// bridge endianness
assign bridge_endian_little = 0;

// cartridge: driven by gba_cart_bus or gb_cart_bus (bus_mode), see below
// directions are 0:IN, 1:OUT; direction is per bank
assign cart_tran_bank3     = bus_ad_oe ? bus_ad_out[7:0]  : 8'hzz;   // AD[7:0]
assign cart_tran_bank3_dir = bus_ad_oe;
assign cart_tran_bank2     = bus_ad_oe ? bus_ad_out[15:8] : 8'hzz;   // AD[15:8]
assign cart_tran_bank2_dir = bus_ad_oe;
assign cart_tran_bank1     = bus_hi_oe ? bus_hi_out : 8'hzz;         // A[23:16] / D[7:0]
assign cart_tran_bank1_dir = bus_hi_oe;
// strobes are always driven so nothing floats into a write
// {PHI, WR#, RD#, CS1#}   (GB: {CLK, /WR, /RD, /CS})
// WR# only ever comes from cart_write_guard, never straight from an engine
assign cart_tran_bank0     = {bus_phi, guard_wr_n, bus_rd_n, bus_cs1_n};
assign cart_tran_bank0_dir = 1'b1;
// pin 30: GBA CS2# (save chip select) / GB /RESET (held high).
// Before an engine owns the slot, leave it to the hardware.
assign cart_tran_pin30     = bus_enable ? (bus_mode ? 1'b1 : bus_cs2_n) : 1'b0;
assign cart_tran_pin30_dir = bus_enable ? 1'b1 : 1'bz;
assign cart_pin30_pwroff_reset = bus_enable;
assign cart_tran_pin31     = 1'bz;      // IRQ, input only
assign cart_tran_pin31_dir = 1'b0;

// link port is unused, set to input only to be safe
// each bit may be bidirectional in some applications
assign port_tran_so = 1'bz;
assign port_tran_so_dir = 1'b0;     // SO is output only
assign port_tran_si = 1'bz;
assign port_tran_si_dir = 1'b0;     // SI is input only
assign port_tran_sck = 1'bz;
assign port_tran_sck_dir = 1'b0;    // clock direction can change
assign port_tran_sd = 1'bz;
assign port_tran_sd_dir = 1'b0;     // SD is input and not used

// tie off the rest of the pins we are not using
assign cram0_a = 'h0;
assign cram0_dq = {16{1'bZ}};
assign cram0_clk = 0;
assign cram0_adv_n = 1;
assign cram0_cre = 0;
assign cram0_ce0_n = 1;
assign cram0_ce1_n = 1;
assign cram0_oe_n = 1;
assign cram0_we_n = 1;
assign cram0_ub_n = 1;
assign cram0_lb_n = 1;

assign cram1_a = 'h0;
assign cram1_dq = {16{1'bZ}};
assign cram1_clk = 0;
assign cram1_adv_n = 1;
assign cram1_cre = 0;
assign cram1_ce0_n = 1;
assign cram1_ce1_n = 1;
assign cram1_oe_n = 1;
assign cram1_we_n = 1;
assign cram1_ub_n = 1;
assign cram1_lb_n = 1;

assign dram_a = 'h0;
assign dram_ba = 'h0;
assign dram_dq = {16{1'bZ}};
assign dram_dqm = 'h0;
assign dram_clk = 'h0;
assign dram_cke = 'h0;
assign dram_ras_n = 'h1;
assign dram_cas_n = 'h1;
assign dram_we_n = 'h1;

assign sram_a = 'h0;
assign sram_dq = {16{1'bZ}};
assign sram_oe_n  = 1;
assign sram_we_n  = 1;
assign sram_ub_n  = 1;
assign sram_lb_n  = 1;

assign dbg_tx = 1'bZ;
assign user1 = 1'bZ;
assign aux_scl = 1'bZ;
assign vpll_feed = 1'bZ;


// for bridge write data, we just broadcast it to all bus devices
// for bridge read data, we have to mux it
// add your own devices here
always @(*) begin
    casex(bridge_addr)
    default: begin
        bridge_rd_data <= 0;
    end
    32'h40xxxxxx: begin
        // save dump buffer, stored byte 0 in [31:24]; mirrored to match the
        // byte order APF reads in (siphon_ctrl.le_mode)
        bridge_rd_data <= le_mode ? {buf_q[7:0], buf_q[15:8], buf_q[23:16], buf_q[31:24]} : buf_q;
    end
    32'h50xxxxxx: begin
        bridge_rd_data <= struct_q;     // open_dataslot_file_t
    end
    32'hF8xxxxxx: begin
        bridge_rd_data <= cmd_bridge_rd_data;
    end
    endcase
end


//
// host/target command handler
//
    wire            reset_n;                // driven by host commands, can be used as core-wide reset
    wire    [31:0]  cmd_bridge_rd_data;
    
// bridge host commands
// synchronous to clk_74a
    wire            status_boot_done = pll_core_locked_s; 
    wire            status_setup_done = pll_core_locked_s; // rising edge triggers a target command
    wire            status_running = reset_n; // we are running as soon as reset_n goes high

    wire            dataslot_requestread;
    wire    [15:0]  dataslot_requestread_id;
    wire            dataslot_requestread_ack = 1;
    wire            dataslot_requestread_ok = 1;

    wire            dataslot_requestwrite;
    wire    [15:0]  dataslot_requestwrite_id;
    wire    [31:0]  dataslot_requestwrite_size;
    wire            dataslot_requestwrite_ack = 1;
    wire            dataslot_requestwrite_ok = 1;

    wire            dataslot_update;
    wire    [15:0]  dataslot_update_id;
    wire    [31:0]  dataslot_update_size;
    
    wire            dataslot_allcomplete;

    wire     [31:0] rtc_epoch_seconds;
    wire     [31:0] rtc_date_bcd;
    wire     [31:0] rtc_time_bcd;
    wire            rtc_valid;

    wire            savestate_supported;
    wire    [31:0]  savestate_addr;
    wire    [31:0]  savestate_size;
    wire    [31:0]  savestate_maxloadsize;

    wire            savestate_start;
    wire            savestate_start_ack;
    wire            savestate_start_busy;
    wire            savestate_start_ok;
    wire            savestate_start_err;

    wire            savestate_load;
    wire            savestate_load_ack;
    wire            savestate_load_busy;
    wire            savestate_load_ok;
    wire            savestate_load_err;
    
    wire            osnotify_inmenu;

// bridge target commands
// synchronous to clk_74a

    wire            target_dataslot_read = 1'b0;
    wire            target_dataslot_write;
    wire            target_dataslot_getfile = 1'b0;
    wire            target_dataslot_openfile;
    
    wire            target_dataslot_ack;        
    wire            target_dataslot_done;
    wire    [2:0]   target_dataslot_err;

    wire    [15:0]  target_dataslot_id;
    wire    [31:0]  target_dataslot_slotoffset;
    wire    [31:0]  target_dataslot_bridgeaddr;
    wire    [31:0]  target_dataslot_length;
    
    wire    [31:0]  target_buffer_param_struct = 32'h5000_0000;
    wire    [31:0]  target_buffer_resp_struct  = 32'h5000_0000;
    
// bridge data slot access
// synchronous to clk_74a

    wire    [9:0]   datatable_addr;
    wire            datatable_wren;
    wire    [31:0]  datatable_data;
    wire    [31:0]  datatable_q;

core_bridge_cmd icb (

    .clk                ( clk_74a ),
    .reset_n            ( reset_n ),

    .bridge_endian_little   ( bridge_endian_little ),
    .bridge_addr            ( bridge_addr ),
    .bridge_rd              ( bridge_rd ),
    .bridge_rd_data         ( cmd_bridge_rd_data ),
    .bridge_wr              ( bridge_wr ),
    .bridge_wr_data         ( bridge_wr_data ),
    
    .status_boot_done       ( status_boot_done ),
    .status_setup_done      ( status_setup_done ),
    .status_running         ( status_running ),

    .dataslot_requestread       ( dataslot_requestread ),
    .dataslot_requestread_id    ( dataslot_requestread_id ),
    .dataslot_requestread_ack   ( dataslot_requestread_ack ),
    .dataslot_requestread_ok    ( dataslot_requestread_ok ),

    .dataslot_requestwrite      ( dataslot_requestwrite ),
    .dataslot_requestwrite_id   ( dataslot_requestwrite_id ),
    .dataslot_requestwrite_size ( dataslot_requestwrite_size ),
    .dataslot_requestwrite_ack  ( dataslot_requestwrite_ack ),
    .dataslot_requestwrite_ok   ( dataslot_requestwrite_ok ),

    .dataslot_update            ( dataslot_update ),
    .dataslot_update_id         ( dataslot_update_id ),
    .dataslot_update_size       ( dataslot_update_size ),
    
    .dataslot_allcomplete   ( dataslot_allcomplete ),

    .rtc_epoch_seconds      ( rtc_epoch_seconds ),
    .rtc_date_bcd           ( rtc_date_bcd ),
    .rtc_time_bcd           ( rtc_time_bcd ),
    .rtc_valid              ( rtc_valid ),
    
    .savestate_supported    ( savestate_supported ),
    .savestate_addr         ( savestate_addr ),
    .savestate_size         ( savestate_size ),
    .savestate_maxloadsize  ( savestate_maxloadsize ),

    .savestate_start        ( savestate_start ),
    .savestate_start_ack    ( savestate_start_ack ),
    .savestate_start_busy   ( savestate_start_busy ),
    .savestate_start_ok     ( savestate_start_ok ),
    .savestate_start_err    ( savestate_start_err ),

    .savestate_load         ( savestate_load ),
    .savestate_load_ack     ( savestate_load_ack ),
    .savestate_load_busy    ( savestate_load_busy ),
    .savestate_load_ok      ( savestate_load_ok ),
    .savestate_load_err     ( savestate_load_err ),

    .osnotify_inmenu        ( osnotify_inmenu ),
    .cart_power             ( cart_power ),
    
    .target_dataslot_read       ( target_dataslot_read ),
    .target_dataslot_write      ( target_dataslot_write ),
    .target_dataslot_getfile    ( target_dataslot_getfile ),
    .target_dataslot_openfile   ( target_dataslot_openfile ),
    
    .target_dataslot_ack        ( target_dataslot_ack ),
    .target_dataslot_done       ( target_dataslot_done ),
    .target_dataslot_err        ( target_dataslot_err ),

    .target_dataslot_id         ( target_dataslot_id ),
    .target_dataslot_slotoffset ( target_dataslot_slotoffset ),
    .target_dataslot_bridgeaddr ( target_dataslot_bridgeaddr ),
    .target_dataslot_length     ( target_dataslot_length ),

    .target_buffer_param_struct ( target_buffer_param_struct ),
    .target_buffer_resp_struct  ( target_buffer_resp_struct ),
    
    .datatable_addr         ( datatable_addr ),
    .datatable_wren         ( datatable_wren ),
    .datatable_data         ( datatable_data ),
    .datatable_q            ( datatable_q )

);



////////////////////////////////////////////////////////////////////////////////////////
//
// SaveSiphon
//

// (bus wires declared at the top of the module)

// ---- GBA engine
    wire    [15:0]  gba_rdata;
    wire            gba_done, gba_busy;
    wire    [15:0]  gba_ad_out;
    wire            gba_ad_oe;
    wire    [7:0]   gba_hi_out;
    wire            gba_hi_oe;
    wire            gba_cs1_n, gba_cs2_n, gba_rd_n, gba_wr_n;

gba_cart_bus cart_bus (
    .clk        ( clk_74a ),
    .enable     ( bus_enable & ~bus_mode ),
    .start      ( bus_start & ~bus_mode ),
    .op         ( bus_op ),
    .addr       ( bus_addr ),
    .wdata      ( bus_wdata ),
    .rdata      ( gba_rdata ),
    .done       ( gba_done ),
    .busy       ( gba_busy ),
    .ad_out     ( gba_ad_out ),
    .ad_oe      ( gba_ad_oe ),
    .ad_in      ( {cart_tran_bank2, cart_tran_bank3} ),
    .hi_out     ( gba_hi_out ),
    .hi_oe      ( gba_hi_oe ),
    .hi_in      ( cart_tran_bank1 ),
    .cs1_n      ( gba_cs1_n ),
    .cs2_n      ( gba_cs2_n ),
    .rd_n       ( gba_rd_n ),
    .wr_n       ( gba_wr_n )
);

// ---- Game Boy engine
    wire    [7:0]   gb_rdata;
    wire            gb_done;
    wire    [15:0]  gb_ad_out;
    wire            gb_ad_oe;
    wire    [7:0]   gb_hi_out;
    wire            gb_hi_oe;
    wire            gb_cs_n, gb_rd_n, gb_wr_n, gb_phi;

gb_cart_bus gb_bus (
    .clk        ( clk_74a ),
    .enable     ( bus_enable & bus_mode ),
    .start      ( bus_start & bus_mode ),
    .op         ( bus_op ),
    .addr       ( bus_addr[15:0] ),
    .wdata      ( bus_wdata ),
    .rdata      ( gb_rdata ),
    .done       ( gb_done ),
    .ad_out     ( gb_ad_out ),
    .ad_oe      ( gb_ad_oe ),
    .hi_out     ( gb_hi_out ),
    .hi_oe      ( gb_hi_oe ),
    .hi_in      ( cart_tran_bank1 ),
    .cs_n       ( gb_cs_n ),
    .rd_n       ( gb_rd_n ),
    .wr_n       ( gb_wr_n ),
    .phi        ( gb_phi )
);

// ---- pin mux: exactly one engine owns the connector (the other is held in
// reset by its enable), and the write guard sees whatever reaches the pins
assign bus_rdata  = bus_mode ? {8'd0, gb_rdata} : gba_rdata;
assign bus_done   = bus_mode ? gb_done   : gba_done;
assign bus_busy   = bus_mode ? 1'b0      : gba_busy;
assign bus_ad_out = bus_mode ? gb_ad_out : gba_ad_out;
assign bus_ad_oe  = bus_enable & (bus_mode ? gb_ad_oe : gba_ad_oe);
assign bus_hi_out = bus_mode ? gb_hi_out : gba_hi_out;
assign bus_hi_oe  = bus_enable & (bus_mode ? gb_hi_oe : gba_hi_oe);
assign bus_cs1_n  = bus_mode ? gb_cs_n   : gba_cs1_n;
assign bus_cs2_n  = bus_mode ? 1'b1      : gba_cs2_n;
assign bus_rd_n   = bus_mode ? gb_rd_n   : gba_rd_n;
assign bus_wr_n   = bus_mode ? gb_wr_n   : gba_wr_n;
assign bus_phi    = bus_mode ? gb_phi    : 1'b0;

// independent write guard on WR#: refuses anything but the GBA Flash
// bank-select sequence, GBA EEPROM read requests and GB mapper-register
// writes, and latches a fault until core reset
cart_write_guard guard (
    .clk         ( clk_74a ),
    .reset_n     ( reset_n ),
    .permit_bank ( permit_bank ),
    .permit_ee   ( permit_ee ),
    .gb_mode     ( bus_mode ),
    .permit_mbc  ( permit_mbc ),
    .cs1_n       ( bus_cs1_n ),
    .cs2_n       ( bus_cs2_n ),
    .rd_n        ( bus_rd_n ),
    .wr_n        ( bus_wr_n ),
    .ad_out      ( bus_ad_out ),
    .ad_oe       ( bus_ad_oe ),
    .hi_out      ( bus_hi_out ),
    .hi_oe       ( bus_hi_oe ),
    .wr_n_pin    ( guard_wr_n ),
    .fault       ( guard_fault ),
    .fault_code  ( guard_fault_code )
);

// buffer: 32K x 32 (128 KiB), written by the sequencer, read by APF
    wire            buf_we;
    wire    [14:0]  buf_waddr;
    wire    [31:0]  buf_wdata;
    reg     [31:0]  save_buf [0:32767];
always @(posedge clk_74a) begin
    if (buf_we) save_buf[buf_waddr] <= buf_wdata;
    buf_q <= save_buf[bridge_addr[16:2]];
end

    wire    [2:0]   ui_mode;
    wire    [1:0]   ui_menu_sel;
    wire    [4:0]   ui_status;
    wire    [7:0]   ui_progress;
    wire    [2:0]   ui_err_code;
    wire    [1:0]   ui_cart_kind;
    wire    [127:0] ui_name;
    wire    [4:0]   ui_name_len;
    wire    [31:0]  ui_code;
    wire            ui_code_len;
    wire    [3:0]   ui_save_label;
    wire    [4:0]   ui_rom_log2;
    wire    [2:0]   ui_mapper;
    wire    [1:0]   ui_done_what;
    wire    [511:0] ui_path;

siphon_ctrl ctrl (
    .clk                ( clk_74a ),
    .reset_n            ( reset_n ),
    .cart_power         ( cart_power ),
    .key_a              ( cont1_key[4] ),
    .key_b              ( cont1_key[5] ),
    .key_up             ( cont1_key[0] ),
    .key_down           ( cont1_key[1] ),

    .permit_bank        ( permit_bank ),
    .permit_ee          ( permit_ee ),
    .permit_mbc         ( permit_mbc ),
    .guard_fault        ( guard_fault ),

    .bus_enable         ( bus_enable ),
    .bus_mode           ( bus_mode ),
    .bus_start          ( bus_start ),
    .bus_op             ( bus_op ),
    .bus_addr           ( bus_addr ),
    .bus_wdata          ( bus_wdata ),
    .bus_rdata          ( bus_rdata ),
    .bus_done           ( bus_done ),

    .buf_we             ( buf_we ),
    .buf_waddr          ( buf_waddr ),
    .buf_wdata          ( buf_wdata ),

    .tgt_openfile       ( target_dataslot_openfile ),
    .tgt_write          ( target_dataslot_write ),
    .tgt_id             ( target_dataslot_id ),
    .tgt_slotoffset     ( target_dataslot_slotoffset ),
    .tgt_bridgeaddr     ( target_dataslot_bridgeaddr ),
    .tgt_length         ( target_dataslot_length ),
    .tgt_ack            ( target_dataslot_ack ),
    .tgt_done           ( target_dataslot_done ),
    .tgt_err            ( target_dataslot_err ),

    .struct_word_addr   ( bridge_addr[9:2] ),
    .struct_q           ( struct_q ),
    .le_mode            ( le_mode ),

    .ui_mode            ( ui_mode ),
    .menu_sel           ( ui_menu_sel ),
    .status             ( ui_status ),
    .progress           ( ui_progress ),
    .err_code           ( ui_err_code ),
    .cart_kind          ( ui_cart_kind ),
    .name_flat          ( ui_name ),
    .name_len           ( ui_name_len ),
    .code_flat          ( ui_code ),
    .code_len           ( ui_code_len ),
    .save_label         ( ui_save_label ),
    .rom_log2           ( ui_rom_log2 ),
    .mapper             ( ui_mapper ),
    .done_what          ( ui_done_what ),
    .path_flat          ( ui_path )
);

// video: 320x240 status screen on the 12.288 MHz pixel clock
assign video_rgb_clock = clk_core_12288;
assign video_rgb_clock_90 = clk_core_12288_90deg;
assign video_skip = 1'b0;

status_screen screen (
    .clk        ( clk_core_12288 ),
    .ui_mode    ( ui_mode ),
    .menu_sel   ( ui_menu_sel ),
    .status     ( ui_status ),
    .progress   ( ui_progress ),
    .err_code   ( ui_err_code ),
    .fault_code ( guard_fault_code ),
    .cart_kind  ( ui_cart_kind ),
    .name_flat  ( ui_name ),
    .name_len   ( ui_name_len ),
    .code_flat  ( ui_code ),
    .code_len   ( ui_code_len ),
    .save_label ( ui_save_label ),
    .rom_log2   ( ui_rom_log2 ),
    .mapper     ( ui_mapper ),
    .done_what  ( ui_done_what ),
    .path_flat  ( ui_path ),
    .rgb        ( video_rgb ),
    .de         ( video_de ),
    .hs         ( video_hs ),
    .vs         ( video_vs )
);



//
// audio i2s silence generator
// see other examples for actual audio generation
//

assign audio_mclk = audgen_mclk;
assign audio_dac = audgen_dac;
assign audio_lrck = audgen_lrck;

// generate MCLK = 12.288mhz with fractional accumulator
    reg         [21:0]  audgen_accum;
    reg                 audgen_mclk;
    parameter   [20:0]  CYCLE_48KHZ = 21'd122880 * 2;
always @(posedge clk_74a) begin
    audgen_accum <= audgen_accum + CYCLE_48KHZ;
    if(audgen_accum >= 21'd742500) begin
        audgen_mclk <= ~audgen_mclk;
        audgen_accum <= audgen_accum - 21'd742500 + CYCLE_48KHZ;
    end
end

// generate SCLK = 3.072mhz by dividing MCLK by 4
    reg [1:0]   aud_mclk_divider;
    wire        audgen_sclk = aud_mclk_divider[1] /* synthesis keep*/;
    reg         audgen_lrck_1;
always @(posedge audgen_mclk) begin
    aud_mclk_divider <= aud_mclk_divider + 1'b1;
end

// shift out audio data as I2S 
// 32 total bits per channel, but only 16 active bits at the start and then 16 dummy bits
//
    reg     [4:0]   audgen_lrck_cnt;    
    reg             audgen_lrck;
    reg             audgen_dac;
always @(negedge audgen_sclk) begin
    audgen_dac <= 1'b0;
    // 48khz * 64
    audgen_lrck_cnt <= audgen_lrck_cnt + 1'b1;
    if(audgen_lrck_cnt == 31) begin
        // switch channels
        audgen_lrck <= ~audgen_lrck;
        
    end 
end


///////////////////////////////////////////////


    wire    clk_core_12288;
    wire    clk_core_12288_90deg;
    
    wire    pll_core_locked;
    wire    pll_core_locked_s;
synch_3 s01(pll_core_locked, pll_core_locked_s, clk_74a);

mf_pllbase mp1 (
    .refclk         ( clk_74a ),
    .rst            ( 0 ),
    
    .outclk_0       ( clk_core_12288 ),
    .outclk_1       ( clk_core_12288_90deg ),
    
    .locked         ( pll_core_locked )
);


    
endmodule
