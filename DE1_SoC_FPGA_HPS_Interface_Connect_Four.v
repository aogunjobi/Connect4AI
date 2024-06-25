module DE1_SoC_Computer (
    // FPGA Pins
    // Clock pins
    CLOCK_50,       // 50 MHz clock
    CLOCK2_50,      // Secondary 50 MHz clock
    CLOCK3_50,      // Tertiary 50 MHz clock
    CLOCK4_50,      // Quaternary 50 MHz clock

    // ADC
    ADC_CS_N,       // ADC chip select
    ADC_DIN,        // ADC data input
    ADC_DOUT,       // ADC data output
    ADC_SCLK,       // ADC serial clock

    // Audio
    AUD_ADCDAT,     // Audio ADC data
    AUD_ADCLRCK,    // Audio ADC left/right clock
    AUD_BCLK,       // Audio bit clock
    AUD_DACDAT,     // Audio DAC data
    AUD_DACLRCK,    // Audio DAC left/right clock
    AUD_XCK,        // Audio transmit clock

    // SDRAM
    DRAM_ADDR,      // SDRAM address bus
    DRAM_BA,        // SDRAM bank address
    DRAM_CAS_N,     // SDRAM column address strobe
    DRAM_CKE,       // SDRAM clock enable
    DRAM_CLK,       // SDRAM clock
    DRAM_CS_N,      // SDRAM chip select
    DRAM_DQ,        // SDRAM data bus
    DRAM_LDQM,      // SDRAM data mask
    DRAM_RAS_N,     // SDRAM row address strobe
    DRAM_UDQM,      // SDRAM data mask
    DRAM_WE_N,      // SDRAM write enable

    // I2C Bus for Configuration of the Audio and Video-In Chips
    FPGA_I2C_SCLK,  // FPGA I2C serial clock
    FPGA_I2C_SDAT,  // FPGA I2C serial data

    // 40-Pin Headers
    GPIO_0,         // General-purpose I/O pin 0
    GPIO_1,         // General-purpose I/O pin 1
    
    // Seven Segment Displays
    HEX0,           // Seven segment display 0
    HEX1,           // Seven segment display 1
    HEX2,           // Seven segment display 2
    HEX3,           // Seven segment display 3
    HEX4,           // Seven segment display 4
    HEX5,           // Seven segment display 5

    // IR
    IRDA_RXD,       // Infrared receiver data
    IRDA_TXD,       // Infrared transmitter data

    // Pushbuttons
    KEY,            // Pushbutton input

    // LEDs
    LEDR,           // LED output

    // PS2 Ports
    PS2_CLK,        // PS/2 clock
    PS2_DAT,        // PS/2 data
    
    PS2_CLK2,       // Secondary PS/2 clock
    PS2_DAT2,       // Secondary PS/2 data

    // Slider Switches
    SW,             // Slider switch input

    // Video-In
    TD_CLK27,       // Video-in clock 27
    TD_DATA,        // Video-in data
    TD_HS,          // Video-in horizontal sync
    TD_RESET_N,     // Video-in reset
    TD_VS,          // Video-in vertical sync

    // VGA
    VGA_B,          // VGA blue color
    VGA_BLANK_N,    // VGA blanking
    VGA_CLK,        // VGA clock
    VGA_G,          // VGA green color
    VGA_HS,         // VGA horizontal sync
    VGA_R,          // VGA red color
    VGA_SYNC_N,     // VGA sync
    VGA_VS,         // VGA vertical sync

    // HPS Pins
    // DDR3 SDRAM
    HPS_DDR3_ADDR,      // HPS DDR3 SDRAM address bus
    HPS_DDR3_BA,        // HPS DDR3 SDRAM bank address
    HPS_DDR3_CAS_N,     // HPS DDR3 SDRAM column address strobe
    HPS_DDR3_CKE,       // HPS DDR3 SDRAM clock enable
    HPS_DDR3_CK_N,      // HPS DDR3 SDRAM clock negative
    HPS_DDR3_CK_P,      // HPS DDR3 SDRAM clock positive
    HPS_DDR3_CS_N,      // HPS DDR3 SDRAM chip select
    HPS_DDR3_DM,        // HPS DDR3 SDRAM data mask
    HPS_DDR3_DQ,        // HPS DDR3 SDRAM data bus
    HPS_DDR3_DQS_N,     // HPS DDR3 SDRAM data strobe negative
    HPS_DDR3_DQS_P,     // HPS DDR3 SDRAM data strobe positive
    HPS_DDR3_ODT,       // HPS DDR3 SDRAM on-die termination
    HPS_DDR3_RAS_N,     // HPS DDR3 SDRAM row address strobe negative
    HPS_DDR3_RESET_N,   // HPS DDR3 SDRAM reset negative
    HPS_DDR3_RZQ,       // HPS DDR3 SDRAM impedance

    // Ethernet
    HPS_ENET_GTX_CLK,   // HPS Ethernet GTX clock
    HPS_ENET_INT_N,     // HPS Ethernet interrupt negative
    HPS_ENET_MDC,       // HPS Ethernet management data clock
    HPS_ENET_MDIO,      // HPS Ethernet management data input/output
    HPS_ENET_RX_CLK,    // HPS Ethernet receive clock
    HPS_ENET_RX_DATA,   // HPS Ethernet receive data
    HPS_ENET_RX_DV,     // HPS Ethernet receive data valid
    HPS_ENET_TX_DATA,   // HPS Ethernet transmit data
    HPS_ENET_TX_EN,     // HPS Ethernet transmit enable

    // Flash
    HPS_FLASH_DATA,     // HPS flash data
    HPS_FLASH_DCLK,     // HPS flash data clock
    HPS_FLASH_NCSO,     // HPS flash chip select

    // Accelerometer
    HPS_GSENSOR_INT,    // HPS accelerometer interrupt
        
    // General Purpose I/O
    HPS_GPIO,           // HPS general-purpose I/O
        
    // I2C
    HPS_I2C_CONTROL,    // HPS I2C control
    HPS_I2C1_SCLK,      // HPS I2C1 serial clock
    HPS_I2C1_SDAT,      // HPS I2C1 serial data
    HPS_I2C2_SCLK,      // HPS I2C2 serial clock
    HPS_I2C2_SDAT,      // HPS I2C2 serial data

    // Pushbutton
    HPS_KEY,            // HPS pushbutton

    // LED
    HPS_LED,            // HPS LED
        
    // SD Card
    HPS_SD_CLK,         // HPS SD card clock
    HPS_SD_CMD,         // HPS SD card command
    HPS_SD_DATA,        // HPS SD card data

    // SPI
    HPS_SPIM_CLK,       // HPS SPI master clock
    HPS_SPIM_MISO,      // HPS SPI master input, slave output
    HPS_SPIM_MOSI,      // HPS SPI master output, slave input
    HPS_SPIM_SS,        // HPS SPI master slave select

    // UART
    HPS_UART_RX,        // HPS UART receive
    HPS_UART_TX,        // HPS UART transmit

    // USB
    HPS_CONV_USB_N,     // HPS USB converter negative
    HPS_USB_CLKOUT,     // HPS USB clock out
    HPS_USB_DATA,       // HPS USB data
    HPS_USB_DIR,        // HPS USB direction
    HPS_USB_NXT,        // HPS USB next
    HPS_USB_STP         // HPS USB stop
);


//=======================================================
// PORT Declarations
//=======================================================

////////////////////////////////////
// FPGA Pins
////////////////////////////////////

// Define input and output pins for various components

input                   CLOCK_50;
input                   CLOCK2_50;
input                   CLOCK3_50;
input                   CLOCK4_50;

inout                   ADC_CS_N;
output                  ADC_DIN;
input                   ADC_DOUT;
output                  ADC_SCLK;

input                   AUD_ADCDAT;
inout                   AUD_ADCLRCK;
inout                   AUD_BCLK;
output                  AUD_DACDAT;
inout                   AUD_DACLRCK;
output                  AUD_XCK;

output      [12:0]      DRAM_ADDR;
output      [1:0]       DRAM_BA;
output                  DRAM_CAS_N;
output                  DRAM_CKE;
output                  DRAM_CLK;
output                  DRAM_CS_N;
inout       [15:0]      DRAM_DQ;
output                  DRAM_LDQM;
output                  DRAM_RAS_N;
output                  DRAM_UDQM;
output                  DRAM_WE_N;

output                  FPGA_I2C_SCLK;
inout                   FPGA_I2C_SDAT;

inout       [35:0]      GPIO_0;
inout       [35:0]      GPIO_1;

output      [6:0]       HEX0;
output      [6:0]       HEX1;
output      [6:0]       HEX2;
output      [6:0]       HEX3;
output      [6:0]       HEX4;
output      [6:0]       HEX5;

input                   IRDA_RXD;
output                  IRDA_TXD;

input       [3:0]       KEY;

output      [9:0]       LEDR;

inout                   PS2_CLK;
inout                   PS2_DAT;

inout                   PS2_CLK2;
inout                   PS2_DAT2;

input       [9:0]       SW;

input                   TD_CLK27;
input       [7:0]       TD_DATA;
input                   TD_HS;
output                  TD_RESET_N;
input                   TD_VS;

output      [7:0]       VGA_B;
output                  VGA_BLANK_N;
output                  VGA_CLK;
output      [7:0]       VGA_G;
output                  VGA_HS;
output      [7:0]       VGA_R;
output                  VGA_SYNC_N;
output                  VGA_VS;

////////////////////////////////////
// HPS Pins
////////////////////////////////////

// Define input and output pins for HPS peripherals

output      [14:0]      HPS_DDR3_ADDR;
output      [2:0]       HPS_DDR3_BA;
output                  HPS_DDR3_CAS_N;
output                  HPS_DDR3_CKE;
output                  HPS_DDR3_CK_N;
output                  HPS_DDR3_CK_P;
output                  HPS_DDR3_CS_N;
output      [3:0]       HPS_DDR3_DM;
inout       [31:0]      HPS_DDR3_DQ;
inout       [3:0]       HPS_DDR3_DQS_N;
inout       [3:0]       HPS_DDR3_DQS_P;
output                  HPS_DDR3_ODT;
output                  HPS_DDR3_RAS_N;
output                  HPS_DDR3_RESET_N;
input                   HPS_DDR3_RZQ;
output                  HPS_DDR3_WE_N;

output                  HPS_ENET_GTX_CLK;
inout                   HPS_ENET_INT_N;
output                  HPS_ENET_MDC;
inout                   HPS_ENET_MDIO;
input                   HPS_ENET_RX_CLK;
input       [3:0]       HPS_ENET_RX_DATA;
input                   HPS_ENET_RX_DV;
output      [3:0]       HPS_ENET_TX_DATA;
output                  HPS_ENET_TX_EN;

inout       [3:0]       HPS_FLASH_DATA;
output                  HPS_FLASH_DCLK;
output                  HPS_FLASH_NCSO;

inout                   HPS_GSENSOR_INT;

inout       [1:0]       HPS_GPIO;

inout                   HPS_I2C_CONTROL;
inout                   HPS_I2C1_SCLK;
inout                   HPS_I2C1_SDAT;
inout                   HPS_I2C2_SCLK;
inout                   HPS_I2C2_SDAT;

inout                   HPS_KEY;

inout                   HPS_LED;

output                  HPS_SD_CLK;
inout                   HPS_SD_CMD;
inout       [3:0]       HPS_SD_DATA;

output                  HPS_SPIM_CLK;
input                   HPS_SPIM_MISO;
output                  HPS_SPIM_MOSI;
inout                   HPS_SPIM_SS;

input                   HPS_UART_RX;
output                  HPS_UART_TX;

inout                   HPS_CONV_USB_N;
input                   HPS_USB_CLKOUT;
inout       [7:0]       HPS_USB_DATA;
input                   HPS_USB_DIR;
input                   HPS_USB_NXT;
output                  HPS_USB_STP;

//=======================================================
// REG/WIRE Declarations
//=======================================================

// Declare registers and wires for various functionalities

wire        [15:0]      hex3_hex0;
assign HEX4 = 7'b1111111;
assign HEX5 = 7'b1111111;

HexDigit Digit0(HEX0, hex3_hex0[3:0]);
HexDigit Digit1(HEX1, hex3_hex0[7:4]);
HexDigit Digit2(HEX2, hex3_hex0[11:8]);
HexDigit Digit3(HEX3, hex3_hex0[15:12]);

//=======================================================
// SRAM/VGA State Machine
//=======================================================
wire [31:0] sram_readdata ;
reg [31:0] data_buffer, sram_writedata ;
reg [7:0] sram_address; 
reg sram_write ;
wire sram_clken = 1'b1;
wire sram_chipselect = 1'b1;

// Rectangle Corners
reg signed [26:0] x1, y1;

reg [31:0] timer ; // May Need to Throttle Write-rate

//=======================================================
// Controls for VGA Memory
//=======================================================
wire [31:0] vga_out_base_address = 32'h0000_0000 ;  // VGA Base Address
reg [7:0] vga_sram_writedata ;
reg [31:0] vga_sram_address; 
reg vga_sram_write ;
wire vga_sram_clken = 1'b1;
wire vga_sram_chipselect = 1'b1;

//=======================================================
// Pixel Address
//=======================================================
reg [9:0] vga_x_cood, vga_y_cood ;
wire hps_reset;
assign LEDR[0] = hps_reset;

reg sim_reset;
wire max_iterations;

reg [5:0] state;

wire hps_move_finished;
reg ai_move_finished;

wire [83:0] gameState;
wire [3:0] aiMove;
wire [3:0] aiMove_defensive;
reg [13:0] row1;
reg [13:0] row2;
reg [13:0] row3;
reg [13:0] row4;
reg [13:0] row5;
reg [13:0] row6;
reg [5:0] move_counter;

// AI Modules Instantiation
// Instantiate AI modules for connect four game
connectFourAI intelligentAI(
    .clk            (CLOCK_50),
    .reset          (hps_reset),
    .gameState      (gameState),
    .aiMove         (aiMove)
);

connectFourAI_strategic strategicAI(
    .clk            (CLOCK_50),
    .reset          (hps_reset),
    .gameState      (gameState),
    .aiMove         (aiMove_defensive)
);

// Assign game state from rows
assign gameState = {row6, row5, row4, row3, row2, row1};

// Top-level Finite State Machine (FSM)
// Define FSM to manage game state transitions
always @(posedge CLOCK_50) begin
    // Reset FSM and AI move flag on HPS reset
    if(hps_reset)begin
        state <= 6'd0;
        ai_move_finished <= 1'b0;
        move_counter <= 5'd0;
    end
    
    // Check if there's a new game state from HPS
    switch (state) begin
        // Read SRAM address 0, used to signal new game state from HPS
        case 6'd0:
            sram_address <= 6'd0 ;
            sram_write <= 1'b0 ;
            state <= 6'd1 ;
        // Wait for one cycle after reading
        case 6'd1:
            state <= 6'd2 ;
        // Read data from SRAM
        case 6'd2:
            data_buffer <= sram_readdata ;
            sram_write <= 1'b0 ;
            state <= 6'd3 ;
        // Proceed if there's a new move, otherwise return to state 0
        case 6'd3:
            // If SRAM data at address 0 is 0, wait for new data
            if (data_buffer == 0) state <= 6'd0 ;
            // If SRAM data is nonzero, proceed to process the move
            else state <= 6'd4 ;
        // Read row 1 from SRAM
        case 6'd4:
            sram_address <= 8'd1 ;
            sram_write <= 1'b0 ;
            state <= 6'd5 ;
        case 6'd5:
            state <= 6'd6 ;
        case 6'd6:
            row1 <= sram_readdata[13:0];
            sram_write <= 1'b0 ;
            state <= 6'd7 ;
        // Read row 2 from SRAM
        case 6'd7:
            sram_address <= 8'd2 ;
            sram_write <= 1'b0 ;
            state <= 6'd8 ;
        case 6'd8:
            state <= 6'd9 ;
        case 6'd9:
            row2 <= sram_readdata[13:0];
            sram_write <= 1'b0 ;
            state <= 6'd10 ;
        // Continue similar process for other rows...

        // Get new AI move from AI compute module, write move to SRAM 7
        case 6'd22:
            sram_address <= 8'd7 ;
            // If not all AI moves have been played, use strategic AI
            if (move_counter < 5'd4) begin
                sram_writedata <= aiMove_defensive;
            end
            // Otherwise, use intelligent AI
            else sram_writedata <= aiMove;
            sram_write <= 1'b1 ;
            state <= 6'd23;
        // Done state
        case 6'd23:
            // Signal the HPS we are done
            sram_address <= 8'd0 ;
            sram_writedata <= 32'b0;
            sram_write <= 1'b1 ;
            move_counter <= move_counter + 5'd1;
            state <= 6'd0 ;
    endcase
end

//=======================================================
// Module: Structural_Interface
//=======================================================
// Description: This module instantiates the top-level system module and connects 
//              all the necessary ports between the FPGA side and the HPS side.
//=======================================================

module Structural_Interface (

    //////////////////////////////////////////////////////
    // Inputs and Outputs
    //////////////////////////////////////////////////////
    input CLOCK_50, CLOCK2_50, CLOCK3_50, CLOCK4_50,
    input AUD_ADCDAT, AUD_XCK, IRDA_RXD, TD_CLK27, TD_DATA,
    input TD_HS, TD_VS, HPS_DDR3_RZQ, HPS_UART_RX, HPS_USB_CLKOUT,
    input [3:0] KEY, [7:0] HPS_ENET_RX_DATA, HPS_SPIM_MISO,
    input [9:0] SW, [35:0] GPIO_0, GPIO_1,
    output [9:0] LEDR, [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5,
    output [7:0] VGA_B, VGA_R, VGA_G,
    output VGA_BLANK_N, VGA_CLK, VGA_HS, VGA_SYNC_N, VGA_VS,
    output DRAM_ADDR [12:0], DRAM_BA [1:0], DRAM_CAS_N, DRAM_CKE,
    output DRAM_CLK, DRAM_CS_N, DRAM_LDQM, DRAM_RAS_N, DRAM_UDQM,
    output DRAM_WE_N, HPS_DDR3_ADDR [14:0], HPS_DDR3_BA [2:0],
    output HPS_DDR3_CAS_N, HPS_DDR3_CKE, HPS_DDR3_CK_N,
    output HPS_DDR3_CK_P, HPS_DDR3_CS_N, HPS_DDR3_ODT,
    output HPS_DDR3_RAS_N, HPS_DDR3_RESET_N, HPS_DDR3_WE_N,
    output HPS_ENET_GTX_CLK, HPS_ENET_MDC, HPS_ENET_RX_CLK,
    output HPS_FLASH_DCLK, HPS_FLASH_NCSO, HPS_GSENSOR_INT,
    output HPS_I2C_CONTROL, HPS_I2C1_SCLK, HPS_I2C1_SDAT,
    output HPS_I2C2_SCLK, HPS_I2C2_SDAT, HPS_KEY, HPS_LED,
    output HPS_CONV_USB_N, HPS_USB_STP, HPS_USB_DIR, HPS_USB_NXT,
    output HPS_SD_CLK, HPS_SD_CMD, HPS_SPIM_CLK, HPS_SPIM_MOSI,
    output HPS_SPIM_SS,

    //////////////////////////////////////////////////////
    // Wires
    //////////////////////////////////////////////////////
    wire [15:0] hex3_hex0,
    wire [31:0] sram_readdata,
    wire [31:0] vga_out_base_address,
    wire [83:0] gameState,
    wire [3:0] aiMove, aiMove_defensive,

    //////////////////////////////////////////////////////
    // Registers
    //////////////////////////////////////////////////////
    reg [31:0] data_buffer, sram_writedata,
    reg [7:0] sram_address, vga_sram_writedata,
    reg [31:0] timer, vga_sram_address,
    reg [9:0] vga_x_cood, vga_y_cood,
    reg [5:0] state, move_counter,
    reg signed [26:0] x1, y1,
    reg sram_write, vga_sram_write,
    reg ai_move_finished, hps_reset, sim_reset, max_iterations;

    // Instantiation of the top-level system module
    Computer_System The_System (

        //////////////////////////////////////////////////////
        // FPGA Side
        //////////////////////////////////////////////////////
        .hps_reset_port_export(hps_reset),
        .max_iterations_port_export(max_iterations),
        .system_pll_ref_clk_clk(CLOCK_50),
        .system_pll_ref_reset_reset(1'b0),
        .onchip_sram_s1_address(sram_address),
        .onchip_sram_s1_clken(sram_clken),
        .onchip_sram_s1_chipselect(sram_chipselect),
        .onchip_sram_s1_write(sram_write),
        .onchip_sram_s1_readdata(sram_readdata),
        .onchip_sram_s1_writedata(sram_writedata),
        .onchip_sram_s1_byteenable(4'b1111),
        .onchip_vga_buffer_s1_address(vga_sram_address),
        .onchip_vga_buffer_s1_clken(vga_sram_clken),
        .onchip_vga_buffer_s1_chipselect(vga_sram_chipselect),
        .onchip_vga_buffer_s1_write(vga_sram_write),
        .onchip_vga_buffer_s1_readdata(),
        .onchip_vga_buffer_s1_writedata(vga_sram_writedata),
        .av_config_SCLK(FPGA_I2C_SCLK),
        .av_config_SDAT(FPGA_I2C_SDAT),
        .clock_bridge_0_in_clk_clk(CLOCK_50),
        .vga_pll_ref_clk_clk(CLOCK2_50),
        .vga_pll_ref_reset_reset(1'b0),
        .vga_CLK(VGA_CLK),
        .vga_BLANK(VGA_BLANK_N),
        .vga_SYNC(VGA_SYNC_N),
        .vga_HS(VGA_HS),
        .vga_VS(VGA_VS),
        .vga_R(VGA_R),
        .vga_G(VGA_G),
        .vga_B(VGA_B),
        .sdram_clk_clk(DRAM_CLK),
        .sdram_addr(DRAM_ADDR),
        .sdram_ba(DRAM_BA),
        .sdram_cas_n(DRAM_CAS_N),
        .sdram_cke(DRAM_CKE),
        .sdram_cs_n(DRAM_CS_N),
        .sdram_dq(DRAM_DQ),
        .sdram_dqm({DRAM_UDQM,DRAM_LDQM}),
        .sdram_ras_n(DRAM_RAS_N),
        .sdram_we_n(DRAM_WE_N),

        //////////////////////////////////////////////////////
        // HPS Side
        //////////////////////////////////////////////////////
        .memory_mem_a(HPS_DDR3_ADDR),
        .memory_mem_ba(HPS_DDR3_BA),
        .memory_mem_ck(HPS_DDR3_CK_P),
        .memory_mem_ck_n(HPS_DDR3_CK_N),
        .memory_mem_cke(HPS_DDR3_CKE),
        .memory_mem_cs_n(HPS_DDR3_CS_N),
        .memory_mem_ras_n(HPS_DDR3_RAS_N),
        .memory_mem_cas_n(HPS_DDR3_CAS_N),
        .memory_mem_we_n(HPS_DDR3_WE_N),
        .memory_mem_reset_n(HPS_DDR3_RESET_N),
        .memory_mem_dq(HPS_DDR3_DQ),
        .memory_mem_dqs(HPS_DDR3_DQS_P),
        .memory_mem_dqs_n(HPS_DDR3_DQS_N),
        .memory_mem_odt(HPS_DDR3_ODT),
        .memory_mem_dm(HPS_DDR3_DM),
        .memory_oct_rzqin(HPS_DDR3_RZQ),
        .hps_io_hps_io_gpio_inst_GPIO35(HPS_ENET_INT_N),
        .hps_io_hps_io_emac1_inst_TX_CLK(HPS_ENET_GTX_CLK),
        .hps_io_hps_io_emac1_inst_TXD0(HPS_ENET_TX_DATA[0]),
        .hps_io_hps_io_emac1_inst_TXD1(HPS_ENET_TX_DATA[1]),
        .hps_io_hps_io_emac1_inst_TXD2(HPS_ENET_TX_DATA[2]),
        .hps_io_hps_io_emac1_inst_TXD3(HPS_ENET_TX_DATA[3]),
        .hps_io_hps_io_emac1_inst_RXD0(HPS_ENET_RX_DATA[0]),
        .hps_io_hps_io_emac1_inst_MDIO(HPS_ENET_MDIO),
        .hps_io_hps_io_emac1_inst_MDC(HPS_ENET_MDC),
        .hps_io_hps_io_emac1_inst_RX_CTL(HPS_ENET_RX_DV),
        .hps_io_hps_io_emac1_inst_TX_CTL(HPS_ENET_TX_EN),
        .hps_io_hps_io_emac1_inst_RX_CLK(HPS_ENET_RX_CLK),
        .hps_io_hps_io_emac1_inst_RXD1(HPS_ENET_RX_DATA[1]),
        .hps_io_hps_io_emac1_inst_RXD2(HPS_ENET_RX_DATA[2]),
        .hps_io_hps_io_emac1_inst_RXD3(HPS_ENET_RX_DATA[3]),
        .hps_io_hps_io_qspi_inst_IO0(HPS_FLASH_DATA[0]),
        .hps_io_hps_io_qspi_inst_IO1(HPS_FLASH_DATA[1]),
        .hps_io_hps_io_qspi_inst_IO2(HPS_FLASH_DATA[2]),
        .hps_io_hps_io_qspi_inst_IO3(HPS_FLASH_DATA[3]),
        .hps_io_hps_io_qspi_inst_SS0(HPS_FLASH_NCSO),
        .hps_io_hps_io_qspi_inst_CLK(HPS_FLASH_DCLK),
        .hps_io_hps_io_gpio_inst_GPIO61(HPS_GSENSOR_INT),
        .hps_io_hps_io_gpio_inst_GPIO40(HPS_GPIO[0]),
        .hps_io_hps_io_gpio_inst_GPIO41(HPS_GPIO[1]),
        .hps_io_hps_io_gpio_inst_GPIO48(HPS_I2C_CONTROL),
        .hps_io_hps_io_i2c0_inst_SDA(HPS_I2C1_SDAT),
        .hps_io_hps_io_i2c0_inst_SCL(HPS_I2C1_SCLK),
        .hps_io_hps_io_i2c1_inst_SDA(HPS_I2C2_SDAT),
        .hps_io_hps_io_i2c1_inst_SCL(HPS_I2C2_SCLK),
        .hps_io_hps_io_gpio_inst_GPIO54(HPS_KEY),
        .hps_io_hps_io_gpio_inst_GPIO53(HPS_LED),
        .hps_io_hps_io_sdio_inst_CMD(HPS_SD_CMD),
        .hps_io_hps_io_sdio_inst_D0(HPS_SD_DATA[0]),
        .hps_io_hps_io_sdio_inst_D1(HPS_SD_DATA[1]),
        .hps_io_hps_io_sdio_inst_CLK(HPS_SD_CLK),
        .hps_io_hps_io_sdio_inst_D2(HPS_SD_DATA[2]),
        .hps_io_hps_io_sdio_inst_D3(HPS_SD_DATA[3]),
        .hps_io_hps_io_spim1_inst_CLK(HPS_SPIM_CLK),
        .hps_io_hps_io_spim1_inst_MOSI(HPS_SPIM_MOSI),
        .hps_io_hps_io_spim1_inst_MISO(HPS_SPIM_MISO),
        .hps_io_hps_io_spim1_inst_SS0(HPS_SPIM_SS),
        .hps_io_hps_io_uart0_inst_RX(HPS_UART_RX),
        .hps_io_hps_io_uart0_inst_TX(HPS_UART_TX),
        .hps_io_hps_io_gpio_inst_GPIO09(HPS_CONV_USB_N),
        .hps_io_hps_io_usb1_inst_D0(HPS_USB_DATA[0]),
        .hps_io_hps_io_usb1_inst_D1(HPS_USB_DATA[1]),
        .hps_io_hps_io_usb1_inst_D2(HPS_USB_DATA[2]),
        .hps_io_hps_io_usb1_inst_D3(HPS_USB_DATA[3]),
        .hps_io_hps_io_usb1_inst_D4(HPS_USB_DATA[4]),
        .hps_io_hps_io_usb1_inst_D5(HPS_USB_DATA[5]),
        .hps_io_hps_io_usb1_inst_D6(HPS_USB_DATA[6]),
        .hps_io_hps_io_usb1_inst_D7(HPS_USB_DATA[7]),
        .hps_io_hps_io_usb1_inst_CLK(HPS_USB_CLKOUT),
        .hps_io_hps_io_usb1_inst_STP(HPS_USB_STP),
        .hps_io_hps_io_usb1_inst_DIR(HPS_USB_DIR),
        .hps_io_hps_io_usb1_inst_NXT(HPS_USB_NXT)
    );

endmodule // end Structural_Interface


//============================================================
// Memory Block using M10K
//============================================================
// This module implements a memory block using M10K blocks
// It has 256 words of 32 bits.
//============================================================

module M10K_256_32( 
    output reg [31:0] q,
    input [31:0] d,
    input [7:0] write_address, read_address,
    input we, clk
);
    // Memory array declaration
    reg [31:0] mem [255:0] /* synthesis ramstyle = "no_rw_check, M10K" */;
	 
    // Memory read and write logic
    always @ (posedge clk) begin
        if (we) begin
            mem[write_address] <= d;
		end
        q <= mem[read_address];
    end
endmodule

//============================================================
// Memory Block using MLAB
//============================================================
// This module implements a memory block using MLAB blocks
// It has 20 words of 32 bits.
//============================================================

module MLAB_20_32(
	output reg signed [31:0] q,
	input  [31:0] data,
	input [7:0] readaddr, writeaddr,
	input wren, clock
);
	// Memory array declaration
	reg signed [31:0] mem [19:0] /* synthesis ramstyle = "no_rw_check, MLAB" */;
	
	// Memory read and write logic
	always @ (posedge clock) begin
		if (wren) begin
			mem[writeaddr] <= data;
		end
		q <= mem[readaddr];
	end
endmodule

//============================================================
// Floating Point to Integer Conversion
//============================================================
// This module converts a 16-bit signed integer to a 
// floating-point representation with a 7-bit exponent 
// and 18-bit mantissa.
//============================================================

module Int2Fp(
	input signed [15:0] iInteger,
	output[26:0] oA		
);
	// Output fields
	wire A_s;
	wire [7:0] A_e;
	wire [17:0] A_f;
    
	// Extract sign and absolute value of input integer
	assign A_s = (iInteger < 0);
	assign abs_input = (iInteger < 0)? -iInteger : iInteger ;
	 
	// Determine the exponent and mantissa of the floating-point number
	wire [7:0] shft_amt;
	assign shft_amt = (abs_input != 0) ? 127 + 18 - $clog2(abs_input) : 0;
	assign A_e = shft_amt;
	assign A_f = (abs_input != 0) ? (abs_input << (18 - shft_amt)) : 0;
	
	// Output floating-point representation
	assign oA = (iInteger == 0) ? 27'b0 : {A_s, A_e, A_f};
endmodule

//============================================================
// Floating Point to Integer Conversion
//============================================================
// This module converts a floating-point representation 
// to a 16-bit signed integer.
//============================================================

module Fp2Int(
	input [26:0] iA,
	output reg [15:0] oInteger
);
	// Extract fields of input floating-point representation
	wire A_s;
	wire [7:0] A_e;
	wire [17:0] A_f;
    
	// Extract sign, exponent, and mantissa
	assign A_s = iA[26];
	assign A_e = iA[25:18];
	assign A_f = iA[17:0];
	 
	// Compute output integer
	always @(*) begin
		if (A_e < 127) oInteger = 16'b0;
		else if (A_e > 141) oInteger = (A_s) ? -32768 : 32767;
		else oInteger = (A_s) ? -((1 << (A_e - 127)) + (A_f >> (A_e - 127))) : ((1 << (A_e - 127)) + (A_f >> (A_e - 127)));
	end
endmodule

//============================================================
// Floating Point Shift
//============================================================
// This module performs left or right shift operation 
// on a floating-point number.
//============================================================

module FpShift(
	input [26:0] iA,
	input [7:0] iShift,
	output [26:0] oShifted
);
	// Extract fields of input floating-point number
	wire A_s;
	wire [7:0] A_e;
	wire [17:0] A_f;
	assign A_s = iA[26];
	assign A_e = iA[25:18];
	assign A_f = iA[17:0];
	
	// Perform shift operation
	assign oShifted = {A_s, A_e + iShift, A_f};
endmodule

//============================================================
// Floating Point Negation
//============================================================
// This module negates the sign of a floating-point number.
//============================================================

module FpNegate(
	input [26:0] iA,
	output [26:0] oNegative
);
	// Extract fields of input floating-point number
	wire A_s;
	wire [7:0] A_e;
	wire [17:0] A_f;
	assign A_s = iA[26];
	assign A_e = iA[25:18];
	assign A_f = iA[17:0];
	
	// Negate the sign
	assign oNegative = {~A_s, A_e, A_f};
endmodule

//============================================================
// Floating Point Absolute Value
//============================================================
// This module computes the absolute value of a 
// floating-point number.
//============================================================

module FpAbs(
	input [26:0] iA,
	output [26:0] oAbs
);
	// Extract fields of input floating-point number
	wire [7:0] A_e;
	wire [17:0] A_f;
	assign A_e = iA[25:18];
	assign A_f = iA[17:0];
	
	// Compute absolute value
	assign oAbs = {1'b0, A_e, A_f};
endmodule

//============================================================
// Floating Point Comparison
//============================================================
// This module compares two floating-point numbers 
// and determines if the first number is greater 
// than or equal to the second number.
//============================================================

module FpCompare(
	input [26:0] iA,
	input [26:0] iB,
	output reg oA_larger
);
	// Extract fields of input floating-point numbers
	wire A_s;
	wire [7:0] A_e;
	wire [17:0] A_f;
	wire B_s;
	wire [7:0] B_e;
	wire [17:0] B_f;
    
	// Extract sign, exponent, and mantissa
	assign A_s = iA[26];
	assign A_e = iA[25:18];
	assign A_f = iA[17:0];
	assign B_s = iB[26];
	assign B_e = iB[25:18];
	assign B_f = iB[17:0];
	
	// Determine which of A, B is larger in magnitude
	wire A_mag_larger ;
	assign A_mag_larger = (A_e > B_e)                   ? 1'b1  :
                         ((A_e == B_e) && (A_f >= B_f)) ? 1'b1  :
                         1'b0;
	
	// Compare sign, exponent, and mantissa
	always @(*) begin
		if (A_s==0 && B_s==1) oA_larger = 1'b1 ;
		else if (A_s==1 && B_s==0) oA_larger = 1'b0 ;
		else if (A_s==0 && B_s==0) oA_larger = A_mag_larger ;
		else if (A_s==1 && B_s==1) oA_larger = ~A_mag_larger ;
		else oA_larger  = 0; // default value
	end
endmodule


/**************************************************************************
 * Fast Inverse Square Root                                                *
 * 5-stage pipeline, Wikipedia algorithm                                  *
 * Magic number 27'd49920718                                               *
 * 1.5 = 27'd33423360                                                     *
 *************************************************************************/
module FpInvSqrt (
    input             clk,    // Clock input
    input      [26:0] A,      // Input
    output reg [26:0] invSqrt // Output
);

    // Extracting sign, exponent, and fraction of A.
    wire        A_sign;
    wire [7:0]  A_exponent;
    wire [17:0] A_fraction;
    assign A_sign = A[26];
    assign A_exponent = A[25:18];
    assign A_fraction = A[17:0];

    // Stage 1
    wire [26:0] y_1, half_A_1, y_1_out;
    assign y_1 = 27'd49920718 - (A >> 1);
    assign half_A_1 = {A_sign, A_exponent-8'd1, A_fraction};
    FpMul s1_mult ( .iA(y_1), .iB(y_1), .oProd(y_1_out) );

    // Stage 2
    reg [26:0] y_2, mult_2_in, half_A_2;
    wire [26:0] y_2_out;
    FpMul s2_mult ( .iA(half_A_2), .iB(mult_2_in), .oProd(y_2_out) );

    // Stage 3
    reg [26:0] y_3, add_3_in;
    wire [26:0] y_3_out;
    FpAdd s3_add ( .clk(clk), .iA({~add_3_in[26], add_3_in[25:0]}), .iB(27'd33423360), .oSum(y_3_out) );

    // Stage 4
    reg [26:0] y_4;

    // Stage 5
    reg [26:0] y_5, mult_5_in;
    FpMul s5_mult ( .iA(y_5), .iB(mult_5_in), .oProd(invSqrt) );

    always @(posedge clk) begin
        // Pipeline stages
        y_2 <= y_1;
        mult_2_in <= y_1_out;
        half_A_2 <= half_A_1;
        y_3 <= y_2;
        add_3_in <= y_2_out;
        y_4 <= y_3;
        y_5 <= y_4;
        mult_5_in <= y_3_out;
    end
endmodule

/**************************************************************************
 * Floating Point Multiplier                                               *
 * Combinational                                                           *
 *************************************************************************/
module FpMul (
    input      [26:0] A,    // First input
    input      [26:0] B,    // Second input
    output reg [26:0] Prod  // Product
);

    // Extracting sign, exponent, and fraction of A and B.
    wire        A_sign, B_sign;
    wire [7:0]  A_exponent, B_exponent;
    wire [17:0] A_fraction, B_fraction;
    assign A_sign = A[26];
    assign A_exponent = A[25:18];
    assign A_fraction = {1'b1, A[17:1]};
    assign B_sign = B[26];
    assign B_exponent = B[25:18];
    assign B_fraction = {1'b1, B[17:1]};

    // Determining product sign.
    wire        Prod_sign;
    assign Prod_sign = A_sign ^ B_sign;

    // Multiplying the fractions of A and B.
    wire [35:0] pre_product_fraction;
    assign pre_product_fraction = A_fraction * B_fraction;

    // Adding exponents of A and B.
    wire [8:0]  pre_product_exponent;
    assign pre_product_exponent = A_exponent + B_exponent;

    // Adjusting exponent and fraction.
    wire [7:0]  Prod_exponent;
    wire [17:0] Prod_fraction;
    assign Prod_exponent = pre_product_fraction[35] ? (pre_product_exponent - 9'd126) : (pre_product_exponent - 9'd127);
    assign Prod_fraction = pre_product_fraction[35] ? pre_product_fraction[34:17] : pre_product_fraction[33:16];

    // Detecting underflow.
    wire        underflow;
    assign underflow = pre_product_exponent < 9'h80;

    // Determining the product.
    assign Prod = underflow        ? 27'b0 :
                 (B_exponent == 8'd0)    ? 27'b0 :
                 (A_exponent == 8'd0)    ? 27'b0 :
                 {Prod_sign, Prod_exponent, Prod_fraction};

endmodule

/**************************************************************************
 * Floating Point Adder                                                    *
 * 2-stage pipeline                                                        *
 *************************************************************************/
module FpAdd (
    input             clk,    // Clock input
    input      [26:0] A,      // First input
    input      [26:0] B,      // Second input
    output reg [26:0] Sum     // Sum
);

    // Extracting sign, exponent, and fraction of A and B.
    wire        A_sign, B_sign;
    wire [7:0]  A_exponent, B_exponent;
    wire [17:0] A_fraction, B_fraction;
    assign A_sign = A[26];
    assign A_exponent = A[25:18];
    assign A_fraction = {1'b1, A[17:1]};
    assign B_sign = B[26];
    assign B_exponent = B[25:18];
    assign B_fraction = {1'b1, B[17:1]};
    wire A_larger;

    // Shifting fractions of A and B to align.
    wire [7:0]  exp_diff_A, exp_diff_B, larger_exp;
    wire [36:0] A_shifted, B_shifted;
    assign exp_diff_A = B_exponent - A_exponent;
    assign exp_diff_B = A_exponent - B_exponent;
    assign larger_exp = (B_exponent > A_exponent) ? B_exponent : A_exponent;
    assign A_shifted = A_larger            ? {1'b0, A_fraction, 18'b0} :
                       (exp_diff_A > 9'd35) ? 37'b0 :
                       ({1'b0, A_fraction, 18'b0} >> exp_diff_A);
    assign B_shifted = ~A_larger           ? {1'b0, B_fraction, 18'b0} :
                       (exp_diff_B > 9'd35) ? 37'b0 :
                       ({1'b0, B_fraction, 18'b0} >> exp_diff_B);

    // Calculating sum or difference of shifted fractions.
    wire [36:0] pre_sum;
    assign pre_sum = ((A_sign ^ B_sign) &  A_larger) ? A_shifted - B_shifted :
                     ((A_sign ^ B_sign) & ~A_larger) ? B_shifted - A_shifted :
                     A_shifted + B_shifted;

    // Buffers for midway results.
    reg  [36:0] buf_pre_sum;
    reg  [7:0]  buf_larger_exp;
    reg         buf_A_e_zero, buf_B_e_zero;
    reg  [26:0] buf_A, buf_B;
    reg         buf_Sum_sign;
    always @(posedge clk) begin
        buf_pre_sum    <= pre_sum;
        buf_larger_exp <= larger_exp;
        buf_A_e_zero   <= (A_exponent == 8'b0);
        buf_B_e_zero   <= (B_exponent == 8'b0);
        buf_A          <= A;
        buf_B          <= B;
        buf_Sum_sign   <= A_larger ? A_sign : B_sign;
    end

    // Converting to positive fraction and a sign bit.
    wire [36:0] pre_fraction;
    assign pre_fraction = buf_pre_sum;

    // Determining output fraction and exponent change.
    wire [17:0] Sum_fraction;
    wire [7:0]  shift_amount;
    assign shift_amount = pre_fraction[36] ? 8'd0  : pre_fraction[35] ? 8'd1  :
                          pre_fraction[34] ? 8'd2  : pre_fraction[33] ? 8'd3  :
                          pre_fraction[32] ? 8'd4  : pre_fraction[31] ? 8'd5  :
                          pre_fraction[30] ? 8'd6  : pre_fraction[29] ? 8'd7  :
                          pre_fraction[28] ? 8'd8  : pre_fraction[27] ? 8'd9  :
                          pre_fraction[26] ? 8'd10 : pre_fraction[25] ? 8'd11 :
                          pre_fraction[24] ? 8'd12 : pre_fraction[23] ? 8'd13 :
                          pre_fraction[22] ? 8'd14 : pre_fraction[21] ? 8'd15 :
                          pre_fraction[20] ? 8'd16 : pre_fraction[19] ? 8'd17 :
                          pre_fraction[18] ? 8'd18 : pre_fraction[17] ? 8'd19 :
                          pre_fraction[16] ? 8'd20 : pre_fraction[15] ? 8'd21 :
                          pre_fraction[14] ? 8'd22 : pre_fraction[13] ? 8'd23 :
                          pre_fraction[12] ? 8'd24 : pre_fraction[11] ? 8'd25 :
                          pre_fraction[10] ? 8'd26 : pre_fraction[9]  ? 8'd27 :
                          pre_fraction[8]  ? 8'd28 : pre_fraction[7]  ? 8'd29 :
                          pre_fraction[6]  ? 8'd30 : pre_fraction[5]  ? 8'd31 :
                          pre_fraction[4]  ? 8'd32 : pre_fraction[3]  ? 8'd33 :
                          pre_fraction[2]  ? 8'd34 : pre_fraction[1]  ? 8'd35 :
                          pre_fraction[0]  ? 8'd36 : 8'd37;
    wire [53:0] pre_fraction_shifted, underflow_shift;
    assign pre_fraction_shifted = {pre_fraction, 17'b0} << (shift_amount + 1);
    assign underflow_shift = {pre_fraction, 17'b0} << shift_amount;
    assign Sum_fraction = pre_fraction_shifted[53:36];

    wire [7:0] Sum_exponent;
    assign Sum_exponent = buf_larger_exp - shift_amount + 8'b1;

    // Detecting underflow.
    wire underflow;
    assign underflow = ~underflow_shift[53];

    always @(posedge clk) begin
        Sum <= (buf_A_e_zero && buf_B_e_zero)    ? 27'b0 :
               buf_A_e_zero                     ? buf_B :
               buf_B_e_zero                     ? buf_A :
               underflow                        ? 27'b0 :
               (pre_fraction == 0)             ? 27'b0 :
               {buf_Sum_sign, Sum_exponent, Sum_fraction};
    end // Output update
endmodule


