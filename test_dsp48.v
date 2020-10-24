`timescale 1ns/100ps
module test_dsp48
#(
parameter C_DEVICE = "spartan6",
parameter C_AWIDTH = 30,
parameter C_BWIDTH = 18,
parameter C_DWIDTH = 25,
parameter C_AREG  = 2'b11 ,
parameter C_BREG  = 2'b11 ,
parameter C_CREG  = 1'b1  ,
parameter C_DREG  = 1'b1  ,
parameter C_ADREG = 1'b1  ,
parameter C_MREG  = 1'b1  ,
parameter C_PREG  = 1'b1  
)
(
input                  I_clk      ,
input                  I_rst      ,
input  [C_AWIDTH-1:0]  I_data_a   ,
input  [C_BWIDTH-1:0]  I_data_b   ,
input  [47:0]          I_data_c   ,
input  [C_DWIDTH-1:0]  I_data_d   ,
input  [47:0]          I_data_pc  ,
input  [3:0]           I_aluctl   ,
input  [4:0]           I_inmode   ,
input  [7:0]           I_opmode   ,
output [47:0]          O_data_p   ,
output [47:0]          O_data_pc  
); 

generate
if(C_DEVICE == "spartan6")
begin: dsp_s6

DSP48A1 
#(
.A0REG             (C_AREG[0]),           // First stage A input pipeline register (0/1)
.A1REG             (C_AREG[1]),           // Second stage A input pipeline register (0/1)
.B0REG             (C_BREG[0]),           // First stage B input pipeline register (0/1)
.B1REG             (C_BREG[1]),           // Second stage B input pipeline register (0/1)
.CARRYINREG        (0),                   // CARRYIN input pipeline register (0/1)
.CARRYINSEL        ("OPMODE5"),           // Specify carry-in source, "CARRYIN" or "OPMODE5" 
.CARRYOUTREG       (0),                   // CARRYOUT output pipeline register (0/1)
.CREG              (C_CREG),              // C input pipeline register (0/1)
.DREG              (C_DREG),              // D pre-adder input pipeline register (0/1)
.MREG              (C_MREG),              // M pipeline register (0/1)
.OPMODEREG         (1),                   // Enable=1/disable=0 OPMODE input pipeline registers
.PREG              (C_PREG),              // P output pipeline register (0/1)
.RSTTYPE           ("SYNC")               // Specify reset type, "SYNC" or "ASYNC" 
)
DSP48A1_inst 
(
// Cascade Ports: 18-bit (each) output: Ports to cascade from one DSP48 to another
.BCOUT             (),                    // 18-bit output: B port cascade output
.PCOUT             (O_data_pc),           // 48-bit output: P cascade output (if used, connect to PCIN of another DSP48A1)
// Data Ports: 1-bit (each) output: Data input and output ports
.CARRYOUT          (),                    // 1-bit output: carry output (if used, connect to CARRYIN pin of another DSP48A1)
.CARRYOUTF         (),                    // 1-bit output: fabric carry output
.M                 (),                    // 36-bit output: fabric multiplier data output
.P                 (O_data_p),            // 48-bit output: data output
// Cascade Ports: 48-bit (each) input: Ports to cascade from one DSP48 to another
.PCIN              (I_data_pc),           // 48-bit input: P cascade input (if used, connect to PCOUT of another DSP48A1)
// Control Input Ports: 1-bit (each) input: Clocking and operation mode
.CLK               (I_clk),               // 1-bit input: clock input
.OPMODE            (I_opmode),            // 8-bit input: operation mode input
// Data Ports: 18-bit (each) input: Data input and output ports
.A                 (I_data_a[17:0]),            // 18-bit input: A data input
.B                 (I_data_b),            // 18-bit input: B data input (connected to fabric or BCOUT of adjacent DSP48A1)
.C                 (I_data_c),            // 48-bit input: C data input
.CARRYIN           (1'b0),                // 1-bit input: carry input signal (if used, connect to CARRYOUT pin of another DSP48A1)
.D                 (I_data_d[17:0]),            // 18-bit input: B pre-adder data input
// Reset/Clock Enable Input Ports: 1-bit (each) input: Reset and enable input ports
.CEA               (1'b1),                // 1-bit input: active high clock enable input for A registers
.CEB               (1'b1),                // 1-bit input: active high clock enable input for B registers
.CEC               (1'b1),                // 1-bit input: active high clock enable input for C registers
.CECARRYIN         (1'b0),                // 1-bit input: active high clock enable input for CARRYIN registers
.CED               (1'b1),                // 1-bit input: active high clock enable input for D registers
.CEM               (1'b1),                // 1-bit input: active high clock enable input for multiplier registers
.CEOPMODE          (1'b1),                // 1-bit input: active high clock enable input for OPMODE registers
.CEP               (1'b1),                // 1-bit input: active high clock enable input for P registers
.RSTA              (I_rst),               // 1-bit input: reset input for A pipeline registers
.RSTB              (I_rst),               // 1-bit input: reset input for B pipeline registers
.RSTC              (I_rst),               // 1-bit input: reset input for C pipeline registers
.RSTCARRYIN        (I_rst),               // 1-bit input: reset input for CARRYIN pipeline registers
.RSTD              (I_rst),               // 1-bit input: reset input for D pipeline registers
.RSTM              (I_rst),               // 1-bit input: reset input for M pipeline registers
.RSTOPMODE         (I_rst),               // 1-bit input: reset input for OPMODE pipeline registers
.RSTP              (I_rst)                // 1-bit input: reset input for P pipeline registers
 );
end
else if(C_DEVICE == "virtex6")
begin:dsp_v6 

DSP48E1 
#(
// Feature Control Attributes: Data Path Selection
.A_INPUT               ("DIRECT"),               // Selects A input source, "DIRECT" (A port) or "CASCADE" (ACIN port)
.B_INPUT               ("DIRECT"),               // Selects B input source, "DIRECT" (B port) or "CASCADE" (BCIN port)
.USE_DPORT             ("TRUE"),                 // Select D port usage (TRUE or FALSE)
.USE_MULT              ("MULTIPLY"),             // Select multiplier usage ("MULTIPLY", "DYNAMIC", or "NONE")
// Pattern Detector Attributes: Pattern Detection Configuration
.AUTORESET_PATDET      ("NO_RESET"),             // "NO_RESET", "RESET_MATCH", "RESET_NOT_MATCH" 
.MASK                  (48'h3fffffffffff),       // 48-bit mask value for pattern detect (1=ignore)
.PATTERN               (48'h000000000000),       // 48-bit pattern match for pattern detect
.SEL_MASK              ("MASK"),                 // "C", "MASK", "ROUNDING_MODE1", "ROUNDING_MODE2" 
.SEL_PATTERN           ("PATTERN"),              // Select pattern value ("PATTERN" or "C")
.USE_PATTERN_DETECT    ("NO_PATDET"),            // Enable pattern detect ("PATDET" or "NO_PATDET")
// Register Control Attributes: Pipeline Register Configuration
.ACASCREG              (C_AREG),                 // Number of pipeline stages between A/ACIN and ACOUT (0, 1 or 2)
.ADREG                 (C_ADREG),                // Number of pipeline stages for pre-adder (0 or 1)
.ALUMODEREG            (0),                      // Number of pipeline stages for ALUMODE (0 or 1)
.AREG                  (C_AREG),                 // Number of pipeline stages for A (0, 1 or 2)
.BCASCREG              (C_BREG),                 // Number of pipeline stages between B/BCIN and BCOUT (0, 1 or 2)
.BREG                  (C_BREG),                 // Number of pipeline stages for B (0, 1 or 2)
.CARRYINREG            (0),                      // Number of pipeline stages for CARRYIN (0 or 1)
.CARRYINSELREG         (0),                      // Number of pipeline stages for CARRYINSEL (0 or 1)
.CREG                  (C_CREG),                 // Number of pipeline stages for C (0 or 1)
.DREG                  (C_DREG),                 // Number of pipeline stages for D (0 or 1)
.INMODEREG             (0),                      // Number of pipeline stages for INMODE (0 or 1)
.MREG                  (C_MREG),                 // Number of multiplier pipeline stages (0 or 1)
.OPMODEREG             (1),                      // Number of pipeline stages for OPMODE (0 or 1)
.PREG                  (C_PREG),                 // Number of pipeline stages for P (0 or 1)
.USE_SIMD              ("ONE48")                 // SIMD selection ("ONE48", "TWO24", "FOUR12")
)
DSP48E1_inst 
(
// Cascade: 30-bit (each) output: Cascade Ports
.ACOUT              (),           // 30-bit output: A port cascade output
.BCOUT              (),           // 18-bit output: B port cascade output
.CARRYCASCOUT       (),           // 1-bit output: Cascade carry output
.MULTSIGNOUT        (),           // 1-bit output: Multiplier sign cascade output
.PCOUT              (O_data_pc),                   // 48-bit output: Cascade output
// Control: 1-bit (each) output: Control Inputs/Status Bits
.OVERFLOW           (),           // 1-bit output: Overflow in add/acc output
.PATTERNBDETECT     (),           // 1-bit output: Pattern bar detect output
.PATTERNDETECT      (),           // 1-bit output: Pattern detect output
.UNDERFLOW          (),           // 1-bit output: Underflow in add/acc output
// Data: 4-bit (each) output: Data Ports
.CARRYOUT           (),           // 4-bit output: Carry output
.P                  (O_data_p),   // 48-bit output: Primary data output
// Cascade: 30-bit (each) input: Cascade Ports
.ACIN               (30'd0),      // 30-bit input: A cascade data input
.BCIN               (18'd0),      // 18-bit input: B cascade input
.CARRYCASCIN        (1'b0),       // 1-bit input: Cascade carry input
.MULTSIGNIN         (1'b0),       // 1-bit input: Multiplier sign input
.PCIN               (I_data_pc),  // 48-bit input: P cascade input
// Control: 4-bit (each) input: Control Inputs/Status Bits
.ALUMODE            (I_aluctl),   // 4-bit input: ALU control input
.CARRYINSEL         (3'd0),       // 3-bit input: Carry select input
.CEINMODE           (1'b0),       // 1-bit input: Clock enable input for INMODEREG
.CLK                (I_clk),      // 1-bit input: Clock input
.INMODE             (I_inmode),   // 5-bit input: INMODE control input
.OPMODE             (I_opmode[6:0]),   // 7-bit input: Operation mode input
.RSTINMODE          (1'b0),       // 1-bit input: Reset input for INMODEREG
// Data: 30-bit (each) input: Data Ports
.A                  (I_data_a),   // 30-bit input: A data input
.B                  (I_data_b),   // 18-bit input: B data input
.C                  (I_data_c),   // 48-bit input: C data input
.CARRYIN            (1'b0),       // 1-bit input: Carry input signal
.D                  (I_data_d),   // 25-bit input: D data input
// Reset/Clock Enable: 1-bit (each) input: Reset/Clock Enable Inputs
.CEA1               (1'b1),       // 1-bit input: Clock enable input for 1st stage AREG
.CEA2               (C_AREG==2),  // 1-bit input: Clock enable input for 2nd stage AREG
.CEAD               (1'b1),       // 1-bit input: Clock enable input for ADREG
.CEALUMODE          (1'b0),       // 1-bit input: Clock enable input for ALUMODERE
.CEB1               (1'b1),       // 1-bit input: Clock enable input for 1st stage BREG
.CEB2               (C_BREG==2),  // 1-bit input: Clock enable input for 2nd stage BREG
.CEC                (1'b1),       // 1-bit input: Clock enable input for CREG
.CECARRYIN          (1'b0),       // 1-bit input: Clock enable input for CARRYINREG
.CECTRL             (1'b1),       // 1-bit input: Clock enable input for OPMODEREG and CARRYINSELREG
.CED                (1'b1),       // 1-bit input: Clock enable input for DREG
.CEM                (1'b1),       // 1-bit input: Clock enable input for MREG
.CEP                (1'b1),       // 1-bit input: Clock enable input for PREG
.RSTA               (1'b0),       // 1-bit input: Reset input for AREG
.RSTALLCARRYIN      (1'b0),       // 1-bit input: Reset input for CARRYINREG
.RSTALUMODE         (1'b0),       // 1-bit input: Reset input for ALUMODEREG
.RSTB               (1'b0),       // 1-bit input: Reset input for BREG
.RSTC               (1'b0),       // 1-bit input: Reset input for CREG
.RSTCTRL            (1'b0),       // 1-bit input: Reset input for OPMODEREG and CARRYINSELREG
.RSTD               (1'b0),       // 1-bit input: Reset input for DREG and ADREG
.RSTM               (1'b0),       // 1-bit input: Reset input for MREG
.RSTP               (1'b0)        // 1-bit input: Reset input for PREG
);
end
endgenerate

endmodule