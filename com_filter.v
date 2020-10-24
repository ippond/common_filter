`timescale 1ns/100ps
module com_filter
#(
parameter C_XILINX_DEVICE = "virtex6",
parameter C_DWIDTH        = 16       ,
parameter C_CWIDTH        = 16       ,
parameter C_CNUM          = 96       ,
parameter C_SYMMETRY      = 1        ,
parameter C_CYCLE_NUM     = 1        ,
parameter C_OUT_UPPBIT    = 32       
)
(
input                     I_clk     ,
input                     I_rst     ,
input      [C_DWIDTH-1:0] I_data    ,
input                     I_data_v  ,
input      [C_CWIDTH-1:0] I_coef    ,
input                     I_coef_v  ,
output reg [C_DWIDTH-1:0] O_data    ,
output reg                O_data_v
);

localparam C_DA_WIDTH = (C_XILINX_DEVICE == "spartan6") ? 18 : 30;
localparam C_DB_WIDTH = (C_XILINX_DEVICE == "spartan6") ? 18 : 18;
localparam C_DD_WIDTH = (C_XILINX_DEVICE == "spartan6") ? 18 : 25;
localparam C_DAD_EXT_WIDTH = C_DA_WIDTH - C_DWIDTH;
localparam C_DAC_EXT_WIDTH = C_DA_WIDTH - C_CWIDTH;
localparam C_DBD_EXT_WIDTH = C_DB_WIDTH - C_DWIDTH;
localparam C_DBC_EXT_WIDTH = C_DB_WIDTH - C_CWIDTH;
localparam C_DD_EXT_WIDTH = C_DD_WIDTH - C_DWIDTH;
localparam C_ALUCTL0 = 4'd0;
localparam C_DSPE1_OPMODE0_C_ADD_M = 8'b0011_0101;
localparam C_DSPA1_OPMODE0_C_ADD_M = 8'b0000_1101;
localparam C_DSPA1_OPMODE0_C_ADD_M_PREADD = 8'b0001_1101;
localparam C_DSPE1_INMODE0_PREADD = 5'b00101;
localparam C_DSPE1_INMODE0 = 5'b00000;
localparam C_DSPA1_OPMODE0_C_ADD_ZERO = 8'b0000_1100;
localparam C_DSPA1_OPMODE0_C_ADD_P = 8'b0000_1110;
localparam C_DSPE1_OPMODE0_C_ADD_ZERO = 8'b0011_0000;
localparam C_DSPE1_OPMODE0_C_ADD_P = 8'b0010_1100;

localparam C_DSP_NUM = (C_CNUM%C_CYCLE_NUM) ? (C_CNUM/C_CYCLE_NUM+1) : (C_CNUM/C_CYCLE_NUM);
localparam C_TOTAL_NUM = C_DSP_NUM*C_CYCLE_NUM;
localparam C_EXT_NUM = C_TOTAL_NUM - C_CNUM;
localparam C_ADDR_WIDTH = F_cal_width2(C_CYCLE_NUM);
localparam C_RD_OFFSET = 2**C_ADDR_WIDTH - C_CYCLE_NUM;
localparam C_RD_OFFSET2 = (C_CYCLE_NUM==2) ? 0 : 1;
localparam C_DELAY_OFFSET = (C_CYCLE_NUM==2) ? (C_TOTAL_NUM-1)*C_CYCLE_NUM+C_DSP_NUM-1+((C_DSP_NUM-1)*2)-C_DSP_NUM+3 : (C_TOTAL_NUM-1)*C_CYCLE_NUM+C_DSP_NUM-1+((C_DSP_NUM-1)*2)-(C_DSP_NUM-3);
localparam C_DVSLR_LEN = 2*C_CYCLE_NUM+C_DSP_NUM*2+6;
localparam C_DELAY_LEN = (C_TOTAL_NUM*C_CYCLE_NUM > C_DELAY_OFFSET) ? (C_TOTAL_NUM*C_CYCLE_NUM) : C_DELAY_OFFSET;
localparam C_DELAY_WIDTH = F_cal_width2(C_DELAY_LEN);
localparam C_AREG0   = (C_XILINX_DEVICE == "virtex6" && C_SYMMETRY == 1) ? 2'b01 : (C_XILINX_DEVICE == "virtex6" && C_SYMMETRY == 0) ? 2'b10 : 2'b11;
localparam C_BREG0   = (C_XILINX_DEVICE == "virtex6") ? 2'b10 : 2'b11;
localparam C_CREG0   = 1'b1;
localparam C_DREG0   = 1'b1;
localparam C_ADREG0  = (C_XILINX_DEVICE == "virtex6" && C_SYMMETRY == 1) ? 1'b1 : 1'b0;
localparam C_MREG0   = 1'b1;
localparam C_PREG0   = 1'b1;

reg [C_CWIDTH*C_TOTAL_NUM-1:0] S_coef_cas = 0;
reg [C_CWIDTH*C_TOTAL_NUM-1:0] S_coef_cas2 = 0;
reg [C_DWIDTH*C_TOTAL_NUM-1:0] S_data_cas = 0;
reg [C_DWIDTH-1:0] S_data_ram [C_DSP_NUM-1:0] [2**C_ADDR_WIDTH-1:0];
wire [7:0] S_opmode;
wire [4:0] S_inmode;
wire [3:0] S_aluctl;
reg  [C_CWIDTH-1:0] S_coef = 0;
reg  [C_EXT_NUM-1+1:0] S_coef_v_slr = 0; 
wire [C_DA_WIDTH-1:0] S_dsp_a [C_DSP_NUM-1:0];
wire [C_DB_WIDTH-1:0] S_dsp_b [C_DSP_NUM-1:0];
wire [C_DD_WIDTH-1:0] S_dsp_d [C_DSP_NUM-1:0]; 
wire [47:0] S_dsp_c [C_DSP_NUM:0];
wire [47:0] S_dsp_p [C_DSP_NUM-1:0];
reg [C_DVSLR_LEN-1:0] S_data_v_slr = 0;
reg [C_ADDR_WIDTH-1:0] S_dram_waddr [C_DSP_NUM-1:0];
reg [C_ADDR_WIDTH-1:0] S_dram_raddr [C_DSP_NUM-1:0];
reg [C_DWIDTH-1:0] S_dram_dout [C_DSP_NUM-1:0];
reg [C_DWIDTH-1:0] S_data = 0;
reg [C_CWIDTH*C_CYCLE_NUM-1:0] S_coef_cas_part [C_DSP_NUM-1:0];
reg [3:0] S_aluctl2 = 0;
reg [4:0] S_inmode2 = 0;
reg [7:0] S_opmode2 = 0;
wire [47:0] S_dsp_p2;
reg [3*C_DSP_NUM*C_DWIDTH-1:0] S_data_slr = 0;
reg [C_DELAY_WIDTH-1:0] S_delayram_waddr = 0;
reg [C_DELAY_WIDTH-1:0] S_delayram_raddr = 0;
reg [C_DWIDTH-1:0] S_delayram [2**C_DELAY_WIDTH-1:0];
reg [C_DWIDTH-1:0] S_data_delay = 0;
reg [C_DELAY_OFFSET:0] S_data_v_delay_slr = 0;
reg [C_DWIDTH-1:0] S_backdata_ram [C_DSP_NUM-1:0] [2**C_ADDR_WIDTH-1:0];
reg [C_ADDR_WIDTH-1:0] S_backram_waddr [C_DSP_NUM-1:0];
reg [C_ADDR_WIDTH-1:0] S_backram_raddr [C_DSP_NUM-1:0];
reg [C_DWIDTH-1:0] S_backram_dout [C_DSP_NUM-1:0];
integer S_i,S_j;

initial
begin
    for(S_i=0;S_i<C_DSP_NUM;S_i=S_i+1)
    for(S_j=0;S_j<2**C_ADDR_WIDTH;S_j=S_j+1)
    begin
    S_data_ram[S_i][S_j] = 0;
    end
    for(S_i=0;S_i<C_DSP_NUM;S_i=S_i+1)
    for(S_j=0;S_j<2**C_ADDR_WIDTH;S_j=S_j+1)
    begin
    S_backdata_ram[S_i][S_j] = 0;
    end
    for(S_i=0;S_i<C_DSP_NUM;S_i=S_i+1)
    begin
    S_dram_waddr[S_i] = 0;
    S_dram_raddr[S_i] = 0;
    S_coef_cas_part[S_i] = 0;
    S_dram_dout[S_i] = 0;
    S_backram_waddr[S_i] = 0;
    S_backram_raddr[S_i] = 0;
    S_backram_dout[S_i] = 0;
    end
end

always @(posedge I_clk)
begin
    if(I_coef_v)
        S_coef <= I_coef;
    else
        S_coef <= 'd0;
    S_coef_v_slr <= (C_CYCLE_NUM==1 || C_TOTAL_NUM == C_CNUM) ? I_coef_v : {S_coef_v_slr[C_EXT_NUM-1:0],I_coef_v};
    if(|S_coef_v_slr)
        S_coef_cas <= {S_coef,S_coef_cas[C_CWIDTH*C_TOTAL_NUM-1:C_CWIDTH]};
    S_data_v_slr <= {S_data_v_slr[C_DVSLR_LEN-2:0],I_data_v};
    S_data <= I_data;
    S_coef_cas2 <= S_coef_cas[C_CWIDTH*C_CNUM-1:0]<<((C_TOTAL_NUM-C_CNUM)*C_CWIDTH);
end

genvar gen_i;
generate
//---------------------------------
//non-symmetry continuous
//---------------------------------
if(C_SYMMETRY==0 && C_CYCLE_NUM==1)
begin:non_mirror_cont

always @(posedge I_clk)
begin
    S_data_cas <= {S_data_cas[C_DWIDTH*C_DSP_NUM-1-C_DWIDTH:0],I_data};
end

assign S_opmode = (C_XILINX_DEVICE == "virtex6") ? C_DSPE1_OPMODE0_C_ADD_M : C_DSPA1_OPMODE0_C_ADD_M;
assign S_inmode = C_DSPE1_INMODE0;
assign S_aluctl = C_ALUCTL0;
assign S_dsp_c[0] = 48'd0;

always @(posedge I_clk)
begin
    O_data <= S_dsp_p[C_DSP_NUM-1][C_OUT_UPPBIT-:C_DWIDTH];
    O_data_v <= S_data_v_slr[4+C_DSP_NUM-1];
end

for(gen_i=0;gen_i<C_DSP_NUM;gen_i=gen_i+1)
begin:data_allo

assign S_dsp_a[gen_i] = {{C_DAD_EXT_WIDTH{S_data_cas[gen_i*C_DWIDTH+C_DWIDTH-1]}},S_data_cas[gen_i*C_DWIDTH+:C_DWIDTH]};
assign S_dsp_b[gen_i] = {{C_DBC_EXT_WIDTH{S_coef_cas[(C_DSP_NUM-gen_i)*C_CWIDTH-1]}},S_coef_cas[(C_DSP_NUM-gen_i)*C_CWIDTH-1-:C_CWIDTH]};
assign S_dsp_c[gen_i+1] = S_dsp_p[gen_i];
assign S_dsp_d[gen_i] = {C_DD_WIDTH{1'b0}};
end

end
//---------------------------------
//non-symmetry discontinuous
//---------------------------------
else if(C_SYMMETRY==0 && C_CYCLE_NUM!=1)
begin:non_mirror_non_cont

localparam C_OUT_OFFSET = 2*C_CYCLE_NUM+1+4+C_DSP_NUM-3-C_CYCLE_NUM+(C_DSP_NUM+1);

assign S_opmode = (C_XILINX_DEVICE == "virtex6") ? C_DSPE1_OPMODE0_C_ADD_M : C_DSPA1_OPMODE0_C_ADD_M;
assign S_inmode = C_DSPE1_INMODE0;
assign S_aluctl = C_ALUCTL0;

always @(posedge I_clk)
begin
    if(S_data_v_slr[0])
        S_data_ram[0][S_dram_waddr[0]] <= S_data;
    if(I_rst)
        S_dram_waddr[0] <= 'd0;
    else if(S_data_v_slr[0])
        S_dram_waddr[0] <= S_dram_waddr[0] + 'd1;
    if(I_rst)
        S_dram_raddr[0] <= 'd0;
    else if(S_data_v_slr[C_CYCLE_NUM])
        S_dram_raddr[0] <= S_dram_waddr[0] + C_RD_OFFSET;
    else
        S_dram_raddr[0] <= S_dram_raddr[0] + 'd1;
    S_dram_dout[0] <= S_data_ram[0][S_dram_raddr[0]];
    S_data_cas[C_DWIDTH-1:0] <= S_data_ram[0][S_dram_raddr[0]];
    if(S_data_v_slr[C_CYCLE_NUM+1])
        S_coef_cas_part[0] <= S_coef_cas[0+:C_CWIDTH*C_CYCLE_NUM];
    else
        S_coef_cas_part[0] <= S_coef_cas_part[0]<<C_CWIDTH;
end

assign S_dsp_a[0] = {{C_DAD_EXT_WIDTH{S_data_cas[C_DWIDTH-1]}},S_data_cas[0+:C_DWIDTH]};
assign S_dsp_b[0] = {{C_DBC_EXT_WIDTH{S_coef_cas_part[0][C_CWIDTH*C_CYCLE_NUM-1]}},S_coef_cas_part[0][C_CWIDTH*C_CYCLE_NUM-1-:C_CWIDTH]};
assign S_dsp_c[0] = 48'd0;
assign S_dsp_d[0] = {C_DD_WIDTH{1'b0}};

for(gen_i=1;gen_i<C_DSP_NUM;gen_i=gen_i+1)
begin:data_allo2

always @(posedge I_clk)
begin
    if(S_data_v_slr[gen_i*2+C_CYCLE_NUM])
        S_data_ram[gen_i][S_dram_waddr[gen_i]] <= S_dram_dout[gen_i-1];
    if(I_rst)
        S_dram_waddr[gen_i] <= 'd0;
    else if(S_data_v_slr[gen_i*2+C_CYCLE_NUM])
        S_dram_waddr[gen_i] <= S_dram_waddr[gen_i] + 'd1;
    if(I_rst)
        S_dram_raddr[gen_i] <= 'd0;
    else if(S_data_v_slr[gen_i*2+C_CYCLE_NUM])
        S_dram_raddr[gen_i] <= S_dram_waddr[gen_i] + C_RD_OFFSET;
    else
        S_dram_raddr[gen_i] <= S_dram_raddr[gen_i] + 'd1;
    S_dram_dout[gen_i] <= S_data_ram[gen_i][S_dram_raddr[gen_i]];
    S_data_cas[gen_i*C_DWIDTH+:C_DWIDTH] <= S_data_ram[gen_i][S_dram_raddr[gen_i]];
    
    if(S_data_v_slr[gen_i*2+C_CYCLE_NUM+1])
        S_coef_cas_part[gen_i] <= S_coef_cas[C_CWIDTH*C_CYCLE_NUM*gen_i+:C_CWIDTH*C_CYCLE_NUM];
    else
        S_coef_cas_part[gen_i] <= S_coef_cas_part[gen_i]<<C_CWIDTH;
end

assign S_dsp_a[gen_i] = {{C_DAD_EXT_WIDTH{S_data_cas[gen_i*C_DWIDTH+C_DWIDTH-1]}},S_data_cas[gen_i*C_DWIDTH+:C_DWIDTH]};
assign S_dsp_b[gen_i] = {{C_DBC_EXT_WIDTH{S_coef_cas_part[gen_i][C_CYCLE_NUM*C_CWIDTH-1]}},S_coef_cas_part[gen_i][C_CYCLE_NUM*C_CWIDTH-1-:C_CWIDTH]};
assign S_dsp_c[gen_i] = S_dsp_p[gen_i-1];
assign S_dsp_d[gen_i] = {C_DD_WIDTH{1'b0}};

end

always @(posedge I_clk)
begin
    if(S_data_v_slr[C_OUT_OFFSET])
    begin
        S_opmode2 <= (C_XILINX_DEVICE == "virtex6") ? C_DSPE1_OPMODE0_C_ADD_ZERO : C_DSPA1_OPMODE0_C_ADD_ZERO;
    end
    else
    begin
        S_opmode2 <= (C_XILINX_DEVICE == "virtex6") ? C_DSPE1_OPMODE0_C_ADD_P : C_DSPA1_OPMODE0_C_ADD_P;
    end
    S_aluctl2 <= C_ALUCTL0;
    S_inmode2 <= C_DSPE1_INMODE0;
end

always @(posedge I_clk)
begin
    O_data <= S_dsp_p2[C_OUT_UPPBIT-:C_DWIDTH];
    O_data_v <= S_data_v_slr[C_OUT_OFFSET+C_CYCLE_NUM+2];
end

test_dsp48
#(
.C_DEVICE (C_XILINX_DEVICE),
.C_AWIDTH (C_DA_WIDTH     ),
.C_BWIDTH (C_DB_WIDTH     ),
.C_DWIDTH (C_DD_WIDTH     ),
.C_AREG   (C_AREG0        ),
.C_BREG   (C_BREG0        ),
.C_CREG   (C_CREG0        ),
.C_DREG   (C_DREG0        ),
.C_ADREG  (C_ADREG0       ),
.C_MREG   (C_MREG0        ),
.C_PREG   (C_PREG0        )
)
test_dsp48_inst1
(
.I_clk        (I_clk),
.I_rst        (1'b0),
.I_data_a     ({C_DA_WIDTH{1'b0}}),
.I_data_b     ({C_DB_WIDTH{1'b0}}),
.I_data_c     (S_dsp_p[C_DSP_NUM-1]),
.I_data_d     ({C_DD_WIDTH{1'b0}}),
.I_data_pc    (48'd0),
.I_aluctl     (S_aluctl2),
.I_inmode     (S_inmode2),
.I_opmode     (S_opmode2),
.O_data_p     (S_dsp_p2),
.O_data_pc    ()
); 
end
//---------------------------------
//symmetry continuous
//---------------------------------
else if(C_SYMMETRY==1 && C_CYCLE_NUM==1)
begin:mirror_cont

assign S_opmode = (C_XILINX_DEVICE == "virtex6") ? C_DSPE1_OPMODE0_C_ADD_M : C_DSPA1_OPMODE0_C_ADD_M_PREADD;
assign S_inmode = (C_XILINX_DEVICE == "virtex6") ? C_DSPE1_INMODE0_PREADD : C_DSPE1_INMODE0;
assign S_aluctl = C_ALUCTL0;

always @(posedge I_clk)
begin
    S_data_cas <= {S_data_cas[C_DWIDTH*C_DSP_NUM-1-C_DWIDTH:0],I_data};
    S_data_slr <= {S_data_slr[3*C_DSP_NUM*C_DWIDTH-C_DWIDTH-1:0],I_data};       
end

assign S_dsp_b[0] = (C_XILINX_DEVICE == "virtex6") ? {{C_DBC_EXT_WIDTH{S_coef_cas[C_TOTAL_NUM*C_CWIDTH-1]}},S_coef_cas[C_TOTAL_NUM*C_CWIDTH-1-:C_CWIDTH]} : {{C_DBD_EXT_WIDTH{S_data_cas[C_DWIDTH-1]}},S_data_cas[0+:C_DWIDTH]};
assign S_dsp_a[0] = (C_XILINX_DEVICE == "virtex6") ? {{C_DAD_EXT_WIDTH{S_data_cas[C_DWIDTH-1]}},S_data_cas[0+:C_DWIDTH]} : {{C_DAC_EXT_WIDTH{S_coef_cas[C_TOTAL_NUM*C_CWIDTH-1]}},S_coef_cas[C_TOTAL_NUM*C_CWIDTH-1-:C_CWIDTH]};
assign S_dsp_d[0] = {C_DD_EXT_WIDTH{1'b0}};
assign S_dsp_c[0] = 48'd0;

for(gen_i=1;gen_i<C_DSP_NUM;gen_i=gen_i+1)
begin:data_backwards

assign S_dsp_b[gen_i] = (C_XILINX_DEVICE == "virtex6") ? {{C_DBC_EXT_WIDTH{S_coef_cas[(C_TOTAL_NUM-gen_i)*C_CWIDTH-1]}},S_coef_cas[(C_TOTAL_NUM-gen_i)*C_CWIDTH-1-:C_CWIDTH]} : {{C_DBD_EXT_WIDTH{S_data_cas[gen_i*C_DWIDTH+C_DWIDTH-1]}},S_data_cas[gen_i*C_DWIDTH+:C_DWIDTH]};
assign S_dsp_a[gen_i] = (C_XILINX_DEVICE == "virtex6") ? {{C_DAD_EXT_WIDTH{S_data_cas[gen_i*C_DWIDTH+C_DWIDTH-1]}},S_data_cas[gen_i*C_DWIDTH+:C_DWIDTH]} : {{C_DAC_EXT_WIDTH{S_coef_cas[(C_TOTAL_NUM-gen_i)*C_CWIDTH-1]}},S_coef_cas[(C_TOTAL_NUM-gen_i)*C_CWIDTH-1-:C_CWIDTH]};
assign S_dsp_d[gen_i] = {{C_DD_EXT_WIDTH{S_data_slr[gen_i*3*C_DWIDTH+C_DWIDTH-1]}},S_data_slr[gen_i*3*C_DWIDTH+:C_DWIDTH]};
assign S_dsp_c[gen_i] = S_dsp_p[gen_i-1];

end

always @(posedge I_clk)
begin
    O_data <= S_dsp_p[C_DSP_NUM-1][C_OUT_UPPBIT-:C_DWIDTH];
    O_data_v <= S_data_v_slr[3+C_DSP_NUM];
end

end

//---------------------------------
//symmetry discontinuous
//---------------------------------
else if(C_SYMMETRY==1 && C_CYCLE_NUM!=1)
begin:mirror_non_cont

localparam C_OUT_OFFSET = 2*C_CYCLE_NUM+1+4+C_DSP_NUM-3-C_CYCLE_NUM+(C_DSP_NUM+1);

assign S_opmode = (C_XILINX_DEVICE == "virtex6") ? C_DSPE1_OPMODE0_C_ADD_M : C_DSPA1_OPMODE0_C_ADD_M_PREADD;
assign S_inmode = (C_XILINX_DEVICE == "virtex6") ? C_DSPE1_INMODE0_PREADD : C_DSPE1_INMODE0;
assign S_aluctl = C_ALUCTL0;

always @(posedge I_clk)
begin
    if(I_rst)
        S_delayram_waddr <= C_DELAY_OFFSET;
    else
        S_delayram_waddr <= S_delayram_waddr + 'd1;
    if(I_rst)
        S_delayram_raddr <= 'd0;
    else
        S_delayram_raddr <= S_delayram_raddr + 'd1;
    S_delayram[S_delayram_waddr] <= I_data;
    S_data_delay <= S_delayram[S_delayram_raddr];
    S_data_v_delay_slr <= {S_data_v_delay_slr[C_DELAY_OFFSET-1:0],I_data_v};
end

always @(posedge I_clk)
begin
    if(S_data_v_slr[0])
        S_data_ram[0][S_dram_waddr[0]] <= S_data;
    if(I_rst)
        S_dram_waddr[0] <= 'd0;
    else if(S_data_v_slr[0])
        S_dram_waddr[0] <= S_dram_waddr[0] + 'd1;
    if(I_rst)
        S_dram_raddr[0] <= 'd0;
    else if(S_data_v_slr[C_CYCLE_NUM])
        S_dram_raddr[0] <= S_dram_waddr[0] + C_RD_OFFSET;
    else
        S_dram_raddr[0] <= S_dram_raddr[0] + 'd1;
    
    S_dram_dout[0] <= S_data_ram[0][S_dram_raddr[0]];
    S_data_cas[C_DWIDTH-1:0] <= S_data_ram[0][S_dram_raddr[0]];
    if(S_data_v_slr[C_CYCLE_NUM+1])
        S_coef_cas_part[0] <= S_coef_cas2[0+:C_CWIDTH*C_CYCLE_NUM];
    else
        S_coef_cas_part[0] <= S_coef_cas_part[0]<<C_CWIDTH;
end

for(gen_i=1;gen_i<C_DSP_NUM;gen_i=gen_i+1)
begin:data_allo2

always @(posedge I_clk)
begin
    if(S_data_v_slr[gen_i*2+C_CYCLE_NUM])
        S_data_ram[gen_i][S_dram_waddr[gen_i]] <= S_dram_dout[gen_i-1];
    if(I_rst)
        S_dram_waddr[gen_i] <= 'd0;
    else if(S_data_v_slr[gen_i*2+C_CYCLE_NUM])
        S_dram_waddr[gen_i] <= S_dram_waddr[gen_i] + 'd1;
    if(I_rst)
        S_dram_raddr[gen_i] <= 'd0;
    else if(S_data_v_slr[gen_i*2+C_CYCLE_NUM])
        S_dram_raddr[gen_i] <= S_dram_waddr[gen_i] + C_RD_OFFSET;
    else
        S_dram_raddr[gen_i] <= S_dram_raddr[gen_i] + 'd1;
    
    S_dram_dout[gen_i] <= (gen_i==C_DSP_NUM-1 && S_data_v_slr[gen_i*2+C_CYCLE_NUM+1]) ? 'd0 : S_data_ram[gen_i][S_dram_raddr[gen_i]];
    S_data_cas[gen_i*C_DWIDTH+:C_DWIDTH] <= S_data_ram[gen_i][S_dram_raddr[gen_i]];
    
    if(S_data_v_slr[gen_i*2+C_CYCLE_NUM+1])
        S_coef_cas_part[gen_i] <= S_coef_cas2[gen_i*C_CWIDTH*C_CYCLE_NUM+:C_CWIDTH*C_CYCLE_NUM];
    else
        S_coef_cas_part[gen_i] <= S_coef_cas_part[gen_i]<<C_CWIDTH;
end
//back
always @(posedge I_clk)
begin
    if(S_data_v_delay_slr[C_DELAY_OFFSET-gen_i*2])
        S_backdata_ram[gen_i][S_backram_waddr[gen_i]] <= S_backram_dout[gen_i-1];
    if(I_rst)
        S_backram_waddr[gen_i] <= 'd0;
    else if(S_data_v_delay_slr[C_DELAY_OFFSET-gen_i*2])
        S_backram_waddr[gen_i] <= S_backram_waddr[gen_i] + 'd1;
    if(I_rst)
        S_backram_raddr[gen_i] <= 'd0;
    else if(S_data_v_delay_slr[C_DELAY_OFFSET-gen_i*2-1+1] && (C_CYCLE_NUM==2))
        S_backram_raddr[gen_i] <= S_backram_waddr[gen_i] - C_RD_OFFSET2;
    else if(S_data_v_delay_slr[C_DELAY_OFFSET-gen_i*2-1-1] && (C_CYCLE_NUM!=2))
        S_backram_raddr[gen_i] <= S_backram_waddr[gen_i] - C_RD_OFFSET2;
    else
        S_backram_raddr[gen_i] <= S_backram_raddr[gen_i] - 'd1;
    S_backram_dout[gen_i] <= S_backdata_ram[gen_i][S_backram_raddr[gen_i]];
end

assign S_dsp_b[gen_i] = (C_XILINX_DEVICE == "virtex6") ? {{C_DBC_EXT_WIDTH{S_coef_cas_part[gen_i][C_CWIDTH*C_CYCLE_NUM-1]}},S_coef_cas_part[gen_i][C_CWIDTH*C_CYCLE_NUM-1-:C_CWIDTH]} : {{C_DBD_EXT_WIDTH{S_dram_dout[gen_i][C_DWIDTH-1]}},S_dram_dout[gen_i][C_DWIDTH-1:0]};
assign S_dsp_a[gen_i] = (C_XILINX_DEVICE == "virtex6") ? {{C_DAD_EXT_WIDTH{S_dram_dout[gen_i][C_DWIDTH-1]}},S_dram_dout[gen_i][C_DWIDTH-1:0]} : {{C_DAC_EXT_WIDTH{S_coef_cas_part[gen_i][C_CWIDTH*C_CYCLE_NUM-1]}},S_coef_cas_part[gen_i][C_CWIDTH*C_CYCLE_NUM-1-:C_CWIDTH]};
assign S_dsp_c[gen_i] = S_dsp_p[gen_i-1];
assign S_dsp_d[gen_i] = {{C_DD_EXT_WIDTH{S_backram_dout[C_DSP_NUM-1-gen_i][C_DWIDTH-1]}},S_backram_dout[C_DSP_NUM-1-gen_i][C_DWIDTH-1:0]};

end
//back
always @(posedge I_clk)
begin
    if(S_data_v_delay_slr[C_DELAY_OFFSET])
        S_backdata_ram[0][S_backram_waddr[0]] <= S_data_delay;
    if(I_rst)
        S_backram_waddr[0] <= 'd0;
    else if(S_data_v_delay_slr[C_DELAY_OFFSET])
        S_backram_waddr[0] <= S_backram_waddr[0] + 'd1;
    if(I_rst)
        S_backram_raddr[0] <= 'd0;
    else if(S_data_v_delay_slr[C_DELAY_OFFSET-1+1] && (C_CYCLE_NUM==2))
        S_backram_raddr[0] <= S_backram_waddr[0] - C_RD_OFFSET2;
    else if(S_data_v_delay_slr[C_DELAY_OFFSET-1-1] && (C_CYCLE_NUM!=2))
        S_backram_raddr[0] <= S_backram_waddr[0] - C_RD_OFFSET2;
    else
        S_backram_raddr[0] <= S_backram_raddr[0] - 'd1;
    S_backram_dout[0] <= S_backdata_ram[0][S_backram_raddr[0]];
end

assign S_dsp_b[0] = (C_XILINX_DEVICE == "virtex6") ? {{C_DBC_EXT_WIDTH{S_coef_cas_part[0][C_CWIDTH*C_CYCLE_NUM-1]}},S_coef_cas_part[0][C_CWIDTH*C_CYCLE_NUM-1-:C_CWIDTH]} : {{C_DBD_EXT_WIDTH{S_dram_dout[0][C_DWIDTH-1]}},S_dram_dout[0][C_DWIDTH-1:0]};
assign S_dsp_a[0] = (C_XILINX_DEVICE == "virtex6") ? {{C_DAD_EXT_WIDTH{S_dram_dout[0][C_DWIDTH-1]}},S_dram_dout[0][C_DWIDTH-1:0]} : {{C_DAC_EXT_WIDTH{S_coef_cas_part[0][C_CWIDTH*C_CYCLE_NUM-1]}},S_coef_cas_part[0][C_CWIDTH*C_CYCLE_NUM-1-:C_CWIDTH]};
assign S_dsp_c[0] = 48'd0;
assign S_dsp_d[0] = {{C_DD_EXT_WIDTH{S_backram_dout[C_DSP_NUM-1][C_DWIDTH-1]}},S_backram_dout[C_DSP_NUM-1][C_DWIDTH-1:0]};

always @(posedge I_clk)
begin
    if(S_data_v_slr[C_OUT_OFFSET])
    begin
        S_opmode2 <= (C_XILINX_DEVICE == "virtex6") ? C_DSPE1_OPMODE0_C_ADD_ZERO : C_DSPA1_OPMODE0_C_ADD_ZERO;
    end
    else
    begin
        S_opmode2 <= (C_XILINX_DEVICE == "virtex6") ? C_DSPE1_OPMODE0_C_ADD_P : C_DSPA1_OPMODE0_C_ADD_P;
    end
    S_aluctl2 <= C_ALUCTL0;
    S_inmode2 <= C_DSPE1_INMODE0;
end

always @(posedge I_clk)
begin
    O_data <= S_dsp_p2[C_OUT_UPPBIT-:C_DWIDTH];
    O_data_v <= S_data_v_slr[C_OUT_OFFSET+C_CYCLE_NUM+2];
end

test_dsp48
#(
.C_DEVICE (C_XILINX_DEVICE),
.C_AWIDTH (C_DA_WIDTH     ),
.C_BWIDTH (C_DB_WIDTH     ),
.C_DWIDTH (C_DD_WIDTH     ),
.C_AREG   (C_AREG0        ),
.C_BREG   (C_BREG0        ),
.C_CREG   (C_CREG0        ),
.C_DREG   (C_DREG0        ),
.C_ADREG  (C_ADREG0       ),
.C_MREG   (C_MREG0        ),
.C_PREG   (C_PREG0        )
)
test_dsp48_inst2
(
.I_clk        (I_clk),
.I_rst        (1'b0),
.I_data_a     ({C_DA_WIDTH{1'b0}}),
.I_data_b     ({C_DB_WIDTH{1'b0}}),
.I_data_c     (S_dsp_p[C_DSP_NUM-1]),
.I_data_d     ({C_DD_WIDTH{1'b0}}),
.I_data_pc    (48'd0),
.I_aluctl     (S_aluctl2),
.I_inmode     (S_inmode2),
.I_opmode     (S_opmode2),
.O_data_p     (S_dsp_p2),
.O_data_pc    ()
); 

end

endgenerate

genvar gen_k;
generate
for(gen_k=0;gen_k<C_DSP_NUM;gen_k=gen_k+1)
begin:dsp_cal

test_dsp48
#(
.C_DEVICE (C_XILINX_DEVICE),
.C_AWIDTH (C_DA_WIDTH     ),
.C_BWIDTH (C_DB_WIDTH     ),
.C_DWIDTH (C_DD_WIDTH     ),
.C_AREG   (C_AREG0        ),
.C_BREG   (C_BREG0        ),
.C_CREG   (C_CREG0        ),
.C_DREG   (C_DREG0        ),
.C_ADREG  (C_ADREG0       ),
.C_MREG   (C_MREG0        ),
.C_PREG   (C_PREG0        )
)
test_dsp48_inst
(
.I_clk        (I_clk),
.I_rst        (1'b0),
.I_data_a     (S_dsp_a[gen_k]),
.I_data_b     (S_dsp_b[gen_k]),
.I_data_c     (S_dsp_c[gen_k]),
.I_data_d     (S_dsp_d[gen_k]),
.I_data_pc    (48'd0),
.I_aluctl     (S_aluctl),
.I_inmode     (S_inmode),
.I_opmode     (S_opmode),
.O_data_p     (S_dsp_p[gen_k]),
.O_data_pc    ()
); 

end
endgenerate


function integer F_cal_width;
input integer I_data;
integer i;
begin
    for(i=1;(2**i)<I_data;i=i+1)
    F_cal_width = i;
    F_cal_width = i;
end
endfunction

function integer F_cal_width2;
input integer I_data;
integer i;
begin
    for(i=1;(2**i)<=I_data;i=i+1)
    F_cal_width2 = i;
    F_cal_width2 = i;
end
endfunction

endmodule

