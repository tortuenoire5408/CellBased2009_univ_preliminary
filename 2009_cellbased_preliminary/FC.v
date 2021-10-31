`timescale 1ns/100ps
module FC(clk, rst, cmd, done, M_RW, M_A, M_D, F_IO, F_CLE, F_ALE, F_REN, F_WEN, F_RB);
input clk;
input rst;
input [32:0] cmd;
output done;
output M_RW;
output [6:0] M_A;
inout  [7:0] M_D;
inout  [7:0] F_IO;
output F_CLE;
output F_ALE;
output F_REN;
output F_WEN;
input  F_RB;
//---------------------------------------------------------------------------
reg done;
reg M_RW;
reg [6:0] M_A;

reg F_CLE;
reg F_ALE;
reg F_REN;
reg F_WEN;
//---------------------------------------------------------------------------
reg F_RW;
reg [3:0] state, next_state, f_state, f_next_state;
reg [7:0] f_io_data, m_d_data;
reg [7:0] mem [127:0];
reg [6:0] cnt_M_LEN, cnt_F_LEN;
reg [32:0] cmd_reg;
//---------------------------------------------------------------------------
wire CMD_RW = cmd_reg[32];
wire [17:0] CMD_F_ADDR = cmd_reg[31:14];
wire [6:0] CMD_M_ADDR = cmd_reg[13:7];
wire [6:0] CMD_LEN = cmd_reg[6:0];
//---------------------------------------------------------------------------
assign M_D = (!M_RW) ? m_d_data : 8'bz;
assign F_IO = (!F_RW) ? f_io_data : 8'bz;
//---------------------------------------------------------------------------
parameter RST = 4'd0,
          RST_F = 4'd1,
          GET_CMD = 4'd2,
          WAIT_CMD = 4'd3,
          READ_M = 4'd4,
          WRITE_M = 4'd5,
          READ_F = 4'd6,
          WRITE_F = 4'd7,
          PEND = 4'd8,
          DONE = 4'd9;
parameter WRITE_CFF = 4'd0,
          WRITE_CHP = 4'd1,
          WRITE_C80 = 4'd2,
          WRITE_C10 = 4'd3,
          WRITE_A0 = 4'd4,
          WRITE_A1 = 4'd5,
          WRITE_A2 = 4'd6,
          WRITE_D = 4'd7,
          READ_D = 4'd8,
          BUFF_TR = 4'd9,
          F_PEND = 4'd10,
          F_DONE = 4'd11;
//---------------------------------------------------------------------------
reg clk_div;
//---------------------------------------------------------------------------

// clk division
always@(posedge clk or posedge rst) begin
  if(rst) clk_div <= 1'b0;
  else clk_div <= ~clk_div;
end

// current state
always@(posedge clk or posedge rst) begin
  if(rst) state <= RST;
  else state <= next_state;
end

// flash current state
always@(posedge clk_div or posedge rst) begin
  if(rst) f_state <= RST;
  else f_state <= f_next_state;
end

// next state
always@(*) begin
  case(state)
    RST: next_state = RST_F;
    RST_F: begin
      if(f_state == F_PEND) next_state = WAIT_CMD;
      else next_state = RST_F;
    end
    WAIT_CMD: next_state = GET_CMD;
    GET_CMD: next_state = (CMD_RW) ? READ_F : READ_M;
    READ_M: next_state = (cnt_M_LEN == CMD_LEN) ? PEND : READ_M;
    READ_F: begin
      if(f_state == F_DONE) next_state = PEND;
      else next_state = READ_F;
    end
    WRITE_M: next_state = (cnt_M_LEN == (CMD_LEN - 7'd1)) ? DONE : WRITE_M;
    WRITE_F: begin
      if(f_state == F_DONE) next_state = DONE;
      else next_state = WRITE_F;
    end
    PEND: next_state = (CMD_RW) ? WRITE_M : WRITE_F;
    DONE: begin
      if(F_RB) next_state = RST_F;
      else next_state = DONE;
    end
    default: next_state = RST_F;
  endcase
end

// flash next state
always@(*) begin
  case (state)
    RST_F: f_next_state = F_PEND;
    WAIT_CMD: f_next_state = F_PEND;
    GET_CMD: begin
      if(CMD_RW) f_next_state = WRITE_CHP;
    end
    READ_M: f_next_state = F_PEND;
    READ_F: begin
      case (f_state)
        WRITE_CHP: f_next_state = WRITE_A0;
        WRITE_A0: f_next_state = WRITE_A1;
        WRITE_A1: f_next_state = WRITE_A2;
        WRITE_A2: f_next_state = READ_D;
        READ_D: begin
          f_next_state = (cnt_F_LEN == CMD_LEN) ? F_DONE : READ_D;
        end
        F_DONE: begin
          if(F_RB) f_next_state = F_PEND;
          else f_next_state = F_DONE;
        end
        F_PEND: f_next_state = WRITE_CHP;
        default: f_next_state = F_PEND;
      endcase
    end
    WRITE_M: f_next_state = F_PEND;
    WRITE_F: begin
      case (f_state)
        WRITE_CHP: f_next_state = WRITE_C80;
        WRITE_C80: f_next_state = WRITE_A0;
        WRITE_A0: f_next_state = WRITE_A1;
        WRITE_A1: f_next_state = WRITE_A2;
        WRITE_A2: f_next_state = WRITE_D;
        WRITE_C10: f_next_state = F_DONE;
        WRITE_D: begin
          f_next_state = (cnt_F_LEN == (CMD_LEN - 7'd1)) ? WRITE_C10 : WRITE_D;
        end
        F_DONE: begin
          if(F_RB) f_next_state = F_PEND;
          else f_next_state = F_DONE;
        end
        F_PEND: f_next_state = WRITE_CHP;
        default: f_next_state = F_PEND;
      endcase
    end
    PEND: begin
      if(!CMD_RW) f_next_state = (CMD_F_ADDR[8]) ? WRITE_CHP : WRITE_C80;
    end
    default: f_next_state = F_PEND;
  endcase
end

//==========================================================================

// done
always@(*) begin
  if(state == RST) done = 1'd0;
  else begin
    if(!clk && !cmd_reg && state == WAIT_CMD) done = 1'd1;
    else if(state == DONE && F_RB) done = 1'd1;
    else done = 1'd0;
  end
end

//cmd_reg
always@(posedge clk or posedge rst) begin
  if(state == RST) cmd_reg <= 33'b0;
  else if(cmd) cmd_reg <= cmd;
end

// read data
integer i;
always@(posedge clk or posedge rst) begin
  if(state == RST || state == RST_F) begin
    for(i = 0; i <= CMD_LEN; i = i + 1) begin
      mem[i] <= 8'd0;
    end
  end else if(state == READ_M) begin
      if(F_RB && M_D>= 0) mem[cnt_M_LEN - 7'd1] <= M_D;
  end else if(state == READ_F) begin
      if(F_RB && F_IO>= 0) mem[cnt_F_LEN - 7'd1] <= F_IO;
  end
end

//Internal Memory=============================================================

//M_RW
always@(*) begin
  if(rst) M_RW = 1'd1;
  else if(state == READ_M) M_RW = 1'd1;
  else if(state == WRITE_M) M_RW = 1'd0;
  else M_RW = 1'd1;
end

//M_A
always@(*) begin
  if(rst) M_A = 0;
  else if(state == READ_M || state == WRITE_M) begin
    M_A = CMD_M_ADDR + cnt_M_LEN;
  end else M_A = 0;
end

//M_D
always@(*) begin
  if(rst) m_d_data = 0;
  else if(state == WRITE_M) begin
    m_d_data = mem[cnt_M_LEN];
  end else m_d_data = 0;
end

//cnt_M_LEN (count mem addr)
always@(posedge clk or posedge rst) begin
  if(rst) cnt_M_LEN <= 0;
  else if(state == READ_M || state == WRITE_M) begin
    if(cnt_M_LEN < CMD_LEN) cnt_M_LEN <= cnt_M_LEN + 7'd1;
  end if(state == GET_CMD || state == PEND) begin
    cnt_M_LEN <= 0;
  end
end

//Flash Memory================================================================

//F_RW
always@(*) begin
  if(state == RST) F_RW = 1'd1;
  else if(state == RST_F) F_RW = 1'd0;
  else if(f_state <= 4'd7) F_RW = 1'd0;
  else F_RW = 1'd1;
end

// F OUT DATA
always@(*) begin
  if(state == RST_F) f_io_data = 8'hFF;
  else  begin
    if(f_state == WRITE_CHP) f_io_data = CMD_F_ADDR[8] ? 8'h01 : 8'h00;
    else if(f_state == WRITE_C80) f_io_data = 8'h80;
    else if(f_state == WRITE_C10) f_io_data = 8'h10;
    else if(f_state == WRITE_A0) f_io_data = CMD_F_ADDR[7:0];
    else if(f_state == WRITE_A1) f_io_data = CMD_F_ADDR[16:9];
    else if(f_state == WRITE_A2) f_io_data = {7'd0, CMD_F_ADDR[17]};
    else if(f_state == WRITE_D) f_io_data = mem[cnt_F_LEN];
    else f_io_data = 8'hFF;
  end
end

//F_CLE
always@(*) begin
  if(state == RST) F_CLE = 1'd0;
  else if(state == RST_F) F_CLE = 1'd1;
  else if(f_state == WRITE_CHP ||
          f_state == WRITE_C80 ||
          f_state == WRITE_C10) F_CLE = 1'd1;
  else F_CLE = 1'd0;
end

//F_ALE
always@(*) begin
  if(state == RST) F_ALE = 1'd0;
  else if(f_state == WRITE_A0 ||
          f_state == WRITE_A1 ||
          f_state == WRITE_A2) F_ALE = 1'd1;
  else F_ALE = 1'd0;
end

//F_WEN
always@(*) begin
  if(state == RST) F_WEN = 1'd1;
  else if(f_state <= 4'd7) F_WEN = ~clk_div;
  else F_WEN = 1'd1;
end

//F_REN
always@(*) begin
  if(state == RST) F_REN = 1'd1;
  else if(f_state == READ_D) F_REN = ~clk_div;
  else F_REN = 1'd1;
end

//cnt_F_LEN (count flash addr)
always@(posedge clk_div or posedge rst) begin
  if(rst) cnt_F_LEN <= 0;
  else if(state == READ_F && f_state == READ_D) begin
    if(cnt_F_LEN < CMD_LEN && F_RB) cnt_F_LEN <= cnt_F_LEN + 7'd1;
  end else if(state == WRITE_F && f_state == WRITE_D) begin
    if(cnt_F_LEN < CMD_LEN && F_RB) cnt_F_LEN <= cnt_F_LEN + 7'd1;
  end if(state == GET_CMD || state == PEND) begin
    cnt_F_LEN <= 0;
  end
end

//---------------------------------------------------------------------------
endmodule