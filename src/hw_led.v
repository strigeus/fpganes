// Copyright (c) 2012-2013 Ludvig Strigeus
// This program is GPL Licensed. See COPYING for the full license.

module LedDriver(input clk,
                 input [31:0] value,
                 input [7:0] enable,
                 output reg [7:0] SSEG_CA,
                 output reg [7:0] SSEG_AN);
  reg [2:0] state;
  reg [3:0] digit;
  reg [16:0] timer;
  
  // Each digit should be displayed for 1 ms. 1 ms = ~20000 cycles.
  always @(posedge clk) begin
    timer <= timer + 1;
    if (timer == 20000) begin
      timer <= 0;
      state <= state + 1;
    end
    case (state)
    0: begin SSEG_AN <= enable[0] ? 8'b11111110 : 8'b11111111; digit = value[3:0];  end
    1: begin SSEG_AN <= enable[1] ? 8'b11111101 : 8'b11111111; digit = value[7:4];  end
    2: begin SSEG_AN <= enable[2] ? 8'b11111011 : 8'b11111111; digit = value[11:8]; end
    3: begin SSEG_AN <= enable[3] ? 8'b11110111 : 8'b11111111; digit = value[15:12];end
    4: begin SSEG_AN <= enable[4] ? 8'b11101111 : 8'b11111111; digit = value[19:16];end
    5: begin SSEG_AN <= enable[5] ? 8'b11011111 : 8'b11111111; digit = value[23:20];end
    6: begin SSEG_AN <= enable[6] ? 8'b10111111 : 8'b11111111; digit = value[27:24];end
    7: begin SSEG_AN <= enable[7] ? 8'b01111111 : 8'b11111111; digit = value[31:28];end
    endcase
    case (digit)
    0: SSEG_CA  <= 8'b11000000;
    1: SSEG_CA  <= 8'b11111001;
    2: SSEG_CA  <= 8'b10100100;
    3: SSEG_CA  <= 8'b10110000;
    4: SSEG_CA  <= 8'b10011001;
    5: SSEG_CA  <= 8'b10010010;
    6: SSEG_CA  <= 8'b10000010;
    7: SSEG_CA  <= 8'b11111000;
    8: SSEG_CA  <= 8'b10000000;
    9: SSEG_CA  <= 8'b10010000;
    10: SSEG_CA <= 8'b10001000; // A
    11: SSEG_CA <= 8'b10000011; // b
    12: SSEG_CA <= 8'b10100111; // c
    13: SSEG_CA <= 8'b10100001; // d
    14: SSEG_CA <= 8'b10000110; // E
    15: SSEG_CA <= 8'b10001110; // F
    endcase
  end
endmodule

