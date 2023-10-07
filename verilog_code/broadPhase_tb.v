`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/12/18 17:47:18
// Design Name: 
// Module Name: tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module tb;
    reg clk = 0;
    reg rst = 0;
    reg signed [18:0] A0_x, A0_y;
    reg signed [18:0] A1_x, A1_y;
    reg signed [18:0] A2_x, A2_y;
    reg signed [18:0] A3_x, A3_y;
    reg signed [18:0] B0_x, B0_y;
    reg signed [18:0] B1_x, B1_y;
    reg signed [18:0] B2_x, B2_y;
    reg signed [18:0] B3_x, B3_y;
    reg signed [18:0] Apos_x, Apos_y;
    reg signed [18:0] Bpos_x, Bpos_y;
//    reg signed [10-1:0] Anorm0_x, Anorm0_y;
//    reg signed [10-1:0] Anorm1_x, Anorm1_y;
//    reg signed [10-1:0] Anorm2_x, Anorm2_y;
//    reg signed [10-1:0] Anorm3_x, Anorm3_y;
//    wire signed [19-1:0] bestPenetration;
//    wire signed [10-1:0] bestNorm_x;
//    wire signed [10-1:0] bestNorm_y;
//    wire signed [2-1:0] bestIndex;
    
    reg [3:0] A_nth, B_nth;
    wire done;
    
    Collide test(
        clk,
        rst,
        A_nth,
        B_nth,
        A0_x, A0_y,
        A1_x, A1_y,
        A2_x, A2_y,
        A3_x, A3_y,
        B0_x, B0_y,
        B1_x, B1_y,
        B2_x, B2_y,
        B3_x, B3_y,
        Apos_x, Apos_y,
        Bpos_x, Bpos_y,
        done
      );
    
    always #1 clk = ~clk;
 
    
    initial begin
        $dumpfile("test.vcd");
        $dumpvars(0,test);
        #1
        A_nth = 4'd0;
        A0_x = {1'b0,10'd150,8'b0}; A0_y = {1'b0,10'd50,8'b0};
        A1_x = {1'b0,10'd50,8'b0}; A1_y = {1'b0,10'd50,8'b0};
        A2_x = {1'b0,10'd50,8'b0}; A2_y = {1'b0,10'd150,8'b0};
        A3_x = {1'b0,10'd150,8'b0}; A3_y = {1'b0,10'd150,8'b0};
        Apos_x = {1'b0,10'd100,8'b0}; Apos_y = {1'b0,10'd100,8'b0};
        B_nth = 4'd0;
        B0_x = {1'b0,10'd200,8'b0}; B0_y = {1'b0,10'd0,8'b0};
        B1_x = {1'b0,10'd100,8'b0}; B1_y = {1'b0,10'd0,8'b0};
        B2_x = {1'b0,10'd100,8'b0}; B2_y = {1'b0,10'd100,8'b0};
        B3_x = {1'b0,10'd200,8'b0}; B3_y = {1'b0,10'd100,8'b0};
        Bpos_x = {1'b0,10'd150,8'b0}; Bpos_y = {1'b0,10'd50,8'b0};
        
         rst = 1;
        #2 rst = 0;
        $display("Hello, iverilog");
        #80 $finish;
    end


endmodule
