`define bodyNum 11
module Type(
    input [2-1:0] din,
    output reg [2-1:0] dout,
    input clk,
    input rst,
    input [4-1:0] nth,
    input ren,
    input wen
    );

    reg [2-1:0] type[0:`bodyNum-1];// 0: static 1: dynamic 2:pig

    always@(posedge clk) begin
        if(rst) begin
            type[0] <= 2'd0; //ground
            type[1] <= 2'd1; //bird
            type[2] <= 2'd2; //pig
            type[3] <= 2'd1;
            type[4] <= 2'd1;
            type[5] <= 2'd1;
            type[6] <= 2'd1;
            type[7] <= 2'd1;
            type[8] <= 2'd1;
            type[9] <= 2'd1;
            type[10] <= 2'd1;
        end
        else begin
            if (wen && ren == 1'b0) begin
                type[nth] <= din;
            end
            else begin
                type[nth] <= type[nth];
            end
        end
    end

    always @ ( * ) begin
      if (wen) begin
        dout = 2'd0;
      end
      else begin
        if (ren) begin
          dout = type[nth];
        end
        else begin
          dout = 2'd0;
        end
      end
    end

endmodule

module Alive(
    input din,
    output reg dout,
    input clk,
    input rst,
    input [3:0] nth,
    input ren,
    input wen
    );

    reg [0:`bodyNum-1] alive;// 0:dead 1:alive

    always@(posedge clk) begin
        if(rst) begin
            alive[0] <= 1'b1;
            alive[1] <= 1'b1;
            alive[2] <= 1'b1;
            alive[3] <= 1'b1;
            alive[4] <= 1'b1;
            alive[5] <= 1'b1;
            alive[6] <= 1'b1;
            alive[7] <= 1'b1;
            alive[8] <= 1'b1;
            alive[9] <= 1'b1;
            alive[10] <= 1'b1;
        end
        else begin
            if (wen && ren == 1'b0) begin
                alive[nth] <= din;
            end else begin
                alive[nth] <= alive[nth];
            end
        end
    end

    always @ ( * ) begin
      if (wen) begin
        dout <= 1'b0;
      end else begin
        if (ren) begin
          dout = alive[nth];
        end else begin
          dout = 1'b0;
        end
      end
    end


endmodule

module Position(
    input signed [18:0] pos_x_in,
    input signed [18:0] pos_y_in,
    output reg signed [18:0] pos_x_out,
    output reg signed [18:0] pos_y_out,
    input clk,
    input rst,
    input [3:0] nth,
    input ren,
    input wen
    );

    reg signed [18:0] posX[0:`bodyNum-1];
    reg signed [18:0] posY[0:`bodyNum-1];

    always@(posedge clk) begin
        if(rst) begin
            posX[0] <= {1'b0,10'd320,8'b0}; posY[0] <= {1'b0,10'd440,8'b0};//ground
            posX[1] <= {1'b0,10'd130,8'b0}; posY[1] <= {1'b0,10'd310,8'b0};
            posX[2] <= {1'b0,10'd450,8'b0}; posY[2] <= {1'b0,10'd390,8'b0};
            posX[3] <= {1'b0,10'd380,8'b0}; posY[3] <= {1'b0,10'd368,8'b0};
            posX[4] <= {1'b0,10'd420,8'b0}; posY[4] <= {1'b0,10'd368,8'b0};
            posX[5] <= {1'b0,10'd480,8'b0}; posY[5] <= {1'b0,10'd368,8'b0};
            posX[6] <= {1'b0,10'd450,8'b0}; posY[6] <= {1'b0,10'd332,8'b0};//horizontal
            posX[7] <= {1'b0,10'd435,8'b0}; posY[7] <= {1'b0,10'd296,8'b0};
            posX[8] <= {1'b0,10'd465,8'b0}; posY[8] <= {1'b0,10'd296,8'b0};
            posX[9] <= {1'b0,10'd450,8'b0}; posY[9] <= {1'b0,10'd260,8'b0};//horizontal
            posX[10] <= {1'b0,10'd0,8'b0}; posY[10] <= {1'b0,10'd0,8'b0};
        end
        else begin
            if (wen && ren == 1'b0) begin
                posX[nth] <= pos_x_in;
                posY[nth] <= pos_y_in;
            end
            else begin
                posX[nth] <= posX[nth];
                posY[nth] <= posY[nth];
            end
        end
    end

    always @ ( * ) begin
      if (wen) begin
        pos_x_out = {1'b0,10'd0,8'b0};
        pos_y_out = {1'b0,10'd0,8'b0};
      end else begin
        if (ren) begin
          pos_x_out = posX[nth];
          pos_y_out = posY[nth];
        end else begin
          pos_x_out = {1'b0,10'd0,8'b0};
          pos_y_out = {1'b0,10'd0,8'b0};
        end
      end
    end

endmodule

module Radian(
    input signed [10-1:0] din,
    output reg signed [10-1:0] dout,
    input clk,
    input rst,
    input [4-1:0] nth,
    input ren,
    input wen
    );
    // 1 signed bit, 2 int bit, 7 float bit
    reg signed [10-1:0] rad[0:`bodyNum-1];

    always@(posedge clk) begin
        if(rst) begin
            rad[0] <= {1'b0,2'b00,7'b000_0000};
            rad[1] <= {1'b0,2'b00,7'b000_0000};
            rad[2] <= {1'b0,2'b00,7'b000_0000};
            rad[3] <= {1'b0,2'b00,7'b000_0000};
            rad[4] <= {1'b0,2'b00,7'b000_0000};
            rad[5] <= {1'b0,2'b00,7'b000_0000};
            rad[6] <= {1'b0,2'b00,7'b000_0000};
            rad[7] <= {1'b0,2'b00,7'b000_0000};
            rad[8] <= {1'b0,2'b00,7'b000_0000};
            rad[9] <= {1'b0,2'b00,7'b000_0000};
            rad[10] <= {1'b0,2'b00,7'b000_0000};
        end
        else begin
            if (wen && ren == 1'b0) begin
                rad[nth] <= din;
            end
            else begin
                rad[nth] <= rad[nth];
            end
        end
    end

    always @ ( * ) begin
      if (wen) begin
        dout = {1'b0,2'b00,7'b000_0000};
      end else begin
        if (ren) begin
          dout = rad[nth];
        end else begin
          dout = {1'b0,2'b00,7'b000_0000};
        end
      end
    end

endmodule

module Vertices(
    input signed [18:0] pos_x_in,
    input signed [18:0] pos_y_in,
    output reg signed [18:0] pos_x_out,
    output reg signed [18:0] pos_y_out,
    input clk,
    input [3:0] nth,
    input [1:0] i,
    input ren,
    input wen
    );

    // Ex:
    //  0       1        ...
    // |0|1|2|3|4|5|6|7| ...

    reg signed [18:0] posX[0:4*`bodyNum-1];
    reg signed [18:0] posY[0:4*`bodyNum-1];

    always@(posedge clk) begin
        if (wen && ren == 1'b0) begin
            posX[4*nth + i] <= pos_x_in;
            posY[4*nth + i] <= pos_y_in;
        end
        else begin
            posX[4*nth + i] <= posX[4*nth + i];
            posY[4*nth + i] <= posY[4*nth + i];
        end
    end

    always @ ( * ) begin
      if (wen) begin
        pos_x_out = {1'b0,10'd0,8'b0};
        pos_y_out = {1'b0,10'd0,8'b0};
      end else begin
        if (ren) begin
          pos_x_out = posX[4*nth + i];
          pos_y_out = posY[4*nth + i];
        end else begin
          pos_x_out = {1'b0,10'd0,8'b0};
          pos_y_out = {1'b0,10'd0,8'b0};
        end
      end
    end

endmodule

//module Norm(
//    input signed [10-1:0] norm_x_in,
//    input signed [10-1:0] norm_y_in,
//    output reg signed [10-1:0] norm_x_out,
//    output reg signed [10-1:0] norm_y_out,
//    input clk,
//    input rst,
//    input [4-1:0] nth,
//    input [1:0] i,
//    input ren,
//    input wen
//    );

//    reg signed [10-1:0] normX[0:40-1];
//    reg signed [10-1:0] normY[0:40-1];

//    always@(posedge clk) begin
//        if (wen && ren == 1'b0) begin
//            normX[4*nth + i] <= norm_x_in;
//            normY[4*nth + i] <= norm_y_in;
//        end
//        else begin
//            normX[4*nth + i] <= normX[4*nth + i];
//            normY[4*nth + i] <= normY[4*nth + i];
//        end
//    end

//    always @ ( * ) begin
//      if (wen) begin
//        norm_x_out = {1'b0,1'b0,8'b0};
//        norm_y_out = {1'b0,1'b0,8'b0};
//      end else begin
//        if (ren) begin
//          norm_x_out = normX[4*nth + i];
//          norm_y_out = normY[4*nth + i];
//        end else begin
//          norm_x_out = {1'b0,1'b0,8'b0};
//          norm_y_out = {1'b0,1'b0,8'b0};
//        end
//      end
//    end

//endmodule
