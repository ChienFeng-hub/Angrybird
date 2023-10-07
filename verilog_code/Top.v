
module Top(
    input clk,
    input rst,
    input slave_ready,
    output reg master_ready,
    output signed [12-1:0] vertice_x,
    output signed [12-1:0] vertice_y,
    output [4-1:0] nth_body,
    output [2-1:0] i_vertex
//    inout PS2_CLK,
//    inout PS2_DATA
    );

    parameter size = 4'd11;
    parameter bodyNum = 4'd4;
    parameter INIT = 4'd0;
    parameter RENDER = 4'd1;
    parameter BROADPHASE = 4'd2;

    reg [2-1:0] typ[0:size-1];
    reg [0:size-1] alive;
    reg signed [18:0] posX[0:size-1];
    reg signed [18:0] posY[0:size-1];
    reg signed [10-1:0] rad[0:size-1];
    reg signed [18:0] verticesX[0:4*size-1];
    reg signed [18:0] verticesY[0:4*size-1];


    reg [4-1:0] state, next_state;
    wire reset, rst_op;
    wire clk25Mhz;
    assign rst_op = rst;
    // debounce db(.pb_debounced(reset), .pb(rst), .clk(clk));
    // onepulse op(.pb_debounced(reset), .clk(clk), .pb_1pulse(rst_op));
    clock_divisor(.clk1(clk25Mhz), .clk(clk));

    reg [4-1:0] nth, nth_prev;
    reg [1:0] i, cnt, i_prev;

    // ---------- Render ---------- //
    reg [19-1:0] ver_x_render, ver_y_render;

    always @ ( posedge clk25Mhz ) begin
      if (slave_ready) begin
        master_ready <= 1'b0;
      end else begin
        master_ready <= 1'b0;
        if (state == RENDER && nth < bodyNum) begin
          master_ready <= 1'b1;
        end
      end
    end

    always @ ( posedge clk25Mhz or posedge rst_op) begin
      if (rst_op) begin
        nth <= 4'b0;
        i <= 2'b0;
        cnt <= 2'b0;
        nth_prev <= 4'b0;
        i_prev <= 2'b0;
      end else begin
        nth <= nth;
        i <= i;
        cnt <= cnt;
        nth_prev <= nth_prev;
        i_prev <= i_prev;

        if (state == RENDER) begin
          if (master_ready) begin
            if (cnt < 1'd1) begin
              cnt <= cnt + 1'b1;
            end  if (cnt < 2'd2) begin
              cnt <= cnt + 1'b1;
              {nth, i} <= {nth, i} + 1'b1;
            end
            else begin
              {nth_prev, i_prev} <= {nth_prev, i_prev} + 1'b1;
              {nth, i} <= {nth, i} + 1'b1;
            end
          end
        end
        else if (state == BROADPHASE) begin
          cnt <= 0;
          nth <= 0;
          i <= 0;
          nth_prev <= 0;
          i_prev <= 0;
        end

      end
    end

    always @ (posedge clk25Mhz) begin
      if (state == INIT) begin
        typ[0] <= 2'd0; //ground
        typ[1] <= 2'd1; //bird
        typ[2] <= 2'd2; //pig
        typ[3] <= 2'd1;
        typ[4] <= 2'd1;
        typ[5] <= 2'd1;
        typ[6] <= 2'd1;
        typ[7] <= 2'd1;
        typ[8] <= 2'd1;
        typ[9] <= 2'd1;
        typ[10] <= 2'd1;

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

      end else if (state == RENDER) begin
        verticesX[4*nth_prev + i_prev] <= ver_x_render;
        verticesY[4*nth_prev + i_prev] <= ver_y_render;
      end else begin
        typ[nth] <= typ[nth];
        alive[nth] <= alive[nth];
        posX[nth] <= posX[nth];
        posY[nth] <= posY[nth];
        verticesX[4*nth + i] <= verticesX[4*nth + i];
        verticesY[4*nth + i] <= verticesY[4*nth + i];
        rad[nth] <= rad[nth];
      end
    end


    Render rr( //number inside #(___) which means how many objects need to caculate
      .render_ready(master_ready),
      .pos_x(posX[nth]),
      .pos_y(posY[nth]),
      .rad(rad[nth]),
      .i(i),
      .vertice_x(ver_x_render),
      .vertice_y(ver_y_render)
    );

    assign vertice_x = ver_x_render[18:7];
    assign vertice_y = ver_y_render[18:7];
    assign nth_body = nth;
    assign i_vertex = i;

    always@(posedge clk25Mhz or posedge rst_op)
        if(rst_op) begin
            state <= INIT;
        end
        else begin
            state <= next_state;
        end

    always@(*)
        case (state)
          INIT: begin
                    next_state = RENDER;
                end
          RENDER: begin
                    if(nth == bodyNum) begin
                        next_state = BROADPHASE;
                    end
                    else begin
                        next_state = state;
                    end
                  end
          BROADPHASE: begin
                        next_state = state;
                      end
          default: next_state = INIT;
        endcase

endmodule

module clock_divisor(clk1, clk);
  input clk;
  output clk1;
  // output clk22;
  reg [21:0] num;
  wire [21:0] next_num;

  always @(posedge clk) begin
    num <= next_num;
  end

  assign next_num = num + 1'b1;
  assign clk1 = num[1];
  // assign clk22 = num[21];
endmodule
