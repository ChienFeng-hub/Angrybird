
module Render(
    input render_ready,
    input [19-1:0] pos_x,
    input [19-1:0] pos_y,
    input [10-1:0] rad,
    input [4-1:0] nth,
    input [2-1:0] i,
    output data_ready,
    output reg signed [19-1:0] vertice_x,
    output reg signed [19-1:0] vertice_y
    );
    
    parameter bird_pig = 11'd32;

    parameter w_wood = 11'd8;
    parameter h_wood = 11'd64;//{1'b0,10'd64,8'b0};
    parameter w_floor = 11'd1023;
    parameter h_floor = 11'd64;
    parameter w_wood_hor = 11'd64;
    parameter h_wood_hor = 11'd8;

    //0 w/2, -h/2
    //1 -w/2, -h/2
    //2 -w/2, h/2
    //3 w/2, h/2

    reg ren, valid;
    wire ready;
    wire signed [10-1:0]x, y;
    wire signed [19-1:0]cos, sin;
    wire signed [11-1:0] w, h;

    cordic_dc cddc(.X(x), .Y(y), .phase(rad), .ready(ready), .valid( render_ready));
    assign data_ready = ready;
    assign cos =  { {9{x[9]}} , x };
    assign sin =  { {9{y[9]}} , y };
    assign w = (nth == 0)? w_floor :
               (nth == 1 | nth == 2)? bird_pig : 
               (nth == 6 | nth == 9)? w_wood_hor : w_wood ;
    assign h = (nth == 0)? h_floor :
               (nth == 1 | nth == 2)? bird_pig : 
               (nth == 6 | nth == 9)? h_wood_hor : h_wood ;

    always @(*) begin
//        vertice_x = 19'b0;
//        vertice_y = 19'b0;
//        if (ready) begin
        if(i==3'd0)begin
            vertice_x = (w/2) * cos - (-h/2) * sin + pos_x;
            vertice_y = (w/2) * sin + (-h/2) * cos + pos_y;
        end else if(i==3'd1)begin
            vertice_x = (-w/2) * cos - (-h/2) * sin + pos_x;
            vertice_y = (-w/2) * sin + (-h/2) * cos + pos_y;
        end else if(i==3'd2)begin
            vertice_x = (-w/2) * cos - h/2 * sin + pos_x;
            vertice_y = (-w/2) * sin + h/2 * cos + pos_y;
        end else if(i==3'd3) begin
            vertice_x = w/2 * cos - h/2 * sin + pos_x;
            vertice_y = w/2 * sin + h/2 * cos + pos_y;
        end else begin
            vertice_x = 19'b0;
            vertice_y = 19'b0;
        end
//            case (i)
//                3'd0 : begin
//                    vertice_x = (w/2) * cos - (-h/2) * sin + pos_x;
//                    vertice_y = (w/2) * sin + (-h/2) * cos + pos_y;
//                end
//                3'd1 : begin
//                    vertice_x = (-w/2) * cos - (-h/2) * sin + pos_x;
//                    vertice_y = (-w/2) * sin + (-h/2) * cos + pos_y;
//                end
//                3'd2 : begin
//                    vertice_x = (-w/2) * cos - h/2 * sin + pos_x;
//                    vertice_y = (-w/2) * sin + h/2 * cos + pos_y;
//                end
//                3'd3 : begin
//                    vertice_x = w/2 * cos - h/2 * sin + pos_x;
//                    vertice_y = w/2 * sin + h/2 * cos + pos_y;
//                end
//                default: begin
//                  vertice_x = 19'b0;
//                  vertice_y = 19'b0;
//                end
//            endcase
        end
//    end

endmodule

module cordic_dc(X, Y, phase, ready, valid);
    output [9:0]X, Y;
    input [9:0]phase;

    input wire valid;
    output wire ready;
    wire [32-1:0]dout;

    assign {Y, X} = { dout[25:16], dout[9:0] };



    cordic_0 cd0(
        .s_axis_phase_tvalid(valid),
        .s_axis_phase_tdata(phase),
        .m_axis_dout_tdata(dout),
        .m_axis_dout_tvalid(ready)
    );

endmodule
