module Support(
  input clk,
  input sup_start,
  input signed [18:0] v0_x, input signed [18:0] v0_y,
  input signed [18:0] v1_x, input signed [18:0] v1_y,
  input signed [18:0] v2_x, input signed [18:0] v2_y,
  input signed [18:0] v3_x, input signed [18:0] v3_y,
  input signed [18:0] pos_x, input signed [18:0] pos_y,
  input signed [10-1:0] dir_x, input signed [10-1:0] dir_y,
  output reg signed [18:0] bestVertex_x, reg signed [18:0] bestVertex_y,
  output done
  );

  reg signed [29-1:0] next_best_proj, best_proj;
  wire signed [29-1:0] distance;
  reg signed [18:0] next_bestVertex_x, next_bestVertex_y;
  reg signed [18:0] vx[0:4-1], vy[0:4-1];
  reg [3-1:0] cnt;
  
  assign done = (cnt == 3'd4) ? 1'b1 : 1'b0;

  always @(posedge clk or posedge sup_start) begin
    if (sup_start) begin
      vx[0] <= v0_x;
      vx[1] <= v1_x;
      vx[2] <= v2_x;
      vx[3] <= v3_x;
      vy[0] <= v0_y;
      vy[1] <= v1_y;
      vy[2] <= v2_y;
      vy[3] <= v3_y;
      bestVertex_x <= v0_x;
      bestVertex_y <= v0_y;
      best_proj <= {1'b1, 28'd1};
      cnt <= 3'b0;
    end else begin
      bestVertex_x <= next_bestVertex_x;
      bestVertex_y <= next_bestVertex_y;
      best_proj <= next_best_proj;
      cnt <= cnt;
      if (cnt < 3'd4) begin
        cnt <= cnt + 3'b1;
      end
    end
  end

  assign  distance = ((vx[cnt] - pos_x) * dir_x  + (vy[cnt] - pos_y) * dir_y);//pipeline
  always @(*) begin
//    distance = {1'b1, 28'd1};
    next_best_proj = best_proj;
    next_bestVertex_x = bestVertex_x;
    next_bestVertex_y = bestVertex_y;   
    if (~sup_start & cnt <= 3'd3) begin
//        distance = ((vx[cnt] - pos_x) * dir_x  + (vy[cnt] - pos_y) * dir_y);
        if (distance > best_proj) begin
          next_best_proj = distance;
          next_bestVertex_x = vx[cnt];
          next_bestVertex_y = vy[cnt];
        end
    end
  end

endmodule //Support
