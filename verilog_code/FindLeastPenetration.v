module FindLeastPenetration(
  input clk, start,
  input signed [18:0] A0_x, input signed [18:0] A0_y,
  input signed [18:0] A1_x, input signed [18:0] A1_y,
  input signed [18:0] A2_x, input signed [18:0] A2_y,
  input signed [18:0] A3_x, input signed [18:0] A3_y,
  input signed [18:0] B0_x, input signed [18:0] B0_y,
  input signed [18:0] B1_x, input signed [18:0] B1_y,
  input signed [18:0] B2_x, input signed [18:0] B2_y,
  input signed [18:0] B3_x, input signed [18:0] B3_y,
  input signed [18:0] Bpos_x, input signed [18:0] Bpos_y,
  input signed [10-1:0] norm0_x, input signed [10-1:0] norm0_y,
  input signed [10-1:0] norm1_x, input signed [10-1:0] norm1_y,
  input signed [10-1:0] norm2_x, input signed [10-1:0] norm2_y,
  input signed [10-1:0] norm3_x, input signed [10-1:0] norm3_y,
  output reg signed [19-1:0] bestPenetration,
  output reg signed [10-1:0] bestNorm_x,
  output reg signed [10-1:0] bestNorm_y,
  output reg [2-1:0] bestIndex,
  output done_out
  );

  reg signed [18:0] Ax[0:4-1];
  reg signed [18:0] Ay[0:4-1];
  reg signed [10-1:0] norm_x[0:4-1];
  reg signed [10-1:0] norm_y[0:4-1];
  reg signed [19-1:0] next_bestPenetration;
  reg signed [10-1:0] next_bestNorm_x;
  reg signed [10-1:0] next_bestNorm_y;
  reg [2-1:0] next_bestIndex;
  reg [3-1:0] cnt;

  wire signed [19-1:0] tempVertex_x;
  wire signed [19-1:0] tempVertex_y;
  wire signed [19-1:0] temp_BAx;
  wire signed [19-1:0] temp_BAy;
  wire signed [29-1:0] distance;
  wire signed [19-1:0] penetration;
  wire sup_done;
  reg sup_start;

  Support s1(
    .clk(clk), .sup_start(sup_start),
    .v0_x(B0_x), .v0_y(B0_y),
    .v1_x(B1_x), .v1_y(B1_y),
    .v2_x(B2_x), .v2_y(B2_y),
    .v3_x(B3_x), .v3_y(B3_y),
    .pos_x(Bpos_x), .pos_y(Bpos_y),
    .dir_x(-norm_x[cnt]), .dir_y(-norm_y[cnt]),
    .bestVertex_x(tempVertex_x), .bestVertex_y(tempVertex_y),
    .done(sup_done)
    );

  always @ (posedge clk or posedge start) begin
    if (start) begin
      Ax[0] <= A0_x;
      Ax[1] <= A1_x;
      Ax[2] <= A2_x;
      Ax[3] <= A3_x;
      Ay[0] <= A0_y;
      Ay[1] <= A1_y;
      Ay[2] <= A2_y;
      Ay[3] <= A3_y;
      norm_x[0] <= norm0_x;
      norm_x[1] <= norm1_x;
      norm_x[2] <= norm2_x;
      norm_x[3] <= norm3_x;
      norm_y[0] <= norm0_y;
      norm_y[1] <= norm1_y;
      norm_y[2] <= norm2_y;
      norm_y[3] <= norm3_y;
      bestPenetration <= {1'b1, 10'b0, 8'b00000001};
      bestNorm_x <= 10'b0;
      bestNorm_y <= 10'b0;
      bestIndex <= 2'b0;
      cnt <= 3'b0;
      sup_start <= 1'b1;
    end else begin
      bestPenetration <= next_bestPenetration;
      bestNorm_x <= next_bestNorm_x;
      bestNorm_y <= next_bestNorm_y;
      bestIndex <= next_bestIndex;
      cnt <= cnt;
      sup_start <= 1'b0;
      if (cnt < 3'd3 & sup_done) begin
        cnt <= cnt + 3'b1;
        if(cnt != 3'd3)begin
            sup_start <= 1'b1;
        end
      end
    end
  end

  always @ ( * ) begin
    next_bestPenetration = bestPenetration;
    next_bestNorm_x = bestNorm_x;
    next_bestNorm_y = bestNorm_y;
    next_bestIndex = bestIndex;
    if (sup_done) begin
        if (penetration > bestPenetration) begin
          next_bestPenetration = penetration;
          next_bestNorm_x = norm_x[cnt];
          next_bestNorm_y = norm_y[cnt];
          next_bestIndex = cnt;
        end
    end
  end

  assign temp_BAx = tempVertex_x - Ax[cnt];
  assign temp_BAy = tempVertex_y - Ay[cnt];
  assign distance = temp_BAx * norm_x[cnt] + temp_BAy * norm_y[cnt];
  assign penetration = distance[26:8];
  assign done_out = (cnt == 3'd3 & sup_done) ? 1'b1 : 1'b0;


endmodule
