module ComputeIncidentEdge(
    input clk,
    input start,
    input signed [10-1:0] referenceNorm_x,
    input signed [10-1:0] referenceNorm_y,
    input signed [10-1:0] norm0_x, input signed [10-1:0] norm0_y,
    input signed [10-1:0] norm1_x, input signed [10-1:0] norm1_y,
    input signed [10-1:0] norm2_x, input signed [10-1:0] norm2_y,
    input signed [10-1:0] norm3_x, input signed [10-1:0] norm3_y,
    output signed [10-1:0] incidentNorm_x,
    output signed [10-1:0] incidentNorm_y,
    output reg [2-1:0] incidentIndex,
    output done_out
);

wire signed [10-1:0] local_normX[0:4-1], local_normY[0:4-1];
wire signed [20-1:0] temp;
reg signed [20-1:0] smallestProduct, next_smallestProduct;
reg [3-1:0] cnt;
reg [2-1:0]next_incidentindex;

assign local_normX[0] = norm0_x;
assign local_normX[1] = norm1_x;
assign local_normX[2] = norm2_x;
assign local_normX[3] = norm3_x;
assign local_normY[0] = norm0_y;
assign local_normY[1] = norm1_y;
assign local_normY[2] = norm2_y;
assign local_normY[3] = norm3_y;

assign temp = { {10{local_normX[cnt][9]}}, local_normX[cnt]} * { {10{referenceNorm_x[9]}},referenceNorm_x} + 
              { {10{local_normY[cnt][9]}}, local_normY[cnt]} * { {10{referenceNorm_y[9]}},referenceNorm_y};
assign incidentNorm_x = local_normX[incidentIndex];
assign incidentNorm_y = local_normY[incidentIndex];
assign done_out = (cnt == 3'd3) ? 1'b1 : 1'b0;

always @(posedge clk) begin
  if (start) begin
    cnt <= 3'b0;
    smallestProduct <= 20'b0;
    incidentIndex <= 2'b0;
  end else begin
    smallestProduct <= next_smallestProduct;
    incidentIndex <= next_incidentindex;
    if (cnt < 3'd3) begin
      cnt <= cnt + 3'd1;
    end
  end
end

always @(*) begin
    next_smallestProduct = smallestProduct;
    next_incidentindex = incidentIndex;
    if (temp < smallestProduct) begin
        next_smallestProduct = temp;
        next_incidentindex = cnt;
    end
end

endmodule