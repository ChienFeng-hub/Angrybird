module PreStep(
    input clk, input rst,
    input signed [19-1:0] cPenetration,
    input signed [19-1:0] cPos_x, input signed [19-1:0] cPos_y,
    input signed [10-1:0] cNorm_x, input signed [10-1:0] cNorm_y,
    input signed [19-1:0] b1Pos_x, input signed [19-1:0] b1Pos_y,
    input signed [19-1:0] b2Pos_x, input signed [19-1:0] b2Pos_y,
    input signed [22-1:0] b1Inv_mass, input signed [22-1:0] b2Inv_mass,
    input signed [22-1:0] b1Inv_I, input signed [22-1:0] b2Inv_I,
    output signed [29-1:0] cMassNorm,
    output signed [29-1:0] cMassTang,
    output signed [19-1:0] cBias, 
    output done_out 
);
    parameter S0 = 3'd0;
    parameter S1 = 3'd1;
    parameter S2 = 3'd2;
    parameter S3 = 3'd3;
    parameter S4 = 3'd4;
    parameter S5 = 3'd5;
    parameter S6 = 3'd6;
    parameter S7 = 3'd7;

    wire signed [10-1:0] k_allowPenetration = 10'b00_00100000;
    wire signed [10-1:0] k_biasFactor = 10'b00_01000000;

    reg [3-1:0] state, next_state;
    wire signed [19-1:0] r1_x, r1_y, r2_x, r2_y;
    wire signed [19-1:0] temp;
    reg signed [38-1:0] a1, b1, c1, d1;
    reg signed [38-1:0] a2, b2, c2, d2;
    wire signed [38-1:0] e1, e2;
    reg signed [19-1:0] rn1, rn2;
    wire signed [19-1:0] next_rn1, next_rn2;
    reg signed [19-1:0] rt1, rt2;
    wire signed [19-1:0] next_rt1, next_rt2;
    reg signed [38-1:0] r1Dotr1, r2Dotr2;
    wire signed [38-1:0] next_r1Dotr1, next_r2Dotr2;
    reg signed [10-1:0] tangent_x, tangent_y; 
    wire signed [10-1:0] next_tangent_x, next_tangent_y; 
    reg signed [38-1:0] tp1, tp2, tp3, tp4;
    wire signed [38-1:0] next_tp1, next_tp2, next_tp3, next_tp4;
    wire signed [60-1:0] kNormal, kTangent;

    assign r1_x = cPos_x - b1Pos_x;
    assign r1_y = cPos_y - b1Pos_y;
    assign r2_x = cPos_x - b2Pos_x;
    assign r2_y = cPos_y - b2Pos_y;

    assign temp = cPenetration + { {10{k_allowPenetration[9]}}, k_allowPenetration[8:0] };
    assign cBias = (temp[18]) ? -(temp <<< 4):15'd0;

    assign e1 = a1 * b1 + c1 * d1;
    assign e2 = a2 * b2 + c2 * d2;

    assign next_rn1 = (state == S0) ? e1[26:8]:rn1;
    assign next_rn2 = (state == S0) ? e2[26:8]:rn2;
    assign next_tangent_x = (state == S1) ? e1[17:8]:tangent_x;
    assign next_tangent_y = (state == S1) ? e2[17:8]:tangent_y;
    assign next_r1Dotr1 = (state == S2) ? e1:r1Dotr1;
    assign next_r2Dotr2 = (state == S2) ? e2:r2Dotr2;
    assign next_rt1 = (state == S3) ? e1[26:8]:rt1;
    assign next_rt2 = (state == S3) ? e2[26:8]:rt2;
    assign next_tp1 = (state == S4) ? e1:tp1;
    assign next_tp2 = (state == S4) ? e2:tp2;
    assign next_tp3 = (state == S5) ? e1:tp3;
    assign next_tp4 = (state == S5) ? e2:tp4;
    assign kNormal = b1Inv_I * tp1 + b2Inv_I * tp2 + 
                      b1Inv_mass{22'b0, b1Inv_mass, 16'b0} + b2Inv_mass{22'b0, b2Inv_mass, 16'b0};
    assign kTangent = b1Inv_I * tp3 + b2Inv_I * tp4 + 
                      b1Inv_mass{22'b0, b1Inv_mass, 16'b0} + b2Inv_mass{22'b0, b2Inv_mass, 16'b0};
    assign cMassNorm = {10'b0, 1'b1, 49'b0} / kNormal[37:7];
    assign cMassTang = {10'b0, 1'b1, 49'b0} / kTangent[37:7];
    assign done_out = (state == S7) ? 1'b1:1'b0;


    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S0;
            rn1 <= 19'd0;
            rn2 <= 19'd0;
            rt1 <= 19'd0;
            rt2 <= 19'd0;
            r1Dotr1 <= 38'd0;
            r2Dotr2 <= 38'd0;
            tangent_x <= 10'd0;
            tangent_y <= 10'd0;
            tp1 <= 38'd0;
            tp2 <= 38'd0;
            tp3 <= 38'd0;
            tp4 <= 38'd0;
        end
        else begin
            state <= next_state;
            rn1 <= next_rn1;
            rn2 <= next_rn2;
            rt1 <= next_rt1;
            rt2 <= next_rt2;
            r1Dotr1 <= next_r1Dotr1;
            r2Dotr2 <= next_r2Dotr2;
            tangent_x <= next_tangent_x;
            tangent_y <= next_tangent_y;
            tp1 <= next_tp1;
            tp2 <= next_tp2;
            tp3 <= next_tp3;
            tp4 <= next_tp4;
        end
    end

    always @(*) begin
        case(state)
            S0: begin
                  next_state = S1;
                  a1 = {{20{r1_x[18]}}, r1_x[17:0]};
                  b1 = {{29{cNorm_x[9]}}, cNorm_x[8:0]};
                  c1 = {{20{r1_y[18]}}, r1_y[17:0]};
                  d1 = {{29{cNorm_y[9]}}, cNorm_y[8:0]};

                  a2 = {{20{r2_x[18]}}, r2_x[17:0]};
                  b2 = {{29{cNorm_x[9]}}, cNorm_x[8:0]};
                  c2 = {{20{r2_y[18]}}, r2_y[17:0]};
                  d2 = {{29{cNorm_y[9]}}, cNorm_y[8:0]};

                end
            S1: begin
                  next_state = S2;
                  a1 = {30'd1, 8'd0};
                  b1 = {{29{cNorm_y[9]}}, cNorm_y[8:0]};
                  c1 = 38'd0;
                  d1 = 38'd0;

                  a2 = -{30'd1, 8'd0};
                  b2 = {{29{cNorm_x[9]}}, cNorm_x[8:0]};
                  c2 = 38'd0;
                  d2 = 38'd0;
                  
                end
            S2: begin
                  next_state = S3;
                  a1 = {{20{r1_x[18]}}, r1_x[17:0]};
                  b1 = {{20{r1_x[18]}}, r1_x[17:0]};
                  c1 = {{20{r1_y[18]}}, r1_y[17:0]};
                  d1 = {{20{r1_y[18]}}, r1_y[17:0]};

                  a2 = {{20{r2_x[18]}}, r2_x[17:0]};
                  b2 = {{20{r2_x[18]}}, r2_x[17:0]};
                  c2 = {{20{r2_y[18]}}, r2_y[17:0]};
                  d2 = {{20{r2_y[18]}}, r2_y[17:0]};

                end
            S3: begin
                  next_state = S4;
                  a1 = {{20{r1_x[18]}}, r1_x[17:0]};
                  b1 = {{29{tangent_x[9]}}, tangent_x[8:0]};
                  c1 = {{20{r1_y[18]}}, r1_y[17:0]};
                  d1 = {{29{tangent_y[9]}}, tangent_y[8:0]};

                  a2 = {{20{r2_x[18]}}, r2_x[17:0]};
                  b2 = {{29{tangent_x[9]}}, tangent_x[8:0]};
                  c2 = {{20{r2_y[18]}}, r2_y[17:0]};
                  d2 = {{29{tangent_y[9]}}, tangent_y[8:0]};

                end    
            S4: begin
                  next_state = S5;
                  a1 = r1Dotr1;
                  b1 = 38'd1;
                  c1 = -{{20{rn1[18]}}, rn1[17:0]};
                  d1 = {{20{rn1[18]}}, rn1[17:0]};

                  a2 = r2Dotr2;
                  b2 = 38'd1;
                  c2 = -{{20{rn2[18]}}, rn2[17:0]};
                  d2 = {{20{rn2[18]}}, rn2[17:0]};

                end   
            S5: begin
                  next_state = S6;
                  a1 = r1Dotr1;
                  b1 = 38'd1;
                  c1 = -{{20{rt1[18]}}, rt1[17:0]};
                  d1 = {{20{rt1[18]}}, rt1[17:0]};

                  a2 = r2Dotr2;
                  b2 = 38'd1;
                  c2 = -{{20{rt2[18]}}, rt2[17:0]};
                  d2 = {{20{rt2[18]}}, rt2[17:0]};

                end 
            S6: begin
                  next_state = S7;
                  a1 = 38'd0;
                  b1 = 38'd0;
                  c1 = 38'd0;
                  d1 = 38'd0;
                  a2 = 38'd0;
                  b2 = 38'd0;
                  c2 = 38'd0;
                  d2 = 38'd0;

                end    
            default: begin
                  next_state = state;
                  a1 = 38'd0;
                  b1 = 38'd0;
                  c1 = 38'd0;
                  d1 = 38'd0;
                  a2 = 38'd0;
                  b2 = 38'd0;
                  c2 = 38'd0;
                  d2 = 38'd0;
                end
        endcase
    end

    
    

endmodule



endmodule