module PreStep(
    input clk, input rst,
    input signed [31-1:0] cPenetration,
    input signed [31-1:0] cPos_x, input signed [31-1:0] cPos_y,
    input signed [22-1:0] cNorm_x, input signed [22-1:0] cNorm_y,
    input signed [31-1:0] b1Pos_x, input signed [31-1:0] b1Pos_y,
    input signed [31-1:0] b2Pos_x, input signed [31-1:0] b2Pos_y,
    input signed [22-1:0] b1Inv_mass, input signed [22-1:0] b2Inv_mass,
    input signed [22-1:0] b1Inv_I, input signed [22-1:0] b2Inv_I,
    output signed [29-1:0] cMassNorm,
    output signed [29-1:0] cMassTang,
    output signed [31-1:0] cBias, 
    output reg done_out 
);
    parameter S0 = 4'd0;
    parameter S1 = 4'd1;
    parameter S2 = 4'd2;
    parameter S3 = 4'd3;
    parameter S4 = 4'd4;
    parameter S5 = 4'd5;
    parameter S6 = 4'd6;
    parameter S7 = 4'd7;
    parameter S8 = 4'd8;
    parameter S9 = 4'd9;

    wire signed [31-1:0] k_allowPenetration = 31'b00000000000_00000011111111111111;
    wire signed [10-1:0] k_biasFactor = 10'b00_01000000;

    reg [4-1:0] state, next_state;
    wire signed [22-1:0] tangent_x, tangent_y;
    wire signed [31-1:0] r1_x, r1_y, r2_x, r2_y;
    wire signed [31-1:0] temp;
    reg signed [35-1:0] a1, b1, c1, d1;
    reg signed [35-1:0] a2, b2, c2, d2;
    wire signed [60-1:0] e1, e2, temp_a1b1, temp_c1d1, temp_a2b2, temp_c2d2;
    reg signed [60-1:0] unuse, unuse_next;
    reg signed [60-1:0] unuse2, unuse2_next;
    reg signed [35-1:0] rn1, rn2;
    reg signed [35-1:0] next_rn1, next_rn2;
    reg signed [35-1:0] rt1, rt2;
    reg signed [35-1:0] next_rt1, next_rt2;
    reg signed [60-1:0] r1Dotr1, r2Dotr2;
    reg signed [60-1:0] next_r1Dotr1, next_r2Dotr2; 
    reg signed [35-1:0] tp1, tp2, tp3, tp4;
    reg signed [35-1:0] next_tp1, next_tp2, next_tp3, next_tp4;
    reg signed [60-1:0] kNormal, kTangent;
    reg signed [60-1:0] kNormal_next, kTangent_next;
    reg dv_valid;
    reg signed [29-1:0] cmass_norm , cmass_tang;
    reg signed [29-1:0] cmass_norm_next , cmass_tang_next;
    wire signed [31-1:0] zero;
    assign zero = 31'd0;

    assign tangent_x = cNorm_y;
    assign tangent_y = -cNorm_x;
    assign r1_x = cPos_x - b1Pos_x;
    assign r1_y = cPos_y - b1Pos_y;
    assign r2_x = cPos_x - b2Pos_x;
    assign r2_y = cPos_y - b2Pos_y;

    assign temp = cPenetration + k_allowPenetration;
    assign cBias = (temp < zero) ? -(temp):31'd0;

    mult_gen_35x35 a1b1(.A(a1), .B(b1), .P(temp_a1b1));
    mult_gen_35x35 c1d1(.A(c1), .B(d1), .P(temp_c1d1));
    mult_gen_35x35 a2b2(.A(a2), .B(b2), .P(temp_a2b2));
    mult_gen_35x35 c2d2(.A(c2), .B(d2), .P(temp_c2d2));

//    assign temp_a1b1 = a1 * b1;
//    assign temp_c1d1 = c1 * d1;
//    assign temp_a2b2 = a2 * b2;
//    assign temp_c2d2 = c2 * d2;
    assign e1 = temp_a1b1 + temp_c1d1;
    assign e2 = temp_a2b2 + temp_c2d2;
    
    wire dout1_valid, dout2_valid;
    reg [35-1:0] divisor_in;
    wire signed [71:0] dv_out;

    div_gen_dv29 dv00(
        .aclk(clk),
        .s_axis_divisor_tdata(divisor_in),
        .s_axis_divisor_tvalid(dv_valid),
        .s_axis_dividend_tdata({1'b0, 23'd1, 40'b0}),
        .s_axis_dividend_tvalid(dv_valid),
        
        .m_axis_dout_tdata(dv_out),
        .m_axis_dout_tvalid(dout1_valid)
    );
    
//    div_gen_dv29 dv01(
//        .aclk(clk),
//        .s_axis_divisor_tdata(kTangent[54:20]),
//        .s_axis_divisor_tvalid(dv_valid),
//        .s_axis_dividend_tdata({1'b0, 23'd1, 40'b0}),
//        .s_axis_dividend_tvalid(dv_valid),
        
//        .m_axis_dout_tdata(cmass_tang),
//        .m_axis_dout_tvalid(dout2_valid)
//    );

    assign cMassNorm = cmass_norm;
    assign cMassTang = cmass_tang;


    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S0;
            rn1 <= 35'd0;
            rn2 <= 35'd0;
            rt1 <= 35'd0;
            rt2 <= 35'd0;
            r1Dotr1 <= 60'd0;
            r2Dotr2 <= 60'd0;
            tp1 <= 35'd0;
            tp2 <= 35'd0;
            tp3 <= 35'd0;
            tp4 <= 35'd0;
            kNormal <= {18'b0, b1Inv_mass, 20'b0} + {18'b0, b2Inv_mass, 20'b0};
            kTangent <= {18'b0, b1Inv_mass, 20'b0} + {18'b0, b2Inv_mass, 20'b0};
            done_out <= 1'b0;
            cmass_norm <= 0;
            cmass_tang <= 0;
        end
        else begin
            state <= next_state;
            done_out <= done_out;
            if (state == S9) begin
                done_out <= 1'b1;
            end
            rn1 <= next_rn1;
            rn2 <= next_rn2;
            rt1 <= next_rt1;
            rt2 <= next_rt2;
            r1Dotr1 <= next_r1Dotr1;
            r2Dotr2 <= next_r2Dotr2;
            tp1 <= next_tp1;
            tp2 <= next_tp2;
            tp3 <= next_tp3;
            tp4 <= next_tp4;
            unuse <= unuse_next;
            unuse2 <= unuse2_next;
            kNormal <= kNormal_next;
            kTangent <= kTangent_next;
            cmass_norm <= cmass_norm_next;
            cmass_tang <= cmass_tang_next;
        end
    end

    always @(*) begin
        next_rn1 = rn1;
        next_rn2 = rn2;
        next_rt1 = rt1;
        next_rt2 = rt2;
        unuse_next = e1;
        unuse2_next = e2;
        next_r1Dotr1 = r1Dotr1;
        next_r2Dotr2 = r2Dotr2;
        next_tp1 = tp1;
        next_tp2 = tp2;
        next_tp3 = tp3;
        next_tp4 = tp4;
        kNormal_next = kNormal;
        kTangent_next = kTangent;
        dv_valid = 1'b0;
        cmass_norm_next = cmass_norm;
        cmass_tang_next = cmass_tang;
        case(state)
            S0: begin
                    next_state = S1;
                    a1 = {{4{r1_x[30]}}, r1_x};
                    b1 = {{13{cNorm_x[21]}}, cNorm_x};
                    c1 = {{4{r1_y[30]}}, r1_y};
                    d1 = {{13{cNorm_y[21]}}, cNorm_y};
                    
                    a2 = {{4{r2_x[30]}}, r2_x};
                    b2 = {{13{cNorm_x[21]}}, cNorm_x};
                    c2 = {{4{r2_y[30]}}, r2_y};
                    d2 = {{13{cNorm_y[21]}}, cNorm_y};
                    
                    next_rn1 = e1[54:20];
                    next_rn2 = e2[54:20];
                end
            S1: begin
                    next_state = S2;
                    a1 = {{4{r1_x[30]}}, r1_x};
                    b1 = {{4{r1_x[30]}}, r1_x};
                    c1 = {{4{r1_y[30]}}, r1_y};
                    d1 = {{4{r1_y[30]}}, r1_y};
                    
                    a2 = {{4{r2_x[30]}}, r2_x};
                    b2 = {{4{r2_x[30]}}, r2_x};
                    c2 = {{4{r2_y[30]}}, r2_y};
                    d2 = {{4{r2_y[30]}}, r2_y};

                    next_r1Dotr1 = e1;
                    next_r2Dotr2 = e2;                    
                end
            S2: begin
                    next_state = S3;
                    a1 = r1Dotr1[54:20];
                    b1 = {1'b0, 14'd1, 20'b0};
                    c1 = -rn1;
                    d1 = rn1;
                    
                    a2 = r2Dotr2[54:20];
                    b2 = {1'b0, 14'd1, 20'b0};
                    c2 = -rn2;
                    d2 = rn2;
                    
                    next_tp1 = e1[54:20];
                    next_tp2 = e2[54:20];
                end
            S3: begin
                    next_state = S4;
                    a1 = {{4{r1_x[30]}}, r1_x};
                    b1 = {{13{tangent_x[21]}}, tangent_x};
                    c1 = {{4{r1_y[30]}}, r1_y};
                    d1 = {{13{tangent_y[21]}}, tangent_y};
                    
                    a2 = {{4{r2_x[30]}}, r2_x};
                    b2 = {{13{tangent_x[21]}}, tangent_x};
                    c2 = {{4{r2_y[30]}}, r2_y};
                    d2 = {{13{tangent_y[21]}}, tangent_y};
                    
                    next_rt1 = e1[54:20];
                    next_rt2 = e2[54:20];
                end    
            S4: begin
                    next_state = S5;
                    a1 = r1Dotr1[54:20];
                    b1 = {1'b0, 14'd1, 20'b0};
                    c1 = -rt1;
                    d1 = rt1;
                    
                    a2 = r2Dotr2[54:20];
                    b2 = {1'b0, 14'd1, 20'b0};
                    c2 = -rt2;
                    d2 = rt2;
                    
                    next_tp3 = e1[54:20];
                    next_tp4 = e2[54:20];
                end   
            S5: begin
                    next_state = S6;
                    a1 = {{13{b1Inv_I[21]}}, b1Inv_I};
                    b1 = tp1;
                    c1 = {{13{b2Inv_I[21]}}, b2Inv_I};
                    d1 = tp2;
                    
                    a2 = {{13{b1Inv_I[21]}}, b1Inv_I};
                    b2 = tp3;
                    c2 = {{13{b2Inv_I[21]}}, b2Inv_I};
                    d2 = tp4;
                    
                    kNormal_next = kNormal + e1;
                    kTangent_next = kTangent + e2;
                end
            S6: begin
                    next_state = S7;
                    dv_valid = 1'b1;
                    divisor_in = kNormal[54:20];
                    next_state = S7;
                end 
            S7: begin
                    next_state = S7;
                    dv_valid = 1'b1;
                    divisor_in = kTangent[54:20];
                    if (dout1_valid) begin
                        next_state = S8;
                        cmass_norm_next = dv_out[30:2];
                    end
                end
            S8: begin
                    next_state = S8;
                    if (dout1_valid) begin
                        next_state = S9;
                        cmass_tang_next = dv_out[30:2];
                    end
                end
            S9: begin
                    next_state = S9;
                end
            
        endcase
    end

    
    

endmodule