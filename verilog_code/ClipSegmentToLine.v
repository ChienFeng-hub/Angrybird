 // ax+by+c = 0
 // [ lineNorm.x, lineNorm.y ][ x ] = [ c1 ]
 // [ incident.x, incident.y ][ y ] = [ c2 ]
 // [ x ] = [ incident.y, -lineNorm.y ][ c1 ] / det
 // [ y ] = [ -incident.x, lineNorm.x ][ c2 ] / det

module Clip(
        input clk,
        input start,
        input signed [10-1:0] side1Norm_x, input signed [10-1:0] side1Norm_y,
        input signed [10-1:0] side2Norm_x, input signed [10-1:0] side2Norm_y,
        input signed [10-1:0] incidentNorm_x, input signed [10-1:0] incidentNorm_y,
        input signed [19-1:0] pos1_x, input signed [19-1:0] pos1_y,
        input signed [19-1:0] pos2_x, input signed [19-1:0] pos2_y,
        input signed [19-1:0] refer1Pos_x, input signed [19-1:0] refer1Pos_y,
        input signed [19-1:0] refer2Pos_x, input signed [19-1:0] refer2Pos_y,
        input signed [10-1:0] referNorm_x, input signed [10-1:0] referNorm_y,
        output reg signed [19-1:0] contact1Pt_x, output reg signed [19-1:0] contact1Pt_y,
        output reg signed [19-1:0] contact2Pt_x, output reg signed [19-1:0] contact2Pt_y,
        output reg signed [19-1:0] contact1Penetration, output reg signed [19-1:0] contact2Penetration,
        output reg [2-1:0] contactNum,
        output done_out
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
	parameter S10 = 4'd10;
	parameter S11 = 4'd11;
	parameter S12 = 4'd12;
	parameter S13 = 4'd13;
	parameter S14 = 4'd14;
	parameter S15 = 4'd15;


	reg signed [20-1:0] lamda1, next_lamda1, lamda2, next_lamda2; 
	reg signed [40-1:0] product1, next_product1, product2, next_product2;
	reg signed [40-1:0] product1_, next_product1_, product2_, next_product2_;
	reg signed [40-1:0] prod1, next_prod1, prod2, next_prod2;
	reg signed [20-1:0] det1, next_det1, det2, next_det2;
	reg signed [40-1:0] newPos_x, newPos_y;
	reg signed [40-1:0] next_newPos_x, next_newPos_y;
	reg signed [40-1:0] newPos_x_, newPos_y_;
	reg signed [40-1:0] next_newPos_x_, next_newPos_y_;
	reg signed [20-1:0] a1, b1, c1, d1, a2, b2, c2, d2;
	wire signed[40-1:0] e1, e2, temp1, temp2, temp3, temp4;
	reg signed [20-1:0] unuse1, unuse2, unuse1_next, unuse2_next;
	reg signed [20-1:0] next_e1, next_e2;
	wire signed[20-1:0] vec1_x, vec1_y, vec2_x, vec2_y;
	wire signed[40-1:0] tmp_pos1_x, tmp_pos1_y, tmp_pos2_x, tmp_pos2_y;
	wire signed[40-1:0] fnl_pos1_x, fnl_pos1_y, fnl_pos2_x, fnl_pos2_y;
	
	
    reg signed [40-1:0] dividend0, dividend1;
    reg signed [20-1:0] divisor0, divisor1;
    wire signed [48-1:0] dv0_out, dv_out_y;
//    wire signed [48-1:0] dv_out_x_, dv_out_y_;
    
    reg dv01_valid, dv23_valid;
	reg [4-1:0] state, next_state;

	//calculate
	mult_gen_20x20 mult_1(.A(a1), .B(b1), .P(temp1));
	mult_gen_20x20 mult_2(.A(c1), .B(d1), .P(temp2));
	mult_gen_20x20 mult_3(.A(a2), .B(b2), .P(temp3));
	mult_gen_20x20 mult_4(.A(c2), .B(d2), .P(temp4));
	assign e1 = temp1 + temp2;
    assign e2 = temp3 + temp4;
    

	assign tmp_pos1_x = (!product1[39]) ? newPos_x : {{1{pos1_x[18]}}, pos1_x};
	assign tmp_pos1_y = (!product1[39]) ? newPos_y : {{1{pos1_y[18]}}, pos1_y};
	assign tmp_pos2_x = (!product2[39]) ? newPos_x : {{1{pos2_x[18]}}, pos2_x};
	assign tmp_pos2_y = (!product2[39]) ? newPos_y : {{1{pos2_y[18]}}, pos2_y};

	assign fnl_pos1_x = (!product1_[39]) ? newPos_x_ : tmp_pos1_x;
	assign fnl_pos1_y = (!product1_[39]) ? newPos_y_ : tmp_pos1_y;
	assign fnl_pos2_x = (!product2_[39]) ? newPos_x_ : tmp_pos2_x;
	assign fnl_pos2_y = (!product2_[39]) ? newPos_y_ : tmp_pos2_y;
	
	assign done_out = (state == S14) ? 1'b1:1'b0;
	
	wire dv0_ready, dv1_ready, dv2_ready, dv3_ready;
    
    div_gen_40dv20 dv0(
        .aclk(clk),
        .s_axis_divisor_tdata(divisor0),
        .s_axis_divisor_tvalid(dv01_valid),
        .s_axis_dividend_tdata(dividend0),
        .s_axis_dividend_tvalid(dv01_valid),
        
        .m_axis_dout_tdata(dv0_out),
        .m_axis_dout_tvalid(dv0_ready)
    );
    
//    div_gen_40dv20 dv1(
//        .aclk(clk),
//        .s_axis_divisor_tdata(divisor1),
//        .s_axis_divisor_tvalid(dv01_valid),
//        .s_axis_dividend_tdata(dividend1),
//        .s_axis_dividend_tvalid(dv01_valid),
        
//        .m_axis_dout_tdata(dv_out_y),
//        .m_axis_dout_tvalid(dv1_ready)
//    );
    

	always @(posedge clk or posedge start) begin
		if (start) begin
			state <= S0;
			lamda1 <=      0;
            lamda2 <=      0;
            det1 <=        0;
            det2 <=        0;
            newPos_x <=    0;
            newPos_y <=    0;
            newPos_x_ <=   0;
            newPos_y_ <=   0;
            product1 <=    0;
            product2 <=    0;
            product1_ <=   0;
            product2_ <=   0;
            prod1 <=       0;
            prod2 <=       0;
            unuse1 <=      0;
            unuse2 <=      0;
		end
		else begin
			state <= next_state;
			lamda1 <= next_lamda1;
			lamda2 <= next_lamda2;
			det1 <= next_det1;
			det2 <= next_det2;
			newPos_x <= next_newPos_x;
			newPos_y <= next_newPos_y;
			newPos_x_ <= next_newPos_x_;
			newPos_y_ <= next_newPos_y_;
			product1 <= next_product1;
			product2 <= next_product2;
			product1_ <= next_product1_;
			product2_ <= next_product2_;
			prod1 <= next_prod1;
			prod2 <= next_prod2;
			unuse1 <= unuse1_next;
			unuse2 <= unuse2_next;
		end
	end

	always @(*) begin
        next_state = state;
        next_lamda1 = lamda1;
        next_lamda2 = lamda2;
        next_det1 = det1;
        next_det2 = det2;
        next_product1 = product1;
        next_product2 = product2;
        next_product1_ = product1_;
        next_product2_ = product2_;
        next_prod1 = prod1;
        next_prod2 = prod2;
        next_newPos_x = newPos_x ;
        next_newPos_y = newPos_y ;
        next_newPos_x_ = newPos_x_ ; 
        next_newPos_y_ = newPos_y_ ; 
        a1 = 0;
        b1 = 0;
        c1 = 0;
        d1 = 0;
        a2 = 0;
        b2 = 0;
        c2 = 0;
        d2 = 0;  
        unuse1_next = e1;
        unuse2_next = e2;
        dv01_valid = 1'b0;
        dv23_valid = 1'b0;
        divisor0 = 20'd1;
//        divisor1 = 20'd1;
        dividend0 = 40'd0;
//        dividend1 = 40'd0;
        case (state)
          S0 : begin
            next_state = S1;
            a1 = { {10{side1Norm_x[9]} }, side1Norm_x }; 
            b1 = { {1{refer1Pos_x[18]}}, refer1Pos_x };
            c1 = { {10{side1Norm_y[9]} }, side1Norm_y };
            d1 = { {1{refer1Pos_y[18]}}, refer1Pos_y };
            next_lamda1 = e1[27:8];//20
            a2 = { {10{incidentNorm_x[9]}}, incidentNorm_x }; 
            b2 = { {1{pos1_x[18]}}, pos1_x };
            c2 = { {10{incidentNorm_y[9]}}, incidentNorm_y };
            d2 = { {1{pos1_y[18]}}, pos1_y };
            next_lamda2 = e2[27:8];//20
          end 

          S1 : begin
            next_state = S2;
            a1 = { {10{side1Norm_x[9]}}, side1Norm_x }; 
            b1 = { {10{incidentNorm_y[9]}}, incidentNorm_y }; 
            c1 = -{{10{side1Norm_y[9]}}, side1Norm_y }; 
            d1 = { {10{incidentNorm_x[9]}}, incidentNorm_x }; 
            next_det1 = e1[27:8];//20
            a2 = { {10{side2Norm_x[9]}}, side2Norm_x }; 
            b2 = { {10{incidentNorm_y[9]}}, incidentNorm_y }; 
            c2 = -{{10{side2Norm_y[9]}}, side2Norm_y }; 
            d2 = { {10{incidentNorm_x[9]}}, incidentNorm_x }; 
            next_det2 = e2[27:8];//20
          end

          S2 : begin
            next_state = S3;
            a1 = { {10{incidentNorm_y[9]}}, incidentNorm_y };  
            b1 = lamda1; 
            c1 = -{{10{side1Norm_y[9]}}, side1Norm_y}; 
            d1 = lamda2; 
            next_newPos_x = e1;//40
            a2 = { {10{side1Norm_x[9]}}, side1Norm_x }; 
            b2 = lamda2; 
            c2 = -{{10{incidentNorm_x[9]}}, incidentNorm_x}; 
            d2 = lamda1; 
            next_newPos_y = e2;//40
          end
          
          S3: begin //divider
            next_state = S4;
            
            dv01_valid = 1'b1;
            divisor0 = det1;
            dividend0 = newPos_x;
           end
          S4: begin
            next_state = S4;
            dv01_valid = 1'b1;
            divisor0 = det1;
            dividend0 = newPos_y;
            if (dv0_ready) begin
                next_state = S5;
                next_newPos_x = dv0_out[41:2];
            end
          end
          S5: begin
            next_state = S5;
            if (dv0_ready) begin
                next_state = S6;
                next_newPos_y = dv0_out[41:2];
            end
          end
          S6 : begin
            next_state = S7;
            a1 = {{1{pos1_x[18]}}, pos1_x} - newPos_x[19:0];
            b1 = {{10{side1Norm_x[9]}}, side1Norm_x }; 
            c1 = {{1{pos1_y[18]}}, pos1_y} - newPos_y[19:0];
            d1 = {{10{side1Norm_y[9]}}, side1Norm_y }; 
            next_product1 = e1;//20
            a2 = {{1{pos2_x[18]}}, pos2_x} - newPos_x[19:0];
            b2 = {{10{side1Norm_x[9]}}, side1Norm_x }; 
            c2 = {{1{pos2_y[18]}}, pos2_y} - newPos_y[19:0];
            d2 = {{10{side1Norm_y[9]}}, side1Norm_y }; 
            next_product2 = e2;//20
          end

          S7 : begin
            next_state = S8;
            a1 = { {10{side2Norm_x[9]}}, side2Norm_x }; 
            b1 = { {1{refer2Pos_x[18]}}, refer2Pos_x };
            c1 = { {10{side2Norm_y[9]}}, side2Norm_y };
            d1 = { {1{refer2Pos_y[18]}}, refer2Pos_y };
            next_lamda1 = e1[27:8];//29
            a2 = { {10{incidentNorm_x[9]}}, incidentNorm_x }; 
            b2 = tmp_pos1_x[19:0];
            c2 = { {10{incidentNorm_y[9]}}, incidentNorm_y };
            d2 = tmp_pos1_y[19:0];
            next_lamda2 = e2[27:8];//29
          end

          S8 : begin
            next_state = S9; //det2
            a1 = { {10{incidentNorm_y[9]}}, incidentNorm_y };  
            b1 = lamda1; 
            c1 = -{ {10{side2Norm_y[9]}}, side2Norm_y }; 
            d1 = lamda2; 
            next_newPos_x_ = e1;
            a2 = { {10{side2Norm_x[9]}}, side2Norm_x }; 
            b2 = lamda2; 
            c2 = -{ {10{incidentNorm_x[9]}}, incidentNorm_x }; 
            d2 = lamda1; 
            next_newPos_y_ = e1;
          end
          S9 : begin
            next_state = S10;
            
            dv01_valid = 1'b1;
            divisor0 = det2;
            dividend0 = newPos_x_;
            
          end
          S10: begin
            next_state = S10;
            dv01_valid = 1'b1;
            divisor0 = det2;
            dividend0 = newPos_y_;
            if (dv0_ready) begin
                next_state = S11;
                next_newPos_x_ = dv0_out[41:2];
            end
          end
          S11 : begin
              next_state = S11;
              if (dv0_ready) begin
                  next_state = S12;
                  next_newPos_y_ = dv0_out[41:2];
              end
            end          
          S12 : begin
              next_state = S13;//wai
              a1 = tmp_pos1_x[19:0] - newPos_x_[19:0];
              b1 = { {10{side2Norm_x[9]}}, side2Norm_x }; 
              c1 = tmp_pos1_y[19:0] - newPos_y_[19:0];
              d1 = { {10{side2Norm_y[9]}}, side2Norm_y }; 
              next_product1_ = e1;//19
              a2 = tmp_pos1_x[19:0] - newPos_x_[19:0];
              b2 = { {10{side2Norm_x[9]}}, side2Norm_x }; 
              c2 = tmp_pos1_y[19:0] - newPos_y_[19:0];
              d2 = { {10{side2Norm_y[9]}}, side2Norm_y }; 
              next_product2_ = e2;//19
          end

          S13 : begin
            next_state = S14;
            a1 = tmp_pos1_x - { {1{refer1Pos_x[18]}}, refer1Pos_x };
            b1 = { {10{referNorm_x[9]}}, referNorm_x }; 
            c1 = tmp_pos1_y - { {1{refer1Pos_y[18]}}, refer1Pos_y };
            d1 = { {10{referNorm_y[9]}}, referNorm_y };
            next_prod1 = e1; 
            a2 = tmp_pos2_x - { {1{refer1Pos_x[18]}}, refer1Pos_x };
            b2 = { {10{referNorm_x[9]}}, referNorm_x }; 
            c2 = tmp_pos2_y - { {1{refer1Pos_y[18]}}, refer1Pos_y };
            d2 = { {10{referNorm_y[9]}}, referNorm_y }; 
            next_prod2 = e2; 
            
          end

          S14 : begin
            next_state = S14;//done
          end
          
        endcase

	end
	
	always @(*) begin
	    
	    contactNum = 2'd0;
        contact1Penetration = 0;
        contact2Penetration = 0;
        contact1Pt_x = 0;
        contact1Pt_y = 0;
        contact2Pt_x = 0;
        contact2Pt_y = 0;
        if(!start)begin
             contactNum = 2'd0;
             contact1Penetration = 0;
             contact2Penetration = 0;
             contact1Pt_x = 0;
             contact1Pt_y = 0;
             contact2Pt_x = 0;
             contact2Pt_y = 0;
             if (prod1[39] & prod2[39]) begin
                   contactNum = 2'd2;
                   contact1Penetration = prod1[26:8];
                   contact2Penetration = prod2[26:8];
                   contact1Pt_x = fnl_pos1_x[18:0];
                   contact1Pt_y = fnl_pos1_y[18:0];
                   contact2Pt_x = fnl_pos2_x[18:0];
                   contact2Pt_y = fnl_pos2_y[18:0];
               end 
               else if (prod1[39] & !prod2[39]) begin
                   contactNum = 2'd1;
                   contact1Penetration = prod1[26:8];
                   contact1Pt_x = fnl_pos1_x[18:0];
                   contact1Pt_y = fnl_pos1_y[18:0];
               end 
               else if (!prod1[39] & prod2[39]) begin
                   contactNum = 2'd1;
                   contact1Penetration = prod2[26:8];
                   contact1Pt_x = fnl_pos2_x[18:0];
                   contact1Pt_y = fnl_pos2_y[18:0];
               end 
        end
	end

	
endmodule