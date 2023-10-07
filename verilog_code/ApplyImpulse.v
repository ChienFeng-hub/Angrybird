module ApplyImpulse(
	input clk, input rst,
	input signed [31-1:0] cPos_x, input signed [31-1:0] cPos_y, //20 bit
	input signed [22-1:0] cNorm_x, input signed [22-1:0] cNorm_y, //20 bit
    input signed [31-1:0] b1Pos_x, input signed [31-1:0] b1Pos_y, //20 bit
    input signed [31-1:0] b2Pos_x, input signed [31-1:0] b2Pos_y, //20 bit
    input signed [22-1:0] b1Inv_mass, input signed [22-1:0] b2Inv_mass, //20 bit
    input signed [22-1:0] b1Inv_I, input signed [22-1:0] b2Inv_I, //20 bit
    input signed [30-1:0] b1Vel_x, input signed [30-1:0] b1Vel_y, //20 bit
    input signed [30-1:0] b2Vel_x, input signed [30-1:0] b2Vel_y, //20 bit
    input signed [22-1:0] b1AngVel, input signed [22-1:0] b2AngVel, //20 bit
    input signed [35-1:0] cPn /*20 bit*/, input signed [35-1:0] cPt, //20 bit
    input signed [29-1:0] cMassNorm, //20 bit
    input signed [29-1:0] cMassTang, //20 bit
    input signed [31-1:0] cBias, //20 bit
    output signed [35-1:0] cPn_aft /*20 bit*/, output signed [35-1:0] cPt_aft, //20 bit
    output signed [30-1:0] b1Vel_x_aft/*20 bit*/, output signed [30-1:0] b1Vel_y_aft, //20 bit
    output signed [30-1:0] b2Vel_x_aft/*20 bit*/, output signed [30-1:0] b2Vel_y_aft, //20 bit 
    output signed [22-1:0] b1AngVel_aft/*20 bit*/, output signed [22-1:0] b2AngVel_aft, //20 bit
    output reg done_out 
	);

	// friction = 0.25, shift 2
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

	reg [4-1:0] state, next_state;
	reg signed [30-1:0] b1Vel_x_sv, b1Vel_y_sv, b1Vel_x_sv_next, b1Vel_y_sv_next;
	reg signed [30-1:0] b2Vel_x_sv, b2Vel_y_sv, b2Vel_x_sv_next, b2Vel_y_sv_next;
	reg signed [22-1:0] b1AngVel_sv, b2AngVel_sv, b1AngVel_sv_next, b2AngVel_sv_next;
	wire signed [22-1:0] tangent_x, tangent_y; 
	wire signed [31-1:0] r1_x, r1_y, r2_x, r2_y;
	reg signed [35-1:0] a1, b1, c1, d1, a2, b2, c2, d2;
	wire signed [60-1:0] e1, e2, temp1, temp2, temp3, temp4;
	reg signed [60-1:0] unuse, unuse_next;
	reg signed [60-1:0] unuse2, unuse2_next;
	reg signed [30-1:0] dvx, next_dvx;
	reg signed [30-1:0] dvy, next_dvy;
	reg signed [35-1:0] vn, next_vn;
	reg signed [35-1:0] dpn, next_dpn; 
	reg signed [35-1:0] cPn_sv, cPn_sv_next;
	reg signed [35-1:0] cPt_sv, cPt_sv_next;
	reg signed [35-1:0] Pn_x, Pn_x_next;
	reg signed [35-1:0] Pn_y, Pn_y_next;
	reg signed [35-1:0] Pt_x, Pt_x_next;
	reg signed [35-1:0] Pt_y, Pt_y_next;
	reg signed [35-1:0] cross1, cross2, cross1_next, cross2_next;
	reg signed [35-1:0] vt, next_vt;
	reg signed [35-1:0] dpt, next_dpt;
	wire signed [35-1:0] low, max, clp_out;
    wire signed [35-1:0] zero;
    assign zero = 35'd0;

	mult_gen_35x35 m1(.A(a1), .B(b1), .P(temp1));
	mult_gen_35x35 m2(.A(c1), .B(d1), .P(temp2));
	mult_gen_35x35 m3(.A(a2), .B(b2), .P(temp3));
	mult_gen_35x35 m4(.A(c2), .B(d2), .P(temp4));
	clamp cp1(.in(cPt_sv + dpt), .low(low), .max(max), .out(clp_out));
	
    assign e1 = temp1 + temp2;
    assign e2 = temp3 + temp4;

    assign r1_x = cPos_x - b1Pos_x;
    assign r1_y = cPos_y - b1Pos_y;
    assign r2_x = cPos_x - b2Pos_x;
    assign r2_y = cPos_y - b2Pos_y;
    assign tangent_x = cNorm_y;
    assign tangent_y = -cNorm_x;
    assign low = -(cPn_sv >>> 2);
    assign max = (cPn_sv >>> 2);

    //output 
    assign cPn_aft = cPn_sv; //(done_out) ? cPn_sv:35'd0;
    assign cPt_aft =  cPt_sv; //(done_out) ? cPt_sv:35'd0;
    assign b1Vel_x_aft = b1Vel_x_sv; //(done_out) ? b1Vel_x_sv:30'd0;
    assign b1Vel_y_aft = b1Vel_y_sv; //(done_out) ? b1Vel_y_sv:30'd0;
    assign b2Vel_x_aft = b2Vel_x_sv; //(done_out) ? b2Vel_x_sv:30'd0;
    assign b2Vel_y_aft = b2Vel_y_sv; //(done_out) ? b2Vel_y_sv:30'd0;
    assign b1AngVel_aft = b1AngVel_sv; //(done_out) ? b1AngVel_sv:22'd0;
    assign b2AngVel_aft = b2AngVel_sv; //(done_out) ? b2AngVel_sv:22'd0;


    always @(posedge clk or posedge rst) begin
    	if (rst) begin
    		state <= S0;
    		done_out <= 1'b0;
    		b1Vel_x_sv <= b1Vel_x;
    		b1Vel_y_sv <= b1Vel_y;
    		b2Vel_x_sv <= b2Vel_x;
    		b2Vel_y_sv <= b2Vel_y;
    		b1AngVel_sv <= b1AngVel;
    		b2AngVel_sv <= b2AngVel;
    		unuse <= 60'd0;
    		unuse2 <= 60'd0;
    		dvx <= 30'd0;
    		dvy <= 30'd0;
    		vn <= 35'd0;
    		dpn <= 35'd0;
    		cPn_sv <= cPn;
    		cPt_sv <= cPt;
    		Pn_x <= 35'd0;
    		Pn_y <= 35'd0;
    		Pt_x <= 35'd0;
    		Pt_y <= 35'd0;
    		cross1 <= 35'd0;
    		cross2 <= 35'd0;
    		vt <= 35'd0;
    		dpt <= 35'd0;

    	end
    	else begin
    		state <= next_state;
    		done_out <= 1'b0;
    		if (state == S15) begin
    			done_out <= 1'b1;
    		end
    		b1Vel_x_sv <= b1Vel_x_sv_next;
    		b1Vel_y_sv <= b1Vel_y_sv_next;
    		b2Vel_x_sv <= b2Vel_x_sv_next;
    		b2Vel_y_sv <= b2Vel_y_sv_next;
    		b1AngVel_sv <= b1AngVel_sv_next;
    		b2AngVel_sv <= b2AngVel_sv_next;
    		unuse <= unuse_next;
    		unuse2 <= unuse2_next;
    		dvx <= next_dvx;
    		dvy <= next_dvy;
    		vn <= next_vn;
    		vt <= next_vt;
    		dpn <= next_dpn;
    		dpt <= next_dpt;
    		cPn_sv <= cPn_sv_next;
    		cPt_sv <= cPt_sv_next;
    		Pn_x <= Pn_x_next;
    		Pn_y <= Pn_y_next;
    		Pt_x <= Pt_x_next;
    		Pt_y <= Pt_y_next;
    		cross1 <= cross1_next;
    		cross2 <= cross2_next;
    	end
    end

    always @(*) begin
    	a1 = 35'd0;
	  	b1 = 35'd0;
	  	c1 = 35'd0;
	  	d1 = 35'd0;
	  	a2 = 35'd0;
	  	b2 = 35'd0;
	  	c2 = 35'd0;
	  	d2 = 35'd0;
	  	unuse_next = e1;
	  	unuse2_next = e2;
	  	next_dvx = dvx;
	  	next_dvy = dvy;
	  	next_vn = vn;
	  	next_vt = vt;
	  	next_dpn = dpn;
	  	next_dpt = dpt;
	  	cPn_sv_next = cPn_sv;
	  	cPt_sv_next = cPt_sv;
	  	Pn_x_next = Pn_x;
	  	Pn_y_next = Pn_y;
	  	Pt_x_next = Pt_x;
	  	Pt_y_next = Pt_y;
	  	cross1_next = cross1;
	  	cross2_next = cross2;
	  	b1Vel_x_sv_next = b1Vel_x_sv;
	  	b1Vel_y_sv_next = b1Vel_y_sv;
	  	b2Vel_x_sv_next = b2Vel_x_sv;
	  	b2Vel_y_sv_next = b2Vel_y_sv;
	  	b1AngVel_sv_next = b1AngVel_sv;
	  	b2AngVel_sv_next = b2AngVel_sv;
        
    	case(state) 
    		S0: begin
    				next_state = S1;
    				a1 = -{{13{b1AngVel_sv[21]}}, b1AngVel_sv};//tp1
    				b1 = {{4{r1_y[30]}}, r1_y};
    				c1 = {{13{b1AngVel_sv[21]}}, b1AngVel_sv};//tp2
    				d1 = {{4{r1_x[30]}}, r1_x};

    				a2 = -{{13{b2AngVel_sv[21]}}, b2AngVel_sv};//tp3
    				b2 = {{4{r2_y[30]}}, r2_y};
    				c2 = {{13{b2AngVel_sv[21]}}, b2AngVel_sv};//tp4
    				d2 = {{4{r2_x[30]}}, r2_x};

    				next_dvx = b2Vel_x_sv + temp3[49:20] - b1Vel_x_sv - temp1[49:20];
    				next_dvy = b2Vel_y_sv + temp4[49:20] - b1Vel_y_sv - temp2[49:20];
    			end
    		S1: begin
    				next_state = S2;
                    a1 = {{5{dvx[29]}}, dvx};
    				b1 = {{13{cNorm_x[21]}}, cNorm_x};
    				c1 = {{5{dvy[29]}}, dvy};
    				d1 = {{13{cNorm_y[21]}}, cNorm_y};

    				next_vn = e1[54:20];
    			end
    		S2: begin
    				next_state = S3;
    				a1 = {{6{cMassNorm[28]}}, cMassNorm};
    				b1 = -vn;
    				c1 = {{6{cMassNorm[28]}}, cMassNorm};
    				d1 = { {4{cBias[30]}}, cBias};

    				next_dpn = e1[54:20];
    			end
    		S3: begin
    				next_state = S4;
//    				cPn_sv_next = (cPn_sv + dpn > zero) ? cPn_sv + dpn : 35'd0;
    				next_dpn = (dpn > zero) ? dpn: 35'd0;
    			end
    		S4: begin
    				next_state = S5;
    				a1 = {{13{cNorm_x[21]}}, cNorm_x};
    				b1 = dpn;
    				c1 = {{13{cNorm_y[21]}}, cNorm_y};
    				d1 = dpn;

    				Pn_x_next = temp1[54:20];
    				Pn_y_next = temp2[54:20];
    			end
    		S5: begin
    				next_state = S6;
    				a1 = {{4{r1_x[30]}}, r1_x};
    				b1 = Pn_y;
    				c1 = -{{4{r1_y[30]}}, r1_y};
    				d1 = Pn_x;

    				a2 = {{4{r2_x[30]}}, r2_x};
    				b2 = Pn_y;
    				c2 = -{{4{r2_y[30]}}, r2_y};
    				d2 = Pn_x;

    				cross1_next = e1[54:20];
    				cross2_next = e2[54:20];
    			end
    		S6: begin
    				next_state = S7;
    				a1 = Pn_x;
    				b1 = {{13{b1Inv_mass[21]}}, b1Inv_mass};
    				c1 = Pn_y;
    				d1 = {{13{b1Inv_mass[21]}}, b1Inv_mass};

    				a2 = Pn_x;
    				b2 = {{13{b2Inv_mass[21]}}, b2Inv_mass};
    				c2 = Pn_y;
    				d2 = {{13{b2Inv_mass[21]}}, b2Inv_mass};
    				
    				b1Vel_x_sv_next = b1Vel_x_sv - temp1[49:20];
    				b1Vel_y_sv_next = b1Vel_y_sv - temp2[49:20];
    				b2Vel_x_sv_next = b2Vel_x_sv + temp3[49:20];
    				b2Vel_y_sv_next = b2Vel_y_sv + temp4[49:20];
    			end
    		S7: begin
    				next_state = S8;
    				a1 = cross1;
    				b1 = {{13{b1Inv_I[21]}}, b1Inv_I};
    				c1 = cross2;
    				d1 = {{13{b2Inv_I[21]}}, b2Inv_I};
    				
    				b1AngVel_sv_next = b1AngVel_sv - temp1[41:20];
    				b2AngVel_sv_next = b2AngVel_sv + temp2[41:20];
    			end
    		S8: begin
    				next_state = S9;
    				a1 = -{{13{b1AngVel_sv[21]}}, b1AngVel_sv};//tp1
                    b1 = {{4{r1_y[30]}}, r1_y};
                    c1 = {{13{b1AngVel_sv[21]}}, b1AngVel_sv};//tp2
                    d1 = {{4{r1_x[30]}}, r1_x};

                    a2 = -{{13{b2AngVel_sv[21]}}, b2AngVel_sv};//tp3
                    b2 = {{4{r2_y[30]}}, r2_y};
                    c2 = {{13{b2AngVel_sv[21]}}, b2AngVel_sv};//tp4
                    d2 = {{4{r2_x[30]}}, r2_x};

                    next_dvx = b2Vel_x_sv + temp3[49:20] - b1Vel_x_sv - temp1[49:20];
                    next_dvy = b2Vel_y_sv + temp4[49:20] - b1Vel_y_sv - temp2[49:20];

    			end
    		S9: begin
    				next_state = S10;
    				a1 = {{5{dvx[29]}}, dvx};
    				b1 = {{13{tangent_x[21]}}, tangent_x};
    				c1 = {{5{dvy[29]}}, dvy};
    				d1 = {{13{tangent_y[21]}}, tangent_y};

    				next_vt = e1[54:20];
    			end
    		S10: begin
    				next_state = S11;
    				a1 = {{6{cMassTang[28]}}, cMassTang};
    				b1 = -vt;

    				next_dpt = temp1[54:20];
    			end
    		S11: begin
    				next_state = S12;
//    				cPt_sv_next = clp_out;
    				next_dpt = clp_out;

    			end
    		S12: begin
    				next_state = S13;
    				a1 = {{13{tangent_x[21]}}, tangent_x};
    				b1 = dpt;
    				c1 = {{13{tangent_y[21]}}, tangent_y};
    				d1 = dpt;

    				Pt_x_next = temp1[54:20];
    				Pt_y_next = temp2[54:20];
    			end
    		S13: begin
    				next_state = S14;
    				a1 = {{4{r1_x[30]}}, r1_x};
    				b1 = Pt_y;
    				c1 = -{{4{r1_y[30]}}, r1_y};
    				d1 = Pt_x;

    				a2 = {{4{r2_x[30]}}, r2_x};
    				b2 = Pt_y;
    				c2 = -{{4{r2_y[30]}}, r2_y};
    				d2 = Pt_x;

    				cross1_next = e1[54:20];
    				cross2_next = e2[54:20];

    			end
    		S14: begin
    				next_state = S15;
    				a1 = Pt_x;
    				b1 = {{13{b1Inv_mass[21]}}, b1Inv_mass};
    				c1 = Pt_y;
    				d1 = {{13{b1Inv_mass[21]}}, b1Inv_mass};

    				a2 = Pt_x;
    				b2 = {{13{b2Inv_mass[21]}}, b2Inv_mass};
    				c2 = Pt_y;
    				d2 = {{13{b2Inv_mass[21]}}, b2Inv_mass};
    				
    				b1Vel_x_sv_next = b1Vel_x_sv - temp1[49:20];
    				b1Vel_y_sv_next = b1Vel_y_sv - temp2[49:20];
    				b2Vel_x_sv_next = b2Vel_x_sv + temp3[49:20];
    				b2Vel_y_sv_next = b2Vel_y_sv + temp4[49:20];
    			end
    		S15: begin
    				next_state = S15;
    				a1 = cross1;
    				b1 = {{13{b1Inv_I[21]}}, b1Inv_I};
    				c1 = cross2;
    				d1 = {{13{b2Inv_I[21]}}, b2Inv_I};
    				
    				if (!done_out) begin
    				  b1AngVel_sv_next = b1AngVel_sv - temp1[41:20];
    				  b2AngVel_sv_next = b2AngVel_sv + temp2[41:20];
    				end
    			end
    		
    	endcase
    end



endmodule

module clamp(in, low, max, out);
	input signed [35-1:0] in, low, max;
	output reg signed [35-1:0] out;

	always @(*) begin
		if (in <= low) begin
			out = low;
		end else if (in >= max) begin
			out = max;
		end else begin
			out = in;
		end
	end

endmodule


