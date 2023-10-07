module Collide(
  input clk,
  input clk_collide,
  input clk_Pen,
  input rst_op,
  input [3:0] A_nth,
  input [3:0] B_nth,
  input signed [18:0] A0_x, input signed [18:0] A0_y,
  input signed [18:0] A1_x, input signed [18:0] A1_y,
  input signed [18:0] A2_x, input signed [18:0] A2_y,
  input signed [18:0] A3_x, input signed [18:0] A3_y,
  input signed [18:0] B0_x, input signed [18:0] B0_y,
  input signed [18:0] B1_x, input signed [18:0] B1_y,
  input signed [18:0] B2_x, input signed [18:0] B2_y,
  input signed [18:0] B3_x, input signed [18:0] B3_y,
  input signed [18:0] Apos_x, input signed [18:0] Apos_y,
  input signed [18:0] Bpos_x, input signed [18:0] Bpos_y,
  output signed [19-1:0] contact1Pt_x, output signed [19-1:0] contact1Pt_y,
  output signed [19-1:0] contact2Pt_x, output signed [19-1:0] contact2Pt_y,
  output reg signed [10-1:0] contact1Norm_x, output reg signed [10-1:0] contact1Norm_y,
  output reg signed [10-1:0] contact2Norm_x, output reg signed [10-1:0] contact2Norm_y,
  output signed [19-1:0] contact1Penetration, output signed [19-1:0] contact2Penetration,
  output [2-1:0] contactNum,
  output contact,
  output done_out
  );
  parameter INIT = 3'd0;
  parameter FLP = 3'd1;
  parameter CIE = 3'd2;
  parameter CLIP = 3'd3;
  parameter DONE = 3'd4;
  
  // state
  reg [3-1:0] state, next_state;
  reg start_flp;
  // for caculate
  wire signed [19-1:0] temp_Ax[0:4-1], temp_Ay[0:4-1];
  wire signed [19-1:0] temp_Bx[0:4-1], temp_By[0:4-1];
  // A normal
  reg signed [10-1:0] Anorm_x[0:4-1], Anorm_y[0:4-1];
  // B normal
  reg signed [10-1:0] Bnorm_x[0:4-1], Bnorm_y[0:4-1];
  // FindLeastPenetration output
  reg signed [19-1:0] PenetrationAB, PenetrationBA, PenetrationAB_next, PenetrationBA_next;
  wire signed [19-1:0] PenAB, PenBA;
  reg signed [10-1:0] Norm_x_AB, Norm_y_AB, Norm_x_BA, Norm_y_BA;
  reg signed [10-1:0] Norm_x_AB_next, Norm_y_AB_next, Norm_x_BA_next, Norm_y_BA_next;
  wire signed [10-1:0] NormxAB, NormyAB, NormxBA, NormyBA;
  reg [2-1:0] IndexAB, IndexBA, IndexAB_next, IndexBA_next;
  wire [2-1:0] iAB, iBA;
  reg AB_done, AB_done_next, BA_done, BA_done_next;
  wire AB_d, BA_d, flp_done;
  // ComputeIncidentFace
  reg signed [10-1:0] referenceNorm_x, referenceNorm_y;
  reg [2-1:0] referenceIndex;
  reg flip;
  reg start_cie;
//  reg cie_done, cie_done_next;
  wire cie_done;
  reg signed [10-1:0] incidentNorm_x, incidentNorm_y, incidentNorm_x_next, incidentNorm_y_next;
  wire signed [10-1:0] outNorm_x, outNorm_y;
  reg  [2-1:0] incidentIndex, incidentIndex_next;
  wire [2-1:0] outIndex;
  reg signed [10-1:0] input_normX[0:4-1], input_normY[0:4-1];
  // Clip
  reg start_clip;
  wire clip_done;
  reg signed [10-1:0] side1Norm_x, side1Norm_y, side2Norm_x, side2Norm_y;
  reg signed [19-1:0] pos1In_x, pos1In_y;
  reg signed [19-1:0] pos2In_x, pos2In_y;
  reg signed [19-1:0] vI_x[0:4-1], vI_y[0:4-1];
  reg signed [19-1:0] vR_x[0:4-1], vR_y[0:4-1];
    
  FindLeastPenetration AB(
    .clk(clk_Pen),
    .start(start_flp),
    .A0_x(A0_x), .A0_y(A0_y),
    .A1_x(A1_x), .A1_y(A1_y),
    .A2_x(A2_x), .A2_y(A2_y),
    .A3_x(A3_x), .A3_y(A3_y),
    .B0_x(B0_x), .B0_y(B0_y),
    .B1_x(B1_x), .B1_y(B1_y),
    .B2_x(B2_x), .B2_y(B2_y),
    .B3_x(B3_x), .B3_y(B3_y),
    .Bpos_x(Bpos_x), .Bpos_y(Bpos_y),
    .norm0_x(Anorm_x[0]), .norm0_y(Anorm_y[0]),
    .norm1_x(Anorm_x[1]), .norm1_y(Anorm_y[1]),
    .norm2_x(Anorm_x[2]), .norm2_y(Anorm_y[2]),
    .norm3_x(Anorm_x[3]), .norm3_y(Anorm_y[3]),
    .bestPenetration(PenAB),//output
    .bestNorm_x(NormxAB),
    .bestNorm_y(NormyAB),
    .bestIndex(iAB),
    .done_out(AB_d)
    );

  FindLeastPenetration BA(
    .clk(clk_Pen),
    .start(start_flp),
    .A0_x(B0_x), .A0_y(B0_y),
    .A1_x(B1_x), .A1_y(B1_y),
    .A2_x(B2_x), .A2_y(B2_y),
    .A3_x(B3_x), .A3_y(B3_y),
    .B0_x(A0_x), .B0_y(A0_y),
    .B1_x(A1_x), .B1_y(A1_y),
    .B2_x(A2_x), .B2_y(A2_y),
    .B3_x(A3_x), .B3_y(A3_y),
    .Bpos_x(Apos_x), .Bpos_y(Apos_y),
    .norm0_x(Bnorm_x[0]), .norm0_y(Bnorm_y[0]),
    .norm1_x(Bnorm_x[1]), .norm1_y(Bnorm_y[1]),
    .norm2_x(Bnorm_x[2]), .norm2_y(Bnorm_y[2]),
    .norm3_x(Bnorm_x[3]), .norm3_y(Bnorm_y[3]),
    .bestPenetration(PenBA),
    .bestNorm_x(NormxBA),
    .bestNorm_y(NormyBA),
    .bestIndex(iBA),
    .done_out(BA_d)
    );
    
  ComputeIncidentEdge cie0(
        .clk(clk_Pen),
        .start(start_cie),
        .referenceNorm_x(referenceNorm_x),
        .referenceNorm_y(referenceNorm_y),
        .norm0_x(input_normX[0]), .norm0_y(input_normY[0]),
        .norm1_x(input_normX[1]), .norm1_y(input_normY[1]),
        .norm2_x(input_normX[2]), .norm2_y(input_normY[2]),
        .norm3_x(input_normX[3]), .norm3_y(input_normY[3]),
        .incidentNorm_x(outNorm_x),//output
        .incidentNorm_y(outNorm_y),
        .incidentIndex(outIndex),
        .done_out(cie_done)
    );

  Clip clip0(
    .clk(clk_collide),
    .start(start_clip),
    .side1Norm_x(side1Norm_x), .side1Norm_y(side1Norm_y),
    .side2Norm_x(side2Norm_x), .side2Norm_y(side2Norm_y),
    .incidentNorm_x(incidentNorm_x), .incidentNorm_y(incidentNorm_y),
    .pos1_x(vI_x[incidentIndex]), .pos1_y(vI_y[incidentIndex]),
    .pos2_x(vI_x[(incidentIndex+1)%4]), .pos2_y(vI_y[(incidentIndex+1)%4]),
    .refer1Pos_x(vR_x[referenceIndex]), .refer1Pos_y(vR_y[referenceIndex]),
    .refer2Pos_x(vR_x[(referenceIndex+1)%4]), .refer2Pos_y(vR_y[(referenceIndex+1)%4]),
    .referNorm_x(referenceNorm_x), .referNorm_y(referenceNorm_y),
    .contact1Pt_x(contact1Pt_x), .contact1Pt_y(contact1Pt_y),//output
    .contact2Pt_x(contact2Pt_x), .contact2Pt_y(contact2Pt_y),
    .contact1Penetration(contact1Penetration), .contact2Penetration(contact2Penetration),
    .contactNum(contactNum),
    .done_out(clip_done)
    );

  assign flp_done = (AB_done & BA_done) ? 1'b1:1'b0;
  assign contact = (PenetrationAB[18] & PenetrationBA[18]);
  assign done_out = (state == DONE) ? 1'b1:1'b0;

  always @(posedge clk or posedge rst_op) begin
    if (rst_op) begin
        PenetrationAB <= 19'd0;
        PenetrationBA <= 19'd0;
        Norm_x_AB <= 10'd0;
        Norm_y_AB <= 10'd0;
        Norm_x_BA <= 10'd0;
        Norm_y_BA <= 10'd0;
        IndexAB <= 2'd0;
        IndexBA <= 2'd0;
        AB_done <= 1'd0;
        BA_done <= 1'd0;
    end else begin
        PenetrationAB <= PenetrationAB_next;
        PenetrationBA <= PenetrationBA_next;
        Norm_x_AB <= Norm_x_AB_next;
        Norm_y_AB <= Norm_y_AB_next;
        Norm_x_BA <= Norm_x_BA_next;
        Norm_y_BA <= Norm_y_BA_next;
        IndexAB <= IndexAB_next;
        IndexBA <= IndexBA_next;
        AB_done <= AB_done_next;
        BA_done <= BA_done_next;
    end
  end
  
  always @(*) begin
    PenetrationAB_next = PenetrationAB;
    PenetrationBA_next = PenetrationBA;
    Norm_x_AB_next = Norm_x_AB;
    Norm_y_AB_next = Norm_y_AB;
    Norm_x_BA_next = Norm_x_BA;
    Norm_y_BA_next = Norm_y_BA;
    IndexAB_next = IndexAB;
    IndexBA_next = IndexBA;
    AB_done_next = AB_done;
    BA_done_next = BA_done;
    if (AB_d & BA_d) begin
    
    PenetrationAB_next = PenAB;
        PenetrationBA_next = PenBA;
        Norm_x_AB_next = NormxAB;
        Norm_y_AB_next = NormyAB;
        Norm_x_BA_next = NormxBA;
        Norm_y_BA_next = NormyBA;
        IndexAB_next = iAB;
        IndexBA_next = iBA;
        AB_done_next = AB_d;
        BA_done_next = BA_d;
    end
  end
  
  always @(posedge clk or posedge rst_op) begin
    if (rst_op) begin
        incidentNorm_x <= 10'd0;
        incidentNorm_y <= 10'd0;
        incidentIndex <= 2'd0;
//        cie_done <= 1'b0;
    end else begin
        incidentNorm_x <= incidentNorm_x_next;
        incidentNorm_y <= incidentNorm_y_next;
        incidentIndex <= incidentIndex_next;
//        cie_done <= cie_done_next;
    end
  end
  
  always @(*) begin
    incidentNorm_x_next = incidentNorm_x;
    incidentNorm_y_next = incidentNorm_y;
    incidentIndex_next = incidentIndex;
//    cie_done_next = cie_done;
    if (cie_done) begin
        incidentNorm_x_next = outNorm_x;
        incidentNorm_y_next = outNorm_y;
        incidentIndex_next  = outIndex;
//        cie_done_next = cie_d;
    end
  end

  always @ (*) begin //cie
    if (PenetrationAB >= PenetrationBA) begin
      referenceNorm_x = Norm_x_AB;
      referenceNorm_y = Norm_y_AB;
      referenceIndex = IndexAB;
      input_normX[0] = Bnorm_x[0];
      input_normX[1] = Bnorm_x[1];
      input_normX[2] = Bnorm_x[2];
      input_normX[3] = Bnorm_x[3];
      input_normY[0] = Bnorm_y[0];
      input_normY[1] = Bnorm_y[1];
      input_normY[2] = Bnorm_y[2];
      input_normY[3] = Bnorm_y[3];
      flip = 1'b0;
    end else begin
      referenceNorm_x = Norm_x_BA;
      referenceNorm_y = Norm_y_BA;
      referenceIndex = IndexBA;
      input_normX[0] = Anorm_x[0];
      input_normX[1] = Anorm_x[1];
      input_normX[2] = Anorm_x[2];
      input_normX[3] = Anorm_x[3];
      input_normY[0] = Anorm_y[0];
      input_normY[1] = Anorm_y[1];
      input_normY[2] = Anorm_y[2];
      input_normY[3] = Anorm_y[3];
      flip = 1'b1;
    end
  end

  always @(*) begin
    if(flip)begin
        vI_x[0] = A0_x;
        vI_x[1] = A1_x;
        vI_x[2] = A2_x;
        vI_x[3] = A3_x;
    end else begin
        vI_x[0] = B0_x;
        vI_x[1] = B1_x;
        vI_x[2] = B2_x;
        vI_x[3] = B3_x;
    end
  end
  
  always @(*) begin
      if(flip)begin
        vI_y[0] = A0_y;
        vI_y[1] = A1_y;
        vI_y[2] = A2_y;
        vI_y[3] = A3_y;
      end else begin
        vI_y[0] = B0_y;
        vI_y[1] = B1_y;
        vI_y[2] = B2_y;
        vI_y[3] = B3_y;
      end
    end
  always @(*) begin
    if(flip)begin
        vR_x[0] = B0_x;
        vR_x[1] = B1_x;
        vR_x[2] = B2_x;
        vR_x[3] = B3_x;
    end else begin
        vR_x[0] = A0_x;
        vR_x[1] = A1_x;
        vR_x[2] = A2_x;
        vR_x[3] = A3_x;
    end
  end
  
  always @(*) begin
      if(flip)begin
        vR_y[0] = B0_y;
        vR_y[1] = B1_y;
        vR_y[2] = B2_y;
        vR_y[3] = B3_y;
      end else begin
        vR_y[0] = A0_y;
        vR_y[1] = A1_y;
        vR_y[2] = A2_y;
        vR_y[3] = A3_y;
      end
    end
    
  always @(*) begin
        if(flip)begin
            contact1Norm_x = -referenceNorm_x;
            contact1Norm_y = -referenceNorm_y;
            contact2Norm_x = -referenceNorm_x;
            contact2Norm_y = -referenceNorm_y;
        end else begin
            contact1Norm_x = referenceNorm_x;
            contact1Norm_y = referenceNorm_y;
            contact2Norm_x = referenceNorm_x;
            contact2Norm_y = referenceNorm_y;
        end
      end
  always @(*) begin
      if(flip)begin
        side1Norm_x = Bnorm_x[(referenceIndex-1)%4];
        side1Norm_y = Bnorm_y[(referenceIndex-1)%4];
        side2Norm_x = Bnorm_x[(referenceIndex+1)%4];
        side2Norm_y = Bnorm_y[(referenceIndex+1)%4];
      end else begin
        side1Norm_x = Anorm_x[(referenceIndex-1)%4];
        side1Norm_y = Anorm_y[(referenceIndex-1)%4];
        side2Norm_x = Anorm_x[(referenceIndex+1)%4];      // wrong pointer
        side2Norm_y = Anorm_y[(referenceIndex+1)%4]; 
      end
    end

   always @ ( posedge clk or posedge rst_op) begin
    if (rst_op) begin
      state <= INIT;
      start_flp <= 1'b1;
      start_cie <= 1'b1;
      start_clip <= 1'b1;
    end else begin
      state <= next_state;
      if (state == FLP) begin
        start_flp <= 1'b0;
      end
      else if (state == CIE) begin
        start_cie <= 1'b0;
      end
      else if (state == CLIP) begin
        start_clip <= 1'b0;
      end
    end
  end

  always @ ( * ) begin
    next_state = state;
    
    case(state)
        INIT : begin
            next_state = FLP;
            end
        FLP : begin
            if (flp_done) begin
                next_state = DONE;
                if (contact) begin
                  next_state = CIE;
                end
              end
            end
        CIE : begin
            if (cie_done) begin
                next_state = CLIP;
              end
            end
        CLIP : begin
            if (clip_done) begin
                next_state = DONE;
              end
            end
        DONE : begin
            next_state = DONE;
            end
        default : begin
            next_state = state;
        end
    endcase
  end

  assign temp_Ax[0] = A0_y - A1_y;
  assign temp_Ax[1] = A1_y - A2_y;
  assign temp_Ax[2] = A2_y - A3_y;
  assign temp_Ax[3] = A3_y - A0_y;
  assign temp_Ay[0] = A1_x - A0_x;
  assign temp_Ay[1] = A2_x - A1_x;
  assign temp_Ay[2] = A3_x - A2_x;
  assign temp_Ay[3] = A0_x - A3_x;

  assign temp_Bx[0] = B0_y - B1_y;
  assign temp_Bx[1] = B1_y - B2_y;
  assign temp_Bx[2] = B2_y - B3_y;
  assign temp_Bx[3] = B3_y - B0_y;
  assign temp_By[0] = B1_x - B0_x;
  assign temp_By[1] = B2_x - B1_x;
  assign temp_By[2] = B3_x - B2_x;
  assign temp_By[3] = B0_x - B3_x;

  always @ ( * ) begin
    if (A_nth == 4'd0) begin
      Anorm_x[0] = {1'b0, 1'b0, 8'b0}; Anorm_y[0] = {1'b1, 1'b1, 8'b0};
      Anorm_x[1] = {1'b1, 1'b1, 8'b0}; Anorm_y[1] = {1'b0, 1'b0, 8'b0};
      Anorm_x[2] = {1'b0, 1'b0, 8'b0}; Anorm_y[2] = {1'b0, 1'b1, 8'b0};
      Anorm_x[3] = {1'b0, 1'b1, 8'b0}; Anorm_y[3] = {1'b0, 1'b0, 8'b0};
    end else if (A_nth == 4'd1) begin
      Anorm_x[0] = temp_Ax[0][14:5]; Anorm_y[0] = temp_Ay[0][14:5];
      Anorm_x[1] = temp_Ax[1][14:5]; Anorm_y[1] = temp_Ay[1][14:5];
      Anorm_x[2] = temp_Ax[2][14:5]; Anorm_y[2] = temp_Ay[2][14:5];
      Anorm_x[3] = temp_Ax[3][14:5]; Anorm_y[3] = temp_Ay[3][14:5];
    end else if (A_nth == 4'd2) begin
      Anorm_x[0] = temp_Ax[0][14:5]; Anorm_y[0] = temp_Ay[0][14:5];
      Anorm_x[1] = temp_Ax[1][14:5]; Anorm_y[1] = temp_Ay[1][14:5];
      Anorm_x[2] = temp_Ax[2][14:5]; Anorm_y[2] = temp_Ay[2][14:5];
      Anorm_x[3] = temp_Ax[3][14:5]; Anorm_y[3] = temp_Ay[3][14:5];
    end else if (A_nth == 4'd6) begin
      Anorm_x[0] = temp_Ax[0][15:6]; Anorm_y[0] = temp_Ay[0][15:6];
      Anorm_x[1] = temp_Ax[1][12:3]; Anorm_y[1] = temp_Ay[1][12:3];
      Anorm_x[2] = temp_Ax[2][15:6]; Anorm_y[2] = temp_Ay[2][15:6];
      Anorm_x[3] = temp_Ax[3][12:3]; Anorm_y[3] = temp_Ay[3][12:3];
    end else if (A_nth == 4'd9) begin
      Anorm_x[0] = temp_Ax[0][15:6]; Anorm_y[0] = temp_Ay[0][15:6];
      Anorm_x[1] = temp_Ax[1][12:3]; Anorm_y[1] = temp_Ay[1][12:3];
      Anorm_x[2] = temp_Ax[2][15:6]; Anorm_y[2] = temp_Ay[2][15:6];
      Anorm_x[3] = temp_Ax[3][12:3]; Anorm_y[3] = temp_Ay[3][12:3];
    end else begin
      Anorm_x[0] = temp_Ax[0][12:3]; Anorm_y[0] = temp_Ay[0][12:3];
      Anorm_x[1] = temp_Ax[1][15:6]; Anorm_y[1] = temp_Ay[1][15:6];
      Anorm_x[2] = temp_Ax[2][12:3]; Anorm_y[2] = temp_Ay[2][12:3];
      Anorm_x[3] = temp_Ax[3][15:6]; Anorm_y[3] = temp_Ay[3][15:6];
    end
  end

  always @ ( * ) begin
    if (B_nth == 4'd0) begin
      Bnorm_x[0] = {1'b0, 1'b0, 8'b0}; Bnorm_y[0] = {1'b1, 1'b1, 8'b0};
      Bnorm_x[1] = {1'b1, 1'b1, 8'b0}; Bnorm_y[1] = {1'b0, 1'b0, 8'b0};
      Bnorm_x[2] = {1'b0, 1'b0, 8'b0}; Bnorm_y[2] = {1'b0, 1'b1, 8'b0};
      Bnorm_x[3] = {1'b0, 1'b1, 8'b0}; Bnorm_y[3] = {1'b0, 1'b0, 8'b0};
    end else if (B_nth == 4'd1) begin
      Bnorm_x[0] = temp_Bx[0][14:5]; Bnorm_y[0] = temp_By[0][14:5];
      Bnorm_x[1] = temp_Bx[1][14:5]; Bnorm_y[1] = temp_By[1][14:5];
      Bnorm_x[2] = temp_Bx[2][14:5]; Bnorm_y[2] = temp_By[2][14:5];
      Bnorm_x[3] = temp_Bx[3][14:5]; Bnorm_y[3] = temp_By[3][14:5];
    end else if (B_nth == 4'd2) begin
      Bnorm_x[0] = temp_Bx[0][14:5]; Bnorm_y[0] = temp_By[0][14:5];
      Bnorm_x[1] = temp_Bx[1][14:5]; Bnorm_y[1] = temp_By[1][14:5];
      Bnorm_x[2] = temp_Bx[2][14:5]; Bnorm_y[2] = temp_By[2][14:5];
      Bnorm_x[3] = temp_Bx[3][14:5]; Bnorm_y[3] = temp_By[3][14:5];
    end else if (B_nth == 4'd6) begin
      Bnorm_x[0] = temp_Bx[0][15:6]; Bnorm_y[0] = temp_By[0][15:6];
      Bnorm_x[1] = temp_Bx[1][12:3]; Bnorm_y[1] = temp_By[1][12:3];
      Bnorm_x[2] = temp_Bx[2][15:6]; Bnorm_y[2] = temp_By[2][15:6];
      Bnorm_x[3] = temp_Bx[3][12:3]; Bnorm_y[3] = temp_By[3][12:3];
    end else if (B_nth == 4'd9) begin
      Bnorm_x[0] = temp_Bx[0][15:6]; Bnorm_y[0] = temp_By[0][15:6];
      Bnorm_x[1] = temp_Bx[1][12:3]; Bnorm_y[1] = temp_By[1][12:3];
      Bnorm_x[2] = temp_Bx[2][15:6]; Bnorm_y[2] = temp_By[2][15:6];
      Bnorm_x[3] = temp_Bx[3][12:3]; Bnorm_y[3] = temp_By[3][12:3];
    end else begin
      Bnorm_x[0] = temp_Bx[0][12:3]; Bnorm_y[0] = temp_By[0][12:3];
      Bnorm_x[1] = temp_Bx[1][15:6]; Bnorm_y[1] = temp_By[1][15:6];
      Bnorm_x[2] = temp_Bx[2][12:3]; Bnorm_y[2] = temp_By[2][12:3];
      Bnorm_x[3] = temp_Bx[3][15:6]; Bnorm_y[3] = temp_By[3][15:6];
    end
  end

endmodule
