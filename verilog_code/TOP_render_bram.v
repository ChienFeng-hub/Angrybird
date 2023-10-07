`timescale 1ns / 1ps

`define INIT 4'd0
`define RENDER 4'd1
`define BROADPHASE 4'd2
`define ADDFORCE 4'd3
`define PRESTEP 4'd4
`define APPLYIMPULSE 4'd5
`define ADDVEL 4'd6
`define ADDVEL2 4'd7
`define DESTROY 4'd8
`define PlayerCTRL 4'd9
`define END 4'd10

module Top(
    input clk,
    input rst,
    input slave_ready,
    output reg master_ready,
    output signed [12-1:0] vertice_x,
    output signed [12-1:0] vertice_y,
    output [4-1:0] nth_body,
    output [2-1:0] i_vertex,
//    output [4-1:0] state_
    inout PS2_CLK,
    inout PS2_DATA
    );



    parameter size = 11;
    parameter bodyNum = 4'd7;
    parameter gravity = 31'b0_0000000000_01010000000000000000; //0.01010
    parameter fps = 5;
    parameter depth = 49;
    parameter iteration = 30;

    parameter constraint_x_up = 10'd150;
    parameter constraint_x_down = 10'd110;
    parameter constraint_y_up = 10'd330;
    parameter constraint_y_down = 10'd290;

    reg [2-1:0] typ[0:size-1];
    reg [0:size-1] alive;
    reg signed [31-1:0] posX[0:size-1];
    reg signed [31-1:0] posY[0:size-1];
    reg signed [31-1:0] velX[0:size-1]; 
    reg signed [31-1:0] velY[0:size-1]; 
    reg signed [22-1:0] rad[0:size-1];
    reg signed [22-1:0] angVel[0:size-1];
    reg signed [18:0] verticesX[0:4*size-1];
    reg signed [18:0] verticesY[0:4*size-1];
    
//    reg signed [19-1:0] contactPt_x[0:depth]; 
//    reg signed [19-1:0] contactPt_y[0:depth]; 
//     reg signed [10-1:0] contactNorm_x[0:depth];
//     reg signed [10-1:0] contactNorm_y[0:depth];
//     reg signed [19-1:0] contactPenetration[0:depth];
    reg [4-1:0] contact_b1 [0:depth];
    reg [4-1:0] contact_b2 [0:depth];

    // reg signed [29-1:0] contactMassNorm[0:depth];
    // reg signed [29-1:0] contactMassTang[0:depth];
    // reg signed [31-1:0] contactBias[0:depth];

    // reg signed [35-1:0] contactPn [0:depth];
    // reg signed [35-1:0] contactPt [0:depth];

    reg [7-1:0] contactNum = 0;
    wire contact;

    reg [7-1:0] iter;
    reg [4-1:0] state, next_state;
    wire data_ready;
    wire rst_op;
    assign rst_op = rst;
    assign state_ = state;
    wire [12-1:0] onehot;
    // =================== Memory Variables ===================//

    reg [3:0] hold_pre, next_hold_pre;
    reg flag_pre;

    reg [3:0] hold_im, next_hold_im;
    reg flag_im;

      // ---------- contact point ----------//
    reg wea_conpt, web_conpt;
    reg [5:0] conpt_x1_addr, conpt_x2_addr, conpt_y1_addr, conpt_y2_addr;
    wire signed [19-1:0] contactPt_x_sub1, contactPt_x_sub2, conpt_x_temp;
    wire signed [19-1:0] contactPt_y_sub1, contactPt_y_sub2, conpt_y_temp;

      // ---------- contact normal ----------//
    reg wea_connom, web_connom;
    reg [5:0] connom_x1_addr, connom_x2_addr, connom_y1_addr, connom_y2_addr;
    wire signed [10-1:0] contactNorm_x_sub1, contactNorm_x_sub2, connom_x_temp;
    wire signed [10-1:0] contactNorm_y_sub1, contactNorm_y_sub2, connom_y_temp;
    

      // ---------- contact penetration ----------//
    reg wea_conpen, web_conpen;
    reg [5:0] conpen_1_addr, conpen_2_addr;
    wire signed [19-1:0] contactPen_sub1, contactPen_sub2, conpen_temp;

      // ---------- contact mass normal ----------//
    reg wea_conmsn;
    reg [5:0] conmsn_1_addr;
    wire signed [29-1:0] contactMassNorm;

      // ---------- contact mass tangent ----------//
    reg wea_conmtan;
    reg [5:0] conmtan_1_addr;
    wire signed [29-1:0] contactMassTang;

      // ---------- contact bias ----------//
    reg wea_conbias;
    reg [5:0] conbias_1_addr;
    wire signed [31-1:0] contactBias;

      // ---------- contact Pn Pt ----------//
    reg wea_conPn, wea_conPt;
    reg [5:0] conPn_1_addr, conPt_1_addr;
    wire signed [35-1:0] cPn_in, cPt_in;
    wire signed [35-1:0] contactPn, contactPt;
    
    // =========================================================//


    // ---------- Mouse--------//

    wire [9 : 0] MOUSE_X_POS , MOUSE_Y_POS;
    wire [3:0] MOUSE_Z_POS;
    wire MOUSE_LEFT , MOUSE_MIDDLE , MOUSE_RIGHT , MOUSE_NEW_EVENT;
    
    // ---------- BroadPhase--------//

    reg [4-1:0] nth_b1, nth_b2;
    reg start_collide;
    wire collide_done;
    
    
    wire signed [19-1:0] contact1Pt_x, contact1Pt_y;
    wire signed [19-1:0] contact2Pt_x, contact2Pt_y;
    wire signed [10-1:0] contact1Norm_x, contact1Norm_y;
    wire signed [10-1:0] contact2Norm_x, contact2Norm_y;
    wire signed [19-1:0] contact1Penetration, contact2Penetration;
    wire [2-1:0] cNum;

    // ---------- PreStep --------- //

    reg [7-1:0] cnt_p;
    reg start_prestep;
    wire prestep_done;
    reg signed [22-1:0] Inv_mass [0:9] ;
    reg signed [22-1:0] Inv_I [0:9] ;
    wire signed [29-1:0] cMassNorm, cMassTang;
    wire signed [31-1:0] cBias;

    // ---------- APPLYIMPULSE --------- //

    reg [7-1:0] cnt_im;
    reg start_impulse;
    wire impulse_done;
    wire signed [35-1:0]cPn_aft, cPt_aft;
    wire signed [30-1:0]b1Vel_x_aft, b1Vel_y_aft, b2Vel_x_aft, b2Vel_y_aft;
    wire signed [22-1:0]b1AngVel_aft, b2AngVel_aft;

    // ---------- Render ---------- //

    reg state_change;
    reg flag;
    reg [4-1:0] nth_r;
    reg [1:0] i_r;
    reg [6-1:0] cnt_r;
    wire signed [19-1:0] ver_x_render, ver_y_render;
    
    // ---------- PlayerCtrl ---------- //
    
    wire signed [31-1:0] origin_x = {1'b0,10'd130,20'b0};
    wire signed [31-1:0] origin_y = {1'b0,10'd310,20'b0};

    // ---------- DESTROY ---------- //

    reg clear;


      //------------------------------------------------------------------//
     //---------------------------Pointer Ctrl---------------------------//
    //------------------------------------------------------------------//

    always @ (posedge clk_25Mhz or posedge rst_op) begin // handshaking
      if (rst_op) begin
        master_ready <= 1'b0;
      end else begin
        master_ready <= 1'b0;
        if (state == `RENDER ) begin
          if (slave_ready) begin
            master_ready <= 1'b0;
          end else begin
            master_ready <= 1'b1;
          end
        end
      end
    end
    
    always @ (posedge master_ready or posedge clear or posedge rst_op) begin // render manipulate
      if ( rst_op | clear) begin
        nth_r <= 4'b0;
        i_r <= 2'b0;
        cnt_r <= 6'b0;
      end else begin
        nth_r <= nth_r;
        i_r <= i_r;
        cnt_r <= cnt_r;
        if (state == `RENDER) begin
          if (master_ready) begin
            if (cnt_r == 0) begin
                cnt_r <= cnt_r + 1;
            end else begin
                cnt_r <= cnt_r + 1;
                {nth_r, i_r} <= {nth_r, i_r} + 1'b1;
            end
          end
        end
      end
      
    end
    
    always @ (posedge clk) begin // b1 b2
      if (state == `BROADPHASE) begin
        start_collide <= 1'b0;
        if (collide_done) begin
          start_collide <= 1'b1;
          nth_b1 <= nth_b1;
          nth_b2 <= nth_b2;
          if (nth_b2 + 4'd1 == bodyNum) begin
            nth_b1 <= nth_b1 + 4'd1;
            nth_b2 <= nth_b1 + 4'd2;
          end
          else begin
            nth_b2 <= nth_b2 + 4'd1;
          end
        end
      end 
      else begin
        nth_b1 <= 4'd0;
        nth_b2 <= 4'd1;
        start_collide <= 1'b1;
      end
    end


    always@(posedge clk or posedge rst)begin // flag prestep ctrl
        flag_pre <= flag_pre;
        if(rst) begin
          flag_pre <= 1'b0;
        end
        else if(prestep_done)begin
          flag_pre <= 1'b1;
        end
        else if(hold_pre == 4'd3)begin
          flag_pre <= 1'b0;
        end
    end

    always @ (posedge clk) begin
        hold_pre <= next_hold_pre;
    end

    always@(*)begin
        next_hold_pre = 4'd0;
        if(flag_pre)begin
            next_hold_pre = hold_pre + 4'd1;
        end
    end  

    always @ (posedge clk) begin //prestep
      if (state == `PRESTEP) begin
        start_prestep <= 1'b0;
        cnt_p <= cnt_p;
        if (prestep_done) begin
          cnt_p <= cnt_p + 7'd1;
          start_prestep <= 1'b1;
        end
        else if (flag_pre) begin
          start_prestep <= 1'b1;
        end
      end 
      else begin
        cnt_p <= 7'd0;
        start_prestep <= 1'b1;
      end
    end


    always@(posedge clk or posedge rst)begin // flag impulse ctrl
        flag_im <= flag_im;
        if(rst) begin
          flag_im <= 1'b0;
        end
        else if(impulse_done)begin
          flag_im <= 1'b1;
        end
        else if(hold_im == 4'd3)begin
          flag_im <= 1'b0;
        end
    end

    always @ (posedge clk) begin
        hold_im <= next_hold_im;
    end

    always@(*)begin
        next_hold_im = 4'd0;
        if(flag_im)begin
            next_hold_im = hold_im + 4'd1;
        end
    end  

    always @ (posedge clk) begin //applyImpulse
      if (state == `APPLYIMPULSE) begin
        start_impulse <= 1'b0;
        cnt_im <= cnt_im;
        if(cnt_im == contactNum) begin
            cnt_im <= 1'b0;
        end
        else if (impulse_done) begin
          start_impulse <= 1'b1;
          cnt_im <= cnt_im + 7'd1;
        end
        else if (flag_im) begin
          start_impulse <= 1'b1;
        end
      end 
      else begin
        cnt_im <= 7'd0;
        start_impulse <= 1'b1;
      end
    end
    
    always@(posedge clk)begin //iteration
      if (state == `APPLYIMPULSE) begin
          iter <= iter;
          if(cnt_im == contactNum) begin
              iter <= iter + 7'd1;
          end
      end 
      else begin
        iter <= 7'd0;
      end
    end

      //--------------------------------------------------------------------------------//
     //-------------------------------------Memory-------------------------------------//
    //--------------------------------------------------------------------------------//

    always @(posedge clk) begin //Vertices
        verticesX[4*nth_r + i_r] <= verticesX[4*nth_r + i_r];
        verticesY[4*nth_r + i_r] <= verticesY[4*nth_r + i_r];
        if(nth_r == 0)begin
            verticesX[0] <= {1'b0,10'd704,8'b0};
            verticesY[0] <= {1'b0,10'd400,8'b0};
            verticesX[1] <= -{1'b0,10'd64,8'b0};
            verticesY[1] <= {1'b0,10'd400,8'b0};
            verticesX[2] <= -{1'b0,10'd64,8'b0};
            verticesY[2] <= {1'b0,10'd480,8'b0};
            verticesX[3] <= {1'b0,10'd704,8'b0};
            verticesY[3] <= {1'b0,10'd480,8'b0};
        end
        else if (state == `RENDER) begin
            verticesX[4*nth_r + i_r] <= ver_x_render;
            verticesY[4*nth_r + i_r] <= ver_y_render;
        end
    end
    

    always @(*) begin // Detect write enable
        wea_conpt = 1'b0;
        web_conpt = 1'b0;
        wea_connom = 1'b0;
        web_connom = 1'b0;
        wea_conpen = 1'b0;
        web_conpen = 1'b0;
        wea_conmsn = 1'b0;
        wea_conmtan = 1'b0;
        wea_conbias = 1'b0;
        wea_conPn = 1'b0;
        wea_conPt = 1'b0;
        case(state)

        `BROADPHASE: begin
          if(collide_done)begin
            if(cNum == 2'd1)begin
                wea_conpt = 1'b1;
                wea_connom = 1'b1;
                wea_conpen = 1'b1;
            end
            else if(cNum == 2'd2)begin
                wea_conpt = 1'b1;
                web_conpt = 1'b1;
                wea_connom = 1'b1;
                web_connom = 1'b1;
                wea_conpen = 1'b1;
                web_conpen = 1'b1;
            end
          end
        end
        `PRESTEP: begin
          wea_conPn = 1'b1;
          wea_conPt = 1'b1;
          if(prestep_done)begin
            wea_conmsn = 1'b1;
            wea_conmtan = 1'b1;
            wea_conbias = 1'b1;
          end
        end

        `APPLYIMPULSE: begin
          if(impulse_done)begin
            wea_conPn = 1'b1;
            wea_conPt = 1'b1;
          end
        end

        default: begin
          wea_conpt = 1'b0;
          web_conpt = 1'b0;
          wea_connom = 1'b0;
          web_connom = 1'b0;
          wea_conpen = 1'b0;
          web_conpen = 1'b0;
          wea_conmsn = 1'b0;
          wea_conmtan = 1'b0;
          wea_conbias = 1'b0;
          wea_conPn = 1'b0;
          wea_conPt = 1'b0;
        end
        endcase
    end

    //------------------------------------------------------------------- contact Pn Pt -------------------------------------------------------------------//

    always @(*) begin // 2 single-mem, width = 35, depth = 50
        conPn_1_addr = 6'd0;
        conPt_1_addr = 6'd0;
        if(state == `PRESTEP)begin // write
            conPn_1_addr = cnt_p[5:0];
            conPt_1_addr = cnt_p[5:0];
        end
        else if(state == `APPLYIMPULSE)begin // read first , then write
            conPn_1_addr = cnt_im[5:0];
            conPt_1_addr = cnt_im[5:0];
        end
    end

    assign cPn_in = (state == `PRESTEP)? 35'd0 : cPn_aft;
    blk_mem_gen_con_cPn blk_mem_gen_con_cPn_inst(
    .clka(clk),
    .wea(wea_conPn),
    .addra(conPn_1_addr),
    .dina(cPn_in),
    .douta(contactPn)
    );

    assign cPt_in = (state == `PRESTEP)? 35'd0 : cPt_aft;
    blk_mem_gen_con_cPt blk_mem_gen_con_cPt_inst(
    .clka(clk),
    .wea(wea_conPt),
    .addra(conPt_1_addr),
    .dina(cPt_in),
    .douta(contactPt)
    );
    //--------------------------------------------------------------------------------------------------------------------------------------//

    //------------------------------------------------------------------- contact mass bias -------------------------------------------------------------------//

    always @(*) begin // 1 single-mem, width = 31, depth = 50
        conbias_1_addr = 6'd0;
        if(state == `PRESTEP)begin // write
            conbias_1_addr = cnt_p[5:0];
        end
        else if(state == `APPLYIMPULSE)begin // read
            conbias_1_addr = cnt_im[5:0];
        end
    end

    blk_mem_gen_con_bias blk_mem_gen_con_bias_inst(
    .clka(clk),
    .wea(wea_conbias),
    .addra(conbias_1_addr),
    .dina(cBias),
    .douta(contactBias)
    );
    //--------------------------------------------------------------------------------------------------------------------------------------//

    //------------------------------------------------------------------- contact mass tangent -------------------------------------------------------------------//

    always @(*) begin // 1 single-mem, width = 29, depth = 50
        conmtan_1_addr = 6'd0;
        if(state == `PRESTEP)begin // write
            conmtan_1_addr = cnt_p[5:0];
        end
        else if(state == `APPLYIMPULSE)begin //read
            conmtan_1_addr = cnt_im[5:0];
        end
    end

    blk_mem_gen_con_cmtan blk_mem_gen_con_cmtan_inst(
    .clka(clk),
    .wea(wea_conmtan),
    .addra(conmtan_1_addr),
    .dina(cMassTang),
    .douta(contactMassTang)
    );
   
    //--------------------------------------------------------------------------------------------------------------------------------------//

    //------------------------------------------------------------------- contact mass normal -------------------------------------------------------------------//

    always @(*) begin // 1 single-mem, width = 29, depth = 50
        conmsn_1_addr = 6'd0;
        if(state == `PRESTEP)begin //write
            conmsn_1_addr = cnt_p[5:0];
        end
        else if(state == `APPLYIMPULSE)begin //read
            conmsn_1_addr = cnt_im[5:0];
        end
    end

    blk_mem_gen_con_cmsn blk_mem_gen_con_cmsn_inst(
    .clka(clk),
    .wea(wea_conmsn),
    .addra(conmsn_1_addr),
    .dina(cMassNorm),
    .douta(contactMassNorm)
    );
   
    //--------------------------------------------------------------------------------------------------------------------------------------//

    //------------------------------------------------------------------- contact penetration -------------------------------------------------------------------//

    always @(*) begin // 1 duel-mem, width = 19, depth = 50
        conpen_1_addr = 6'd0;
        conpen_2_addr = 6'd0;
        if(state == `BROADPHASE)begin //write
            conpen_1_addr = contactNum[5:0];
            conpen_2_addr = contactNum[5:0] + 1;
        end
        else if(state == `PRESTEP)begin //read
            conpen_1_addr = cnt_p[5:0];
            conpen_2_addr = cnt_p[5:0];
        end
    end

    blk_mem_gen_con_pen blk_mem_gen_con_pen_inst(
    .clka(clk),
    .wea(wea_conpen),
    .addra(conpen_1_addr),
    .dina(contact1Penetration),
    .douta(contactPen_sub1),
    
    .clkb(clk),
    .web(web_conpen),
    .addrb(conpen_2_addr),
    .dinb(contact2Penetration),
    .doutb(contactPen_sub2)
    );
    

    assign conpen_temp = (contactPen_sub1 == 0)? contactPen_sub2 : contactPen_sub1;
   
    //--------------------------------------------------------------------------------------------------------------------------------------//
    
    //------------------------------------------------------------------- contact normal -------------------------------------------------------------------//

    always @(*) begin // 2 duel-mem, width = 10, depth = 50
        connom_x1_addr = 6'd0;
        connom_x2_addr = 6'd0;
        connom_y1_addr = 6'd0;
        connom_y2_addr = 6'd0;
        if(state == `BROADPHASE)begin //write
            connom_x1_addr = contactNum[5:0];
            connom_x2_addr = contactNum[5:0] + 1;
            connom_y1_addr = contactNum[5:0];
            connom_y2_addr = contactNum[5:0] + 1;
        end
        else if(state == `PRESTEP)begin //read
            connom_x1_addr = cnt_p[5:0];
            connom_x2_addr = cnt_p[5:0];
            connom_y1_addr = cnt_p[5:0];
            connom_y2_addr = cnt_p[5:0];
        end
        else if(state == `APPLYIMPULSE)begin //read
            connom_x1_addr = cnt_im[5:0];
            connom_x2_addr = cnt_im[5:0];
            connom_y1_addr = cnt_im[5:0];
            connom_y2_addr = cnt_im[5:0];
        end
    end

    blk_mem_gen_con_nom blk_mem_gen_con_nomx(
    .clka(clk),
    .wea(wea_connom),
    .addra(connom_x1_addr),
    .dina(contact1Norm_x),
    .douta(contactNorm_x_sub1),
    
    .clkb(clk),
    .web(web_connom),
    .addrb(connom_x2_addr),
    .dinb(contact2Norm_x),
    .doutb(contactNorm_x_sub2)
    );
    
    blk_mem_gen_con_nom blk_mem_gen_con_nomy(
    .clka(clk),
    .wea(wea_connom),
    .addra(connom_y1_addr),
    .dina(contact1Norm_y),
    .douta(contactNorm_y_sub1),
    
    .clkb(clk),
    .web(web_connom),
    .addrb(connom_y2_addr),
    .dinb(contact2Norm_y),
    .doutb(contactNorm_y_sub2)
    );

    assign connom_x_temp = (contactNorm_x_sub1 == 0)? contactNorm_x_sub2 : contactNorm_x_sub1;
    assign connom_y_temp = (contactNorm_y_sub1 == 0)? contactNorm_y_sub2 : contactNorm_y_sub1;
    //--------------------------------------------------------------------------------------------------------------------------------------//
    
    //------------------------------------------------------------------- contact point -------------------------------------------------------------------//

    always @(*) begin // 2 duel-mem, width = 19, depth = 50
        conpt_x1_addr = 6'd0;
        conpt_x2_addr = 6'd0;
        conpt_y1_addr = 6'd0;
        conpt_y2_addr = 6'd0;
        if(state == `BROADPHASE)begin //write
            conpt_x1_addr = contactNum[5:0];
            conpt_x2_addr = contactNum[5:0] + 1;
            conpt_y1_addr = contactNum[5:0];
            conpt_y2_addr = contactNum[5:0] + 1;
        end
        else if(state == `PRESTEP)begin //read
            conpt_x1_addr = cnt_p[5:0];
            conpt_x2_addr = cnt_p[5:0];
            conpt_y1_addr = cnt_p[5:0];
            conpt_y2_addr = cnt_p[5:0];
        end
        else if(state == `APPLYIMPULSE)begin //read
            conpt_x1_addr = cnt_im[5:0];
            conpt_x2_addr = cnt_im[5:0];
            conpt_y1_addr = cnt_im[5:0];
            conpt_y2_addr = cnt_im[5:0];
        end
    end
    
    blk_mem_gen_con_pt blk_mem_gen_con_ptx(
    .clka(clk),
    .wea(wea_conpt),
    .addra(conpt_x1_addr),
    .dina(contact1Pt_x),
    .douta(contactPt_x_sub1),
    
    .clkb(clk),
    .web(web_conpt),
    .addrb(conpt_x2_addr),
    .dinb(contact2Pt_x),
    .doutb(contactPt_x_sub2)
    );
    
    blk_mem_gen_con_pt blk_mem_gen_con_pty(
    .clka(clk),
    .wea(wea_conpt),
    .addra(conpt_x1_addr),
    .dina(contact1Pt_y),
    .douta(contactPt_y_sub1),
    
    .clkb(clk),
    .web(web_conpt),
    .addrb(conpt_x2_addr),
    .dinb(contact2Pt_y),
    .doutb(contactPt_y_sub2)
    );

    assign conpt_x_temp = (contactPt_x_sub1 == 0)? contactPt_x_sub2 : contactPt_x_sub1;
    assign conpt_y_temp = (contactPt_y_sub1 == 0)? contactPt_y_sub2 : contactPt_y_sub1;
    //--------------------------------------------------------------------------------------------------------------------------------------//


    always @(posedge clk )begin
        contactNum <= contactNum;    
        contact_b1[contactNum] <= contact_b1[contactNum];
        contact_b2[contactNum] <= contact_b2[contactNum];
        if(state == `DESTROY)begin
            contactNum <= 0;
        end
        else if(state == `BROADPHASE)begin
            if(collide_done)begin
                if(cNum == 2'd1)begin
                    contactNum <= contactNum + 7'd1;
                    contact_b1[contactNum] <= nth_b1;
                    contact_b2[contactNum] <= nth_b2;
                end
                else if(cNum == 2'd2) begin
                    contactNum <= contactNum + 7'd2;
                    contact_b1[contactNum] <= nth_b1;
                    contact_b2[contactNum] <= nth_b2;
                    contact_b1[contactNum + 1] <= nth_b1;
                    contact_b2[contactNum + 1] <= nth_b2;
                end
            end
        end 
    end

      //-------------------------------------------------------------------------------------//
     //--------------------------------------Variables Ctrl---------------------------------//
    //-------------------------------------------------------------------------------------//

    wire signed [21:0] angv [0:3];
    assign angv[0] = (angVel[3] >>> fps);
    assign angv[1] = (angVel[4] >>> fps);
    assign angv[2] = (angVel[5] >>> fps);
    assign angv[3] = (angVel[6] >>> fps);

    always @ (posedge clk or posedge rst_op) begin
      if (rst_op) begin
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
    
        Inv_mass[0] <=  22'b0_0_00000000000000000000; //ground
        Inv_mass[1] <=  22'b0_0_00000001101000110110; //bird
        Inv_mass[2] <=  22'b0_0_00000000000000000000; //pig
        Inv_mass[3] <=  22'b0_0_00000001010001111010; //3
        Inv_mass[4] <=  22'b0_0_00000001010001111010; //4
        Inv_mass[5] <=  22'b0_0_00000001010001111010; //5
        Inv_mass[6] <=  22'b0_0_00000001010001111010; //6
        Inv_mass[7] <=  22'b0_0_00000001010001111010; //7
        Inv_mass[8] <=  22'b0_0_00000001010001111010; //8
        Inv_mass[9] <=  22'b0_0_00000001010001111010; //9
        
        Inv_I[0] <=  22'b0_0_00000000000000000000; //ground
        Inv_I[1] <=  22'b0_0_00000000000010000000; //bird  
        Inv_I[2] <=  22'b0_0_00000000000000000000; //pig   
        Inv_I[3] <=  22'b0_0_00000000000000010011; //3     
        Inv_I[4] <=  22'b0_0_00000000000000010011; //4     
        Inv_I[5] <=  22'b0_0_00000000000000010011; //5     
        Inv_I[6] <=  22'b0_0_00000000000000010011; //6     
        Inv_I[7] <=  22'b0_0_00000000000000010011; //7     
        Inv_I[8] <=  22'b0_0_00000000000000010011; //8     
        Inv_I[9] <=  22'b0_0_00000000000000010011; //9     
    
        posX[0] <= {1'b0,10'd320,20'b0}; posY[0] <= {1'b0,10'd440,20'b0};//ground
        posX[1] <= {1'b0,10'd130,20'b0}; posY[1] <= {1'b0,10'd310,20'b0};//bird
        posX[2] <= {1'b0,10'd450,20'b0}; posY[2] <= {1'b0,10'd380,20'b0};//pig
        posX[3] <= {1'b0,10'd380,20'b0}; posY[3] <= {1'b0,10'd368,20'b0};
        posX[4] <= {1'b0,10'd420,20'b0}; posY[4] <= {1'b0,10'd368,20'b0};
        posX[5] <= {1'b0,10'd480,20'b0}; posY[5] <= {1'b0,10'd368,20'b0};
        posX[6] <= {1'b0,10'd450,20'b0}; posY[6] <= {1'b0,10'd331,20'b0};//horizontal
//        posX[7] <= {1'b0,10'd435,20'b0}; posY[7] <= {1'b0,10'd296,20'b0};
//        posX[8] <= {1'b0,10'd465,20'b0}; posY[8] <= {1'b0,10'd296,20'b0};
//        posX[9] <= {1'b0,10'd450,20'b0}; posY[9] <= {1'b0,10'd260,20'b0};//horizontal
//        posX[10]  <= {1'b0,10'd0,20'b0}; posY[10] <=  {1'b0,10'd0,20'b0};

        velX[0]  <= 31'd0; velY[0]  <= 31'd0;//ground
        velX[1]  <= 31'd0; velY[1]  <= 31'd0;//bird
        velX[2]  <= 31'd0; velY[2]  <= 31'd0;//pig
        velX[3]  <= 31'd0; velY[3]  <= 31'd0;
        velX[4]  <= 31'd0; velY[4]  <= 31'd0;
        velX[5]  <= 31'd0; velY[5]  <= 31'd0;
        velX[6]  <= 31'd0; velY[6]  <= 31'd0;//horizontal
        velX[7]  <= 31'd0; velY[7]  <= 31'd0;
        velX[8]  <= 31'd0; velY[8]  <= 31'd0;
        velX[9]  <= 31'd0; velY[9]  <= 31'd0;//horizontal
        velX[10] <= 31'd0; velY[10] <= 31'd0;

        angVel[0] <= {1'b0,1'b0,20'b00000000000000000000};
        angVel[1] <= {1'b0,1'b0,20'b00000000000000000000};
        angVel[2] <= {1'b0,1'b0,20'b00000000000000000000};
        angVel[3] <= {1'b0,1'b0,20'b00000000000000000000};
        angVel[4] <= {1'b0,1'b0,20'b00000000000000000000};
        angVel[5] <= {1'b0,1'b0,20'b00000000000000000000};
        angVel[6] <= {1'b0,1'b0,20'b00000000000000000000};
        angVel[7] <= {1'b0,1'b0,20'b00000000000000000000};
        angVel[8] <= {1'b0,1'b0,20'b00000000000000000000};
        angVel[9] <= {1'b0,1'b0,20'b00000000000000000000};
        angVel[10] <= {1'b0,1'b0,20'b00000000000000000000};

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

      end else begin
//        typ[nth] <= typ[nth];
//        alive[nth] <= alive[nth];
//        posX[nth] <= posX[nth];
//        posY[nth] <= posY[nth];
//        rad[nth] <= rad[nth];

        if (state == `RENDER) begin
            if(flag && !state_change)begin
                posX[1] <= {1'b0,MOUSE_X_POS,20'b0};
                posY[1] <= {1'b0,MOUSE_Y_POS,20'b0};
            end
        end
        else if (state == `PlayerCTRL) begin
            velX[1] <= (state_change)? velX[1] : (origin_x - posX[1] );
            velY[1] <= (state_change)? velY[1] : (origin_y - posY[1] );
        end
        else if (state == `ADDFORCE) begin
            // velY[0] <= velY[0] + gravity;//ground
            velY[1] <= velY[1] + gravity;
//            velY[2] <= velY[2] + gravity;
            velY[3] <= velY[3] + gravity;
            velY[4] <= velY[4] + gravity;
            velY[5] <= velY[5] + gravity;
            velY[6] <= velY[6] + gravity;
//            velY[7] <= velY[7] + gravity;
//            velY[8] <= velY[8] + gravity;
//            velY[9] <= velY[9] + gravity;
        end
        else if (state == `APPLYIMPULSE)begin
            if(impulse_done) begin
              angVel[contact_b1[cnt_im]] <= b1AngVel_aft;
              angVel[contact_b2[cnt_im]] <= b2AngVel_aft;
              velX[contact_b1[cnt_im]] <= {b1Vel_x_aft[29], b1Vel_x_aft}; velY[contact_b1[cnt_im]] <= {b1Vel_y_aft[29], b1Vel_y_aft};
              velX[contact_b2[cnt_im]] <= {b2Vel_x_aft[29], b2Vel_x_aft}; velY[contact_b2[cnt_im]] <= {b2Vel_y_aft[29], b2Vel_y_aft};
            end
        end
        else if (state == `ADDVEL) begin
            posX[1] <= posX[1] + (velX[1] >>> fps); posY[1] <= posY[1] + (velY[1] >>> fps );
//            posX[2] <= posX[2] + (velX[2] >>> fps); posY[2] <= posY[2] + (velY[2] >>> fps );
            posX[3] <= posX[3] + (velX[3] >>> fps); posY[3] <= posY[3] + (velY[3] >>> fps );
            posX[4] <= posX[4] + (velX[4] >>> fps); posY[4] <= posY[4] + (velY[4] >>> fps );
            posX[5] <= posX[5] + (velX[5] >>> fps); posY[5] <= posY[5] + (velY[5] >>> fps );
            posX[6] <= posX[6] + (velX[6] >>> fps); posY[6] <= posY[6] + (velY[6] >>> fps );
        end                                                                           
        else if (state == `ADDVEL2) begin                                             
//            posX[7] <= posX[7] + (velX[7] >>> fps); posY[7] <= posY[7] + (velY[7] >>> fps );
//            posX[8] <= posX[8] + (velX[8] >>> fps); posY[8] <= posY[8] + (velY[8] >>> fps );
//            posX[9] <= posX[9] + (velX[9] >>> fps); posY[9] <= posY[9] + (velY[9] >>> fps );
            if((rad[3] + angv[0][21:12]) >= 10'b0_11_0010_010)begin                              
                rad[3] <= (rad[3] + angv[0][21:12]) + (10'b1_00_1101_110 + 10'b1_00_1101_110); 
            end else begin                                                                
                rad[3] <= (rad[3] + angv[0][21:12]);                                           
            end
            
            if((rad[4] + angv[1][21:12]) >= 10'b0_11_0010_010)begin                              
                rad[4] <= (rad[4] + angv[1][21:12]) + (10'b1_00_1101_110 + 10'b1_00_1101_110); 
            end else begin                                                                
                rad[4] <= (rad[4] + angv[1][21:12]);                                                                                                                      
            end  
            
            if((rad[5] + angv[2][21:12]) >= 10'b0_11_0010_010)begin                              
                rad[5] <= (rad[5] + angv[2][21:12]) + (10'b1_00_1101_110 + 10'b1_00_1101_110); 
            end else begin                                                                
                rad[5] <= (rad[5] + angv[2][21:12]);                                           
            end                                                                           
            
            if((rad[6] + angv[3][21:12]) >= 10'b0_11_0010_010)begin                              
                rad[6] <= (rad[6] + angv[3][21:12]) + (10'b1_00_1101_110 + 10'b1_00_1101_110); 
            end else begin                                                                
                rad[6] <= (rad[6] + angv[3][21:12]);                                           
            end                                                                           
                                                                                          
//            if(rad[7] + angVel[7] >= 10'b0_11_0010_010)begin                              
//                rad[7] <= (rad[7] + (angVel[7]>>>fps)) + (10'b1_00_1101_110 + 10'b1_00_1101_110); 
//            end else begin                                                                
//                rad[7] <= (rad[7] + (angVel[7]>>>fps));                                           
//            end                                                                           
                                                                                          
//            if(rad[8] + angVel[8] >= 10'b0_11_0010_010)begin                              
//                rad[8] <= (rad[8] + (angVel[8]>>>fps)) + (10'b1_00_1101_110 + 10'b1_00_1101_110); 
//            end else begin                                                                
//                rad[8] <= (rad[8] + (angVel[8]>>>fps));                                           
//            end                                                                           
                                                                                          
//            if(rad[9] + angVel[9] >= 10'b0_11_0010_010)begin                              
//                rad[9] <= (rad[9] + (angVel[9]>>>fps)) + (10'b1_00_1101_110 + 10'b1_00_1101_110); 
//            end else begin                                                                
//                rad[9] <= (rad[9] + (angVel[9]>>>fps));                                           
//            end
                                                                                       
        end 
      end
    end

      //-----------------------------------------------------------------------------------------//
     //-------------------------------------Module Connection-----------------------------------//
    //-----------------------------------------------------------------------------------------//

    clock_divisor clk_dv(.clk2(clk_dv3), .clk1(clk_25Mhz), .clk(clk));
    
    MouseCtl #(
      .SYSCLK_FREQUENCY_HZ(108000000),
      .CHECK_PERIOD_MS(500),
      .TIMEOUT_PERIOD_MS(100)
    )MC1(
        .clk(clk),
        .rst(1'b0),
        .xpos(MOUSE_X_POS),
        .ypos(MOUSE_Y_POS),
        .zpos(MOUSE_Z_POS),
        .left(MOUSE_LEFT),
        .middle(MOUSE_MIDDLE),
        .right(MOUSE_RIGHT),
        .new_event(MOUSE_NEW_EVENT),
        .value(12'd0),
        .setx(1'b0),
        .sety(1'b0),
        .setmax_x(1'b0),
        .setmax_y(1'b0),
        .ps2_clk(PS2_CLK),
        .ps2_data(PS2_DATA)
    );

    ApplyImpulse aply_impls(
	  .clk(clk_25Mhz),  .rst(start_impulse),
  	  .cPos_x( {conpt_x_temp, 12'b0} ), .cPos_y( {conpt_y_temp, 12'b0} ), //20 bit
  	  .cNorm_x( {connom_x_temp, 12'b0} ), .cNorm_y( {connom_y_temp, 12'b0} ), //20 bit
      .b1Pos_x( posX[ contact_b1[cnt_im] ] ), .b1Pos_y( posY[ contact_b1[cnt_im] ] ), //20 bit
      .b2Pos_x( posX[ contact_b2[cnt_im] ] ), .b2Pos_y( posY[ contact_b2[cnt_im] ] ), //20 bit
      .b1Inv_mass( Inv_mass[ contact_b1[cnt_im] ] ), .b2Inv_mass( Inv_mass[ contact_b2[cnt_im] ] ), //20 bit
      .b1Inv_I(Inv_I[ contact_b1[cnt_im] ]), .b2Inv_I(Inv_I[ contact_b2[cnt_im] ]), //20 bit
      .b1Vel_x( velX[contact_b1[cnt_im]][29:0] ), .b1Vel_y( velY[contact_b1[cnt_im]][29:0] ), //20 bit
      .b2Vel_x( velX[contact_b2[cnt_im]][29:0] ), .b2Vel_y( velY[contact_b2[cnt_im]][29:0] ), //20 bit
      .b1AngVel( angVel[ contact_b1[cnt_im] ] ), .b2AngVel( angVel[ contact_b2[cnt_im] ] ), //20 bit
      .cPn( contactPn ) /*20 bit*/, .cPt( contactPt ), //20 bit
      .cMassNorm(contactMassNorm), //20 bit
      .cMassTang(contactMassTang), //20 bit
      .cBias(contactBias), //20 bit
      .cPn_aft(cPn_aft) /*20 bit*/, .cPt_aft(cPt_aft), //20 bit
      .b1Vel_x_aft(b1Vel_x_aft)/*20 bit*/, .b1Vel_y_aft(b1Vel_y_aft), //20 bit
      .b2Vel_x_aft(b2Vel_x_aft)/*20 bit*/, .b2Vel_y_aft(b2Vel_y_aft), //20 bit 
      .b1AngVel_aft(b1AngVel_aft)/*20 bit*/, .b2AngVel_aft(b2AngVel_aft), //20 bit
      .done_out(impulse_done) 
	);

    PreStep pr_stp(
      .clk(clk_25Mhz),  .rst(start_prestep),
      .cPenetration( {conpen_temp, 12'b0} ),
      .cPos_x( {conpt_x_temp, 12'b0} ), .cPos_y( {conpt_y_temp, 12'b0} ),
      .cNorm_x( {connom_x_temp, 12'b0} ), .cNorm_y( {connom_y_temp, 12'b0} ),
      .b1Pos_x( posX[ contact_b1[cnt_p] ] ), .b1Pos_y( posY[ contact_b1[cnt_p] ] ),
      .b2Pos_x( posX[ contact_b2[cnt_p] ] ), .b2Pos_y( posY[ contact_b2[cnt_p] ] ),
      .b1Inv_mass(Inv_mass[ contact_b1[cnt_p] ]), .b2Inv_mass(Inv_mass[ contact_b2[cnt_p] ]),
      .b1Inv_I(Inv_I[ contact_b1[cnt_p] ]), .b2Inv_I(Inv_I[ contact_b2[cnt_p] ]),
      .cMassNorm(cMassNorm),
      .cMassTang(cMassTang),
      .cBias(cBias), 
      .done_out(prestep_done) 
    );

    Collide cc(
      .clk(clk),
      .clk_collide(clk_dv3),
      .clk_Pen(clk_dv3),
      .rst_op(start_collide),
      .A_nth(nth_b1),
      .B_nth(nth_b2),
      .A0_x(verticesX[4*nth_b1]), .A0_y(verticesY[4*nth_b1]),
      .A1_x(verticesX[4*nth_b1 + 1]), .A1_y(verticesY[4*nth_b1 + 1]),
      .A2_x(verticesX[4*nth_b1 + 2]), .A2_y(verticesY[4*nth_b1 + 2]),
      .A3_x(verticesX[4*nth_b1 + 3]), .A3_y(verticesY[4*nth_b1 + 3]),
      .B0_x(verticesX[4*nth_b2]), .B0_y(verticesY[4*nth_b2]),
      .B1_x(verticesX[4*nth_b2 + 1]), .B1_y(verticesY[4*nth_b2 + 1]),
      .B2_x(verticesX[4*nth_b2 + 2]), .B2_y(verticesY[4*nth_b2 + 2]),
      .B3_x(verticesX[4*nth_b2 + 3]), .B3_y(verticesY[4*nth_b2 + 3]),
      .Apos_x( posX[nth_b1][30:12] ), .Apos_y( posY[nth_b1][30:12] ),
      .Bpos_x( posX[nth_b2][30:12] ), .Bpos_y( posY[nth_b2][30:12] ),
      .contact1Pt_x(contact1Pt_x), .contact1Pt_y(contact1Pt_y),
      .contact2Pt_x(contact2Pt_x), .contact2Pt_y(contact2Pt_y),
      .contact1Norm_x(contact1Norm_x), .contact1Norm_y(contact1Norm_y),
      .contact2Norm_x(contact2Norm_x), .contact2Norm_y(contact2Norm_y),
      .contact1Penetration(contact1Penetration), .contact2Penetration(contact2Penetration),
      .contactNum(cNum),
      .contact(contact),
      .done_out(collide_done)
    );

    Render rr( //number inside #(___) which means how many objects need to caculate
      .render_ready(master_ready),
      .pos_x(posX[nth_r][30:12]),
      .pos_y(posY[nth_r][30:12]),
      .rad(rad[nth_r]),
      .nth(nth_r),
      .i(i_r),
      .data_ready(data_ready),
      .vertice_x(ver_x_render),
      .vertice_y(ver_y_render)
    );

      //-------------------------------------------------------------------------//
     //----------------------------Output Assignment----------------------------//
    //-------------------------------------------------------------------------//
    reg end_flag;
    assign vertice_x = (nth_r == 4'd7)? {1'b0,MOUSE_X_POS,1'b0} : ver_x_render[18:7];
    assign vertice_y = (nth_r == 4'd7)? {1'b0,MOUSE_Y_POS,1'b0} : ver_y_render[18:7];
    assign onehot = (!state_change) ? {10'b0, 1'b1, 1'b0}: (end_flag) ? {10'b0, 1'b0, 1'b1} : {10'b0, 1'b0, 1'b0};
    assign nth_body = nth_r;
    assign i_vertex = i_r;
    

      //-----------------------------------------------------------------------//
     //-------------------------------Signal Ctrl-----------------------------//
    //-----------------------------------------------------------------------//

    always@(posedge clk or posedge rst_op)begin
        if(rst_op)begin
            flag <= 1'b0;
        end
        else begin
            flag <= flag;
            if(MOUSE_LEFT & constraint_x_down < MOUSE_X_POS & MOUSE_X_POS < constraint_x_up & constraint_y_down < MOUSE_Y_POS & MOUSE_Y_POS < constraint_y_up)begin
                flag <= 1'b1;
            end
        end
    end

    always@(posedge clk or posedge rst_op)begin
        if(rst_op)begin
            state_change <= 1'b0;
        end
        else begin
            state_change <= state_change;
            if(MOUSE_RIGHT && flag) begin
                state_change <= 1'b1;
            end
        end
    end

      //------------------------------------------------------------------------//
     //----------------------------State transition----------------------------//
    //------------------------------------------------------------------------//
    
    always@(posedge clk or posedge rst_op)
        if(rst_op) begin
            state <= `INIT;
            end_flag <= 0;
        end
        else begin
            state <= next_state;
            end_flag <= end_flag;
            if (MOUSE_MIDDLE) begin
                end_flag <= 1'b1;
            end
        end

    always@(*) begin
        clear = 1'b0;
        case (state)
          `INIT: begin
                    next_state = `RENDER;
                end
          `RENDER: begin
                    next_state = state;
                    if(cnt_r == 6'd30) begin
                        next_state = `BROADPHASE;
                    end
                  end
          `PlayerCTRL: begin
                          next_state = `DESTROY;
                          if(state_change)begin
                              next_state = `BROADPHASE;
                          end
                      end
          `BROADPHASE: begin
                        if (nth_b1 == (bodyNum - 4'd1) ) begin
                          next_state = `ADDFORCE;
                        end
                        else begin
                          next_state = state;
                        end
                      end
          `ADDFORCE: begin
                        next_state = `PRESTEP;
//                        next_state = `ADDVEL;
                     end
          `PRESTEP : begin
                        next_state = state;
                        if(cnt_p == contactNum)begin
                          next_state = `APPLYIMPULSE;
                        end
                    end
          `APPLYIMPULSE : begin
                            next_state = state;
                            if(iter == iteration)begin
                              next_state = `ADDVEL;
                            end
                          end
          `ADDVEL: begin
                        next_state = `ADDVEL2;                     
                     end
          `ADDVEL2: begin
                         next_state = `DESTROY;                     
                      end
          `DESTROY :   begin
                         next_state = `RENDER;
                         clear = 1'b1;                 
                      end
          default: next_state = `INIT;
        endcase
    end

endmodule

//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------


module clock_divisor(clk2, clk1, clk);
  input clk;
  output clk1;
  output clk2;
  // output clk22;
  reg [21:0] num;
  wire [21:0] next_num;

  always @(posedge clk) begin
    num <= next_num;
  end

  assign next_num = num + 1;
  assign clk1 = num[1];
//  assign clk1 = clk;
  assign clk2 = num[3];
//  assign clk2 = clk;
//   assign clk22 = num[21];
endmodule
