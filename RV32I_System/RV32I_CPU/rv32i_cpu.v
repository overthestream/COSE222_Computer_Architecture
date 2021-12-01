//
//  Author: Prof. Taeweon Suh
//          Computer Science & Engineering
//          Korea University
//  Date: July 14, 2020
//  Description: Skeleton design of RV32I Single-cycle CPU
//

`timescale 1ns/1ns
`define simdelay 1

module rv32i_cpu (
		      input         clk, reset,
            output [31:0] pc,		  		// program counter for instruction fetch
            input  [31:0] inst, 			// incoming instruction
            output        Memwrite, 	// 'memory write' control signal
            output [31:0] Memaddr,  	// memory address 
            output [31:0] MemWdata, 	// data to write to memory
            input  [31:0] MemRdata); 	// data read from memory

  wire        auipc, lui;
  wire        alusrc, regwrite;
  wire [4:0]  alucontrol;
  wire        memtoreg, memwrite;
  wire        branch, jal, jalr;


  // ################################################## 노정훈 start

    //  assign Memwrite = memwrite ;
    wire [31:0] IF_ID_inst; //wire for instruction fetch 
    wire        IF_ID_memread;    // memread (for stall)
    assign      IF_ID_memread = (IF_ID_inst[6:0] == 7'b0000011); // wire for control stall 

  // Instantiate Controller
  controller i_controller(
    .opcode		(IF_ID_inst[6:0]), 
		.funct7		(IF_ID_inst[31:25]), 
		.funct3		(IF_ID_inst[14:12]), 
  // ################################################## 노정훈  end
		.auipc		(auipc),
		.lui			(lui),
		.memtoreg	(memtoreg),
		.memwrite	(memwrite),
		.branch		(branch),
		.alusrc		(alusrc),
		.regwrite	(regwrite),
		.jal			(jal),
		.jalr			(jalr),
		.alucontrol	(alucontrol));

  // Instantiate Datapath
  datapath i_datapath(
		.clk				(clk),
		.reset			(reset),
		.auipc			(auipc),
		.lui				(lui),
		.memtoreg		(memtoreg),
		.memwrite		(memwrite),
		.branch			(branch),
		.alusrc			(alusrc),
		.regwrite		(regwrite),
		.jal				(jal),
		.jalr				(jalr),
		.alucontrol	(alucontrol),
		.pc				  (pc),
		.inst				(inst),
		.MemWdata		(MemWdata),
		.MemRdata		(MemRdata),
  // wire for instruction fetch : ##################### 노정훈 start
  	.EX_MEM_aluout	(Memaddr), 
    .IF_ID_inst     (IF_ID_inst),
    .EX_MEM_memwrite(Memwrite),
    .IF_ID_memread  (IF_ID_memread)
  // ################################################## 노정훈 End
    );

endmodule


//
// Instruction Decoder 
// to generate control signals for datapath
//
module controller(input  [6:0] opcode,
                  input  [6:0] funct7,
                  input  [2:0] funct3,
                  output       auipc,
                  output       lui,
                  output       alusrc,
                  output [4:0] alucontrol,
                  output       branch,
                  output       jal,
                  output       jalr,
                  output       memtoreg,
                  output       memwrite,
                  output       regwrite);

	maindec i_maindec(
		.opcode		(opcode),
		.auipc		(auipc),
		.lui			(lui),
		.memtoreg	(memtoreg),
		.memwrite	(memwrite),
		.branch		(branch),
		.alusrc		(alusrc),
		.regwrite	(regwrite),
		.jal			(jal),
		.jalr			(jalr));

	aludec i_aludec( 
		.opcode     (opcode),
		.funct7     (funct7),
		.funct3     (funct3),
		.alucontrol (alucontrol));


endmodule


//
// RV32I Opcode map = Inst[6:0]
//
`define OP_R			7'b0110011
`define OP_I_ARITH	7'b0010011
`define OP_I_LOAD  	7'b0000011
`define OP_I_JALR  	7'b1100111
`define OP_S			7'b0100011
`define OP_B			7'b1100011
`define OP_U_LUI		7'b0110111
`define OP_J_JAL		7'b1101111

//
// Main decoder generates all control signals except alucontrol 
//
//
module maindec(input  [6:0] opcode,
               output       auipc,
               output       lui,
               output       regwrite,
               output       alusrc,
               output       memtoreg, memwrite,
               output       branch, 
               output       jal,
               output       jalr);

  reg [8:0] controls;

  assign {auipc, lui, regwrite, alusrc, 
			 memtoreg, memwrite, branch, jal, 
			 jalr} = controls;

  always @(*)
  begin
    case(opcode)
      `OP_R: 			controls <= #`simdelay 9'b0010_0000_0; // R-type
      `OP_I_ARITH: 	controls <= #`simdelay 9'b0011_0000_0; // I-type Arithmetic
      `OP_I_LOAD: 	controls <= #`simdelay 9'b0011_1000_0; // I-type Load
      `OP_S: 			controls <= #`simdelay 9'b0001_0100_0; // S-type Store
      `OP_B: 			controls <= #`simdelay 9'b0000_0010_0; // B-type Branch
      `OP_U_LUI: 		controls <= #`simdelay 9'b0111_0000_0; // LUI
      `OP_J_JAL: 		controls <= #`simdelay 9'b0011_0001_0; // JAL
      
      `OP_I_JALR: controls  <= #`simdelay 9'b0011_0000_1;

      default:    	controls <= #`simdelay 9'b0000_0000_0; // ???
    endcase
  end

endmodule

//
// ALU decoder generates ALU control signal (alucontrol)
//
module aludec(input      [6:0] opcode,
              input      [6:0] funct7,
              input      [2:0] funct3,
              output reg [4:0] alucontrol);

  always @(*)

    case(opcode)

      `OP_R:   		// R-type
		begin
			case({funct7,funct3})
			 10'b0000000_000: alucontrol <= #`simdelay 5'b00000; // addition (add)
			 10'b0100000_000: alucontrol <= #`simdelay 5'b10000; // subtraction (sub)
			 10'b0000000_111: alucontrol <= #`simdelay 5'b00001; // and (and)
			 10'b0000000_110: alucontrol <= #`simdelay 5'b00010; // or (or)
          default:         alucontrol <= #`simdelay 5'bxxxxx; // ???
        endcase
		end

      `OP_I_ARITH:   // I-type Arithmetic
		begin
			case(funct3)
			 3'b000:  alucontrol <= #`simdelay 5'b00000; // addition (addi)
       
       // ############################################################ 노정훈 : Start 
       3'b001:  alucontrol <= #`simdelay 5'b00100; // ssli (slli)
       // ############################################################ 노정훈 : End   
       3'b100:  alucontrol <= #`simdelay 5'b00011; // xor (xori)
			 3'b110:  alucontrol <= #`simdelay 5'b00010; // or (ori)
			 3'b111:  alucontrol <= #`simdelay 5'b00001; // and (andi)
          default: alucontrol <= #`simdelay 5'bxxxxx; // ???
        endcase
		end

      `OP_I_LOAD: 	// I-type Load (LW, LH, LB...)
      	alucontrol <= #`simdelay 5'b00000;  // addition 

      `OP_B:   		// B-type Branch (BEQ, BNE, ...)
      	alucontrol <= #`simdelay 5'b10000;  // subtraction 

      `OP_S:   		// S-type Store (SW, SH, SB)
      	alucontrol <= #`simdelay 5'b00000;  // addition 

      `OP_U_LUI: 		// U-type (LUI)
      	alucontrol <= #`simdelay 5'b00000;  // addition

      `OP_I_JALR:   // I-type (JALR)
        alucontrol <= #`simdelay 5'b00000;  // addition 

      default: 
      	alucontrol <= #`simdelay 5'b00000;  // 

    endcase
    
endmodule


//
// CPU datapath
//
module datapath(input         clk, reset,
                input  [31:0] inst,
                input         auipc,
                input         lui,
                input         regwrite,
                input         memtoreg,
                input         memwrite,
                input         alusrc, 
                input  [4:0]  alucontrol,
                input         branch,
                input         jal,
                input         jalr,
                output reg [31:0] pc,
                output     [31:0] MemWdata,
                input      [31:0] MemRdata,
                // ############################################## 노정훈 start
                input             IF_ID_memread,
                output reg [31:0] EX_MEM_aluout,
                output reg [31:0] IF_ID_inst,
                output reg        EX_MEM_memwrite
                // ############################################## 노정훈 end
                );

  wire [4:0]  rs1, rs2, rd;
  wire [2:0]  funct3;
// ############################################## 노정훈 start
  reg [31:0] rs1_data, rs2_data;
// ############################################## 노정훈 end
  reg  [31:0] rd_data;
  wire [20:1] jal_imm;
  wire [31:0] se_jal_imm;
  wire [12:1] br_imm;
  wire [31:0] se_br_imm;
  wire [31:0] se_imm_itype;
  wire [31:0] se_imm_stype;
  wire [31:0] auipc_lui_imm;
  reg  [31:0] alusrc1;
  reg  [31:0] alusrc2;
  wire [31:0] branch_dest, jal_dest, jalr_dest;
  wire	  	  Nflag, Zflag, Cflag, Vflag;
  wire  		  f3beq, f3blt, f3bgeu;
  // ##################################### bne
  wire        f3bne;
  wire        bne_taken;
  // ###################################### 

  // ############################################ 노정훈 last milestone 
  wire        f3bltu, f3bge;
  wire        bltu_taken, bge_taken;
  // ############################################
  wire  		  beq_taken;
  wire  		  blt_taken;
  wire        bgeu_taken;

  //######################################################### 노정훈 start
    
    // wire for Hazard Detection Unit &  Forwarding Unit 
    wire        stall;
    wire        fw_rs1_exe, fw_rs2_exe, fw_rs1_Rdata, fw_rs2_Rdata, fw_rs1_mem, fw_rs2_mem, fw_rs1_wb, fw_rs2_wb;
    wire [31:0] aluout;
    // wire for data from register file
    wire [31:0] rf_rs1_data, rf_rs2_data;

    // wire for flush IF 
    reg         IF_flush;
    reg         flush_flag;

    always@(posedge clk)
    begin
      if     (stall)
        IF_flush <= IF_flush;
      else if(flush_flag)
        IF_flush <= 1'b0;
      else if(inst[6:0] == 7'b1101111 | inst[6:0] == 7'b1100111 | inst[6:0] == 7'b1100011)
        IF_flush <= 1'b1;
      else 
        IF_flush <= 1'b0;
    end

    // IF_ID FF
    reg [31:0]  IF_ID_pc;
    always@(posedge clk)
    begin
      if(stall)
        begin
          IF_ID_inst <= IF_ID_inst;
          IF_ID_pc   <= IF_ID_pc;
        end
      else if(IF_flush || flush_flag)
        begin
          IF_ID_inst <= 0;
          IF_ID_pc   <= 0;
        end
      else
      begin 
          IF_ID_inst  <= inst;
          IF_ID_pc    <= pc;
      end
    end

    // ID_EX FF
    assign rs1 = IF_ID_inst[19:15];
    assign rs2 = IF_ID_inst[24:20];
    assign rd  = IF_ID_inst[11:7];

    reg        ID_EX_auipc, ID_EX_lui, ID_EX_regwrite, ID_EX_memtoreg, ID_EX_memwrite, ID_EX_memread, ID_EX_alusrc, ID_EX_branch, ID_EX_jal, ID_EX_jalr;
    reg [4:0]  ID_EX_alucont, ID_EX_rs1, ID_EX_rs2, ID_EX_rd;
    reg [31:0] ID_EX_rs1_data, ID_EX_rs2_data, ID_EX_pc, ID_EX_imm_u, ID_EX_imm_i, ID_EX_imm_s, ID_EX_inst, ID_EX_imm_b, ID_EX_imm_j;
    always@(posedge clk)
    begin
        if(stall)
        begin
          ID_EX_auipc       <= 1'b0;
          ID_EX_lui         <= 1'b0;
          ID_EX_regwrite    <= 1'b0;
          ID_EX_memtoreg    <= 1'b0;
          ID_EX_memwrite    <= 1'b0;
          ID_EX_memread     <= 1'b0;
          ID_EX_alusrc      <= 1'b0;
          ID_EX_branch      <= 1'b0;
          ID_EX_jal         <= 1'b0;
          ID_EX_jalr        <= 1'b0;
          ID_EX_alucont     <= 5'b00000;
          ID_EX_rs1         <= 5'b00000;
          ID_EX_rs2         <= 5'b00000;
          ID_EX_rd          <= 5'b00000;
          ID_EX_rs1_data    <= 32'b0;
          ID_EX_rs2_data    <= 32'b0;
          ID_EX_inst        <= 32'b0;
          flush_flag        <= 1'b0;
        end
        else
        begin
          ID_EX_auipc           <= auipc;
          ID_EX_lui             <= lui;
          ID_EX_regwrite        <= regwrite;
          ID_EX_memtoreg        <= memtoreg;
          ID_EX_memwrite        <= memwrite;
          ID_EX_memread         <= IF_ID_memread;
          ID_EX_alusrc          <= alusrc;
          ID_EX_alucont         <= alucontrol;
          ID_EX_rs1             <= rs1;
          ID_EX_rs2             <= rs2;
          ID_EX_rd              <= rd;
          ID_EX_rs1_data        <= rs1_data;
          ID_EX_rs2_data        <= rs2_data;
          ID_EX_pc              <= IF_ID_pc;
          ID_EX_imm_u           <= auipc_lui_imm;
          ID_EX_imm_i           <= se_imm_itype;
          ID_EX_imm_s           <= se_imm_stype;
          ID_EX_inst            <= IF_ID_inst;
          ID_EX_branch          <= branch;
          ID_EX_jal             <= jal;
          ID_EX_jalr            <= jalr;
          ID_EX_imm_b           <= se_br_imm;
          ID_EX_imm_j           <= se_jal_imm;
          flush_flag            <= IF_flush;
        end
    end

    // EX_MEM FF
    reg [31:0] EX_MEM_rs2_data, EX_MEM_pc;
    reg [4:0]  EX_MEM_rd;
    reg        EX_MEM_regwrite, EX_MEM_memtoreg, EX_MEM_memread, EX_MEM_jal;
    always @(posedge clk)
    begin
      EX_MEM_rs2_data   <= ID_EX_rs2_data;
      EX_MEM_aluout     <= aluout;
      EX_MEM_pc         <= ID_EX_pc;
      EX_MEM_rd         <= ID_EX_rd;
      EX_MEM_regwrite   <= ID_EX_regwrite;
      EX_MEM_memtoreg   <= ID_EX_memtoreg;
      EX_MEM_memwrite   <= ID_EX_memwrite;
      EX_MEM_memread    <= ID_EX_memread;
      EX_MEM_jal        <= ID_EX_jal;
    end

    // MEM_WB FF 
    reg [31:0] MEM_WB_aluout, MEM_WB_MemRdata, MEM_WB_pc;
    reg [4:0]  MEM_WB_rd;
    reg        MEM_WB_regwrite, MEM_WB_memtoreg, MEM_WB_jal;
    always @(posedge clk)
    begin
      MEM_WB_aluout     <= EX_MEM_aluout;
      MEM_WB_MemRdata   <= MemRdata;
      MEM_WB_rd         <= EX_MEM_rd;
      MEM_WB_regwrite   <= EX_MEM_regwrite;
      MEM_WB_memtoreg   <= EX_MEM_memtoreg;
      MEM_WB_jal        <= EX_MEM_jal;
      MEM_WB_pc         <= EX_MEM_pc;
    end

    // Forwarding Unit  
    assign fw_rs1_exe       = (rs1 != 0) & (rs1 == ID_EX_rd)  & (~ID_EX_memwrite);
    assign fw_rs2_exe       = (rs2 != 0) & (rs2 == ID_EX_rd)  & (~ID_EX_memwrite);
    assign fw_rs1_Rdata     = (rs1 != 0) & (rs1 == EX_MEM_rd) & (EX_MEM_memread); 
    assign fw_rs2_Rdata     = (rs2 != 0) & (rs2 == EX_MEM_rd) & (EX_MEM_memread);
    assign fw_rs1_mem       = (rs1 != 0) & (rs1 == EX_MEM_rd) & (~EX_MEM_memread);
    assign fw_rs2_mem       = (rs2 != 0) & (rs2 == EX_MEM_rd) & (~EX_MEM_memread);
    assign fw_rs1_wb        = (rs1 != 0) & (rs1 == MEM_WB_rd);
    assign fw_rs2_wb        = (rs2 != 0) & (rs2 == MEM_WB_rd);

    always@ (*)
    begin
      if     (fw_rs1_exe)   rs1_data = aluout;
      else if(fw_rs1_Rdata) rs1_data = MemRdata;
      else if(fw_rs1_mem)   rs1_data = EX_MEM_aluout;
      else if(fw_rs1_wb)    rs1_data = rd_data;
      else                  rs1_data = rf_rs1_data;
    end
    always@ (*)
    begin
      if     (fw_rs2_exe)   rs2_data = aluout;
      else if(fw_rs2_Rdata) rs2_data = MemRdata;
      else if(fw_rs2_mem)   rs2_data = EX_MEM_aluout;
      else if(fw_rs2_wb)    rs2_data = rd_data;
      else                  rs2_data = rf_rs2_data;
    end

    // Hazard Detection Unit 
    assign stall = (ID_EX_memread) & ((ID_EX_rd == rs1) | (ID_EX_rd == rs2));
  ///////////////////////
  

  assign funct3 = ID_EX_inst[14:12];

  //
  // PC (Program Counter) logic 
  //
  assign f3beq  = (funct3 == 3'b000);
  assign f3blt  = (funct3 == 3'b100);
  assign f3bgeu = (funct3 == 3'b111);
  assign f3bne  = (funct3 == 3'b001);

  // ###########################################################

  wire   branch_taken;
  assign branch_taken = (beq_taken | bne_taken | bge_taken | blt_taken | bgeu_taken | bltu_taken);

  assign f3bge  = (funct3 == 3'b101);
  assign f3bltu = (funct3 == 3'b110);

  assign bge_taken   =  ID_EX_branch & f3bge  & (Nflag == Vflag);
  assign bltu_taken  =  ID_EX_branch & f3bltu & (~Cflag);

  // ###########################################################

  assign beq_taken   =  ID_EX_branch & f3beq  & Zflag;
  assign blt_taken   =  ID_EX_branch & f3blt  & (Nflag != Vflag);
  assign bgeu_taken  =  ID_EX_branch & f3bgeu & Cflag;
  assign bne_taken   =  ID_EX_branch & f3bne  & (~Zflag);

  assign branch_dest = (ID_EX_pc + ID_EX_imm_b);
  assign jal_dest 	 = (ID_EX_pc + ID_EX_imm_j);
  assign jalr_dest   = aluout[31:0];
  
  //######################################################### 노정훈 end

  always @(posedge clk, posedge reset)
  begin
    if (reset)  pc <= 32'b0;
	  else 
	  begin
	    if      (branch_taken)          // branch_taken
			  pc <= #`simdelay branch_dest;
		  else if (ID_EX_jal)             // jal
				pc <= #`simdelay jal_dest;
      else if (ID_EX_jalr)
        pc <= #`simdelay jalr_dest;
      // ######################## stall 추가
		  else if (stall | IF_flush | flush_flag)
        pc <= #`simdelay pc;
      //##################
      else
			  pc <= #`simdelay (pc + 4);
	  end
  end

  //######################################################### 노정훈 start

  // JAL immediate
  assign jal_imm[20:1]    = {IF_ID_inst[31], IF_ID_inst[19:12], IF_ID_inst[20], IF_ID_inst[30:21]};
  assign se_jal_imm[31:0] = {{11{jal_imm[20]}}, jal_imm[20:1],1'b0};

  // Branch immediate
  assign br_imm[12:1]     = {IF_ID_inst[31], IF_ID_inst[7], IF_ID_inst[30:25], IF_ID_inst[11:8]};
  assign se_br_imm[31:0]  = {{19{br_imm[12]}}, br_imm[12:1],1'b0};

  // 
  // Register File 
  //
  regfile i_regfile(
    .clk			(clk),
    .we		  	(MEM_WB_regwrite),
    .rs1			(rs1),
    .rs2			(rs2),
    .rd			  (MEM_WB_rd),
    .rd_data	(rd_data),
    .rs1_data	(rf_rs1_data),
    .rs2_data	(rf_rs2_data));

  assign MemWdata = EX_MEM_rs2_data;

  //######################################################### 노정훈 end

	//
	// ALU 
	//
	alu i_alu(
		.a			  (alusrc1),
		.b		  	(alusrc2),
  //######################################################### 노정훈 start
		.alucont	(ID_EX_alucont),
  //######################################################### 노정훈 end
		.result 	(aluout),
		.N			  (Nflag),
		.Z			  (Zflag),
		.C			  (Cflag),
		.V			  (Vflag));

	// 1st source to ALU (alusrc1)
	always@(*)
	begin
    if      (ID_EX_auipc)	  alusrc1[31:0]  =  ID_EX_pc;
		else if (ID_EX_lui)     alusrc1[31:0]  =  32'b0;
		else          		      alusrc1[31:0]  =  ID_EX_rs1_data[31:0];
	end
	
	// 2nd source to ALU (alusrc2)
	always@(*)
	begin
    if	    (ID_EX_auipc  | ID_EX_lui)		  alusrc2[31:0] = ID_EX_imm_u[31:0];
		else if (ID_EX_alusrc & ID_EX_memwrite)	alusrc2[31:0] = ID_EX_imm_s[31:0];
		else if (ID_EX_alusrc)					        alusrc2[31:0] = ID_EX_imm_i[31:0];
		else									                  alusrc2[31:0] = ID_EX_rs2_data[31:0];
	end
	
	assign se_imm_itype[31:0]  = {{20{IF_ID_inst[31]}}, IF_ID_inst[31:20]};
	assign se_imm_stype[31:0]  = {{20{IF_ID_inst[31]}}, IF_ID_inst[31:25], IF_ID_inst[11:7]};
	assign auipc_lui_imm[31:0] = {IF_ID_inst[31:12], 12'b0};


	// Data selection for writing to RF
	always@(*)
	begin 
		if	    (MEM_WB_jal)	    rd_data[31:0] = MEM_WB_pc + 4;
		else if (MEM_WB_memtoreg)	rd_data[31:0] = MEM_WB_MemRdata; 
		else					          	rd_data[31:0] = MEM_WB_aluout; 
	end
	
endmodule