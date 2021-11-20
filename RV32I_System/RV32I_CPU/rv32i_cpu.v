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

  assign Memwrite = memwrite ;

  // ##### 노정훈 : Start #####

  wire [31:0] pc_if, inst_if, pc_id, inst_id;

  assign pc_if = pc;
  assign inst_if = inst;

  

  IF_ID_FF pl0(
     .pc   (pc_if),
     .inst (inst_if),
     .clk  (clk),
     .en   (1'b1),
     .pc_out   (pc_id[31:0]),
     .inst_out (inst_id[31:0])
  );

  // ##### 노정훈 : End   #####

  // Instantiate Controller
  controller i_controller(
    // ##### 노정훈 : Start #####
    .opcode		(inst_id[6:0]), 
		.funct7		(inst_id[31:25]), 
		.funct3		(inst_id[14:12]), 
    // ##### 노정훈 : End   #####
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
		.alucontrol		(alucontrol),
		.pc				(pc),
    // ##### 노정훈 : Start #####
    .pc_id      (pc_id),
		.inst				(inst_id),
    // ##### 노정훈 : End   #####
		.aluout			(Memaddr), 
		.MemWdata		(MemWdata),
		.MemRdata		(MemRdata));

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
       
       // ##### 노정훈 : Start #####
       3'b001:  alucontrol <= #`simdelay 5'b00100; // ssli (slli)
       // ##### 노정훈 : End   #####
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

                // ##### 노정훈 : Start #####
                input  [31:0] pc_id,
                output [31:0] pc,
                // ##### 노정훈 : End   #####
                output [31:0] aluout,
                output [31:0] MemWdata,
                input  [31:0] MemRdata);

  wire [4:0]  rs1, rs2, rd;
  wire [2:0]  funct3;
  wire [31:0] rs1_data, rs2_data;
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
  wire		  Nflag, Zflag, Cflag, Vflag;
  wire		  f3beq, f3blt, f3bgeu;
  wire		  beq_taken;
  wire		  blt_taken;
  wire      bgeu_taken;

  assign rs1 = inst[19:15];
  assign rs2 = inst[24:20];
  assign rd  = inst[11:7];
  assign funct3  = inst[14:12];

  // ##### 노정훈 : Start #####
  wire [31:0] pc_ex, rs1_ex, rs2_ex, immu_ex, imms_ex, immi_ex, se_jal_imm_ex, se_br_imm_ex;
  wire [4:0] rd_ex;

  reg [31:0] pc_mem_in;
  wire z_ex;
  assign z_ex = Zflag;
  
  // ##### 노정훈 : End   #####


  //
  // PC (Program Counter) logic 
  //
  assign f3beq  = (funct3 == 3'b000);
  assign f3blt  = (funct3 == 3'b100);
  assign f3bgeu = (funct3 == 3'b111);

  assign beq_taken  =  branch & f3beq & Zflag;
  assign blt_taken  =  branch & f3blt & (Nflag != Vflag);
  assign bgeu_taken =  branch & f3bgeu & Cflag;

  assign branch_dest = (pc_ex + se_br_imm_ex);
  assign jal_dest 	= (pc_ex + se_jal_imm_ex);
  assign jalr_dest  = aluout[31:0];

  always @(posedge clk, posedge reset)
  begin
     if (reset)  pc_mem_in <= 32'b0;
	  else 
	  begin
	      if (beq_taken | blt_taken | bgeu_taken) // branch_taken
				pc_mem_in <= #`simdelay branch_dest;
		   else if (jal) // jal
				pc_mem_in <= #`simdelay jal_dest;
       else if (jalr)
        pc_mem_in <= #`simdelay jalr_dest;
		   else 
				pc_mem_in <= #`simdelay (pc_ex + 4);
	  end
  end


  // JAL immediate
  assign jal_imm[20:1] = {inst[31],inst[19:12],inst[20],inst[30:21]};
  assign se_jal_imm[31:0] = {{11{jal_imm[20]}},jal_imm[20:1],1'b0};

  // Branch immediate
  assign br_imm[12:1] = {inst[31],inst[7],inst[30:25],inst[11:8]};
  assign se_br_imm[31:0] = {{19{br_imm[12]}},br_imm[12:1],1'b0};


  // ##### 노정훈 : Start #####
  ID_EX_FF pl1(
    .clk      (clk),
    .en       (1'b1),
    
    .pc       (pc_id),
    .rs1      (rs1_data),
    .rs2      (rs2_data),
    .rd       (rd),
    .imm_u    (auipc_lui_imm),
    .imm_s    (se_imm_stype),
    .imm_i    (se_imm_itype),
    .imm_jal  (se_jal_imm),
    .imm_br   (se_br_imm),

    .pc_out   (pc_ex),
    .rs1_out  (rs1_ex),
    .rs2_out  (rs2_ex),
    .rd_out   (rd_ex),
    .imm_u_out (immu_ex),
    .imm_s_out (imms_ex),
    .imm_i_out (immi_ex),
    .imm_jal_out (se_jal_imm_ex),
    .imm_br_out  (se_br_imm_ex)
  );
  // ##### 노정훈 : End   #####


  // 
  // Register File 
  //
  regfile i_regfile(
    .clk			(clk),
    .we			(regwrite),
    .rs1			(rs1),
    .rs2			(rs2),
    .rd			(rd),
    .rd_data	(rd_data),
    .rs1_data	(rs1_data),
    .rs2_data	(rs2_data));


	assign MemWdata = rs2_data;


	//
	// ALU 
	//
	alu i_alu(
		.a			(alusrc1),
		.b			(alusrc2),
		.alucont	(alucontrol),
		.result	(aluout),
		.N			(Nflag),
		.Z			(Zflag),
		.C			(Cflag),
		.V			(Vflag));

	// 1st source to ALU (alusrc1)
	always@(*)
	begin
    // ##### 노정훈 : Start #####
		if      (auipc)	alusrc1[31:0]  =  pc_ex;
    // ##### 노정훈 : End   #####
		else if (lui) 		alusrc1[31:0]  =  32'b0;
    // ##### 노정훈 : Start #####
		else          		alusrc1[31:0]  =  rs1_ex[31:0];
    // ##### 노정훈 : End   #####
	end
	
	// 2nd source to ALU (alusrc2)
	always@(*)
	begin
    // ##### 노정훈 : Start #####
		if	     (auipc | lui)			alusrc2[31:0] = immu_ex[31:0];
		else if (alusrc & memwrite)	alusrc2[31:0] = imms_ex[31:0];
		else if (alusrc)					alusrc2[31:0] = immi_ex[31:0];
		else									alusrc2[31:0] = rs2_ex[31:0];
    // ##### 노정훈 : End   #####
	end
	
	assign se_imm_itype[31:0] = {{20{inst[31]}},inst[31:20]};
	assign se_imm_stype[31:0] = {{20{inst[31]}},inst[31:25],inst[11:7]};
	assign auipc_lui_imm[31:0] = {inst[31:12],12'b0};


	// Data selection for writing to RF
	always@(*)
	begin
		if	     (jal)			rd_data[31:0] = pc + 4;
		else if (memtoreg)	rd_data[31:0] = MemRdata;
		else						rd_data[31:0] = aluout;
	end
	
endmodule
