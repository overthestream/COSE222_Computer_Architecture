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



  // ##### 노정훈 : Start #####
  wire [2:0] mem_wb_control_in, mem_wb_control_out;
  wire [5:0] mem_control;
  assign mem_wb_control_in = {mem_control[1], mem_control[5], mem_control[4]};
  assign Memwrite = mem_control[3] ;
  wire [31:0] pc_if, inst_if, pc_id, inst_id, mem_alu, mem_read, wb_pc_mem, wb_pc_wb;

  assign pc_if = pc;
  assign inst_if = inst;

  wire [4:0] wbreg, wbreg_out;
  wire ex_memread, rd_ex, pc_write, if_id_write, stall_control;
  IF_ID_FF pl0(
     .pc   (pc_if),
     .inst (inst_if),
     .clk  (clk),
     .en   (~if_id_write),
     .pc_out   (pc_id[31:0]),
     .inst_out (inst_id[31:0])
  );

  MEM_WB_FF pl3(
    .clk    (clk),
    .read_data  (MemRdata),
    .address    (Memaddr),
    .rd         (wbreg),
    .wb_pc      (wb_pc_mem),
    .control    (mem_wb_control_in),

    .control_out(mem_wb_control_out),
    .wb_pc_out  (wb_pc_wb),
    .read_data_out (mem_read),
    .address_out (mem_alu),
    .rd_out     (wbreg_out)
  );

  hazard_detection hdu(.ex_memread (ex_memread),
                       .ex_rd      (rd_ex),
                       .id_rs1     (inst_if[19:15]),
                       .id_rs2     (inst_if[24:20]),
                       .pc_write   (pc_write),
                       .if_id_write(if_id_write),
                       .stall_control (stall_control));

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
    .memtoreg   (memtoreg),
		.memwrite		(memwrite),
		.branch			(branch),
		.alusrc			(alusrc),
    .regwrite   (regwrite),
		.jal				(jal),
		.jalr				(jalr),
		.alucontrol		(alucontrol),
		.pc				(pc),
    // ##### 노정훈 : Start #####
    .stall_control  (stall_control),
    .pc_write       (pc_write),
    .rd_ex          (rd_ex),
		.memtoreg_wb		(mem_wb_control_out[0]),
		.regwrite_wb		(mem_wb_control_out[1]),
    .pc_id      (pc_id),
		.inst				(inst_id),
    .rd_out     (wbreg),
    .wb_pc      (wb_pc_wb),
    .wb_pc_out  (wb_pc_mem),
    .wb_rd      (wbreg_out),
    .wb_rddata  (mem_read),
    .wb_rdalu   (mem_alu),
		.alumem			(Memaddr), 
    .mem_control(mem_control),
    .jal_wb     (mem_wb_control_out[2]),
    .ex_memread (ex_memread),
    // ##### 노정훈 : End   #####

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
                input  [4:0]  alucontrol, stall_control,
                input         branch,
                input         jal,
                input         jalr,

                // ##### 노정훈 : Start #####
                input   jal_wb, memtoreg_wb, regwrite_wb, pc_write,
                input  [31:0] pc_id, wb_rddata, wb_rdalu, wb_pc,
                input  [4:0]  wb_rd,
                output reg [31:0] pc,
                output ex_memread, 
                output [31:0] alumem, wb_pc_out,
                output [4:0]  rd_out, rd_ex,
                output [5:0]  mem_control,
                // ##### 노정훈 : End   #####
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


  reg [31:0] pc_mem_in;
  
  wire [31:0] next_pc_ex, next_pc_mem, alu_ex;
  assign next_pc_ex = pc_ex+4;
  assign wb_pc_out = next_pc_mem;
  wire Zflag_mem;

  wire [31:0] branch_mem, jal_mem, jalr_mem;

  wire [13:0] ID_EX_control_in, ID_EX_control_out;
  assign ex_memread =ID_EX_control_out[9];
  assign ID_EX_control_in = {auipc, lui, regwrite, memtoreg, memwrite, alusrc, alucontrol [4:0], branch, jal, jalr};
  
  wire [5:0] EX_MEM_control_in, EX_MEM_control_out;
    
  assign EX_MEM_control_in = (stall_control) ? {14{1'b0}} : {ID_EX_control_out[11:9],ID_EX_control_out[2:0]} ;
  
  assign mem_control = EX_MEM_control_out;
  // ##### 노정훈 : End   #####


  //
  // PC (Program Counter) logic 
  //
  assign f3beq  = (funct3 == 3'b000);
  assign f3blt  = (funct3 == 3'b100);
  assign f3bgeu = (funct3 == 3'b111);

  // ##### 노정훈 : Start   #####
  assign beq_taken  =  mem_control[2] & f3beq & Zflag_mem;
  assign blt_taken  =  mem_control[2] & f3blt & (Nflag != Vflag);
  assign bgeu_taken =  mem_control[2] & f3bgeu & Cflag;
  // ##### 노정훈 : End   #####

  assign branch_dest = (pc_ex + se_br_imm_ex);
  assign jal_dest 	= (pc_ex + se_jal_imm_ex);
  assign jalr_dest  = alu_ex[31:0];

  always @(posedge clk, posedge reset)
  begin
    if (reset)  pc <= 32'b0;
	  else 
	  begin
	    if (beq_taken | blt_taken | bgeu_taken) // branch_taken
      // ##### 노정훈 : Start   #####
				pc <= #`simdelay branch_mem;
		  else if (mem_control[1]) // jal
				pc <= #`simdelay jal_mem;
      else if (mem_control[0])
        pc <= #`simdelay jalr_mem;
		  else if (pc_write)
        pc <= pc;
      else
				pc <= #`simdelay pc+4; // next_pc_mem; pc_id+4?
      // ##### 노정훈 : End   #####
	  end
  end


  // JAL immediate
  assign jal_imm[20:1] = {inst[31],inst[19:12],inst[20],inst[30:21]};
  assign se_jal_imm[31:0] = {{11{jal_imm[20]}},jal_imm[20:1],1'b0};

  // Branch immediate
  assign br_imm[12:1] = {inst[31],inst[7],inst[30:25],inst[11:8]};
  assign se_br_imm[31:0] = {{19{br_imm[12]}},br_imm[12:1],1'b0};


  // ##### 노정훈 : Start #####

  wire [4:0] rsreg1_out, rsreg2_out;

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
    .control  (ID_EX_control_in),

    .rsreg1   (rs1),
    .rsreg2   (rs2),

    .rsreg1_out (rsreg1_out),
    .rsreg2_out (rsreg2_out),

    .control_out  (ID_EX_control_out),
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

  EX_MEM_FF pl2(
    .clk   (clk),
    .en   (1'b1),

    .pc         (next_pc_ex),
    .branch     (branch_dest),
    .jal        (jal_dest),
    .jalr       (jalr_dest),
    .wr_data_in (rs2_ex),
    .result     (alu_ex),
    .rd         (rd_ex),
    .zflag      (Zflag),
    .control    (EX_MEM_control_in),

    .control_out(EX_MEM_control_out),
    .pc_out     (next_pc_mem),
    .branch_out (branch_mem),
    .jal_out    (jal_mem),
    .jalr_out   (jalr_mem),
    .wr_data_out(MemWdata),
    .result_out (alumem),
    .rd_out     (rd_out),
    .zflag_out  (Zflag_mem)
  );
  // ##### 노정훈 : End   #####


  // 
  // Register File 
  //
  regfile i_regfile(
    .clk			(clk),
    .we			(regwrite_wb),
    .rs1			(rs1),
    .rs2			(rs2),
    .rd			(wb_rd),
    .rd_data	(rd_data),
    .rs1_data	(rs1_data),
    .rs2_data	(rs2_data));

  // ##### 노정훈 : Start   ######
	//assign MemWdata = rs2_data;
  // ##### 노정훈 : End   #####

	//
	// ALU 
	//
	alu i_alu(
		.a			(alusrc1),
		.b			(alusrc2),
    // ##### 노정훈 : Start #####
		.alucont	(ID_EX_control_out[7:3]),
		.result	(alu_ex),
    // ##### 노정훈 : End   #####
		.N			(Nflag),
		.Z			(Zflag),
		.C			(Cflag),
		.V			(Vflag));

  // ##### 노정훈 : Start #####
  wire fw_rs1_mem, fw_rs2_mem, fw_rs1_wb, fw_rs2_wb;
  forwarding_unit fu( .ex_rs1 (rsreg1_out),
                      .ex_rs2 (rsreg2_out), 
                      .mem_rd (rd_out),
                      .wb_rd  (wb_rd),
											.regwrite_mem (EX_MEM_control_out[5]), 
                      .regwrite_wb (regwrite_wb),
											.fw_rs1_mem  (fw_rs1_mem),
                      .fw_rs2_mem  (fw_rs2_mem),
                      .fw_rs1_wb   (fw_rs1_wb),
                      .fw_rs2_wb   (fw_rs2_wb));
  // ##### 노정훈 : End   #####

	// 1st source to ALU (alusrc1)
	always@(*)
	begin
    // ##### 노정훈 : Start #####
    if      (fw_rs1_mem)  alusrc1[31:0] = alumem;
    else if (fw_rs1_wb)   alusrc1[31:0] = rd_data;
		else if (ID_EX_control_out[13])	alusrc1[31:0]  =  pc_ex;
		else if (ID_EX_control_out[12]) 		alusrc1[31:0]  =  32'b0;
		else          		alusrc1[31:0]  =  rs1_ex[31:0];
    // ##### 노정훈 : End   #####
	end
	
	// 2nd source to ALU (alusrc2)
	always@(*)
	begin
    // ##### 노정훈 : Start #####
		if      (fw_rs2_mem)  alusrc2[31:0] = alumem;
    else if (fw_rs2_wb)   alusrc2[31:0] = rd_data;
		else if	(ID_EX_control_out[13] | ID_EX_control_out[12])			alusrc2[31:0] = immu_ex[31:0];
		else if (ID_EX_control_out[8] & ID_EX_control_out[9])	alusrc2[31:0] = imms_ex[31:0];
		else if (ID_EX_control_out[8])					alusrc2[31:0] = immi_ex[31:0];
		else									alusrc2[31:0] = rs2_ex[31:0];
    // ##### 노정훈 : End   #####
	end
	
	assign se_imm_itype[31:0] = {{20{inst[31]}},inst[31:20]};
	assign se_imm_stype[31:0] = {{20{inst[31]}},inst[31:25],inst[11:7]};
	assign auipc_lui_imm[31:0] = {inst[31:12],12'b0};


	// Data selection for writing to RF
	always@(*)
	begin
    // ##### 노정훈 : Start #####
		if	     (jal_wb)			rd_data[31:0] = wb_pc;
		else if (memtoreg_wb)	rd_data[31:0] = wb_rddata;
		else						rd_data[31:0] = wb_rdalu;
    // ##### 노정훈 : End   #####
	end
	
endmodule
