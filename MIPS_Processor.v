
/******************************************************************
* Description
*	This is the top-level of a MIPS processor that can execute the next set of instructions:
*		add
*		addi
*		sub
*		ori
*		or
*		and
*		nor
* This processor is written Verilog-HDL. Also, it is synthesizable into hardware.
* Parameter MEMORY_DEPTH configures the program memory to allocate the program to
* be execute. If the size of the program changes, thus, MEMORY_DEPTH must change.
* This processor was made for computer architecture class at ITESO.
* Version:
*	1.5
* Author:
*	Dr. José Luis Pizano Escalante
* email:
*	luispizano@iteso.mx
* Date:
*	2/09/2018
******************************************************************/

module MIPS_Processor
#(
	parameter MEMORY_DEPTH = 32,
	parameter DATA_WIDTH = 32
)

(
	// Inputs
	input clk,
	input reset,
	input [7:0] PortIn,
	// Output
	output [31:0] ALUResultOut,
	output [31:0] PortOut
);
//******************************************************************/
//******************************************************************/
assign  PortOut = 0;

//******************************************************************/
//******************************************************************/

// signals to connect modules
wire PCSrc; 
wire BranchNE_true_wire;
wire BranchEQ_true_wire;
wire BranchNE_wire;
wire BranchEQ_wire;
wire MemRead_wire;
wire MemtoReg_wire;
wire MemWrite_wire;
wire RegDst_wire;
wire NotZeroANDBrachNE;
wire ZeroANDBrachEQ;
wire ORForBranch;
wire ALUSrc_wire;
wire RegWrite_wire;
wire Zero_wire;
wire Jal_wire;
wire JR_wire;
wire Jump_wire;
wire Flush_wire;
wire [2:0] ALUOp_wire;
wire [3:0] ALUOperation_wire;
wire [4:0] WriteRegister_wire;
wire [4:0] Final_WriteRegister_wire;
wire [31:0] PC_wire;
wire [31:0] BranchedPC_wire;
wire [31:0] MUX_RFWrite_wire;
wire [31:0] RAMReadData_wire;
wire [31:0] Instruction_wire;
wire [31:0] ReadData1_wire;
wire [31:0] ReadData2_wire;
wire [31:0] InmmediateExtend_wire;
wire [31:0] ReadData2OrInmmediate_wire;
wire [31:0] ALUResult_wire;
wire [31:0] PC_4_wire;
wire [31:0] PC_8_wire;
wire [31:0] InmmediateExtendAnded_wire;
wire [31:0] PCtoBranch_wire;
wire [31:0] ShiftToBranch_wire;
wire [31:0] ShiftedBranch_wire;
wire [31:0] NewPC_wire;  
wire [31:0] FinalPC_wire; 
wire [31:0] WriteData_wire; 
wire [31:0] AbsolutePC_wire;

wire [31:0] IFID_PC_4_wire;
wire [31:0] IFID_Instruction_wire;

//Agregado en Tarea 2
//IF_ID
wire IFID_Jump_wire;
wire IFID_Jal_wire;
wire IFID_RegDst_wire;
wire IFID_BranchEQ_wire;
wire IFID_BranchNE_wire;
wire IFID_MemRead_wire;
wire IFID_MemtoReg_wire;
wire IFID_MemWrite_wire;
wire IFID_ALUSrc_wire;
wire IFID_RegWrite_wire;
wire [2:0] IFID_ALUOp_wire;

//ID_EX
wire IDEX_Jump_wire;
wire IDEX_Jal_wire;
wire IDEX_RegDst_wire;
wire IDEX_BranchEQ_wire;
wire IDEX_BranchNE_wire;
wire IDEX_MemRead_wire;
wire IDEX_MemtoReg_wire;
wire IDEX_MemWrite_wire;
wire IDEX_ALUSrc_wire;
wire IDEX_RegWrite_wire;
wire [3:0] IDEX_ALUOperation_wire;
wire [31:0] IDEX_PC_4_wire;
wire [31:0] IDEX_ReadData1_wire;
wire [31:0] IDEX_ReadData2_wire;
wire [31:0] IDEX_InmmediateExtend_wire;
wire [31:0] IDEX_Instruction_wire;

//EX_MEM
wire EXMEM_Jump_wire; 
wire EXMEM_Jal_wire;
wire EXMEM_BranchEQ_wire;
wire EXMEM_BranchNE_wire;
wire EXMEM_MemRead_wire;
wire EXMEM_MemtoReg_wire; 
wire EXMEM_MemWrite_wire;
wire EXMEM_RegWrite_wire;
wire EXMEM_Zero_wire;
wire [31:0] EXMEM_NewPC_wire;
wire [31:0] EXMEM_ALUResult_wire; 
wire [31:0] EXMEM_ReadData2_wire; 
wire [31:0] EXMEM_FwdB_wire;
wire [31:0] EXMEM_PC_4_wire;
wire [4:0]  EXMEM_WriteRegister_wire;

//MEM_WB
wire MEMWB_MemtoReg_wire;
wire MEMWB_Jal_wire;
wire MEMWB_RegWrite_wire;
wire [31:0] MEMWB_RAMReadData_wire;
wire [31:0] MEMWB_ALUResult_wire;
wire [31:0] MEMWB_PC_4_wire;
wire [4:0] MEMWB_WriteRegister_wire;

//FW UNIT
wire [31:0] FwdA_wire;
wire [31:0] FwdB_wire;
wire [1:0] FwdA_sel_wire;
wire [1:0] FwdB_sel_wire;

wire hazard_write_wire;
wire stall_sel_wire;

//******************************************************************/
//******************************************************************/
//Register agregados en Tarea 2
/*
* Se necesitan 4 PLRegister:
* IF/ID
* ID/EX
* EX/MEM
* MEM/WB
* 
* Entre cada uno de los registros se agregan nuevos cables
* con una nomenclatura que identifica de donde salen o entran
* Los DataInput y DataOutput están concatenados
//******************************************************************/
//******************************************************************/

//****************************FETCH**********************************

Multiplexer2to1 //seleccionamos cual sera la siguiente instruccion 
#(
	.NBits(32)
)
MUX_ForBranch
(
	.Selector(PCSrc),
	.MUX_Data0(PC_4_wire),
	.MUX_Data1(NewPC_wire),
	
	.MUX_Output(BranchedPC_wire)

);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForJump
(
	.Selector(Jump_wire),
	.MUX_Data0(BranchedPC_wire),
	.MUX_Data1({PC_4_wire[31:28], IFID_Instruction_wire[25:0], 2'b0}),
	
	.MUX_Output(FinalPC_wire)

);

Multiplexer2to1 //seleccionamos entre pc o jump
#(
	.NBits(32)
)
MUX_ForJR
(
	.Selector(JR_wire),
	.MUX_Data0(FinalPC_wire),
	.MUX_Data1(ReadData1_wire),
	
	.MUX_Output(AbsolutePC_wire)

);

PC_Register
#(
	.N(32)
)
program_counter
(
	.clk(clk),
	.reset(reset),
	.write(hazard_write_wire),
	.NewPC(AbsolutePC_wire), //se cambió para aumentar 4 o al salto
	.PCValue(PC_wire)
);

Adder32bits
PC_Plus_4
(
	.Data0(PC_wire),
	.Data1(4),
	
	.Result(PC_4_wire)
);

ProgramMemory
#(
	.MEMORY_DEPTH(300)
)
ROMProgramMemory
(
	.Address(PC_wire),
	.Instruction(Instruction_wire)
);


//******************************************************************

PipeRegister

#(
	.N(64)
)
Register_IFID
(
	 .clk(clk),
	 .reset(reset),
	 .enable(hazard_write_wire),
	 .flush(Flush_wire),
	 .DataInput({PC_4_wire, Instruction_wire}),
	
	
	.DataOutput({IFID_PC_4_wire, IFID_Instruction_wire})
);

//******************************DECODE******************************

HazardUnit
Hazard_Unit(
	.IDEX_MemRead(IDEX_MemRead_wire),
	.jump(Jump_wire),
	.jal(MEMWB_Jal_wire),
	.branch(PCSrc),
	.jr(JR_wire),
	.IDEX_Rt(IDEX_Instruction_wire[20:16]),
	.IFID_Rs(IFID_Instruction_wire[25:21]),
	.IFID_Rt(IFID_Instruction_wire[20:16]),
	
	.IFID_write(hazard_write_wire),
	.stall_sel(stall_sel_wire),
	.flush(Flush_wire)
);

Multiplexer2to1
#(
	.NBits(13)
)
MUX_ForHazard
(
	.Selector(stall_sel_wire),
	.MUX_Data0({Jump_wire, Jal_wire, RegDst_wire,
					BranchNE_wire, BranchEQ_wire, MemRead_wire,
					MemtoReg_wire, MemWrite_wire, ALUOp_wire,
					ALUSrc_wire, RegWrite_wire}),
					
	.MUX_Data1(0),
	
	.MUX_Output({IFID_Jump_wire, IFID_Jal_wire, IFID_RegDst_wire,
					IFID_BranchNE_wire, IFID_BranchEQ_wire, IFID_MemRead_wire,
					IFID_MemtoReg_wire, IFID_MemWrite_wire, IFID_ALUOp_wire,
					IFID_ALUSrc_wire, IFID_RegWrite_wire})

);

RegisterFile
Register_File
(
	.clk(clk),
	.reset(reset),
	.RegWrite(MEMWB_RegWrite_wire),
	.WriteRegister(Final_WriteRegister_wire), //elige escribir RA o dato
	.ReadRegister1(IFID_Instruction_wire[25:21]),
	.ReadRegister2(IFID_Instruction_wire[20:16]),
	.WriteData(WriteData_wire), //JumpAdress 
	.ReadData1(ReadData1_wire),
	.ReadData2(ReadData2_wire)

);

assign Zero_wire = ((ReadData1_wire - ReadData2_wire)==0)? 1'b1: 1'b0;

Control
ControlUnit
(
	.OP(IFID_Instruction_wire[31:26]),
	.Jump(Jump_wire),
	.Jal(Jal_wire),
	.RegDst(RegDst_wire),
	.BranchNE(BranchNE_wire),
	.BranchEQ(BranchEQ_wire),
	.MemRead(MemRead_wire),
	.MemtoReg(MemtoReg_wire),
	.MemWrite(MemWrite_wire),
	.ALUOp(ALUOp_wire),
	.ALUSrc(ALUSrc_wire),
	.RegWrite(RegWrite_wire)
	
);

Adder32bits 
Branch_Adder //Agrega PC4 al JumpAddress para hacerla de 32 bits
(
	.Data0(ShiftedBranch_wire),
	.Data1(IFID_PC_4_wire),
	
	.Result(NewPC_wire) //queda PC4 + JumpAddress[25-0] + 00
);


SignExtend
SignExtendForConstants
(   
	.DataInput(IFID_Instruction_wire[15:0]),
   .SignExtendOutput(InmmediateExtend_wire)
);

ShiftLeft2 
Shift_For_Branch //Mueve la direccion << 2 para poder accedar a memoria (lo haca multiplo de 4) 
(   
	.DataInput(InmmediateExtend_wire),
	.DataOutput(ShiftedBranch_wire)

);

Multiplexer2to1 //vemos si vamos a hacer jal o ejecutaremos la siguiente instruccion
#(
	.NBits(32)
)
MUX_ForJal
(
	.Selector(MEMWB_Jal_wire),
	.MUX_Data0(MUX_RFWrite_wire),
	.MUX_Data1(MEMWB_PC_4_wire),
	
	.MUX_Output(WriteData_wire)

);

Multiplexer2to1 //seleccionamos el registro en el que escribiremos RA/Registro N
#(
	.NBits(5)
)
MUX_ForWriteRegister
(
	.Selector(MEMWB_Jal_wire),
	.MUX_Data0(MEMWB_WriteRegister_wire),
	.MUX_Data1(5'b11111),
	
	.MUX_Output(Final_WriteRegister_wire)

);

ALUControl
ArithmeticLogicUnitControl
(
	.ALUOp(ALUOp_wire),
	.ALUFunction(InmmediateExtend_wire[5:0]),
	.ALUOperation(ALUOperation_wire),
	.JRsel(JR_wire)
	

);

//******************************************************************

PipeRegister

#(
	.N(174)
)
Register_IDEX
(
	 .clk(clk),
	 .reset(reset),
	 .enable(1),
	 .flush(0),
	 .DataInput({IFID_Jump_wire, IFID_Jal_wire, IFID_RegDst_wire,
					IFID_BranchNE_wire, IFID_BranchEQ_wire, IFID_MemRead_wire,
					IFID_MemtoReg_wire, IFID_MemWrite_wire, ALUOperation_wire,
					IFID_ALUSrc_wire, IFID_RegWrite_wire, IFID_PC_4_wire,
					ReadData1_wire, ReadData2_wire, InmmediateExtend_wire,
					IFID_Instruction_wire}),
	
	
	.DataOutput({IDEX_Jump_wire, IDEX_Jal_wire, IDEX_RegDst_wire,
				IDEX_BranchNE_wire, IDEX_BranchEQ_wire, IDEX_MemRead_wire,
				IDEX_MemtoReg_wire, IDEX_MemWrite_wire, IDEX_ALUOperation_wire,
				IDEX_ALUSrc_wire, IDEX_RegWrite_wire, IDEX_PC_4_wire, 
				IDEX_ReadData1_wire, IDEX_ReadData2_wire, IDEX_InmmediateExtend_wire,
				IDEX_Instruction_wire})
);

//*******************************EXECUTE*******************************

Multiplexer3to1
#(
	.NBits(32)
)
MUX_Forward_A
(
	.Selector(FwdA_sel_wire),
	.MUX_Data0(IDEX_ReadData1_wire),
	.MUX_Data1(MUX_RFWrite_wire),
	.MUX_Data2(EXMEM_ALUResult_wire),
	.MUX_Output(FwdA_wire)

);

Multiplexer3to1
#(
	.NBits(32)
)
MUX_Forward_B
(
	.Selector(FwdB_sel_wire),
	.MUX_Data0(IDEX_ReadData2_wire),
	.MUX_Data1(MUX_RFWrite_wire),
	.MUX_Data2(EXMEM_ALUResult_wire),
	.MUX_Output(FwdB_wire)

);

Multiplexer2to1 //seleccionamos si vamos a leer de los registros o el valor de inmediato
#(
	.NBits(32)
)
MUX_ForReadDataAndInmediate
(
	.Selector(IDEX_ALUSrc_wire),
	.MUX_Data0(FwdB_wire),
	.MUX_Data1(IDEX_InmmediateExtend_wire),
	
	.MUX_Output(ReadData2OrInmmediate_wire)

);


Multiplexer2to1 //se selecciona el registro a escribir
#(
	.NBits(5)
)
MUX_ForRTypeAndIType
(
	.Selector(IDEX_RegDst_wire),
	.MUX_Data0(IDEX_Instruction_wire[20:16]),
	.MUX_Data1(IDEX_Instruction_wire[15:11]),
	
	.MUX_Output(WriteRegister_wire)

);

ALU
ArithmeticLogicUnit 
(
	.ALUOperation(IDEX_ALUOperation_wire),
	.A(FwdA_wire),
	.B(ReadData2OrInmmediate_wire),
	.ALUResult(ALUResult_wire),
	.shamt(IDEX_Instruction_wire[10:6])
);

ForwardUnit
Forward_Unit
(						 
	.EXMEM_RegWrite(EXMEM_RegWrite_wire),
	.MEMWB_RegWrite(MEMWB_RegWrite_wire),
	.IDEX_Rt(IDEX_Instruction_wire[20:16]),
	.IDEX_Rs(IDEX_Instruction_wire[25:21]),
	.MEMWB_Rd(MEMWB_WriteRegister_wire),
	.EXMEM_Rd(EXMEM_WriteRegister_wire), 
	.FwdA(FwdA_sel_wire),
	.FwdB(FwdB_sel_wire)
);
//******************************************************************

PipeRegister
#(
	.N(110)
)
Register_EXMEM
(
	 .clk(clk),
	 .reset(reset),
	 .enable(1),
	 .flush(0),
	 .DataInput({IDEX_Jump_wire, IDEX_Jal_wire,
				IDEX_BranchEQ_wire, IDEX_BranchNE_wire, IDEX_MemRead_wire,
				IDEX_MemtoReg_wire, IDEX_MemWrite_wire,
				IDEX_RegWrite_wire,Zero_wire, ALUResult_wire, 
				FwdB_wire, WriteRegister_wire, IDEX_PC_4_wire
				}),
	
	.DataOutput({EXMEM_Jump_wire, EXMEM_Jal_wire,
				EXMEM_BranchEQ_wire, EXMEM_BranchNE_wire, EXMEM_MemRead_wire,
				EXMEM_MemtoReg_wire, EXMEM_MemWrite_wire,
				EXMEM_RegWrite_wire,EXMEM_Zero_wire, EXMEM_ALUResult_wire, 
				EXMEM_FwdB_wire, EXMEM_WriteRegister_wire, EXMEM_PC_4_wire
	
	
	})
);

//***************************MEMORY*********************************

DataMemory
#(
	.DATA_WIDTH(DATA_WIDTH)
)
RAMDataMemory
(
	.WriteData(EXMEM_FwdB_wire),
	.Address(EXMEM_ALUResult_wire),
	.clk(clk),
	.MemWrite(EXMEM_MemWrite_wire),
	.MemRead(EXMEM_MemRead_wire),
	.ReadData(RAMReadData_wire)
);

assign BranchNE_true_wire = BranchNE_wire & ~(Zero_wire);
assign BranchEQ_true_wire = BranchEQ_wire & Zero_wire;
assign PCSrc = BranchNE_true_wire|BranchEQ_true_wire; //Define si es un salto u otra instruccion


//*****************************************************************/

PipeRegister
#(
	.N(104)
)
Register_MEMWB
(
	 .clk(clk),
	 .reset(reset),
	 .enable(1),
	 .flush(0),
	 .DataInput({EXMEM_MemtoReg_wire, EXMEM_RegWrite_wire,RAMReadData_wire,
		EXMEM_ALUResult_wire, EXMEM_WriteRegister_wire, EXMEM_Jal_wire, EXMEM_PC_4_wire }),
	
	.DataOutput({MEMWB_MemtoReg_wire, MEMWB_RegWrite_wire,MEMWB_RAMReadData_wire,
		MEMWB_ALUResult_wire, MEMWB_WriteRegister_wire, MEMWB_Jal_wire, MEMWB_PC_4_wire })
);

//**************************WB******************************/

Multiplexer2to1
#(
	.NBits(DATA_WIDTH)
)
MUX_ForWriteDataToFR
(
	.Selector(MEMWB_MemtoReg_wire),
	.MUX_Data0(MEMWB_ALUResult_wire),
	.MUX_Data1(MEMWB_RAMReadData_wire),
	
	.MUX_Output(MUX_RFWrite_wire)

);

assign ALUResultOut = alu_result_wire;

endmodule

