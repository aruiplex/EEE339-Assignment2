// MIPS single Cycle processor originaly developed for simulation by Patterson and Hennesy
// Modified for synthesis using the QuartusII package by Dr. S. Ami-Nejad. Feb. 2009 

// Register File
module RegisterFile (Read1,Read2,Writereg,WriteData,RegWrite,Data1,Data2,X,Y,Z,T,clock,reset);

	input 	[4:0] Read1,Read2,Writereg; // the registers numbers to read or write
	input 	[31:0] WriteData; 			// data to write
	input 	RegWrite; 					// The write control
	input 	clock, reset; 				// The clock to trigger writes
	output 	[31:0] Data1, Data2; 		// the register values read;
	output 	[31:0] X,Y,Z,T;
	reg 	[31:0] RF[31:0]; 			// 32 registers each 32 bits long
	integer	k;
	
	// Read from registers independent of clock	
	assign 	Data1 = RF[Read1];
	assign 	Data2 = RF[Read2]; 
	assign	X = RF[7];
	assign	Y = RF[1];
	assign	Z = RF[2];
	assign	T = RF[3];
	// write the register with new value on the falling edge of the clock if RegWrite is high
	always @(posedge clock or posedge reset)
		if (reset) for(k=0;k<32;k=k+1) RF[k]<=32'h00000000;
		// Register 0 is a read only register with the content of 0
		else	if (RegWrite & (Writereg!=0)) RF[Writereg] <= WriteData;
endmodule

//ALU Control 
module ALUControl (ALUOp, FuncCode, ALUCtl);

	input 	[1:0] 	ALUOp;
	input 	[5:0] 	FuncCode;
	output	[3:0]	ALUCtl;
	reg		[3:0]	ALUCtl;
	
	always@( ALUOp, FuncCode)
	begin
	case(ALUOp)
	2'b00:	ALUCtl = 4'b0010;
	2'b01:	ALUCtl = 4'b0110;
	2'b10:	case(FuncCode)
				6'b 100000: ALUCtl = 4'b0010;
				6'b 100010: ALUCtl = 4'b0110;
				6'b 100100: ALUCtl = 4'b0000;
				6'b 100101: ALUCtl = 4'b0001;
				6'b 101010: ALUCtl = 4'b0111;
				default:	ALUCtl = 4'bxxxx;
			endcase	
	2'b11:	ALUCtl = 4'b0000;
	default:ALUCtl = 4'bxxxx;
	endcase
	end
endmodule

//ALU
module MIPSALU (ALUctl, A, B, ALUOut, Zero);
	input	[3:0] 	ALUctl;
	input	[31:0] 	A,B;
	output	[31:0] 	ALUOut;
	output 	Zero;
	reg		[31:0] ALUOut;
	
	assign Zero = (ALUOut==0); //Zero is true if ALUOut is 0
	always @(ALUctl, A, B) begin //reevaluate if these change
	case (ALUctl)
		0: ALUOut <= A & B;
		1: ALUOut <= A | B;
		2: ALUOut <= A + B;
		6: ALUOut <= A - B;
		7: ALUOut <= A < B ? 1:0;
		// .... Add more ALU operations here
		default: ALUOut <= A; 
		endcase
	end
endmodule

// Data Memory
module DataMemory(Address, DWriteData, MemRead, MemWrite, clock, reset, DReadData);
input 	[31:0] 	Address, DWriteData;
input			MemRead, MemWrite, clock, reset;
output 	[31:0]	DReadData;
reg		[31:0] 	DMem[7:0];	

assign  DReadData = DMem[Address[2:0]];
always @(posedge clock or posedge reset)begin
		if (reset) begin
			DMem[0]=32'h00000005;
			DMem[1]=32'h0000000A;
			DMem[2]=32'h00000055;
			DMem[3]=32'h000000AA;
			DMem[4]=32'h00005555;
			DMem[5]=32'h00008888;
			DMem[6]=32'h00550000;
			DMem[7]=32'h00004444;
			end else
			if (MemWrite) DMem[Address[2:0]] <= DWriteData;
		end
endmodule

// Main Controller
module Control (opcode,RegDst,Branch,MemRead,MemtoReg,ALUOp,MemWrite,ALUSrc,RegWrite,Jump);

input 	[5:0] 	opcode;
output	[1:0] 	ALUOp;
output	RegDst,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,Jump;
reg		[1:0]	ALUOp;
reg 	RegDst,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,Jump;

parameter R_Format = 6'b000000, LW = 6'b100011, SW = 6'b101011, BEQ=6'b000100, J=6'b000010, ANDI=6'b001100;
always @(opcode)begin
	case(opcode)
		R_Format:{RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b 1001000100;
		LW: 	 {RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b 0111100000;
		SW: 	 {RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b x1x0010000;
		BEQ:	 {RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b x0x0001010;
		J:		 {RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b xxxxxxxxx1;
		ANDI:	 {RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b 0101000110; 
		// .... Add more instructions here
		default: {RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b xxxxxxxxxx;
		endcase
	end
endmodule 

// Datapath
module DataPath(RegDst, Branch, MemRead, MemtoReg, ALUOp, MemWrite,
ALUSrc, RegWrite, Jump, clock, reset, opcode, X,Y,Z,T, ALUResultOut, DReadData,Instruction,PC);

input 	RegDst,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,Jump,clock,reset;
input 	[1:0] 	ALUOp;
output 	[5:0] 	opcode;
output	[31:0]	PC, X,Y,Z,T, ALUResultOut ,DReadData, Instruction;

reg 	[31:0] PC, InstructionMemory[0:31];
wire 	[31:0] SignExtendOffset, PCOffset, PCValue, ALUResultOut,
		IAddress, DAddress, IMemOut, DmemOut, DWriteData, Instruction,
		RWriteData, DReadData, ALUAin, ALUBin, JAddress;
wire 	[3:0] ALUctl;
wire 	Zero;
wire 	[4:0] WriteReg;

//Instruction fields, to improve code readability
wire [5:0] 	funct;
wire [4:0] 	rs, rt, rd, shamt;
wire [15:0] offset;

assign	JAddress={PC[31:28],(Instruction[25:0] << 2)};

//Instantiate local ALU controller
ALUControl alucontroller(ALUOp,funct,ALUctl);

// Instantiate ALU
MIPSALU ALU(ALUctl, ALUAin, ALUBin, ALUResultOut, Zero);

// Instantiate Register File
RegisterFile REG(rs, rt, WriteReg, RWriteData, RegWrite, ALUAin, DWriteData, X,Y,Z,T, clock,reset);

// Instantiate Data Memory
DataMemory datamemory(ALUResultOut, DWriteData, MemRead, MemWrite, clock, reset, DReadData);

// Instantiate Instruction Memory
IMemory IMemory_inst (
	.address ( PC[6:2] ),
	.q ( Instruction )
	);
	  
// Synthesize multiplexers
assign 	WriteReg	= (RegDst)			? rd 				: rt;
assign	ALUBin		= (ALUSrc) 			? SignExtendOffset 	: DWriteData;
assign	PCValue		= (Branch & Zero)	? PC+4+PCOffset 	: ((Jump)		? JAddress	: PC+4);
assign	RWriteData 	= (MemtoReg)		? DReadData			: ALUResultOut;	

// Acquire the fields of the R_Format Instruction for clarity	
assign {opcode, rs, rt, rd, shamt, funct} = Instruction;
// Acquire the immediate field of the I_Format instructions
assign offset = Instruction[15:0];
//sign-extend lower 16 bits
assign SignExtendOffset = { {16{offset[15]}} , offset[15:0]};
// Multiply by 4 the PC offset
assign PCOffset = SignExtendOffset << 2;
// Write the address of the next instruction into the program counter 
always @(posedge clock ) begin
if (reset) PC<=32'h00000000; else
	PC <= PCValue;
end
endmodule

module MIPS1CYCLE(clock, reset,opcode, ALUResultOut ,DReadData, X,Y,Z,T,Instruction,PC);
	input 	clock, 	reset;
	output	[5:0] 	opcode;
	output	[31:0]	ALUResultOut ,DReadData; // For simulation purposes
	output	[31:0]	PC,X,Y,Z,T;
	output	[31:0]	Instruction;
	
	wire [1:0] ALUOp;
	wire [5:0] opcode;
	wire [31:0] SignExtend,ALUResultOut ,DReadData,Instruction;
	wire RegDst,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite;

	// Instantiate the Datapath
	DataPath MIPSDP (RegDst,Branch,MemRead,MemtoReg,ALUOp,
	MemWrite,ALUSrc,RegWrite,Jump,clock, reset, opcode, X,Y,Z,T, ALUResultOut ,DReadData,Instruction,PC);

	//Instantiate the combinational control unit
	Control MIPSControl (opcode,RegDst,Branch,MemRead,MemtoReg,ALUOp,MemWrite,ALUSrc,RegWrite,Jump);
endmodule
