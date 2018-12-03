/******************************************************************
* Description
*	This is the data memory for the MIPS processor
*	1.0
* Author:
*	Dr. José Luis Pizano Escalante
* email:
*	luispizano@iteso.mx
* Date:
*	01/03/2014
******************************************************************/

module DataMemory 
#(	parameter DATA_WIDTH=8,
	parameter MEMORY_DEPTH = 256

)
(
	input [DATA_WIDTH-1:0] WriteData,
	input [DATA_WIDTH-1:0]  Address,
	input MemWrite,MemRead, clk,
	output  [DATA_WIDTH-1:0]  ReadData
);

	//Real Address variable
	wire [9:0] realAddress;
	assign realAddress = (Address - 268500992)/4;
	
	// Declare the RAM variable
	reg [DATA_WIDTH-1:0] ram[MEMORY_DEPTH-1:0];
	wire [DATA_WIDTH-1:0] ReadDataAux;

	always @ (posedge clk)
	begin
		// Write
		if (MemWrite)
			ram[realAddress] <= WriteData;
	end
	assign ReadDataAux = ram[realAddress];
  	assign ReadData = {DATA_WIDTH{MemRead}}& ReadDataAux;

endmodule
