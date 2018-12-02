
module ForwardUnit(
	input EXMEM_RegWrite,
	input MEMWB_RegWrite,
	input [4:0] IDEX_Rt,
	input [4:0] IDEX_Rs,
	input [4:0] MEMWB_Rd,
	input [4:0] EXMEM_Rd, 
	output reg [1:0] FwdA,
	output reg [1:0] FwdB
);

	reg [1:0] Fwd_A=0;
	reg [1:0] Fwd_B=0;


always @ (*)
begin


		//ForwardA
		if (EXMEM_RegWrite && (EXMEM_Rd != 0) && (EXMEM_Rd == IDEX_Rs))
			FwdA <= 2'b10;
		
		else	if (MEMWB_RegWrite && (MEMWB_Rd != 0) && (EXMEM_Rd != IDEX_Rs) && (MEMWB_Rd == IDEX_Rs))
			FwdA <= 2'b01;
		
		else 
			FwdA <= 2'b00;		
			
	
			//ForwardB
		if (MEMWB_RegWrite && (MEMWB_Rd != 0) && (EXMEM_Rd != IDEX_Rt) && (MEMWB_Rd == IDEX_Rt))
			FwdB <= 2'b01;
			
		else if (EXMEM_RegWrite && (EXMEM_Rd != 0) && (EXMEM_Rd == IDEX_Rt))
			FwdB <= 2'b10;
			
		else 
			FwdB <= 2'b00;


end

//

endmodule 