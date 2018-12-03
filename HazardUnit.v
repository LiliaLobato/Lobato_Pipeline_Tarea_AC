module HazardUnit(
	input IDEX_MemRead,
	input jal,
	input jump,
	input branch,
	input jr,
	input [4:0] IDEX_Rt,
	input [4:0] IFID_Rs,
	input [4:0] IFID_Rt,
	
	output reg IFID_write,
	output reg stall_sel,
	output flush
);




always @ (*)
begin


		//ForwardA
		if (IDEX_MemRead && ((IDEX_Rt == IFID_Rs) || (IDEX_Rt == IFID_Rt)))
			begin
			stall_sel <= 1'b1;
			IFID_write <= 1'b0; 

			end
		else 
			begin
			stall_sel <= 1'b0;
			IFID_write <= 1'b1; 
			end
	
end

assign flush = (jump || jal || branch || jr);

endmodule 

