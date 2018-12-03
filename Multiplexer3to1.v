module Multiplexer3to1
#(
	parameter NBits=32
)
(
	input [1:0] Selector,
	input [NBits-1:0] MUX_Data0,
	input [NBits-1:0] MUX_Data1,
	input [NBits-1:0] MUX_Data2,
	
	output reg [NBits-1:0] MUX_Output

);

	always@(Selector,MUX_Data1,MUX_Data0,MUX_Data2) begin
		case (Selector)
			0: MUX_Output = MUX_Data0;
			1: MUX_Output = MUX_Data1;
			2: MUX_Output = MUX_Data2;
			
			default: MUX_Output =MUX_Data0; 
		endcase

			
	end

endmodule
