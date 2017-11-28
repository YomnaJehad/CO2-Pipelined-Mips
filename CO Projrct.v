module fetch(IFIDIR,pc,CLK);
output [31:0] IFIDIR;
input [6:0] pc;
input CLK;
reg[7:0] instructionMemory [127:0];

initial
begin
instructionMemory[0]=8'b00110011;
instructionMemory[1]=8'b00110011;
instructionMemory[2]=8'b00110011;
instructionMemory[3]=8'b00110011;
end
 
always@(posedge CLK);
begin
	assign IFIDIR[7:0] = instructionMemory[pc];
	assign IFIDIR[15:8] = instructionMemory[pc+1];
	assign IFIDIR[23:16] = instructionMemory[pc+2];
	assign IFIDIR[31:24] = instructionMemory[pc+3];
end
endmodule

module board;

reg CLK;
reg [6:0] pc;
wire [31:0] IFIDIR;

fetch f1 (IFIDIR,pc,CLK);

initial
begin
CLK = 0;
pc = 0;
#8 pc=4;
end

//always
//begin
//#5 CLK = ~CLK;
//end

endmodule
