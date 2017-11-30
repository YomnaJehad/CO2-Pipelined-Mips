

module controlUnit (OP , RegDest , Branch , MemRead , MemtoReg , ALUOP , MemWrite , ALUSrc , RegWrite );

input [5:0]OP; //instruction[26:31]
output RegDest , Branch , MemRead, MemtoReg , MemWrite, ALUSrc, RegWrite; // 8 Control outputs
output [1:0] ALUOP;

reg RegDest , Branch , MemRead, MemtoReg , MemWrite, ALUSrc, RegWrite; // 8 Control outputs
reg [1:0] ALUOP;


parameter [5:0] R = 0,
		sw = 43,
		lw  = 35,
		beq = 4;


always @(OP)
	begin
		case(OP)
			R: begin

				RegDest<=1;
				Branch <=0;
				MemRead <=0;
				MemtoReg <=0;
				MemWrite <=0;
				ALUSrc <=0;
				RegWrite <=1;
				ALUOP <=2'b 1x;
			end
			sw: begin
				RegDest<=1'b x;
				Branch <=0;
				MemRead <=0;
				MemtoReg <=1'b x;
				MemWrite <=1;
				ALUSrc <=1;
				RegWrite <=0;
				ALUOP <=2'b 00;

			end
		
			lw: begin
				RegDest<=0;
				Branch <=0;
				MemRead <=1;
				MemtoReg <=1;
				MemWrite <=0;
				ALUSrc <=1;
				RegWrite <=1;
				ALUOP <=2'b 00;

			end

			beq: begin
				RegDest<=1'b x;
				Branch <=1;
				MemRead <=0;
				MemtoReg <=1'b x;
				MemWrite <=0;
				ALUSrc <=0;
				RegWrite <=0;
				ALUOP <=2'b 01;

			end
			


		endcase 
	end



endmodule
