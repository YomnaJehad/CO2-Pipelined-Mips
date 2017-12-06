

/*Let's begin with the Fetch stage, /*i'll assume i'm getting lines of Binary Instructions from the GUI and that these
instructions are reserved in the Instruction memory*/
//Fetch Stage


module hazardUnit(IFIDIRrs,IFIDIRrt, IDEXIRrt,IDEXIR_MemRead, stall);
input [4:0] IFIDIRrs,IFIDIRrt, IDEXIRrt;
input IDEXIR_MemRead;
output reg stall;
//IDEXIR[20:16],IFIDIR[20:16],IFIDIR[25:21],);
	always @ (IFIDIRrs,IFIDIRrt, IDEXIRrt)
	begin
		if(IDEXIR_MemRead &&((IDEXIRrt == IFIDIRrt)||(IDEXIRrt==IFIDIRrs)))

		begin	
			stall <= 1;
			
		end
		else
		begin
			stall <= 0;
		end
	end

endmodule


module forwardUnit(ID_EX_rs,ID_EX_rt,EX_MEM_rd,MEM_WB_rd,EX_MEM_RegWrite,MEM_WB_RegWrite,branch,forwardA,forwardB);
input [4:0] ID_EX_rs,ID_EX_rt,EX_MEM_rd,MEM_WB_rd;
input EX_MEM_RegWrite,MEM_WB_RegWrite;
output reg [1:0] forwardA,forwardB;
input branch;
always @(ID_EX_rs or ID_EX_rt or EX_MEM_rd or MEM_WB_rd or EX_MEM_RegWrite or MEM_WB_RegWrite or branch)
begin


//execution forwarding
if ((EX_MEM_RegWrite||branch) && (EX_MEM_rd == ID_EX_rs)) 
  	forwardA <= 2'b 10 ;
//memory forwarding

else if ((MEM_WB_RegWrite||branch) && (EX_MEM_rd != ID_EX_rs) && (MEM_WB_rd == ID_EX_rs))   
  	forwardA <= 2'b 01 ; 

else 
	forwardA <= 2'b 00 ;



//execution forwarding
if ((EX_MEM_RegWrite||branch) && (EX_MEM_rd == ID_EX_rt))
begin  
  	forwardB <= 2'b 10 ;
end

//memory forwarding

else if ((MEM_WB_RegWrite||branch) && (EX_MEM_rd != ID_EX_rt) && (MEM_WB_rd == ID_EX_rt))
  	forwardB <= 2'b 01;
else 
	forwardB <= 2'b 00;
	
end

endmodule

module Data_Memory(CLK, IE_MEM_ALUresult, IE_MEM_rt, MemWrite, MemRead, MEM_WB_ReadData);

reg [7:0] DataMem [127:0];	

input CLK;
input [31:0] IE_MEM_ALUresult;
input MemWrite;
input MemRead;
input [31:0] IE_MEM_rt;
output reg [31:0] MEM_WB_ReadData;

initial
begin

DataMem [41] = 00;
DataMem [42] = 00;
DataMem [43] = 00;
DataMem [44] = 20;

end

always@(posedge CLK)
begin

//#10 $monitor ("Memory 1 = %d %d %d %d",DataMem [25],DataMem [26],DataMem [27],DataMem [28]);

if(MemRead)
begin

 MEM_WB_ReadData [31:24] <= DataMem [IE_MEM_ALUresult + 3];
 MEM_WB_ReadData [23:16] <= DataMem [IE_MEM_ALUresult + 2];
 MEM_WB_ReadData [15:8]  <= DataMem [IE_MEM_ALUresult + 1];
 MEM_WB_ReadData [7:0]   <= DataMem [IE_MEM_ALUresult + 0];

end

else if (MemWrite)
begin

DataMem [IE_MEM_ALUresult + 3] <= IE_MEM_rt [31:24];
DataMem [IE_MEM_ALUresult + 2] <= IE_MEM_rt [23:16];
DataMem [IE_MEM_ALUresult + 1] <= IE_MEM_rt [15:8];
DataMem [IE_MEM_ALUresult + 0] <= IE_MEM_rt [7:0];

end



end
endmodule


module fetch(IFIDIR,CLK, ALUresult, pc, Branch, zeroFlag, tmp_Branch, no_op, stall);
output wire [31:0] IFIDIR;
input CLK;
reg[7:0] instructionMemory [127:0];

input Branch, zeroFlag;
output reg [31:0] pc;
wire Sel;
assign Sel = Branch & zeroFlag;
input wire [31:0]ALUresult;
output reg no_op;
input wire tmp_Branch, stall;

always@(Sel)
begin
if(Sel)
begin
pc = ALUresult;
end
end

always@ (stall)
begin
if(stall)
begin
pc = pc - 4;
end
end


always@(Branch)
begin
if(Branch)
begin
no_op = 0;
end
end

initial
begin
pc = 0;
no_op = 0;
$readmemb("test.txt",instructionMemory);
end
	assign IFIDIR[7:0]   = instructionMemory [pc+3];
	assign IFIDIR[15:8]  = instructionMemory [pc+2];
	assign IFIDIR[23:16] = instructionMemory [pc+1];
	assign IFIDIR[31:24] = instructionMemory [pc+0];
always@(tmp_Branch)
begin
if(tmp_Branch)
begin
no_op = 1;
end
end
always@(posedge CLK)
begin
	if(Sel)
	begin
	pc <= pc+4;
	end
	else
	begin
	if(~no_op)
	begin
	pc <= pc+4;
	end
	end
end

endmodule


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
				ALUOP <=2'b 10;
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

module regfile (clk, WE, RR1, RR2, WR, mux, result, WD, RD1, RD2);

 input clk;
input WE, mux;
 
input signed [31:0] result,WD;
input [4:0] RR1,RR2, WR;
output reg  signed [31:0] RD1, RD2;
reg signed [31:0] registers1 [0:31];

initial
begin

registers1 [16] <= 0;
registers1 [17] <= 1;
registers1 [18] <= 2;
registers1 [19] <= 3;
registers1 [20] <= 4;
registers1 [21] <= 5;
registers1 [22] <= 6;
registers1 [23] <= 7;

#5000 
#10 $monitor ("$s0 : %d , $s1 : %d, $s2 : %d RunMonitor",registers1 [16],registers1 [17],registers1 [18]);
#10 $monitor ("$s3 : %d , $s4 : %d, $s5 : %d RunMonitor",registers1 [19],registers1 [20],registers1 [21]);
#10 $monitor ("$s6 : %d , $s7 : %d RunMonitor",registers1 [22], registers1 [23]);

end
always @ (RR1, RR2)
	begin
		RD1 <= registers1[RR1];
		RD2 <= registers1[RR2];
	end

always @(posedge clk)
begin
if (WE)
  
  registers1[WR] <= (mux)? WD : result ;  
 
end
 
endmodule


module writeRegMux(in1 ,in2, RegDst, out);

output [4:0]out;
input [4:0] in1,in2;
input RegDst;
assign out=(RegDst)?in1:in2;

endmodule

module signExtend(in,out);
input signed [15:0] in;
output signed [31:0] out;

assign out = {{16{in[15]}},{in[15:0]}};
endmodule

module ALUSrcMux(in1,in2,ALUSrc, out);
input [31:0] in1,in2;
input ALUSrc;
output [31:0] out;
assign out= (ALUSrc)?in1:in2;

endmodule


module ALUControl (ALUOP, func, ALUCtrl);
input [1:0] ALUOP;
input [5:0] func;
output reg [2:0] ALUCtrl;
//ALUControl output should be like this 
parameter [2:0] andd=3'b 000,
		orr =3'b 001, 
		add =3'b 010,
		sub =3'b 110,
		slt =3'b 111;
always @(func or ALUOP)
begin
case(ALUOP)
	2'b 00: ALUCtrl <= 3'b 010;
	2'b 01: ALUCtrl <= 3'b 110;
	2'b 10: 
	begin
		case (func)
			32: ALUCtrl <= 3'b 010;
			34: ALUCtrl <= 3'b 110;
			36: ALUCtrl <= 3'b 000;
			37: ALUCtrl <= 3'b 001;
			42: ALUCtrl <= 3'b 111;
			0:  ALUCtrl <= 3'b 100;

		endcase
	end

endcase 

end

endmodule


//beq whole uppercrap 

module ALU(d1,d2,op,result,shift,zeroFlag);
 input signed [31:0] d1,d2;
 input [2:0] op;
 input [4:0]shift;
 output  reg signed [31:0]result;
 output reg zeroFlag;
 always @ (d1, d2, op)
    begin
      
      case(op)
        
        3'b 010 : result <= d1+d2;
        3'b 110 :
	begin

	result   = d1-d2;
	zeroFlag <=  (result)?0:1;	
	
	end 

        3'b 000 : result <= d1 & d2;
        3'b 001 : result <= d1 | d2;
        3'b 100 : result <= d2 << shift;   

/*        4'b 0101: result <= d1 >> shift;       
        4'b 0110: result <= $signed(d1 )>>> shift;
        4'b 0111 : result <= (d1>d2)?1:0;       
        4'b 1000 : result <= (d1<d2)?1:0;
*/  
      endcase
 end
endmodule

module forwardingUnitSuperMux(d1_IDIEIR, d2_IDIEIR, forwardA, forwardB, result, ReadData, d1_to_alu, d2_to_alu);
input [31:0] d1_IDIEIR, d2_IDIEIR, result, ReadData;
input [1:0] forwardA, forwardB;
output reg [31:0] d1_to_alu, d2_to_alu;

always @(forwardA or forwardB or d1_IDIEIR or d2_IDIEIR or result or ReadData) 
begin
	if(forwardA == 2)
	begin
	d1_to_alu = result;		
	end
	else if(forwardA == 1)
	begin
	d1_to_alu = ReadData;
	end
	else 
	begin
	d1_to_alu = d1_IDIEIR;
	end
	if(forwardB == 2)
	begin
	d2_to_alu = result;		
	end
	else if(forwardB == 1)
	begin
	d2_to_alu = ReadData;
	end
	else 
	begin
	d2_to_alu = d2_IDIEIR;
	end
end

endmodule

module testbench();

reg clk;

//reg [31:0] instruction;
wire [31:0] IFIDIR;
wire [31:0] result;
wire RegDest , Branch , MemRead , MemtoReg , MemWrite , ALUSrc , RegWrite;
wire [1:0] ALUOP;
wire [2:0] ALUCtrl;
wire [31:0] d1,d2;
reg [4:0] shift;
wire [4:0] out;
wire [31:0] SE_out;
wire [31:0] ReadData;
wire [31:0] M2_out;
wire zeroFlag;
reg [31:0] bridge;
wire [31:0] pc;
wire [31:0] ALUresult;
wire no_op;
initial 
begin
clk       <= 0;
bridge    <= 0;
//#10 finalPC    <= 0;
//#10 instruction = 32'b 000000_00000_00001_00010_00000_100000;
//shift <= 0;
//#200 $monitor(" %d",d1);

end

reg [31:0] IFIDIR_IFIDIR;
reg [31:0] pc_IFIDIR;

reg [31:0] d1_IDIEIR;
reg [31:0] d2_IDIEIR;
reg [31:0] SE_out_IDIEIR;
reg [31:0] SE_out_shifted_IDIEIR;
reg RegDest_IDIEIR, Branch_IDIEIR, MemRead_IDIEIR, MemtoReg_IDIEIR, MemWrite_IDIEIR, ALUSrc_IDIEIR;
reg [1:0] ALUOP_IDIEIR;
reg [31:0] IFIDIR_IDIEIR;
reg [4:0] rd_IDIEIR, rt_IDIEIR, rs_IDIEIR;
reg RegWrite_IDIEIR;
reg [31:0] pc_IDIEIR;

reg [31:0] result_IEMEMIR;
reg [31:0] d2_IEMEMIR;
reg MemtoReg_IEMEMIR, MemRead_IEMEMIR, MemWrite_IEMEMIR;
reg [4:0] out_IEMEMIR;
reg RegWrite_IEMEMIR;
reg [4:0] rd_IEMEMIR;
reg [4:0] rt_IEMEMIR;

reg [4:0] rd_IMEMWBIR;
reg [4:0] rt_IMEMWBIR;
reg RegWrite_IMEMWBIR, MemtoReg_IMEMWBIR;
reg [31:0] address_IMEMWBIR;
reg [4:0] out_IMEMWBIR;
assign ALUresult = pc_IDIEIR + (SE_out_IDIEIR<<2);

wire [1:0] forwardA, forwardB; 
wire [31:0] d1_to_alu, d2_to_alu;
wire stall;

/*pcModule    PC (bridge,fetchPC);*/
fetch       G0 (IFIDIR,clk, ALUresult, pc, Branch_IDIEIR, zeroFlag, Branch, no_op, stall);

controlUnit G1 (IFIDIR_IFIDIR [31:26] , RegDest , Branch , MemRead , MemtoReg , ALUOP , MemWrite , ALUSrc , RegWrite);
regfile     G3 (clk, RegWrite_IMEMWBIR, IFIDIR_IFIDIR[25:21], IFIDIR_IFIDIR[20:16], out_IMEMWBIR, MemtoReg_IMEMWBIR, address_IMEMWBIR, ReadData, d1, d2);
signExtend  S1 (IFIDIR_IFIDIR[15:0],SE_out);

ALUControl  G2 (ALUOP_IDIEIR, IFIDIR_IDIEIR [5:0], ALUCtrl);
forwardingUnitSuperMux SM (d1_IDIEIR, d2_IDIEIR, forwardA, forwardB, result_IEMEMIR, ReadData, d1_to_alu, d2_to_alu);
ALUSrcMux   M2 (SE_out_shifted_IDIEIR ,d2_to_alu,ALUSrc_IDIEIR, M2_out);
ALU         G4 (d1_to_alu,M2_out,ALUCtrl,result,IFIDIR_IDIEIR[10:6],zeroFlag);
writeRegMux M1 (rd_IDIEIR ,rt_IDIEIR, RegDest_IDIEIR, out);


Data_Memory G5 (clk, result_IEMEMIR, d2_IEMEMIR, MemWrite_IEMEMIR, MemRead_IEMEMIR, ReadData);


forwardUnit fU0 (rs_IDIEIR, rt_IDIEIR, rd_IEMEMIR, rd_IMEMWBIR, RegWrite_IEMEMIR, RegWrite_IMEMWBIR, branch_IDIEIR, forwardA, forwardB);
hazardUnit  HU0 (IFIDIR_IFIDIR [25:21], IFIDIR_IFIDIR [20:16], rt_IDIEIR, MemRead_IDIEIR, stall);

always@(negedge clk)
begin

if(stall == 1)
begin

SE_out_IDIEIR <=0;
SE_out_shifted_IDIEIR <=0;
d1_IDIEIR <=0;
d2_IDIEIR <=0;
RegDest_IDIEIR <=0;
Branch_IDIEIR <=0;
MemRead_IDIEIR <=0;
MemtoReg_IDIEIR <=0;
ALUOP_IDIEIR <=0;
MemWrite_IDIEIR <=0;
ALUSrc_IDIEIR <=0;
RegWrite_IDIEIR <=0;
IFIDIR_IDIEIR <=0;
rd_IDIEIR <=0;
rt_IDIEIR <=0;
rs_IDIEIR <=0;
RegWrite_IDIEIR <=0;
pc_IDIEIR <=0;

end


end


always@(posedge clk)
begin
if(no_op)
begin
IFIDIR_IFIDIR <= 0;
end
else
begin
IFIDIR_IFIDIR <= IFIDIR;
end
pc_IFIDIR <= pc+4;

SE_out_IDIEIR <= SE_out;
SE_out_shifted_IDIEIR <= (SE_out<<2);
d1_IDIEIR <= d1;
d2_IDIEIR <= d2;
RegDest_IDIEIR <= RegDest; 
Branch_IDIEIR <= Branch;
MemRead_IDIEIR <= MemRead;
MemtoReg_IDIEIR <= MemtoReg; 
ALUOP_IDIEIR <= ALUOP;
MemWrite_IDIEIR <= MemWrite;
ALUSrc_IDIEIR <= ALUSrc; 
RegWrite_IDIEIR <= RegWrite;
IFIDIR_IDIEIR <= IFIDIR_IFIDIR;
rd_IDIEIR <= IFIDIR_IFIDIR [15:11];
rt_IDIEIR <= IFIDIR_IFIDIR [20:16];
rs_IDIEIR <= IFIDIR_IFIDIR [25:21];
RegWrite_IDIEIR <= RegWrite;
pc_IDIEIR <= pc_IFIDIR;

result_IEMEMIR <= result;
d2_IEMEMIR <= d2_IDIEIR;
MemtoReg_IEMEMIR <= MemtoReg_IDIEIR; 
MemRead_IEMEMIR <= MemRead_IDIEIR;
MemWrite_IEMEMIR <= MemWrite_IDIEIR;
out_IEMEMIR <= out;
RegWrite_IEMEMIR <= RegWrite_IDIEIR;
rd_IEMEMIR <= rd_IDIEIR;
rt_IEMEMIR <= rt_IDIEIR;

rd_IMEMWBIR <= rd_IEMEMIR;
rt_IMEMWBIR <= rt_IEMEMIR;
RegWrite_IMEMWBIR <= RegWrite_IEMEMIR;
out_IMEMWBIR <= out_IEMEMIR;
address_IMEMWBIR <= result_IEMEMIR;
MemtoReg_IMEMWBIR <= MemtoReg_IEMEMIR;


end

always 
begin

#50 clk = ~clk;

end
endmodule

	

