module Data_Memory(CLK, IE_MEM_ALUresult, IE_MEM_rt, MemWrite, MemRead, MEM_WB_ReadData);

reg [7:0] DataMem [127:0];

input CLK;
input [31:0] IE_MEM_ALUresult;
input MemWrite;
input MemRead;
input [31:0] IE_MEM_rt;
output reg [31:0] MEM_WB_ReadData;


always@(posedge CLK)
begin

if(MemRead)
begin

 MEM_WB_ReadData [31:24] <= DataMem [IE_MEM_ALUresult];
 MEM_WB_ReadData [23:16] <= DataMem [IE_MEM_ALUresult + 1];
 MEM_WB_ReadData [15:8]  <= DataMem [IE_MEM_ALUresult + 2];
 MEM_WB_ReadData [7:0]   <= DataMem [IE_MEM_ALUresult + 3];

end

else if (MemWrite)
begin

DataMem [IE_MEM_ALUresult] <= IE_MEM_rt [31:24];
DataMem [IE_MEM_ALUresult + 1] <= IE_MEM_rt [23:16];
DataMem [IE_MEM_ALUresult + 2] <= IE_MEM_rt [15:8];
DataMem [IE_MEM_ALUresult + 3] <= IE_MEM_rt [7:0];

end

end

endmodule
module MEM_TB ();



reg CLK;
reg MemWrite;
reg MemRead;
reg [31:0] IE_MEM_rt;
wire [31:0] MEM_WB_ReadData;
reg [31:0] IE_MEM_ALUresult;

Data_Memory M(CLK, IE_MEM_ALUresult, IE_MEM_rt, MemWrite, MemRead, MEM_WB_ReadData);


initial
begin

CLK = 0;
MemRead  = 0;
MemWrite = 1;
IE_MEM_rt = 20;
IE_MEM_ALUresult = 0;
#100
MemRead  = 1;
MemWrite = 0;
IE_MEM_ALUresult = 0;

end

always
begin

#10 CLK = ~CLK;

end


endmodule






















