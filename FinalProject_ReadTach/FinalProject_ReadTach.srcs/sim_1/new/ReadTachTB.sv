`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10.07.2022 17:15:40
// Design Name: 
// Module Name: ReadTachTB
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module ReadTachTB;

logic rst;
logic clk; 
logic TACH;
wire [15:0] RPM;
wire [15:0] led;

// Initate Unit Under Test
readTACH uut(.Tach(TACH),.rst(rst),.clk(clk),.RPM(RPM),.led(led));

// Create TACH and Clock Signal
always 
    #10 clk = ~clk;
always
    #100 TACH = ~TACH;



initial begin
    
    clk = 0;
    TACH = 0;
    rst = 1;
    // Wait for global reset
    #50 rst = 0;
    
/*    #2500 in_raw = 1; 
    #500 in_raw = 0;
    #500 in_raw = 1;        
    #400 in_raw = 0;
    #500 in_raw = 1;  
    #500 in_raw = 0;
    #500 in_raw = 1;  
*/
end    
endmodule
