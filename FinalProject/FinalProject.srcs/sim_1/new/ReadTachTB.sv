`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10.07.2022 17:03:21
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


module FinalProjectwrapperTB;

logic rst;
logic clk; 
logic TACH;
logic MISO;
wire SPIclk;
wire CS;
wire PWM;
wire [15:0] led;
wire [3:0] D1_AN;     // 7 Segment Anodes for 4 digits
wire [7:0] D1_SEG;     // 7 Segement Cathodes for the symbols of each digit    


// Initate Unit Under Test
FinalProjectwrapper uut(  .Tach(TACH),
                          .rst(rst),
                          .clk(clk),
                          .MISO(MISO),
                          .SPIclk(SPIclk),
                          .CS(CS),
                          .PWM(PWM),
                          .led(led),
                          .D1_AN(D1_AN),
                          .D1_SEG(D1_SEG));
        
// Create TACH and Clock Signal
always begin
    #5 clk <= ~clk;
end

always begin
    #500000 TACH <= ~TACH;          // The Tach Signal is way slower in reality (faster here, for a better overview)
end 

// Simulate MISO Line
always @ (posedge(SPIclk)) begin 
    if (CS == 0) MISO <= ~MISO;       // Simply return an alternating MISO Signal to Simulate Sensor output   
    else MISO <= 0;
end
 
initial begin
    
    clk <= 0;
    TACH <= 0;
    MISO <= 0;
    rst <= 1;    // Wait for global reset
    #50 rst <= 0;

end    
endmodule
