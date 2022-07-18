`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10.07.2022 17:13:53
// Design Name: 
// Module Name: readTACH
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


module readTACH(
    input Tach, clk, rst,
    output logic [15:0] led,
    output logic [15:0] RPM
);

logic [15:0] RPMcounter;
logic [28:0] OneHzclkCounter;
logic [1:0] tachhistory;

logic tach1=1'b0, tach2=1'b0;
always @(posedge clk) begin
  tach1 <= Tach;
  tach2 <= tach1;
end
wire ntach = tach2;

always @ (posedge(clk), posedge(rst)) begin
    led[15] <= 0;
    tachhistory[1:0] <= {tachhistory[0], Tach}; 
    if (rst) begin 
                RPMcounter <= 0;
                OneHzclkCounter <= 0;
             end 
    else if (clk) begin 
        if (OneHzclkCounter > 100) RPMcounter <= 0;     // Reset RPMCounter
        if (OneHzclkCounter > 100000000)                // Reset One Hertz Counter
        begin 
            RPM <= (RPMcounter * 60);// >> 1;   // Divide by 1
            OneHzclkCounter <= 0;           
        end
        else OneHzclkCounter <= OneHzclkCounter + 1;     
        led[4:0] <= {tachhistory[1:0],Tach, Tach}; // debuging 
        if (tachhistory[1:0] == 2'b01) begin             // If Rising edge is detected  increment RPM Counter
            RPMcounter <= RPMcounter + 1;
            led[15] <= ~led[15];  
        end
    end
end 

endmodule 