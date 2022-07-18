`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.06.2022 11:49:09
// Design Name: 
// Module Name: FinalProject
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

module FinalProjectwrapper(
    input clk,              // 100MHz general clock from Boolean Board
    input rst,              // btn[0] for general reset
    input MISO,             // Master In Slave Out (SPI BUS)
    input Tach,             // Tachometer Pin of Fan (goes high two times for every rotation)
    output logic SPIclk,    // PModA -> See Constraints or documentation    
    output CS,              // ChipSelect (SPI BUS)
    output [15:0] led,      // Debug LED
    output PWM,             // Fan Control Signal in ~25kHz PWM form
    output [3:0] D1_AN,     // 7 Segment Anodes for 4 digits
    output [7:0] D1_SEG     // 7 Segement Cathodes for the symbols of each digit     
);

// Signals to connnect the different Modules 
logic [11:0] ADCraw;
logic [24:0] Voltage;
logic [24:0] Temp;
logic [7:0] DutyCycle;
logic [15:0] RPM;

// Initiate and connect the different modules to one another.

SPIMCP3201 SPI_Communication(.clk(clk),.rst(rst),.MISO(MISO),.SPIclk(SPIclk),.CS(CS),.ADCrawout(ADCraw),.led(led[11:0]));

ADCtoVoltage A2V(.ADCraw(ADCraw),.clk(clk),.Voltage(Voltage));

VoltageToTemperature V2T(.Voltage(Voltage),.Temperature(Temp),.clk(clk));

TemperatureToDutyCycle T2D(.Temperature(Temp),.clk(clk),.DutyCycle(DutyCycle));

DutyCycleToPWM D2P(.clk(clk),.rst(rst),.PWM(PWM),.DutyCycle(DutyCycle));

readTACH rT(.clk(clk),.Tach(Tach),.led(led),.RPM(RPM),.rst(rst));

SegDrive SevenSegmentDriver(.bin1({1'b0,1'b0,Temp[11:0]}),.bin2(RPM[13:0]),.clk(clk),.D1_AN(D1_AN),.D1_SEG(D1_SEG),.rst(rst));

endmodule


//////////////////////////////////////////////////////////////
//////////////////       Modules         /////////////////////
//////////////////////////////////////////////////////////////

/////////////  SPI Communication to ADC   /////////////
module SPIMCP3201(
    input clk,              // 100MHz internal Clock
    input rst,              // btn0
    input MISO,             // PModA -> See Constraints or documentation - SPIBUS
    output logic SPIclk,    // PModA -> See Constraints or documentation - SPIBUS   
    output CS,              // PMODA -> See Constraints or documentation - SPIBUS
    output logic [11:0] ADCrawout,       // raw output of the ADC
    output [11:0] led       // LEDs purely for debuging. ADC Data from [11:0]; RPM rising edges on led15
);

logic [0:15] ADCraw; // Raw ADC Data; -> The Data will be written to bits [2:13] 13 = MSB; Array is inverted because data is received LSB first.
    

//  SPIClk 100kHz ClockDivider + Shifted Clock to account for Tsucs (see datasheet) //
logic [28:0] SPIcounter;
logic [28:0] SPIShiftcounter;
logic SPIclkShifted;
    
always @ (posedge(clk), posedge(rst)) begin     
      //  SPIClk 100kHz ClockDivider -> Used as SPIClk
     if (SPIcounter >= 300) SPIclk  <= 0;   // Give the Clock Signal a 50% Duty Cycle to reliably trigger the Slave/ADC
     if (rst) SPIcounter <= 0;
     else if (SPIcounter == (1000-1))begin  // 100MHz/1000 => 100khz  
        SPIcounter <= 0;
        SPIclk <= 1;
     end
     else SPIcounter <= SPIcounter + 1;
     
     // Shifted Clock to account for Tsucs (see datasheet) -> Pretty much Copy Pasted the 100kHz Clockdivider from obove
     if (SPIShiftcounter >= 500) SPIclkShifted  <= 1;       // Turn on after half the clockcycle has passed
     if (SPIShiftcounter >= 800) SPIclkShifted  <= 0;       // Turn of after 80% to achieve 30% DutyCycle. No particular reason for 30%
     if (rst) SPIShiftcounter <= 0;
     else if (SPIShiftcounter == (1000-1))begin             // 100MHz/1000 => 100khz  
        SPIShiftcounter <= 0;
     end
     else SPIShiftcounter <= SPIShiftcounter + 1;     
end

//  Sample ClockDivider( 1Hz ) + SPI MOSI Line Readout //
logic [28:0] Samplecounter;
logic SampleClock;                                                                  // Is also used as Variable for CS  

always @ (posedge(SPIclkShifted) or posedge(rst)) begin                            
     if (rst) Samplecounter <= 0;
     else if (Samplecounter == (100000-1))  begin                                   // 100kHz / 100000 => 1Hz  
                                                Samplecounter <= 0;                 // restart counting 
                                                SampleClock <= 1;                   // turn on cs
                                            end
     else begin 
             Samplecounter <= Samplecounter + 1;
             if (Samplecounter > 13) begin                                          // wait 15 clock cycles before turning cs off
                                        SampleClock  <= 0;                          // turn off CS
                                        ADCrawout[11:0] <= ADCraw[2:13];            // give out the final output data
                                     end
             
             if (Samplecounter < 14) begin                                          // While CS is active turn Read Data from MISO on positive Edges 
                                       ADCraw[Samplecounter] <= MISO;
                                     end    
         end  
end

assign led[11:0] = ADCrawout[11:0];
assign CS = ~SampleClock; 
    
endmodule


/////////////  ADC to Voltage Conversion   /////////////
module ADCtoVoltage(
    input [11:0] ADCraw,
    input clk,
    output logic [24:0] Voltage  
);
logic [24:0] Voltageint;                        // Intermediate Value for Voltage Conversion

always @ (posedge clk) begin 
    Voltageint <= ADCraw * 12'd3330;            // ADC Value is multiplied with the 3V33 to follow the equation: V = ADC*(3330/4096)
    Voltage <= Voltageint>>12;                  // Divide by 4096
end
endmodule 


/////////////  Voltage to Temperature Conversion   /////////////
module VoltageToTemperature(
    input [11:0] Voltage,
    input clk,
    output logic [24:0] Temperature
);

logic [24:0] Temperatureint1;
logic [24:0] Temperatureint2;  

always@ (posedge(clk)) begin
    Temperatureint1 <= Voltage * 659;
    Temperatureint2 <= Temperatureint1 >> 13;        // divide by 8196
    Temperature <= Temperatureint2 - 25;             // 22,84 would be accurate but 23 is good enough
end

endmodule     


/////////////  Temperature to DutyCycle Conversion   /////////////
module TemperatureToDutyCycle(
    input [24:0] Temperature, 
    input clk,
    output logic [7:0] DutyCycle
);

always @ (posedge(clk)) begin 
    /*
    if (Temperature < 29) DutyCycle <= 40;
    else DutyCycle <= 90;
    */
    if (Temperature <= 25) DutyCycle <= 20;
    else if (Temperature >= 35) DutyCycle <= 100;
    else DutyCycle <= ((Temperature - 25) * 8) + 20; 
       
end

endmodule 


/////////////  DutyCycle to PWM Conversion   /////////////
module DutyCycleToPWM(
    input clk, rst,                 
    input [7:0] DutyCycle,      // Input Duty Cycle for Fan Control
    output logic PWM            // Fan Control out
);

logic [11:0] PWMcounter;  // 12 bits 4096. Intentional overflow didnt work so count to 4000 results in exact 25khZ Frequency for the PWM signal 

//  PWM ~25kHz ClockDivider
always @ (posedge(clk), posedge(rst)) begin
     if (PWMcounter >= (40* DutyCycle)) PWM  <= 0;   // Give the Clock Signal the DutyCycle times 40 to convert to correct timescale (100*40=4000) -> set low after DutyCycle time
     if (rst) PWMcounter <= 0;
     else if (PWMcounter > (4000-1)) begin           // Reset at 4000 for 25kHz Frequency
        PWM <= 1;                                    // Start High period for PWM Signal
        PWMcounter <= 0;                             // reset Counter
     end
     else PWMcounter <= PWMcounter + 1;
end
endmodule



/////////////  Temperature to DutyCycle Conversion   /////////////
module readTACH(
    input Tach, clk, rst,           
    output logic [15:0] led,        // debug LED
    output logic [15:0] RPM         // Rotations per Minute of the Fan
);

logic [15:0] RPMcounter;
logic [28:0] OneHzclkCounter;
logic [1:0] tachhistory;


always @ (posedge(clk), posedge(rst)) begin
    tachhistory[1:0] <= {tachhistory[0], Tach}; 
    if (rst) begin 
                RPMcounter <= 0;
                OneHzclkCounter <= 0;
             end 
    else if (clk) begin 
        if (OneHzclkCounter == 100) RPMcounter <= 0;     // Reset RPMCounter
        if (OneHzclkCounter > 100000000) begin           // Reset One Hertz Counter
            RPM[15:0] <= (RPMcounter[15:0] * 30);        // Multiply with 60  to get from Rotations per second to Minute and Divide by 2 because of 2* high per rotation
            OneHzclkCounter <= 0;           
        end
        else OneHzclkCounter <= OneHzclkCounter + 1;     
        if (tachhistory[1:0] == 2'b10) begin             // If Falling edge is detected  increment RPM Counter
            RPMcounter <= RPMcounter + 1'b1;
            led[15] <= ~led[15];  
        end
    end
end 

endmodule 



////// SEVEN SEGMENT DRIVER ///////////
module SegDrive(
    input [13:0] bin1,          // Rotating Value 1  Each of the Values is displayed for 1 second
    input [13:0] bin2,          // Rotating Value 2
    input clk,                  // 100Mhz Input clock
    input rst,
    output [3:0] D1_AN,         
    output [7:0] D1_SEG  
         
);

// 1Hz Clockdivider  (needed to alternate display output between values)
logic [28:0] CLKcounter;
logic OneHzclk;
always @ (posedge clk) 
begin 
    if (CLKcounter >=10) OneHzclk <= 0;   // Leave it on for 10 clock cycles to reliably trigger
    if (rst) CLKcounter <= 0;
    else if (CLKcounter == (100000000-1)) // 100MHz/100000000 => 1Hz  
    begin  
        CLKcounter <= 0;
        OneHzclk <= 1;
    end
    else CLKcounter <= CLKcounter + 1;    
end

// switch between display values
logic dispval;      // selection state
logic [13:0] bin;   // Displayed Value 
always @ (posedge(OneHzclk))
begin 
    dispval = ~dispval;
    if (dispval == 1'b1) bin = bin1;
    else if (dispval == 1'b0) bin = bin2;
end


// 1 kHz ClockDivider  //
reg [28:0] DispCount;
reg DispClk;


always @ (posedge(clk), posedge(rst))
begin
    DispClk <= 0;
    if (rst) DispCount <= 0;
    else if (DispCount == (100000-1))       // 100khz 
    begin  
        DispCount <= 0;
        DispClk <= 1;
    end 
    else DispCount <= DispCount + 1;
end


// BCD Converter - copied from realdigital.org 
reg [15:0] bcd;
reg [3:0] I0,I1,I2,I3;
integer i;
	
always @(bin) begin
    bcd=0;		 	
    for (i=0;i<14;i=i+1) begin					             //Iterate once for each bit in input number
        if (bcd[3:0] >= 5) bcd[3:0] = bcd[3:0] + 3;		     //If any BCD digit is >= 5, add three
        if (bcd[7:4] >= 5) bcd[7:4] = bcd[7:4] + 3;
        if (bcd[11:8] >= 5) bcd[11:8] = bcd[11:8] + 3;
        if (bcd[15:12] >= 5) bcd[15:12] = bcd[15:12] + 3;
        bcd = {bcd[14:0],bin[13-i]};				        //Shift one bit, and shift in proper bit from input 
    end
    I0 = bcd[3:0];
    I1 = bcd[7:4];
    I2 = bcd[11:8];
    I3 = bcd[15:12];
end


// Simple Counter (0-3) used as input for Multiplexer and Decoder
reg [1:0] B;
always @ (posedge (DispClk))begin
    if (B == 0'b11)
        B <= 0;
    else B <= B + 0'b01;
end

// Multiplexer 4to1
reg [1:0] Bnew;
always @ (B, I0, I1, I2, I3) begin
    // Switch input ports
    Bnew[0] = B[1];
    Bnew[1] = B[0];
    case (Bnew) 
        2'b00: A <= I0;
        2'b01: A <= I1;
        2'b10: A <= I2;
        2'b11: A <= I3;
        default: A <= I0;
    endcase
end

// Decoder 4 to 1
reg [3:0] An;
always @ (B) begin
    // First Turn all off (Turned off in High state)
    An[0] <= 1;
    An[1] <= 1;
    An[2] <= 1;
    An[3] <= 1;
    // Then turn new one on depending on counter state
    case (B)
        2'b00: begin 
            An[0] <= 0;
            An[1] <= 1;
            An[2] <= 1;
            An[3] <= 1;
        end
        2'b01: begin 
            An[0] <= 1;
            An[1] <= 1;
            An[2] <= 0;
            An[3] <= 1;
        end
        2'b10: begin 
            An[0] <= 1;
            An[1] <= 0;
            An[2] <= 1;
            An[3] <= 1;
        end
        
        2'b11:begin 
            An[0] <= 1;
            An[1] <= 1;
            An[2] <= 1;
            An[3] <= 0;
        end
    
    default: begin 
            An[0] <= 1;
            An[1] <= 1;
            An[2] <= 1;
            An[3] <= 1;
        end
    endcase
end

// 7 Segment Controller -> Translate input Digits into actuall Numbers on Display
reg [7:0] SEG;
reg [3:0] A;
 
always @(A) begin

    case (A) 
    0'b0000: begin
                    SEG[0] <= 0;
                    SEG[1] <= 0;   
                    SEG[2] <= 0;
                    SEG[3] <= 0;
                    SEG[4] <= 0;
                    SEG[5] <= 0;
                    SEG[6] <= 1;
                    SEG[7] <= 1;
                  end
    0'b0001: begin
                    SEG[0] <= 1;
                    SEG[1] <= 0;   
                    SEG[2] <= 0;
                    SEG[3] <= 1;
                    SEG[4] <= 1;
                    SEG[5] <= 1;
                    SEG[6] <= 1;
                    SEG[7] <= 1;
                  end
    0'b0010: begin
                    SEG[0] <= 0;
                    SEG[1] <= 0;   
                    SEG[2] <= 1;
                    SEG[3] <= 0;
                    SEG[4] <= 0;
                    SEG[5] <= 1;
                    SEG[6] <= 0;
                    SEG[7] <= 1;
                  end
    0'b0011: begin
                    SEG[0] <= 0;
                    SEG[1] <= 0;   
                    SEG[2] <= 0;
                    SEG[3] <= 0;
                    SEG[4] <= 1;
                    SEG[5] <= 1;
                    SEG[6] <= 0;
                    SEG[7] <= 1;
                  end
    0'b0100: begin
                    SEG[0] <= 1;
                    SEG[1] <= 0;   
                    SEG[2] <= 0;
                    SEG[3] <= 1;
                    SEG[4] <= 1;
                    SEG[5] <= 0;
                    SEG[6] <= 0;
                    SEG[7] <= 1;
                  end 
    0'b0101: begin
                    SEG[0] <= 0;
                    SEG[1] <= 1;   
                    SEG[2] <= 0;
                    SEG[3] <= 0;
                    SEG[4] <= 1;
                    SEG[5] <= 0;
                    SEG[6] <= 0;
                    SEG[7] <= 1;
                  end
    0'b0110: begin
                    SEG[0] <= 0;
                    SEG[1] <= 1;   
                    SEG[2] <= 0;
                    SEG[3] <= 0;
                    SEG[4] <= 0;
                    SEG[5] <= 0;
                    SEG[6] <= 0;
                    SEG[7] <= 1;
                  end
    0'b0111: begin
                    SEG[0] <= 0;
                    SEG[1] <= 0;   
                    SEG[2] <= 0;
                    SEG[3] <= 1;
                    SEG[4] <= 1;
                    SEG[5] <= 1;
                    SEG[6] <= 1;
                    SEG[7] <= 1;
                  end
    0'b1000: begin
                    SEG[0] <= 0;
                    SEG[1] <= 0;   
                    SEG[2] <= 0;
                    SEG[3] <= 0;
                    SEG[4] <= 0;
                    SEG[5] <= 0;
                    SEG[6] <= 0;
                    SEG[7] <= 1;
                  end
    0'b1001: begin
                    SEG[0] <= 0;
                    SEG[1] <= 0;   
                    SEG[2] <= 0;
                    SEG[3] <= 0;
                    SEG[4] <= 1;
                    SEG[5] <= 0;
                    SEG[6] <= 0;
                    SEG[7] <= 1;
                  end
    0'b1010: begin
                    SEG[0] <= 0;
                    SEG[1] <= 0;   
                    SEG[2] <= 0;
                    SEG[3] <= 1;
                    SEG[4] <= 0;
                    SEG[5] <= 0;
                    SEG[6] <= 0;
                    SEG[7] <= 1;
                  end
    0'b1011: begin
                    SEG[0] <= 1;
                    SEG[1] <= 1;   
                    SEG[2] <= 0;
                    SEG[3] <= 0;
                    SEG[4] <= 0;
                    SEG[5] <= 0;
                    SEG[6] <= 0;
                    SEG[7] <= 1;
                  end
    0'b1100: begin
                    SEG[0] <= 0;
                    SEG[1] <= 1;   
                    SEG[2] <= 1;
                    SEG[3] <= 0;
                    SEG[4] <= 0;
                    SEG[5] <= 0;
                    SEG[6] <= 1;
                    SEG[7] <= 1;
                  end
    0'b1101: begin
                    SEG[0] <= 1;
                    SEG[1] <= 0;   
                    SEG[2] <= 0;
                    SEG[3] <= 0;
                    SEG[4] <= 0;
                    SEG[5] <= 1;
                    SEG[6] <= 0;
                    SEG[7] <= 1;
                  end
    0'b1110: begin
                    SEG[0] <= 0;
                    SEG[1] <= 1;   
                    SEG[2] <= 1;
                    SEG[3] <= 0;
                    SEG[4] <= 0;
                    SEG[5] <= 0;
                    SEG[6] <= 0;
                    SEG[7] <= 1;
                  end
    0'b1111: begin
                    SEG[0] <= 0;
                    SEG[1] <= 1;   
                    SEG[2] <= 1;
                    SEG[3] <= 1;
                    SEG[4] <= 0;
                    SEG[5] <= 0;
                    SEG[6] <= 0;
                    SEG[7] <= 1;
                  end
    default: begin   
                    SEG[0] <= 1;
                    SEG[1] <= 1;   
                    SEG[2] <= 1;
                    SEG[3] <= 1;
                    SEG[4] <= 1;
                    SEG[5] <= 1;
                    SEG[6] <= 1;
                    SEG[7] <= 1;
             end                
    endcase
    //if (D1_AN == 4'b0111) SEG[7] <= 1;   // Decimal Point of first number could be turned on here
end
assign D1_SEG = SEG;    
assign D1_AN = An;

endmodule
