
?
Command: %s
1870*	planAhead2?
?read_checkpoint -auto_incremental -incremental C:/Vivado/FinalProject/FinalProject.srcs/utils_1/imports/synth_1/FinalProjectwrapper.dcp2default:defaultZ12-2866h px? 
?
;Read reference checkpoint from %s for incremental synthesis3154*	planAhead2l
XC:/Vivado/FinalProject/FinalProject.srcs/utils_1/imports/synth_1/FinalProjectwrapper.dcp2default:defaultZ12-5825h px? 
T
-Please ensure there are no constraint changes3725*	planAheadZ12-7989h px? 
?
Command: %s
53*	vivadotcl2O
;synth_design -top FinalProjectwrapper -part xc7s50csga324-12default:defaultZ4-113h px? 
:
Starting synth_design
149*	vivadotclZ4-321h px? 
?
@Attempting to get a license for feature '%s' and/or device '%s'
308*common2
	Synthesis2default:default2
xc7s502default:defaultZ17-347h px? 
?
0Got license for feature '%s' and/or device '%s'
310*common2
	Synthesis2default:default2
xc7s502default:defaultZ17-349h px? 
V
Loading part %s157*device2#
xc7s50csga324-12default:defaultZ21-403h px? 
?
[Reference run did not run incremental synthesis because %s; reverting to default synthesis
2138*designutils2+
the design is too small2default:defaultZ20-4072h px? 
?
?Flow is switching to default flow due to incremental criteria not met. If you would like to alter this behaviour and have the flow terminate instead, please set the following parameter config_implementation {autoIncr.Synth.RejectBehavior Terminate}2229*designutilsZ20-4379h px? 
?
HMultithreading enabled for synth_design using a maximum of %s processes.4828*oasys2
22default:defaultZ8-7079h px? 
a
?Launching helper process for spawning children vivado processes4827*oasysZ8-7078h px? 
`
#Helper process launched with PID %s4824*oasys2
118642default:defaultZ8-7075h px? 
?
.identifier '%s' is used before its declaration4750*oasys2
A2default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
3382default:default8@Z8-6901h px? 
?
.identifier '%s' is used before its declaration4750*oasys2
A2default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
3392default:default8@Z8-6901h px? 
?
.identifier '%s' is used before its declaration4750*oasys2
A2default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
3402default:default8@Z8-6901h px? 
?
.identifier '%s' is used before its declaration4750*oasys2
A2default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
3412default:default8@Z8-6901h px? 
?
.identifier '%s' is used before its declaration4750*oasys2
A2default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
3422default:default8@Z8-6901h px? 
?
%s*synth2?
wStarting RTL Elaboration : Time (s): cpu = 00:00:04 ; elapsed = 00:00:04 . Memory (MB): peak = 1453.965 ; gain = 0.000
2default:defaulth px? 
?
synthesizing module '%s'%s4497*oasys2'
FinalProjectwrapper2default:default2
 2default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
222default:default8@Z8-6157h px? 
?
synthesizing module '%s'%s4497*oasys2

SPIMCP32012default:default2
 2default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
662default:default8@Z8-6157h px? 
?
?Register %s in module %s has both Set and reset with same priority. This may cause simulation mismatches. Consider rewriting code 
4878*oasys2

SPIclk_reg2default:default2

SPIMCP32012default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
862default:default8@Z8-7137h px? 
?
?Register %s in module %s has both Set and reset with same priority. This may cause simulation mismatches. Consider rewriting code 
4878*oasys2#
SampleClock_reg2default:default2

SPIMCP32012default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
1122default:default8@Z8-7137h px? 
?
?Register %s in module %s has both Set and reset with same priority. This may cause simulation mismatches. Consider rewriting code 
4878*oasys2!
ADCrawout_reg2default:default2

SPIMCP32012default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
1182default:default8@Z8-7137h px? 
?
?Register %s in module %s has both Set and reset with same priority. This may cause simulation mismatches. Consider rewriting code 
4878*oasys2

ADCraw_reg2default:default2

SPIMCP32012default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
1182default:default8@Z8-7137h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2

SPIMCP32012default:default2
 2default:default2
12default:default2
12default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
662default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2 
ADCtoVoltage2default:default2
 2default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
1342default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2 
ADCtoVoltage2default:default2
 2default:default2
22default:default2
12default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
1342default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2(
VoltageToTemperature2default:default2
 2default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
1492default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2(
VoltageToTemperature2default:default2
 2default:default2
32default:default2
12default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
1492default:default8@Z8-6155h px? 
?
Pwidth (%s) of port connection '%s' does not match port width (%s) of module '%s'689*oasys2
252default:default2
Voltage2default:default2
122default:default2(
VoltageToTemperature2default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
482default:default8@Z8-689h px? 
?
synthesizing module '%s'%s4497*oasys2*
TemperatureToDutyCycle2default:default2
 2default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
1682default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2*
TemperatureToDutyCycle2default:default2
 2default:default2
42default:default2
12default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
1682default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2"
DutyCycleToPWM2default:default2
 2default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
1892default:default8@Z8-6157h px? 
?
?Register %s in module %s has both Set and reset with same priority. This may cause simulation mismatches. Consider rewriting code 
4878*oasys2
PWM_reg2default:default2"
DutyCycleToPWM2default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
1992default:default8@Z8-7137h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2"
DutyCycleToPWM2default:default2
 2default:default2
52default:default2
12default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
1892default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2
readTACH2default:default2
 2default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
2122default:default8@Z8-6157h px? 
?
?Register %s in module %s has both Set and reset with same priority. This may cause simulation mismatches. Consider rewriting code 
4878*oasys2
RPM_reg2default:default2
readTACH2default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
2322default:default8@Z8-7137h px? 
?
?Register %s in module %s has both Set and reset with same priority. This may cause simulation mismatches. Consider rewriting code 
4878*oasys2
led_reg2default:default2
readTACH2default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
2382default:default8@Z8-7137h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
readTACH2default:default2
 2default:default2
62default:default2
12default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
2122default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2
SegDrive2default:default2
 2default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
2482default:default8@Z8-6157h px? 
?
default block is never used226*oasys2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
3372default:default8@Z8-226h px? 
?
default block is never used226*oasys2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
3552default:default8@Z8-226h px? 
?
default block is never used226*oasys2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
3972default:default8@Z8-226h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
SegDrive2default:default2
 2default:default2
72default:default2
12default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
2482default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2'
FinalProjectwrapper2default:default2
 2default:default2
82default:default2
12default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
222default:default8@Z8-6155h px? 
?
%s*synth2?
wFinished RTL Elaboration : Time (s): cpu = 00:00:05 ; elapsed = 00:00:05 . Memory (MB): peak = 1453.965 ; gain = 0.000
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
M
%s
*synth25
!Start Handling Custom Attributes
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Handling Custom Attributes : Time (s): cpu = 00:00:06 ; elapsed = 00:00:06 . Memory (MB): peak = 1453.965 ; gain = 0.000
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished RTL Optimization Phase 1 : Time (s): cpu = 00:00:06 ; elapsed = 00:00:06 . Memory (MB): peak = 1453.965 ; gain = 0.000
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2 
00:00:00.0132default:default2
1453.9652default:default2
0.0002default:defaultZ17-268h px? 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px? 
>

Processing XDC Constraints
244*projectZ1-262h px? 
=
Initializing timing engine
348*projectZ1-569h px? 
?
Parsing XDC File [%s]
179*designutils2]
GC:/Vivado/FinalProject/FinalProject.srcs/constrs_1/new/FinalProject.xdc2default:default8Z20-179h px? 
?
Finished Parsing XDC File [%s]
178*designutils2]
GC:/Vivado/FinalProject/FinalProject.srcs/constrs_1/new/FinalProject.xdc2default:default8Z20-178h px? 
?
?Implementation specific constraints were found while reading constraint file [%s]. These constraints will be ignored for synthesis but will be used in implementation. Impacted constraints are listed in the file [%s].
233*project2[
GC:/Vivado/FinalProject/FinalProject.srcs/constrs_1/new/FinalProject.xdc2default:default29
%.Xil/FinalProjectwrapper_propImpl.xdc2default:defaultZ1-236h px? 
H
&Completed Processing XDC Constraints

245*projectZ1-263h px? 
?
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2
00:00:002default:default2
1459.4102default:default2
0.0002default:defaultZ17-268h px? 
~
!Unisim Transformation Summary:
%s111*project29
%No Unisim elements were transformed.
2default:defaultZ1-111h px? 
?
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common24
 Constraint Validation Runtime : 2default:default2
00:00:002default:default2 
00:00:00.0042default:default2
1459.4102default:default2
0.0002default:defaultZ17-268h px? 
?
[Reference run did not run incremental synthesis because %s; reverting to default synthesis
2138*designutils2+
the design is too small2default:defaultZ20-4072h px? 
?
?Flow is switching to default flow due to incremental criteria not met. If you would like to alter this behaviour and have the flow terminate instead, please set the following parameter config_implementation {autoIncr.Synth.RejectBehavior Terminate}2229*designutilsZ20-4379h px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
}Finished Constraint Validation : Time (s): cpu = 00:00:12 ; elapsed = 00:00:12 . Memory (MB): peak = 1459.410 ; gain = 5.445
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
V
%s
*synth2>
*Start Loading Part and Timing Information
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
J
%s
*synth22
Loading part: xc7s50csga324-1
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Loading Part and Timing Information : Time (s): cpu = 00:00:12 ; elapsed = 00:00:12 . Memory (MB): peak = 1459.410 ; gain = 5.445
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
Z
%s
*synth2B
.Start Applying 'set_property' XDC Constraints
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished applying 'set_property' XDC Constraints : Time (s): cpu = 00:00:12 ; elapsed = 00:00:12 . Memory (MB): peak = 1459.410 ; gain = 5.445
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished RTL Optimization Phase 2 : Time (s): cpu = 00:00:13 ; elapsed = 00:00:13 . Memory (MB): peak = 1459.410 ; gain = 5.445
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
L
%s
*synth24
 Start RTL Component Statistics 
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Detailed RTL Component Info : 
2default:defaulth p
x
? 
:
%s
*synth2"
+---Adders : 
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input   29 Bit       Adders := 5     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input   25 Bit       Adders := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input   16 Bit       Adders := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    4 Bit       Adders := 32    
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    3 Bit       Adders := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    2 Bit       Adders := 1     
2default:defaulth p
x
? 
=
%s
*synth2%
+---Registers : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               29 Bit    Registers := 5     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               25 Bit    Registers := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               24 Bit    Registers := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               22 Bit    Registers := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               16 Bit    Registers := 4     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               14 Bit    Registers := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               12 Bit    Registers := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                8 Bit    Registers := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                2 Bit    Registers := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                1 Bit    Registers := 7     
2default:defaulth p
x
? 
9
%s
*synth2!
+---Muxes : 
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input   29 Bit        Muxes := 5     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input   16 Bit        Muxes := 5     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input   15 Bit        Muxes := 2     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input   14 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   3 Input    8 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   4 Input    4 Bit        Muxes := 2     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    4 Bit        Muxes := 24    
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    2 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    1 Bit        Muxes := 9     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   3 Input    1 Bit        Muxes := 1     
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
O
%s
*synth27
#Finished RTL Component Statistics 
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
H
%s
*synth20
Start Part Resource Summary
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2k
WPart Resources:
DSPs: 120 (col length:60)
BRAMs: 150 (col length: RAMB18 60 RAMB36 30)
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Finished Part Resource Summary
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
W
%s
*synth2?
+Start Cross Boundary and Area Optimization
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
H
&Parallel synthesis criteria is not met4829*oasysZ8-7080h px? 
?
RFound unconnected internal register '%s' and it is trimmed from '%s' to '%s' bits.3455*oasys2#
A2V/Voltage_reg2default:default2
242default:default2
122default:default2\
FC:/Vivado/FinalProject/FinalProject.srcs/sources_1/new/FinalProject.sv2default:default2
1432default:default8@Z8-3936h px? 
~
%s
*synth2f
RDSP Report: Generating DSP A2V/Voltageint_reg, operation Mode is: (A*(B:0xd02))'.
2default:defaulth p
x
? 
}
%s
*synth2e
QDSP Report: register A2V/Voltageint_reg is absorbed into DSP A2V/Voltageint_reg.
2default:defaulth p
x
? 
z
%s
*synth2b
NDSP Report: operator A2V/Voltageint0 is absorbed into DSP A2V/Voltageint_reg.
2default:defaulth p
x
? 
?
%s
*synth2l
XDSP Report: Generating DSP V2T/Temperatureint1_reg, operation Mode is: (A2*(B:0x293))'.
2default:defaulth p
x
? 

%s
*synth2g
SDSP Report: register A2V/Voltage_reg is absorbed into DSP V2T/Temperatureint1_reg.
2default:defaulth p
x
? 
?
%s
*synth2o
[DSP Report: register V2T/Temperatureint1_reg is absorbed into DSP V2T/Temperatureint1_reg.
2default:defaulth p
x
? 
?
%s
*synth2l
XDSP Report: operator V2T/Temperatureint10 is absorbed into DSP V2T/Temperatureint1_reg.
2default:defaulth p
x
? 
o
%s
*synth2W
CDSP Report: Generating DSP rT/RPM0, operation Mode is: A*(B:0x1e).
2default:defaulth p
x
? 
g
%s
*synth2O
;DSP Report: operator rT/RPM0 is absorbed into DSP rT/RPM0.
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Cross Boundary and Area Optimization : Time (s): cpu = 00:00:21 ; elapsed = 00:00:21 . Memory (MB): peak = 1459.410 ; gain = 5.445
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?---------------------------------------------------------------------------------
Start ROM, RAM, DSP, Shift Register and Retiming Reporting
2default:defaulth px? 
~
%s*synth2f
R---------------------------------------------------------------------------------
2default:defaulth px? 
?
%s*synth2p
\
DSP: Preliminary Mapping Report (see note below. The ' indicates corresponding REG is set)
2default:defaulth px? 
?
%s*synth2?
?+--------------------+-----------------+--------+--------+--------+--------+--------+------+------+------+------+-------+------+------+
2default:defaulth px? 
?
%s*synth2?
?|Module Name         | DSP Mapping     | A Size | B Size | C Size | D Size | P Size | AREG | BREG | CREG | DREG | ADREG | MREG | PREG | 
2default:defaulth px? 
?
%s*synth2?
?+--------------------+-----------------+--------+--------+--------+--------+--------+------+------+------+------+-------+------+------+
2default:defaulth px? 
?
%s*synth2?
?|ADCtoVoltage        | (A*(B:0xd02))'  | 12     | 12     | -      | -      | 24     | 0    | 0    | -    | -    | -     | 1    | 0    | 
2default:defaulth px? 
?
%s*synth2?
?|FinalProjectwrapper | (A2*(B:0x293))' | 12     | 10     | -      | -      | 22     | 1    | 0    | -    | -    | -     | 1    | 0    | 
2default:defaulth px? 
?
%s*synth2?
?|readTACH            | A*(B:0x1e)      | 16     | 5      | -      | -      | 21     | 0    | 0    | -    | -    | -     | 0    | 0    | 
2default:defaulth px? 
?
%s*synth2?
?+--------------------+-----------------+--------+--------+--------+--------+--------+------+------+------+------+-------+------+------+

2default:defaulth px? 
?
%s*synth2?
?Note: The table above is a preliminary report that shows the DSPs inferred at the current stage of the synthesis flow. Some DSP may be reimplemented as non DSP primitives later in the synthesis flow. Multiple instantiated DSPs are reported only once.
2default:defaulth px? 
?
%s*synth2?
?---------------------------------------------------------------------------------
Finished ROM, RAM, DSP, Shift Register and Retiming Reporting
2default:defaulth px? 
~
%s*synth2f
R---------------------------------------------------------------------------------
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
R
%s
*synth2:
&Start Applying XDC Timing Constraints
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Applying XDC Timing Constraints : Time (s): cpu = 00:00:29 ; elapsed = 00:00:30 . Memory (MB): peak = 1459.410 ; gain = 5.445
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
F
%s
*synth2.
Start Timing Optimization
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
{Finished Timing Optimization : Time (s): cpu = 00:00:29 ; elapsed = 00:00:30 . Memory (MB): peak = 1459.410 ; gain = 5.445
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
E
%s
*synth2-
Start Technology Mapping
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
{Finished Technology Mapping : Time (s): cpu = 00:00:30 ; elapsed = 00:00:30 . Memory (MB): peak = 1470.297 ; gain = 16.332
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2'
Start IO Insertion
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
Q
%s
*synth29
%Start Flattening Before IO Insertion
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
T
%s
*synth2<
(Finished Flattening Before IO Insertion
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
H
%s
*synth20
Start Final Netlist Cleanup
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Finished Final Netlist Cleanup
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
uFinished IO Insertion : Time (s): cpu = 00:00:34 ; elapsed = 00:00:34 . Memory (MB): peak = 1485.145 ; gain = 31.180
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
O
%s
*synth27
#Start Renaming Generated Instances
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Renaming Generated Instances : Time (s): cpu = 00:00:34 ; elapsed = 00:00:34 . Memory (MB): peak = 1485.145 ; gain = 31.180
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
L
%s
*synth24
 Start Rebuilding User Hierarchy
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Rebuilding User Hierarchy : Time (s): cpu = 00:00:34 ; elapsed = 00:00:34 . Memory (MB): peak = 1485.145 ; gain = 31.180
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Start Renaming Generated Ports
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Renaming Generated Ports : Time (s): cpu = 00:00:34 ; elapsed = 00:00:34 . Memory (MB): peak = 1485.145 ; gain = 31.180
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
M
%s
*synth25
!Start Handling Custom Attributes
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Handling Custom Attributes : Time (s): cpu = 00:00:34 ; elapsed = 00:00:34 . Memory (MB): peak = 1485.145 ; gain = 31.180
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
J
%s
*synth22
Start Renaming Generated Nets
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Renaming Generated Nets : Time (s): cpu = 00:00:34 ; elapsed = 00:00:34 . Memory (MB): peak = 1485.145 ; gain = 31.180
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Start Writing Synthesis Report
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
A
%s
*synth2)

Report BlackBoxes: 
2default:defaulth p
x
? 
J
%s
*synth22
+-+--------------+----------+
2default:defaulth p
x
? 
J
%s
*synth22
| |BlackBox name |Instances |
2default:defaulth p
x
? 
J
%s
*synth22
+-+--------------+----------+
2default:defaulth p
x
? 
J
%s
*synth22
+-+--------------+----------+
2default:defaulth p
x
? 
A
%s*synth2)

Report Cell Usage: 
2default:defaulth px? 
E
%s*synth2-
+------+--------+------+
2default:defaulth px? 
E
%s*synth2-
|      |Cell    |Count |
2default:defaulth px? 
E
%s*synth2-
+------+--------+------+
2default:defaulth px? 
E
%s*synth2-
|1     |BUFG    |     2|
2default:defaulth px? 
E
%s*synth2-
|2     |CARRY4  |    53|
2default:defaulth px? 
E
%s*synth2-
|3     |DSP48E1 |     3|
2default:defaulth px? 
E
%s*synth2-
|6     |LUT1    |     9|
2default:defaulth px? 
E
%s*synth2-
|7     |LUT2    |   128|
2default:defaulth px? 
E
%s*synth2-
|8     |LUT3    |    37|
2default:defaulth px? 
E
%s*synth2-
|9     |LUT4    |    71|
2default:defaulth px? 
E
%s*synth2-
|10    |LUT5    |    65|
2default:defaulth px? 
E
%s*synth2-
|11    |LUT6    |   136|
2default:defaulth px? 
E
%s*synth2-
|12    |FDCE    |   175|
2default:defaulth px? 
E
%s*synth2-
|13    |FDRE    |    91|
2default:defaulth px? 
E
%s*synth2-
|14    |IBUF    |     4|
2default:defaulth px? 
E
%s*synth2-
|15    |OBUF    |    28|
2default:defaulth px? 
E
%s*synth2-
|16    |OBUFT   |     3|
2default:defaulth px? 
E
%s*synth2-
+------+--------+------+
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Writing Synthesis Report : Time (s): cpu = 00:00:34 ; elapsed = 00:00:34 . Memory (MB): peak = 1485.145 ; gain = 31.180
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
r
%s
*synth2Z
FSynthesis finished with 0 errors, 0 critical warnings and 2 warnings.
2default:defaulth p
x
? 
?
%s
*synth2?
~Synthesis Optimization Runtime : Time (s): cpu = 00:00:26 ; elapsed = 00:00:33 . Memory (MB): peak = 1485.145 ; gain = 25.734
2default:defaulth p
x
? 
?
%s
*synth2?
Synthesis Optimization Complete : Time (s): cpu = 00:00:34 ; elapsed = 00:00:34 . Memory (MB): peak = 1485.145 ; gain = 31.180
2default:defaulth p
x
? 
B
 Translating synthesized netlist
350*projectZ1-571h px? 
?
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2 
00:00:00.0112default:default2
1497.2662default:default2
0.0002default:defaultZ17-268h px? 
f
-Analyzing %s Unisim elements for replacement
17*netlist2
562default:defaultZ29-17h px? 
j
2Unisim Transformation completed in %s CPU seconds
28*netlist2
02default:defaultZ29-28h px? 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px? 
u
)Pushed %s inverter(s) to %s load pin(s).
98*opt2
02default:default2
02default:defaultZ31-138h px? 
?
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2 
00:00:00.0012default:default2
1501.0232default:default2
0.0002default:defaultZ17-268h px? 
~
!Unisim Transformation Summary:
%s111*project29
%No Unisim elements were transformed.
2default:defaultZ1-111h px? 
g
$Synth Design complete, checksum: %s
562*	vivadotcl2
bf8b32212default:defaultZ4-1430h px? 
U
Releasing license: %s
83*common2
	Synthesis2default:defaultZ17-83h px? 
?
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
382default:default2
172default:default2
02default:default2
02default:defaultZ4-41h px? 
^
%s completed successfully
29*	vivadotcl2 
synth_design2default:defaultZ4-42h px? 
?
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2"
synth_design: 2default:default2
00:00:372default:default2
00:00:382default:default2
1501.0232default:default2
47.0592default:defaultZ17-268h px? 
?
 The %s '%s' has been generated.
621*common2

checkpoint2default:default2\
HC:/Vivado/FinalProject/FinalProject.runs/synth_1/FinalProjectwrapper.dcp2default:defaultZ17-1381h px? 
?
%s4*runtcl2?
|Executing : report_utilization -file FinalProjectwrapper_utilization_synth.rpt -pb FinalProjectwrapper_utilization_synth.pb
2default:defaulth px? 
?
Exiting %s at %s...
206*common2
Vivado2default:default2,
Sun Jul 10 19:16:13 20222default:defaultZ17-206h px? 


End Record