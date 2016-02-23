EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CONN_01X10 P5
U 1 1 56C75F10
P 5650 3550
F 0 "P5" H 5650 4100 50  0000 C CNN
F 1 "CONN_01X10" V 5750 3550 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x10" H 5650 3550 50  0001 C CNN
F 3 "" H 5650 3550 50  0000 C CNN
	1    5650 3550
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X10 P6
U 1 1 56C760B7
P 6300 3550
F 0 "P6" H 6300 4100 50  0000 C CNN
F 1 "CONN_01X10" V 6400 3550 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x10" H 6300 3550 50  0001 C CNN
F 3 "" H 6300 3550 50  0000 C CNN
	1    6300 3550
	1    0    0    -1  
$EndComp
NoConn ~ 6100 4000
NoConn ~ 6100 3900
NoConn ~ 6100 3800
NoConn ~ 6100 3700
NoConn ~ 6100 3600
NoConn ~ 6100 3500
NoConn ~ 6100 3400
NoConn ~ 6100 3300
NoConn ~ 5450 3300
NoConn ~ 5450 3400
NoConn ~ 5450 3500
NoConn ~ 5450 3600
NoConn ~ 5450 3200
NoConn ~ 5450 3100
$Comp
L CONN_01X03 P3
U 1 1 56C76388
P 3900 2550
F 0 "P3" H 3900 2750 50  0000 C CNN
F 1 "CONN_01X03" V 4000 2550 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x03" H 3900 2550 50  0001 C CNN
F 3 "" H 3900 2550 50  0000 C CNN
	1    3900 2550
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X03 P4
U 1 1 56C76475
P 3900 3150
F 0 "P4" H 3900 3350 50  0000 C CNN
F 1 "CONN_01X03" V 4000 3150 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x03" H 3900 3150 50  0001 C CNN
F 3 "" H 3900 3150 50  0000 C CNN
	1    3900 3150
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X03 P2
U 1 1 56C764FA
P 3900 1950
F 0 "P2" H 3900 2150 50  0000 C CNN
F 1 "CONN_01X03" V 4000 1950 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x03" H 3900 1950 50  0001 C CNN
F 3 "" H 3900 1950 50  0000 C CNN
	1    3900 1950
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X03 P1
U 1 1 56C7653D
P 3900 1350
F 0 "P1" H 3900 1550 50  0000 C CNN
F 1 "CONN_01X03" V 4000 1350 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x03" H 3900 1350 50  0001 C CNN
F 3 "" H 3900 1350 50  0000 C CNN
	1    3900 1350
	-1   0    0    1   
$EndComp
Wire Wire Line
	5450 3700 4900 3700
Wire Wire Line
	5450 3800 4800 3800
Wire Wire Line
	5450 3900 4700 3900
Wire Wire Line
	5450 4000 4600 4000
Wire Wire Line
	3600 1100 3600 3600
Wire Wire Line
	3600 1100 6000 1100
Wire Wire Line
	6000 1100 6000 3100
Wire Wire Line
	6000 3100 6100 3100
$Comp
L GND #PWR01
U 1 1 56C7675B
P 3600 3600
F 0 "#PWR01" H 3600 3350 50  0001 C CNN
F 1 "GND" H 3600 3450 50  0000 C CNN
F 2 "" H 3600 3600 50  0000 C CNN
F 3 "" H 3600 3600 50  0000 C CNN
	1    3600 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 4000 4600 3250
Wire Wire Line
	4600 3250 4100 3250
Wire Wire Line
	4700 3900 4700 2650
Wire Wire Line
	4700 2650 4100 2650
Wire Wire Line
	4800 3800 4800 2050
Wire Wire Line
	4800 2050 4100 2050
Wire Wire Line
	4900 3700 4900 1450
Wire Wire Line
	4900 1450 4100 1450
NoConn ~ 4100 3150
NoConn ~ 4100 2550
NoConn ~ 4100 1950
NoConn ~ 4100 1350
Wire Wire Line
	4100 3050 4100 2850
Wire Wire Line
	4100 2850 3600 2850
Connection ~ 3600 2850
Wire Wire Line
	4100 2450 4100 2250
Wire Wire Line
	4100 2250 3600 2250
Connection ~ 3600 2250
Wire Wire Line
	4100 1850 4100 1650
Wire Wire Line
	4100 1650 3600 1650
Connection ~ 3600 1650
Wire Wire Line
	4100 1250 4100 1100
Connection ~ 4100 1100
$EndSCHEMATC
