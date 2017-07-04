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
LIBS:rusty-parts
LIBS:sensor_head-cache
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
L CONN_01X02 P1
U 1 1 5803CEA6
P 3150 2250
F 0 "P1" H 3150 2400 50  0000 C CNN
F 1 "COND_LEFT" V 3250 2250 50  0000 C CNN
F 2 "sensor_head_footprints:cond_10mm" H 3150 2250 50  0001 C CNN
F 3 "" H 3150 2250 50  0000 C CNN
	1    3150 2250
	-1   0    0    -1  
$EndComp
$Comp
L CONN_01X02 P2
U 1 1 5803CEC7
P 3150 2750
F 0 "P2" H 3150 2900 50  0000 C CNN
F 1 "COND_RIGHT" V 3250 2750 50  0000 C CNN
F 2 "sensor_head_footprints:cond_10mm" H 3150 2750 50  0001 C CNN
F 3 "" H 3150 2750 50  0000 C CNN
	1    3150 2750
	-1   0    0    -1  
$EndComp
$Comp
L MS5803 U1
U 1 1 5803D50A
P 3150 1550
F 0 "U1" H 2850 1900 50  0000 L CNN
F 1 "MS5803" H 3150 1900 50  0000 L CNN
F 2 "sensor_head_footprints:MS5803-through" H 2200 1550 50  0001 C CIN
F 3 "" H 2200 1550 50  0000 C CNN
	1    3150 1550
	1    0    0    -1  
$EndComp
$Comp
L THERMISTOR TH1
U 1 1 5803D590
P 3500 3200
F 0 "TH1" V 3600 3250 50  0000 C CNN
F 1 "THERMISTOR" V 3400 3200 50  0000 C BNN
F 2 "Discret:R1" H 3500 3200 50  0001 C CNN
F 3 "" H 3500 3200 50  0000 C CNN
	1    3500 3200
	0    1    1    0   
$EndComp
Wire Wire Line
	3600 1400 4300 1400
Wire Wire Line
	3600 1500 4200 1500
Wire Wire Line
	2700 1600 2500 1600
Wire Wire Line
	2500 1600 2500 2000
Wire Wire Line
	2500 2000 3900 2000
Wire Wire Line
	3600 1600 3750 1600
Wire Wire Line
	3750 1600 3750 2000
Connection ~ 3750 2000
Wire Wire Line
	2700 1700 2500 1700
Connection ~ 2500 1700
Wire Wire Line
	2700 1500 2500 1500
Wire Wire Line
	2500 1500 2500 1000
Wire Wire Line
	2500 1000 4400 1000
$Comp
L C_Small C1
U 1 1 5803D6AC
P 3900 1700
F 0 "C1" H 3910 1770 50  0000 L CNN
F 1 "100nF" H 3910 1620 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Disc_D6_P5" H 3900 1700 50  0001 C CNN
F 3 "" H 3900 1700 50  0000 C CNN
	1    3900 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 1600 3900 1500
Connection ~ 3900 1500
Wire Wire Line
	3900 1800 3900 2100
Connection ~ 3900 2000
Wire Wire Line
	3350 2200 4500 2200
Wire Wire Line
	3350 2300 4500 2300
Wire Wire Line
	3350 2700 4500 2700
Wire Wire Line
	3350 2800 4500 2800
Wire Wire Line
	3250 3200 3100 3200
Wire Wire Line
	3100 3200 3100 3400
Wire Wire Line
	3100 3400 4000 3400
Wire Wire Line
	3900 2100 4500 2100
Wire Wire Line
	4200 1500 4200 2000
Wire Wire Line
	4200 2000 4500 2000
Wire Wire Line
	4300 1400 4300 1900
Wire Wire Line
	4300 1900 4500 1900
Wire Wire Line
	4400 1000 4400 1800
Wire Wire Line
	4400 1800 4500 1800
Wire Wire Line
	3750 3200 3900 3200
Wire Wire Line
	3900 3200 3900 2900
Wire Wire Line
	3900 2900 4500 2900
Wire Wire Line
	4000 3400 4000 3000
Wire Wire Line
	4000 3000 4500 3000
$Comp
L CONN_01X06 P3
U 1 1 5803DB4F
P 4700 2050
F 0 "P3" H 4700 2400 50  0000 C CNN
F 1 "CONN_01X06" V 4800 2050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06" H 4700 2050 50  0001 C CNN
F 3 "" H 4700 2050 50  0000 C CNN
	1    4700 2050
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X04 P4
U 1 1 5803DBA4
P 4700 2850
F 0 "P4" H 4700 3100 50  0000 C CNN
F 1 "CONN_01X04" V 4800 2850 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04" H 4700 2850 50  0001 C CNN
F 3 "" H 4700 2850 50  0000 C CNN
	1    4700 2850
	1    0    0    -1  
$EndComp
NoConn ~ 2700 1400
$Comp
L MS5803 U2
U 1 1 5803FC16
P 3350 4150
F 0 "U2" H 3050 4500 50  0000 L CNN
F 1 "MS5803" H 3350 4500 50  0000 L CNN
F 2 "sensor_head_footprints:ms5803-surface" H 2400 4150 50  0001 C CIN
F 3 "" H 2400 4150 50  0000 C CNN
	1    3350 4150
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X04 P5
U 1 1 5803FC58
P 4750 4150
F 0 "P5" H 4750 4400 50  0000 C CNN
F 1 "CONN_01X04" V 4850 4150 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04" H 4750 4150 50  0001 C CNN
F 3 "" H 4750 4150 50  0000 C CNN
	1    4750 4150
	1    0    0    -1  
$EndComp
$Comp
L C_Small C2
U 1 1 5803FC86
P 4100 4350
F 0 "C2" H 4110 4420 50  0000 L CNN
F 1 "100nF" H 4110 4270 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Disc_D6_P5" H 4100 4350 50  0001 C CNN
F 3 "" H 4100 4350 50  0000 C CNN
	1    4100 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 4000 4550 4000
Wire Wire Line
	3800 4100 4550 4100
Wire Wire Line
	2900 4200 2800 4200
Wire Wire Line
	2800 4200 2800 4600
Wire Wire Line
	2800 4600 4400 4600
Wire Wire Line
	2900 4300 2800 4300
Connection ~ 2800 4300
Wire Wire Line
	3800 4200 3900 4200
Wire Wire Line
	3900 4200 3900 4600
Connection ~ 3900 4600
Wire Wire Line
	4100 4100 4100 4250
Connection ~ 4100 4100
Wire Wire Line
	4100 4450 4100 4600
Connection ~ 4100 4600
Wire Wire Line
	4400 4600 4400 4200
Wire Wire Line
	4400 4200 4550 4200
Wire Wire Line
	4550 4300 4500 4300
Wire Wire Line
	4500 4300 4500 4650
Wire Wire Line
	4500 4650 2700 4650
Wire Wire Line
	2700 4650 2700 4100
Wire Wire Line
	2700 4100 2900 4100
NoConn ~ 2900 4000
Text Label 2500 1200 0    60   ~ 0
sda1
Text Label 3700 1400 0    60   ~ 0
scl1
Text Label 3700 1500 0    60   ~ 0
gnd1
Text Label 2550 2000 0    60   ~ 0
vdd1
Text Label 3550 2200 0    60   ~ 0
cond_left_a
Text Label 3500 2300 0    60   ~ 0
cond_left_b
Text Label 3950 4000 0    60   ~ 0
scl2
Text Label 3950 4100 0    60   ~ 0
gnd2
Text Label 2750 4100 0    60   ~ 0
sda2
Text Label 2950 4600 0    60   ~ 0
vdd2
$EndSCHEMATC
