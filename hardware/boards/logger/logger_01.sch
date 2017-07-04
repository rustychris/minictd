EESchema Schematic File Version 2
LIBS:logger_01-rescue
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
LIBS:logger_parts
LIBS:myparts
LIBS:teensy
EELAYER 26 0
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
Text Label 1900 1100 2    60   ~ 0
3V7
$Comp
L GND #PWR01
U 1 1 58197426
P 1900 1750
F 0 "#PWR01" H 1900 1500 50  0001 C CNN
F 1 "GND" H 1900 1600 50  0000 C CNN
F 2 "" H 1900 1750 50  0000 C CNN
F 3 "" H 1900 1750 50  0000 C CNN
	1    1900 1750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 581984F8
P 2400 4850
F 0 "#PWR02" H 2400 4600 50  0001 C CNN
F 1 "GND" H 2400 4700 50  0000 C CNN
F 2 "" H 2400 4850 50  0000 C CNN
F 3 "" H 2400 4850 50  0000 C CNN
	1    2400 4850
	1    0    0    -1  
$EndComp
Text Label 3900 1200 0    60   ~ 0
3V3
Text Label 6950 1650 0    60   ~ 0
3V3
Text Label 6950 2800 0    60   ~ 0
3V3
$Comp
L R_Small R2
U 1 1 5831E7EC
P 5550 2600
F 0 "R2" H 5580 2620 50  0000 L CNN
F 1 "50k" H 5580 2560 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 5550 2600 50  0001 C CNN
F 3 "" H 5550 2600 50  0000 C CNN
F 4 "RT0805DRE0722K1L" H 5550 2600 60  0001 C CNN "part_num"
F 5 "subbing a 12k low T coeff." H 5550 2600 60  0001 C CNN "notes"
F 6 "311-2825-1-ND" H 5550 2600 60  0001 C CNN "digikey_num"
	1    5550 2600
	1    0    0    -1  
$EndComp
$Comp
L R_Small R1
U 1 1 5831E8CD
P 5550 2200
F 0 "R1" H 5580 2220 50  0000 L CNN
F 1 "50k" H 5580 2160 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 5550 2200 50  0001 C CNN
F 3 "" H 5550 2200 50  0000 C CNN
F 4 "RT0805DRE0722K1L" H 5550 2200 60  0001 C CNN "part_num"
F 5 "subbing a 22k low T coeff" H 5550 2200 60  0001 C CNN "notes"
F 6 "311-2825-1-ND" H 5550 2200 60  0001 C CNN "digikey_num"
	1    5550 2200
	1    0    0    -1  
$EndComp
$Comp
L C_Small C8
U 1 1 5831F56F
P 7350 3000
F 0 "C8" H 7360 3070 50  0000 L CNN
F 1 "0.1uF" H 7360 2920 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 7350 3000 50  0001 C CNN
F 3 "" H 7350 3000 50  0000 C CNN
F 4 "08055C104KAT2A" H 7350 3000 60  0001 C CNN "part_num"
	1    7350 3000
	1    0    0    -1  
$EndComp
Text Label 2700 6550 2    60   ~ 0
DAC
Text Label 6550 2000 0    60   ~ 0
DAC
Text Label 5900 2400 0    60   ~ 0
Half3v3
$Comp
L R_Small R3
U 1 1 58321775
P 7850 2250
F 0 "R3" H 7880 2270 50  0000 L CNN
F 1 "500" H 7880 2210 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 7850 2250 50  0001 C CNN
F 3 "" H 7850 2250 50  0000 C CNN
F 4 "RR1220P-471-D" H 7850 2250 60  0001 C CNN "part_num"
F 5 "that's a 25ppm/C 470.  doesn't need to be that nice." H 7850 2250 60  0001 C CNN "notes"
F 6 "RR12P470DCT-ND" H 7850 2250 60  0001 C CNN "digikey_num"
	1    7850 2250
	1    0    0    -1  
$EndComp
Text Label 8700 2200 2    60   ~ 0
CurSense1
Text Label 9000 2200 0    60   ~ 0
CurSense2
Text Label 7850 2450 0    60   ~ 0
CurDriveA
Text Label 7850 3300 0    60   ~ 0
CurDriveB
$Comp
L OPA2350 U5
U 2 1 58321FE0
P 7050 2100
F 0 "U5" H 7050 2250 60  0000 L CNN
F 1 "MCP6072" H 7050 1950 60  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 7050 2100 60  0001 C CNN
F 3 "http://www.microchip.com/mymicrochip/filehandler.aspx?ddocname=en540513" H 7050 2100 60  0001 C CNN
F 4 "MCP6072-E/SN" H 7050 2100 60  0001 C CNN "part_num"
F 5 "MCP6072-E/SN-ND" H 7050 2100 60  0001 C CNN "digikey_num"
	2    7050 2100
	1    0    0    -1  
$EndComp
Text Label 9950 500  0    60   ~ 0
3V3
Text Label 10600 1050 0    60   ~ 0
A12
Text Label 10600 1450 0    60   ~ 0
A13
Text Label 5000 5050 0    60   ~ 0
A13
Text Label 5000 6350 0    60   ~ 0
A12
$Comp
L C_Small C7
U 1 1 58325918
P 5200 2600
F 0 "C7" H 5210 2670 50  0000 L CNN
F 1 "0.1uF" H 5210 2520 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5200 2600 50  0001 C CNN
F 3 "" H 5200 2600 50  0000 C CNN
F 4 "08055C104KAT2A" H 5200 2600 60  0001 C CNN "part_num"
F 5 "478-1395-1-ND" H 5200 2600 60  0001 C CNN "digikey_num"
	1    5200 2600
	1    0    0    -1  
$EndComp
$Comp
L AD623 U6
U 1 1 583262DB
P 10200 5700
F 0 "U6" H 10400 5850 50  0000 L CNN
F 1 "AD623" H 10400 5550 50  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 10200 5700 50  0001 C CNN
F 3 "" H 10200 5700 50  0000 C CNN
F 4 "AD623ARZ" H 10200 5700 60  0001 C CNN "part_num"
F 5 "AD623ARZ-ND" H 10200 5700 60  0001 C CNN "digikey_num"
	1    10200 5700
	1    0    0    -1  
$EndComp
Text Label 9000 3450 0    60   ~ 0
3V3
Text Label 8650 4100 2    60   ~ 0
Half3v3
Text Label 10100 4850 0    60   ~ 0
3V3
$Comp
L R_Small R4
U 1 1 58326E21
P 10300 6150
F 0 "R4" H 10330 6170 50  0000 L CNN
F 1 "10k" H 10330 6110 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 10300 6150 50  0001 C CNN
F 3 "" H 10300 6150 50  0000 C CNN
F 4 "RT0805DRD0712K1L" H 10300 6150 60  0001 C CNN "part_num"
	1    10300 6150
	1    0    0    -1  
$EndComp
Text Label 9500 5600 0    60   ~ 0
VSense1
Text Label 9500 5800 0    60   ~ 0
VSense2
Text Label 10650 5700 0    60   ~ 0
A10
Text Label 10650 5350 0    60   ~ 0
A11
Text Label 5000 6450 0    60   ~ 0
A11
Text Label 5000 6550 0    60   ~ 0
A10
$Comp
L CONN_01X02 P1
U 1 1 5832AE94
P 8650 5750
F 0 "P1" H 8650 5900 50  0000 C CNN
F 1 "CONN_01X02" V 8750 5750 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 8650 5750 50  0001 C CNN
F 3 "" H 8650 5750 50  0000 C CNN
	1    8650 5750
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X02 P2
U 1 1 5832B9BA
P 9350 2550
F 0 "P2" H 9350 2700 50  0000 C CNN
F 1 "CONN_01X02" V 9450 2550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 9350 2550 50  0001 C CNN
F 3 "" H 9350 2550 50  0000 C CNN
	1    9350 2550
	1    0    0    -1  
$EndComp
$Comp
L R_Small R6
U 1 1 58373041
P 2350 6700
F 0 "R6" H 2380 6720 50  0000 L CNN
F 1 "10k" H 2380 6660 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 2350 6700 50  0001 C CNN
F 3 "" H 2350 6700 50  0000 C CNN
F 4 "RT0805DRD0712K1L" H 2350 6700 60  0001 C CNN "part_num"
	1    2350 6700
	1    0    0    -1  
$EndComp
$Comp
L R_Small R5
U 1 1 58373125
P 2150 6700
F 0 "R5" H 2180 6720 50  0000 L CNN
F 1 "10k" H 2180 6660 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 2150 6700 50  0001 C CNN
F 3 "" H 2150 6700 50  0000 C CNN
F 4 "RT0805DRD0712K1L" H 2150 6700 60  0001 C CNN "part_num"
F 5 "311-2703-1-ND" H 2150 6700 60  0001 C CNN "digikey_num"
	1    2150 6700
	1    0    0    -1  
$EndComp
Text Label 2550 6950 0    60   ~ 0
SCL
Text Label 2550 7050 0    60   ~ 0
SDA
$Comp
L CONN_01X04 P3
U 1 1 58374FC5
P 1700 7000
F 0 "P3" H 1700 7250 50  0000 C CNN
F 1 "CONN_01X04" V 1800 7000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 1700 7000 50  0001 C CNN
F 3 "" H 1700 7000 50  0000 C CNN
F 4 "S1011EC-12-ND" H 1700 7000 60  0001 C CNN "part_num"
F 5 "PRPC012SAAN-RC" H 1700 7000 60  0001 C CNN "digikey_num"
	1    1700 7000
	-1   0    0    1   
$EndComp
Wire Wire Line
	1400 1750 3300 1750
Wire Wire Line
	1400 1200 2150 1200
Wire Wire Line
	3050 1200 3900 1200
Wire Wire Line
	3650 1200 3650 1400
Wire Wire Line
	3650 1600 3650 1800
Connection ~ 3650 1200
Wire Wire Line
	6950 2400 6950 2400
Wire Wire Line
	6950 1800 6950 1650
Wire Wire Line
	6700 2200 6600 2200
Wire Wire Line
	6600 2200 6600 2650
Wire Wire Line
	6600 2650 7500 2650
Wire Wire Line
	7500 2650 7500 2100
Wire Wire Line
	7400 2100 7850 2100
Connection ~ 7500 2100
Wire Wire Line
	7400 3350 7850 3350
Wire Wire Line
	7550 3350 7550 3900
Wire Wire Line
	7550 3900 6600 3900
Wire Wire Line
	6600 3900 6600 3450
Wire Wire Line
	6600 3450 6700 3450
Connection ~ 7550 3350
Wire Wire Line
	5550 2700 5550 2850
Wire Wire Line
	6950 2800 6950 3050
Wire Wire Line
	6950 2900 7350 2900
Connection ~ 6950 2900
Wire Wire Line
	5550 2300 5550 2500
Wire Wire Line
	5200 2400 5900 2400
Connection ~ 5550 2400
Wire Wire Line
	5200 2800 5550 2800
Connection ~ 5550 2800
Wire Wire Line
	5550 2050 5550 2100
Wire Wire Line
	6700 2000 6550 2000
Wire Wire Line
	5200 2700 5200 2800
Wire Wire Line
	9000 3450 9000 3900
Wire Wire Line
	9000 4500 9000 4550
Wire Wire Line
	8750 4300 8650 4300
Wire Wire Line
	8650 4300 8650 4500
Wire Wire Line
	8650 4500 9450 4500
Wire Wire Line
	9450 4500 9450 4200
Wire Wire Line
	10100 4850 10100 5400
Wire Wire Line
	10300 6000 10300 6050
Wire Wire Line
	10200 6000 10200 6300
Wire Wire Line
	10200 6300 10300 6300
Wire Wire Line
	10300 6300 10300 6250
Wire Wire Line
	10500 5700 10650 5700
Wire Wire Line
	2350 7050 2350 6800
Wire Wire Line
	2150 6950 2150 6800
Connection ~ 2350 7050
Connection ~ 2150 6950
$Comp
L GND #PWR03
U 1 1 58375965
P 2250 7150
F 0 "#PWR03" H 2250 6900 50  0001 C CNN
F 1 "GND" H 2250 7000 50  0000 C CNN
F 2 "" H 2250 7150 50  0000 C CNN
F 3 "" H 2250 7150 50  0000 C CNN
	1    2250 7150
	1    0    0    -1  
$EndComp
$Comp
L AD623 U8
U 1 1 58375F30
P 7600 5750
F 0 "U8" H 7800 5900 50  0000 L CNN
F 1 "AD623" H 7800 5600 50  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 7600 5750 50  0001 C CNN
F 3 "" H 7600 5750 50  0000 C CNN
F 4 "AD623ARZ" H 7600 5750 60  0001 C CNN "part_num"
F 5 "AD623ARZ-ND" H 7600 5750 60  0001 C CNN "digikey_num"
	1    7600 5750
	1    0    0    -1  
$EndComp
Text Label 7500 4950 0    60   ~ 0
3V3
Wire Wire Line
	7500 4950 7500 5450
Wire Wire Line
	7500 6050 7500 6200
$Comp
L R_Small R8
U 1 1 58376DAA
P 7700 6200
F 0 "R8" H 7730 6220 50  0000 L CNN
F 1 "10k" H 7730 6160 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 7700 6200 50  0001 C CNN
F 3 "" H 7700 6200 50  0000 C CNN
F 4 "RT0805DRD0712K1L" H 7700 6200 60  0001 C CNN "part_num"
F 5 "low temp coeff." H 7700 6200 60  0001 C CNN "notes"
	1    7700 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 6050 7600 6400
Wire Wire Line
	7600 6400 7700 6400
Wire Wire Line
	7700 6400 7700 6300
Wire Wire Line
	7700 6100 7700 6050
$Comp
L R_Small R7
U 1 1 58377A5D
P 6850 6000
F 0 "R7" H 6880 6020 50  0000 L CNN
F 1 "120k" H 6880 5960 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 6850 6000 50  0001 C CNN
F 3 "" H 6850 6000 50  0000 C CNN
F 4 "RT0805DRD07120KL" H 6850 6000 60  0001 C CNN "part_num"
F 5 "311-2702-1-ND" H 6850 6000 60  0001 C CNN "digikey_num"
	1    6850 6000
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P4
U 1 1 58377B75
P 6650 5550
F 0 "P4" H 6650 5700 50  0000 C CNN
F 1 "RTD" V 6750 5550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 6650 5550 50  0001 C CNN
F 3 "" H 6650 5550 50  0000 C CNN
	1    6650 5550
	-1   0    0    1   
$EndComp
Wire Wire Line
	6850 5300 7500 5300
Wire Wire Line
	6850 5300 6850 5500
Connection ~ 7500 5300
Wire Wire Line
	6850 5600 6850 5900
Wire Wire Line
	6850 6100 6850 6150
Wire Wire Line
	6850 6150 7500 6150
Connection ~ 7500 6150
Wire Wire Line
	6850 5650 7300 5650
Connection ~ 6850 5650
Text Label 2700 6750 2    60   ~ 0
A0
Text Label 7900 5750 0    60   ~ 0
A0
Text Label 10300 6000 0    60   ~ 0
U6gain
Text Label 7700 6100 0    60   ~ 0
U8gain
Text Label 6900 5650 0    60   ~ 0
Tbridge
Text Label 7700 6400 0    60   ~ 0
U8gain2
Text Label 10200 6300 0    60   ~ 0
U6gain2
$Comp
L C_Small C12
U 1 1 583D1292
P 10250 650
F 0 "C12" H 10260 720 50  0000 L CNN
F 1 "0.1uF" H 10260 570 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 10250 650 50  0001 C CNN
F 3 "" H 10250 650 50  0000 C CNN
F 4 "08055C104KAT2A" H 10250 650 60  0001 C CNN "part_num"
	1    10250 650 
	1    0    0    -1  
$EndComp
$Comp
L C_Small C11
U 1 1 583D2AE2
P 7850 5150
F 0 "C11" H 7860 5220 50  0000 L CNN
F 1 "0.1uF" H 7860 5070 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 7850 5150 50  0001 C CNN
F 3 "" H 7850 5150 50  0000 C CNN
F 4 "08055C104KAT2A" H 7850 5150 60  0001 C CNN "part_num"
	1    7850 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 5050 7850 5000
Wire Wire Line
	7850 5000 7500 5000
Connection ~ 7500 5000
Wire Wire Line
	7850 5250 7850 5250
$Comp
L C_Small C9
U 1 1 583D3760
P 9250 3650
F 0 "C9" H 9260 3720 50  0000 L CNN
F 1 "0.1uF" H 9260 3570 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9250 3650 50  0001 C CNN
F 3 "" H 9250 3650 50  0000 C CNN
F 4 "08055C104KAT2A" H 9250 3650 60  0001 C CNN "part_num"
	1    9250 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 3750 9250 3800
Wire Wire Line
	9250 3550 9250 3500
Wire Wire Line
	9250 3500 9000 3500
Connection ~ 9000 3500
$Comp
L C_Small C10
U 1 1 583D3F2E
P 10300 5000
F 0 "C10" H 10310 5070 50  0000 L CNN
F 1 "0.1uF" H 10310 4920 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 10300 5000 50  0001 C CNN
F 3 "" H 10300 5000 50  0000 C CNN
F 4 "08055C104KAT2A" H 10300 5000 60  0001 C CNN "part_num"
	1    10300 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	10100 4900 10300 4900
Connection ~ 10100 4900
$Comp
L C_Small C13
U 1 1 590ED9D6
P 1900 1500
F 0 "C13" H 1910 1570 50  0000 L CNN
F 1 "10uF" H 1910 1420 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1900 1500 50  0001 C CNN
F 3 "" H 1900 1500 50  0000 C CNN
F 4 "GRM21BR61E106KA73L" H 1900 1500 60  0001 C CNN "part_num"
F 5 "490-5523-1-ND" H 1900 1500 60  0001 C CNN "digikey_num"
F 6 "switched to higher c" H 1900 1500 60  0001 C CNN "notes"
	1    1900 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 1100 1900 1400
Connection ~ 1900 1200
Wire Wire Line
	1900 1750 1900 1600
Connection ~ 1900 1750
$Comp
L OPA2350 U5
U 1 1 590F2780
P 7050 3350
F 0 "U5" H 7050 3500 60  0000 L CNN
F 1 "MCP6072" H 7050 3200 60  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 7050 3350 60  0001 C CNN
F 3 "http://www.microchip.com/mymicrochip/filehandler.aspx?ddocname=en540513" H 7050 3350 60  0001 C CNN
F 4 "MCP6072-E/SN" H 7050 3350 60  0001 C CNN "part_num"
F 5 "MCP6072-E/SN-ND" H 7050 3350 60  0001 C CNN "digikey"
	1    7050 3350
	1    0    0    -1  
$EndComp
$Comp
L TR_CT T1
U 1 1 590F33E1
P 8250 2700
F 0 "T1" H 8250 2950 50  0000 C CNN
F 1 "1:1 pulse xfmr" H 8250 2400 35  0000 C CNN
F 2 "my_footprints:6pin_pulse_transformer" H 8250 2700 50  0001 C CNN
F 3 "http://www.murata-ps.com/data/magnetics/kmp_78253j.pdf" H 8250 2700 50  0001 C CNN
F 4 "78253/55JC" H 8250 2700 60  0001 C CNN "part_num"
F 5 "811-2415-ND" H 8250 2700 60  0001 C CNN "digikey_num"
F 6 "footprint probably not right" H 8250 2700 60  0001 C CNN "notes"
	1    8250 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 2350 7850 2500
Wire Wire Line
	7850 3350 7850 2900
Wire Wire Line
	7850 2100 7850 2150
$Comp
L R_Small R14
U 1 1 590F426C
P 8850 2500
F 0 "R14" H 8880 2520 50  0000 L CNN
F 1 "10" H 8880 2460 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 8850 2500 50  0001 C CNN
F 3 "" H 8850 2500 50  0000 C CNN
F 4 "RT0805DRD0710RL" H 8850 2500 60  0001 C CNN "part_num"
F 5 "that's a 25ppm/C 470" H 8850 2500 60  0001 C CNN "notes"
F 6 "311-2700-1-ND" H 8850 2500 60  0001 C CNN "digikey_num"
	1    8850 2500
	0    1    1    0   
$EndComp
Wire Wire Line
	8650 2500 8750 2500
Wire Wire Line
	8950 2500 9150 2500
Wire Wire Line
	9150 2600 9050 2600
Wire Wire Line
	9050 2600 9050 2900
Wire Wire Line
	9050 2900 8650 2900
$Comp
L AD623 U7
U 1 1 590F4FE5
P 10050 1450
F 0 "U7" H 10250 1600 50  0000 L CNN
F 1 "AD623" H 10250 1300 50  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 10050 1450 50  0001 C CNN
F 3 "" H 10050 1450 50  0000 C CNN
F 4 "AD623ARZ" H 10050 1450 60  0001 C CNN "part_num"
F 5 "AD623ARZ-ND" H 10050 1450 60  0001 C CNN "digikey_num"
	1    10050 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 1350 8700 2500
Wire Wire Line
	8700 1350 9050 1350
Connection ~ 8700 2500
Wire Wire Line
	9000 1550 9000 2500
Wire Wire Line
	9000 1550 9050 1550
Connection ~ 9000 2500
Wire Wire Line
	9250 1350 9750 1350
Wire Wire Line
	9250 1550 9750 1550
Wire Wire Line
	9950 1750 9950 1950
Wire Wire Line
	9350 1300 9350 1350
Connection ~ 9350 1350
Wire Wire Line
	9600 1300 9600 1550
Connection ~ 9600 1550
Connection ~ 9950 1900
$Comp
L R_Small R19
U 1 1 590F7951
P 10250 1800
F 0 "R19" H 10280 1820 50  0000 L CNN
F 1 "10k" H 10280 1760 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 10250 1800 50  0001 C CNN
F 3 "" H 10250 1800 50  0000 C CNN
F 4 "RT0805DRD0712K1L" H 10250 1800 60  0001 C CNN "part_num"
F 5 "low temp coeff." H 10250 1800 60  0001 C CNN "notes"
	1    10250 1800
	0    1    1    0   
$EndComp
Wire Wire Line
	10150 1800 10150 1750
Wire Wire Line
	10050 1750 10050 1900
Wire Wire Line
	10050 1900 10400 1900
Wire Wire Line
	10400 1900 10400 1800
Wire Wire Line
	10400 1800 10350 1800
Text Label 7800 950  2    60   ~ 0
Half3v3
Wire Wire Line
	10050 1050 10050 1150
Wire Wire Line
	10600 1450 10350 1450
Wire Wire Line
	8600 1050 10600 1050
Connection ~ 10050 1050
Wire Wire Line
	10250 800  10250 750 
Wire Wire Line
	9350 1100 9350 1050
Wire Wire Line
	9600 1100 9600 1050
Connection ~ 9600 1050
Connection ~ 9350 1050
Wire Wire Line
	9950 500  9950 1150
Wire Wire Line
	10250 550  9950 550 
Connection ~ 9950 550 
Text Notes 10450 1850 0    60   ~ 0
Gain approx 10
Wire Wire Line
	5200 2500 5200 2400
Wire Wire Line
	9350 5200 9350 5150
Connection ~ 9350 5150
Wire Wire Line
	9600 4200 9600 5200
Connection ~ 9600 5150
Wire Wire Line
	9250 5600 9900 5600
Wire Wire Line
	9250 5800 9900 5800
Wire Wire Line
	9350 5400 9350 5800
Connection ~ 9350 5800
Wire Wire Line
	9600 5400 9600 5600
Connection ~ 9600 5600
Wire Wire Line
	9900 5350 10650 5350
Wire Wire Line
	10200 5350 10200 5400
Wire Wire Line
	9900 5150 9900 5350
Connection ~ 10200 5350
Wire Wire Line
	8850 5800 9050 5800
Wire Wire Line
	9050 5600 8950 5600
Wire Wire Line
	8950 5600 8950 5700
Wire Wire Line
	8950 5700 8850 5700
Wire Wire Line
	8650 4100 8750 4100
Wire Wire Line
	9350 5150 9900 5150
Wire Wire Line
	9450 4200 9600 4200
Connection ~ 9600 4800
Wire Wire Line
	7150 5850 7300 5850
$Comp
L R_Small R13
U 1 1 59114414
P 7150 6000
F 0 "R13" H 7180 6020 50  0000 L CNN
F 1 "120k" H 7180 5960 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 7150 6000 50  0001 C CNN
F 3 "" H 7150 6000 50  0000 C CNN
F 4 "RT0805DRD07120KL" H 7150 6000 60  0001 C CNN "part_num"
F 5 "311-2702-1-ND" H 7150 6000 60  0001 C CNN "digikey_num"
	1    7150 6000
	1    0    0    -1  
$EndComp
$Comp
L R_Small R12
U 1 1 591144C3
P 7150 5450
F 0 "R12" H 7180 5470 50  0000 L CNN
F 1 "120k" H 7180 5410 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 7150 5450 50  0001 C CNN
F 3 "" H 7150 5450 50  0000 C CNN
F 4 "RT0805DRD07120KL" H 7150 5450 60  0001 C CNN "part_num"
F 5 "311-2702-1-ND" H 7150 5450 60  0001 C CNN "digikey_num"
	1    7150 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 5300 7150 5350
Connection ~ 7150 5300
Wire Wire Line
	7150 5550 7150 5900
Wire Wire Line
	7150 6100 7150 6150
Connection ~ 7150 6150
Connection ~ 7150 5850
Wire Wire Line
	7600 5450 8250 5450
Wire Wire Line
	9600 4800 8250 4800
Wire Wire Line
	8250 4800 8250 5450
$Comp
L OPA2350 U4
U 2 1 5911595D
P 9100 4200
F 0 "U4" H 9100 4350 60  0000 L CNN
F 1 "MCP6072" H 9100 4050 60  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 9100 4200 60  0001 C CNN
F 3 "http://www.microchip.com/mymicrochip/filehandler.aspx?ddocname=en540513" H 9100 4200 60  0001 C CNN
F 4 "MCP6072-E/SN" H 9100 4200 60  0001 C CNN "part_num"
F 5 "MCP6072-E/SN-ND" H 9100 4200 60  0001 C CNN "digikey"
	2    9100 4200
	1    0    0    -1  
$EndComp
$Comp
L OPA2350 U4
U 1 1 59115D73
P 8250 1050
F 0 "U4" H 8250 1200 60  0000 L CNN
F 1 "MCP6072" H 8250 900 60  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 8250 1050 60  0001 C CNN
F 3 "http://www.microchip.com/mymicrochip/filehandler.aspx?ddocname=en540513" H 8250 1050 60  0001 C CNN
F 4 "MCP6072-E/SN" H 8250 1050 60  0001 C CNN "part_num"
F 5 "MCP6072-E/SN-ND" H 8250 1050 60  0001 C CNN "digikey_num"
	1    8250 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 1150 7800 1150
Wire Wire Line
	7800 1150 7800 1400
Wire Wire Line
	7800 1400 8650 1400
Wire Wire Line
	8650 1400 8650 1050
Connection ~ 8650 1050
Wire Wire Line
	8150 1350 8150 1450
Text Label 8150 650  0    60   ~ 0
3V3
Wire Wire Line
	8150 650  8150 750 
Wire Wire Line
	7800 950  7900 950 
$Comp
L R_Small R10
U 1 1 59119EB9
P 4550 2600
F 0 "R10" H 4580 2620 50  0000 L CNN
F 1 "50k" H 4580 2560 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 4550 2600 50  0001 C CNN
F 3 "" H 4550 2600 50  0000 C CNN
F 4 "RT0805DRE0722K1L" H 4550 2600 60  0001 C CNN "part_num"
F 5 "subbing a 12k low T coeff." H 4550 2600 60  0001 C CNN "notes"
F 6 "311-2825-1-ND" H 4550 2600 60  0001 C CNN "digikey_num"
	1    4550 2600
	-1   0    0    1   
$EndComp
$Comp
L R_Small R11
U 1 1 59119F93
P 4550 2250
F 0 "R11" H 4580 2270 50  0000 L CNN
F 1 "120k" H 4580 2210 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 4550 2250 50  0001 C CNN
F 3 "" H 4550 2250 50  0000 C CNN
F 4 "RT0805DRE0722K1L" H 4550 2250 60  0001 C CNN "part_num"
F 5 "subbing a 12k low T coeff." H 4550 2250 60  0001 C CNN "notes"
F 6 "311-2825-1-ND" H 4550 2250 60  0001 C CNN "digikey_num"
	1    4550 2250
	-1   0    0    1   
$EndComp
Text Label 4550 2050 0    60   ~ 0
3V7
Wire Wire Line
	4550 2700 4550 2800
$Comp
L C_Small C6
U 1 1 591898E1
P 3650 1500
F 0 "C6" H 3660 1570 50  0000 L CNN
F 1 "1uF" H 3660 1420 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3650 1500 50  0001 C CNN
F 3 "" H 3650 1500 50  0000 C CNN
F 4 "GRM21BR61E106KA73L" H 3650 1500 60  0001 C CNN "part_num"
F 5 "490-5523-1-ND" H 3650 1500 60  0001 C CNN "digikey_num"
F 6 "switched to higher c" H 3650 1500 60  0001 C CNN "notes"
	1    3650 1500
	1    0    0    -1  
$EndComp
$Comp
L C_Small C14
U 1 1 5918A868
P 9150 1350
F 0 "C14" H 9160 1420 50  0000 L CNN
F 1 "10uF" H 9160 1270 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9150 1350 50  0001 C CNN
F 3 "" H 9150 1350 50  0000 C CNN
F 4 "GRM21BR61E106KA73L" H 9150 1350 60  0001 C CNN "part_num"
F 5 "490-5523-1-ND" H 9150 1350 60  0001 C CNN "digikey_num"
F 6 "switched to higher c" H 9150 1350 60  0001 C CNN "notes"
	1    9150 1350
	0    1    1    0   
$EndComp
$Comp
L C_Small C15
U 1 1 5918AA36
P 9150 1550
F 0 "C15" H 9160 1620 50  0000 L CNN
F 1 "10uF" H 9160 1470 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9150 1550 50  0001 C CNN
F 3 "" H 9150 1550 50  0000 C CNN
F 4 "GRM21BR61E106KA73L" H 9150 1550 60  0001 C CNN "part_num"
F 5 "490-5523-1-ND" H 9150 1550 60  0001 C CNN "digikey_num"
F 6 "switched to higher c" H 9150 1550 60  0001 C CNN "notes"
	1    9150 1550
	0    1    1    0   
$EndComp
Wire Wire Line
	6950 3700 6950 3650
Text Label 5550 2050 0    60   ~ 0
3V3
Text Label 6600 3250 2    60   ~ 0
Half3v3
Wire Wire Line
	6600 3250 6700 3250
$Comp
L C_Small C16
U 1 1 5918DE48
P 9150 5600
F 0 "C16" V 9200 5450 50  0000 L CNN
F 1 "10uF" V 9050 5500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9150 5600 50  0001 C CNN
F 3 "" H 9150 5600 50  0000 C CNN
F 4 "GRM21BR61E106KA73L" H 9150 5600 60  0001 C CNN "part_num"
F 5 "490-5523-1-ND" H 9150 5600 60  0001 C CNN "digikey_num"
F 6 "switched to higher c" H 9150 5600 60  0001 C CNN "notes"
	1    9150 5600
	0    1    1    0   
$EndComp
$Comp
L C_Small C17
U 1 1 5918E663
P 9150 5800
F 0 "C17" V 9100 5650 50  0000 L CNN
F 1 "10uF" V 9250 5700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9150 5800 50  0001 C CNN
F 3 "" H 9150 5800 50  0000 C CNN
F 4 "GRM21BR61E106KA73L" H 9150 5800 60  0001 C CNN "part_num"
F 5 "490-5523-1-ND" H 9150 5800 60  0001 C CNN "digikey_num"
F 6 "switched to higher c" H 9150 5800 60  0001 C CNN "notes"
	1    9150 5800
	0    1    1    0   
$EndComp
$Comp
L R_Small R16
U 1 1 59191765
P 9350 5300
F 0 "R16" H 9380 5320 50  0000 L CNN
F 1 "120k" H 9380 5260 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 9350 5300 50  0001 C CNN
F 3 "" H 9350 5300 50  0000 C CNN
F 4 "RT0805DRD07120KL" H 9350 5300 60  0001 C CNN "part_num"
F 5 "311-2702-1-ND" H 9350 5300 60  0001 C CNN "digikey_num"
	1    9350 5300
	1    0    0    -1  
$EndComp
$Comp
L R_Small R18
U 1 1 59191839
P 9600 5300
F 0 "R18" H 9630 5320 50  0000 L CNN
F 1 "120k" H 9630 5260 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 9600 5300 50  0001 C CNN
F 3 "" H 9600 5300 50  0000 C CNN
F 4 "RT0805DRD07120KL" H 9600 5300 60  0001 C CNN "part_num"
F 5 "311-2702-1-ND" H 9600 5300 60  0001 C CNN "digikey_num"
	1    9600 5300
	1    0    0    -1  
$EndComp
$Comp
L R_Small R15
U 1 1 59191DC5
P 9350 1200
F 0 "R15" H 9380 1220 50  0000 L CNN
F 1 "120k" H 9380 1160 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 9350 1200 50  0001 C CNN
F 3 "" H 9350 1200 50  0000 C CNN
F 4 "RT0805DRD07120KL" H 9350 1200 60  0001 C CNN "part_num"
F 5 "311-2702-1-ND" H 9350 1200 60  0001 C CNN "digikey_num"
	1    9350 1200
	1    0    0    -1  
$EndComp
$Comp
L R_Small R17
U 1 1 5919263A
P 9600 1200
F 0 "R17" H 9630 1220 50  0000 L CNN
F 1 "120k" H 9630 1160 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 9600 1200 50  0001 C CNN
F 3 "" H 9600 1200 50  0000 C CNN
F 4 "RT0805DRD07120KL" H 9600 1200 60  0001 C CNN "part_num"
F 5 "311-2702-1-ND" H 9600 1200 60  0001 C CNN "digikey_num"
	1    9600 1200
	1    0    0    -1  
$EndComp
NoConn ~ 7850 2700
NoConn ~ 8650 2700
Wire Wire Line
	2850 4750 2400 4750
Wire Wire Line
	2400 4750 2400 4850
Wire Wire Line
	4550 2050 4550 2150
Wire Wire Line
	4550 2350 4550 2500
Wire Wire Line
	4250 2400 4750 2400
Connection ~ 4550 2400
Text Label 4750 2400 0    60   ~ 0
A1
Text Label 5050 6850 0    60   ~ 0
3V7
Wire Wire Line
	2350 6450 2350 6600
Wire Wire Line
	2150 6250 2150 6600
Connection ~ 2150 6450
Connection ~ 2350 6450
NoConn ~ 4850 5150
NoConn ~ 4850 5250
NoConn ~ 4850 5350
NoConn ~ 4850 5450
NoConn ~ 4850 5550
NoConn ~ 4850 5650
NoConn ~ 4850 5750
NoConn ~ 4850 5850
NoConn ~ 4850 5950
NoConn ~ 4850 6050
NoConn ~ 4850 6150
NoConn ~ 4850 6250
NoConn ~ 2850 4850
NoConn ~ 2850 4950
NoConn ~ 2850 5250
NoConn ~ 2850 5350
NoConn ~ 2850 5450
NoConn ~ 2850 5550
NoConn ~ 2850 5650
NoConn ~ 2850 5750
NoConn ~ 2850 6150
NoConn ~ 2850 6250
NoConn ~ 2850 6350
NoConn ~ 2850 6450
NoConn ~ 2850 7150
NoConn ~ 2850 7250
NoConn ~ 2850 7350
NoConn ~ 4850 7350
NoConn ~ 4850 7250
NoConn ~ 4850 7150
NoConn ~ 4850 6950
NoConn ~ 4850 6750
NoConn ~ 4850 6650
Wire Wire Line
	5000 5050 4850 5050
Wire Wire Line
	5000 6350 4850 6350
Wire Wire Line
	4850 6450 5000 6450
Wire Wire Line
	5000 6550 4850 6550
Wire Wire Line
	2850 6550 2700 6550
Text Label 2700 6850 2    60   ~ 0
A1
Wire Wire Line
	2700 6850 2850 6850
Wire Wire Line
	2700 6750 2850 6750
Wire Wire Line
	1900 6450 2350 6450
Text Label 2150 6250 0    60   ~ 0
T3V3
Text Label 5000 7050 0    60   ~ 0
T3V3
Wire Wire Line
	5000 7050 4850 7050
Wire Wire Line
	1900 6450 1900 6850
Wire Wire Line
	1900 6950 2850 6950
Wire Wire Line
	1900 7050 2850 7050
Wire Wire Line
	2250 7150 1900 7150
$Comp
L Micro_SD_Card J1
U 1 1 5922E566
P 2600 3200
F 0 "J1" H 2550 3917 50  0000 C CNN
F 1 "Micro_SD_Card" H 2550 3826 50  0000 C CNN
F 2 "my_footprints:molex5031821852" H 3750 3500 50  0001 C CNN
F 3 "http://www.molex.com/pdm_docs/sd/5031821852_sd.pdf" H 2600 3200 50  0001 C CNN
F 4 "WM12834CT-ND" H 2600 3200 60  0001 C CNN "digikey_num"
F 5 "5031821852" H 2600 3200 60  0001 C CNN "part_num"
	1    2600 3200
	1    0    0    -1  
$EndComp
Text Label 1500 3200 2    60   ~ 0
T3V3
Wire Wire Line
	950  3200 1700 3200
Text Label 2700 6650 2    60   ~ 0
SCK
Wire Wire Line
	2700 6650 2850 6650
Text Label 2700 5950 2    60   ~ 0
DOUT
Text Label 2700 6050 2    60   ~ 0
DIN
Wire Wire Line
	2700 6050 2850 6050
Wire Wire Line
	2850 5950 2700 5950
Text Label 1500 3300 2    60   ~ 0
SCK
Text Label 1500 3100 2    60   ~ 0
DOUT
Text Label 1450 3500 2    60   ~ 0
DIN
$Comp
L GND #PWR04
U 1 1 59230264
P 1200 3600
F 0 "#PWR04" H 1200 3350 50  0001 C CNN
F 1 "GND" H 1200 3450 50  0000 C CNN
F 2 "" H 1200 3600 50  0000 C CNN
F 3 "" H 1200 3600 50  0000 C CNN
	1    1200 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 3100 1700 3100
Wire Wire Line
	1700 3300 1500 3300
Wire Wire Line
	1450 3500 1700 3500
Wire Wire Line
	1700 3400 1200 3400
Wire Wire Line
	1200 3400 1200 3600
Text Label 1500 3000 2    60   ~ 0
CS
Wire Wire Line
	1500 3000 1700 3000
NoConn ~ 1700 2900
NoConn ~ 1700 3600
Wire Wire Line
	3400 3800 3400 3900
Text Label 2700 5850 2    60   ~ 0
CS
$Comp
L LD3985M33R U1
U 1 1 5929A63B
P 2600 1250
F 0 "U1" H 2600 1567 50  0000 C CNN
F 1 "LD3985M33R" H 2600 1476 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-5" H 2600 1350 50  0001 C CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/group2/96/39/79/10/c9/1a/48/92/CD00003395/files/CD00003395.pdf/jcr:content/translations/en.CD00003395.pdf" H 2600 1250 50  0001 C CNN
F 4 "497-3504-1-ND" H 2600 1250 60  0001 C CNN "digikey_num"
F 5 "LD3985M33R" H 2600 1250 60  0001 C CNN "part_num"
	1    2600 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 1550 2600 1750
Connection ~ 2600 1750
$Comp
L C_Small C1
U 1 1 5929AF3F
P 3050 1550
F 0 "C1" H 3060 1620 50  0000 L CNN
F 1 "0.01uF" H 3060 1470 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3050 1550 50  0001 C CNN
F 3 "" H 3050 1550 50  0000 C CNN
F 4 "CC0805KRX7R9BB103" H 3050 1550 60  0001 C CNN "part_num"
F 5 "311-1136-1-ND" H 3050 1550 60  0001 C CNN "digikey_num"
F 6 "keep low for fast startup. 10nF rec." H 3050 1550 60  0001 C CNN "notes"
	1    3050 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 1350 3050 1450
Wire Wire Line
	3050 1750 3050 1650
Connection ~ 3050 1750
Text Label 2050 950  2    60   ~ 0
EN_3V3
Wire Wire Line
	2050 950  2100 950 
Wire Wire Line
	2100 950  2100 1350
Wire Wire Line
	2100 1350 2150 1350
Text Label 2700 5150 2    60   ~ 0
EN_3V3
Wire Wire Line
	2700 5150 2850 5150
$Comp
L C_Small C2
U 1 1 592CD7BC
P 4250 2600
F 0 "C2" H 4260 2670 50  0000 L CNN
F 1 "0.1uF" H 4260 2520 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4250 2600 50  0001 C CNN
F 3 "" H 4250 2600 50  0000 C CNN
F 4 "08055C104KAT2A" H 4250 2600 60  0001 C CNN "part_num"
F 5 "478-1395-1-ND" H 4250 2600 60  0001 C CNN "digikey_num"
	1    4250 2600
	-1   0    0    1   
$EndComp
Wire Wire Line
	4250 2500 4250 2400
Wire Wire Line
	4550 2750 4250 2750
Wire Wire Line
	4250 2750 4250 2700
Connection ~ 4550 2750
$Comp
L Teensy3.2 U2
U 1 1 592CFA05
P 3850 6050
F 0 "U2" H 3850 7637 60  0000 C CNN
F 1 "Teensy3.2" H 3850 7531 60  0000 C CNN
F 2 "my_footprints:Teensy32_for_logger" H 3850 5250 60  0001 C CNN
F 3 "" H 3850 5250 60  0000 C CNN
	1    3850 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 2450 6950 2400
Wire Wire Line
	10100 6000 10100 6350
$Comp
L Battery_Cell-RESCUE-logger_01 BT1
U 1 1 592DB096
P 1400 1550
F 0 "BT1" H 1518 1646 50  0000 L CNN
F 1 "Batt3.7V" H 1518 1555 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" V 1400 1610 50  0001 C CNN
F 3 "" V 1400 1610 50  0001 C CNN
	1    1400 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 1200 1400 1350
Wire Wire Line
	1400 1750 1400 1650
Wire Wire Line
	4850 6850 5050 6850
Wire Wire Line
	2850 5850 2700 5850
NoConn ~ 2850 5050
$Comp
L C_Small C3
U 1 1 5938B421
P 950 3350
F 0 "C3" H 960 3420 50  0000 L CNN
F 1 "10uF" H 960 3270 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 950 3350 50  0001 C CNN
F 3 "" H 950 3350 50  0000 C CNN
F 4 "GRM21BR61E106KA73L" H 950 3350 60  0001 C CNN "part_num"
F 5 "490-5523-1-ND" H 950 3350 60  0001 C CNN "digikey_num"
F 6 "switched to higher c" H 950 3350 60  0001 C CNN "notes"
	1    950  3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  3250 950  3200
Wire Wire Line
	1200 3550 950  3550
Wire Wire Line
	950  3550 950  3450
Connection ~ 1200 3550
$Comp
L C_Small C4
U 1 1 5938CE71
P 8150 5600
F 0 "C4" H 8160 5670 50  0000 L CNN
F 1 "0.1uF" H 8160 5520 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 8150 5600 50  0001 C CNN
F 3 "" H 8150 5600 50  0000 C CNN
F 4 "08055C104KAT2A" H 8150 5600 60  0001 C CNN "part_num"
	1    8150 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 5750 8150 5700
Wire Wire Line
	8150 5500 8150 5450
Connection ~ 8150 5450
$Comp
L GNDA #PWR05
U 1 1 5938F32F
P 3650 1800
F 0 "#PWR05" H 3650 1550 50  0001 C CNN
F 1 "GNDA" H 3655 1627 50  0000 C CNN
F 2 "" H 3650 1800 50  0001 C CNN
F 3 "" H 3650 1800 50  0001 C CNN
	1    3650 1800
	1    0    0    -1  
$EndComp
$Comp
L L_Small L1
U 1 1 5938F4F5
P 3400 1750
F 0 "L1" V 3222 1750 50  0000 C CNN
F 1 "Wire" V 3313 1750 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 3400 1750 50  0001 C CNN
F 3 "" H 3400 1750 50  0001 C CNN
	1    3400 1750
	0    1    1    0   
$EndComp
Connection ~ 3650 1750
$Comp
L GNDA #PWR06
U 1 1 5939026B
P 4550 2800
F 0 "#PWR06" H 4550 2550 50  0001 C CNN
F 1 "GNDA" H 4555 2627 50  0000 C CNN
F 2 "" H 4550 2800 50  0001 C CNN
F 3 "" H 4550 2800 50  0001 C CNN
	1    4550 2800
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR07
U 1 1 593902DC
P 5550 2850
F 0 "#PWR07" H 5550 2600 50  0001 C CNN
F 1 "GNDA" H 5555 2677 50  0000 C CNN
F 2 "" H 5550 2850 50  0001 C CNN
F 3 "" H 5550 2850 50  0001 C CNN
	1    5550 2850
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR08
U 1 1 59391F95
P 6950 2450
F 0 "#PWR08" H 6950 2200 50  0001 C CNN
F 1 "GNDA" H 6955 2277 50  0000 C CNN
F 2 "" H 6950 2450 50  0001 C CNN
F 3 "" H 6950 2450 50  0001 C CNN
	1    6950 2450
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR09
U 1 1 59392275
P 6950 3700
F 0 "#PWR09" H 6950 3450 50  0001 C CNN
F 1 "GNDA" H 6955 3527 50  0000 C CNN
F 2 "" H 6950 3700 50  0001 C CNN
F 3 "" H 6950 3700 50  0001 C CNN
	1    6950 3700
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR010
U 1 1 59392658
P 8150 1450
F 0 "#PWR010" H 8150 1200 50  0001 C CNN
F 1 "GNDA" H 8155 1277 50  0000 C CNN
F 2 "" H 8150 1450 50  0001 C CNN
F 3 "" H 8150 1450 50  0001 C CNN
	1    8150 1450
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR011
U 1 1 593939A7
P 9950 1950
F 0 "#PWR011" H 9950 1700 50  0001 C CNN
F 1 "GNDA" H 9955 1777 50  0000 C CNN
F 2 "" H 9950 1950 50  0001 C CNN
F 3 "" H 9950 1950 50  0001 C CNN
	1    9950 1950
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR012
U 1 1 59393B23
P 10250 800
F 0 "#PWR012" H 10250 550 50  0001 C CNN
F 1 "GNDA" H 10255 627 50  0000 C CNN
F 2 "" H 10250 800 50  0001 C CNN
F 3 "" H 10250 800 50  0001 C CNN
	1    10250 800 
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR013
U 1 1 59396D03
P 9000 4550
F 0 "#PWR013" H 9000 4300 50  0001 C CNN
F 1 "GNDA" H 9005 4377 50  0000 C CNN
F 2 "" H 9000 4550 50  0001 C CNN
F 3 "" H 9000 4550 50  0001 C CNN
	1    9000 4550
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR014
U 1 1 59396DA8
P 9250 3800
F 0 "#PWR014" H 9250 3550 50  0001 C CNN
F 1 "GNDA" H 9255 3627 50  0000 C CNN
F 2 "" H 9250 3800 50  0001 C CNN
F 3 "" H 9250 3800 50  0001 C CNN
	1    9250 3800
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR015
U 1 1 593974AD
P 7500 6200
F 0 "#PWR015" H 7500 5950 50  0001 C CNN
F 1 "GNDA" H 7505 6027 50  0000 C CNN
F 2 "" H 7500 6200 50  0001 C CNN
F 3 "" H 7500 6200 50  0001 C CNN
	1    7500 6200
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR016
U 1 1 5939751E
P 8150 5750
F 0 "#PWR016" H 8150 5500 50  0001 C CNN
F 1 "GNDA" H 8155 5577 50  0000 C CNN
F 2 "" H 8150 5750 50  0001 C CNN
F 3 "" H 8150 5750 50  0001 C CNN
	1    8150 5750
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR017
U 1 1 5939769A
P 7850 5300
F 0 "#PWR017" H 7850 5050 50  0001 C CNN
F 1 "GNDA" H 7855 5127 50  0000 C CNN
F 2 "" H 7850 5300 50  0001 C CNN
F 3 "" H 7850 5300 50  0001 C CNN
	1    7850 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 5250 7850 5300
$Comp
L GNDA #PWR018
U 1 1 59398B18
P 10100 6350
F 0 "#PWR018" H 10100 6100 50  0001 C CNN
F 1 "GNDA" H 10105 6177 50  0000 C CNN
F 2 "" H 10100 6350 50  0001 C CNN
F 3 "" H 10100 6350 50  0001 C CNN
	1    10100 6350
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR019
U 1 1 59398BBD
P 10300 5150
F 0 "#PWR019" H 10300 4900 50  0001 C CNN
F 1 "GNDA" H 10305 4977 50  0000 C CNN
F 2 "" H 10300 5150 50  0001 C CNN
F 3 "" H 10300 5150 50  0001 C CNN
	1    10300 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 5150 10300 5100
Text Notes 2700 1900 0    31   ~ 0
DRC fails b/c it does not see this\nas driving power for GNDA.
Wire Wire Line
	3650 1750 3500 1750
$Comp
L GNDA #PWR020
U 1 1 593A60A1
P 7350 3150
F 0 "#PWR020" H 7350 2900 50  0001 C CNN
F 1 "GNDA" H 7355 2977 50  0000 C CNN
F 2 "" H 7350 3150 50  0001 C CNN
F 3 "" H 7350 3150 50  0001 C CNN
	1    7350 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 3150 7350 3100
$Comp
L GNDA #PWR021
U 1 1 593A6B26
P 3400 3900
F 0 "#PWR021" H 3400 3650 50  0001 C CNN
F 1 "GNDA" H 3405 3727 50  0000 C CNN
F 2 "" H 3400 3900 50  0001 C CNN
F 3 "" H 3400 3900 50  0001 C CNN
	1    3400 3900
	1    0    0    -1  
$EndComp
Text Notes 700  750  0    60   ~ 0
Add BH123 holder for BOM\nAdd crystal for teensy RTC
$Comp
L Crystal Y?
U 1 1 594403CE
P 1150 4850
F 0 "Y?" H 1150 5118 50  0000 C CNN
F 1 "Crystal" H 1150 5027 50  0000 C CNN
F 2 "" H 1150 4850 50  0001 C CNN
F 3 "" H 1150 4850 50  0001 C CNN
F 4 "CFS-20632768HZFB" H 1150 4850 60  0001 C CNN "part_num"
F 5 "300-8763-ND" H 1150 4850 60  0001 C CNN "digikey_num"
	1    1150 4850
	1    0    0    -1  
$EndComp
$EndSCHEMATC