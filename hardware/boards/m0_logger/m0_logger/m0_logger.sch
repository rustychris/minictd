EESchema Schematic File Version 4
LIBS:m0_logger-cache
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
$Comp
L m0_logger:itsy_m0 U3
U 1 1 5BFAFF92
P 5750 1950
F 0 "U3" H 5900 2698 50  0000 C CNN
F 1 "itsy_m0" H 5900 2607 50  0000 C CNN
F 2 "m0_logger_footprints:ItsyBitsyM0" H 5750 1950 50  0001 C CNN
F 3 "" H 5750 1950 50  0001 C CNN
F 4 "1528-2554-ND" H 5750 1950 50  0001 C CNN "Vendor"
	1    5750 1950
	1    0    0    -1  
$EndComp
$Comp
L Connector:Micro_SD_Card J5
U 1 1 5BFB0129
P 9400 1600
F 0 "J5" H 9350 2317 50  0000 C CNN
F 1 "Micro_SD_Card" H 9350 2226 50  0000 C CNN
F 2 "Connector_Card:microSD_HC_Hirose_DM3AT-SF-PEJM5" H 10550 1900 50  0001 C CNN
F 3 "http://katalog.we-online.de/em/datasheet/693072010801.pdf" H 9400 1600 50  0001 C CNN
F 4 "HR1964CT-ND" H 9400 1600 50  0001 C CNN "Vendor"
	1    9400 1600
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 5BFB041F
P 1450 5950
F 0 "J2" H 1369 5525 50  0000 C CNN
F 1 "PressureA" H 1369 5616 50  0000 C CNN
F 2 "Connector_Molex:Molex_PicoBlade_53047-0810_1x04_P1.25mm_Vertical" H 1450 5950 50  0001 C CNN
F 3 "~" H 1450 5950 50  0001 C CNN
F 4 "609-3333-ND" H 1450 5950 50  0001 C CNN "Vendor"
	1    1450 5950
	1    0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J3
U 1 1 5BFB04EB
P 7200 3000
F 0 "J3" H 7279 2992 50  0000 L CNN
F 1 "GPS" H 7279 2901 50  0000 L CNN
F 2 "Connector_Molex:Molex_PicoBlade_53047-0810_1x04_P1.25mm_Vertical" H 7200 3000 50  0001 C CNN
F 3 "~" H 7200 3000 50  0001 C CNN
F 4 "609-3333-ND" H 7200 3000 50  0001 C CNN "Vendor"
	1    7200 3000
	1    0    0    -1  
$EndComp
$Comp
L m0_logger:af_bno055 U2
U 1 1 5BFB08F3
P 2650 3750
F 0 "U2" H 2550 4225 50  0000 C CNN
F 1 "af_bno055" H 2550 4134 50  0000 C CNN
F 2 "m0_logger_footprints:afBNO055" H 2650 3750 50  0001 C CNN
F 3 "" H 2650 3750 50  0001 C CNN
F 4 "1528-1426-ND" H 2650 3750 50  0001 C CNN "Vendor"
F 5 "Also available direct from adafruit" H 2650 3750 50  0001 C CNN "Notes"
	1    2650 3750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J4
U 1 1 5BFB09EF
P 1150 5200
F 0 "J4" H 1070 4775 50  0000 C CNN
F 1 "PressureB" H 1070 4866 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 1150 5200 50  0001 C CNN
F 3 "~" H 1150 5200 50  0001 C CNN
F 4 "609-3255-ND" H 1150 5200 50  0001 C CNN "Vendor"
	1    1150 5200
	-1   0    0    1   
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5BFB0B22
P 2000 5450
F 0 "C1" H 2092 5496 50  0000 L CNN
F 1 "0.1uF" H 2050 5400 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2000 5450 50  0001 C CNN
F 3 "~" H 2000 5450 50  0001 C CNN
F 4 "478-1395-1-ND" H 2000 5450 50  0001 C CNN "Vendor"
	1    2000 5450
	1    0    0    -1  
$EndComp
$Comp
L Timer_RTC:DS3231M U4
U 1 1 5BFB0E34
P 9450 3200
F 0 "U4" H 9300 3150 50  0000 C CNN
F 1 "DS3231S" H 9600 3250 50  0000 C CNN
F 2 "Package_SO:SOIC-16W_7.5x10.3mm_P1.27mm" H 9450 2600 50  0001 C CNN
F 3 "http://datasheets.maximintegrated.com/en/ds/DS3231.pdf" H 9720 3250 50  0001 C CNN
F 4 "DS3231S#-ND" H 9450 3200 50  0001 C CNN "Vendor"
	1    9450 3200
	1    0    0    -1  
$EndComp
Text Notes 9000 3900 0    50   ~ 0
N.B. 5-12 must be grounded
$Comp
L Device:Battery_Cell BT1
U 1 1 5BFB10E0
P 10300 3200
F 0 "BT1" H 10418 3296 50  0000 L CNN
F 1 "Battery_Cell" H 10418 3205 50  0000 L CNN
F 2 "Battery:BatteryHolder_Keystone_500" V 10300 3260 50  0001 C CNN
F 3 "~" V 10300 3260 50  0001 C CNN
F 4 "36-500-ND" H 10300 3200 50  0001 C CNN "Vendor"
	1    10300 3200
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J1
U 1 1 5BFB1453
P 7550 900
F 0 "J1" H 7470 575 50  0000 C CNN
F 1 "LiPO" H 7470 666 50  0000 C CNN
F 2 "Connector_JST:JST_PH_S2B-PH-K_1x02_P2.00mm_Horizontal" H 7550 900 50  0001 C CNN
F 3 "~" H 7550 900 50  0001 C CNN
F 4 "455-1719-ND" H 7550 900 50  0001 C CNN "Vendor"
	1    7550 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 1650 5150 1650
Text Label 5150 1650 0    50   ~ 0
3V
NoConn ~ 5400 1550
NoConn ~ 5400 1750
NoConn ~ 5400 1850
NoConn ~ 6400 1750
Text Label 8150 1600 0    50   ~ 0
3V
$Comp
L power:GND #PWR03
U 1 1 5BFB6112
P 6800 1750
F 0 "#PWR03" H 6800 1500 50  0001 C CNN
F 1 "GND" H 6805 1577 50  0000 C CNN
F 2 "" H 6800 1750 50  0001 C CNN
F 3 "" H 6800 1750 50  0001 C CNN
	1    6800 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 1650 6800 1750
Wire Wire Line
	6400 1650 6800 1650
Wire Wire Line
	8000 1800 8000 1950
$Comp
L power:GND #PWR05
U 1 1 5BFB6193
P 8000 2000
F 0 "#PWR05" H 8000 1750 50  0001 C CNN
F 1 "GND" H 8005 1827 50  0000 C CNN
F 2 "" H 8000 2000 50  0001 C CNN
F 3 "" H 8000 2000 50  0001 C CNN
	1    8000 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5BFB626A
P 9450 3650
F 0 "#PWR07" H 9450 3400 50  0001 C CNN
F 1 "GND" H 9455 3477 50  0000 C CNN
F 2 "" H 9450 3650 50  0001 C CNN
F 3 "" H 9450 3650 50  0001 C CNN
	1    9450 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	9450 3650 9450 3600
Wire Wire Line
	8950 3000 8750 3000
Text Label 8750 3000 0    50   ~ 0
SCL
Wire Wire Line
	8950 3100 8750 3100
Text Label 8750 3100 0    50   ~ 0
SDA
Wire Wire Line
	9450 2800 9450 2700
Wire Wire Line
	9350 2800 9350 2650
Text Label 9350 2550 0    50   ~ 0
3V
Wire Wire Line
	9450 2700 10300 2700
Wire Wire Line
	10300 2700 10300 3000
Wire Wire Line
	10300 3300 10300 3600
Wire Wire Line
	10300 3600 9450 3600
Connection ~ 9450 3600
$Comp
L sensor_head-cache:MS5803 U1
U 1 1 5BFB09B1
P 2800 5150
F 0 "U1" H 2800 5615 50  0000 C CNN
F 1 "MS5803" H 2800 5524 50  0000 C CNN
F 2 "m0_logger_footprints:ms5803_rh" H 1850 5150 50  0001 C CIN
F 3 "" H 1850 5150 50  0000 C CNN
F 4 "223-1625-5-ND" H 2800 5150 50  0001 C CNN "Vendor"
	1    2800 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 5100 3350 5100
Wire Wire Line
	2150 3500 2000 3500
Text Label 2000 3500 0    50   ~ 0
3V
$Comp
L power:GND #PWR02
U 1 1 5BFB88D8
P 1800 3800
F 0 "#PWR02" H 1800 3550 50  0001 C CNN
F 1 "GND" H 1805 3627 50  0000 C CNN
F 2 "" H 1800 3800 50  0001 C CNN
F 3 "" H 1800 3800 50  0001 C CNN
	1    1800 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 3700 1800 3700
Wire Wire Line
	1800 3700 1800 3800
Wire Wire Line
	2150 3900 2000 3900
Text Label 2000 3800 0    50   ~ 0
SDA
Wire Wire Line
	2000 3800 2150 3800
Text Label 2000 3900 0    50   ~ 0
SCL
Wire Wire Line
	10200 2200 10300 2200
Wire Wire Line
	10300 2200 10300 2250
$Comp
L power:GND #PWR08
U 1 1 5BFCCC7B
P 10300 2250
F 0 "#PWR08" H 10300 2000 50  0001 C CNN
F 1 "GND" H 10305 2077 50  0000 C CNN
F 2 "" H 10300 2250 50  0001 C CNN
F 3 "" H 10300 2250 50  0001 C CNN
	1    10300 2250
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5BFCCCBE
P 7800 1750
F 0 "C3" H 7600 1750 50  0000 L CNN
F 1 "47uF" H 7600 1850 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7800 1750 50  0001 C CNN
F 3 "~" H 7800 1750 50  0001 C CNN
F 4 "490-9961-1-ND" H 7800 1750 50  0001 C CNN "Vendor"
	1    7800 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 1650 7800 1600
Wire Wire Line
	7800 1600 8500 1600
Wire Wire Line
	8000 1950 7800 1950
Wire Wire Line
	7800 1950 7800 1850
Connection ~ 8000 1950
Wire Wire Line
	8000 1950 8000 2000
Text Label 8150 1700 0    50   ~ 0
SCK
NoConn ~ 8500 1300
NoConn ~ 8500 2000
Wire Wire Line
	6150 3050 6150 3200
Wire Wire Line
	6150 3200 6400 3200
Text Label 6400 3200 0    50   ~ 0
SD_CS
Text Notes 6100 3400 0    50   ~ 0
Mimic Feather\nM0 logger
Text Label 8150 1400 0    50   ~ 0
SD_CS
Wire Wire Line
	8150 1400 8500 1400
Wire Wire Line
	8150 1700 8500 1700
Text Label 8150 1900 0    50   ~ 0
MISO
Wire Wire Line
	8150 1900 8500 1900
Text Label 8150 1500 0    50   ~ 0
MOSI
Wire Wire Line
	8150 1500 8500 1500
Wire Wire Line
	5400 2650 5150 2650
Text Label 5150 2650 0    50   ~ 0
MOSI
Wire Wire Line
	5400 2750 5150 2750
Text Label 5150 2750 0    50   ~ 0
MISO
$Comp
L Device:C_Small C4
U 1 1 5BFD9EBF
P 9100 2650
F 0 "C4" V 9000 2600 50  0000 L CNN
F 1 "0.1uF" V 9200 2600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9100 2650 50  0001 C CNN
F 3 "~" H 9100 2650 50  0001 C CNN
F 4 "478-1395-1-ND" V 9100 2650 50  0001 C CNN "Vendor"
	1    9100 2650
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5BFD9F3E
P 8900 2700
F 0 "#PWR06" H 8900 2450 50  0001 C CNN
F 1 "GND" H 8905 2527 50  0000 C CNN
F 2 "" H 8900 2700 50  0001 C CNN
F 3 "" H 8900 2700 50  0001 C CNN
	1    8900 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 2700 8900 2650
Wire Wire Line
	8900 2650 9000 2650
Wire Wire Line
	9200 2650 9350 2650
Connection ~ 9350 2650
Wire Wire Line
	9350 2650 9350 2550
Text Label 7050 2550 0    50   ~ 0
SCL
$Comp
L Device:R_US R2
U 1 1 5BFDBF81
P 6850 2350
F 0 "R2" H 6918 2396 50  0000 L CNN
F 1 "5.6k" H 6918 2305 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 6890 2340 50  0001 C CNN
F 3 "~" H 6850 2350 50  0001 C CNN
F 4 "A126380CT-ND" H 6850 2350 50  0001 C CNN "Vendor"
	1    6850 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R1
U 1 1 5BFDC001
P 6700 2350
F 0 "R1" H 6500 2400 50  0000 L CNN
F 1 "5.6k" H 6500 2300 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 6740 2340 50  0001 C CNN
F 3 "~" H 6700 2350 50  0001 C CNN
F 4 "A126380CT-ND" H 6700 2350 50  0001 C CNN "Vendor"
	1    6700 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 2200 6850 2200
Wire Wire Line
	6400 2550 6850 2550
Connection ~ 6850 2200
Wire Wire Line
	6850 2200 7100 2200
Text Label 7100 2200 0    50   ~ 0
3V
Wire Wire Line
	6850 2500 6850 2550
Connection ~ 6850 2550
Wire Wire Line
	6850 2550 7050 2550
Wire Wire Line
	6400 2650 6700 2650
Text Label 7050 2650 0    50   ~ 0
SDA
Wire Wire Line
	6700 2500 6700 2650
Connection ~ 6700 2650
Wire Wire Line
	6700 2650 7050 2650
Wire Wire Line
	2950 4000 3150 4000
Text Label 3150 4000 0    50   ~ 0
3V
NoConn ~ 2350 5000
Wire Wire Line
	2350 5200 2300 5200
Wire Wire Line
	2300 5300 2350 5300
Wire Wire Line
	3250 5200 3300 5200
Wire Wire Line
	1350 5300 1850 5300
Wire Wire Line
	1850 5300 1850 5600
Wire Wire Line
	1850 5600 2000 5600
Wire Wire Line
	3350 5600 3350 5100
Wire Wire Line
	2000 5350 2000 5200
Wire Wire Line
	2000 5550 2000 5600
Connection ~ 2000 5600
Wire Wire Line
	2000 5600 3350 5600
Wire Wire Line
	1350 5200 2000 5200
Wire Wire Line
	2300 5300 2300 5200
Wire Wire Line
	2300 5200 2000 5200
Connection ~ 2300 5200
Connection ~ 2000 5200
Wire Wire Line
	2300 5300 2300 5500
Wire Wire Line
	2300 5500 3300 5500
Wire Wire Line
	3300 5500 3300 5200
Connection ~ 2300 5300
Wire Wire Line
	2350 5100 1350 5100
Wire Wire Line
	3250 5000 3250 4600
Wire Wire Line
	3250 4600 2000 4600
Wire Wire Line
	2000 4600 2000 5000
Wire Wire Line
	2000 5000 1350 5000
Text Label 1550 5000 0    50   ~ 0
P_SCL
Text Label 1550 5100 0    50   ~ 0
P_SDA
Text Label 1550 5200 0    50   ~ 0
P_3V
Text Label 1550 5300 0    50   ~ 0
P_GND
NoConn ~ 2950 3700
NoConn ~ 2950 3800
NoConn ~ 2950 3900
NoConn ~ 2150 3600
NoConn ~ 2150 4000
Wire Wire Line
	1250 6050 1100 6050
Wire Wire Line
	1100 6050 1100 6200
$Comp
L power:GND #PWR01
U 1 1 5C009FA2
P 1100 6200
F 0 "#PWR01" H 1100 5950 50  0001 C CNN
F 1 "GND" H 1105 6027 50  0000 C CNN
F 2 "" H 1100 6200 50  0001 C CNN
F 3 "" H 1100 6200 50  0001 C CNN
	1    1100 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 5950 1000 5950
Text Label 1000 5950 0    50   ~ 0
3V
Text Label 1000 5850 0    50   ~ 0
SDA
Text Label 1000 5750 0    50   ~ 0
SCL
Wire Wire Line
	1000 5750 1250 5750
Wire Wire Line
	1250 5850 1000 5850
Wire Wire Line
	6400 2750 6750 2750
Wire Wire Line
	6750 2750 6750 2900
Wire Wire Line
	6750 2900 7000 2900
Wire Wire Line
	7000 3000 6650 3000
Wire Wire Line
	6650 3000 6650 2850
Wire Wire Line
	6650 2850 6400 2850
Text Label 6800 3100 0    50   ~ 0
3V
Wire Wire Line
	6800 3100 7000 3100
$Comp
L power:GND #PWR04
U 1 1 5C017BC7
P 6850 3250
F 0 "#PWR04" H 6850 3000 50  0001 C CNN
F 1 "GND" H 6855 3077 50  0000 C CNN
F 2 "" H 6850 3250 50  0001 C CNN
F 3 "" H 6850 3250 50  0001 C CNN
	1    6850 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 3200 6850 3200
Wire Wire Line
	6850 3200 6850 3250
Wire Wire Line
	5400 2550 5150 2550
Text Label 5150 2550 0    50   ~ 0
SCK
Connection ~ 6800 1650
Text Notes 7300 750  0    50   ~ 0
3.5 - 6V battery input
$Comp
L Device:C_Small C2
U 1 1 5C0218A0
P 7500 1750
F 0 "C2" H 7300 1750 50  0000 L CNN
F 1 "0.1uF" H 7300 1850 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7500 1750 50  0001 C CNN
F 3 "~" H 7500 1750 50  0001 C CNN
F 4 "478-1395-1-ND" H 7500 1750 50  0001 C CNN "Vendor"
	1    7500 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7500 1650 7500 1600
Wire Wire Line
	7500 1600 7800 1600
Connection ~ 7800 1600
Wire Wire Line
	7800 1950 7500 1950
Wire Wire Line
	7500 1950 7500 1850
Connection ~ 7800 1950
NoConn ~ 9950 3000
NoConn ~ 9950 3300
NoConn ~ 8950 3400
NoConn ~ 5950 3050
NoConn ~ 5850 3050
NoConn ~ 5750 3050
NoConn ~ 6400 2450
NoConn ~ 6400 2350
NoConn ~ 6400 1950
NoConn ~ 6400 1850
NoConn ~ 5400 2450
NoConn ~ 5400 2150
NoConn ~ 5400 1950
Wire Wire Line
	8500 1800 8000 1800
Text Label 6550 2750 0    50   ~ 0
TX
Text Label 6500 2850 0    50   ~ 0
RX
$Comp
L Driver_Motor:DRV8833PWP U5
U 1 1 5C16D181
P 6200 5150
F 0 "U5" H 6200 5600 50  0000 C CNN
F 1 "DRV8833PWP" H 6350 5350 50  0000 C CNN
F 2 "Package_SO:HTSSOP-16-1EP_4.4x5mm_P0.65mm_EP3.4x5mm_Mask2.46x2.31mm_ThermalVias" H 6650 5600 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/drv8833.pdf" H 6050 5700 50  0001 C CNN
F 4 "296-29434-1-ND" H 6200 5150 50  0001 C CNN "Vendor"
	1    6200 5150
	1    0    0    -1  
$EndComp
Text Label 6550 1350 0    50   ~ 0
Vbat
Wire Wire Line
	6200 5850 6100 5850
Wire Wire Line
	6100 5850 6100 6000
Connection ~ 6100 5850
$Comp
L power:GND #PWR010
U 1 1 5C17426E
P 6100 6000
F 0 "#PWR010" H 6100 5750 50  0001 C CNN
F 1 "GND" H 6105 5827 50  0000 C CNN
F 2 "" H 6100 6000 50  0001 C CNN
F 3 "" H 6100 6000 50  0001 C CNN
	1    6100 6000
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C6
U 1 1 5C17444F
P 6900 4400
F 0 "C6" H 6750 4400 50  0000 L CNN
F 1 "2.2uF" H 6650 4300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6900 4400 50  0001 C CNN
F 3 "~" H 6900 4400 50  0001 C CNN
	1    6900 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 4450 6500 4100
Text Label 6500 3950 0    50   ~ 0
Vbat
$Comp
L power:GND #PWR011
U 1 1 5C17D8DD
P 7100 4600
F 0 "#PWR011" H 7100 4350 50  0001 C CNN
F 1 "GND" H 7105 4427 50  0000 C CNN
F 2 "" H 7100 4600 50  0001 C CNN
F 3 "" H 7100 4600 50  0001 C CNN
	1    7100 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 4300 6900 4250
Wire Wire Line
	6900 4250 6400 4250
Wire Wire Line
	6400 4250 6400 4450
$Comp
L Device:C_Small C7
U 1 1 5C186C34
P 7300 4400
F 0 "C7" H 7300 4500 50  0000 L CNN
F 1 "47uF" H 7300 4300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7300 4400 50  0001 C CNN
F 3 "~" H 7300 4400 50  0001 C CNN
	1    7300 4400
	1    0    0    -1  
$EndComp
Connection ~ 6500 4100
Wire Wire Line
	6500 4100 6500 3950
Wire Wire Line
	7300 4100 7300 4300
Wire Wire Line
	6900 4500 6900 4600
Wire Wire Line
	6900 4600 7100 4600
Connection ~ 7100 4600
Wire Wire Line
	7100 4600 7300 4600
Wire Wire Line
	7300 4600 7300 4500
$Comp
L Device:C_Small C8
U 1 1 5C197B2C
P 7550 4400
F 0 "C8" H 7550 4500 50  0000 L CNN
F 1 "47uF" H 7550 4300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7550 4400 50  0001 C CNN
F 3 "~" H 7550 4400 50  0001 C CNN
F 4 "490-9961-1-ND" H 7550 4400 50  0001 C CNN "Vendor"
	1    7550 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 4300 7550 4100
Wire Wire Line
	7550 4100 7300 4100
Connection ~ 7300 4100
Wire Wire Line
	7300 4600 7550 4600
Wire Wire Line
	7550 4600 7550 4500
Connection ~ 7300 4600
$Comp
L Device:C_Small C5
U 1 1 5C19F487
P 5350 4700
F 0 "C5" H 5250 4800 50  0000 L CNN
F 1 "0.1uF" H 5350 4600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5350 4700 50  0001 C CNN
F 3 "~" H 5350 4700 50  0001 C CNN
F 4 "16V, X7R" H 5350 4700 50  0001 C CNN "Notes"
F 5 "478-1395-1-ND" H 5350 4700 50  0001 C CNN "Vendor"
	1    5350 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 4850 5350 4850
Wire Wire Line
	5350 4850 5350 4800
Wire Wire Line
	5350 4100 6500 4100
Wire Wire Line
	5350 4100 5350 4600
Text Label 5400 5250 0    50   ~ 0
AIN1
Wire Wire Line
	5400 5250 5600 5250
Text Label 5400 5350 0    50   ~ 0
AIN2
Text Label 5400 5450 0    50   ~ 0
BIN1
Text Label 5400 5550 0    50   ~ 0
BIN2
Wire Wire Line
	5400 5350 5600 5350
Wire Wire Line
	5600 5450 5400 5450
Wire Wire Line
	5400 5550 5600 5550
Text Label 6450 2250 0    50   ~ 0
AIN1
Wire Wire Line
	6450 2250 6400 2250
Text Label 6450 2150 0    50   ~ 0
AIN2
Wire Wire Line
	6450 2150 6400 2150
Text Label 6450 2050 0    50   ~ 0
BIN1
Wire Wire Line
	6450 2050 6400 2050
Text Label 5200 2050 2    50   ~ 0
BIN2
Text Label 4900 2850 0    50   ~ 0
DRV_nSLEEP
Wire Wire Line
	4900 2850 5400 2850
Text Label 5500 4500 0    50   ~ 0
DRV_nSLEEP
Wire Wire Line
	5500 4500 5500 4750
Wire Wire Line
	5500 4750 5600 4750
NoConn ~ 6800 5050
$Comp
L power:GND #PWR09
U 1 1 5C1E2F22
P 4850 5200
F 0 "#PWR09" H 4850 4950 50  0001 C CNN
F 1 "GND" H 4855 5027 50  0000 C CNN
F 2 "" H 4850 5200 50  0001 C CNN
F 3 "" H 4850 5200 50  0001 C CNN
	1    4850 5200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5C1E3107
P 5100 4950
F 0 "R3" V 5000 4850 50  0000 C CNN
F 1 "0.47" V 5000 5000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5030 4950 50  0001 C CNN
F 3 "~" H 5100 4950 50  0001 C CNN
F 4 "73L3R47JCT-ND" V 5100 4950 50  0001 C CNN "Vendor"
F 5 "that's a shade over 400mA, and a 5% tol" V 5100 4950 50  0001 C CNN "Notes"
	1    5100 4950
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 5C1E3187
P 5100 5050
F 0 "R4" V 5200 4950 50  0000 C CNN
F 1 "0.47" V 5200 5100 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5030 5050 50  0001 C CNN
F 3 "~" H 5100 5050 50  0001 C CNN
F 4 "0.5 gives 400 mA chopping" V 5100 5050 50  0001 C CNN "Notes"
F 5 "73L3R47JCT-ND" V 5100 5050 50  0001 C CNN "Vendor"
	1    5100 5050
	0    1    1    0   
$EndComp
Wire Wire Line
	4950 5050 4850 5050
Wire Wire Line
	4850 5050 4850 5200
Wire Wire Line
	4950 4950 4850 4950
Wire Wire Line
	4850 4950 4850 5050
Connection ~ 4850 5050
Wire Wire Line
	5250 4950 5300 4950
Wire Wire Line
	5600 5050 5300 5050
$Comp
L Connector_Generic:Conn_01x02 J6
U 1 1 5C1F8D63
P 7350 5250
F 0 "J6" H 7450 5200 50  0000 C CNN
F 1 "MotA" H 7500 5300 50  0000 C CNN
F 2 "Connector_Molex:Molex_PicoBlade_53047-0410_1x02_P1.25mm_Vertical" H 7350 5250 50  0001 C CNN
F 3 "~" H 7350 5250 50  0001 C CNN
F 4 "455-1719-ND" H 7350 5250 50  0001 C CNN "Vendor"
	1    7350 5250
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J7
U 1 1 5C1F913A
P 7350 5500
F 0 "J7" H 7450 5450 50  0000 C CNN
F 1 "MotB" H 7500 5550 50  0000 C CNN
F 2 "Connector_Molex:Molex_PicoBlade_53047-0410_1x02_P1.25mm_Vertical" H 7350 5500 50  0001 C CNN
F 3 "~" H 7350 5500 50  0001 C CNN
F 4 "455-1719-ND" H 7350 5500 50  0001 C CNN "Vendor"
	1    7350 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 5250 7150 5250
Wire Wire Line
	7150 5350 6800 5350
Wire Wire Line
	6800 5450 6950 5450
Wire Wire Line
	6950 5450 6950 5500
Wire Wire Line
	6950 5500 7150 5500
Wire Wire Line
	6800 5550 6900 5550
Wire Wire Line
	6900 5550 6900 5600
Wire Wire Line
	6900 5600 7150 5600
Wire Wire Line
	6500 4100 7300 4100
Wire Wire Line
	6550 900  6550 1550
Wire Wire Line
	6550 1550 6400 1550
Wire Wire Line
	7350 1000 7150 1000
Wire Wire Line
	7150 1000 7150 1650
Wire Wire Line
	6800 1650 7150 1650
$Comp
L Connector_Generic:Conn_01x03 J8
U 1 1 5C229DB3
P 6900 700
F 0 "J8" V 6900 950 50  0000 R CNN
F 1 "Conn_01x03" V 6800 1350 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6900 700 50  0001 C CNN
F 3 "~" H 6900 700 50  0001 C CNN
	1    6900 700 
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6550 900  6800 900 
Wire Wire Line
	6900 900  6900 1000
Wire Wire Line
	7100 1000 7100 900 
Wire Wire Line
	7100 900  7350 900 
Wire Wire Line
	6900 1000 7100 1000
NoConn ~ 7000 900 
Text Label 6850 5250 0    50   ~ 0
AO1
Text Label 6850 5350 0    50   ~ 0
AO2
Text Label 6850 5450 0    50   ~ 0
BO1
Text Label 6850 5550 0    50   ~ 0
BO2
Wire Wire Line
	5200 2050 5400 2050
NoConn ~ 6050 3050
Text Label 5100 2250 0    50   ~ 0
ASEN
Text Label 5100 2350 0    50   ~ 0
BSEN
Wire Wire Line
	5100 2250 5400 2250
Wire Wire Line
	5400 2350 5100 2350
Text Label 4650 4750 0    50   ~ 0
ASEN
Text Label 4650 5550 0    50   ~ 0
BSEN
Wire Wire Line
	5300 4750 5300 4950
Connection ~ 5300 4950
Wire Wire Line
	5300 4950 5600 4950
Wire Wire Line
	5300 5050 5300 5550
Wire Wire Line
	5300 5550 4650 5550
Connection ~ 5300 5050
Wire Wire Line
	5300 5050 5250 5050
Wire Wire Line
	5300 4750 4650 4750
$EndSCHEMATC
