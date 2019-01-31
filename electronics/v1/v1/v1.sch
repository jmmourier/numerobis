EESchema Schematic File Version 4
LIBS:v1-cache
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
Text Notes 700  1200 0    50   ~ 0
left Motor
Text Notes 700  2400 0    50   ~ 0
right Motor
$Comp
L Connector:Conn_01x06_Female J101
U 1 1 5C2465EA
P 1250 1200
F 0 "J101" H 1144 675 50  0000 C CNN
F 1 "Conn_01x06_Female" H 1144 766 50  0000 C CNN
F 2 "Connectors_Molex:Molex_KK-6410-06_06x2.54mm_Straight" H 1250 1200 50  0001 C CNN
F 3 "~" H 1250 1200 50  0001 C CNN
	1    1250 1200
	-1   0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x06_Female J102
U 1 1 5C24661A
P 1250 2500
F 0 "J102" H 1144 1975 50  0000 C CNN
F 1 "Conn_01x06_Female" H 1144 2066 50  0000 C CNN
F 2 "Connectors_Molex:Molex_KK-6410-06_06x2.54mm_Straight" H 1250 2500 50  0001 C CNN
F 3 "~" H 1250 2500 50  0001 C CNN
	1    1250 2500
	-1   0    0    -1  
$EndComp
Text Notes 9300 750  0    50   ~ 0
power
Text Notes 6150 850  0    50   ~ 0
controler
Text Notes 1650 4350 0    50   ~ 0
VL53L0X - 1\n
Text Notes 2150 3300 0    50   ~ 0
ground sensor\n1 et 2
Text Notes 2000 6000 0    50   ~ 0
distance sensor
Text Label 4750 6700 0    50   ~ 0
distanceSensor1
Text Label 3950 6700 0    50   ~ 0
distanceSensor2
Text Label 3100 6700 0    50   ~ 0
distanceSensor3
Text Label 1400 6400 2    50   ~ 0
distanceSensorEmitter
Text Label 3200 3550 0    50   ~ 0
groundSensor1
Text Label 1700 3550 0    50   ~ 0
groundSensor2
Text Label 3200 3650 0    50   ~ 0
groundSensorEmitter
Text Label 1950 1000 0    50   ~ 0
MotorLeftP
Text Label 1600 1500 0    50   ~ 0
MotorLeftN
Text Label 3700 2300 0    50   ~ 0
MotorRightP
Text Label 3700 2800 0    50   ~ 0
MotorRightN
Text Label 1950 2700 0    50   ~ 0
encoderAlimentation
Text Label 1700 1400 0    50   ~ 0
encoderAlimentation
Text Label 1700 1200 0    50   ~ 0
CoderLeftA
Text Label 1700 1300 0    50   ~ 0
CoderLeftB
Text Label 1950 2500 0    50   ~ 0
CoderRightA
Text Label 1950 2600 0    50   ~ 0
CoderRightB
Text Label 4200 5350 2    50   ~ 0
TOF2_GPIO1
Text Label 4200 5250 2    50   ~ 0
TOF2_XSHUT
Text Label 4200 5150 2    50   ~ 0
TOF_SCL
Text Label 4200 5050 2    50   ~ 0
TOF_SDA
Text Label 4200 4850 2    50   ~ 0
TOF_VIN
Text Label 4200 4750 2    50   ~ 0
TOF2_VDD
Text Notes 4200 4400 0    50   ~ 0
VL53L0X - 2\n
Text Label 1450 5350 2    50   ~ 0
TOF1_GPIO1
Text Label 1450 5250 2    50   ~ 0
TOF1_XSHUT
Text Label 1450 5150 2    50   ~ 0
TOF_SCL
Text Label 1450 5050 2    50   ~ 0
TOF_SDA
Text Label 1450 4850 2    50   ~ 0
TOF_VIN
Text Label 1450 4750 2    50   ~ 0
TOF1_VDD
Text Notes 9150 3950 0    50   ~ 0
interface
Text Label 9550 5950 2    50   ~ 0
Led1
Text Label 9550 6300 2    50   ~ 0
Led2
Text Label 10650 4800 0    50   ~ 0
Button1
Text Label 10000 4800 0    50   ~ 0
Button2
Text Label 9350 4800 0    50   ~ 0
Reset
Text Label 7850 1050 2    50   ~ 0
batterieP
Text Label 9550 1950 0    50   ~ 0
batterieN
$Comp
L arduinoProMini:ArduinoProMini IC101
U 1 1 5C241451
P 6250 2400
F 0 "IC101" H 6300 3744 40  0000 C CNN
F 1 "ArduinoProMini" H 6300 3668 40  0000 C CNN
F 2 "lib:pro_mini" H 6250 2400 30  0001 C CIN
F 3 "" H 6250 2400 60  0000 C CNN
	1    6250 2400
	1    0    0    -1  
$EndComp
$Comp
L miniVictoryL298N:miniVictoryL298N PCB101
U 1 1 5C2B9144
P 3350 1100
F 0 "PCB101" H 3475 1165 50  0000 C CNN
F 1 "miniVictoryL298N" H 3475 1074 50  0000 C CNN
F 2 "" H 3350 1100 50  0001 C CNN
F 3 "" H 3350 1100 50  0001 C CNN
	1    3350 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 1000 4300 1500
Wire Wire Line
	4300 1500 4100 1500
Wire Wire Line
	1450 1000 4300 1000
Wire Wire Line
	4200 1100 4200 1400
Wire Wire Line
	4200 1400 4100 1400
Wire Wire Line
	4200 2300 4200 1750
Wire Wire Line
	4200 1750 4100 1750
Wire Wire Line
	1450 2300 4200 2300
Wire Wire Line
	4100 1650 4300 1650
Wire Wire Line
	1450 2600 1950 2600
Wire Wire Line
	1450 2700 1950 2700
$Comp
L power:GND #PWR0102
U 1 1 5C2B9EC9
P 1450 1100
F 0 "#PWR0102" H 1450 850 50  0001 C CNN
F 1 "GND" H 1455 927 50  0000 C CNN
F 2 "" H 1450 1100 50  0001 C CNN
F 3 "" H 1450 1100 50  0001 C CNN
	1    1450 1100
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5C2B9EE3
P 2650 2400
F 0 "#PWR0104" H 2650 2150 50  0001 C CNN
F 1 "GND" H 2655 2227 50  0000 C CNN
F 2 "" H 2650 2400 50  0001 C CNN
F 3 "" H 2650 2400 50  0001 C CNN
	1    2650 2400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5C2B9EF6
P 2750 1400
F 0 "#PWR0101" H 2750 1150 50  0001 C CNN
F 1 "GND" H 2755 1227 50  0000 C CNN
F 2 "" H 2750 1400 50  0001 C CNN
F 3 "" H 2750 1400 50  0001 C CNN
	1    2750 1400
	0    1    1    0   
$EndComp
Wire Wire Line
	1450 1200 1700 1200
Wire Wire Line
	1700 1300 1450 1300
Wire Wire Line
	1450 1400 1700 1400
Wire Wire Line
	1950 2500 1450 2500
Text Label 7350 1500 0    50   ~ 0
CoderLeftA
Text Label 7350 2000 0    50   ~ 0
CoderLeftB
Text Label 7350 1600 0    50   ~ 0
CoderRightA
Text Label 7350 2100 0    50   ~ 0
CoderRightB
Text Label 2750 1600 2    50   ~ 0
IN1_HBridge
Text Label 2750 1700 2    50   ~ 0
IN2_HBridge
Text Label 2750 1850 2    50   ~ 0
IN3_HBridge
Text Label 2750 1950 2    50   ~ 0
IN4_HBridge
Wire Wire Line
	2750 1600 2850 1600
Wire Wire Line
	2750 1700 2850 1700
Wire Wire Line
	2750 1850 2850 1850
Wire Wire Line
	2750 1950 2850 1950
Text Label 7350 1800 0    50   ~ 0
IN1_HBridge
Text Label 7350 2300 0    50   ~ 0
IN2_HBridge
Text Label 7350 1900 0    50   ~ 0
IN3_HBridge
Text Label 7350 2500 0    50   ~ 0
IN4_HBridge
Text Label 10500 1500 0    50   ~ 0
encoderAlimentation
Text Label 2800 1300 2    50   ~ 0
alimHBridge
Wire Wire Line
	2800 1300 2850 1300
Text Label 8600 1300 2    50   ~ 0
alimHBridge
$Comp
L QTR-1A:QTR-1A U103
U 1 1 5C2BF47E
P 2750 3450
F 0 "U103" H 2781 3615 50  0000 C CNN
F 1 "QTR-1A" H 2781 3524 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x03_Pitch2.54mm" H 2750 3450 50  0001 C CNN
F 3 "" H 2750 3450 50  0001 C CNN
	1    2750 3450
	1    0    0    -1  
$EndComp
$Comp
L QTR-1A:QTR-1A U102
U 1 1 5C2BF51E
P 1350 3450
F 0 "U102" H 1381 3615 50  0000 C CNN
F 1 "QTR-1A" H 1381 3524 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x03_Pitch2.54mm" H 1350 3450 50  0001 C CNN
F 3 "" H 1350 3450 50  0001 C CNN
	1    1350 3450
	1    0    0    -1  
$EndComp
Text Label 1700 3650 0    50   ~ 0
groundSensorEmitter
Text Label 7350 3000 0    50   ~ 0
groundSensor2
Text Label 7350 2900 0    50   ~ 0
groundSensor1
Text Label 7350 1700 0    50   ~ 0
arduinoGroundSensorEmitter
Wire Wire Line
	1600 3550 1700 3550
Wire Wire Line
	1600 3650 1700 3650
Wire Wire Line
	3000 3550 3200 3550
Wire Wire Line
	3000 3650 3200 3650
Wire Wire Line
	1600 3750 2050 3750
Wire Wire Line
	2400 3750 2400 4000
Wire Wire Line
	2400 4000 3100 4000
Wire Wire Line
	3100 4000 3100 3750
Wire Wire Line
	3100 3750 3000 3750
$Comp
L power:GND #PWR0106
U 1 1 5C2C1C26
P 2050 3850
F 0 "#PWR0106" H 2050 3600 50  0001 C CNN
F 1 "GND" H 2055 3677 50  0000 C CNN
F 2 "" H 2050 3850 50  0001 C CNN
F 3 "" H 2050 3850 50  0001 C CNN
	1    2050 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 3750 2050 3850
Connection ~ 2050 3750
Wire Wire Line
	2050 3750 2400 3750
$Comp
L VL53L0X:VL53L0X_module U105
U 1 1 5C2C35D5
P 4450 4650
F 0 "U105" H 4450 4750 50  0000 L CNN
F 1 "VL53L0X_module" H 4250 4850 50  0000 L CNN
F 2 "" H 4450 4650 50  0001 C CNN
F 3 "" H 4450 4650 50  0001 C CNN
	1    4450 4650
	1    0    0    -1  
$EndComp
$Comp
L VL53L0X:VL53L0X_module U104
U 1 1 5C2C3643
P 1750 4650
F 0 "U104" H 1850 4750 50  0000 L CNN
F 1 "VL53L0X_module" H 1600 4850 50  0000 L CNN
F 2 "" H 1750 4650 50  0001 C CNN
F 3 "" H 1750 4650 50  0001 C CNN
	1    1750 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 4850 1600 4850
Text Label 10500 1200 0    50   ~ 0
TOF_VIN
Text Label 7350 3300 0    50   ~ 0
TOF_SDA
Text Label 7350 3400 0    50   ~ 0
TOF_SCL
Wire Wire Line
	1450 5050 1600 5050
Wire Wire Line
	1450 5150 1600 5150
$Comp
L power:GND #PWR0107
U 1 1 5C2CFB98
P 1000 4950
F 0 "#PWR0107" H 1000 4700 50  0001 C CNN
F 1 "GND" H 1005 4777 50  0000 C CNN
F 2 "" H 1000 4950 50  0001 C CNN
F 3 "" H 1000 4950 50  0001 C CNN
	1    1000 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 4950 1600 4950
NoConn ~ 950  5350
Wire Wire Line
	950  5350 1600 5350
NoConn ~ 1000 4750
Wire Wire Line
	1000 4750 1600 4750
NoConn ~ 3750 4750
Wire Wire Line
	3750 4750 4300 4750
Text Notes 2100 5800 0    50   ~ 0
TOFX_XSHUT is not level shifted, componant needed !
$Comp
L power:GND #PWR0108
U 1 1 5C2D91EA
P 3700 4950
F 0 "#PWR0108" H 3700 4700 50  0001 C CNN
F 1 "GND" H 3705 4777 50  0000 C CNN
F 2 "" H 3700 4950 50  0001 C CNN
F 3 "" H 3700 4950 50  0001 C CNN
	1    3700 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 4850 4300 4850
Wire Wire Line
	4200 5050 4300 5050
Wire Wire Line
	4300 4950 3700 4950
Wire Wire Line
	4200 5150 4300 5150
NoConn ~ 3700 5350
Wire Wire Line
	3700 5350 4300 5350
Text Notes 5550 6150 0    50   ~ 0
TODO\n- set resistor value for distance sensor\n
$Comp
L Switch:SW_Push SW102
U 1 1 5C2E5C52
P 9850 5100
F 0 "SW102" V 9804 5248 50  0000 L CNN
F 1 "SW_Push" V 9895 5248 50  0000 L CNN
F 2 "Buttons_Switches_THT:SW_PUSH_6mm_h4.3mm" H 9850 5300 50  0001 C CNN
F 3 "" H 9850 5300 50  0001 C CNN
	1    9850 5100
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW103
U 1 1 5C2E5C94
P 10500 5100
F 0 "SW103" V 10454 5248 50  0000 L CNN
F 1 "SW_Push" V 10545 5248 50  0000 L CNN
F 2 "Buttons_Switches_THT:SW_PUSH_6mm_h4.3mm" H 10500 5300 50  0001 C CNN
F 3 "" H 10500 5300 50  0001 C CNN
	1    10500 5100
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW101
U 1 1 5C2E5D64
P 9200 5100
F 0 "SW101" V 9154 5248 50  0000 L CNN
F 1 "SW_Push" V 9245 5248 50  0000 L CNN
F 2 "Buttons_Switches_THT:SW_PUSH_6mm_h4.3mm" H 9200 5300 50  0001 C CNN
F 3 "" H 9200 5300 50  0001 C CNN
	1    9200 5100
	0    1    1    0   
$EndComp
$Comp
L Device:R R103
U 1 1 5C2E8244
P 10500 4600
F 0 "R103" H 10570 4646 50  0000 L CNN
F 1 "R" H 10570 4555 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10430 4600 50  0001 C CNN
F 3 "~" H 10500 4600 50  0001 C CNN
	1    10500 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R102
U 1 1 5C2E827E
P 9850 4600
F 0 "R102" H 9920 4646 50  0000 L CNN
F 1 "R" H 9920 4555 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9780 4600 50  0001 C CNN
F 3 "~" H 9850 4600 50  0001 C CNN
	1    9850 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R101
U 1 1 5C2E8351
P 9200 4600
F 0 "R101" H 9270 4646 50  0000 L CNN
F 1 "R" H 9270 4555 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9130 4600 50  0001 C CNN
F 3 "~" H 9200 4600 50  0001 C CNN
	1    9200 4600
	1    0    0    -1  
$EndComp
Text Label 9600 4250 0    50   ~ 0
VIN_Button
Text Label 10500 1300 0    50   ~ 0
VIN_Button
Wire Wire Line
	9600 4250 9600 4350
Wire Wire Line
	9600 4350 9200 4350
Wire Wire Line
	9200 4350 9200 4450
Wire Wire Line
	9600 4350 9850 4350
Wire Wire Line
	9850 4350 9850 4450
Connection ~ 9600 4350
Wire Wire Line
	9850 4350 10500 4350
Wire Wire Line
	10500 4350 10500 4450
Connection ~ 9850 4350
Wire Wire Line
	9200 4750 9200 4800
Wire Wire Line
	9200 4800 9350 4800
Wire Wire Line
	9850 4750 9850 4800
Wire Wire Line
	9850 4800 10000 4800
Wire Wire Line
	10500 4750 10500 4800
Wire Wire Line
	10500 4800 10650 4800
Wire Wire Line
	10500 4800 10500 4900
Connection ~ 10500 4800
Wire Wire Line
	9850 4800 9850 4900
Connection ~ 9850 4800
Wire Wire Line
	9200 4800 9200 4900
Connection ~ 9200 4800
$Comp
L power:GND #PWR0109
U 1 1 5C2F38E5
P 9900 5450
F 0 "#PWR0109" H 9900 5200 50  0001 C CNN
F 1 "GND" H 9905 5277 50  0000 C CNN
F 2 "" H 9900 5450 50  0001 C CNN
F 3 "" H 9900 5450 50  0001 C CNN
	1    9900 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 5450 9900 5350
Wire Wire Line
	9900 5350 9850 5350
Wire Wire Line
	9850 5350 9850 5300
Wire Wire Line
	9850 5350 9200 5350
Wire Wire Line
	9200 5350 9200 5300
Connection ~ 9850 5350
Wire Wire Line
	9900 5350 10500 5350
Wire Wire Line
	10500 5350 10500 5300
Connection ~ 9900 5350
Text Label 5150 3100 2    50   ~ 0
Reset
Wire Wire Line
	5150 3100 5250 3100
Wire Wire Line
	5250 3100 5250 3200
Wire Wire Line
	5250 3200 5350 3200
Connection ~ 5250 3100
Wire Wire Line
	5250 3100 5350 3100
Text Label 7950 5450 2    50   ~ 0
Button2
Text Label 7350 3600 0    50   ~ 0
Button1
Text Label 7350 2700 0    50   ~ 0
Led1
Text Label 7950 5550 2    50   ~ 0
Led2
$Comp
L Device:R R107
U 1 1 5C2FEE8B
P 9850 5950
F 0 "R107" V 9643 5950 50  0000 C CNN
F 1 "R" V 9734 5950 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9780 5950 50  0001 C CNN
F 3 "~" H 9850 5950 50  0001 C CNN
	1    9850 5950
	0    1    1    0   
$EndComp
$Comp
L Device:R R108
U 1 1 5C2FEEE5
P 9850 6300
F 0 "R108" V 9643 6300 50  0000 C CNN
F 1 "R" V 9734 6300 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9780 6300 50  0001 C CNN
F 3 "~" H 9850 6300 50  0001 C CNN
	1    9850 6300
	0    1    1    0   
$EndComp
$Comp
L Device:LED D101
U 1 1 5C2FEF7E
P 10250 5950
F 0 "D101" H 10242 5695 50  0000 C CNN
F 1 "LED" H 10242 5786 50  0000 C CNN
F 2 "LEDs:LED_D5.0mm" H 10250 5950 50  0001 C CNN
F 3 "~" H 10250 5950 50  0001 C CNN
	1    10250 5950
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D102
U 1 1 5C2FEFEA
P 10250 6300
F 0 "D102" H 10242 6045 50  0000 C CNN
F 1 "LED" H 10242 6136 50  0000 C CNN
F 2 "LEDs:LED_D5.0mm" H 10250 6300 50  0001 C CNN
F 3 "~" H 10250 6300 50  0001 C CNN
	1    10250 6300
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 5C302846
P 10800 6150
F 0 "#PWR0111" H 10800 5900 50  0001 C CNN
F 1 "GND" H 10805 5977 50  0000 C CNN
F 2 "" H 10800 6150 50  0001 C CNN
F 3 "" H 10800 6150 50  0001 C CNN
	1    10800 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	10800 6150 10800 5950
Wire Wire Line
	10800 5950 10600 5950
Wire Wire Line
	10400 6300 10600 6300
Wire Wire Line
	10600 6300 10600 5950
Connection ~ 10600 5950
Wire Wire Line
	10600 5950 10400 5950
Wire Wire Line
	10100 6300 10000 6300
Wire Wire Line
	10000 5950 10100 5950
Wire Wire Line
	9700 5950 9550 5950
Wire Wire Line
	9700 6300 9550 6300
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J105
U 1 1 5C3198A8
P 8300 6100
F 0 "J105" H 8350 6417 50  0000 C CNN
F 1 "Conn_02x03_Odd_Even" H 8350 6326 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03_Pitch2.54mm" H 8300 6100 50  0001 C CNN
F 3 "~" H 8300 6100 50  0001 C CNN
	1    8300 6100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0113
U 1 1 5C319AE0
P 7950 6250
F 0 "#PWR0113" H 7950 6000 50  0001 C CNN
F 1 "GND" H 7955 6077 50  0000 C CNN
F 2 "" H 7950 6250 50  0001 C CNN
F 3 "" H 7950 6250 50  0001 C CNN
	1    7950 6250
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Photo D106
U 1 1 5C31A439
P 3000 6900
F 0 "D106" V 2904 7057 50  0000 L CNN
F 1 "D_Photo" V 2995 7057 50  0000 L CNN
F 2 "LEDs:LED_D5.0mm_Horizontal_O3.81mm_Z3.0mm" H 2950 6900 50  0001 C CNN
F 3 "~" H 2950 6900 50  0001 C CNN
	1    3000 6900
	0    1    1    0   
$EndComp
$Comp
L Device:D_Photo D107
U 1 1 5C31A4C9
P 3850 6900
F 0 "D107" V 3754 7057 50  0000 L CNN
F 1 "D_Photo" V 3845 7057 50  0000 L CNN
F 2 "LEDs:LED_D5.0mm_Horizontal_O3.81mm_Z3.0mm" H 3800 6900 50  0001 C CNN
F 3 "~" H 3800 6900 50  0001 C CNN
	1    3850 6900
	0    1    1    0   
$EndComp
$Comp
L Device:D_Photo D108
U 1 1 5C31A65F
P 4650 6900
F 0 "D108" V 4554 7057 50  0000 L CNN
F 1 "D_Photo" V 4645 7057 50  0000 L CNN
F 2 "LEDs:LED_D5.0mm_Horizontal_O3.81mm_Z3.0mm" H 4600 6900 50  0001 C CNN
F 3 "~" H 4600 6900 50  0001 C CNN
	1    4650 6900
	0    1    1    0   
$EndComp
$Comp
L Device:R R112
U 1 1 5C31F322
P 3000 6450
F 0 "R112" H 3070 6496 50  0000 L CNN
F 1 "R" H 3070 6405 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2930 6450 50  0001 C CNN
F 3 "~" H 3000 6450 50  0001 C CNN
	1    3000 6450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R113
U 1 1 5C31F36E
P 3850 6450
F 0 "R113" H 3920 6496 50  0000 L CNN
F 1 "R" H 3920 6405 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3780 6450 50  0001 C CNN
F 3 "~" H 3850 6450 50  0001 C CNN
	1    3850 6450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R114
U 1 1 5C31F3D6
P 4650 6450
F 0 "R114" H 4720 6496 50  0000 L CNN
F 1 "R" H 4720 6405 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4580 6450 50  0001 C CNN
F 3 "~" H 4650 6450 50  0001 C CNN
	1    4650 6450
	1    0    0    -1  
$EndComp
Text Label 7350 3500 0    50   ~ 0
distanceSensor3
Text Label 7350 3200 0    50   ~ 0
distanceSensor2
Text Label 7350 3100 0    50   ~ 0
distanceSensor1
Text Label 7350 2200 0    50   ~ 0
distanceSensorEmitter
$Comp
L Device:R R115
U 1 1 5C31F996
P 1400 6700
F 0 "R115" H 1470 6746 50  0000 L CNN
F 1 "R" H 1470 6655 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1330 6700 50  0001 C CNN
F 3 "~" H 1400 6700 50  0001 C CNN
	1    1400 6700
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D103
U 1 1 5C31FA4F
P 1700 6800
F 0 "D103" V 1738 6683 50  0000 R CNN
F 1 "LED" V 1647 6683 50  0000 R CNN
F 2 "LEDs:LED_D5.0mm_Horizontal_O3.81mm_Z3.0mm" H 1700 6800 50  0001 C CNN
F 3 "~" H 1700 6800 50  0001 C CNN
	1    1700 6800
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D104
U 1 1 5C31FB0E
P 2100 6800
F 0 "D104" V 2138 6683 50  0000 R CNN
F 1 "LED" V 2047 6683 50  0000 R CNN
F 2 "LEDs:LED_D5.0mm_Horizontal_O3.81mm_Z3.0mm" H 2100 6800 50  0001 C CNN
F 3 "~" H 2100 6800 50  0001 C CNN
	1    2100 6800
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D105
U 1 1 5C31FB62
P 2500 6800
F 0 "D105" V 2538 6683 50  0000 R CNN
F 1 "LED" V 2447 6683 50  0000 R CNN
F 2 "LEDs:LED_D5.0mm_Horizontal_O3.81mm_Z3.0mm" H 2500 6800 50  0001 C CNN
F 3 "~" H 2500 6800 50  0001 C CNN
	1    2500 6800
	0    -1   -1   0   
$EndComp
$Comp
L Transistor_BJT:TIP122 Q102
U 1 1 5C31FDE8
P 1900 7250
F 0 "Q102" H 2107 7296 50  0000 L CNN
F 1 "TIP122" H 2107 7205 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-220-3_Vertical" H 2100 7175 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/TI/TIP120.pdf" H 1900 7250 50  0001 L CNN
	1    1900 7250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R109
U 1 1 5C33B69F
P 1700 6450
F 0 "R109" H 1770 6496 50  0000 L CNN
F 1 "R" H 1770 6405 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1630 6450 50  0001 C CNN
F 3 "~" H 1700 6450 50  0001 C CNN
	1    1700 6450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R110
U 1 1 5C33B707
P 2100 6450
F 0 "R110" H 2170 6496 50  0000 L CNN
F 1 "R" H 2170 6405 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2030 6450 50  0001 C CNN
F 3 "~" H 2100 6450 50  0001 C CNN
	1    2100 6450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R111
U 1 1 5C33B765
P 2500 6450
F 0 "R111" H 2570 6496 50  0000 L CNN
F 1 "R" H 2570 6405 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2430 6450 50  0001 C CNN
F 3 "~" H 2500 6450 50  0001 C CNN
	1    2500 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 6950 1700 7050
Wire Wire Line
	1700 7050 2000 7050
Wire Wire Line
	2000 7050 2100 7050
Wire Wire Line
	2100 7050 2100 6950
Connection ~ 2000 7050
Wire Wire Line
	2100 7050 2500 7050
Wire Wire Line
	2500 7050 2500 6950
Connection ~ 2100 7050
Wire Wire Line
	1700 7250 1400 7250
Wire Wire Line
	1400 7250 1400 6850
Wire Wire Line
	1400 6550 1400 6400
Wire Wire Line
	1700 6650 1700 6600
Wire Wire Line
	2100 6650 2100 6600
Wire Wire Line
	2500 6650 2500 6600
$Comp
L power:GND #PWR0115
U 1 1 5C35596A
P 2000 7600
F 0 "#PWR0115" H 2000 7350 50  0001 C CNN
F 1 "GND" H 2005 7427 50  0000 C CNN
F 2 "" H 2000 7600 50  0001 C CNN
F 3 "" H 2000 7600 50  0001 C CNN
	1    2000 7600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 7600 2000 7450
$Comp
L power:GND #PWR0114
U 1 1 5C358926
P 3850 7200
F 0 "#PWR0114" H 3850 6950 50  0001 C CNN
F 1 "GND" H 3855 7027 50  0000 C CNN
F 2 "" H 3850 7200 50  0001 C CNN
F 3 "" H 3850 7200 50  0001 C CNN
	1    3850 7200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 7000 3000 7150
Wire Wire Line
	3000 7150 3850 7150
Wire Wire Line
	3850 7150 3850 7200
Wire Wire Line
	3850 7150 3850 7000
Connection ~ 3850 7150
Wire Wire Line
	3850 7150 4650 7150
Wire Wire Line
	4650 7150 4650 7000
Wire Wire Line
	4650 6700 4650 6600
Wire Wire Line
	4750 6700 4650 6700
Connection ~ 4650 6700
Wire Wire Line
	3950 6700 3850 6700
Wire Wire Line
	3850 6700 3850 6600
Connection ~ 3850 6700
Wire Wire Line
	3100 6700 3000 6700
Wire Wire Line
	3000 6600 3000 6700
Connection ~ 3000 6700
Text Label 2800 6050 0    50   ~ 0
alimDistanceSensor
Wire Wire Line
	1700 6150 1700 6300
Wire Wire Line
	2100 6300 2100 6150
Connection ~ 2100 6150
Wire Wire Line
	2100 6150 1700 6150
Wire Wire Line
	4650 6150 4650 6300
Wire Wire Line
	2100 6150 2500 6150
Wire Wire Line
	3850 6300 3850 6150
Wire Wire Line
	3850 6150 4650 6150
Wire Wire Line
	2500 6300 2500 6150
Text Label 10500 1400 0    50   ~ 0
alimDistanceSensor
Wire Wire Line
	3850 6150 3000 6150
Wire Wire Line
	3000 6150 3000 6300
Connection ~ 3850 6150
Wire Wire Line
	3000 6150 2800 6150
Connection ~ 3000 6150
Connection ~ 2500 6150
Wire Wire Line
	2800 6050 2800 6150
Connection ~ 2800 6150
Wire Wire Line
	2800 6150 2500 6150
Text Label 5200 1600 2    50   ~ 0
AlimArduino
Text Label 8600 1400 2    50   ~ 0
AlimArduino
$Comp
L power:GND #PWR0105
U 1 1 5C39D1C1
P 5000 3450
F 0 "#PWR0105" H 5000 3200 50  0001 C CNN
F 1 "GND" H 5005 3277 50  0000 C CNN
F 2 "" H 5000 3450 50  0001 C CNN
F 3 "" H 5000 3450 50  0001 C CNN
	1    5000 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 3450 5000 3400
Wire Wire Line
	5000 3400 5250 3400
Wire Wire Line
	5250 3400 5250 3500
Wire Wire Line
	5250 3500 5350 3500
Connection ~ 5250 3400
Wire Wire Line
	5250 3400 5350 3400
Wire Wire Line
	5250 3500 5250 3600
Wire Wire Line
	5250 3600 5350 3600
Connection ~ 5250 3500
Wire Wire Line
	7250 3600 7350 3600
Wire Wire Line
	7250 3500 7350 3500
Wire Wire Line
	7250 3400 7350 3400
Wire Wire Line
	7250 3300 7350 3300
Wire Wire Line
	7250 3200 7350 3200
Wire Wire Line
	7250 3100 7350 3100
Wire Wire Line
	7250 3000 7350 3000
Wire Wire Line
	7250 2900 7350 2900
Wire Wire Line
	7250 2700 7350 2700
Wire Wire Line
	7250 2500 7350 2500
Wire Wire Line
	7250 2300 7350 2300
Wire Wire Line
	7250 2200 7350 2200
Wire Wire Line
	7250 2100 7350 2100
Wire Wire Line
	7250 2000 7350 2000
Wire Wire Line
	7250 1900 7350 1900
Wire Wire Line
	7250 1800 7350 1800
Wire Wire Line
	7250 1700 7350 1700
Wire Wire Line
	7250 1600 7350 1600
Wire Wire Line
	7250 1500 7350 1500
Text Label 7500 1300 0    50   ~ 0
RX
Text Label 7500 1400 0    50   ~ 0
TX
Wire Wire Line
	7500 1400 7250 1400
Wire Wire Line
	7250 1300 7500 1300
Wire Bus Line
	700  3050 4400 3050
Wire Bus Line
	650  4200 4400 4200
Wire Bus Line
	700  5900 4400 5900
Wire Bus Line
	4700 700  4700 4200
$Comp
L Transistor_BJT:TIP122 Q101
U 1 1 5C44B456
P 3150 5250
F 0 "Q101" H 3357 5296 50  0000 L CNN
F 1 "TIP122" H 3357 5205 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-220-3_Vertical" H 3350 5175 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/TI/TIP120.pdf" H 3150 5250 50  0001 L CNN
	1    3150 5250
	1    0    0    -1  
$EndComp
Text Label 2700 4850 0    50   ~ 0
TOF2_XSHUT_CTRL
$Comp
L Device:R R106
U 1 1 5C452729
P 2700 5100
F 0 "R106" H 2770 5146 50  0000 L CNN
F 1 "R" H 2770 5055 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2630 5100 50  0001 C CNN
F 3 "~" H 2700 5100 50  0001 C CNN
	1    2700 5100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 5C4527B1
P 3250 5500
F 0 "#PWR0110" H 3250 5250 50  0001 C CNN
F 1 "GND" H 3255 5327 50  0000 C CNN
F 2 "" H 3250 5500 50  0001 C CNN
F 3 "" H 3250 5500 50  0001 C CNN
	1    3250 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 5250 3550 5050
Wire Wire Line
	3550 5050 3250 5050
Wire Wire Line
	3550 5250 4300 5250
Wire Wire Line
	3250 5500 3250 5450
Wire Wire Line
	2950 5250 2700 5250
Wire Wire Line
	2700 4950 2700 4850
Text Label 7350 2600 0    50   ~ 0
TOF2_XSHUT_CTRL
Wire Wire Line
	7350 2600 7250 2600
Text Notes 10500 1100 0    50   ~ 0
5V
$Comp
L Connector:Conn_01x02_Female J103
U 1 1 5C47F120
P 8200 4850
F 0 "J103" H 8227 4826 50  0000 L CNN
F 1 "Conn_01x02_Female" H 8227 4735 50  0000 L CNN
F 2 "Connectors_Molex:Molex_KK-6410-02_02x2.54mm_Straight" H 8200 4850 50  0001 C CNN
F 3 "~" H 8200 4850 50  0001 C CNN
	1    8200 4850
	1    0    0    -1  
$EndComp
Text Notes 8100 4750 0    50   ~ 0
battery
Text Label 7850 4850 2    50   ~ 0
batterieP
Text Label 7850 4950 2    50   ~ 0
batterieN
Text Notes 8400 1200 0    50   ~ 0
RAW
Wire Wire Line
	5200 1600 5350 1600
NoConn ~ 5200 1300
Wire Wire Line
	5200 1300 5350 1300
$Comp
L Regulator_Linear:L7805 U101
U 1 1 5C4DF2C4
P 9550 1150
F 0 "U101" H 9550 1392 50  0000 C CNN
F 1 "L7805" H 9550 1301 50  0000 C CNN
F 2 "TO_SOT_Packages_THT:TO-220-3_Vertical" H 9575 1000 50  0001 L CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/41/4f/b3/b0/12/d4/47/88/CD00000444.pdf/files/CD00000444.pdf/jcr:content/translations/en.CD00000444.pdf" H 9550 1100 50  0001 C CNN
	1    9550 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 1150 10250 1200
Wire Wire Line
	10250 1200 10500 1200
Wire Wire Line
	10500 1300 10250 1300
Wire Wire Line
	10250 1300 10250 1200
Connection ~ 10250 1200
Wire Wire Line
	10500 1400 10250 1400
Wire Wire Line
	10250 1400 10250 1300
Connection ~ 10250 1300
Wire Wire Line
	10500 1500 10250 1500
Wire Wire Line
	10250 1500 10250 1400
Connection ~ 10250 1400
$Comp
L power:GND #PWR0103
U 1 1 5C50C120
P 9350 2300
F 0 "#PWR0103" H 9350 2050 50  0001 C CNN
F 1 "GND" H 9355 2127 50  0000 C CNN
F 2 "" H 9350 2300 50  0001 C CNN
F 3 "" H 9350 2300 50  0001 C CNN
	1    9350 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 1050 8950 1150
Wire Wire Line
	8950 1150 9050 1150
Wire Wire Line
	8950 1150 8950 1300
Wire Wire Line
	8950 1300 8600 1300
Connection ~ 8950 1150
Wire Wire Line
	8950 1300 8950 1400
Wire Wire Line
	8950 1400 8600 1400
Connection ~ 8950 1300
$Comp
L Connector_Generic:Conn_02x02_Odd_Even J104
U 1 1 5C581025
P 8300 5450
F 0 "J104" H 8350 5667 50  0000 C CNN
F 1 "Conn_02x02_Odd_Even" H 8350 5576 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x02_Pitch2.54mm" H 8300 5450 50  0001 C CNN
F 3 "~" H 8300 5450 50  0001 C CNN
	1    8300 5450
	1    0    0    -1  
$EndComp
Text Label 8700 5450 0    50   ~ 0
RX
Text Label 8700 5550 0    50   ~ 0
TX
Wire Wire Line
	8700 5550 8600 5550
Wire Wire Line
	8700 5450 8600 5450
Wire Wire Line
	8100 5450 7950 5450
Wire Wire Line
	7950 5550 8100 5550
Wire Wire Line
	7950 6250 7950 6200
Wire Wire Line
	7950 6200 8100 6200
Text Label 8000 6000 2    50   ~ 0
RX
Text Label 8000 6100 2    50   ~ 0
TX
Text Label 8750 6000 0    50   ~ 0
batterieP
$Comp
L power:GND #PWR0112
U 1 1 5C5B31B4
P 8800 6200
F 0 "#PWR0112" H 8800 5950 50  0001 C CNN
F 1 "GND" H 8805 6027 50  0000 C CNN
F 2 "" H 8800 6200 50  0001 C CNN
F 3 "" H 8800 6200 50  0001 C CNN
	1    8800 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 6200 8800 6200
Wire Wire Line
	8000 6100 8100 6100
Wire Wire Line
	8000 6000 8100 6000
Wire Wire Line
	8600 6000 8700 6000
NoConn ~ 950  5250
Wire Wire Line
	950  5250 1600 5250
Wire Wire Line
	8600 6100 8700 6100
Wire Wire Line
	8700 6100 8700 6000
Connection ~ 8700 6000
Wire Wire Line
	8700 6000 8750 6000
Text Label 6450 5550 0    50   ~ 0
TOF_SDA
Text Label 6450 5650 0    50   ~ 0
TOF_SCL
$Comp
L Device:R R105
U 1 1 5C61CC33
P 6300 5400
F 0 "R105" H 6370 5446 50  0000 L CNN
F 1 "2.2k" H 6370 5355 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 6230 5400 50  0001 C CNN
F 3 "~" H 6300 5400 50  0001 C CNN
	1    6300 5400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R104
U 1 1 5C61CC9F
P 5900 5400
F 0 "R104" H 5970 5446 50  0000 L CNN
F 1 "2.2k" H 5970 5355 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5830 5400 50  0001 C CNN
F 3 "~" H 5900 5400 50  0001 C CNN
	1    5900 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 5550 6450 5550
Text Label 10500 1600 0    50   ~ 0
5V
Wire Wire Line
	10500 1600 10250 1600
Wire Wire Line
	10250 1600 10250 1500
Connection ~ 10250 1500
Text Label 6200 5150 0    50   ~ 0
5V
Wire Wire Line
	6200 5150 6200 5200
Wire Wire Line
	6200 5200 6300 5200
Wire Wire Line
	6300 5200 6300 5250
Connection ~ 6200 5200
Text Notes 6100 5000 0    50   ~ 0
pull up I2C
Wire Wire Line
	5900 5250 5900 5200
Wire Wire Line
	5900 5200 6200 5200
Wire Wire Line
	5900 5550 5900 5650
Wire Wire Line
	5900 5650 6450 5650
$Comp
L Device:C C102
U 1 1 5C68551F
P 9900 1350
F 0 "C102" H 10015 1396 50  0000 L CNN
F 1 "100n" H 10015 1305 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.50mm" H 9938 1200 50  0001 C CNN
F 3 "~" H 9900 1350 50  0001 C CNN
	1    9900 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C101
U 1 1 5C685591
P 9050 1350
F 0 "C101" H 9165 1396 50  0000 L CNN
F 1 "100n" H 9165 1305 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.50mm" H 9088 1200 50  0001 C CNN
F 3 "~" H 9050 1350 50  0001 C CNN
	1    9050 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 1150 9050 1200
Connection ~ 9050 1150
Wire Wire Line
	9900 1200 9900 1150
Wire Wire Line
	9900 1150 10250 1150
Wire Wire Line
	9050 1150 9250 1150
Wire Wire Line
	9850 1150 9900 1150
Connection ~ 9900 1150
Wire Wire Line
	7850 4850 7950 4850
Wire Wire Line
	8000 4950 7950 4950
$Comp
L Device:LED D109
U 1 1 5C7420E5
P 10250 1900
F 0 "D109" V 10288 1783 50  0000 R CNN
F 1 "LED" V 10197 1783 50  0000 R CNN
F 2 "LEDs:LED_D5.0mm" H 10250 1900 50  0001 C CNN
F 3 "~" H 10250 1900 50  0001 C CNN
	1    10250 1900
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R116
U 1 1 5C74226E
P 9950 2200
F 0 "R116" H 10020 2246 50  0000 L CNN
F 1 "R" H 10020 2155 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9880 2200 50  0001 C CNN
F 3 "~" H 9950 2200 50  0001 C CNN
	1    9950 2200
	0    1    1    0   
$EndComp
Wire Wire Line
	10250 1600 10250 1750
Connection ~ 10250 1600
Text Notes 8500 2750 0    50   ~ 0
/!\ the input voltage shoud not be \nunder 7V for 7805 and 6.5 for V7805-2000
Text Notes 7700 5700 0    50   ~ 0
put jumper when RX TX not used\n
$Comp
L Connector:Conn_01x03_Female J106
U 1 1 5C34BD2F
P 6850 4200
F 0 "J106" H 6877 4226 50  0000 L CNN
F 1 "Conn_01x03_Female" H 6877 4135 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 6850 4200 50  0001 C CNN
F 3 "~" H 6850 4200 50  0001 C CNN
	1    6850 4200
	1    0    0    -1  
$EndComp
Text Label 6550 4200 2    50   ~ 0
groundSensorEmitter
Text Label 6550 4100 2    50   ~ 0
5V
Text Label 6550 4300 2    50   ~ 0
arduinoGroundSensorEmitter
Wire Wire Line
	6550 4300 6650 4300
Wire Wire Line
	6550 4200 6650 4200
Wire Wire Line
	6550 4100 6650 4100
Text Notes 5300 4550 0    50   ~ 0
- use jumper between 1 and 2 for alway active ground sensor emitter\n- use jumper between 2 and 3 for arduino controlled ground sensor emitter\n(info, arduino pin can deliver 40mA, each sensor drive 17mA)
Wire Wire Line
	4300 1650 4300 2800
Wire Wire Line
	1450 2400 2650 2400
Wire Wire Line
	1450 2800 4300 2800
Wire Wire Line
	2100 1100 2100 1500
Wire Wire Line
	2100 1100 4200 1100
Wire Wire Line
	2750 1400 2850 1400
Wire Wire Line
	1450 1500 2100 1500
$Comp
L Device:LED D110
U 1 1 5C416EB4
P 8750 1700
F 0 "D110" V 8788 1583 50  0000 R CNN
F 1 "LED" V 8697 1583 50  0000 R CNN
F 2 "LEDs:LED_D5.0mm" H 8750 1700 50  0001 C CNN
F 3 "~" H 8750 1700 50  0001 C CNN
	1    8750 1700
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R117
U 1 1 5C416FDC
P 9000 2200
F 0 "R117" H 9070 2246 50  0000 L CNN
F 1 "R" H 9070 2155 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8930 2200 50  0001 C CNN
F 3 "~" H 9000 2200 50  0001 C CNN
	1    9000 2200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8950 1500 8950 1400
Connection ~ 8950 1400
Wire Wire Line
	9050 1500 9050 1750
Wire Wire Line
	9050 1750 9350 1750
Wire Wire Line
	9350 1750 9350 1950
Wire Wire Line
	9550 1450 9550 1750
Wire Wire Line
	9550 1750 9350 1750
Connection ~ 9350 1750
Wire Wire Line
	9900 1500 9900 1750
Wire Wire Line
	9900 1750 9550 1750
Connection ~ 9550 1750
Wire Wire Line
	9550 1950 9350 1950
Connection ~ 9350 1950
Wire Wire Line
	10250 2200 10100 2200
Wire Wire Line
	10250 2050 10250 2200
Wire Wire Line
	9800 2200 9350 2200
Wire Wire Line
	9350 1950 9350 2200
Connection ~ 9350 2200
Wire Wire Line
	9350 2200 9150 2200
Wire Wire Line
	9350 2200 9350 2300
Wire Wire Line
	8850 2200 8750 2200
Wire Wire Line
	8750 2200 8750 1850
Wire Wire Line
	8750 1550 8750 1500
Wire Wire Line
	8750 1500 8950 1500
$Comp
L Switch:SW_Push SW104
U 1 1 5C4D9482
P 8250 1050
F 0 "SW104" H 8250 1335 50  0000 C CNN
F 1 "SW_Push" H 8250 1244 50  0000 C CNN
F 2 "Buttons_Switches_THT:SW_CuK_OS102011MA1QN1_SPDT_Angled" H 8250 1250 50  0001 C CNN
F 3 "" H 8250 1250 50  0001 C CNN
	1    8250 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 1050 8950 1050
Wire Wire Line
	8050 1050 7850 1050
Text Notes 8100 700  0    50   ~ 0
main switch
$Comp
L power:+BATT #PWR0116
U 1 1 5C4F6393
P 7950 4800
F 0 "#PWR0116" H 7950 4650 50  0001 C CNN
F 1 "+BATT" H 7965 4973 50  0000 C CNN
F 2 "" H 7950 4800 50  0001 C CNN
F 3 "" H 7950 4800 50  0001 C CNN
	1    7950 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7950 4800 7950 4850
Connection ~ 7950 4850
Wire Wire Line
	7950 4850 8000 4850
$Comp
L power:-BATT #PWR0117
U 1 1 5C504B67
P 7950 5000
F 0 "#PWR0117" H 7950 4850 50  0001 C CNN
F 1 "-BATT" H 7965 5173 50  0000 C CNN
F 2 "" H 7950 5000 50  0001 C CNN
F 3 "" H 7950 5000 50  0001 C CNN
	1    7950 5000
	-1   0    0    1   
$EndComp
Wire Wire Line
	7950 5000 7950 4950
Connection ~ 7950 4950
Wire Wire Line
	7950 4950 7850 4950
$EndSCHEMATC
