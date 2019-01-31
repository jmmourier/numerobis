EESchema Schematic File Version 4
LIBS:v3-cache
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
L Connector:Conn_01x06_Female MOT_L_1
U 1 1 5C2465EA
P 1250 1200
F 0 "MOT_L_1" H 1144 675 50  0000 C CNN
F 1 "Conn_01x06_Female" H 1144 766 50  0000 C CNN
F 2 "Connectors_Molex:Molex_KK-6410-06_06x2.54mm_Straight" H 1250 1200 50  0001 C CNN
F 3 "~" H 1250 1200 50  0001 C CNN
	1    1250 1200
	-1   0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x06_Female MOT_R_1
U 1 1 5C24661A
P 1250 2500
F 0 "MOT_R_1" H 1144 1975 50  0000 C CNN
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
Text Notes 3950 5800 0    50   ~ 0
VL53L0X - 1\n
Text Notes 1800 3200 0    50   ~ 0
ground sensor\n1 et 2
Text Label 2850 3450 0    50   ~ 0
groundSensor1
Text Label 1350 3450 0    50   ~ 0
groundSensor2
Text Label 2850 3550 0    50   ~ 0
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
Text Label 2000 5350 2    50   ~ 0
TOF2_GPIO1
Text Label 2000 5250 2    50   ~ 0
TOF2_XSHUT
Text Label 2000 5150 2    50   ~ 0
TOF_SCL
Text Label 2000 5050 2    50   ~ 0
TOF_SDA
Text Label 2000 4850 2    50   ~ 0
TOF_VIN
Text Label 2000 4750 2    50   ~ 0
TOF2_VDD
Text Notes 750  4850 0    50   ~ 0
VL53L0X - 2\n
Text Label 3600 5350 2    50   ~ 0
TOF1_GPIO1
Text Label 3600 5250 2    50   ~ 0
TOF1_XSHUT
Text Label 3600 5150 2    50   ~ 0
TOF_SCL
Text Label 3600 5050 2    50   ~ 0
TOF_SDA
Text Label 3600 4850 2    50   ~ 0
TOF_VIN
Text Label 3600 4750 2    50   ~ 0
TOF1_VDD
Text Notes 9100 2900 0    50   ~ 0
interface
Text Label 9550 5950 2    50   ~ 0
Led1
Text Label 9550 6300 2    50   ~ 0
Led2
Text Label 10700 3550 0    50   ~ 0
Button1
Text Label 10050 3550 0    50   ~ 0
Button2
Text Label 9400 3550 0    50   ~ 0
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
F 2 "mini victory l298n - H bridge driver:miniVictoryL298N" H 3350 1100 50  0001 C CNN
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
L QTR-1A:QTR-1A QTR_2
U 1 1 5C2BF47E
P 2400 3350
F 0 "QTR_2" H 2431 3515 50  0000 C CNN
F 1 "QTR-1A" H 2431 3424 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x03_Pitch2.54mm" H 2400 3350 50  0001 C CNN
F 3 "" H 2400 3350 50  0001 C CNN
	1    2400 3350
	1    0    0    -1  
$EndComp
$Comp
L QTR-1A:QTR-1A QTR_1
U 1 1 5C2BF51E
P 1000 3350
F 0 "QTR_1" H 1031 3515 50  0000 C CNN
F 1 "QTR-1A" H 1031 3424 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x03_Pitch2.54mm" H 1000 3350 50  0001 C CNN
F 3 "" H 1000 3350 50  0001 C CNN
	1    1000 3350
	1    0    0    -1  
$EndComp
Text Label 1350 3550 0    50   ~ 0
groundSensorEmitter
Text Label 7350 3000 0    50   ~ 0
groundSensor2
Text Label 7350 2900 0    50   ~ 0
groundSensor1
Text Label 7350 1700 0    50   ~ 0
arduinoGroundSensorEmitter
Wire Wire Line
	1250 3450 1350 3450
Wire Wire Line
	1250 3550 1350 3550
Wire Wire Line
	2650 3450 2850 3450
Wire Wire Line
	2650 3550 2850 3550
Wire Wire Line
	1250 3650 1700 3650
Wire Wire Line
	2050 3650 2050 3900
Wire Wire Line
	2050 3900 2750 3900
Wire Wire Line
	2750 3900 2750 3650
Wire Wire Line
	2750 3650 2650 3650
$Comp
L power:GND #PWR0106
U 1 1 5C2C1C26
P 1700 3750
F 0 "#PWR0106" H 1700 3500 50  0001 C CNN
F 1 "GND" H 1705 3577 50  0000 C CNN
F 2 "" H 1700 3750 50  0001 C CNN
F 3 "" H 1700 3750 50  0001 C CNN
	1    1700 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 3650 1700 3750
Connection ~ 1700 3650
Wire Wire Line
	1700 3650 2050 3650
$Comp
L VL53L0X:VL53L0X_module VL53_2
U 1 1 5C2C35D5
P 2250 4650
F 0 "VL53_2" H 2250 4750 50  0000 L CNN
F 1 "VL53L0X_module" H 2050 4850 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x07_Pitch2.54mm" H 2250 4650 50  0001 C CNN
F 3 "" H 2250 4650 50  0001 C CNN
	1    2250 4650
	1    0    0    -1  
$EndComp
$Comp
L VL53L0X:VL53L0X_module VL53_1
U 1 1 5C2C3643
P 3900 4650
F 0 "VL53_1" H 4000 4750 50  0000 L CNN
F 1 "VL53L0X_module" H 3750 4850 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x07_Pitch2.54mm" H 3900 4650 50  0001 C CNN
F 3 "" H 3900 4650 50  0001 C CNN
	1    3900 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 4850 3750 4850
Text Label 10500 1200 0    50   ~ 0
TOF_VIN
Text Label 7350 3300 0    50   ~ 0
TOF_SDA
Text Label 7350 3400 0    50   ~ 0
TOF_SCL
Wire Wire Line
	3600 5050 3750 5050
Wire Wire Line
	3600 5150 3750 5150
$Comp
L power:GND #PWR0107
U 1 1 5C2CFB98
P 3150 4950
F 0 "#PWR0107" H 3150 4700 50  0001 C CNN
F 1 "GND" H 3155 4777 50  0000 C CNN
F 2 "" H 3150 4950 50  0001 C CNN
F 3 "" H 3150 4950 50  0001 C CNN
	1    3150 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 4950 3750 4950
NoConn ~ 3100 5350
Wire Wire Line
	3100 5350 3750 5350
NoConn ~ 3150 4750
Wire Wire Line
	3150 4750 3750 4750
NoConn ~ 1550 4750
Wire Wire Line
	1550 4750 2100 4750
$Comp
L power:GND #PWR0108
U 1 1 5C2D91EA
P 1500 4950
F 0 "#PWR0108" H 1500 4700 50  0001 C CNN
F 1 "GND" H 1505 4777 50  0000 C CNN
F 2 "" H 1500 4950 50  0001 C CNN
F 3 "" H 1500 4950 50  0001 C CNN
	1    1500 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 4850 2100 4850
Wire Wire Line
	2000 5050 2100 5050
Wire Wire Line
	2100 4950 1500 4950
Wire Wire Line
	2000 5150 2100 5150
NoConn ~ 1500 5350
Wire Wire Line
	1500 5350 2100 5350
Text Notes 5550 6150 0    50   ~ 0
TODO\n
$Comp
L Switch:SW_Push BUTT2
U 1 1 5C2E5C52
P 9900 3850
F 0 "BUTT2" V 9854 3998 50  0000 L CNN
F 1 "SW_Push" V 9945 3998 50  0000 L CNN
F 2 "Buttons_Switches_THT:SW_PUSH_6mm_h4.3mm" H 9900 4050 50  0001 C CNN
F 3 "" H 9900 4050 50  0001 C CNN
	1    9900 3850
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push BUTT1
U 1 1 5C2E5C94
P 10550 3850
F 0 "BUTT1" V 10504 3998 50  0000 L CNN
F 1 "SW_Push" V 10595 3998 50  0000 L CNN
F 2 "Buttons_Switches_THT:SW_PUSH_6mm_h4.3mm" H 10550 4050 50  0001 C CNN
F 3 "" H 10550 4050 50  0001 C CNN
	1    10550 3850
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push reset1
U 1 1 5C2E5D64
P 9250 3850
F 0 "reset1" V 9204 3998 50  0000 L CNN
F 1 "SW_Push" V 9295 3998 50  0000 L CNN
F 2 "Buttons_Switches_THT:SW_PUSH_6mm_h4.3mm" H 9250 4050 50  0001 C CNN
F 3 "" H 9250 4050 50  0001 C CNN
	1    9250 3850
	0    1    1    0   
$EndComp
$Comp
L Device:R R103
U 1 1 5C2E8244
P 10550 3350
F 0 "R103" H 10620 3396 50  0000 L CNN
F 1 "R" H 10620 3305 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10480 3350 50  0001 C CNN
F 3 "~" H 10550 3350 50  0001 C CNN
	1    10550 3350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R102
U 1 1 5C2E827E
P 9900 3350
F 0 "R102" H 9970 3396 50  0000 L CNN
F 1 "R" H 9970 3305 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9830 3350 50  0001 C CNN
F 3 "~" H 9900 3350 50  0001 C CNN
	1    9900 3350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R101
U 1 1 5C2E8351
P 9250 3350
F 0 "R101" H 9320 3396 50  0000 L CNN
F 1 "R" H 9320 3305 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9180 3350 50  0001 C CNN
F 3 "~" H 9250 3350 50  0001 C CNN
	1    9250 3350
	1    0    0    -1  
$EndComp
Text Label 9650 3000 0    50   ~ 0
VIN_Button
Text Label 10500 1300 0    50   ~ 0
VIN_Button
Wire Wire Line
	9650 3000 9650 3100
Wire Wire Line
	9650 3100 9250 3100
Wire Wire Line
	9250 3100 9250 3200
Wire Wire Line
	9650 3100 9900 3100
Wire Wire Line
	9900 3100 9900 3200
Connection ~ 9650 3100
Wire Wire Line
	9900 3100 10550 3100
Wire Wire Line
	10550 3100 10550 3200
Connection ~ 9900 3100
Wire Wire Line
	9250 3500 9250 3550
Wire Wire Line
	9250 3550 9400 3550
Wire Wire Line
	9900 3500 9900 3550
Wire Wire Line
	9900 3550 10050 3550
Wire Wire Line
	10550 3500 10550 3550
Wire Wire Line
	10550 3550 10700 3550
Wire Wire Line
	10550 3550 10550 3650
Connection ~ 10550 3550
Wire Wire Line
	9900 3550 9900 3650
Connection ~ 9900 3550
Wire Wire Line
	9250 3550 9250 3650
Connection ~ 9250 3550
$Comp
L power:GND #PWR0109
U 1 1 5C2F38E5
P 9950 4200
F 0 "#PWR0109" H 9950 3950 50  0001 C CNN
F 1 "GND" H 9955 4027 50  0000 C CNN
F 2 "" H 9950 4200 50  0001 C CNN
F 3 "" H 9950 4200 50  0001 C CNN
	1    9950 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9950 4200 9950 4100
Wire Wire Line
	9950 4100 9900 4100
Wire Wire Line
	9900 4100 9900 4050
Wire Wire Line
	9900 4100 9250 4100
Wire Wire Line
	9250 4100 9250 4050
Connection ~ 9900 4100
Wire Wire Line
	9950 4100 10550 4100
Wire Wire Line
	10550 4100 10550 4050
Connection ~ 9950 4100
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
Text Label 10050 4650 2    50   ~ 0
Button2
Text Label 6300 3950 2    50   ~ 0
Button1
Text Label 7350 2700 0    50   ~ 0
Led1
Text Label 10050 4750 2    50   ~ 0
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
L Device:LED LED1
U 1 1 5C2FEF7E
P 10250 5950
F 0 "LED1" H 10242 5695 50  0000 C CNN
F 1 "LED" H 10242 5786 50  0000 C CNN
F 2 "LEDs:LED_D5.0mm" H 10250 5950 50  0001 C CNN
F 3 "~" H 10250 5950 50  0001 C CNN
	1    10250 5950
	-1   0    0    1   
$EndComp
$Comp
L Device:LED LED2
U 1 1 5C2FEFEA
P 10250 6300
F 0 "LED2" H 10242 6045 50  0000 C CNN
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
L Connector_Generic:Conn_02x03_Odd_Even ext1
U 1 1 5C3198A8
P 10400 5300
F 0 "ext1" H 10450 5617 50  0000 C CNN
F 1 "Conn_02x03_Odd_Even" H 10450 5526 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03_Pitch2.54mm" H 10400 5300 50  0001 C CNN
F 3 "~" H 10400 5300 50  0001 C CNN
	1    10400 5300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0113
U 1 1 5C319AE0
P 10050 5450
F 0 "#PWR0113" H 10050 5200 50  0001 C CNN
F 1 "GND" H 10055 5277 50  0000 C CNN
F 2 "" H 10050 5450 50  0001 C CNN
F 3 "" H 10050 5450 50  0001 C CNN
	1    10050 5450
	1    0    0    -1  
$EndComp
Text Label 7350 3500 0    50   ~ 0
distanceSensor3
Text Label 7350 3200 0    50   ~ 0
distanceSensor2
Text Label 7350 3100 0    50   ~ 0
distanceSensor1
Text Label 10500 1400 0    50   ~ 0
alimDistanceSensor
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
Text Label 700  5200 0    50   ~ 0
TOF2_XSHUT_CTRL
Wire Wire Line
	1350 5250 2100 5250
Text Label 7350 2600 0    50   ~ 0
TOF2_XSHUT_CTRL
Wire Wire Line
	7350 2600 7250 2600
Text Notes 10500 1100 0    50   ~ 0
5V
$Comp
L Connector:Conn_01x02_Female BATT1
U 1 1 5C47F120
P 8750 6050
F 0 "BATT1" H 8777 6026 50  0000 L CNN
F 1 "Conn_01x02_Female" H 8777 5935 50  0000 L CNN
F 2 "Connectors_Molex:Molex_KK-6410-02_02x2.54mm_Straight" H 8750 6050 50  0001 C CNN
F 3 "~" H 8750 6050 50  0001 C CNN
	1    8750 6050
	1    0    0    -1  
$EndComp
Text Notes 8650 5950 0    50   ~ 0
battery
Text Label 8400 6050 2    50   ~ 0
batterieP
Text Label 8400 6150 2    50   ~ 0
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
P 10400 4650
F 0 "J104" H 10450 4867 50  0000 C CNN
F 1 "Conn_02x02_Odd_Even" H 10450 4776 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x02_Pitch2.54mm" H 10400 4650 50  0001 C CNN
F 3 "~" H 10400 4650 50  0001 C CNN
	1    10400 4650
	1    0    0    -1  
$EndComp
Text Label 10800 4650 0    50   ~ 0
RX
Text Label 10800 4750 0    50   ~ 0
TX
Wire Wire Line
	10800 4750 10700 4750
Wire Wire Line
	10800 4650 10700 4650
Wire Wire Line
	10200 4650 10050 4650
Wire Wire Line
	10050 4750 10200 4750
Wire Wire Line
	10050 5450 10050 5400
Wire Wire Line
	10050 5400 10200 5400
Text Label 10100 5200 2    50   ~ 0
RX
Text Label 10100 5300 2    50   ~ 0
TX
Text Label 10850 5200 0    50   ~ 0
batterieP
$Comp
L power:GND #PWR0112
U 1 1 5C5B31B4
P 10900 5400
F 0 "#PWR0112" H 10900 5150 50  0001 C CNN
F 1 "GND" H 10905 5227 50  0000 C CNN
F 2 "" H 10900 5400 50  0001 C CNN
F 3 "" H 10900 5400 50  0001 C CNN
	1    10900 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	10700 5400 10900 5400
Wire Wire Line
	10100 5300 10200 5300
Wire Wire Line
	10100 5200 10200 5200
Wire Wire Line
	10700 5200 10800 5200
NoConn ~ 3100 5250
Wire Wire Line
	3100 5250 3750 5250
Wire Wire Line
	10700 5300 10800 5300
Wire Wire Line
	10800 5300 10800 5200
Connection ~ 10800 5200
Wire Wire Line
	10800 5200 10850 5200
Text Label 4250 7000 0    50   ~ 0
TOF_SDA
Text Label 4250 7100 0    50   ~ 0
TOF_SCL
$Comp
L Device:R R105
U 1 1 5C61CC33
P 4100 6850
F 0 "R105" H 4170 6896 50  0000 L CNN
F 1 "2.2k" H 4170 6805 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4030 6850 50  0001 C CNN
F 3 "~" H 4100 6850 50  0001 C CNN
	1    4100 6850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R104
U 1 1 5C61CC9F
P 3700 6850
F 0 "R104" H 3770 6896 50  0000 L CNN
F 1 "2.2k" H 3770 6805 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3630 6850 50  0001 C CNN
F 3 "~" H 3700 6850 50  0001 C CNN
	1    3700 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 7000 4250 7000
Text Label 10500 1600 0    50   ~ 0
5V
Wire Wire Line
	10500 1600 10250 1600
Wire Wire Line
	10250 1600 10250 1500
Connection ~ 10250 1500
Text Label 4000 6600 0    50   ~ 0
5V
Wire Wire Line
	4000 6600 4000 6650
Wire Wire Line
	4000 6650 4100 6650
Wire Wire Line
	4100 6650 4100 6700
Connection ~ 4000 6650
Text Notes 3900 6450 0    50   ~ 0
pull up I2C
Wire Wire Line
	3700 6700 3700 6650
Wire Wire Line
	3700 6650 4000 6650
Wire Wire Line
	3700 7000 3700 7100
Wire Wire Line
	3700 7100 4250 7100
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
	8400 6050 8500 6050
Wire Wire Line
	8550 6150 8500 6150
$Comp
L Device:LED 5V_1
U 1 1 5C7420E5
P 10250 2000
F 0 "5V_1" V 10288 1883 50  0000 R CNN
F 1 "LED" V 10197 1883 50  0000 R CNN
F 2 "LEDs:LED_D5.0mm" H 10250 2000 50  0001 C CNN
F 3 "~" H 10250 2000 50  0001 C CNN
	1    10250 2000
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
	10250 1600 10250 1700
Connection ~ 10250 1600
Text Notes 8500 2750 0    50   ~ 0
/!\ the input voltage shoud not be \nunder 7V for 7805 and 6.5 for V7805-2000
Text Notes 9800 4900 0    50   ~ 0
put jumper when RX TX not used\n
$Comp
L Connector:Conn_01x03_Female J106
U 1 1 5C34BD2F
P 4250 3850
F 0 "J106" H 4277 3876 50  0000 L CNN
F 1 "Conn_01x03_Female" H 4277 3785 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 4250 3850 50  0001 C CNN
F 3 "~" H 4250 3850 50  0001 C CNN
	1    4250 3850
	1    0    0    -1  
$EndComp
Text Label 3950 3850 2    50   ~ 0
groundSensorEmitter
Text Label 3950 3750 2    50   ~ 0
5V
Text Label 3950 3950 2    50   ~ 0
arduinoGroundSensorEmitter
Wire Wire Line
	3950 3950 4050 3950
Wire Wire Line
	3950 3850 4050 3850
Wire Wire Line
	3950 3750 4050 3750
Text Notes 2600 4250 0    50   ~ 0
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
L Device:LED POWER_1
U 1 1 5C416EB4
P 8750 1700
F 0 "POWER_1" V 8788 1583 50  0000 R CNN
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
L Switch:SW_Push PWR_1
U 1 1 5C4D9482
P 8250 1050
F 0 "PWR_1" H 8250 1335 50  0000 C CNN
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
P 8500 6000
F 0 "#PWR0116" H 8500 5850 50  0001 C CNN
F 1 "+BATT" H 8515 6173 50  0000 C CNN
F 2 "" H 8500 6000 50  0001 C CNN
F 3 "" H 8500 6000 50  0001 C CNN
	1    8500 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 6000 8500 6050
Connection ~ 8500 6050
Wire Wire Line
	8500 6050 8550 6050
$Comp
L power:-BATT #PWR0117
U 1 1 5C504B67
P 8500 6200
F 0 "#PWR0117" H 8500 6050 50  0001 C CNN
F 1 "-BATT" H 8515 6373 50  0000 C CNN
F 2 "" H 8500 6200 50  0001 C CNN
F 3 "" H 8500 6200 50  0001 C CNN
	1    8500 6200
	-1   0    0    1   
$EndComp
Wire Wire Line
	8500 6200 8500 6150
Connection ~ 8500 6150
Wire Wire Line
	8500 6150 8400 6150
$Comp
L Device:Battery BT101
U 1 1 5C4D88AA
P 7850 6100
F 0 "BT101" H 7958 6146 50  0000 L CNN
F 1 "Battery" H 7958 6055 50  0000 L CNN
F 2 "" V 7850 6160 50  0001 C CNN
F 3 "~" V 7850 6160 50  0001 C CNN
	1    7850 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 5900 8400 5900
Wire Wire Line
	8400 5900 8400 6050
Wire Wire Line
	8400 6150 8400 6300
Wire Wire Line
	8400 6300 7850 6300
$Comp
L GP2Y0D810Z0F:GP2Y0D810Z0F SHARP1
U 1 1 5C535FAC
P 5350 5200
F 0 "SHARP1" H 5331 5375 50  0000 C CNN
F 1 "GP2Y0D810Z0F" H 5331 5284 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 5350 5200 50  0001 C CNN
F 3 "" H 5350 5200 50  0001 C CNN
	1    5350 5200
	1    0    0    -1  
$EndComp
$Comp
L GP2Y0D810Z0F:GP2Y0D810Z0F SHARP2
U 1 1 5C5360AC
P 6500 5200
F 0 "SHARP2" H 6481 5375 50  0000 C CNN
F 1 "GP2Y0D810Z0F" H 6481 5284 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 6500 5200 50  0001 C CNN
F 3 "" H 6500 5200 50  0001 C CNN
	1    6500 5200
	1    0    0    -1  
$EndComp
$Comp
L GP2Y0D810Z0F:GP2Y0D810Z0F SHARP3
U 1 1 5C536158
P 7550 5200
F 0 "SHARP3" H 7531 5375 50  0000 C CNN
F 1 "GP2Y0D810Z0F" H 7531 5284 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 7550 5200 50  0001 C CNN
F 3 "" H 7550 5200 50  0001 C CNN
	1    7550 5200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 5C5362F3
P 8250 5650
F 0 "#PWR0110" H 8250 5400 50  0001 C CNN
F 1 "GND" H 8255 5477 50  0000 C CNN
F 2 "" H 8250 5650 50  0001 C CNN
F 3 "" H 8250 5650 50  0001 C CNN
	1    8250 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 5650 8250 5600
Wire Wire Line
	8250 5450 7750 5450
Wire Wire Line
	6700 5450 6700 5600
Wire Wire Line
	6700 5600 8250 5600
Connection ~ 8250 5600
Wire Wire Line
	8250 5600 8250 5450
Wire Wire Line
	5550 5450 5550 5600
Wire Wire Line
	5550 5600 6700 5600
Connection ~ 6700 5600
Text Label 10500 1700 0    50   ~ 0
alimSharp
Wire Wire Line
	10500 1700 10250 1700
Connection ~ 10250 1700
Text Label 6700 4600 0    50   ~ 0
alimSharp
Wire Wire Line
	6700 4600 6700 4750
Wire Wire Line
	6700 4750 5800 4750
Wire Wire Line
	5800 4750 5800 5350
Wire Wire Line
	5800 5350 5550 5350
Wire Wire Line
	6700 4750 7100 4750
Wire Wire Line
	8000 4750 8000 5350
Wire Wire Line
	8000 5350 7750 5350
Connection ~ 6700 4750
Wire Wire Line
	6700 5350 7100 5350
Wire Wire Line
	7100 5350 7100 4750
Connection ~ 7100 4750
Wire Wire Line
	7100 4750 8000 4750
Text Label 5600 5250 0    50   ~ 0
distanceSensor1
Text Label 6750 5250 0    50   ~ 0
distanceSensor2
Text Label 7800 5250 0    50   ~ 0
distanceSensor3
Wire Wire Line
	5550 5250 5600 5250
Wire Wire Line
	6700 5250 6750 5250
Wire Wire Line
	7750 5250 7800 5250
$Comp
L VL53L0X:VL53L0X_module VL53_3
U 1 1 5C669E61
P 2250 6050
F 0 "VL53_3" H 2250 6150 50  0000 L CNN
F 1 "VL53L0X_module" H 2050 6250 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x07_Pitch2.54mm" H 2250 6050 50  0001 C CNN
F 3 "" H 2250 6050 50  0001 C CNN
	1    2250 6050
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:PN2222A Q102
U 1 1 5C66A6EB
P 1200 6850
F 0 "Q102" H 1391 6896 50  0000 L CNN
F 1 "PN2222A" H 1391 6805 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Wide" H 1400 6775 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/PN/PN2222A.pdf" H 1200 6850 50  0001 L CNN
	1    1200 6850
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:PN2222A Q101
U 1 1 5C6FB04E
P 1250 5450
F 0 "Q101" H 1441 5496 50  0000 L CNN
F 1 "PN2222A" H 1441 5405 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Wide" H 1450 5375 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/PN/PN2222A.pdf" H 1250 5450 50  0001 L CNN
	1    1250 5450
	1    0    0    -1  
$EndComp
Text Label 2000 6150 2    50   ~ 0
TOF3_VDD
Text Label 2000 6250 2    50   ~ 0
TOF_VIN
Text Label 2000 6450 2    50   ~ 0
TOF_SDA
Text Label 2000 6550 2    50   ~ 0
TOF_SCL
Text Label 2000 6650 2    50   ~ 0
TOF3_XSHUT
Wire Wire Line
	1300 6650 2100 6650
Wire Wire Line
	2100 6550 2000 6550
Wire Wire Line
	2100 6450 2000 6450
$Comp
L power:GND #PWR0115
U 1 1 5C734B83
P 1350 6350
F 0 "#PWR0115" H 1350 6100 50  0001 C CNN
F 1 "GND" H 1355 6177 50  0000 C CNN
F 2 "" H 1350 6350 50  0001 C CNN
F 3 "" H 1350 6350 50  0001 C CNN
	1    1350 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 6350 2100 6350
Wire Wire Line
	2100 6250 2000 6250
NoConn ~ 1500 6150
Wire Wire Line
	1500 6150 2100 6150
$Comp
L power:GND #PWR0118
U 1 1 5C7802E3
P 1300 7100
F 0 "#PWR0118" H 1300 6850 50  0001 C CNN
F 1 "GND" H 1305 6927 50  0000 C CNN
F 2 "" H 1300 7100 50  0001 C CNN
F 3 "" H 1300 7100 50  0001 C CNN
	1    1300 7100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0114
U 1 1 5C78033C
P 1350 5700
F 0 "#PWR0114" H 1350 5450 50  0001 C CNN
F 1 "GND" H 1355 5527 50  0000 C CNN
F 2 "" H 1350 5700 50  0001 C CNN
F 3 "" H 1350 5700 50  0001 C CNN
	1    1350 5700
	1    0    0    -1  
$EndComp
Text Label 600  6600 0    50   ~ 0
TOF3_XSHUT_CTRL
Text Label 7350 2200 0    50   ~ 0
TOF3_XSHUT_CTRL
$Comp
L Device:R R109
U 1 1 5C78FA26
P 800 6850
F 0 "R109" V 593 6850 50  0000 C CNN
F 1 "R" V 684 6850 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 730 6850 50  0001 C CNN
F 3 "~" H 800 6850 50  0001 C CNN
	1    800  6850
	0    1    1    0   
$EndComp
$Comp
L Device:R R106
U 1 1 5C78FABF
P 850 5450
F 0 "R106" V 643 5450 50  0000 C CNN
F 1 "R" V 734 5450 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 780 5450 50  0001 C CNN
F 3 "~" H 850 5450 50  0001 C CNN
	1    850  5450
	0    1    1    0   
$EndComp
Wire Wire Line
	700  5200 600  5200
Wire Wire Line
	600  5200 600  5450
Wire Wire Line
	600  5450 700  5450
Wire Wire Line
	600  6600 600  6850
Wire Wire Line
	600  6850 650  6850
Wire Wire Line
	950  6850 1000 6850
Wire Wire Line
	1000 5450 1050 5450
Wire Wire Line
	1350 5650 1350 5700
Wire Wire Line
	1300 7050 1300 7100
Text Notes 750  6300 0    50   ~ 0
VL53L0X - 3\n
NoConn ~ 1950 6750
Wire Wire Line
	1950 6750 2100 6750
Text Label 7350 3600 0    50   ~ 0
interface1
Text Label 6300 4050 2    50   ~ 0
interface1
Text Label 6300 4150 2    50   ~ 0
IR_start
$Comp
L Connector_Generic:Conn_01x03 JumperInterface1
U 1 1 5C53FC5E
P 6550 4050
F 0 "JumperInterface1" H 6629 4092 50  0000 L CNN
F 1 "Conn_01x03" H 6629 4001 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 6550 4050 50  0001 C CNN
F 3 "~" H 6550 4050 50  0001 C CNN
	1    6550 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 3950 6350 3950
Wire Wire Line
	6300 4050 6350 4050
Wire Wire Line
	6300 4150 6350 4150
$Comp
L Connector_Generic:Conn_01x03 MicroStart1
U 1 1 5C584F71
P 8250 4050
F 0 "MicroStart1" H 8330 4092 50  0000 L CNN
F 1 "Conn_01x03" H 8330 4001 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 8250 4050 50  0001 C CNN
F 3 "~" H 8250 4050 50  0001 C CNN
	1    8250 4050
	1    0    0    -1  
$EndComp
Text Label 7950 3950 2    50   ~ 0
IR_start
Wire Wire Line
	10250 1700 10250 1800
Wire Wire Line
	10250 2200 10250 2150
Text Label 10500 1800 0    50   ~ 0
5V_IR
Wire Wire Line
	10500 1800 10250 1800
Connection ~ 10250 1800
Wire Wire Line
	10250 1800 10250 1850
Text Label 7950 4150 2    50   ~ 0
5V_IR
$Comp
L power:GND #PWR0119
U 1 1 5C5B988A
P 7650 4050
F 0 "#PWR0119" H 7650 3800 50  0001 C CNN
F 1 "GND" H 7655 3877 50  0000 C CNN
F 2 "" H 7650 4050 50  0001 C CNN
F 3 "" H 7650 4050 50  0001 C CNN
	1    7650 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7950 3950 8050 3950
Wire Wire Line
	8050 4050 7650 4050
Wire Wire Line
	8050 4150 7950 4150
$EndSCHEMATC
