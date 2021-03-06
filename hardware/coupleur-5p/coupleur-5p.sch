EESchema Schematic File Version 4
EELAYER 30 0
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
L Connector:RJ45 J5
U 1 1 607C9BB5
P 7600 4450
F 0 "J5" H 7270 4546 50  0000 R CNN
F 1 "RJ45" H 7270 4455 50  0000 R CNN
F 2 "Connector_RJ:RJ45_Amphenol_54602-x08_Horizontal" V 7600 4475 50  0001 C CNN
F 3 "~" V 7600 4475 50  0001 C CNN
F 4 "54601-908WPLF" H 7600 4450 50  0001 C CNN "P/N"
	1    7600 4450
	0    1    -1   0   
$EndComp
$Comp
L Connector:RJ45 J4
U 1 1 607CBB1B
P 6500 4450
F 0 "J4" H 6170 4546 50  0000 R CNN
F 1 "RJ45" H 6170 4455 50  0000 R CNN
F 2 "Connector_RJ:RJ45_Amphenol_54602-x08_Horizontal" V 6500 4475 50  0001 C CNN
F 3 "~" V 6500 4475 50  0001 C CNN
F 4 "54601-908WPLF" H 6500 4450 50  0001 C CNN "P/N"
	1    6500 4450
	0    1    -1   0   
$EndComp
Wire Wire Line
	6400 4050 6400 3950
Connection ~ 6400 3350
Wire Wire Line
	6500 4050 6500 3950
Wire Wire Line
	6500 3950 6400 3950
Connection ~ 6400 3950
Wire Wire Line
	6400 3950 6400 3350
Wire Wire Line
	6600 4050 6600 3950
Connection ~ 6600 3250
Wire Wire Line
	6700 4050 6700 3950
Wire Wire Line
	6700 3950 6600 3950
Connection ~ 6600 3950
Wire Wire Line
	6600 3950 6600 3250
Text Label 6750 3250 0    50   ~ 0
GND
Text Label 6750 3350 0    50   ~ 0
+12V
$Comp
L Device:R_Small R1
U 1 1 6241D7A8
P 4700 3900
F 0 "R1" V 4504 3900 50  0000 C CNN
F 1 "100" V 4595 3900 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 4700 3900 50  0001 C CNN
F 3 "~" H 4700 3900 50  0001 C CNN
	1    4700 3900
	0    1    1    0   
$EndComp
$Comp
L Connector:RJ45 J1
U 1 1 607C9BAE
P 4300 4450
F 0 "J1" H 3970 4546 50  0000 R CNN
F 1 "RJ45" H 3970 4455 50  0000 R CNN
F 2 "Connector_RJ:RJ45_Amphenol_54602-x08_Horizontal" V 4300 4475 50  0001 C CNN
F 3 "~" V 4300 4475 50  0001 C CNN
F 4 "54601-908WPLF" H 4300 4450 50  0001 C CNN "P/N"
	1    4300 4450
	0    1    -1   0   
$EndComp
$Comp
L Connector:RJ45 J6
U 1 1 6242236B
P 8750 4450
F 0 "J6" H 8420 4546 50  0000 R CNN
F 1 "RJ45" H 8420 4455 50  0000 R CNN
F 2 "Connector_RJ:RJ45_Amphenol_54602-x08_Horizontal" V 8750 4475 50  0001 C CNN
F 3 "~" V 8750 4475 50  0001 C CNN
F 4 "54601-908WPLF" H 8750 4450 50  0001 C CNN "P/N"
	1    8750 4450
	0    1    -1   0   
$EndComp
$Comp
L Connector:RJ45 J3
U 1 1 624256A4
P 5400 4450
F 0 "J3" H 5070 4546 50  0000 R CNN
F 1 "RJ45" H 5070 4455 50  0000 R CNN
F 2 "Connector_RJ:RJ45_Amphenol_54602-x08_Horizontal" V 5400 4475 50  0001 C CNN
F 3 "~" V 5400 4475 50  0001 C CNN
F 4 "54601-908WPLF" H 5400 4450 50  0001 C CNN "P/N"
	1    5400 4450
	0    1    -1   0   
$EndComp
$Comp
L Device:R_Small R2
U 1 1 62425F3A
P 9150 3900
F 0 "R2" V 8954 3900 50  0000 C CNN
F 1 "100" V 9045 3900 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 9150 3900 50  0001 C CNN
F 3 "~" H 9150 3900 50  0001 C CNN
	1    9150 3900
	0    1    1    0   
$EndComp
Wire Wire Line
	4000 3550 4000 4050
Wire Wire Line
	4000 3550 4700 3550
Wire Wire Line
	4100 3450 4100 4050
Wire Wire Line
	4100 3450 4700 3450
Wire Wire Line
	4200 3350 4200 3950
Wire Wire Line
	4200 3350 4700 3350
Wire Wire Line
	4300 4050 4300 3950
Wire Wire Line
	4300 3950 4200 3950
Connection ~ 4200 3950
Wire Wire Line
	4200 3950 4200 4050
Wire Wire Line
	4400 3250 4400 3950
Wire Wire Line
	4400 3250 4700 3250
Wire Wire Line
	4400 3950 4500 3950
Wire Wire Line
	4500 3950 4500 4050
Connection ~ 4400 3950
Wire Wire Line
	4400 3950 4400 4050
Wire Wire Line
	4600 4050 4600 3900
Wire Wire Line
	4700 4050 4800 4050
Wire Wire Line
	4800 4050 4800 3900
Wire Wire Line
	5100 4050 5100 3550
Wire Wire Line
	5200 4050 5200 3450
Wire Wire Line
	6200 4050 5800 4050
Wire Wire Line
	6300 4050 6300 3950
Wire Wire Line
	6300 3950 5700 3950
Wire Wire Line
	5700 3950 5700 4050
Wire Wire Line
	7300 4050 6900 4050
Wire Wire Line
	7400 4050 7400 3950
Wire Wire Line
	7400 3950 6800 3950
Wire Wire Line
	6800 3950 6800 4050
Wire Wire Line
	8450 4050 8000 4050
Wire Wire Line
	8550 4050 8550 3950
Wire Wire Line
	8550 3950 7900 3950
Wire Wire Line
	7900 3950 7900 4050
Wire Wire Line
	9050 4050 9050 3900
Wire Wire Line
	9150 4050 9250 4050
Wire Wire Line
	9250 4050 9250 3900
Wire Wire Line
	7500 4050 7500 3950
Wire Wire Line
	6400 3350 7500 3350
Wire Wire Line
	7600 4050 7600 3950
Wire Wire Line
	7600 3950 7500 3950
Connection ~ 7500 3950
Wire Wire Line
	7500 3950 7500 3350
Wire Wire Line
	8650 4050 8650 3950
Wire Wire Line
	8650 3350 7500 3350
Connection ~ 7500 3350
Wire Wire Line
	8750 4050 8750 3950
Wire Wire Line
	8750 3950 8650 3950
Connection ~ 8650 3950
Wire Wire Line
	8650 3950 8650 3350
Wire Wire Line
	8850 4050 8850 3950
Wire Wire Line
	6600 3250 7700 3250
Wire Wire Line
	8950 4050 8950 3950
Wire Wire Line
	8950 3950 8850 3950
Connection ~ 8850 3950
Wire Wire Line
	8850 3950 8850 3250
Wire Wire Line
	7700 4050 7700 3950
Connection ~ 7700 3250
Wire Wire Line
	7700 3250 8850 3250
Wire Wire Line
	7800 4050 7800 3950
Wire Wire Line
	7800 3950 7700 3950
Connection ~ 7700 3950
Wire Wire Line
	7700 3950 7700 3250
Wire Wire Line
	5300 4050 5300 3950
Connection ~ 5300 3350
Wire Wire Line
	5300 3350 6400 3350
Wire Wire Line
	5400 4050 5400 3950
Wire Wire Line
	5400 3950 5300 3950
Connection ~ 5300 3950
Wire Wire Line
	5300 3950 5300 3350
Wire Wire Line
	5500 4050 5500 3950
Connection ~ 5500 3250
Wire Wire Line
	5500 3250 6600 3250
Wire Wire Line
	5600 4050 5600 3950
Wire Wire Line
	5600 3950 5500 3950
Connection ~ 5500 3950
Wire Wire Line
	5500 3950 5500 3250
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 62445A44
P 4900 3350
F 0 "J2" H 4980 3342 50  0000 L CNN
F 1 "Conn_01x04" H 4980 3251 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 4900 3350 50  0001 C CNN
F 3 "~" H 4900 3350 50  0001 C CNN
	1    4900 3350
	1    0    0    -1  
$EndComp
Connection ~ 4700 3350
Wire Wire Line
	4700 3350 5300 3350
Connection ~ 4700 3450
Wire Wire Line
	4700 3450 5200 3450
Connection ~ 4700 3550
Wire Wire Line
	4700 3550 5100 3550
Connection ~ 4700 3250
Wire Wire Line
	4700 3250 5500 3250
Text Label 4250 3550 0    50   ~ 0
CANH
Text Label 4250 3450 0    50   ~ 0
CANL
$EndSCHEMATC
