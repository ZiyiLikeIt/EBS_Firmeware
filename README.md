# EBS_Firmeware
The BLE based Basestation project of EVRS

The Electronic Voting Response System (EVRS) is our capstone project in the UoM. It contains multiple transmitters (ETX) based on customized PCB and CC2650MODA, a base station (EBS) using CC2650 LaunchXL, a basestation controller (EBC) based on Raspberry Zero W and a PC client software (EPC) based on Java. 

The EBS is taking charge of discovering all ETX, enquiring data and reporting data to the EBC using UART

The wireless communication protocol would be Bluetooth LE.

Device

	CC2650 Launch XL Rev 1.3

Environment

	CodeComposerStudio 8.1.0.00011 
	Compiler TI v18.1.3.LTS
	tirtos_cc13xx_cc26xx_2_21_01_08
	ble_sdk_2_02_02_25
	xdctools_3_32_00_06