#ifndef __QMC5883P_REG_H__
#define __QMC5883P_REG_H__

#define QMC5883P_ADDRESS 							0x2C

//normal mode: continuously make measurements in selecting output rate(ODR register) and selecting range


#define QMC5883P_REG_MODE_CONFIG 			    0x0A
#define QMC5883P_MODE_Normal					0x01
#define QMC5883P_MODE_Single					0x02
#define QMC5883P_MODE_Continue				    0x03
#define QMC5883P_MODE_Suspend					0x00

#define QMC5883P_REG_CHIPID_CONFIG		        0x00		//default value is 0x80

#define QMC5883P_REG_XLSB 						0x01
#define QMC5883P_REG_XMSB 						0x02

#define QMC5883P_REG_YLSB 						0x03
#define QMC5883P_REG_YMSB 						0x04


#define QMC5883P_REG_ZLSB 						0x05
#define QMC5883P_REG_ZMSB 						0x06

#define QMC5883P_REG_Status						0x09
/*
Data Ready Register (DRDY), it is set when all three-axis data is ready and loaded to the output data registers in 
each mode. It is reset to ¡°0¡± by reading the status register through I2C commands 
DRDY: ¡°0¡±: no new data, ¡°1¡±: new data is ready
*/


//the <1:0> of 0x0A is selcting ODR
#define QMC5883P_REG_Ctrl1			 			0x0A

//the <1:0> of 0x0B is selcting RNG
#define QMC5883P_REG_Ctrl2			 			0x0B



#endif
