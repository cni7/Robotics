#ifndef SPI_COM_H
#define SPI_COM_H


typedef struct{

	signed Set_Speed_M1 :16;
	signed Set_Speed_M2 :16;
	signed Act_Speed_M1 :16;
	signed Act_Speed_M2 :16;
	signed Encoder_M1 :32;
	signed Encoder_M2 :32;                                                                            
} MotorDataType;


void Send_Read_Motor_Data(MotorDataType *);

#endif
