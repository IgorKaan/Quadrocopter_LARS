/*
 * MPU9250.c
 *
 *  Created on: Feb 28, 2019
 *      Author: Desert
 */

#include "MPU9250.h"
#include "spi.h"

const uint32_t _i2cRate = 400000;

extern int32_t gyro_bias[3], accel_bias[3];
extern int32_t accel_bias_reg[3];
extern volatile uint32_t sysTick_Time;
extern float fGX_Cal;
extern float fGY_Cal;
extern float fGZ_Cal;
extern float gyroX;
extern float gyroY;
extern float gyroZ;
extern float accelX;
extern float accelY;
extern float accelZ;
extern float gyroX_filtered;
extern float gyroY_filtered;
extern float gyroZ_filtered;
extern float accelX_filtered;
extern float accelY_filtered;
extern float accelZ_filtered;

uint8_t acc_ofset_data[12];
uint8_t acc_ofset_data_corrected[12];

// переменные для калмана
float varVolt = 0; // среднее отклонение (расчет в программе)
float varProcess = 0.8; // скорость реакции на изменение (подбирается вручную)
float Pc = 0.0;
float G = 0.0;
float P = 1.0;
float Xp = 0.0;
float Zp = 0.0;
float Xe = 0.0;
uint8_t test = 0x00;
uint8_t value = 0;

float K = 0.1;

uint8_t _buffer[21];

static uint8_t _mag_adjust[3];

float expRunningAverageGX(float newVal) {
  static float filVal = 0;
  filVal += (newVal - filVal) * K;
  return filVal;
}

float expRunningAverageGY(float newVal) {
  static float filVal = 0;
  filVal += (newVal - filVal) * K;
  return filVal;
}

float expRunningAverageGZ(float newVal) {
  static float filVal = 0;
  filVal += (newVal - filVal) * K;
  return filVal;
}

float expRunningAverageAX(float newVal) {
  static float filVal = 0;
  filVal += (newVal - filVal) * K;
  return filVal;
}

float expRunningAverageAY(float newVal) {
  static float filVal = 0;
  filVal += (newVal - filVal) * K;
  return filVal;
}

float expRunningAverageAZ(float newVal) {
  static float filVal = 0;
  filVal += (newVal - filVal) * K;
  return filVal;
}

float filter(float val) { //функция фильтрации
	Pc = P + varProcess;
	G = Pc/(Pc + varVolt);
	P = (1-G)*Pc;
	Xp = Xe;
	Zp = Xp;
	Xe = G*(val-Zp)+Xp; // "фильтрованное" значение
return(Xe);
}

__weak void MPU9250_OnActivate()
{
}

static inline void MPU9250_Activate()
{
	MPU9250_OnActivate();
	HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_RESET);
}

static inline void MPU9250_Deactivate()
{
	HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_SET);
}

uint8_t SPIx_WriteRead(uint8_t Byte)
{
	uint8_t receivedbyte = 0;
	if(HAL_SPI_TransmitReceive(&hspi1,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x1000)!=HAL_OK)
	{
		return -1;
	}
	else
	{
	}
	return receivedbyte;
}

void MPU_SPI_Write (uint8_t *p_buffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
	MPU9250_Activate();
	SPIx_WriteRead(WriteAddr);
	while(NumByteToWrite>=0x01)
	{
		SPIx_WriteRead(*p_buffer);
		NumByteToWrite--;
		p_buffer++;
	}
	MPU9250_Deactivate();
}

void MPU_SPI_Read(uint8_t *p_buffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	MPU9250_Activate();
	uint8_t data = ReadAddr | READWRITE_CMD;
	HAL_SPI_Transmit(&MPU9250_SPI, &data, 1, HAL_MAX_DELAY);
	if (HAL_SPI_Receive(&MPU9250_SPI, p_buffer, NumByteToRead, HAL_MAX_DELAY) == HAL_OK) {
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);


	}
	else {
//		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	}
	MPU9250_Deactivate();
}

/* writes a byte to MPU9250 register given a register address and data */
void writeRegister(uint8_t subAddress, uint8_t data)
{
	MPU_SPI_Write(&data, subAddress, 1);
	HAL_Delay(10);
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
	MPU_SPI_Read(dest, subAddress, count);
}

/* writes a register to the AK8963 given a register address and data */
void writeAK8963Register(uint8_t subAddress, uint8_t data)
{
	// set slave 0 to the AK8963 and set for write
	writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR);

	// set the register to the desired AK8963 sub address
	writeRegister(I2C_SLV0_REG,subAddress);

	// store the data for write
	writeRegister(I2C_SLV0_DO,data);

	// enable I2C and send 1 byte
	writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1);
}

/* reads registers from the AK8963 */
void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
	// set slave 0 to the AK8963 and set for read
	writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG);

	// set the register to the desired AK8963 sub address
	writeRegister(I2C_SLV0_REG,subAddress);

	// enable I2C and request the bytes
	writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | count);

	// takes some time for these registers to fill
	HAL_Delay(1);

	// read the bytes off the MPU9250 EXT_SENS_DATA registers
	readRegisters(EXT_SENS_DATA_00,count,dest);
}

/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
static uint8_t whoAmI(){
	// read the WHO AM I register
	readRegisters(WHO_AM_I, 1, _buffer);

	// return the register value
	return _buffer[0];
}

/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
static int whoAmIAK8963(){
	// read the WHO AM I register
	readAK8963Registers(AK8963_WHO_AM_I, 1, _buffer);
	// return the register value
	return _buffer[0];
}

/* starts communication with the MPU-9250 */
uint8_t MPU9250_Init()
{
	// select clock source to gyro
	writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL);
	// enable I2C master mode
	writeRegister(USER_CTRL, I2C_MST_EN);
	// set the I2C bus speed to 400 kHz
	writeRegister(I2C_MST_CTRL, I2C_MST_CLK);

	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);
	// reset the MPU9250
	writeRegister(PWR_MGMNT_1, PWR_RESET);
	// wait for MPU-9250 to come back up
	HAL_Delay(10);
	// reset the AK8963
	writeAK8963Register(AK8963_CNTL2, AK8963_RESET);
	// select clock source to gyro
	writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL);

	// check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
	uint8_t who = whoAmI();
	if((who != 0x71) &&( who != 0x73))
	{
		//return 1;
	}

	// enable accelerometer and gyro
	writeRegister(PWR_MGMNT_2, SEN_ENABLE);

	// setting accel range to 8G as default
	writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_16G);

	// setting the gyro range to 500DPS as default
	writeRegister(GYRO_CONFIG, GYRO_FS_SEL_2000DPS);

	// setting bandwidth to 184Hz as default
	writeRegister(ACCEL_CONFIG2, DLPF_10);

	// setting gyro bandwidth to 184Hz
	writeRegister(CONFIG, DLPF_10);

	// setting the sample rate divider to 0 as default
	writeRegister(SMPDIV, 0x00);

	// enable I2C master mode
	writeRegister(USER_CTRL, I2C_MST_EN);

	// set the I2C bus speed to 400 kHz
	writeRegister(I2C_MST_CTRL, I2C_MST_CLK);

	// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	if( whoAmIAK8963() != 0x48 )
	{
		return 1;
	}

	/* get the magnetometer calibration */
	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);

	HAL_Delay(100); // long wait between AK8963 mode changes

	// set AK8963 to FUSE ROM access
	writeAK8963Register(AK8963_CNTL1, AK8963_FUSE_ROM);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// read the AK8963 ASA registers and compute magnetometer scale factors
	readAK8963Registers(AK8963_ASA, 3, _mag_adjust);

	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// set AK8963 to 16 bit resolution, 100 Hz update rate
	writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS2);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// select clock source to gyro
	writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL);

	// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
	readAK8963Registers(AK8963_HXL, 7 , _buffer);
//	power up gyro
//	writeRegister(PWR_MGMNT_1, 0x00);
//	//delay(100);
//	writeRegister(PWR_MGMNT_1, 0x01);
//
//	writeRegister(CONFIG, 0x03);
//
//	writeRegister(SMPDIV, 0x04);
//	//MPU6050_SMPLRT_DIV
//	//gyro config 500
//
//	writeRegister(GYRO_CONFIG, 0x08);
//	//accel config 8g
//	writeRegister(ACCEL_CONFIG, 0x10);

	// successful init, return 0
	return 0;
}

/* sets the accelerometer full scale range to values other than default */
void MPU9250_SetAccelRange(AccelRange range)
{
	writeRegister(ACCEL_CONFIG, range);
}

/* sets the gyro full scale range to values other than default */
void MPU9250_SetGyroRange(GyroRange range)
{
	writeRegister(GYRO_CONFIG, range);
}

/* sets the DLPF bandwidth to values other than default */
void MPU9250_SetDLPFBandwidth(DLPFBandwidth bandwidth)
{
	writeRegister(ACCEL_CONFIG2, bandwidth);
	writeRegister(CONFIG, bandwidth);
}

/* sets the sample rate divider to values other than default */
void MPU9250_SetSampleRateDivider(SampleRateDivider srd)
{
	/* setting the sample rate divider to 19 to facilitate setting up magnetometer */
	writeRegister(SMPDIV,19);

	if(srd > 9)
	{
		// set AK8963 to Power Down
		writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// set AK8963 to 16 bit resolution, 8 Hz update rate
		writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS1);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		readAK8963Registers(AK8963_HXL, 7, _buffer);

	}
	else
	{
		// set AK8963 to Power Down
		writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
		// long wait between AK8963 mode changes
		HAL_Delay(100);
		// set AK8963 to 16 bit resolution, 100 Hz update rate
		writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		readAK8963Registers(AK8963_HXL, 7 , _buffer);
	}

	writeRegister(SMPDIV, srd);
}

uint16_t ii, packet_count, fifo_count;

void MPU9250_calibrate()
{
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    //uint16_t ii, packet_count, fifo_count;
    //int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device
    writeRegister(PWR_MGMNT_1, PWR_RESET); // Write a one to bit 7 reset bit; toggle reset device
    HAL_Delay(100);

    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
    // else use the internal oscillator, bits 2:0 = 001
    writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL);
    writeRegister(PWR_MGMNT_2, SEN_ENABLE);
    HAL_Delay(200);

    // Configure device for bias calculation
    writeRegister(INT_ENABLE, INT_DISABLE);   // Disable all interrupts
    writeRegister(FIFO_EN, 0x00);      // Disable FIFO
    writeRegister(PWR_MGMNT_1, SEN_ENABLE);   // Turn on internal clock source
    writeRegister(I2C_MST_CTRL, 0x00); // Disable I2C master
    writeRegister(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    writeRegister(USER_CTRL, 0x0C);    // Reset FIFO and DMP
    HAL_Delay(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    writeRegister(CONFIG, DLPF_184);      // Set low-pass filter to 184 Hz
    writeRegister(SMPDIV, 0x00);  // Set sample rate to 1 kHz
    writeRegister(GYRO_CONFIG, GYRO_FS_SEL_250DPS);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_2G); // Set accelerometer full-scale to 2 g, maximum sensitivity

    //uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    writeRegister(USER_CTRL, 0x40);   // Enable FIFO
    writeRegister(FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9250)
    HAL_Delay(28); // accumulate 40 samples in 27 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    writeRegister(FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    //readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
    uint8_t fifoCount_buffer[2];
    readRegisters(FIFO_COUNT, 2, fifoCount_buffer);

    fifo_count = (fifoCount_buffer[0] << 8) | fifoCount_buffer[1];
    packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++)
    {
        int32_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        readRegisters(FIFO_READ, 12, data);
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];
    }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[2] += (int32_t) accelsensitivity;}

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1] / 4)       & 0xFF;
    data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2] / 4)       & 0xFF;

    // Push gyro biases to hardware registers
    writeRegister(XG_OFFSET_H, data[0]);
    writeRegister(XG_OFFSET_L, data[1]);
    writeRegister(YG_OFFSET_H, data[2]);
    writeRegister(YG_OFFSET_L, data[3]);
    writeRegister(ZG_OFFSET_H, data[4]);
    writeRegister(ZG_OFFSET_L, data[5]);

//    data[0] = (-accel_bias[0] / 8  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
//    data[1] = (-accel_bias[0] / 8)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
//    data[2] = (-accel_bias[1] / 8  >> 8) & 0xFF;
//    data[3] = (-accel_bias[1] / 8)       & 0xFF;
//    data[4] = (-accel_bias[2] / 8  >> 8) & 0xFF;
//    data[5] = (-accel_bias[2] / 8)       & 0xFF;
//
//	MPU9250_writeReg(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
//	MPU9250_writeReg(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
//	MPU9250_writeReg(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
//	MPU9250_writeReg(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
//	MPU9250_writeReg(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
//	MPU9250_writeReg(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

    // Output scaled gyro biases for display in the main program
//    dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
//    dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
//    dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.


     //int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
     //readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
//     data[0] = MPU9250_readReg(MPU9250_ADDRESS, XA_OFFSET_H);
//     data[1] = MPU9250_readReg(MPU9250_ADDRESS, XA_OFFSET_L);
//     data[2] = MPU9250_readReg(MPU9250_ADDRESS, YA_OFFSET_H);
//     data[3] = MPU9250_readReg(MPU9250_ADDRESS, YA_OFFSET_L);
//     data[4] = MPU9250_readReg(MPU9250_ADDRESS, ZA_OFFSET_H);
//     data[5] = MPU9250_readReg(MPU9250_ADDRESS, ZA_OFFSET_L);
//     acc_ofset_data[0] = MPU9250_readReg(MPU9250_ADDRESS, XA_OFFSET_H);
//     acc_ofset_data[1] = MPU9250_readReg(MPU9250_ADDRESS, XA_OFFSET_L);
//     acc_ofset_data[2] = MPU9250_readReg(MPU9250_ADDRESS, YA_OFFSET_H);
//     acc_ofset_data[3] = MPU9250_readReg(MPU9250_ADDRESS, YA_OFFSET_L);
//     acc_ofset_data[4] = MPU9250_readReg(MPU9250_ADDRESS, ZA_OFFSET_H);
//     acc_ofset_data[5] = MPU9250_readReg(MPU9250_ADDRESS, ZA_OFFSET_L);
//     accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]  );
//     accel_bias_reg[1] = (int32_t) (((int16_t)data[2] << 8) | data[3]  );
//     accel_bias_reg[2] = (int32_t) (((int16_t)data[4] << 8) | data[5]  );


     //((uint16_t)_buffer[0] << 8) + _buffer[1]))
     //readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
     //accel_bias_reg[1] = (uint16_t)((uint16_t)data[2]) << 8) + data[3]);
     //readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
     //accel_bias_reg[2] = (uint16_t)((uint16_t)data[4]) << 8) + data[5]);
//     int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
//     readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
//     accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
//     readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
//     accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
//     readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
//     accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

     //uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
//     uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    // for(ii = 0; ii < 3; ii++) {
    //     if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    // }

     // Construct total accelerometer bias, including calculated average accelerometer bias from above
//     accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
//     accel_bias_reg[1] -= (accel_bias[1] / 8);
//     accel_bias_reg[2] -= (accel_bias[2] / 8);
//
//     data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
//     data[1] = (accel_bias_reg[0])      & 0xFF;
//     //data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
//     data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
//     data[3] = (accel_bias_reg[1])      & 0xFF;
//     //data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
//     data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
//     data[5] = (accel_bias_reg[2])      & 0xFF;
//     //data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

//     Apparently this is not working for the acceleration biases in the MPU-9250
//     Are we handling the temperature correction bit properly?
//     Push accelerometer biases to hardware registers
//     MPU9250_writeReg(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
//     MPU9250_writeReg(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
//     MPU9250_writeReg(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
//     MPU9250_writeReg(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
//     MPU9250_writeReg(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
//     MPU9250_writeReg(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

//	MPU9250_writeReg(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
//	MPU9250_writeReg(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
//	MPU9250_writeReg(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
//	MPU9250_writeReg(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
//	MPU9250_writeReg(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
//	MPU9250_writeReg(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

//	acc_ofset_data_corrected[0] = MPU9250_readReg(MPU9250_ADDRESS, XA_OFFSET_H);
//	acc_ofset_data_corrected[1] = MPU9250_readReg(MPU9250_ADDRESS, XA_OFFSET_L);
//	acc_ofset_data_corrected[2] = MPU9250_readReg(MPU9250_ADDRESS, YA_OFFSET_H);
//	acc_ofset_data_corrected[3] = MPU9250_readReg(MPU9250_ADDRESS, YA_OFFSET_L);
//	acc_ofset_data_corrected[4] = MPU9250_readReg(MPU9250_ADDRESS, ZA_OFFSET_H);
//	acc_ofset_data_corrected[5] = MPU9250_readReg(MPU9250_ADDRESS, ZA_OFFSET_L);

}

/* read the data, each argiment should point to a array for x, y, and x */
void MPU9250_GetData(int16_t* AccData, int16_t* MagData, int16_t* GyroData)
{
	// grab the data from the MPU9250
	readRegisters(ACCEL_OUT, 21, _buffer);

	// combine into 16 bit values
	AccData[0] = (((int16_t)_buffer[0]) << 8) | _buffer[1];
	AccData[1] = (((int16_t)_buffer[2]) << 8) | _buffer[3];
	AccData[2] = (((int16_t)_buffer[4]) << 8) | _buffer[5];
	GyroData[0] = (((int16_t)_buffer[8]) << 8) |_buffer[9];
	GyroData[1] = (((int16_t)_buffer[10]) << 8) | _buffer[11];
	GyroData[2] = (((int16_t)_buffer[12]) << 8) | _buffer[13];

	accelX = (((int16_t)((uint16_t)_buffer[0] << 8) + _buffer[1])) / 2048.0f * 9.8f;
	accelY = (((int16_t)((uint16_t)_buffer[2] << 8) + _buffer[3])) / 2048.0f * 9.8f;
	accelZ = (((int16_t)((uint16_t)_buffer[4] << 8) + _buffer[5])) / 2048.0f * 9.8f;
//	accelX=((((int16_t)((uint16_t)_buffer[6] << 8) + _buffer[7])))/4096.0f*9.8f;
//	accelY=((((int16_t)((uint16_t)_buffer[8] << 8) + _buffer[9])))/4096.0f*9.8f;
	gyroX = (((int16_t)((uint16_t)_buffer[8] << 8) + _buffer[9])) / 16.4f * 3.14f / 180.0f;
	gyroY = (((int16_t)((uint16_t)_buffer[10] << 8) + _buffer[11])) / 16.4f * 3.14f / 180.0f;
	gyroZ = (((int16_t)((uint16_t)_buffer[12] << 8) + _buffer[13])) / 16.4f * 3.14f / 180.0f;
	accelX = accelX - (accel_bias[0] / 16384.0f * 9.8f);
	accelY = accelY - (accel_bias[1] / 16384.0f * 9.8f);
	accelZ = accelZ - (accel_bias[2] / 16384.0f * 9.8f);
	gyroX_filtered = expRunningAverageGX(gyroX) - 0.0028;
	gyroY_filtered = expRunningAverageGY(gyroY) - 0.020;
	gyroZ_filtered = expRunningAverageGZ(gyroZ) - 0.004;
	accelX_filtered = expRunningAverageAX(accelX);
	accelY_filtered = expRunningAverageAY(accelY);
	accelZ_filtered = expRunningAverageAZ(accelZ);
}
