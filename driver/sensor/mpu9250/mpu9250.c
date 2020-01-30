#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef unsigned char uint8_t;
typedef signed char int8_t;
typedef unsigned short uint16_t;
typedef signed short int16_t;
typedef unsigned int uint32_t;
typedef signed int int32_t;

/*

Gyroscope Full Scale Range	Accelerometer Full Scale Range	Magnetometer Full Scale Range
+/- 250 (deg/s)				+/- 2 (g)						+/- 4800 (uT)
+/- 500 (deg/s)				+/- 4 (g)	
+/- 1000 (deg/s)			+/- 8 (g)	
+/- 2000 (deg/s)			+/- 16 (g)	

*/

float gRes = 500.0/16384.0;
float aRes = 4.0/16384.0;
float mRes = 10.0*4912.0/16384.0;

int gyro;
int mag;

int16_t accelCount[3];
int16_t gyroCount[3];
int16_t magCount[3];
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};
float ax, ay, az, gx, gy, gz, mx, my, mz;
int16_t tempCount;
float temperature;
float SelfTest[6];

const float PI = 3.14159265358979323846f;
const float GyroMeasError = PI * (60.0f / 180.0f);
const float beta = sqrt(3.0f / 4.0f) * GyroMeasError;
const float GyroMeasDrift = PI * (1.0f / 180.0f);
const float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;
#define Kp 2.0f * 5.0f
#define Ki 0.0f

float pitch, yaw, roll;
float deltat = 0.0f;
int lastUpdate = 0, firstUpdate = 0, Now = 0;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float eInt[3] = {0.0f, 0.0f, 0.0f};

void MPU9250_MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2.0f * q1 * mx;
            _2q1my = 2.0f * q1 * my;
            _2q1mz = 2.0f * q1 * mz;
            _2q2mx = 2.0f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = sqrt(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            norm = 1.0f/norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * deltat;
            q2 += qDot2 * deltat;
            q3 += qDot3 * deltat;
            q4 += qDot4 * deltat;
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;
} 
void MPU9250_MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, bx, bz;
            float vx, vy, vz, wx, wy, wz;
            float ex, ey, ez;
            float pa, pb, pc;

            // Auxiliary variables to avoid repeated arithmetic
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;   

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
            hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
            bx = sqrt((hx * hx) + (hy * hy));
            bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

            // Estimated direction of gravity and magnetic field
            vx = 2.0f * (q2q4 - q1q3);
            vy = 2.0f * (q1q2 + q3q4);
            vz = q1q1 - q2q2 - q3q3 + q4q4;
            wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
            wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
            wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

            // Error is cross product between estimated direction and measured direction of gravity
            ex = (ay * vz - az * vy) + (my * wz - mz * wy);
            ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
            ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
            if (Ki > 0.0f)
            {
                eInt[0] += ex;      // accumulate integral error
                eInt[1] += ey;
                eInt[2] += ez;
            }
            else
            {
                eInt[0] = 0.0f;     // prevent integral wind up
                eInt[1] = 0.0f;
                eInt[2] = 0.0f;
            }

            // Apply feedback terms
            gx = gx + Kp * ex + Ki * eInt[0];
            gy = gy + Kp * ey + Ki * eInt[1];
            gz = gz + Kp * ez + Ki * eInt[2];

            // Integrate rate of change of quaternion
            pa = q2;
            pb = q3;
            pc = q4;
            q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
            q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
            q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
            q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

            // Normalise quaternion
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
            norm = 1.0f / norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;
}
static void writeRaw8(int fd, uint8_t data) {
	data&=0xFF;
	wiringPiI2CWrite(fd,data);
}
static void writeReg8(int fd, uint8_t reg, uint8_t data) {
	data&=0xFF;
	wiringPiI2CWriteReg8(fd,reg,data);
}
static uint8_t readRaw8(int fd) {
	return wiringPiI2CRead(fd);
}
static uint8_t readReg8(int fd, uint8_t reg) {
	return wiringPiI2CReadReg8(fd,reg);
}
static void alternateReadReg8(int fd, uint8_t reg, uint8_t* data, uint8_t count) {
	for(int i=0;i<count;i++) {
		data[i]=readReg8(fd,reg);
		reg++;
	}
}
static void readAccelData(int16_t* buffer) {
	uint8_t data[6];
	alternateReadReg8(gyro, 0x3B, &data[0], 6);
	buffer[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);
	buffer[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);  
	buffer[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]); 
}
static void readGyroData(int16_t* buffer) {
	uint8_t data[6];
	alternateReadReg8(gyro, 0x43, &data[0], 6);
	buffer[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);
	buffer[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);  
	buffer[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]); 
}
static void readMagnetData(int16_t* buffer) {
	uint8_t data[7];
	if(readReg8(mag, 0x02) & 0x01) {
		alternateReadReg8(mag, 0x03, &data[0], 7);
		uint8_t c = data[6];
		if(!(c & 0x08)) {
			buffer[0] = (int16_t)(((int16_t)data[1] << 8) | data[0]);
			buffer[1] = (int16_t)(((int16_t)data[3] << 8) | data[2]);
			buffer[2] = (int16_t)(((int16_t)data[5] << 8) | data[4]); 
		}
	}
}
static int16_t readTempData() {
	uint8_t data[2];
	alternateReadReg8(gyro, 0x41, &data[0], 2);
	return (int16_t)(((int16_t)data[0]) << 8 | data[1]);
}

void reset() {
	writeReg8(gyro, 0x6B, 0x80);
	delay(100);
}
static void setupGyroAccel() {  
	writeReg8(gyro, 0x6B, 0x00);
	delay(100);
	writeReg8(gyro, 0x6B, 0x01);
	writeReg8(gyro, 0x1A, 0x03);
	writeReg8(gyro, 0x19, 0x04);
	writeReg8(gyro, 0x1B, 0x10);//Gscale here
	writeReg8(gyro, 0x1C, 0x10);//Ascale here
	writeReg8(gyro, 0x1D, 0x80);
	writeReg8(gyro, 0x37, 0x22);    
	writeReg8(gyro, 0x38, 0x01);
}
static void calibrateGyroAccel(float* bufferA, float* bufferB) {
	uint8_t data[12];
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
	writeReg8(gyro, 0x6B, 0x80);
	delay(100);  
	writeReg8(gyro, 0x6B, 0x01);  
	writeReg8(gyro, 0x6C, 0x00); 
	delay(200);
	
	writeReg8(gyro, 0x38, 0x00);
	writeReg8(gyro, 0x23, 0x00);
	writeReg8(gyro, 0x6B, 0x00);
	writeReg8(gyro, 0x24, 0x00);
	writeReg8(gyro, 0x6A, 0x00);
	writeReg8(gyro, 0x6A, 0x0C);
	delay(15);
	
	writeReg8(gyro, 0x1A, 0x01);
	writeReg8(gyro, 0x19, 0x00);
	writeReg8(gyro, 0x1B, 0x00);
	writeReg8(gyro, 0x1C, 0x00);
 
	uint16_t  gyrosensitivity  = 131;
	uint16_t  accelsensitivity = 16384;
	
	writeReg8(gyro, 0x6A, 0x40);
	writeReg8(gyro, 0x23, 0x78);
	delay(80);

	writeReg8(gyro, 0x23, 0x00);
	alternateReadReg8(gyro, 0x72, &data[0], 2);
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count/12;
	
	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		alternateReadReg8(gyro, 0x74, &data[0], 12);
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]);
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]);
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]);    
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]);
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]);
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);
    
   		accel_bias[0] += (int32_t) accel_temp[0];
    	accel_bias[1] += (int32_t) accel_temp[1];
    	accel_bias[2] += (int32_t) accel_temp[2];
    	gyro_bias[0]  += (int32_t) gyro_temp[0];
    	gyro_bias[1]  += (int32_t) gyro_temp[1];
    	gyro_bias[2]  += (int32_t) gyro_temp[2];         
	}
    accel_bias[0] /= (int32_t) packet_count;
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
	if(accel_bias[2] > 0L) {
		accel_bias[2] -= (int32_t) accelsensitivity;
	} else {
		accel_bias[2] += (int32_t) accelsensitivity;
	}
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF;
	data[1] = (-gyro_bias[0]/4)       & 0xFF;
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;

	writeReg8(gyro, 0x13, data[0]);
	writeReg8(gyro, 0x14, data[1]);
	writeReg8(gyro, 0x15, data[2]);
	writeReg8(gyro, 0x16, data[3]);
	writeReg8(gyro, 0x17, data[4]);
	writeReg8(gyro, 0x18, data[5]);
	
	bufferA[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
	bufferA[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	bufferA[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

	int32_t accel_bias_reg[3] = {0, 0, 0};
	alternateReadReg8(gyro, 0x77, &data[0], 2);
	accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	alternateReadReg8(gyro, 0x7A, &data[0], 2);
	accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	alternateReadReg8(gyro, 0x7D, &data[0], 2);
	accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  
	uint32_t mask = 1uL;
	uint8_t mask_bit[3] = {0, 0, 0};
  
	for(ii = 0; ii < 3; ii++) {
		if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01;
	}
	accel_bias_reg[0] -= (accel_bias[0]/8);
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);
 
	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0];
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1];
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2];
	
	writeReg8(gyro, 0x77, data[0]);
	writeReg8(gyro, 0x78, data[1]);
	writeReg8(gyro, 0x7A, data[2]);
	writeReg8(gyro, 0x7B, data[3]);
	writeReg8(gyro, 0x7D, data[4]);
	writeReg8(gyro, 0x7E, data[5]);

	
	bufferB[0] = (float)accel_bias[0]/(float)accelsensitivity; 
	bufferB[1] = (float)accel_bias[1]/(float)accelsensitivity;
	bufferB[2] = (float)accel_bias[2]/(float)accelsensitivity;
}
static void testGyroAccel(float* destination) {
	uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
	uint8_t selfTest[6];
	int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
	float factoryTrim[6];
	uint8_t FS = 0;
   
	writeReg8(gyro, 0x19, 0x00);
	writeReg8(gyro, 0x1A, 0x02);
	writeReg8(gyro, 0x1B, FS<<3);
	writeReg8(gyro, 0x1D, 0x02);
	writeReg8(gyro, 0x1C, FS<<3);

	for( int ii = 0; ii < 200; ii++) {
		alternateReadReg8(gyro, 0x3B, &rawData[0], 6);
		aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
		aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
  
		alternateReadReg8(gyro, 0x43, &rawData[0], 6);
		gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
		gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
	}
	for (int ii =0; ii < 3; ii++) {
  		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}
	writeReg8(gyro, 0x1C, 0xE0);
	writeReg8(gyro, 0x1B, 0xE0);
	delay(25);
	for( int ii = 0; ii < 200; ii++) {
		alternateReadReg8(gyro, 0x3B, &rawData[0], 6);
		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
  
		alternateReadReg8(gyro, 0x43, &rawData[0], 6);
		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
	}
	for (int ii =0; ii < 3; ii++) {
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}
	writeReg8(gyro, 0x1C, 0x00);
 	writeReg8(gyro, 0x1B, 0x00);
	delay(25);
	selfTest[0] = readReg8(gyro, 0x0D);
	selfTest[1] = readReg8(gyro, 0x0E);
	selfTest[2] = readReg8(gyro, 0x0F);
	selfTest[3] = readReg8(gyro, 0x00);
	selfTest[4] = readReg8(gyro, 0x01);
	selfTest[5] = readReg8(gyro, 0x02);
	
	factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) ));
	factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) ));
	factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) ));
	factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) ));
	factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) ));
	factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) ));

	for (int i = 0; i < 3; i++) {
		destination[i] = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.0;
		destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.0;
	}
}
static void setupMagnet(float* buffer) {
	uint8_t data[3];
	writeReg8(mag, 0x0A, 0x00); 
	delay(10);
	writeReg8(mag, 0x0A, 0x0F);
	delay(10);
	alternateReadReg8(mag, 0x10, &data[0], 3);
	buffer[0] =  (float)(data[0] - 128)/256.0f + 1.0f;
	buffer[1] =  (float)(data[1] - 128)/256.0f + 1.0f;  
	buffer[2] =  (float)(data[2] - 128)/256.0f + 1.0f; 
	writeReg8(mag, 0x0A, 0x00);
	delay(10);
	writeReg8(mag, 0x0A, 0x01 << 4 | 0x06);//Mscale here
	delay(10);
}

void MPU9250_setupSensors() {
	reset();
	delay(200);
    calibrateGyroAccel(gyroBias, accelBias);
   	delay(150);
	setupGyroAccel();
	setupMagnet(magCalibration);
	delay(150);
}
void MPU9250_setupComs(char gyroAddress,char magAddress) {
	wiringPiSetupGpio();
	gyro=wiringPiI2CSetup(gyroAddress);
	mag=wiringPiI2CSetup(magAddress);
}

typedef struct {
	float axx,ayy,azz,gxx,gyy,gzz,mxx,myy,mzz;
	int16_t tempDataA;
} mpu9250_data;
mpu9250_data MPU9250_readAllData() {

	if(readReg8(gyro, 0x3A) & 0x01) {
		readAccelData(accelCount);  
		ax = (float)accelCount[0]*aRes - accelBias[0];
		ay = (float)accelCount[1]*aRes - accelBias[1];   
		az = (float)accelCount[2]*aRes - accelBias[2];  
		
		readGyroData(gyroCount);
		gx = (float)gyroCount[0]*gRes - gyroBias[0];
		gy = (float)gyroCount[1]*gRes - gyroBias[1];  
		gz = (float)gyroCount[2]*gRes - gyroBias[2];   

		readMagnetData(magCount);
		mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];
		my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];  
		mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];

	}
	MPU9250_MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
		
	// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
	// In this coordinate system, the positive z-axis is down toward Earth. 
	// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
	// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
	// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
	// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
	// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
	// applied in the correct order which for this configuration is yaw, pitch, and then roll.
	// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
	yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
	pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
	roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
	pitch *= 180.0f / PI;
	yaw   *= 180.0f / PI; 
	yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
	roll  *= 180.0f / PI;

	int16_t tempData = readTempData();

	return (mpu9250_data){.axx=ax,.ayy=ay,.azz=az,.gxx=gx,.gyy=gy,.gzz=gz,.mxx=mx,.myy=my,.mzz=mz,.tempDataA=tempData};
}
