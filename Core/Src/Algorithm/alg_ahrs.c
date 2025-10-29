/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : alg_ahrs.c
 *  Description  : This file contains AHRS algorithm functions
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:06
 *  LastEditTime : 2023-08-14 16:31:56
 */


#include "alg_ahrs.h" 
#include "alg_math.h"		  

float carrier_gravity;


/**
  * @brief      According to the data of the acceleration, the data of the magnetometer is initialized by the quaternion
  * @param      q[4] :The Quad Array needs to be updated
  * @param      ax :Accelerometer x-axis data
  * @param      ay :Accelerometer y-axis data
  * @param      az :Accelerometer z-axis data
  * @param      mx :Geomagnetic pole x-axis data
  * @param      my :Geomagnetic pole y-axis data
  * @param      mz :Geomagnetic pole z-axis data
  * @retval         NULL
  */
void AHRS_Init(float quat[4], float ax, float ay, float az, float mx, float my, float mz) {
    float local_height, latitude, local_gravity;

    if (quat) {
        AHRS_GetHeight(&local_height);
        AHRS_GetLatitude(&latitude);
        
        float sinlat = sinf(latitude * 0.017453f);
        float sin2lat = sinf((latitude * 2.0f) * 0.017453f);
        local_gravity = (((0.0053024f * sinlat * sinlat) + 1.0f) - (0.0000059f * sin2lat * sin2lat)) * 9.7803f;
        
        float h_ang = (local_height / 6370900.0f) + 1.0f;
        carrier_gravity = local_gravity / (h_ang * h_ang);
        
        float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
        if (ax != 0.0f || ay != 0.0f || az != 0.0f) {
            float init_accel[3];
            float sqsum = ax * ax + ay * ay + az * az;
            float invsqrt = Math_InvSqrt(sqsum);
            
            init_accel[0] = invsqrt * ax;
            init_accel[1] = invsqrt * ay;
            init_accel[2] = invsqrt * az;

            pitch = asinf(-init_accel[0]);
            roll = atan2f(init_accel[1], init_accel[2]);
        }
        
        if (mx != 0.0f || my != 0.0f || mz != 0.0f) {
            float sp = sinf(pitch);
            float cp = cosf(pitch);
            float sr = sinf(roll);
            float cr = cosf(roll);
            float vs24 = cp * mx + sp * sr * my + sp * cr * mz;
            float vs25 = cr * my - sr * mz;
            yaw = atan2f(vs24, vs25);
        }
        
        AHRS_AngleToQuat(quat, yaw, pitch, roll);
    }
}


/* * 
  * @brief      AHRS data update function
  * @param      q[4] :The Quad Array needs to be updated
  * @param      gx :x-axis data of gyroscope
  * @param      gy :y-axis data of gyroscope
  * @param      gz :z-axis data of gyroscope
  * @param      ax :Accelerometer x-axis data
  * @param      ay :Accelerometer y-axis data
  * @param      az :Accelerometer z-axis data
  * @param      mx :Geomagnetic pole x-axis data
  * @param      my :Geomagnetic pole y-axis data
  * @param      mz :Geomagnetic pole z-axis data
  * @retval     1 succese ; 0 falue
  */
uint8_t AHRS_Update(float quat[4], const float timing_time, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {

    if (!quat && timing_time == 0.0f && gx == 0 && gy == 0 && gz == 0 && ax == 0 && ay == 0 && az == 0) {
        return 0;
    }
    if (!mx || !my || !mz) {
        return AHRS_UpdateIMU(quat, timing_time, gx, gy, gz, ax, ay, az);
    }
    else {
        float w_t[4][4]; 
        float accel_local[3]; 
        float gyro_local[3]; 
        float mag_local[3];
        float quat_add[4]; 

        float Accel_Kp[3] = {0.2f, 0.2f, 0.2f}; 
        float accel_error[3]; 
        float mag_error[3];
        float AccelVector[3];
        float MagVector[3];
        float hx, hy, bx, bz;

        gyro_local[0] = gx;
        gyro_local[1] = gy;
        gyro_local[2] = gz;
        accel_local[0] = ax;
        accel_local[1] = ay;
        accel_local[2] = az;
        mag_local[0] = mx;
        mag_local[1] = my;
        mag_local[2] = mz;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        AccelVector[0] = (quat[1] * quat[3] - quat[0] * quat[2]) * 2.0f;
        AccelVector[1] = (quat[0] * quat[1] + quat[2] * quat[3]) * 2.0f;
        AccelVector[2] = (quat[0] * quat[0] + quat[3] * quat[3]) * 2.0f - 1.0f;

        // Normalise accelerometer measurement
        float acc_sqrsum = 0;
        for (int32_t i = 0; i < 3; ++i) 
            acc_sqrsum += accel_local[i] * accel_local[i];
        float acc_sqrt = Math_InvSqrt(acc_sqrsum);       

        // Normalise magnetometer measurement
        float mag_sqrsum = 0;
        for (int32_t i = 0; i < 3; ++i)
            mag_sqrsum += mag_local[i] * mag_local[i];
        float mag_sqrt = Math_InvSqrt(mag_sqrsum);
        for (int32_t i = 0; i < 3; ++i)
            mag_local[i] *= mag_sqrt;

        // Angular Velocity Compensation
        float gyro_abssum = 0;         
        for (int32_t i = 0; i < 3; ++i)                        
            gyro_abssum += fabs(accel_local[i]);
        float diff_grav = fabs(acc_sqrt - carrier_gravity); 

        if (diff_grav > 0.1f || gyro_abssum > 0.12f) {
            float vs0 = 0.0f, vs1 = 1.0f, vs2 = 0.0f, vs3 = 0.0f;
            if (diff_grav > 0.0f) 
                vs0 = 0.1f / diff_grav;
            if (vs0 < 0.3f)
                vs0 = 0.0f;
            if (gyro_abssum > 0.0f)
                vs3 = 0.12f / gyro_abssum;
            if (vs0 < 1.0f)
                vs1 = vs0 * vs1;
            if (vs3 < 1.0f)
                vs1 = vs1 * vs3;
            for (int32_t i = 0; i < 3; ++i) 
                Accel_Kp[i] = vs1 * 0.2f;
            UNUSED(vs2);
        }

        for (int32_t i = 0; i < 3; ++i)
            accel_local[i] *= acc_sqrt;   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mag_local[0] * (0.5f - quat[2] * quat[2] - quat[3] * quat[3]) 
            + mag_local[1] * (quat[1] * quat[2] - quat[0] * quat[3]) 
            + mag_local[2] * (quat[1] * quat[3] + quat[0] * quat[2]));
        hy = 2.0f * (mag_local[0] * (quat[1] * quat[2] + quat[0] * quat[3]) 
            + mag_local[1] * (0.5f - quat[1] * quat[1] - quat[3] * quat[3]) 
            + mag_local[2] * (quat[2] * quat[3] - quat[0] * quat[1]));

        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mag_local[0] * (quat[1] * quat[3] - quat[0] * quat[2]) 
            + mag_local[1] * (quat[2] * quat[3] + quat[0] * quat[1]) 
            + mag_local[2] * (0.5f - quat[1] * quat[1] - quat[2] * quat[2]));

        MagVector[0] = 2.0f * (bx * (0.5f - quat[2] * quat[2] - quat[3] * quat[3]) 
                                + bz * (quat[1] * quat[3] - quat[0] * quat[2]));
        MagVector[1] = 2.0f * (bx * (quat[1] * quat[2] - quat[0] * quat[3]) 
                                + bz * (quat[0] * quat[1] + quat[2] * quat[3]));
        MagVector[2] = 2.0f * (bx * (quat[0] * quat[2] + quat[1] * quat[3]) 
                                + bz * (0.5f - quat[1] * quat[1] - quat[2] * quat[2]));  
        
        // Error is sum of cross product between estimated and measured direction of gravity
        accel_error[0] = (accel_local[1] * AccelVector[2]) - (accel_local[2] * AccelVector[1]);
        accel_error[1] = (accel_local[2] * AccelVector[0]) - (accel_local[0] * AccelVector[2]);
        accel_error[2] = (accel_local[0] * AccelVector[1]) - (accel_local[1] * AccelVector[0]);
                        
        mag_error[0] = (mag_local[1] * MagVector[2] - mag_local[2] * MagVector[1]);
        mag_error[1] = (mag_local[2] * MagVector[0] - mag_local[0] * MagVector[2]);
        mag_error[2] = (mag_local[0] * MagVector[1] - mag_local[1] * MagVector[0]);

        gyro_local[0] += Accel_Kp[0] * accel_error[0] + mag_error[0];
        gyro_local[1] += Accel_Kp[1] * accel_error[1] + mag_error[1];
        gyro_local[2] += Accel_Kp[2] * accel_error[2] + mag_error[2];

        memset(w_t, 0, sizeof(w_t));
        w_t[0][1] = -gyro_local[0]; w_t[0][2] = -gyro_local[1]; w_t[0][3] = -gyro_local[2];
        w_t[1][0] =  gyro_local[0]; w_t[1][2] =  gyro_local[2]; w_t[1][3] = -gyro_local[1];
        w_t[2][0] =  gyro_local[1]; w_t[2][1] = -gyro_local[2]; w_t[2][3] =  gyro_local[0];
        w_t[3][0] =  gyro_local[2]; w_t[3][1] =  gyro_local[1]; w_t[3][2] = -gyro_local[0]; 

        for (int32_t i = 0; i < 4; ++i) {
            float tmp = 0.0f;
            for (int32_t j = 0; j < 4; ++j)
                tmp += w_t[i][j] * quat[j];
            quat_add[i] = tmp * timing_time / 2.0f;
        }   

        for (int32_t i = 0; i < 4; ++i)
            quat[i] = quat[i] + quat_add[i];

        float sqrsum = 0.0f;
        for (int32_t i = 0; i < 4; ++i)
            sqrsum += quat[i] * quat[i];
        float invsqrt = Math_InvSqrt(sqrsum);
        for (int32_t i = 0; i < 4; ++i)
            quat[i] *= invsqrt;

        return 1;
    }
}


/**
  * @brief      AHRS data update function (There is no geomagnetic pole)
  * @param      q[4] :The Quad Array needs to be updated
  * @param      gx :x-axis data of gyroscope
  * @param      gy :y-axis data of gyroscope
  * @param      gz :z-axis data of gyroscope
  * @param      ax :Accelerometer x-axis data
  * @param      ay :Accelerometer y-axis data
  * @param      za :Accelerometer z-axis data
  * @retval     NULL
  */
uint8_t AHRS_UpdateIMU(float quat[4], const float timing_time, float gx, float gy, float gz, float ax, float ay, float az) {
    float w_t[4][4]; 
    float accel_local[3]; 
    float gyro_local[3]; 
    float quat_add[4]; 

    float Accel_Kp[3] = {0.2f, 0.2f, 0.2f}; 
    float accel_error[3] = {0.0f, 0.0f, 0.0f}; 
    float AccelVector[3];

    gyro_local[0] = gx;
    gyro_local[1] = gy;
    gyro_local[2] = gz;
    accel_local[0] = ax;
    accel_local[1] = ay;
    accel_local[2] = az;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    AccelVector[0] = (quat[1] * quat[3] - quat[0] * quat[2]) * 2.0f;
    AccelVector[1] = (quat[0] * quat[1] + quat[2] * quat[3]) * 2.0f;
    AccelVector[2] = (quat[0] * quat[0] + quat[3] * quat[3]) * 2.0f - 1.0f;

    // Normalise accelerometer measurement
    float acc_sqrsum = 0;
    for (int32_t i = 0; i < 3; ++i) 
        acc_sqrsum += accel_local[i] * accel_local[i];
    float acc_sqrt = Math_InvSqrt(acc_sqrsum);       
    
    float gyro_abssum = 0;         
    for (int32_t i = 0; i < 3; ++i)                        
        gyro_abssum += fabs(accel_local[i]);
    float diff_grav = fabs(acc_sqrt - carrier_gravity); 

    if (diff_grav > 0.1f || gyro_abssum > 0.12f) {
        float vs0 = 0.0f, vs1 = 1.0f, vs2 = 0.0f, vs3 = 0.0f;
        if (diff_grav > 0.0f) 
            vs0 = 0.1f / diff_grav;
        if (vs0 < 0.3f)
            vs0 = 0.0f;
        if (gyro_abssum > 0.0f)
            vs3 = 0.12f / gyro_abssum;
        if (vs0 < 1.0f)
            vs1 = vs0 * vs1;
        if (vs3 < 1.0f)
            vs1 = vs1 * vs3;
        for (int32_t i = 0; i < 3; ++i) 
            Accel_Kp[i] = vs1 * 0.2f;
        UNUSED(vs2);
    }

    for (int32_t i = 0; i < 3; ++i)
        accel_local[i] *= acc_sqrt;   

    // Error is sum of cross product between estimated and measured direction of gravity
    accel_error[0] = (accel_local[1] * AccelVector[2]) - (accel_local[2] * AccelVector[1]);
    accel_error[1] = (accel_local[2] * AccelVector[0]) - (accel_local[0] * AccelVector[2]);
    accel_error[2] = (accel_local[0] * AccelVector[1]) - (accel_local[1] * AccelVector[0]);

    gyro_local[0] += Accel_Kp[0] * accel_error[0];
    gyro_local[1] += Accel_Kp[1] * accel_error[1];
    gyro_local[2] += Accel_Kp[2] * accel_error[2];
    
    memset(w_t, 0, sizeof(w_t));
    w_t[0][1] = -gyro_local[0]; w_t[0][2] = -gyro_local[1]; w_t[0][3] = -gyro_local[2];
    w_t[1][0] =  gyro_local[0]; w_t[1][2] =  gyro_local[2]; w_t[1][3] = -gyro_local[1];
    w_t[2][0] =  gyro_local[1]; w_t[2][1] = -gyro_local[2]; w_t[2][3] =  gyro_local[0];
    w_t[3][0] =  gyro_local[2]; w_t[3][1] =  gyro_local[1]; w_t[3][2] = -gyro_local[0]; 

    for (int32_t i = 0; i < 4; ++i) {
        float tmp = 0.0f;
        for (int32_t j = 0; j < 4; ++j)
            tmp += w_t[i][j] * quat[j];
        quat_add[i] = tmp * timing_time / 2.0f;
    }   

    for (int32_t i = 0; i < 4; ++i)
        quat[i] = quat[i] + quat_add[i];

    float sqrsum = 0.0f;
    for (int32_t i = 0; i < 4; ++i)
        sqrsum += quat[i] * quat[i];
    float invsqrt = Math_InvSqrt(sqrsum);
    for (int32_t i = 0; i < 4; ++i)
        quat[i] *= invsqrt;

    return 1;
}


/**
  * @brief          Calculate the corresponding Euler angle yaw according to the size of the quaternion
  * @param[in]      quaternion array, not NULL
  * @retval         Returned yaw angle yaw in rad
  */
float AHRS_GetYaw(const float quat[4]) {
    float vs0 = (quat[0] * quat[3] + quat[1] * quat[2]) * 2.0f;
    float vs1 = (quat[0] * quat[0] + quat[1] * quat[1]) * 2.0f - 1.0f;
    return Math_Rad2Angle(atan2f(vs0, vs1));
}


/**
  * @brief          Calculate the corresponding Euler angle pitch according to the size of the quaternion
  * @param[in]      quaternion array, not NULL
  * @retval         Returned pitch angle yaw in rad
  */
float AHRS_GetPitch(const float quat[4]) {
    float vs0 = (quat[0] * quat[2] - quat[1] * quat[3]) * 2.0f;
    return Math_Rad2Angle(asinf(vs0));
}


/**
  * @brief          Calculate the corresponding Euler angle roll angle according to the size of the quaternion
  * @param[in]      quaternion array, not NULL
  * @retval         Return the roll angle roll in rad
  */
float AHRS_GetRoll(const float quat[4]) {
    float vs0 = (quat[0] * quat[1] + quat[2] * quat[3]) * 2.0f;
    float vs1 = (quat[0] * quat[0] + quat[3] * quat[3]) * 2.0f - 1.0f;
    return Math_Rad2Angle(atan2f(vs0, vs1));
}


/**
  * @brief      AHRS Quad Array initnation
  * @param      q[4] :The Quad Array 
  * @param      yaw :Yaw axis data pointer
  * @param      pitch :Pitch axis data pointer
  * @param      roll :roll axis data pointer
  * @retval     NULL
  */
void AHRS_GetAngle(float q[4], float *yaw, float *pitch, float *roll) {
    if (!q || !yaw || !pitch || !roll) {
        *yaw   = AHRS_GetYaw(q);
        *pitch = AHRS_GetPitch(q);
        *roll  = AHRS_GetRoll(q);
    }
} 


/* * 
  * @brief      AHRS data update function
  * @param      q[4] :The Quad Array needs to be updated
  * @param      gx :x-axis data of gyroscope
  * @param      gy :y-axis data of gyroscope
  * @param      gz :z-axis data of gyroscope
  * @param      ax :Accelerometer x-axis data
  * @param      ay :Accelerometer y-axis data
  * @param      za :Accelerometer z-axis data
  * @param      mx :Geomagnetic pole x-axis data
  * @param      my :Geomagnetic pole y-axis data
  * @param      mz :Geomagnetic pole z-axis data
  * @retval     NULL
  */
void AHRS_MahonyUpdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
    static float sampleFreq, integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
    static uint32_t ahrs_cont = 0;
    float twoKi = (0.06f * 0.5f);	            // 2 * integral gain
    float twoKp = (10.0f * 0.5f);	            // 2 * proportional gain    

    if (ahrs_cont == 0) {
        ahrs_cont = DWT_GetTimeline_ms();
        sampleFreq = 500.0f;
    }
    else {
        ahrs_cont++;
        sampleFreq = ahrs_cont * 1000 / DWT_GetTimeline_ms();
    }
    
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		AHRS_MahonyUpdateIMU(q, gx, gy, gz, ax, ay, az);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = Math_InvSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = Math_InvSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q[0] * q[0];
        q0q1 = q[0] * q[1];
        q0q2 = q[0] * q[2];
        q0q3 = q[0] * q[3];
        q1q1 = q[1] * q[1];
        q1q2 = q[1] * q[2];
        q1q3 = q[1] * q[3];
        q2q2 = q[2] * q[2];
        q2q3 = q[2] * q[3];
        q3q3 = q[3] * q[3];   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;

        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = Math_InvSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}


/**
  * @brief      AHRS data update function (There is no geomagnetic pole)
  * @param      q[4] :The Quad Array needs to be updated
  * @param      gx :x-axis data of gyroscope
  * @param      gy :y-axis data of gyroscope
  * @param      gz :z-axis data of gyroscope
  * @param      ax :Accelerometer x-axis data
  * @param      ay :Accelerometer y-axis data
  * @param      za :Accelerometer z-axis data
  * @retval     NULL
  */
void AHRS_MahonyUpdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az) {
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
    static float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
    static uint32_t ahrs_cont = 0;
    float sampleFreq;
    float twoKi = (0.06f * 0.5f);	            // 2 * integral gain
    float twoKp = (10.0f * 0.5f);	            // 2 * proportional gain    
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        if (ahrs_cont == 0) {
            ahrs_cont = DWT_GetTimeline_ms();
            sampleFreq = 500.0f;
        }
        else {
            ahrs_cont++;
            sampleFreq = ahrs_cont * 1000 / DWT_GetTimeline_ms();
        }
		// Normalise accelerometer measurement
		recipNorm = Math_InvSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q[1] * q[3] - q[0] * q[2];
		halfvy = q[0] * q[1] + q[2] * q[3];
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = Math_InvSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}


/* * 
  * @brief      AHRS Madgwick data update function
  * @param      q[4] :The Quad Array needs to be updated
  * @param      gx :x-axis data of gyroscope
  * @param      gy :y-axis data of gyroscope
  * @param      gz :z-axis data of gyroscope
  * @param      ax :Accelerometer x-axis data
  * @param      ay :Accelerometer y-axis data
  * @param      za :Accelerometer z-axis data
  * @param      mx :Geomagnetic pole x-axis data
  * @param      my :Geomagnetic pole y-axis data
  * @param      mz :Geomagnetic pole z-axis data
  * @retval     NULL
  */
void AHRS_MadgwickUpdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    static uint32_t ahrs_cont = 0;
    float sampleFreq;
    float beta = (10.0f * 0.5f);	            // 2 * proportional gain

    if (ahrs_cont == 0) {
        ahrs_cont = DWT_GetTimeline_ms();
        sampleFreq = 500.0f;
    }
    else {
        ahrs_cont++;
        sampleFreq = ahrs_cont * 1000 / DWT_GetTimeline_ms();
    }
    
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		AHRS_MadgwickUpdateIMU(q, gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
	qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
	qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
	qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = Math_InvSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = Math_InvSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q[0] * mx;
		_2q0my = 2.0f * q[0] * my;
		_2q0mz = 2.0f * q[0] * mz;
		_2q1mx = 2.0f * q[1] * mx;
		_2q0 = 2.0f * q[0];
		_2q1 = 2.0f * q[1];
		_2q2 = 2.0f * q[2];
		_2q3 = 2.0f * q[3];
		_2q0q2 = 2.0f * q[0] * q[2];
		_2q2q3 = 2.0f * q[2] * q[3];
		q0q0 = q[0] * q[0];
		q0q1 = q[0] * q[1];
		q0q2 = q[0] * q[2];
		q0q3 = q[0] * q[3];
		q1q1 = q[1] * q[1];
		q1q2 = q[1] * q[2];
		q1q3 = q[1] * q[3];
		q2q2 = q[2] * q[2];
		q2q3 = q[2] * q[3];
		q3q3 = q[3] * q[3];

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q[3] + _2q0mz * q[2] + mx * q1q1 + _2q1 * my * q[2] + _2q1 * mz * q[3] - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q[3] + my * q0q0 - _2q0mz * q[1] + _2q1mx * q[2] - my * q1q1 + my * q2q2 + _2q2 * mz * q[3] - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q[2] + _2q0my * q[1] + mz * q0q0 + _2q1mx * q[3] - mz * q1q1 + _2q2 * my * q[3] - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q[2] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q[3] + _2bz * q[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q[1] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q[3] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q[2] + _2bz * q[0]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q[3] - _4bz * q[1]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q[2] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q[2] - _2bz * q[0]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q[1] + _2bz * q[3]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q[0] - _4bz * q[2]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q[3] + _2bz * q[1]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q[0] + _2bz * q[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = Math_InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q[0] += qDot1 * (1.0f / sampleFreq);
	q[1] += qDot2 * (1.0f / sampleFreq);
	q[2] += qDot3 * (1.0f / sampleFreq);
	q[3] += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = Math_InvSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}


/**
  * @brief      AHRS Madgwick data update function (There is no geomagnetic pole)
  * @param      q[4] :The Quad Array needs to be updated
  * @param      gx :x-axis data of gyroscope
  * @param      gy :y-axis data of gyroscope
  * @param      gz :z-axis data of gyroscope
  * @param      ax :Accelerometer x-axis data
  * @param      ay :Accelerometer y-axis data
  * @param      za :Accelerometer z-axis data
  * @retval     NULL
  */
void AHRS_MadgwickUpdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az) {
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    volatile float beta = (10.0f * 0.5f);	            // 2 * proportional gain
    static uint32_t ahrs_cont = 0;
    float sampleFreq;
    
    if (ahrs_cont == 0) {
        ahrs_cont = DWT_GetTimeline_ms();
        sampleFreq = 500.0f;
    }
    else {
        ahrs_cont++;
        sampleFreq = ahrs_cont * 1000 / DWT_GetTimeline_ms();
    }

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
	qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
	qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
	qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = Math_InvSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q[0];
		_2q1 = 2.0f * q[1];
		_2q2 = 2.0f * q[2];
		_2q3 = 2.0f * q[3];
		_4q0 = 4.0f * q[0];
		_4q1 = 4.0f * q[1];
		_4q2 = 4.0f * q[2];
		_8q1 = 8.0f * q[1];
		_8q2 = 8.0f * q[2];
		q0q0 = q[0] * q[0];
		q1q1 = q[1] * q[1];
		q2q2 = q[2] * q[2];
		q3q3 = q[3] * q[3];

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q[3] - _2q1 * ax + 4.0f * q2q2 * q[3] - _2q2 * ay;
		recipNorm = Math_InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q[0] += qDot1 * (1.0f / sampleFreq);
	q[1] += qDot2 * (1.0f / sampleFreq);
	q[2] += qDot3 * (1.0f / sampleFreq);
	q[3] += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = Math_InvSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}


/**
  * @brief          Returns the current gravitational acceleration
  * @param[in]      NULL
  * @retval         Returns the acceleration of gravity in m/s2
  */
float AHRS_GetCarrierGravity(void) {
    return carrier_gravity;
}


/**
 * @brief        : Convert angle to quaternion
 * @param         [float] quat
 * @param         [float] yaw
 * @param         [float] pitch
 * @param         [float] roll
 * @return        [type]
 */
void AHRS_AngleToQuat(float quat[4], const float yaw, const float pitch, const float roll) {
    float angle_sin[3], angle_cos[3];
    angle_sin[0] = sinf(yaw / 2.0f);
    angle_sin[1] = sinf(pitch / 2.0f);
    angle_sin[2] = sinf(roll / 2.0f);
    angle_cos[0] = cosf(yaw / 2.0f);
    angle_cos[1] = cosf(pitch / 2.0f);
    angle_cos[2] = cosf(roll / 2.0f);
    
    quat[0] = angle_sin[0] * angle_sin[1] * angle_sin[2] + angle_cos[0] * angle_cos[1] * angle_cos[2];
    quat[1] = angle_cos[0] * angle_cos[1] * angle_sin[2] - angle_sin[0] * angle_sin[1] * angle_cos[2];
    quat[2] = angle_cos[0] * angle_sin[1] * angle_cos[2] + angle_sin[0] * angle_cos[1] * angle_sin[2];
    quat[3] = angle_sin[0] * angle_cos[1] * angle_cos[2] - angle_cos[0] * angle_sin[1] * angle_sin[2];
}


/**
 * @brief        : Returns the current position altitude
 * @param         [float] *height
 * @return        [type]
 */
void AHRS_GetHeight(float *height) {
    *height = 0.0f;
}


/**
 * @brief        : Returns the current position dimension
 * @param         [float] *latit
 * @return        [type]
 */
void AHRS_GetLatitude(float *latit) {
    *latit = 22.0f;
}
