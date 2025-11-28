#ifndef EKF_H
#define EKF_H

#include "simple_matrix.h"
#include <cmath>

struct Quaternion {
    float w, x, y, z;
    
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}
    
    void normalize() {
        float norm = std::sqrt(w*w + x*x + y*y + z*z);
        if (norm > 0) { w/=norm; x/=norm; y/=norm; z/=norm; }
    }
    
    Quaternion conjugate() const { return Quaternion(w, -x, -y, -z); }
    
    Quaternion operator*(const Quaternion& other) const {
        return Quaternion(
            w*other.w - x*other.x - y*other.y - z*other.z,
            w*other.x + x*other.w + y*other.z - z*other.y,
            w*other.y - x*other.z + y*other.w + z*other.x,
            w*other.z + x*other.y - y*other.x + z*other.w
        );
    }
};

class EKF {
public:
    // Nominal State
    float px, py, pz;
    float vx, vy, vz;
    Quaternion q;
    float bax, bay, baz;
    float bgx, bgy, bgz;

    // Error Covariance (15x15)
    // Order: dp, dv, dtheta, dba, dbg
    Mat P;
    
    // Process Noise
    float sigma_acc;
    float sigma_gyro;
    float sigma_acc_bias;
    float sigma_gyro_bias;

    bool initialized;

    EKF();
    void reset();
    
    // Initialization using Accel + Mag
    void initQuaternion(float ax, float ay, float az, float mx, float my, float mz);
    void initFromAccel(float ax, float ay, float az);

    // Prediction Step
    void predict(float dt, float ax, float ay, float az, float gx, float gy, float gz);
    void predictLinearAccel(float dt, float lax, float lay, float laz, float gx, float gy, float gz);
    
    // Updates
    void updateMag(float mx, float my, float mz, float magNoise);
    void updateZeroVelocity(float velNoise);
    void updatePosition(float x, float y, float z, float posNoise);
    void updateHeight(float z, float heightNoise);
    void updateGravity(float ax, float ay, float az, float gravityNoise);
    
    // Getters
    Quaternion getOrientation() const { return q; }
    void getPosition(float& x, float& y, float& z) const { x=px; y=py; z=pz; }
    void getVelocity(float& x, float& y, float& z) const { x=vx; y=vy; z=vz; }

private:
    Mat getRotationMatrix(const Quaternion& q);
};

#endif
