#include <jni.h>
#include <cmath>
#include <vector>
#include "ekf.h"

struct FusionState {
    EKF ekf;
    bool hasAccel;
    bool hasGyro;
    bool hasMag;
    bool hasLinAcc;
    bool initialized;
    long long lastTs;
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float lax, lay, laz;
    float flax, flay, flaz;
    float startPressure;
    std::vector<float> pressureBuf;
    int stationaryCount;
    float beta;
    float magNormRef;
};

static FusionState state;

static void ensureSamplePeriod(long long ts) {
    // EKF handles dt in predict, but we need to track lastTs
    // logic moved to sensor callbacks
}

extern "C" JNIEXPORT void JNICALL
Java_com_olsc_t6dof_sensors_NativeFusion_init(JNIEnv*, jobject, jfloat beta) {
    state = FusionState();
    state.beta = beta;
    if (state.beta < 0.1f) state.beta = 0.1f;
    state.ekf.reset();
    state.pressureBuf.reserve(150);
    state.initialized = false;
    state.stationaryCount = 0;
    state.magNormRef = 0;
    state.flax = state.flay = state.flaz = 0;
}

extern "C" JNIEXPORT void JNICALL
Java_com_olsc_t6dof_sensors_NativeFusion_addAccelerometer(JNIEnv*, jobject, jlong ts, jfloat ax, jfloat ay, jfloat az) {
    state.hasAccel = true;
    state.ax = ax; state.ay = ay; state.az = az;
    
    // Check for stationary (ZUPT)
    if (state.hasGyro) {
        float a_mag = std::sqrt(ax*ax + ay*ay + az*az);
        float g_mag = std::sqrt(state.gx*state.gx + state.gy*state.gy + state.gz*state.gz);
        float lin_mag = state.hasLinAcc ? std::sqrt(state.lax*state.lax + state.lay*state.lay + state.laz*state.laz) : 100.0f;
        
        // ZUPT: If stationary
        // Relaxed thresholds with linear accel aid
        bool isStationary = (std::abs(a_mag - 9.81f) < 1.2f && g_mag < 0.25f) || (lin_mag < 0.08f && g_mag < 0.25f);
        
        if (isStationary) { 
            state.stationaryCount++;
        } else {
            state.stationaryCount = 0;
        }
        
        // Trigger updates if stationary for enough samples
        if (state.stationaryCount > 4) { // ~20-40ms
            // Zero Velocity Update (stronger zeroing)
            state.ekf.updateZeroVelocity(0.001f);
            
            // Gravity/Tilt Update
            // Use accelerometer to correct Roll/Pitch drift
            // Trust accel gravity vector when stationary
            state.ekf.updateGravity(ax, ay, az, 0.005f); // Lower noise = higher trust
            
            state.stationaryCount = 5; // Cap counter to avoid overflow
        }
    }
    
    if (!state.initialized && state.hasAccel && !state.hasMag) {
        state.ekf.initFromAccel(state.ax, state.ay, state.az);
        if (state.ekf.initialized) state.initialized = true;
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_olsc_t6dof_sensors_NativeFusion_addGyroscope(JNIEnv*, jobject, jlong ts, jfloat gx, jfloat gy, jfloat gz) {
    state.hasGyro = true;
    state.gx = gx; state.gy = gy; state.gz = gz;
    
    if (state.lastTs == 0) {
        state.lastTs = ts;
        return;
    }
    
    float dt = (ts - state.lastTs) / 1e9f;
    state.lastTs = ts;
    
    if (dt <= 0 || dt > 0.5f) return; // Skip invalid dt
    
    if (state.initialized) {
        if (state.hasLinAcc) {
            float lmx = std::sqrt(state.flax*state.flax + state.flay*state.flay + state.flaz*state.flaz);
            if (lmx < 0.03f) {
                state.ekf.predict(dt, state.ax, state.ay, state.az, state.gx, state.gy, state.gz);
            } else {
                state.ekf.predictLinearAccel(dt, state.flax, state.flay, state.flaz, state.gx, state.gy, state.gz);
            }
        } else {
            state.ekf.predict(dt, state.ax, state.ay, state.az, state.gx, state.gy, state.gz);
        }
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_olsc_t6dof_sensors_NativeFusion_addMagnetometer(JNIEnv*, jobject, jlong ts, jfloat mx, jfloat my, jfloat mz) {
    state.hasMag = true;
    float norm = std::sqrt(mx*mx+my*my+mz*mz);
    if (norm > 0) { mx/=norm; my/=norm; mz/=norm; }
    state.mx = mx; state.my = my; state.mz = mz;
    if (state.magNormRef == 0 && norm > 0) state.magNormRef = norm;

    // Initialize orientation if needed
    if (!state.initialized && state.hasAccel && state.hasMag) {
        state.ekf.initQuaternion(state.ax, state.ay, state.az, state.mx, state.my, state.mz);
        if (state.ekf.initialized) {
            state.initialized = true;
        }
    } else if (state.initialized) {
        float magNoise = 0.2f;
        float dev = std::abs(norm - state.magNormRef);
        if (dev < 5.0f && std::sqrt(state.gx*state.gx + state.gy*state.gy + state.gz*state.gz) < 0.5f) {
            state.ekf.updateMag(mx, my, mz, magNoise);
        }
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_olsc_t6dof_sensors_NativeFusion_addLinearAcceleration(JNIEnv*, jobject, jlong ts, jfloat ax, jfloat ay, jfloat az) {
    state.hasLinAcc = true;
    state.lax = ax; state.lay = ay; state.laz = az;
    float alpha = 0.85f;
    state.flax = alpha*state.flax + (1.0f-alpha)*ax;
    state.flay = alpha*state.flay + (1.0f-alpha)*ay;
    state.flaz = alpha*state.flaz + (1.0f-alpha)*az;
    float m = std::sqrt(state.flax*state.flax + state.flay*state.flay + state.flaz*state.flaz);
    if (m < 0.02f) { state.flax = state.flay = state.flaz = 0; }
}

extern "C" JNIEXPORT void JNICALL
Java_com_olsc_t6dof_sensors_NativeFusion_addPressure(JNIEnv*, jobject, jlong ts, jfloat p) {
    if (state.startPressure == 0) state.startPressure = p;
    state.pressureBuf.push_back(p);
    if (state.pressureBuf.size() > 150) state.pressureBuf.erase(state.pressureBuf.begin());
    float avg=0; for (float v: state.pressureBuf) avg+=v; avg/=state.pressureBuf.size();
    float alt = 44330.0f*(1.0f-std::pow(avg/state.startPressure,0.190294957f));
    
    if (state.initialized) {
        state.ekf.updateHeight(alt, 1.5f);
    }
}

extern "C" JNIEXPORT jfloatArray JNICALL
Java_com_olsc_t6dof_sensors_NativeFusion_getPose(JNIEnv* env, jobject) {
    Quaternion q = state.ekf.getOrientation();
    
    // Correction for 90 degree offset (Model X vs Phone -Z)
    // We want Model X (Red Line) to align with Phone Forward (-Z).
    // This requires a rotation from Model Frame to Phone Frame.
    // Rotate Y 90 deg: X->-Z, Z->X.
    float s = 0.70710678f; // sin(45)
    float c = 0.70710678f; // cos(45)
    Quaternion q_rot(c, 0, s, 0);
    
    // q_final = q_body_to_world * q_model_to_body
    q = q * q_rot;
    
    float px, py, pz;
    state.ekf.getPosition(px, py, pz);

    // EKF tracks ENU (East-North-Up) in World frame.
    // q is Body-to-World rotation.
    // px, py, pz are World coordinates.
    // Output format: [x, y, z, qw, qx, qy, qz]
    
    jfloatArray arr = env->NewFloatArray(7);
    jfloat out[7];
    out[0] = px;
    out[1] = py;
    out[2] = pz;
    out[3] = q.w;
    out[4] = q.x;
    out[5] = q.y;
    out[6] = q.z;
    
    env->SetFloatArrayRegion(arr, 0, 7, out);
    return arr;
}
