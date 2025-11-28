#include "ekf.h"
#include <cmath>
#include <algorithm>

#define G_CONST 9.81f

EKF::EKF() {
    reset();
}

void EKF::reset() {
    px = py = pz = 0;
    vx = vy = vz = 0;
    q = Quaternion(1, 0, 0, 0);
    bax = bay = baz = 0;
    bgx = bgy = bgz = 0;

    P = Mat::identity(15) * 0.01f;
    
    // Initial uncertainties
    P(0,0)=P(1,1)=P(2,2) = 1.0f; // Pos
    P(3,3)=P(4,4)=P(5,5) = 0.1f; // Vel
    P(6,6)=P(7,7)=P(8,8) = 0.5f; // Angle (Increased for faster initial convergence)
    P(9,9)=P(10,10)=P(11,11) = 0.001f; // Acc Bias
    P(12,12)=P(13,13)=P(14,14) = 0.0001f; // Gyro Bias

    // Tuned Noise parameters
    sigma_acc = 0.8f;
    sigma_gyro = 0.1f;
    sigma_acc_bias = 1.0e-4f;
    sigma_gyro_bias = 1.0e-5f;
    
    initialized = false;
}

void EKF::initQuaternion(float ax, float ay, float az, float mx, float my, float mz) {
    // 1. Normalize Accel (Up vector in Body)
    float normA = std::sqrt(ax*ax + ay*ay + az*az);
    if (normA < 0.1f) return; // Invalid accel
    float ax_n = ax/normA;
    float ay_n = ay/normA;
    float az_n = az/normA;

    // 2. Normalize Mag
    float normM = std::sqrt(mx*mx + my*my + mz*mz);
    if (normM < 0.1f) return; // Invalid mag
    float mx_n = mx/normM;
    float my_n = my/normM;
    float mz_n = mz/normM;

    // 3. Compute East = Mag x Accel (Cross product)
    // Mag is roughly North. Accel is Up. North x Up = East.
    // E = m x a
    float ex = my_n*az_n - mz_n*ay_n;
    float ey = mz_n*ax_n - mx_n*az_n;
    float ez = mx_n*ay_n - my_n*ax_n;
    float normE = std::sqrt(ex*ex + ey*ey + ez*ez);
    if (normE < 0.01f) return; // Mag and Accel parallel?
    ex /= normE; ey /= normE; ez /= normE;

    // 4. Compute North = Accel x East
    // N = a x E
    float nx = ay_n*ez - az_n*ey;
    float ny = az_n*ex - ax_n*ez;
    float nz = ax_n*ey - ay_n*ex;
    
    // 5. Rotation Matrix rows are E, N, A (Up)
    // R_wb = [ E_b^T ]
    //        [ N_b^T ]
    //        [ A_b^T ]
    // This R transforms World vector to Body?
    // No. 
    // Let's check: R * [1, 0, 0]^T (East in World) = Col 0 of R.
    // If Rows are E_b, N_b, A_b.
    // R * [1,0,0] = [ex, nx, ax_n]^T. This is NOT East in Body.
    // Wait.
    // R_body_from_world = [E_b, N_b, A_b] (Columns).
    // R_world_from_body = (R_body_from_world)^T.
    // So R_world_from_body has E_b, N_b, A_b as ROWS.
    // Let's re-verify.
    // v_w = R_wb * v_b.
    // v_b = R_bw * v_w.
    // v_b = R_bw * [1,0,0] (East World) = Col 0 of R_bw.
    // Col 0 of R_bw should be East vector in Body frame.
    // So Col 0 of R_bw is E_b.
    // So R_bw = [E_b, N_b, A_b].
    // And R_wb = R_bw^T.
    // So R_wb has E_b, N_b, A_b as ROWS.
    
    Mat R(3,3);
    R(0,0) = ex; R(0,1) = ey; R(0,2) = ez;
    R(1,0) = nx; R(1,1) = ny; R(1,2) = nz;
    R(2,0) = ax_n; R(2,1) = ay_n; R(2,2) = az_n;
    
    // Convert R to Quaternion
    float tr = R(0,0) + R(1,1) + R(2,2);
    float w, x, y, z;
    if (tr > 0) {
        float S = std::sqrt(tr + 1.0f) * 2.0f;
        w = 0.25f * S;
        x = (R(2,1) - R(1,2)) / S;
        y = (R(0,2) - R(2,0)) / S;
        z = (R(1,0) - R(0,1)) / S;
    } else if ((R(0,0) > R(1,1)) && (R(0,0) > R(2,2))) {
        float S = std::sqrt(1.0f + R(0,0) - R(1,1) - R(2,2)) * 2.0f;
        w = (R(2,1) - R(1,2)) / S;
        x = 0.25f * S;
        y = (R(0,1) + R(1,0)) / S;
        z = (R(0,2) + R(2,0)) / S;
    } else if (R(1,1) > R(2,2)) {
        float S = std::sqrt(1.0f + R(1,1) - R(0,0) - R(2,2)) * 2.0f;
        w = (R(0,2) - R(2,0)) / S;
        x = (R(0,1) + R(1,0)) / S;
        y = 0.25f * S;
        z = (R(1,2) + R(2,1)) / S;
    } else {
        float S = std::sqrt(1.0f + R(2,2) - R(0,0) - R(1,1)) * 2.0f;
        w = (R(1,0) - R(0,1)) / S;
        x = (R(0,2) + R(2,0)) / S;
        y = (R(1,2) + R(2,1)) / S;
        z = 0.25f * S;
    }
    
    q = Quaternion(w, x, y, z);
    q.normalize();
    initialized = true;
}

void EKF::initFromAccel(float ax, float ay, float az) {
    float normA = std::sqrt(ax*ax + ay*ay + az*az);
    if (normA < 0.1f) return;
    float ax_n = ax/normA;
    float ay_n = ay/normA;
    float az_n = az/normA;

    float ex, ey, ez;
    float nx, ny, nz;

    float cx = 1.0f, cy = 0.0f, cz = 0.0f;
    ex = cy*az_n - cz*ay_n;
    ey = cz*ax_n - cx*az_n;
    ez = cx*ay_n - cy*ax_n;
    float normE = std::sqrt(ex*ex + ey*ey + ez*ez);
    if (normE < 0.01f) {
        cx = 0.0f; cy = 1.0f; cz = 0.0f;
        ex = cy*az_n - cz*ay_n;
        ey = cz*ax_n - cx*az_n;
        ez = cx*ay_n - cy*ax_n;
        normE = std::sqrt(ex*ex + ey*ey + ez*ez);
        if (normE < 0.01f) return;
    }
    ex /= normE; ey /= normE; ez /= normE;

    nx = ay_n*ez - az_n*ey;
    ny = az_n*ex - ax_n*ez;
    nz = ax_n*ey - ay_n*ex;

    Mat R(3,3);
    R(0,0) = ex; R(0,1) = ey; R(0,2) = ez;
    R(1,0) = nx; R(1,1) = ny; R(1,2) = nz;
    R(2,0) = ax_n; R(2,1) = ay_n; R(2,2) = az_n;

    float tr = R(0,0) + R(1,1) + R(2,2);
    float w, x, y, z;
    if (tr > 0) {
        float S = std::sqrt(tr + 1.0f) * 2.0f;
        w = 0.25f * S;
        x = (R(2,1) - R(1,2)) / S;
        y = (R(0,2) - R(2,0)) / S;
        z = (R(1,0) - R(0,1)) / S;
    } else if ((R(0,0) > R(1,1)) && (R(0,0) > R(2,2))) {
        float S = std::sqrt(1.0f + R(0,0) - R(1,1) - R(2,2)) * 2.0f;
        w = (R(2,1) - R(1,2)) / S;
        x = 0.25f * S;
        y = (R(0,1) + R(1,0)) / S;
        z = (R(0,2) + R(2,0)) / S;
    } else if (R(1,1) > R(2,2)) {
        float S = std::sqrt(1.0f + R(1,1) - R(0,0) - R(2,2)) * 2.0f;
        w = (R(0,2) - R(2,0)) / S;
        x = (R(0,1) + R(1,0)) / S;
        y = 0.25f * S;
        z = (R(1,2) + R(2,1)) / S;
    } else {
        float S = std::sqrt(1.0f + R(2,2) - R(0,0) - R(1,1)) * 2.0f;
        w = (R(1,0) - R(0,1)) / S;
        x = (R(0,2) + R(2,0)) / S;
        y = (R(1,2) + R(2,1)) / S;
        z = 0.25f * S;
    }

    q = Quaternion(w, x, y, z);
    q.normalize();
    initialized = true;
}

Mat EKF::getRotationMatrix(const Quaternion& q) {
    Mat R(3, 3);
    float w=q.w, x=q.x, y=q.y, z=q.z;
    float x2=x*x, y2=y*y, z2=z*z;
    float xy=x*y, xz=x*z, yz=y*z;
    float wx=w*x, wy=w*y, wz=w*z;

    R(0,0) = 1 - 2*(y2 + z2);
    R(0,1) = 2*(xy - wz);
    R(0,2) = 2*(xz + wy);
    
    R(1,0) = 2*(xy + wz);
    R(1,1) = 1 - 2*(x2 + z2);
    R(1,2) = 2*(yz - wx);
    
    R(2,0) = 2*(xz - wy);
    R(2,1) = 2*(yz + wx);
    R(2,2) = 1 - 2*(x2 + y2);
    
    return R;
}

void EKF::predict(float dt, float ax, float ay, float az, float gx, float gy, float gz) {
    if (!initialized) return;

    // 1. Nominal State Propagation
    float ax_c = ax - bax;
    float ay_c = ay - bay;
    float az_c = az - baz;
    float gx_c = gx - bgx;
    float gy_c = gy - bgy;
    float gz_c = gz - bgz;

    Mat R = getRotationMatrix(q);
    
    // Accel in World Frame
    float acc_w_x = R(0,0)*ax_c + R(0,1)*ay_c + R(0,2)*az_c;
    float acc_w_y = R(1,0)*ax_c + R(1,1)*ay_c + R(1,2)*az_c;
    float acc_w_z = R(2,0)*ax_c + R(2,1)*ay_c + R(2,2)*az_c - G_CONST;

    float a_mag = std::sqrt(acc_w_x*acc_w_x + acc_w_y*acc_w_y + acc_w_z*acc_w_z);
    if (a_mag < 0.05f) { acc_w_x = 0; acc_w_y = 0; acc_w_z = 0; }

    px += vx * dt + 0.5f * acc_w_x * dt * dt;
    py += vy * dt + 0.5f * acc_w_y * dt * dt;
    pz += vz * dt + 0.5f * acc_w_z * dt * dt;

    vx += acc_w_x * dt;
    vy += acc_w_y * dt;
    vz += acc_w_z * dt;

    float v_lim = 3.0f;
    if (vx > v_lim) vx = v_lim; if (vx < -v_lim) vx = -v_lim;
    if (vy > v_lim) vy = v_lim; if (vy < -v_lim) vy = -v_lim;
    if (vz > v_lim) vz = v_lim; if (vz < -v_lim) vz = -v_lim;

    // Velocity damping when acceleration is near zero
    float g_mag = std::sqrt(gx_c*gx_c + gy_c*gy_c + gz_c*gz_c);
    if (a_mag < 0.1f && g_mag < 0.2f) {
        float damping = std::exp(-dt * 5.0f);
        vx *= damping; vy *= damping; vz *= damping;
        float vmag = std::sqrt(vx*vx + vy*vy + vz*vz);
        if (vmag < 0.02f) { vx = 0; vy = 0; vz = 0; }
    }

    // Quaternion Integration
    float angle = std::sqrt(gx_c*gx_c + gy_c*gy_c + gz_c*gz_c) * dt;
    if (angle > 1e-6f) {
        float s = std::sin(angle/2.0f);
        float c = std::cos(angle/2.0f);
        Quaternion q_delta(c, (gx_c*dt/angle)*s, (gy_c*dt/angle)*s, (gz_c*dt/angle)*s);
        q = q * q_delta;
        q.normalize();
    }

    // 2. Error State Covariance
    Mat F = Mat::identity(15);
    
    // dPos/dVel
    F(0,3) = dt; F(1,4) = dt; F(2,5) = dt;

    // dVel/dTheta = -R * [a_body]x * dt
    Mat SkewA(3, 3);
    SkewA(0,1) = -az_c; SkewA(0,2) = ay_c;
    SkewA(1,0) = az_c;  SkewA(1,2) = -ax_c;
    SkewA(2,0) = -ay_c; SkewA(2,1) = ax_c;
    
    Mat F_v_theta = (R * SkewA) * (-dt);
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) F(3+i, 6+j) = F_v_theta(i,j);

    // dVel/dBa = -R * dt
    Mat F_v_ba = R * (-dt);
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) F(3+i, 9+j) = F_v_ba(i,j);

    // dTheta/dTheta = I - [w]x * dt (approx)
    F(6,7) = gz_c*dt; F(6,8) = -gy_c*dt;
    F(7,6) = -gz_c*dt; F(7,8) = gx_c*dt;
    F(8,6) = gy_c*dt; F(8,7) = -gx_c*dt;

    // dTheta/dBg = -I * dt
    F(6,12) = -dt; F(7,13) = -dt; F(8,14) = -dt;

    Mat Q = Mat::identity(15);
    float q_vel = sigma_acc * dt * dt;
    Q(3,3)=Q(4,4)=Q(5,5) = q_vel;
    float q_ang = sigma_gyro * dt * dt;
    Q(6,6)=Q(7,7)=Q(8,8) = q_ang;
    float q_ba = sigma_acc_bias * dt;
    Q(9,9)=Q(10,10)=Q(11,11) = q_ba;
    float q_bg = sigma_gyro_bias * dt;
    Q(12,12)=Q(13,13)=Q(14,14) = q_bg;
    
    P = F * P * F.transpose() + Q;
}

void EKF::predictLinearAccel(float dt, float lax, float lay, float laz, float gx, float gy, float gz) {
    if (!initialized) return;

    float ax_c = lax - bax;
    float ay_c = lay - bay;
    float az_c = laz - baz;
    float gx_c = gx - bgx;
    float gy_c = gy - bgy;
    float gz_c = gz - bgz;

    Mat R = getRotationMatrix(q);
    float acc_w_x = R(0,0)*ax_c + R(0,1)*ay_c + R(0,2)*az_c;
    float acc_w_y = R(1,0)*ax_c + R(1,1)*ay_c + R(1,2)*az_c;
    float acc_w_z = R(2,0)*ax_c + R(2,1)*ay_c + R(2,2)*az_c;

    float a_mag = std::sqrt(acc_w_x*acc_w_x + acc_w_y*acc_w_y + acc_w_z*acc_w_z);
    if (a_mag < 0.05f) { acc_w_x = 0; acc_w_y = 0; acc_w_z = 0; }

    px += vx * dt + 0.5f * acc_w_x * dt * dt;
    py += vy * dt + 0.5f * acc_w_y * dt * dt;
    pz += vz * dt + 0.5f * acc_w_z * dt * dt;

    vx += acc_w_x * dt;
    vy += acc_w_y * dt;
    vz += acc_w_z * dt;

    float v_lim = 3.0f;
    if (vx > v_lim) vx = v_lim; if (vx < -v_lim) vx = -v_lim;
    if (vy > v_lim) vy = v_lim; if (vy < -v_lim) vy = -v_lim;
    if (vz > v_lim) vz = v_lim; if (vz < -v_lim) vz = -v_lim;

    // Velocity damping when acceleration is near zero
    float g_mag = std::sqrt(gx_c*gx_c + gy_c*gy_c + gz_c*gz_c);
    if (a_mag < 0.1f && g_mag < 0.2f) {
        float damping = std::exp(-dt * 5.0f);
        vx *= damping; vy *= damping; vz *= damping;
        float vmag = std::sqrt(vx*vx + vy*vy + vz*vz);
        if (vmag < 0.02f) { vx = 0; vy = 0; vz = 0; }
    }

    float angle = std::sqrt(gx_c*gx_c + gy_c*gy_c + gz_c*gz_c) * dt;
    if (angle > 1e-6f) {
        float s = std::sin(angle/2.0f);
        float c = std::cos(angle/2.0f);
        Quaternion q_delta(c, (gx_c*dt/angle)*s, (gy_c*dt/angle)*s, (gz_c*dt/angle)*s);
        q = q * q_delta;
        q.normalize();
    }

    Mat F = Mat::identity(15);
    F(0,3) = dt; F(1,4) = dt; F(2,5) = dt;
    Mat SkewA(3, 3);
    SkewA(0,1) = -az_c; SkewA(0,2) = ay_c;
    SkewA(1,0) = az_c;  SkewA(1,2) = -ax_c;
    SkewA(2,0) = -ay_c; SkewA(2,1) = ax_c;
    Mat F_v_theta = (R * SkewA) * (-dt);
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) F(3+i, 6+j) = F_v_theta(i,j);
    Mat F_v_ba = R * (-dt);
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) F(3+i, 9+j) = F_v_ba(i,j);
    F(6,7) = gz_c*dt; F(6,8) = -gy_c*dt;
    F(7,6) = -gz_c*dt; F(7,8) = gx_c*dt;
    F(8,6) = gy_c*dt; F(8,7) = -gx_c*dt;
    F(6,12) = -dt; F(7,13) = -dt; F(8,14) = -dt;
    Mat Q = Mat::identity(15);
    float q_vel = sigma_acc * dt * dt;
    Q(3,3)=Q(4,4)=Q(5,5) = q_vel;
    float q_ang = sigma_gyro * dt * dt;
    Q(6,6)=Q(7,7)=Q(8,8) = q_ang;
    float q_ba = sigma_acc_bias * dt;
    Q(9,9)=Q(10,10)=Q(11,11) = q_ba;
    float q_bg = sigma_gyro_bias * dt;
    Q(12,12)=Q(13,13)=Q(14,14) = q_bg;
    P = F * P * F.transpose() + Q;
}

void EKF::updateZeroVelocity(float velNoise) {
    if (!initialized) return;

    // H = [0 I 0 0 0]
    Mat H(3, 15);
    H(0,3) = 1; H(1,4) = 1; H(2,5) = 1;
    
    Mat V = Mat::identity(3) * velNoise;
    Mat PHt = P * H.transpose();
    Mat S = H * PHt + V;
    Mat Sinv = S.inverse();
    Mat K = PHt * Sinv;
    
    // z = 0, h = v
    Mat y(3,1);
    y(0,0) = 0 - vx;
    y(1,0) = 0 - vy;
    y(2,0) = 0 - vz;
    
    Mat dx = K * y;
    float s00 = S(0,0);
    float s11 = S(1,1);
    float s22 = S(2,2);
    float d2 = (y(0,0)*y(0,0))/std::max(s00,1e-6f) + (y(1,0)*y(1,0))/std::max(s11,1e-6f) + (y(2,0)*y(2,0))/std::max(s22,1e-6f);
    if (d2 > 16.0f) return;
    
    // Update States
    px += dx(0,0); py += dx(1,0); pz += dx(2,0);
    vx += dx(3,0); vy += dx(4,0); vz += dx(5,0);
    
    float droll = dx(6,0); float dpitch = dx(7,0); float dyaw = dx(8,0);
    float angle = std::sqrt(droll*droll + dpitch*dpitch + dyaw*dyaw);
    if (angle > 1e-6f) {
        float s = std::sin(angle/2);
        float c = std::cos(angle/2);
        Quaternion dq(c, droll/angle*s, dpitch/angle*s, dyaw/angle*s);
        q = q * dq;
        q.normalize();
    }
    
    bax += dx(9,0); bay += dx(10,0); baz += dx(11,0);
    bgx += dx(12,0); bgy += dx(13,0); bgz += dx(14,0);
    
    P = (Mat::identity(15) - K * H) * P;
}

void EKF::updateGravity(float ax, float ay, float az, float gravityNoise) {
    if (!initialized) return;
    
    // Normalise measurement
    float norm = std::sqrt(ax*ax + ay*ay + az*az);
    if (norm < 0.1f) return;
    float mx = ax/norm; float my = ay/norm; float mz = az/norm;
    
    // Predicted Gravity in Body Frame = R^T * [0,0,1]^T
    // = Row 2 of R
    Mat R = getRotationMatrix(q);
    float hx = R(2,0); float hy = R(2,1); float hz = R(2,2);
    
    // H = Skew(pred_g)
    Mat H(3, 15);
    H(0, 7) = -hz; H(0, 8) = hy;
    H(1, 6) = hz;  H(1, 8) = -hx;
    H(2, 6) = -hy; H(2, 7) = hx;
    
    Mat V = Mat::identity(3) * gravityNoise;
    Mat PHt = P * H.transpose();
    Mat S = H * PHt + V;
    Mat Sinv = S.inverse();
    Mat K = PHt * Sinv;
    
    Mat y(3,1);
    y(0,0) = mx - hx;
    y(1,0) = my - hy;
    y(2,0) = mz - hz;
    
    Mat dx = K * y;
    float d2 = y(0,0)*y(0,0) + y(1,0)*y(1,0) + y(2,0)*y(2,0);
    if (d2 > 0.5f) return;
    
    // Update (Only affect orientation and bias?)
    // Technically gravity update affects tilt.
    // Bias is observable via velocity update, but here we just update all.
    
    float droll = dx(6,0); float dpitch = dx(7,0); float dyaw = dx(8,0);
    float angle = std::sqrt(droll*droll + dpitch*dpitch + dyaw*dyaw);
    if (angle > 1e-6f) {
        float s = std::sin(angle/2);
        float c = std::cos(angle/2);
        Quaternion dq(c, droll/angle*s, dpitch/angle*s, dyaw/angle*s);
        q = q * dq;
        q.normalize();
    }
    
    // Gravity doesn't directly observe Pos/Vel, but correlations in P might update them.
    // Bias update
    bax += dx(9,0); bay += dx(10,0); baz += dx(11,0);
    bgx += dx(12,0); bgy += dx(13,0); bgz += dx(14,0);
    
    P = (Mat::identity(15) - K * H) * P;
}

void EKF::updateMag(float mx, float my, float mz, float magNoise) {
    if (!initialized) return;
    
    // Normalize
    float norm = std::sqrt(mx*mx + my*my + mz*mz);
    if (norm < 0.1f) return;
    mx/=norm; my/=norm; mz/=norm;
    
    // Rotate to World
    Mat R = getRotationMatrix(q);
    float m_wx = R(0,0)*mx + R(0,1)*my + R(0,2)*mz;
    
    // We expect m_wx = 0 (East component is 0 if North is Y)
    // Residual y = 0 - m_wx
    // H = [0 ... 0 -m_wz m_wy ... ]
    
    float m_wy = R(1,0)*mx + R(1,1)*my + R(1,2)*mz;
    float m_wz = R(2,0)*mx + R(2,1)*my + R(2,2)*mz;
    
    Mat H(1, 15);
    H(0, 7) = -m_wz;
    H(0, 8) = m_wy;
    
    Mat V = Mat::identity(1) * magNoise;
    Mat PHt = P * H.transpose();
    Mat S = H * PHt + V;
    Mat Sinv = S.inverse();
    Mat K = PHt * Sinv;
    
    Mat y(1,1);
    y(0,0) = 0 - m_wx;
    
    Mat dx = K * y;
    float s = S(0,0);
    if (std::abs(y(0,0)) > 3.0f * std::sqrt(std::max(s,1e-6f))) return;
    
    // Apply
    float droll = dx(6,0); float dpitch = dx(7,0); float dyaw = dx(8,0);
    float angle = std::sqrt(droll*droll + dpitch*dpitch + dyaw*dyaw);
    if (angle > 1e-6f) {
        float s = std::sin(angle/2);
        float c = std::cos(angle/2);
        Quaternion dq(c, droll/angle*s, dpitch/angle*s, dyaw/angle*s);
        q = q * dq;
        q.normalize();
    }
    
    bax += dx(9,0); bay += dx(10,0); baz += dx(11,0);
    bgx += dx(12,0); bgy += dx(13,0); bgz += dx(14,0);
    
    P = (Mat::identity(15) - K * H) * P;
}

void EKF::updateHeight(float z, float heightNoise) {
    if (!initialized) return;
    Mat H(1, 15);
    H(0,2) = 1; 
    
    Mat V = Mat::identity(1) * heightNoise;
    Mat PHt = P * H.transpose();
    Mat S = H * PHt + V;
    Mat Sinv = S.inverse();
    Mat K = PHt * Sinv;
    
    Mat y(1,1);
    y(0,0) = z - pz;
    
    Mat dx = K * y;
    float s = S(0,0);
    if (std::abs(y(0,0)) > 3.0f * std::sqrt(std::max(s,1e-6f))) return;
    
    px += dx(0,0); py += dx(1,0); pz += dx(2,0);
    vx += dx(3,0); vy += dx(4,0); vz += dx(5,0);
    
    float droll = dx(6,0); float dpitch = dx(7,0); float dyaw = dx(8,0);
    float angle = std::sqrt(droll*droll + dpitch*dpitch + dyaw*dyaw);
    if (angle > 1e-6f) {
        float s = std::sin(angle/2);
        float c = std::cos(angle/2);
        Quaternion dq(c, droll/angle*s, dpitch/angle*s, dyaw/angle*s);
        q = q * dq;
        q.normalize();
    }
    
    bax += dx(9,0); bay += dx(10,0); baz += dx(11,0);
    bgx += dx(12,0); bgy += dx(13,0); bgz += dx(14,0);
    
    P = (Mat::identity(15) - K * H) * P;
}

void EKF::updatePosition(float x, float y, float z, float posNoise) {
    if (!initialized) return;
    Mat H(3, 15);
    H(0,0) = 1; H(1,1) = 1; H(2,2) = 1;
    
    Mat V = Mat::identity(3) * posNoise;
    Mat PHt = P * H.transpose();
    Mat S = H * PHt + V;
    Mat K = PHt * S.inverse();
    
    Mat residual(3,1);
    residual(0,0) = x - px;
    residual(1,0) = y - py;
    residual(2,0) = z - pz;
    
    Mat dx = K * residual;
    
    px += dx(0,0); py += dx(1,0); pz += dx(2,0);
    vx += dx(3,0); vy += dx(4,0); vz += dx(5,0);
    
    float droll = dx(6,0); float dpitch = dx(7,0); float dyaw = dx(8,0);
    float angle = std::sqrt(droll*droll + dpitch*dpitch + dyaw*dyaw);
    if (angle > 1e-6f) {
        float s = std::sin(angle/2);
        float c = std::cos(angle/2);
        Quaternion dq(c, droll/angle*s, dpitch/angle*s, dyaw/angle*s);
        q = q * dq;
        q.normalize();
    }
    
    bax += dx(9,0); bay += dx(10,0); baz += dx(11,0);
    bgx += dx(12,0); bgy += dx(13,0); bgz += dx(14,0);
    
    P = (Mat::identity(15) - K * H) * P;
}
