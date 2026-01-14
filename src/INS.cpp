#include "Arduino.h"
#include "INS.h"
#include "ArduHelpers.h"




Quaternionf statAttQuat(const Vector3f& b_a_s,
                        const Vector3f& b_m_s,
                        const Vector3f& n_M,
                        const Vector3f& n_G)
{
    Vector3f D_n = n_G.normalized();
    Vector3f M_n = n_M.normalized();

    Vector3f N_n = (M_n - (M_n.dot(D_n)) * D_n).normalized();
    Vector3f E_n = D_n.cross(N_n);

    Vector3f D_b = -b_a_s.normalized();
    Vector3f M_b = b_m_s.normalized();

    Vector3f N_b = (M_b - (M_b.dot(D_b)) * D_b).normalized();
    Vector3f E_b = D_b.cross(N_b);

    Matrix3f R_n;
    R_n.col(0) = N_n;
    R_n.col(1) = E_n;
    R_n.col(2) = D_n;

    Matrix3f R_b;
    R_b.col(0) = N_b;
    R_b.col(1) = E_b;
    R_b.col(2) = D_b;

    Matrix3f R_bn = R_n * R_b.transpose();
    return Quaternionf(R_bn);
}




Vector3f quatToEuler(const Quaternionf&      q,
                           NavFrame          frame,
                           RotationType      type,
                           AngleUnit         angle,
                           RotationDirection dir)
{
    // Normalize quaternion for safety
    Quaternionf q_use = q.normalized();

    // If quaternion represents Nav → Body, invert it to Body → Nav
    if (dir == RotationDirection::NavToBody)
    {
        q_use = q_use.conjugate();
    }

    // Body → navigation rotation matrix
    Matrix3f R = q_use.toRotationMatrix();

    // Convert ENU ↔ NED if required
    if (frame == NavFrame::ENU)
    {
        Matrix3f T;
        T << 0, 1, 0,
             1, 0, 0,
             0, 0,-1;
        R = T * R * T.transpose();
    }

    float roll, pitch, yaw;

    if (type == RotationType::Intrinsic)
    {
        // Intrinsic ZYX (yaw–pitch–roll)
        roll  =  atan2(R(2,1), R(2,2));
        pitch = -asin(R(2,0));
        yaw   =  atan2(R(1,0), R(0,0));
    }
    else
    {
        // Extrinsic ZYX == Intrinsic XYZ
        roll  =  atan2(R(1,2), R(2,2));
        pitch = -asin(R(0,2));
        yaw   =  atan2(R(0,1), R(0,0));
    }

    Vector3f euler(roll, pitch, yaw);

    // Convert to degrees if requested
    if (angle == AngleUnit::Degrees)
    {
        euler *= 180.0f / F_PI;
    }

    return euler;
}




Quaternionf deltaQuatFromGyro(const Vector3f& omega_b, float dt)
{
    Vector3f dtheta = omega_b * dt;
    float angle = dtheta.norm();

    if (angle < 1e-8f)
        return Quaternionf::Identity();

    Vector3f axis = dtheta / angle;
    return Quaternionf(AngleAxisf(angle, axis));
}




Vector3f CompFilt::get_b_R_ang_n()
{
    return quatToEuler(q_bn);
}




Quaternionf CompFilt::get_b_R_quat_n()
{
    return q_bn;
}




void CompFilt::updateMag(const Vector3f& _n_M)
{
    n_M = _n_M;
}




void CompFilt::updateGrav(const Vector3f& _n_G)
{
    n_G = _n_G;
}




void CompFilt::updateTau(float _tau)
{
    tau = _tau;
}




// https://www.hackster.io/hibit/complementary-filter-and-relative-orientation-with-mpu9250-d4f79d
// https://youtu.be/0rlvvYgmTvI?t=297
void CompFilt::update(const Vector3f& b_a_s,
                      const Vector3f& b_g_s,
                      const Vector3f& b_m_s,
                      unsigned long   _timestamp)
{
    // Handle timestamping
    if (_timestamp >= 0)
        timestamp = _timestamp;
    else
        timestamp = micros();
    
    dt = (timestamp - prevTimestamp) / 1000000.0;

    prevTimestamp = timestamp;

    Quaternionf q_am = statAttQuat(b_a_s,
                                   b_m_s,
                                   n_M,
                                   n_G);

    // Only use gyro data after the filter has been initialized/primed with the accelerometer and magnetometer
    if (isInit)
    {
        // 1. Gyro integration
        Quaternionf dq  = deltaQuatFromGyro(b_g_s, dt);
        Quaternionf q_g = q_bn * dq;

        // 2. Error quaternion
        Quaternionf q_err = q_am * q_g.conjugate();

        // 3. Gain
        float alpha = 1.0f - expf(-dt / tau);

        // 4. Apply partial correction
        AngleAxisf  aa(q_err);
        Quaternionf q_corr(AngleAxisf(alpha * aa.angle(),
                           aa.axis()));

        // 5. Update estimate
        q_bn = q_corr * q_g;
        q_bn.normalize();
    }
    else
    {
        q_bn   = q_am;
        isInit = true;
    }
}
