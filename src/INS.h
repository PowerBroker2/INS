#pragma once
#include "Arduino.h"
#include "eigen.h"
#include <Eigen/Geometry>

using Eigen::Vector3f;
using Eigen::Matrix3f;
using Eigen::Quaternionf;
using Eigen::AngleAxisf;

/// @brief Navigation frame options
enum class NavFrame
{
    NED, ///< North-East-Down frame
    ENU  ///< East-North-Up frame
};

/// @brief Euler rotation type
enum class RotationType
{
    Intrinsic, ///< Rotation about body axes
    Extrinsic  ///< Rotation about fixed axes
};

/// @brief Angle units for Euler angles
enum class AngleUnit
{
    Radians, ///< Output in radians
    Degrees  ///< Output in degrees
};

/// @brief Specifies the rotation direction of the quaternion
enum class RotationDirection
{
    BodyToNav, ///< Quaternion rotates from body frame to navigation frame
    NavToBody  ///< Quaternion rotates from navigation frame to body frame
};

// https://stackoverflow.com/a/11498248/9860973
float wrapPitch(float angle);

// https://stackoverflow.com/a/11498248/9860973
float wrapRoll(float angle);

// https://stackoverflow.com/a/11498248/9860973
float wrapYaw(float angle);

// https://stackoverflow.com/a/11498248/9860973
float wrapYaw(float angle);

/**
 * @brief Compute static body-to-NED attitude quaternion using a 2-vector TRIAD solution.
 *
 * This function estimates the sensor orientation assuming the sensor is
 * stationary (no rotation, no linear acceleration). The attitude is determined
 * by aligning:
 *   - the measured gravity direction (from accelerometer)
 *   - the measured Earth magnetic field direction (from magnetometer)
 * with their known reference directions in the NED frame.
 *
 * The method implements a closed-form 2-vector Wahba / TRIAD solution:
 *   1. Construct orthonormal triads in the NED frame using n_G and n_M.
 *   2. Construct orthonormal triads in the body frame using b_a_s and b_m_s.
 *   3. Compute the rotation matrix that aligns the body triad to the NED triad.
 *
 * @param b_a_s  Specific force measured by the accelerometer in the body frame
 *               (m/s²). When static, this vector is approximately −gravity.
 * @param b_m_s  Earth magnetic field measured by the magnetometer in the body
 *               frame (arbitrary units, calibrated).
 * @param n_M    Earth magnetic field vector expressed in the NED frame.
 *               Includes magnetic inclination and declination.
 * @param n_G    Gravity vector expressed in the NED frame (typically [0, 0, 1]).
 *
 * @return Quaternionf representing the rotation from body frame to NED frame.
 *         The quaternion satisfies:
 *           v_ned = q * v_body * q⁻¹
 *
 * @warning
 *   - This method is sensitive to noise and magnetic disturbances.
 *   - Behavior is undefined if gravity and magnetic field vectors become
 *     collinear or near-zero.
 */
Quaternionf statAttQuat(const Vector3f& b_a_s,
                        const Vector3f& b_m_s,
                        const Vector3f& n_M = Vector3f(1, 0, 0),
                        const Vector3f& n_G = Vector3f(0, 0, 1));

Vector3f quatToEuler(const Quaternionf&      q,
                           NavFrame          frame = NavFrame::NED,
                           RotationType      type  = RotationType::Intrinsic,
                           AngleUnit         angle = AngleUnit::Degrees,
                           RotationDirection dir   = RotationDirection::BodyToNav);

class CompFilt
{
public:
    CompFilt()
        : isInit(false),
          tau(0),
          n_G(0.0, 0.0, 9.81),
          n_M(1.0, 0.0, 0.0),
          timestamp(0.0),
          prevTimestamp(0.0),
          dt(0.0),
          q_bn(Quaternionf::Identity())
    {}

    Vector3f    get_b_R_ang_n();  // Euler angles that describes the intrinsic frame rotation from the NED frame to the vehicle's body frame
    Quaternionf get_b_R_quat_n(); // Quaternion   that describes the intrinsic frame rotation from the NED frame to the vehicle's body frame

    void updateMag(const Vector3f& _n_M);
    void updateGrav(const Vector3f& _n_G);
    void updateTau(float _tau);
    void update(const Vector3f& b_a_s,
                const Vector3f& b_g_s,
                const Vector3f& b_m_s,
                unsigned long   _timestamp);

    virtual ~CompFilt() = default;

protected:
    bool  isInit;
    float tau;

    Vector3f n_G; // Ideal gravity vector (m/s^2) in the NED frame
    Vector3f n_M; // Ideal magnetic field (nT) in the NED frame

    unsigned long timestamp;     // Current update timestamp (us)
    unsigned long prevTimestamp; // Previous update timestamp (us)
    float         dt;            // Difference between current and previous update timestamps (s)

    Quaternionf q_bn; // Quaternion that describes the intrinsic frame rotation from the NED frame to the vehicle's body frame
};
