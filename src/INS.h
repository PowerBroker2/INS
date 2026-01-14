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

/**
 * @brief Compute attitude quaternion from accelerometer and magnetometer vectors.
 *
 * This function estimates the body-to-navigation frame orientation using
 * measured accelerometer and magnetometer vectors in the body frame and
 * known gravity and magnetic field directions in the navigation frame.
 *
 * The algorithm constructs orthonormal bases in both frames:
 *  - The "Down" axis is defined by gravity (accelerometer / reference gravity)
 *  - The "North" axis is obtained by projecting the magnetic field onto the
 *    horizontal plane (orthogonal to gravity)
 *  - The "East" axis is formed by the cross product of Down and North
 *
 * The resulting rotation matrix is converted to a quaternion representing
 * the rotation from body frame to navigation frame.
 *
 * @param b_a_s Measured specific force (accelerometer) vector in body frame.
 *              Expected to point opposite to gravity.
 * @param b_m_s Measured magnetic field vector in body frame.
 * @param n_M   Reference magnetic field vector in navigation frame.
 * @param n_G   Reference gravity vector in navigation frame.
 *
 * @return Quaternionf representing the rotation from body frame to navigation frame.
 *
 * @note All input vectors must be non-zero and non-collinear with gravity.
 * @note Magnetometer vectors are assumed to be free of hard/soft iron distortion.
 * @note The resulting quaternion is normalized implicitly via orthonormal bases.
 */
Quaternionf statAttQuat(const Vector3f& b_a_s,
                        const Vector3f& b_m_s,
                        const Vector3f& n_M = Vector3f(1, 0, 0),
                        const Vector3f& n_G = Vector3f(0, 0, 1));

/**
 * @brief Convert a quaternion to Euler angles (roll, pitch, yaw).
 *
 * This function converts an orientation quaternion into Euler angles using
 * a ZYX rotation sequence (yaw–pitch–roll). It supports intrinsic and extrinsic
 * rotation conventions, navigation frame selection (NED or ENU), angle units,
 * and rotation direction.
 *
 * The quaternion is normalized for numerical safety. If the quaternion
 * represents a navigation-to-body rotation, it is inverted internally to
 * obtain a body-to-navigation rotation before conversion.
 *
 * A direction cosine matrix (DCM) is formed from the quaternion and optionally
 * transformed between NED and ENU navigation frames prior to Euler extraction.
 *
 * @param q     Input orientation quaternion.
 * @param frame Navigation frame convention:
 *              - NavFrame::NED (North-East-Down)
 *              - NavFrame::ENU (East-North-Up)
 * @param type  Rotation convention:
 *              - RotationType::Intrinsic (body-fixed rotations)
 *              - RotationType::Extrinsic (space-fixed rotations)
 * @param angle Output angle unit:
 *              - AngleUnit::Radians
 *              - AngleUnit::Degrees
 * @param dir   Direction of the rotation represented by the quaternion:
 *              - RotationDirection::BodyToNav
 *              - RotationDirection::NavToBody
 *
 * @return Vector3f containing Euler angles in the order:
 *         (roll, pitch, yaw).
 *
 * @note Euler angles follow a ZYX sequence (yaw–pitch–roll).
 * @note Gimbal lock occurs at ±90° pitch.
 * @note ENU/NED conversion assumes standard axis definitions.
 * @note The returned angles are in radians unless degrees are requested.
 */
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
