/**
 * @file      State.cpp
 * @brief     Implementation of LiDAR-Inertial state representation.
 * @author    Seungwon Choi
 * @email     csw3575@snu.ac.kr
 * @date      2025-11-18
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#include "State.h"
#include "LieUtils.h"
#include <spdlog/spdlog.h>

namespace lio {

const float kGravityNorm = 9.81f;  // m/s²
const float kInitCov = 0.01f;

// Constructor
State::State() {
    Reset();
}

// Copy constructor
State::State(const State& other)
    : m_rotation(other.m_rotation)
    , m_position(other.m_position)
    , m_velocity(other.m_velocity)
    , m_gyro_bias(other.m_gyro_bias)
    , m_acc_bias(other.m_acc_bias)
    , m_gravity(other.m_gravity)
    , m_covariance(other.m_covariance)
{}

void State::Reset() {
    m_rotation = Eigen::Matrix3f::Identity();
    m_position = Eigen::Vector3f::Zero();
    m_velocity = Eigen::Vector3f::Zero();
    m_gyro_bias = Eigen::Vector3f::Zero();
    m_acc_bias = Eigen::Vector3f::Zero();
    m_gravity = Eigen::Vector3f(0.0f, 0.0f, -kGravityNorm);
    
    // Initialize covariance
    m_covariance = Eigen::Matrix<float, 18, 18>::Identity() * kInitCov;
    
    // Smaller initial uncertainty for rotation
    m_covariance.block<3,3>(m_rot_idx, m_rot_idx) = 
        Eigen::Matrix3f::Identity() * 0.00001f;
    
    // Smaller initial uncertainty for biases and gravity
    m_covariance.block<9,9>(m_gyr_bias_idx, m_gyr_bias_idx) = 
        Eigen::Matrix<float, 9, 9>::Identity() * 0.00001f;
}

State State::operator+(const Eigen::Matrix<float, 18, 1>& delta) const {
    State result = *this;
    
    // Rotation update: R_new = R * Exp(delta_R)
    Eigen::Vector3f delta_rot = delta.segment<3>(m_rot_idx);
    SO3 delta_R = SO3::Exp(delta_rot);
    result.m_rotation = (SO3(m_rotation) * delta_R).Matrix();
    
    // Other states: simple addition
    result.m_position += delta.segment<3>(m_pos_idx);
    result.m_velocity += delta.segment<3>(m_vel_idx);
    result.m_gyro_bias += delta.segment<3>(m_gyr_bias_idx);
    result.m_acc_bias += delta.segment<3>(m_acc_bias_idx);
    result.m_gravity += delta.segment<3>(m_grav_idx);
    
    return result;
}

State& State::operator=(const State& other) {
    if (this != &other) {
        m_rotation = other.m_rotation;
        m_position = other.m_position;
        m_velocity = other.m_velocity;
        m_gyro_bias = other.m_gyro_bias;
        m_acc_bias = other.m_acc_bias;
        m_gravity = other.m_gravity;
        m_covariance = other.m_covariance;
    }
    return *this;
}

Eigen::Matrix<float, 18, 1> State::operator-(const State& other) const {
    Eigen::Matrix<float, 18, 1> delta;
    
    // Rotation difference: log(R_other^T * R_this)
    Eigen::Matrix3f rot_diff = other.m_rotation.transpose() * m_rotation;
    SO3 rot_so3(rot_diff);
    Eigen::Vector3f log_rot = rot_so3.Log();  // Member function, not static
    delta.segment<3>(m_rot_idx) = log_rot;
    
    // Other states: simple subtraction
    delta.segment<3>(m_pos_idx) = m_position - other.m_position;
    delta.segment<3>(m_vel_idx) = m_velocity - other.m_velocity;
    delta.segment<3>(m_gyr_bias_idx) = m_gyro_bias - other.m_gyro_bias;
    delta.segment<3>(m_acc_bias_idx) = m_acc_bias - other.m_acc_bias;
    delta.segment<3>(m_grav_idx) = m_gravity - other.m_gravity;
    
    return delta;
}

State& State::operator+=(const Eigen::Matrix<float, 18, 1>& delta) {
    *this = *this + delta;
    return *this;
}

SE3 State::GetPose() const {
    return SE3(SO3(m_rotation), m_position);
}

void State::SetPose(const SE3& pose) {
    m_rotation = pose.RotationMatrix();
    m_position = pose.Translation();
}

Eigen::Matrix<float, 18, 1> State::ToVector() const {
    Eigen::Matrix<float, 18, 1> vec;
    
    // Rotation as log(R)
    SO3 rot_so3(m_rotation);
    Eigen::Vector3f log_rot = rot_so3.Log();  // Member function, not static
    vec.segment<3>(m_rot_idx) = log_rot;
    
    // Other states
    vec.segment<3>(m_pos_idx) = m_position;
    vec.segment<3>(m_vel_idx) = m_velocity;
    vec.segment<3>(m_gyr_bias_idx) = m_gyro_bias;
    vec.segment<3>(m_acc_bias_idx) = m_acc_bias;
    vec.segment<3>(m_grav_idx) = m_gravity;
    
    return vec;
}

void State::FromVector(const Eigen::Matrix<float, 18, 1>& state_vector) {
    // Rotation from log(R)
    Eigen::Vector3f log_rot = state_vector.segment<3>(m_rot_idx);
    SO3 rot_so3 = SO3::Exp(log_rot);
    m_rotation = rot_so3.Matrix();
    
    // Other states
    m_position = state_vector.segment<3>(m_pos_idx);
    m_velocity = state_vector.segment<3>(m_vel_idx);
    m_gyro_bias = state_vector.segment<3>(m_gyr_bias_idx);
    m_acc_bias = state_vector.segment<3>(m_acc_bias_idx);
    m_gravity = state_vector.segment<3>(m_grav_idx);
}

void State::Print() const {
    spdlog::info("========== LIO State ==========");
    spdlog::info("Position: [{:.4f}, {:.4f}, {:.4f}]", 
                 m_position.x(), m_position.y(), m_position.z());
    spdlog::info("Velocity: [{:.4f}, {:.4f}, {:.4f}]", 
                 m_velocity.x(), m_velocity.y(), m_velocity.z());
    spdlog::info("Gyro Bias: [{:.6f}, {:.6f}, {:.6f}]", 
                 m_gyro_bias.x(), m_gyro_bias.y(), m_gyro_bias.z());
    spdlog::info("Acc Bias: [{:.6f}, {:.6f}, {:.6f}]", 
                 m_acc_bias.x(), m_acc_bias.y(), m_acc_bias.z());
    spdlog::info("Gravity: [{:.4f}, {:.4f}, {:.4f}]", 
                 m_gravity.x(), m_gravity.y(), m_gravity.z());
    
    Eigen::Vector3f euler = m_rotation.eulerAngles(0, 1, 2) * 180.0f / M_PI;
    spdlog::info("Rotation (RPY deg): [{:.2f}, {:.2f}, {:.2f}]", 
                 euler.x(), euler.y(), euler.z());
    spdlog::info("===============================");
}

} // namespace lio
