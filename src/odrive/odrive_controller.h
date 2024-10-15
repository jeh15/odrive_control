#pragma once

#include "odrive_socket.h"
#include "motor_controller.h"
#include "estop.h"

namespace base::odriveodrive_controller {

    /// @brief Implementation of MotorController for ODrive controllers
    class ODriveController : public starq::MotorController, private starq::Estop {
    public:
        using Ptr = std::shared_ptr<ODriveController>;

        /// @brief Connect to ODrive via CAN
        /// @param socket The ODrive socket.
        /// @param can_id The CAN ID of the ODrive.
        ODriveController(const ODriveSocket::Ptr socket, const uint8_t can_id);

        /// @brief Disconnect from ODrive.
        ~ODriveController();

        /// @brief Set the gear ratio
        /// @param gear_ratio The gear ratio of the motor.
        /// @return If the gear ratio was set successfully.
        bool setGearRatio(const double gear_ratio) override;

        /// @brief Set the axis state.
        /// @param state The state to set the ODrive to.
        /// @return If the command was sent successfully.
        bool setState(const uint32_t state) override;

        /// @brief Set the control mode.
        /// @param control_mode The control mode to set the ODrive to.
        /// @param input_mode The input mode to set the ODrive to. (default: 0x1)
        /// @return If the command was sent successfully.
        bool setControlMode(const uint32_t control_mode, const uint32_t input_mode = 0x1) override;

        /// @brief Set the position.
        /// @param pos The position to set the ODrive to. [rad]
        /// @param vel_ff The velocity feedforward to set the ODrive to. [rad/s] (default: 0)
        /// @param torque_ff The torque feedforward to set the ODrive to. [N-m] (default: 0)
        /// @return If the command was sent successfully.
        bool setPosition(const double pos, const double vel_ff = 0.F, const double torque_ff = 0.F) override;

        /// @brief Set the velocity.
        /// @param vel The velocity to set the ODrive to. [rad/s]
        /// @param torque_ff The torque feedforward to set the ODrive to. [N-m] (default: 0)
        /// @return If the command was sent successfully.
        bool setVelocity(const double vel, const double torque_ff = 0.F) override;

        /// @brief Set the torque.
        /// @param torque The torque to set the ODrive to. [N-m]
        /// @return If the command was sent successfully.
        bool setTorque(const double torque) override;

        /// @brief Set the position gain.
        /// @param pos_gain The position gain to set the ODrive to.
        /// @return If the command was sent successfully.
        bool setPosGain(const double pos_gain);

        /// @brief Set the velocity gain.
        /// @param vel_gain The velocity gain to set the ODrive to.
        /// @param vel_integrator_gain The velocity integrator gain to set the ODrive to.
        /// @return If the command was sent successfully.
        bool setVelGains(const double vel_gain, const double vel_integrator_gain);

        /// @brief Set the velocity limit..
        /// @param velocity_limit The velocity limit to set the ODrive to.
        /// @param current_limit The current limit to set the ODrive to.
        /// @return If the command was sent successfully.
        bool setLimits(const double velocity_limit, const double current_limit);

        /// @brief Clear errors.
        /// @return If the command was sent successfully.
        bool clearErrors();

        /// @brief Get the CAN ID.
        /// @return The CAN ID.
        uint8_t getCANID() const;

        /// @brief Get the encoder position estimate.
        /// @return The encoder position estimate in radians.
        double getPositionEstimate() override;

        /// @brief Get the encoder velocity estimate.
        /// @return The encoder velocity estimate in radians per second.
        double getVelocityEstimate() override;

        /// @brief Get the controller torque estimate.
        /// @return The controller torque estimate in Newton-meters.
        double getTorqueEstimate() override;

        /// @brief Get the axis error.
        /// @return The axis error.
        uint32_t getAxisError();

        /// @brief Get the axis state.
        /// @return The axis state.
        uint8_t getAxisState();

        /// @brief Get the Iq setpoint.
        /// @return The Iq setpoint in Amps.
        double getIqSetpoint();

        /// @brief Get the Iq measured.
        /// @return The Iq measured in Amps.
        double getIqMeasured();

        /// @brief Get the FET temperature.
        /// @return The FET temperature in degrees Celsius.
        double getFETTemperature();

        /// @brief Get the motor temperature.
        /// @return The motor temperature in degrees Celsius.
        double getMotorTemperature();

        /// @brief Get the DC bus voltage.
        /// @return The DC bus voltage in Volts.
        double getBusVoltage();

        /// @brief Get the DC bus current.
        /// @return The DC bus current in Amps.
        double getBusCurrent();

        /// @brief Print the ODrive info.
        void printInfo();

        /// @brief Get the error name.
        /// @return The error name.
        std::string getErrorName();

    private:
        /// @brief E-stop callback
        void estop(int sig) override;

        const ODriveSocket::Ptr socket_;
        const uint8_t can_id_;

        double gear_ratio_ = 1.0f;
        uint32_t axis_state_ = 0;
        uint32_t control_mode_ = 0;
        uint32_t input_mode_ = 0;
        double pos_gain_ = 0.0f;
        double vel_gain_ = 0.0f;
        double integrator_gain_ = 0.0f;
        double velocity_limit_ = 0.0f;
        double current_limit_ = 0.0f;
    };

}
