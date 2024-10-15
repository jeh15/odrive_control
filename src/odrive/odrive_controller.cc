#include "odrive_controller.h"

#include <iostream>
#include <cmath>


namespace base::odrive_controller {

    ODriveController::ODriveController(const ODriveSocket::Ptr socket, const uint8_t can_id)
        : Estop(), socket_(socket), can_id_(can_id){
    }

    ODriveController::~ODriveController() {
        this->setState(AxisState::IDLE);
    }

    bool ODriveController::setGearRatio(const double gear_ratio) {
        gear_ratio_ = gear_ratio;
        return true;
    }

    bool ODriveController::setState(const uint32_t state) {
        uint32_t axis_error = socket_->getAxisError(can_id_);
        if (axis_error != 0x0) {
            std::cerr << "Motor Error: " << axis_error << " (" << getErrorName() << ")" << std::endl;
            return false;
        }

        if (axis_state_ == state)
            return true;

        if (!socket_->setAxisState(can_id_, state)) {
            std::cerr << "Failed to set axis state." << std::endl;
            return false;
        }

        axis_state_ = state;
        return true;
    }

    bool ODriveController::setControlMode(const uint32_t control_mode, const uint32_t input_mode) {
        uint32_t axis_error = socket_->getAxisError(can_id_);
        if (axis_error != 0x0) {
            std::cerr << "Motor Error: " << axis_error << " (" << getErrorName() << ")" << std::endl;
            return false;
        }

        if (control_mode_ == control_mode && input_mode_ == input_mode)
            return true;

        if (!socket_->setControlMode(can_id_, control_mode, input_mode)) {
            std::cerr << "Failed to set control/input mode." << std::endl;
            return false;
        }

        control_mode_ = control_mode;
        input_mode_ = input_mode;
        return true;
    }

    bool ODriveController::setPosGain(const double pos_gain) {
        uint32_t axis_error = socket_->getAxisError(can_id_);
        if (axis_error != 0x0) {
            std::cerr << "Motor Error: " << axis_error << " (" << getErrorName() << ")" << std::endl;
            return false;
        }

        if (pos_gain_ == pos_gain)
            return true;

        if (!socket_->setPosGain(can_id_, pos_gain)) {
            std::cerr << "Failed to set position gain." << std::endl;
            return false;
        }

        pos_gain_ = pos_gain;
        return true;
    }

    bool ODriveController::setVelGains(const double vel_gain, const double integrator_gain) {
        uint32_t axis_error = socket_->getAxisError(can_id_);
        if (axis_error != 0x0) {
            std::cerr << "Motor Error: " << axis_error << " (" << getErrorName() << ")" << std::endl;
            return false;
        }

        if (vel_gain_ == vel_gain && integrator_gain_ == integrator_gain)
            return true;

        if (!socket_->setVelGains(can_id_, vel_gain, integrator_gain)) {
            std::cerr << "Failed to set velocity gains." << std::endl;
            return false;
        }

        vel_gain_ = vel_gain;
        integrator_gain_ = integrator_gain;
        return true;
    }

    bool ODriveController::setLimits(const double velocity_limit, const double current_limit) {
        uint32_t axis_error = socket_->getAxisError(can_id_);
        if (axis_error != 0x0) {
            std::cerr << "Motor Error: " << axis_error << " (" << getErrorName() << ")" << std::endl;
            return false;
        }

        if (velocity_limit_ == velocity_limit && current_limit_ == current_limit)
            return true;

        if (!socket_->setLimits(can_id_, velocity_limit, current_limit)) {
            std::cerr << "Failed to set limits." << std::endl;
            return false;
        }

        velocity_limit_ = velocity_limit;
        current_limit_ = current_limit;
        return true;
    }

    bool ODriveController::clearErrors() {
        return socket_->clearErrors(can_id_);
    }

    bool ODriveController::setPosition(const double pos, const double vel_ff, const double torque_ff) {
        uint32_t axis_error = socket_->getAxisError(can_id_);
        if (axis_error != 0x0) {
            std::cerr << "Motor Error: " << axis_error << " (" << getErrorName() << ")" << std::endl;
            return false;
        }

        // Convert from radians to revolutions and apply gear ratio
        const double pos_rev = gear_ratio_ * pos / (2.0f * M_PI);
        const double vel_ff_rev = gear_ratio_ * vel_ff / (2.0f * M_PI);
        const double torque_ff_N = torque_ff / gear_ratio_;

        return socket_->setPosition(can_id_, pos_rev, vel_ff_rev, torque_ff_N);
    }

    bool ODriveController::setVelocity(const double vel, const double torque_ff) {

        uint32_t axis_error = socket_->getAxisError(can_id_);
        if (axis_error != 0x0) {
            std::cerr << "Motor Error: " << axis_error << " (" << getErrorName() << ")" << std::endl;
            return false;
        }

        // Convert from radians to revolutions and apply gear ratio
        const double vel_rev = gear_ratio_ * vel / (2.0f * M_PI);
        const double torque_ff_N = torque_ff / gear_ratio_;

        return socket_->setVelocity(can_id_, vel_rev, torque_ff_N);
    }

    bool ODriveController::setTorque(const double torque) {

        uint32_t axis_error = socket_->getAxisError(can_id_);
        if (axis_error != 0x0) {
            std::cerr << "Motor Error: " << axis_error << " (" << getErrorName() << ")" << std::endl;
            return false;
        }

        // Apply gear ratio
        const double torque_N = torque / gear_ratio_;

        return socket_->setTorque(can_id_, torque_N);
    }

    uint8_t ODriveController::getCANID() const {
        return can_id_;
    }

    uint32_t ODriveController::getAxisError() {
        return socket_->getAxisError(can_id_);
    }

    uint8_t ODriveController::getAxisState() {

        return socket_->getAxisState(can_id_);
    }

    double ODriveController::getIqSetpoint() {

        return socket_->getIqSetpoint(can_id_);
    }

    double ODriveController::getIqMeasured() {

        return socket_->getIqMeasured(can_id_);
    }

    double ODriveController::getFETTemperature() {

        return socket_->getFETTemperature(can_id_);
    }

    double ODriveController::getMotorTemperature() {

        return socket_->getMotorTemperature(can_id_);
    }

    double ODriveController::getBusVoltage() {

        return socket_->getBusVoltage(can_id_);
    }

    double ODriveController::getBusCurrent() {

        return socket_->getBusCurrent(can_id_);
    }

    double ODriveController::getPositionEstimate() {
        // Convert from revolutions to radians and apply gear ratio
        return socket_->getPosEstimate(can_id_) * (2.0f * M_PI) / gear_ratio_;
    }

    double ODriveController::getVelocityEstimate() {
        // Convert from revolutions to radians and apply gear ratio
        return socket_->getVelEstimate(can_id_) * (2.0f * M_PI) / gear_ratio_;
    }

    double ODriveController::getTorqueEstimate() {
        // Apply gear ratio
        return socket_->getTorqueEstimate(can_id_) * gear_ratio_;
    }

    void ODriveController::printInfo() {
        std::cout << "  CAN ID: " << (int)getCANID() << std::endl;
        std::cout << "  Axis error: " << getAxisError() << " (" << getErrorName() << ")" << std::endl;
        std::cout << "  Axis state: " << (int)getAxisState() << std::endl;
        std::cout << "  Iq setpoint: " << getIqSetpoint() << std::endl;
        std::cout << "  Iq measured: " << getIqMeasured() << std::endl;
        std::cout << "  FET temperature: " << getFETTemperature() << std::endl;
        std::cout << "  Motor temperature: " << getMotorTemperature() << std::endl;
        std::cout << "  Bus voltage: " << getBusVoltage() << std::endl;
        std::cout << "  Bus current: " << getBusCurrent() << std::endl;
        std::cout << "  Position estimate: " << getPositionEstimate() << std::endl;
        std::cout << "  Velocity estimate: " << getVelocityEstimate() << std::endl;
        std::cout << "  Torque estimate: " << getTorqueEstimate() << std::endl;
        std::cout << std::endl;
    }

    std::string ODriveController::getErrorName() {
        // Based on ODrive v0.6.8
        switch (getAxisError()) {
        case 0x0:
            return "NONE";
        case 0x1:
            return "INITIALIZING";
        case 0x2:
            return "TIMING_ERROR";
        case 0x4:
            return "BRAKE_RESISTOR_DISARMED";
        case 0x8:
            return "MISSING_ESTIMATE";
        case 0x10:
            return "BAD_CONFIG";
        case 0x20:
            return "DRV_FAULT";
        case 0x40:
            return "MISSING_INPUT";
        case 0x100:
            return "DC_BUS_OVER_VOLTAGE";
        case 0x200:
            return "DC_BUS_UNDER_VOLTAGE";
        case 0x400:
            return "DC_BUS_OVER_CURRENT";
        case 0x800:
            return "DC_BUS_OVER_REGEN_CURRENT";
        case 0x1000:
            return "CURRENT_LIMIT_VIOLATION";
        case 0x2000:
            return "MOTOR_OVER_TEMP";
        case 0x4000:
            return "INVERTER_OVER_TEMP";
        case 0x8000:
            return "VELOCITY_LIMIT_VIOLATION";
        case 0x10000:
            return "POSITION_LIMIT_VIOLATION";
        case 0x1000000:
            return "WATCHDOG_TIMER_EXPIRED";
        case 0x2000000:
            return "ESTOP_REQUESTED";
        case 0x4000000:
            return "SPINOUT_DETECTED";
        case 0x8000000:
            return "BRAKE_RESISTOR_DISARMED";
        case 0x10000000:
            return "THERMISTOR_DISCONNECTED";
        case 0x40000000:
            return "CALIBRATION_ERROR";
        default:
            return "MULTIPLE_ERRORS";
        };
    }

    void ODriveController::estop(int sig) {
        this->setState(AxisState::IDLE);
        std::cout << "Stopped motor " << (int)getCANID() << " (SIG " << sig << ")" << std::endl;
    }
}
