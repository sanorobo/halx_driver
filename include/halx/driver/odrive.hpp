#ifndef SRC_ODRIVE_H_
#define SRC_ODRIVE_H_

// this is for Odrive3.6 and it's derivative board
// some difference
// CAN cmd id (odrive pro)
// - 0x003 : get_error
// - 0x004 : rx_sdo
// - 0x005 : tx_sdo
// - 0x008, 0x00A, 0x01D : none
// - 0x015 : get temperature
// - 0x019 : set absolute position
// - 0x01C : get torque
// - 0x01F : enter dfu mode
// - reserved for CANOpen message?
//   - 0x000 : get version
//   - 0x700 : none

#include <halx/core.hpp>
#include <halx/peripheral/can.hpp>

namespace halx::driver
{

class Odrive
{
public:
    enum class Mode : uint8_t {
        DISABLE = 0,    // axis_state_t::IDLE
        TORQUE = 1,     // TORQUE_CONTROL, PASSTHROUGH
        VELOCITY = 2,   // VELOCITY_CONTROL, PASSTHROUGH
        POSITION = 3    // POSITION_CONTROL, TRAP_TRAJ
    };

    enum class ControlMode : uint32_t {
        VOLTAGE_CONTROL,
        TORQUE_CONTROL,
        VELOCITY_CONTROL,
        POSITION_CONTROL
    };

    enum class InputMode : uint32_t {
        INACTIVE = 0x00,    //入力無効
        PASSTHROUGH, //直接制御 : 有効な入力：input_pos, input_vel 有効なコントロールモード：すべて
        VEL_RAMP, //傾斜速度制御  : 有効な入力：input_vel 有効なコントロールモード CONTROL_MODE_VELOCITY_CONTROL
        POS_FILTER,   // 2次位置フィルタ位置制御 : 有効な入力:input_pos
                    // 有効なコントロールモード:CONTROL_MODE_POSITION_CONTROL
        MIX_CHANNELS, //未実装
        TRAP_TRAJ,    //台形軌道位置制御
        TORQUE_RAMP,  //トルクの傾斜制御
        MIRROR       //他の軸のミラーをする
    };

    enum class cmd_t : uint32_t{
        reserved_nmt = 0x000,
        s2m_heart_beat = 0x001,
        m2s_estop = 0x002,
        s2m_get_motor_error = 0x003,
        s2m_encoder_error = 0x004,
        s2m_sensorless_error = 0x005,
        m2s_set_axis_node_id = 0x006,
        m2s_set_axis_state = 0x007,
        m2s_set_axis_startup = 0x008,
        s2m_get_encoder_estimate = 0x009,
        s2m_get_encoder_count = 0x00A,
        m2s_controller_modes = 0x00B,
        m2s_set_input_pos = 0x00C,
        m2s_set_input_vel = 0x00D,
        m2s_set_input_torque = 0x00E,
        m2s_set_limits = 0x00F,
        m2s_set_traj_vel_limit = 0x011,
        m2s_set_traj_accel_limit = 0x012,
        m2s_set_traj_inertia = 0x013,
        s2m_get_iq = 0x014,
        s2m_get_sensorless_estimate = 0x015,
        m2s_reboot = 0x016,
        s2m_get_bus_voltage_current = 0x017,
        m2s_clear_errors = 0x018,
        m2s_set_linear_count = 0x019,
        m2s_set_pos_gain = 0x01A,
        m2s_set_vel_gains = 0x01B,
        s2m_get_adc_voltage = 0x01C,
        s2m_get_controller_error = 0x01D,
        reserved_heart_beat = 0x700
    };

    enum class AxisError : uint32_t {
        INVALID_STATE = 0x1,
        MOTOR_FAILED = 0x40,
        SENSORLESS_ESTIMATOR_FAILED = 0x80,
        ENCODER_FAILED = 0x100,
        CONTROLLER_FAILED = 0x200,
        WATCHDOG_TIMER_EXPIRED = 0x800,
        MIN_ENDSTOP_PRESSED = 0x1000,
        MAX_ENDSTOP_PRESSED = 0x2000,
        ESTOP_REQUESTED = 0x4000,
        HOMING_WITHOUT_ENDSTOP = 0x20000,
        OVER_TEMP = 0x40000,
        UNKNOWN_POSITION = 0x80000,
    };

    enum class AxisState : uint8_t {
        UNDEFINED = 0x0,
        IDLE = 0x1,
        STARTUP_SEQUENCE = 0x2,
        FULL_CALIBRATION_SEQUENCE = 0x3,
        MOTOR_CALIBRATION = 0x4,
        ENCODER_INDEX_SEARCH = 0x6,
        ENCODER_OFFSET_CALIBRATION = 0x7,
        CLOSED_LOOP_CONTROL = 0x8,
        LOCKIN_SPIN = 0x9,
        ENCODER_DIR_FIND = 0xA,
        HOMING = 0xB,
        ENCODER_HALL_POLARITY_CALIBRATION = 0xC,
        ENCODER_HALL_PHASE_CALIBRATION = 0xD
    };

private:
    struct Params {
        // heartbeat msg
        uint32_t axis_error;
        AxisState axis_state;
        bool motor_error_flag;
        bool encoder_error_flag;
        bool controller_error_flag;
        bool trajectory_done_flag;
        // get_encoder_estimate msg
        float pos_estimate;
        float vel_estimate;
        std::optional<uint32_t> last_update_heartbeat_;
        std::optional<uint32_t> last_update_encoder_estimate_;
    };
    struct InputPos {
        float pos_ref;
        float vel_ref;
        float torque_ref;
    };

    peripheral::CanBase &can_;
    uint8_t node_id_;
    core::RingBuffer<peripheral::CanMessage> rx_queue_{64};
    size_t filter_index_;
    Params params_;
    Mode mode_;
    InputPos input_;
    uint32_t timeout_;

public:
    Odrive(peripheral::CanBase &can, uint8_t node_id, Mode mode = Mode::DISABLE, uint32_t timeout = 1000) : can_(can), node_id_(node_id), mode_(mode), timeout_(timeout)
    {
        auto filter_index = can_.attach_rx_queue({(uint32_t)(node_id_ << 5), (uint32_t)(0x3F << 5), false}, rx_queue_);
        // auto filter_index = can_.attach_rx_queue({0, 0, false}, rx_queue_);
        if (!filter_index) {
        std::terminate();
        }
        filter_index_ = *filter_index;
    }

    ~Odrive() {
        can_.detach_rx_filter(filter_index_);
    }

    void update()
    {
        while (auto msg = rx_queue_.pop()) {
            if(msg->id >> 5 != node_id_) continue;
            switch((cmd_t)(msg->id & 0b00011111))
            {
            case cmd_t::s2m_heart_beat:
                if(msg->dlc == 8)
                {
                    params_.axis_error = (uint32_t)(msg->data[0] | (msg->data[1] << 8) | (msg->data[2] << 16) | (msg->data[3] << 24));
                    params_.axis_state = (AxisState)msg->data[4];
                    params_.motor_error_flag = msg->data[5] & 0b00000001;
                    params_.encoder_error_flag = msg->data[6] & 0b00000001;
                    params_.controller_error_flag = msg->data[7] & 0b00000001;
                    params_.trajectory_done_flag = msg->data[7] & 0b10000000;
                    params_.last_update_heartbeat_ = core::get_tick();

                    if(params_.axis_error == 0
                        && params_.motor_error_flag == false
                        && params_.encoder_error_flag == false
                        && params_.controller_error_flag == false)
                    {
                        // All systems nominal
                        if(mode_ != Mode::DISABLE && params_.axis_state != AxisState::CLOSED_LOOP_CONTROL)
                        {
                            // if axis_state is not CLOSED_LOOP_CONTROL, make it CLOSED_LOOP_CONTROL
                            peripheral::CanMessage msg{};
                            msg.id = (node_id_ << 5) | (uint32_t)cmd_t::m2s_set_axis_state;
                            msg.ide = false;
                            msg.dlc = 4;
                            uint32_t tx_data = (uint32_t)AxisState::CLOSED_LOOP_CONTROL;
                            std::memcpy(&msg.data[0], &tx_data, sizeof(uint32_t));
                            can_.transmit(msg, 5);
                        }
                        if(mode_ == Mode::DISABLE && params_.axis_state != AxisState::IDLE)
                        {
                            // if axis_state is not IDLE, make it IDLE
                            peripheral::CanMessage msg{};
                            msg.id = (node_id_ << 5) | (uint32_t)cmd_t::m2s_set_axis_state;
                            msg.ide = false;
                            msg.dlc = 4;
                            uint32_t tx_data = (uint32_t)AxisState::IDLE;
                            std::memcpy(&msg.data[0], &tx_data, sizeof(uint32_t));
                            can_.transmit(msg, 5);
                        }
                    }
                }
                break;
            case cmd_t::s2m_get_encoder_estimate:
                if(msg->dlc == 8)
                {
                    std::memcpy(&params_.pos_estimate, &msg->data[0], sizeof(float));
                    std::memcpy(&params_.vel_estimate, &msg->data[4], sizeof(float));
                    params_.last_update_encoder_estimate_ = core::get_tick();
                }
                break;
            default:
                break;
            }
        }
    }

    bool connected() {
        if (params_.last_update_heartbeat_){
            if (static_cast<uint32_t>(core::get_tick() - *params_.last_update_heartbeat_) >= timeout_) {
                return false;
            }
        } else {
            return false;
        }
        return true;
    }

    std::optional<float> get_pos_estimate() {
        if (params_.last_update_encoder_estimate_){
            if (static_cast<uint32_t>(core::get_tick() - *params_.last_update_encoder_estimate_) >= timeout_) {
                return std::nullopt;
            }
        } else {
            return std::nullopt;
        }
        return params_.pos_estimate;
    }

    std::optional<float> get_vel_estimate() {
        if (params_.last_update_encoder_estimate_){
            if (static_cast<uint32_t>(core::get_tick() - *params_.last_update_encoder_estimate_) >= timeout_) {
                return std::nullopt;
            }
        } else {
            return std::nullopt;
        }
        return params_.vel_estimate;
    }

    bool get_trajectory_done() {
        if (params_.last_update_heartbeat_){
            if (static_cast<uint32_t>(core::get_tick() - *params_.last_update_heartbeat_) >= timeout_) {
                return false;
            }
        } else {
            return false;
        }
        return params_.trajectory_done_flag;
    }

    bool set_mode(Mode mode) {
        mode_ = mode;
        if(!connected()) return false;
        ControlMode control_mode;
        InputMode input_mode;
        switch (mode_)
        {
        case Mode::TORQUE:
            control_mode = ControlMode::TORQUE_CONTROL;
            input_mode = InputMode::PASSTHROUGH;
            break;
        case Mode::VELOCITY:
            control_mode = ControlMode::VELOCITY_CONTROL;
            input_mode = InputMode::PASSTHROUGH;
            break;
        case Mode::POSITION:
            control_mode = ControlMode::POSITION_CONTROL;
            input_mode = InputMode::TRAP_TRAJ;
            break;
        default:
        case Mode::DISABLE:
            control_mode = ControlMode::TORQUE_CONTROL;
            input_mode = InputMode::INACTIVE;
            break;
        }
        peripheral::CanMessage msg{};
        msg.id = (node_id_ << 5) | (uint32_t)cmd_t::m2s_controller_modes;
        msg.ide = false;
        msg.dlc = 8;
        std::memcpy(&msg.data[0], &control_mode, sizeof(ControlMode));
        std::memcpy(&msg.data[4], &input_mode, sizeof(InputMode));
        return can_.transmit(msg, 5);
    }

    bool set_input_pos(InputPos input) {
        peripheral::CanMessage msg{};
        msg.id = (node_id_ << 5) | (uint32_t)cmd_t::m2s_set_input_pos;
        msg.ide = false;
        msg.dlc = 8;
        int16_t vel_raw = (int16_t)(input.vel_ref * 1000.0f);
        int16_t torque_raw = (int16_t)(input.torque_ref * 1000.0f);
        std::memcpy(&msg.data[0], &input.pos_ref, sizeof(float));
        std::memcpy(&msg.data[4], &vel_raw, sizeof(int16_t));
        std::memcpy(&msg.data[6], &torque_raw, sizeof(int16_t));
        return can_.transmit(msg, 5);
    }

    bool set_limits(float vel_limit, float current_limit) {
        peripheral::CanMessage msg{};
        msg.id = (node_id_ << 5) | (uint32_t)cmd_t::m2s_set_limits;
        msg.ide = false;
        msg.dlc = 8;
        std::memcpy(&msg.data[0], &vel_limit, sizeof(float));
        std::memcpy(&msg.data[4], &current_limit, sizeof(float));
        return can_.transmit(msg, 5);
    }
};

}

#endif /* SRC_ODRIVE_H_ */
