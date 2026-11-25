#pragma once

#include <godot_cpp/classes/rigid_body3d.hpp>
#include <godot_cpp/templates/vector.hpp>

namespace godot {

class Wheel;

class Vehicle : public RigidBody3D {
    GDCLASS(Vehicle, RigidBody3D)

private:
    Vector<Wheel*> wheels;

    float max_engine_torque = 2500.0f;
    float max_brake_force = 4000.0f;
    float max_steer_angle_deg = 32.0f;
    float steer_speed = 3.0f;

    float throttle_input = 0.0f;
    float brake_input = 0.0f;
    float steer_input = 0.0f;
    float current_steer_angle = 0.0f;

    float default_linear_damp  = 0.1f;
    float default_angular_damp = 0.5f;
    float stability_linear_damp_boost  = 2.0f;
    float stability_angular_damp_boost = 6.0f;

    float stopped_hold_timer  = 0.0f;
    bool  was_stopped_boosted = false;
    static constexpr float STOP_HOLD_TIME = 0.4f;

    void collect_wheels();

protected:
    static void _bind_methods();

public:
    Vehicle();
    ~Vehicle() override = default;

    void _ready() override;
    void _physics_process(double delta) override;

    bool is_nearly_stopped() const {
        return get_linear_velocity().length_squared()  < 0.09f &&
               get_angular_velocity().length_squared() < 0.09f;
    }

    int get_wheel_count() const { return wheels.size(); }

    void set_throttle_input(float p_value) { throttle_input = Math::clamp(p_value, -1.0f, 1.0f); }
    float get_throttle_input() const { return throttle_input; }

    void set_brake_input(float p_value) { brake_input = Math::clamp(p_value, 0.0f, 1.0f); }
    float get_brake_input() const { return brake_input; }

    void set_steer_input(float p_value) { steer_input = Math::clamp(p_value, -1.0f, 1.0f); }
    float get_steer_input() const { return steer_input; }

    void set_max_engine_torque(float p_torque) { max_engine_torque = p_torque; }
    float get_max_engine_torque() const { return max_engine_torque; }

    void set_steer_speed(float p_speed) { steer_speed = p_speed; }
    float get_steer_speed() const { return steer_speed; }

    void set_max_brake_force(float p_force) { max_brake_force = p_force; }
    float get_max_brake_force() const { return max_brake_force; }

    void set_max_steer_angle_deg(float p_deg) { max_steer_angle_deg = p_deg; }
    float get_max_steer_angle_deg() const { return max_steer_angle_deg; }
};

}