#include <vehicle/vehicle.hpp>
#include <vehicle/wheel.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

Vehicle::Vehicle() {
    set_angular_damp(0.5f);
    set_linear_damp(0.1f);
    set_can_sleep(false);

    // default values for dynamic boost
    default_linear_damp  = get_linear_damp();
    default_angular_damp = get_angular_damp();
}

void Vehicle::_ready() {
    // skip editor
    if (Engine::get_singleton()->is_editor_hint()) {
        set_process(false);
        set_physics_process(false);
        return;
    }
    
    set_can_sleep(false);
    collect_wheels();
}

void Vehicle::collect_wheels() {
    wheels.clear();
    int child_count = get_child_count();
    for (int i = 0; i < child_count; ++i) {
        Wheel* wheel = Object::cast_to<Wheel>(get_child(i));
        if (wheel) {
            wheel->set_vehicle(this);
            wheels.push_back(wheel);
        }
    }
}

void Vehicle::_physics_process(double delta) {
    if (wheels.is_empty()) return;

    // smooth steering
    float target_steer = Math::deg_to_rad(max_steer_angle_deg) * steer_input;
    current_steer_angle = Math::move_toward(current_steer_angle, target_steer, steer_speed * static_cast<float>(delta));

    // we calculate the number of driving wheels for torque distribution
    int driven_count = 0;
    for (int i = 0; i < wheels.size(); ++i) {
        if (wheels[i]->get_is_driven()) {
            driven_count++;
        }
    }

    float torque_per_wheel = (driven_count > 0) ? (max_engine_torque * throttle_input) / static_cast<float>(driven_count) : 0.0f;
    float brake_per_wheel = (max_brake_force * brake_input) / static_cast<float>(wheels.size());

    // we are updating the physics of each wheel
    for (int i = 0; i < wheels.size(); ++i) {
        Wheel* wheel = wheels[i];
        if (wheel->get_is_steerable()) {
            wheel->set_steering_angle(current_steer_angle);
        }
        wheel->process_wheel_physics(delta, torque_per_wheel, brake_per_wheel);
    }

    // stabilizing the body when stopped: damping residual rocking
    // using custom mods (pitch/roll/heave)
    if (is_nearly_stopped()) {
        stopped_hold_timer = STOP_HOLD_TIME;
    }

    bool apply_stability = (stopped_hold_timer > 0.0f);
    if (apply_stability) {
        stopped_hold_timer -= static_cast<float>(delta);
    }

    if (apply_stability && !was_stopped_boosted) {
        set_linear_damp(default_linear_damp  + stability_linear_damp_boost);
        set_angular_damp(default_angular_damp + stability_angular_damp_boost);
        was_stopped_boosted = true;
    }
    else if (!apply_stability && was_stopped_boosted) {
        set_linear_damp(default_linear_damp);
        set_angular_damp(default_angular_damp);
        was_stopped_boosted = false;
    }
}

void Vehicle::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_throttle_input", "value"), &Vehicle::set_throttle_input);
    ClassDB::bind_method(D_METHOD("get_throttle_input"), &Vehicle::get_throttle_input);

    ClassDB::bind_method(D_METHOD("set_brake_input", "value"), &Vehicle::set_brake_input);
    ClassDB::bind_method(D_METHOD("get_brake_input"), &Vehicle::get_brake_input);

    ClassDB::bind_method(D_METHOD("set_steer_input", "value"), &Vehicle::set_steer_input);
    ClassDB::bind_method(D_METHOD("get_steer_input"), &Vehicle::get_steer_input);

    ClassDB::bind_method(D_METHOD("set_steer_speed", "torque"), &Vehicle::set_steer_speed);
    ClassDB::bind_method(D_METHOD("get_steer_speed"), &Vehicle::get_steer_speed);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "steer_speed"), "set_steer_speed", "get_steer_speed");

    ClassDB::bind_method(D_METHOD("set_max_engine_torque", "torque"), &Vehicle::set_max_engine_torque);
    ClassDB::bind_method(D_METHOD("get_max_engine_torque"), &Vehicle::get_max_engine_torque);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_engine_torque"), "set_max_engine_torque", "get_max_engine_torque");

    ClassDB::bind_method(D_METHOD("set_max_brake_force", "force"), &Vehicle::set_max_brake_force);
    ClassDB::bind_method(D_METHOD("get_max_brake_force"), &Vehicle::get_max_brake_force);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_brake_force"), "set_max_brake_force", "get_max_brake_force");

    ClassDB::bind_method(D_METHOD("set_max_steer_angle_deg", "deg"), &Vehicle::set_max_steer_angle_deg);
    ClassDB::bind_method(D_METHOD("get_max_steer_angle_deg"), &Vehicle::get_max_steer_angle_deg);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_steer_angle_deg"), "set_max_steer_angle_deg", "get_max_steer_angle_deg");
}
