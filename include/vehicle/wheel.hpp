#pragma once

#include <godot_cpp/classes/shape_cast3d.hpp>
#include <godot_cpp/classes/rigid_body3d.hpp>

namespace godot {

class Vehicle;

class Wheel : public ShapeCast3D {
    GDCLASS(Wheel, ShapeCast3D)

private:
    int wheel_segments = 16;
    float wheel_radius = 0.5f;
    float wheel_width = 0.35f;
    float suspension_length = 0.4f;
    float shapecast_offset = 0.25f;
    float max_slope_angle_deg = 90.0f;

    float damper_stiffness = 3700.0f;
    float spring_stiffness = 35000.0f;
    float spring_stiffness_progression = 0.5f;
    float rebound_damper_multiplier = 1.8f;

    float lateral_grip = 1.2f;
    float forward_grip = 1.80f;
    float rolling_resistance = 0.025f;

    float camber_droop_deg = 2.5f;
    float camber_compressed_deg = -4.0f;

    float prev_compression = 0.0f;
    float current_compression = 0.0f;
    float filtered_comp_vel = 0.0f;
    float steering_angle = 0.0f;
    
    bool is_driven = true;
    bool is_steerable = false;
    bool is_left_wheel = true;
    bool was_colliding = false;

    float pacejka_B = 10.5f;
    float pacejka_C = 1.45f;
    float pacejka_E = 0.2f;

    Vehicle* vehicle = nullptr;
    Node3D* wheel_mesh = nullptr;
    Basis wheel_mesh_basis = {};
    double spin_angle = 0.0;
    float wheel_angular_velocity = 0.0f;

    void _update_shape_geometry();
    bool _raycast_height(Vector3 &out_ground_point, Vector3 &out_ground_normal);
    float _pacejka(float slip, float b, float c, float d, float e);

protected:
    static void _bind_methods();

public:
    Wheel();
    ~Wheel() override = default;

    void _ready() override;
    void process_wheel_physics(double delta, float drive_torque, float brake_force);
    void process_wheel_visual(float d, float target_y, float target_ang_vel, bool instant_y);

public:
    void set_vehicle(Vehicle* p_vehicle) { vehicle = p_vehicle; }
    void set_steering_angle(float p_angle_rad);

    void set_wheel_segments(int value);
    void set_wheel_radius(float p_radius);
    void set_wheel_width(float p_width);
    void set_suspension_length(float p_length);

    int get_wheel_segments() const { return wheel_segments; }
    float get_wheel_radius() const { return wheel_radius; }
    float get_wheel_width() const { return wheel_width; }
    float get_suspension_length() const { return suspension_length; }

    void set_spring_stiffness(float p) { spring_stiffness = p; }
    float get_spring_stiffness() const { return spring_stiffness; }

    void set_damper_stiffness(float p) { damper_stiffness = p; }
    float get_damper_stiffness() const { return damper_stiffness; }

    void set_damper_rebound(float p) { rebound_damper_multiplier = p; }
    float get_damper_rebound() const { return rebound_damper_multiplier; }

    void set_lateral_grip(float p) { lateral_grip = p; }
    float get_lateral_grip() const { return lateral_grip; }

    void set_is_steerable(bool p) { is_steerable = p; }
    bool get_is_steerable() const { return is_steerable; }

    void set_is_driven(bool p) { is_driven = p; }
    bool get_is_driven() const { return is_driven; }

    float get_current_compression() const { return current_compression; }

    void set_camber_droop_deg(float value) { camber_droop_deg = value; }
    float get_camber_droop_deg() const { return camber_droop_deg; }

    void set_camber_compressed_deg(float value) { camber_compressed_deg = value; }
    float get_camber_compressed_deg() const { return camber_compressed_deg; }

    void set_wheel_mesh(Node3D* p_mesh) { wheel_mesh = p_mesh; }
    Node3D* get_wheel_mesh() const { return wheel_mesh; }
};

}