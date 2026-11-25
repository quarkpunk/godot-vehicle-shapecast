#include <vehicle/wheel.hpp>
#include <vehicle/vehicle.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/convex_polygon_shape3d.hpp>
#include <godot_cpp/classes/physics_direct_space_state3d.hpp>
#include <godot_cpp/classes/physics_ray_query_parameters3d.hpp>
#include <godot_cpp/classes/world3d.hpp>

using namespace godot;

Wheel::Wheel() {
    set_enabled(false);
    set_max_results(8);
}

void Wheel::_ready() {
    // skip editor
    if (Engine::get_singleton()->is_editor_hint()) {
        set_process(false);
        set_physics_process(false);
        return;
    }

    is_left_wheel = (get_position().x < 0.0f);
    if (wheel_mesh) {
        wheel_mesh_basis = wheel_mesh->get_transform().basis;
    }

    // build wheel geometry
    update_shape_geometry();

    if (vehicle) {
        add_exception(vehicle);
    }
}

void Wheel::set_wheel_segments(int value) {
    wheel_segments = Math::max(3, value);
    update_shape_geometry();
}

void Wheel::set_wheel_radius(float p_radius) {
    wheel_radius = Math::max(0.05f, p_radius);
    update_shape_geometry();
}

void Wheel::set_wheel_width(float p_width) {
    wheel_width = Math::max(0.05f, p_width);
    update_shape_geometry();
}

void Wheel::set_suspension_length(float p_length) {
    suspension_length = Math::max(0.05f, p_length);
    set_target_position(Vector3(0.0f, -(suspension_length + shapecast_offset), 0.0f));
}

void Wheel::set_steering_angle(float p_angle_rad) {
    if (!is_steerable) return;

    steering_angle = p_angle_rad;
    Vector3 rot = get_rotation();
    rot.y = steering_angle;
    set_rotation(rot);
}

void Wheel::update_shape_geometry() {
    Ref<ConvexPolygonShape3D> poly = get_shape();
    if (poly.is_null()) {
        poly.instantiate();
        set_shape(poly);
    }

    PackedVector3Array points;
    points.resize(wheel_segments * 2);

    float half_width = wheel_width * 0.5f;
    for (int i = 0; i < wheel_segments; ++i) {
        float angle = (Math::TAU * i) / static_cast<float>(wheel_segments);
        float y = Math::cos(angle) * wheel_radius;
        float z = Math::sin(angle) * wheel_radius;

        // move the geometry up by shapecast_offset to start casting from inside the body
        points.set(i, Vector3(-half_width, y + shapecast_offset, z));
        points.set(i + wheel_segments, Vector3(half_width, y + shapecast_offset, z));
    }

    poly->set_points(points);
    set_target_position(Vector3(0.0f, -(suspension_length + shapecast_offset), 0.0f));
}

// top-down safety raycast if the wheel falls under the terrain or gets stuck in a wall
// the beam finds the surface and returns the wheel to the top
bool Wheel::_raycast_rescue(Vector3 &out_ground_point, Vector3 &out_ground_normal) {
    PhysicsDirectSpaceState3D *space = get_world_3d()->get_direct_space_state();
    if (!space) {
        return false;
    }

    const Transform3D xform = get_global_transform();
    const Vector3 up_dir = xform.basis.get_column(1).normalized();
    const Vector3 anchor = xform.origin;

    // hit the beam well above the attachment point (from inside the body) to the bottom point of the wheel
    Vector3 ray_start = anchor + up_dir * (wheel_radius + 0.5f);
    Vector3 ray_end = anchor - up_dir * (suspension_length + wheel_radius);

    Ref<PhysicsRayQueryParameters3D> params = PhysicsRayQueryParameters3D::create(
        ray_start, ray_end, get_collision_mask(),
        { vehicle->get_rid() }
    );

    Dictionary hit = space->intersect_ray(params);
    if (hit.is_empty()) {
        return false;
    }

    Vector3 hit_pos = hit["position"];
    Vector3 hit_norm = hit["normal"];

    // check if the surface is actually higher than the bottom of the completely broken wheel
    float dist_from_anchor = (anchor - hit_pos).dot(up_dir);
    if (dist_from_anchor < (suspension_length + wheel_radius)) {
        out_ground_point = hit_pos;
        out_ground_normal = hit_norm;
        return true;
    }

    return false;
}

void Wheel::process_wheel_physics(double delta, float drive_torque, float brake_force) {
    if (!vehicle) return;

    const float d = static_cast<float>(delta);
    const float max_slope_rad = Math::deg_to_rad(max_slope_angle_deg);
    const int wheel_count = Math::max(1, vehicle->get_wheel_count());
    const float vehicle_mass = vehicle->get_mass();
    const float mass_per_wheel = vehicle_mass / static_cast<float>(wheel_count);

    Transform3D wheel_xform = get_global_transform();
    Vector3 wheel_up = wheel_xform.basis.get_column(1).normalized();
    Vector3 wheel_right = wheel_xform.basis.get_column(0).normalized();
    Vector3 wheel_forward = -wheel_xform.basis.get_column(2).normalized();
    Vector3 anchor_pos = wheel_xform.origin;

    // update shapecast
    force_shapecast_update();

    const bool colliding = is_colliding();
    const int hit_count = colliding ? get_collision_count() : 0;

    // Separate contacts into
    // SUPPORT (ground/rocks)
    // WALLS (vertical obstacles)
    Vector3 ground_normal(0, 0, 0);
    Vector3 ground_point(0, 0, 0);
    Vector3 wall_normal(0, 0, 0);
    Vector3 wall_point(0, 0, 0);
    int ground_hits = 0;
    int wall_hits = 0;

    // calculate contact ponts
    for (int i = 0; i < hit_count; ++i) {
        Vector3 n = get_collision_normal(i);
        Vector3 p = get_collision_point(i);
        float slope = Math::acos(Math::clamp(n.dot(Vector3::UP), -1.0f, 1.0f));

        if (slope <= max_slope_rad) {
            ground_normal += n;
            ground_point += p;
            ground_hits++;
        }
        else {
            wall_normal += n;
            wall_point += p;
            wall_hits++;
        }
    }

    // 1. collision with a wall
    // even if we stand on the ground
    if (wall_hits > 0) {
        wall_normal = (wall_normal / static_cast<float>(wall_hits)).normalized();
        wall_point /= static_cast<float>(wall_hits);

        // project wall normal onto the horizontal plane to avoid throwing the car upwards
        Vector3 push_dir = wall_normal;
        push_dir.y = 0.0f;
        if (push_dir.length_squared() > 0.0001f) {
            push_dir.normalize();

            // contact positions
            const Vector3 wall_offset = wall_point - vehicle->get_global_position();
            const Vector3 point_vel = vehicle->get_linear_velocity() + vehicle->get_angular_velocity().cross(wall_offset);

            // entry speed into the wall
            const float vel_into_wall = -point_vel.dot(push_dir);

            // harshly reduce the speed directed towards the wall (bumper effect)
            if (vel_into_wall > 0.0f) {
                float stop_force = (mass_per_wheel * vel_into_wall) / d;
                stop_force = Math::min(stop_force, vehicle_mass * 30.0f);
                vehicle->apply_force(push_dir * stop_force, wall_offset);
            }

            // constant pushing if the wheel is already pressed into the wall (anti-penetration)
            Vector3 current_wheel_center = anchor_pos - wheel_up * (suspension_length * (1.0f - current_compression));
            float dist_to_wall = (current_wheel_center - wall_point).dot(push_dir);
            float penetration = wheel_radius - dist_to_wall;

            // soft progressive expulsion (like an elastic air chamber)
            if (penetration > 0.0f) {
                float depenetration_force = penetration * spring_stiffness * 0.45f;
                depenetration_force = Math::min(depenetration_force, vehicle_mass * 15.0f);
                vehicle->apply_force(push_dir * depenetration_force, wall_offset);
            }

            // tread effect: if wheel accelerate towards an obstacle, the wheel climbs up
            if (is_driven && Math::abs(drive_torque) > 0.001f) {
                // check if we heading towards this wall
                float driving_into_wall = -wheel_forward.dot(push_dir) * Math::sign(drive_torque);
                if (driving_into_wall > 0.2f) {
                    float climb_force = Math::min(Math::abs(drive_torque / wheel_radius) * 0.65f, vehicle_mass * 8.0f);
                    vehicle->apply_force(Vector3::UP * climb_force * driving_into_wall, wall_offset);
                }
            }
        }
    }

    // 2. suspension compression calculation + safety raycast from sag
    bool has_ground = (ground_hits > 0);
    bool rescued_by_ray = false;

    if (has_ground) {
        ground_normal = (ground_normal / static_cast<float>(ground_hits)).normalized();
        ground_point /= static_cast<float>(ground_hits);

        float safe_fraction = get_closest_collision_safe_fraction();
        float total_cast = suspension_length + shapecast_offset;
        float hit_dist = safe_fraction * total_cast;

        // if safe_fraction == 0 (the cast is stuck in the wall from the side)
        // prevent the suspension from compressing to 1.0!
        // calculate the compression directly based on the height of the ground point found
        if (safe_fraction < 0.001f) {
            float vertical_dist = (anchor_pos - ground_point).dot(wheel_up) - wheel_radius;
            current_compression = Math::clamp(1.0f - (vertical_dist / suspension_length), 0.0f, 1.0f);
        }
        else {
            float actual_sus_len = Math::max(0.0f, hit_dist - shapecast_offset);
            current_compression = Math::clamp(1.0f - (actual_sus_len / suspension_length), 0.0f, 1.0f);
        }
    }

    // check for subsidence under the collision/terrain
    // if ShapeCast cant see the ground OR the wheel is deeper than ground
    Vector3 rescue_point, rescue_normal;
    if (_raycast_rescue(rescue_point, rescue_normal)) {
        float slope = Math::acos(Math::clamp(rescue_normal.dot(Vector3::UP), -1.0f, 1.0f));

        if (slope <= max_slope_rad) {
            const float ray_sus_len = (anchor_pos - rescue_point).dot(wheel_up) - wheel_radius;
            const float ray_compression = Math::clamp(1.0f - (ray_sus_len / suspension_length), 0.0f, 1.0f);

            // if the raycast shows that the ground is higher (more compression)
            // than ShapeCast thinks, then we correct it
            if (!has_ground || ray_compression > current_compression + 0.05f) {
                has_ground = true;
                rescued_by_ray = true;
                ground_point = rescue_point;
                ground_normal = rescue_normal;
                current_compression = ray_compression;
            }
        }
    }

    // 3. wheel in the air
    // no ground, no safety reaycast activation
    if (!has_ground) {
        prev_compression = 0.0f;
        current_compression = 0.0f;
        filtered_comp_vel = 0.0f;
        was_colliding = false;

        if (is_driven && Math::abs(drive_torque) > 0.001f) {
            wheel_angular_velocity += (drive_torque * 0.02f) * d;
        }
        else {
            wheel_angular_velocity = Math::move_toward(
                wheel_angular_velocity, 0.0f, (brake_force * 0.05f + 2.0f) * d
            );
        }

        wheel_angular_velocity = Math::clamp(wheel_angular_velocity, -40.0f, 40.0f);
        if (wheel_mesh) {
            wheel_mesh->set_position(Vector3(0.0f, -suspension_length, 0.0f));
        }

        return;
    }

    // 4. physics of suspension and traction on the ground
    if (!was_colliding) {
        prev_compression = current_compression;
        filtered_comp_vel = 0.0f;
        was_colliding = true;
    }

    float raw_vel = (current_compression - prev_compression) / d;
    raw_vel = Math::clamp(raw_vel, -6.0f, 6.0f);
    filtered_comp_vel = Math::lerp(filtered_comp_vel, raw_vel, 0.3f);
    prev_compression = current_compression;

    // simulated soft pneumatic tire
    // simulated main suspension + asymmetric shock absorber
    // the first 20% of compression is a soft deflection of the tire itself (coefficient 0.35)
    float tire_cushion = Math::smoothstep(0.0f, 0.25f, current_compression);
    float effective_comp = current_compression * Math::lerp(0.35f, 1.0f, tire_cushion);

    float stiffness_mult = 1.0f + spring_stiffness_progression * current_compression * current_compression;
    float eff_damper = (filtered_comp_vel < 0.0f)
        ? damper_stiffness * rebound_damper_multiplier
        : damper_stiffness;

    float spring_force = effective_comp * (spring_stiffness * stiffness_mult) + filtered_comp_vel * eff_damper;
    spring_force = Math::clamp(spring_force, 0.0f, vehicle_mass * 20.0f);

    Vector3 force_offset = ground_point - vehicle->get_global_position();
    vehicle->apply_force(ground_normal * spring_force, force_offset);

    // vectors of motion in the ground plane
    Vector3 forward_ground = (wheel_forward - ground_normal * wheel_forward.dot(ground_normal)).normalized();
    Vector3 right_ground = (wheel_right - ground_normal * wheel_right.dot(ground_normal)).normalized();
    Vector3 point_vel = vehicle->get_linear_velocity() + vehicle->get_angular_velocity().cross(force_offset);
    float forward_speed = point_vel.dot(forward_ground);
    float lateral_speed = point_vel.dot(right_ground);

    // effective tire load (considering the contact patch)
    // even with light contact, the tread grips the ground (at least 35% of the static wheel load)
    float static_wheel_load = mass_per_wheel * 9.81f;
    float contact_patch_preload = static_wheel_load * 0.35f;
    float effective_load = Math::max(spring_force, contact_patch_preload);

    // increased grip at low speeds
    float speed_sq = point_vel.length_squared();
    float offroad_grip_boost = (speed_sq < 6.0f) ? Math::lerp(1.65f, 1.0f, speed_sq / 6.0f) : 1.0f;

    // lateral grip
    float slip_ratio = Math::abs(lateral_speed) / Math::max(Math::abs(forward_speed), 0.35f);

    // steeper approach to the peak of grip - the tire immediately "rests" with its sidewall on the ground
    const float lat_curve = Math::sin(Math::min<float>(slip_ratio * 3.5f, Math::PI * 0.5f));
    const float lat_cancel = (mass_per_wheel * Math::abs(lateral_speed)) / d;
    const float max_lat_force = effective_load * lateral_grip * offroad_grip_boost * lat_curve;
    float F_lat = -Math::sign(lateral_speed) * Math::min(max_lat_force, lat_cancel);

    if (Math::abs(lateral_speed) < 0.01f) {
        F_lat = 0.0f;
    }

    vehicle->apply_force(right_ground * F_lat, force_offset);

    // longitudinal thrust
    float F_lon = 0.0f;
    bool has_throttle = is_driven && (Math::abs(drive_torque) > 0.001f);

    if (has_throttle) {
        float F_traction = drive_torque / wheel_radius;
        float max_lon_force = effective_load * forward_grip * offroad_grip_boost;

        // reverse while moving
        // are we trying to move AGAINST the current flow
        bool is_reversing_motion = (Math::sign(F_traction) != Math::sign(forward_speed)) 
                                && (Math::abs(forward_speed) > 0.1f);

        if (is_reversing_motion) {
            // when shifting into reverse, the tread acts as a powerful brake and traction
            // we increase the traction limit by 1.8 times and increase the torque to instantly eliminate inertia
            float reverse_max_force = max_lon_force * 1.8f;
            F_lon += Math::clamp(F_traction * 1.5f, -reverse_max_force, reverse_max_force);
        }
        else {
            // normal acceleration in the direction of movement
            F_lon += Math::clamp(F_traction, -max_lon_force, max_lon_force);
        }
    }

    // rolling resistance and brakes
    // important: if you press the gas, dont let the brakes block acceleration
    float active_brake = has_throttle ? 0.0f : brake_force;
    float resistance = active_brake + rolling_resistance * spring_force;

    if (Math::abs(forward_speed) > 0.05f) {
        F_lon -= Math::sign(forward_speed) * resistance;
    }
    else if (!has_throttle) {
        float lon_cancel = (mass_per_wheel * Math::abs(forward_speed)) / d;
        F_lon -= Math::sign(forward_speed) * Math::min(lon_cancel, effective_load * forward_grip);
    }

    vehicle->apply_force(forward_ground * F_lon, force_offset);

    // 5. wheel visual mesh update
    float target_y = -suspension_length * (1.0f - current_compression);
    float target_ang_vel = forward_speed / wheel_radius;
    process_wheel_visual(d, target_y, target_ang_vel, rescued_by_ray);
}

void Wheel::process_wheel_visual(float d, float target_y, float target_ang_vel, bool instant_y) {
    if (!wheel_mesh) return;

    // suspension ride smoothness
    float current_y = wheel_mesh->get_position().y;
    float smooth_y = instant_y
        ? target_y
        : Math::lerp(current_y, target_y, Math::min(1.0f, 25.0f * d));

    // calculates camber from current_compression (0.0 in air -> 1.0 at full compression)
    float camber_deg = Math::lerp(camber_droop_deg, camber_compressed_deg, current_compression);
    
    // camber
    // in air (+2.5°): bottom of the wheel is pulled toward the center of car
    // in compression (-4.0°): bottom of the wheel is pulled outward from car
    float side_sign = is_left_wheel ? 1.0f : -1.0f;
    float camber_rad = Math::deg_to_rad(camber_deg) * side_sign;

    // micro X-shift (for realistic lever geometry)
    float lateral_shift = Math::sin(camber_rad) * wheel_radius * 0.4f;
    wheel_mesh->set_position(Vector3(lateral_shift, smooth_y, 0.0f));

    // rotation: first camber in Z, then roll in X
    // then initial rotation of the mesh model
    wheel_angular_velocity = Math::lerp(wheel_angular_velocity, target_ang_vel, 15.0f * d);
    spin_angle = Math::fmod(spin_angle - wheel_angular_velocity * d, Math::TAU);

    const Basis camber_basis = Basis(Vector3(0.0f, 0.0f, 1.0f), camber_rad);
    const Basis spin_basis   = Basis(Vector3(1.0f, 0.0f, 0.0f), static_cast<float>(spin_angle));
    wheel_mesh->set_basis(camber_basis * spin_basis * wheel_mesh_basis);
}

void Wheel::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_wheel_segments", "segments"), &Wheel::set_wheel_segments);
    ClassDB::bind_method(D_METHOD("get_wheel_segments"), &Wheel::get_wheel_segments);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "wheel_segments"), "set_wheel_segments", "get_wheel_segments");

    ClassDB::bind_method(D_METHOD("set_wheel_radius", "radius"), &Wheel::set_wheel_radius);
    ClassDB::bind_method(D_METHOD("get_wheel_radius"), &Wheel::get_wheel_radius);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wheel_radius"), "set_wheel_radius", "get_wheel_radius");

    ClassDB::bind_method(D_METHOD("set_wheel_width", "width"), &Wheel::set_wheel_width);
    ClassDB::bind_method(D_METHOD("get_wheel_width"), &Wheel::get_wheel_width);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wheel_width"), "set_wheel_width", "get_wheel_width");

    ClassDB::bind_method(D_METHOD("set_suspension_length", "length"), &Wheel::set_suspension_length);
    ClassDB::bind_method(D_METHOD("get_suspension_length"), &Wheel::get_suspension_length);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_length"), "set_suspension_length", "get_suspension_length");

    ClassDB::bind_method(D_METHOD("set_spring_stiffness", "stiffness"), &Wheel::set_spring_stiffness);
    ClassDB::bind_method(D_METHOD("get_spring_stiffness"), &Wheel::get_spring_stiffness);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "spring_stiffness"), "set_spring_stiffness", "get_spring_stiffness");

    ClassDB::bind_method(D_METHOD("set_damper_stiffness", "damper"), &Wheel::set_damper_stiffness);
    ClassDB::bind_method(D_METHOD("get_damper_stiffness"), &Wheel::get_damper_stiffness);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "damper_stiffness"), "set_damper_stiffness", "get_damper_stiffness");

    ClassDB::bind_method(D_METHOD("set_damper_rebound", "rebound"), &Wheel::set_damper_rebound);
    ClassDB::bind_method(D_METHOD("get_damper_rebound"), &Wheel::get_damper_rebound);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "damper_rebound"), "set_damper_rebound", "get_damper_rebound");

    ClassDB::bind_method(D_METHOD("set_lateral_grip", "grip"), &Wheel::set_lateral_grip);
    ClassDB::bind_method(D_METHOD("get_lateral_grip"), &Wheel::get_lateral_grip);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "lateral_grip"), "set_lateral_grip", "get_lateral_grip");

    ClassDB::bind_method(D_METHOD("set_is_steerable", "steerable"), &Wheel::set_is_steerable);
    ClassDB::bind_method(D_METHOD("get_is_steerable"), &Wheel::get_is_steerable);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "is_steerable"), "set_is_steerable", "get_is_steerable");

    ClassDB::bind_method(D_METHOD("set_is_driven", "driven"), &Wheel::set_is_driven);
    ClassDB::bind_method(D_METHOD("get_is_driven"), &Wheel::get_is_driven);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "is_driven"), "set_is_driven", "get_is_driven");

    ClassDB::bind_method(D_METHOD("set_camber_droop_deg", "camber_droop"), &Wheel::set_camber_droop_deg);
    ClassDB::bind_method(D_METHOD("get_camber_droop_deg"), &Wheel::get_camber_droop_deg);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "camber_droop_deg"), "set_camber_droop_deg", "get_camber_droop_deg");

    ClassDB::bind_method(D_METHOD("set_camber_compressed_deg", "camber_compressed"), &Wheel::set_camber_compressed_deg);
    ClassDB::bind_method(D_METHOD("get_camber_compressed_deg"), &Wheel::get_camber_compressed_deg);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "camber_compressed"), "set_camber_compressed_deg", "get_camber_compressed_deg");

    ClassDB::bind_method(D_METHOD("set_wheel_mesh", "mesh"), &Wheel::set_wheel_mesh);
    ClassDB::bind_method(D_METHOD("get_wheel_mesh"), &Wheel::get_wheel_mesh);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "wheel_mesh", PROPERTY_HINT_NODE_TYPE, "Node3D"), "set_wheel_mesh", "get_wheel_mesh");
}