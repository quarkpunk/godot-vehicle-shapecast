extends Camera3D

@export var target: Node3D
@export var orbit_speed: float = 0.3
@export var zoom_speed: float = 5.0
@export var min_distance: float = 1.0
@export var max_distance: float = 20.0
@export var start_distance: float = 10.0
@export var smooth_damp_time: float = 0.15
var _current_distance: float
var _velocity: Vector3 = Vector3.ZERO
var _horizontal_angle: float = 0.0
var _vertical_angle: float = 20.0
var _is_dragging: bool = false

func _ready() -> void:
	_current_distance = start_distance
	if target == null:
		print("camera: not found target node")

func _input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_LEFT:
			_is_dragging = event.pressed
		
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			_current_distance = max(_current_distance - zoom_speed, min_distance)
			get_viewport().set_input_as_handled()
		
		if event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			_current_distance = min(_current_distance + zoom_speed, max_distance)
			get_viewport().set_input_as_handled()
	
	if event is InputEventMouseMotion and _is_dragging:
		var mouse_delta = event.relative
		_horizontal_angle -= mouse_delta.x * orbit_speed * 0.01
		_vertical_angle += mouse_delta.y * orbit_speed * 0.01
		_vertical_angle = clamp(_vertical_angle, -80.0, 80.0)
		
		get_viewport().set_input_as_handled()

func _physics_process(delta: float) -> void:
	if target == null:
		return
	
	var h_rad: float = deg_to_rad(_horizontal_angle)
	var v_rad: float = deg_to_rad(_vertical_angle)
	var x: float = _current_distance * cos(v_rad) * sin(h_rad)
	var y: float = _current_distance * sin(v_rad)
	var z: float = _current_distance * cos(v_rad) * cos(h_rad)
	
	var orbit_position = Vector3(x, y, z)
	var target_position = target.global_position + orbit_position
	
	global_position = _smooth_damp(global_position, target_position, _velocity, smooth_damp_time, delta)
	look_at(target.global_position, Vector3.UP)

func _smooth_damp(_current: Vector3, _target: Vector3, velocity: Vector3, smooth_time: float, delta: float) -> Vector3:
	var omega: float = 2.0 / smooth_time
	var x: float = omega * delta
	var exp_neg: float = exp(-x)
	
	var temp_velocity: Vector3 = (_target - _current) * omega * exp_neg
	velocity = velocity * exp_neg + temp_velocity * 0.5
	
	var result: Vector3 = _target - (_target - _current - velocity * delta) * exp_neg
	return result
