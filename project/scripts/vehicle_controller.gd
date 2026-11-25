extends Node

@export var vehicle: Node3D

func _ready() -> void:
	if not vehicle:
		print("vehicle: not found Vehicle node")
		queue_free()

func _physics_process(_delta: float) -> void:
	vehicle.set_steer_input(Input.get_axis("Right", "Left"))
	vehicle.set_throttle_input(Input.get_axis("Backward", "Throttle"))
	vehicle.set_brake_input(Input.is_action_pressed("Brake"))
