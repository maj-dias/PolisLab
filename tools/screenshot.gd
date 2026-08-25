extends Node

## Dev helper: renders the scene, writes an overview PNG plus a low-altitude
## PNG where the cars and signal heads are actually legible, then quits.

@export var warmup_frames := 60

func _ready() -> void:
	_capture.call_deferred()

func _save(name: String) -> void:
	await RenderingServer.frame_post_draw
	var image: Image = get_viewport().get_texture().get_image()
	var absolute := ProjectSettings.globalize_path("res://%s" % name)
	var err := image.save_png(absolute)
	print("[shot] %s -> %s" % ["ok" if err == OK else "FAILED err=%d" % err, name])

## End-to-end check of the routing stack -- the piece the dispatch layer needs.
func _report_routing() -> void:
	var city := get_node_or_null("Main/City")
	if city == null:
		return
	var net = city.get_network()
	if net == null:
		print("[route] no network")
		return

	var corner_a: int = net.nearest_junction(Vector3(-400, 0, -400))
	var corner_b: int = net.nearest_junction(Vector3(400, 0, 400))
	var route: PackedInt32Array = net.find_route(corner_a, corner_b)
	print("[route] %d -> %d: %d hops, %.0f m, ETA %.0f s (%.1f km/h effective)" % [
		corner_a, corner_b, route.size(),
		net.route_length(route), net.route_travel_time(route),
		(net.route_length(route) / max(net.route_travel_time(route), 0.001)) * 3.6])

	# The route should prefer avenues and the ring road over the fine grid, so
	# compare it against the straight-line distance.
	var from_pos: Vector3 = net.get_junction_position(corner_a)
	var to_pos: Vector3 = net.get_junction_position(corner_b)
	var straight: float = from_pos.distance_to(to_pos)
	print("[route] straight-line %.0f m -> detour factor %.2f" % [
		straight, net.route_length(route) / max(straight, 0.001)])


func _capture() -> void:
	for i in warmup_frames:
		await get_tree().process_frame

	_report_routing()

	var traffic := get_node_or_null("Main/Traffic")
	if traffic:
		print("[shot] traffic: %d cars, mean speed %.1f m/s" % [
			traffic.get_active_vehicle_count(), traffic.get_average_speed()])

	await _save("shot.png")

	# Drop down to street level over a mid-town intersection.
	var camera := get_viewport().get_camera_3d()
	camera.global_position = Vector3(210.0, 62.0, 210.0)
	camera.look_at(Vector3(70.0, 6.0, 70.0), Vector3.UP)
	camera.fov = 55.0

	for i in 4:
		await get_tree().process_frame
	await _save("shot_street.png")

	await _check_regeneration()
	get_tree().quit()


## Regenerating the city hands out a brand-new RoadNetwork; traffic has to
## follow it instead of driving on the discarded one.
func _check_regeneration() -> void:
	var city := get_node_or_null("Main/City")
	var traffic := get_node_or_null("Main/Traffic")
	if city == null or traffic == null:
		return

	var before: int = city.get_network().get_segment_count()
	print("[regen] before: %d segments, %d cars" % [before, traffic.get_active_vehicle_count()])

	city.city_seed = 4242
	city.blocks_x = 8
	city.blocks_z = 8
	city.generate()
	await get_tree().process_frame

	var after: int = city.get_network().get_segment_count()
	print("[regen] after:  %d segments, %d cars" % [after, traffic.get_active_vehicle_count()])

	# If traffic were still holding the old graph, a route id valid only in the
	# new one would be meaningless -- so route on the new network and check the
	# cars are actually inside its bounds.
	var net = city.get_network()
	var route: PackedInt32Array = net.find_route(0, net.get_junction_count() - 1)
	print("[regen] route on new graph: %d hops, %.0f m" % [route.size(), net.route_length(route)])
