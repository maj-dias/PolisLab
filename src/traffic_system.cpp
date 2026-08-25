#include "traffic_system.h"

#include "city_generator.h"

#include <godot_cpp/classes/box_mesh.hpp>
#include <godot_cpp/classes/cylinder_mesh.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/callable_method_pointer.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <algorithm>
#include <cmath>

using namespace godot;

namespace {

// Intelligent Driver Model constants. These are the usual urban-traffic values:
// comfortable acceleration well under 2 m/s^2 and a ~1.3 s headway.
constexpr double IDM_MAX_ACCEL = 1.9;
constexpr double IDM_COMFORT_BRAKE = 2.8;
constexpr double IDM_MIN_GAP = 2.4;
constexpr double IDM_HEADWAY = 1.3;
constexpr double IDM_EMERGENCY_BRAKE = 8.0;

constexpr double CAR_LENGTH = 4.3;
constexpr double CAR_WIDTH = 1.8;
constexpr double CAR_HEIGHT = 1.35;
constexpr double SIGNAL_HEAD_Y = 4.7;
constexpr double SIGNAL_POLE_HEIGHT = 5.0;

const Color SIGNAL_GREEN(0.13, 0.86, 0.28);
const Color SIGNAL_YELLOW(0.96, 0.74, 0.10);
const Color SIGNAL_RED(0.92, 0.14, 0.11);

// Cars look best as a spread of desaturated body colours with a few bright ones.
Color pick_car_color(const Ref<RandomNumberGenerator> &p_rng) {
	if (p_rng->randf() < 0.22) {
		return Color((real_t)p_rng->randf_range(0.4, 0.95), (real_t)p_rng->randf_range(0.1, 0.5),
				(real_t)p_rng->randf_range(0.1, 0.4));
	}
	const double shade = p_rng->randf_range(0.12, 0.85);
	const double tint = p_rng->randf_range(-0.05, 0.05);
	return Color((real_t)(shade + tint), (real_t)shade, (real_t)(shade - tint));
}

Ref<StandardMaterial3D> make_material(double p_roughness, double p_metallic, bool p_unshaded) {
	Ref<StandardMaterial3D> material;
	material.instantiate();
	material->set_flag(BaseMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
	material->set_roughness(p_roughness);
	material->set_metallic(p_metallic);
	if (p_unshaded) {
		material->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
	}
	return material;
}

} // namespace

TrafficSystem::TrafficSystem() {
	rng.instantiate();
}

void TrafficSystem::_bind_methods() {
	ClassDB::bind_method(D_METHOD("restart"), &TrafficSystem::restart);
	ClassDB::bind_method(D_METHOD("get_active_vehicle_count"), &TrafficSystem::get_active_vehicle_count);
	ClassDB::bind_method(D_METHOD("get_average_speed"), &TrafficSystem::get_average_speed);

#define POLIS_PROPERTY(m_type, m_name, m_hint, m_hint_string)                                  \
	ClassDB::bind_method(D_METHOD("set_" #m_name, "value"), &TrafficSystem::set_##m_name);      \
	ClassDB::bind_method(D_METHOD("get_" #m_name), &TrafficSystem::get_##m_name);               \
	ADD_PROPERTY(PropertyInfo(m_type, #m_name, m_hint, m_hint_string), "set_" #m_name, "get_" #m_name);

	POLIS_PROPERTY(Variant::NODE_PATH, city_path, PROPERTY_HINT_NODE_PATH_VALID_TYPES, "CityGenerator")
	POLIS_PROPERTY(Variant::INT, vehicle_count, PROPERTY_HINT_RANGE, "0,6000,1")
	POLIS_PROPERTY(Variant::INT, random_seed, PROPERTY_HINT_NONE, "")
	POLIS_PROPERTY(Variant::FLOAT, lane_width, PROPERTY_HINT_RANGE, "2,8,0.1")
	POLIS_PROPERTY(Variant::FLOAT, green_time, PROPERTY_HINT_RANGE, "3,60,0.5")
	POLIS_PROPERTY(Variant::FLOAT, yellow_time, PROPERTY_HINT_RANGE, "1,10,0.5")
	POLIS_PROPERTY(Variant::FLOAT, time_scale, PROPERTY_HINT_RANGE, "0,8,0.1")
	POLIS_PROPERTY(Variant::BOOL, show_signal_heads, PROPERTY_HINT_NONE, "")
#undef POLIS_PROPERTY

	BIND_ENUM_CONSTANT(PHASE_EW_GREEN);
	BIND_ENUM_CONSTANT(PHASE_EW_YELLOW);
	BIND_ENUM_CONSTANT(PHASE_NS_GREEN);
	BIND_ENUM_CONSTANT(PHASE_NS_YELLOW);
}

void TrafficSystem::_ready() {
	restart();
}

void TrafficSystem::_on_city_generated() {
	// The city just rebuilt itself, which means a brand-new RoadNetwork. Our
	// vehicles are indexed against the old one, so they have to be respawned.
	if (rebuilding) {
		return;
	}
	restart();
}

void TrafficSystem::restart() {
	if (rebuilding) {
		return;
	}
	rebuilding = true;

	TypedArray<Node> children = get_children();
	for (int i = 0; i < children.size(); i++) {
		Node *child = Object::cast_to<Node>(children[i]);
		if (child) {
			remove_child(child);
			child->queue_free();
		}
	}
	vehicle_node = nullptr;
	head_node = nullptr;
	vehicle_multimesh.unref();
	head_multimesh.unref();

	rng->set_seed((uint64_t)random_seed);

	_resolve_network();
	if (network.is_null()) {
		rebuilding = false;
		return;
	}

	_build_signals();
	_spawn_vehicles();
	_build_visuals();
	_sync_transforms();
	_sync_signal_colors();

	int controlled = 0;
	for (const SignalState &state : signals) {
		if (state.controlled) {
			controlled++;
		}
	}
	UtilityFunctions::print(vformat("[PolisLab] traffic: %d vehicles, %d signalised intersections, %d signal heads",
			(int)vehicles.size(), controlled, (int)signal_heads.size()));

	rebuilding = false;
}

void TrafficSystem::_resolve_network() {
	network.unref();

	CityGenerator *city = nullptr;
	if (!city_path.is_empty()) {
		city = Object::cast_to<CityGenerator>(get_node_or_null(city_path));
	}
	if (!city) {
		// Fall back to the first CityGenerator sibling, so the scene works even
		// with the path left unset.
		Node *parent = get_parent();
		if (parent) {
			for (int i = 0; i < parent->get_child_count(); i++) {
				city = Object::cast_to<CityGenerator>(parent->get_child(i));
				if (city) {
					break;
				}
			}
		}
	}
	ERR_FAIL_NULL_MSG(city, "TrafficSystem needs a CityGenerator; set city_path.");

	// Every generate() hands out a fresh RoadNetwork, so follow the city rather
	// than caching a graph that can silently go stale under us.
	const Callable on_generated = callable_mp(this, &TrafficSystem::_on_city_generated);
	if (!city->is_connected("city_generated", on_generated)) {
		city->connect("city_generated", on_generated);
	}

	// _ready order between siblings is not something to rely on.
	if (city->get_network().is_null()) {
		city->generate();
	}
	network = city->get_network();
}

void TrafficSystem::_build_signals() {
	const int count = network->junction_count_fast();
	signals.assign(count, SignalState());
	junction_radius.assign(count, 4.5);
	signal_heads.clear();

	const double cycle = 2.0 * (green_time + yellow_time);

	for (int j = 0; j < count; j++) {
		const std::vector<int> &attached = network->segments_of(j);

		double radius = 4.0;
		for (int s : attached) {
			switch (network->segment_at(s).road_class) {
				case RoadNetwork::ROAD_HIGHWAY:
					radius = std::max(radius, 11.5);
					break;
				case RoadNetwork::ROAD_AVENUE:
					radius = std::max(radius, 8.5);
					break;
				default:
					radius = std::max(radius, 5.0);
					break;
			}
		}
		junction_radius[j] = radius;

		if (attached.size() < 3) {
			continue; // dead ends and simple bends stay uncontrolled
		}
		signals[j].controlled = true;

		// Stagger the phase so the grid produces green waves rather than one
		// city-wide blink.
		double offset = rng->randf_range(0.0, cycle);
		if (offset < green_time) {
			signals[j].phase = PHASE_EW_GREEN;
			signals[j].timer = (float)(green_time - offset);
		} else if ((offset -= green_time) < yellow_time) {
			signals[j].phase = PHASE_EW_YELLOW;
			signals[j].timer = (float)(yellow_time - offset);
		} else if ((offset -= yellow_time) < green_time) {
			signals[j].phase = PHASE_NS_GREEN;
			signals[j].timer = (float)(green_time - offset);
		} else {
			signals[j].phase = PHASE_NS_YELLOW;
			signals[j].timer = (float)(yellow_time - (offset - green_time));
		}

		// One head per approach, mounted near-side on the driver's right.
		const Vector3 junction = network->junction_at(j);
		for (int s : attached) {
			const RoadNetwork::Segment &segment = network->segment_at(s);
			const int other = segment.a == j ? segment.b : segment.a;
			Vector3 outward = network->junction_at(other) - junction;
			outward.y = 0;
			if (outward.length_squared() < 0.01) {
				continue;
			}
			outward.normalize();

			// Traffic arrives along -outward, so its right-hand side is this.
			const Vector3 right(outward.z, 0, -outward.x);
			const Vector3 base = junction + outward * (real_t)(radius + 1.2) + right * (real_t)(radius * 0.85);

			SignalHead head;
			head.junction = j;
			head.east_west = std::abs(outward.x) > std::abs(outward.z);
			head.transform.origin = Vector3(base.x, (real_t)SIGNAL_HEAD_Y, base.z);
			signal_heads.push_back(head);
		}
	}
}

void TrafficSystem::_spawn_vehicles() {
	vehicles.clear();
	if (vehicle_count <= 0 || network->segment_count_fast() == 0) {
		return;
	}
	vehicles.reserve(vehicle_count);

	for (int i = 0; i < vehicle_count; i++) {
		Vehicle vehicle;
		vehicle.length = CAR_LENGTH;
		vehicle.color = pick_car_color(rng);
		_assign_new_route(vehicle);
		if (vehicle.segment < 0) {
			continue;
		}
		// Scatter them along their first segment so the city does not start
		// with every car stacked on a junction.
		const double length = network->segment_at(vehicle.segment).length;
		vehicle.s = rng->randf_range(0.0, std::max(length - CAR_LENGTH, 0.0));
		vehicle.speed = vehicle.desired_speed * rng->randf_range(0.3, 0.9);
		vehicles.push_back(vehicle);
	}
}

void TrafficSystem::_assign_new_route(Vehicle &r_vehicle) {
	const int count = network->junction_count_fast();
	if (count < 2) {
		r_vehicle.segment = -1;
		return;
	}

	const int origin = (r_vehicle.to >= 0 && r_vehicle.to < count) ? r_vehicle.to : rng->randi_range(0, count - 1);

	for (int attempt = 0; attempt < 16; attempt++) {
		const int destination = rng->randi_range(0, count - 1);
		if (destination == origin) {
			continue;
		}
		const PackedInt32Array route = network->find_route(origin, destination);
		if (route.size() < 2) {
			continue;
		}
		const int segment = network->find_segment(route[0], route[1]);
		if (segment < 0) {
			continue;
		}

		r_vehicle.route = route;
		r_vehicle.route_index = 0;
		r_vehicle.from = route[0];
		r_vehicle.to = route[1];
		r_vehicle.segment = segment;
		r_vehicle.s = 0.0;
		r_vehicle.desired_speed = network->segment_at(segment).speed_limit * rng->randf_range(0.82, 1.06);
		return;
	}

	r_vehicle.segment = -1;
}

void TrafficSystem::_advance_to_next_segment(Vehicle &r_vehicle) {
	r_vehicle.route_index++;
	if (r_vehicle.route_index + 1 >= r_vehicle.route.size()) {
		// Arrived. Pick a fresh destination so the city never empties out --
		// this is the hook the dispatch layer will replace later.
		_assign_new_route(r_vehicle);
		return;
	}

	const int from = r_vehicle.route[r_vehicle.route_index];
	const int to = r_vehicle.route[r_vehicle.route_index + 1];
	const int segment = network->find_segment(from, to);
	if (segment < 0) {
		_assign_new_route(r_vehicle);
		return;
	}

	r_vehicle.from = from;
	r_vehicle.to = to;
	r_vehicle.segment = segment;
	r_vehicle.desired_speed = network->segment_at(segment).speed_limit * rng->randf_range(0.82, 1.06);
}

bool TrafficSystem::_approach_is_open(int p_junction, bool p_east_west) const {
	if (p_junction < 0 || p_junction >= (int)signals.size()) {
		return true;
	}
	const SignalState &state = signals[p_junction];
	if (!state.controlled) {
		return true;
	}
	return p_east_west ? state.phase == PHASE_EW_GREEN : state.phase == PHASE_NS_GREEN;
}

double TrafficSystem::_stop_line_distance(int p_junction) const {
	if (p_junction < 0 || p_junction >= (int)junction_radius.size()) {
		return 5.0;
	}
	return junction_radius[p_junction] + 1.0;
}

void TrafficSystem::_step_signals(double p_delta) {
	for (SignalState &state : signals) {
		if (!state.controlled) {
			continue;
		}
		state.timer -= (float)p_delta;
		if (state.timer > 0.0f) {
			continue;
		}
		switch (state.phase) {
			case PHASE_EW_GREEN:
				state.phase = PHASE_EW_YELLOW;
				state.timer = (float)yellow_time;
				break;
			case PHASE_EW_YELLOW:
				state.phase = PHASE_NS_GREEN;
				state.timer = (float)green_time;
				break;
			case PHASE_NS_GREEN:
				state.phase = PHASE_NS_YELLOW;
				state.timer = (float)yellow_time;
				break;
			default:
				state.phase = PHASE_EW_GREEN;
				state.timer = (float)green_time;
				break;
		}
	}
}

void TrafficSystem::_step_vehicles(double p_delta) {
	const int bucket_count = network->segment_count_fast() * 2;
	if ((int)lane_buckets.size() != bucket_count) {
		lane_buckets.assign(bucket_count, std::vector<int>());
	}
	for (std::vector<int> &bucket : lane_buckets) {
		bucket.clear();
	}

	// Order every car within its own lane so the vehicle in front is simply the
	// next entry in the bucket.
	for (int i = 0; i < (int)vehicles.size(); i++) {
		const Vehicle &vehicle = vehicles[i];
		if (vehicle.segment < 0) {
			continue;
		}
		const int direction = network->segment_at(vehicle.segment).a == vehicle.from ? 0 : 1;
		lane_buckets[vehicle.segment * 2 + direction].push_back(i);
	}
	for (std::vector<int> &bucket : lane_buckets) {
		if (bucket.size() > 1) {
			std::sort(bucket.begin(), bucket.end(),
					[this](int p_left, int p_right) { return vehicles[p_left].s < vehicles[p_right].s; });
		}
	}

	for (const std::vector<int> &bucket : lane_buckets) {
		for (size_t k = 0; k < bucket.size(); k++) {
			Vehicle &vehicle = vehicles[bucket[k]];
			const RoadNetwork::Segment &segment = network->segment_at(vehicle.segment);

			double gap = 1.0e9;
			double closing_speed = 0.0;
			bool constrained = false;

			if (k + 1 < bucket.size()) {
				const Vehicle &leader = vehicles[bucket[k + 1]];
				gap = (leader.s - leader.length * 0.5) - (vehicle.s + vehicle.length * 0.5);
				closing_speed = vehicle.speed - leader.speed;
				constrained = true;
			}

			// A red or amber light ahead behaves as a stationary obstacle at the
			// stop line -- but only until the car has actually crossed it, so a
			// vehicle already inside the box always clears the intersection.
			const double stop_line = segment.length - _stop_line_distance(vehicle.to);
			if (vehicle.s < stop_line) {
				const Vector3 from = network->junction_at(vehicle.from);
				const Vector3 to = network->junction_at(vehicle.to);
				const bool east_west = std::abs(to.x - from.x) > std::abs(to.z - from.z);
				if (!_approach_is_open(vehicle.to, east_west)) {
					const double light_gap = stop_line - (vehicle.s + vehicle.length * 0.5);
					if (light_gap < gap) {
						gap = light_gap;
						closing_speed = vehicle.speed;
						constrained = true;
					}
				}
			}

			double interaction = 0.0;
			if (constrained) {
				const double desired_gap = IDM_MIN_GAP + std::max(0.0,
						vehicle.speed * IDM_HEADWAY +
								vehicle.speed * closing_speed / (2.0 * std::sqrt(IDM_MAX_ACCEL * IDM_COMFORT_BRAKE)));
				const double ratio = desired_gap / std::max(gap, 0.35);
				interaction = ratio * ratio;
			}

			const double speed_ratio = vehicle.speed / std::max(vehicle.desired_speed, 0.1);
			const double free_road = 1.0 - speed_ratio * speed_ratio * speed_ratio * speed_ratio;
			double acceleration = IDM_MAX_ACCEL * (free_road - interaction);
			acceleration = std::clamp(acceleration, -IDM_EMERGENCY_BRAKE, IDM_MAX_ACCEL);

			vehicle.speed = std::max(0.0, vehicle.speed + acceleration * p_delta);
			vehicle.s += vehicle.speed * p_delta;
		}
	}

	// Hand over to the next segment only after the whole field has moved, so
	// the lane ordering used above stays valid for the entire step.
	for (Vehicle &vehicle : vehicles) {
		if (vehicle.segment < 0) {
			continue;
		}
		const double length = network->segment_at(vehicle.segment).length;
		if (vehicle.s >= length) {
			vehicle.s -= length;
			_advance_to_next_segment(vehicle);
		}
	}
}

void TrafficSystem::_sync_transforms() {
	if (vehicle_multimesh.is_null()) {
		return;
	}
	for (int i = 0; i < (int)vehicles.size(); i++) {
		const Vehicle &vehicle = vehicles[i];
		if (vehicle.segment < 0) {
			// Routing failed for this one; collapse it instead of leaving a
			// stale instance parked at the world origin.
			vehicle_multimesh->set_instance_transform(i, Transform3D(Basis().scaled(Vector3()), Vector3()));
			continue;
		}
		const Vector3 from = network->junction_at(vehicle.from);
		const Vector3 to = network->junction_at(vehicle.to);

		Vector3 forward = to - from;
		forward.y = 0;
		const real_t length = forward.length();
		forward = length > 0.001f ? forward / length : Vector3(1, 0, 0);
		const Vector3 right(-forward.z, 0, forward.x);

		// Keep right, centred in the nearside half of the carriageway, so cars
		// on a wide avenue do not ride the centre line.
		const double half_width = network->segment_at(vehicle.segment).width * 0.5;
		const double offset = std::min(half_width * 0.5, half_width - lane_width * 0.5);

		Basis basis;
		basis.set_column(0, right);
		basis.set_column(1, Vector3(0, 1, 0));
		basis.set_column(2, -forward);

		Transform3D transform(basis,
				from + forward * (real_t)vehicle.s + right * (real_t)offset +
						Vector3(0, (real_t)(CAR_HEIGHT * 0.5), 0));
		vehicle_multimesh->set_instance_transform(i, transform);
	}
}

void TrafficSystem::_sync_signal_colors() {
	if (head_multimesh.is_null()) {
		return;
	}

	bool dirty = false;
	for (const SignalState &state : signals) {
		if (state.controlled && state.phase != state.last_drawn_phase) {
			dirty = true;
			break;
		}
	}
	if (!dirty) {
		return;
	}

	for (int i = 0; i < (int)signal_heads.size(); i++) {
		const SignalHead &head = signal_heads[i];
		const SignalState &state = signals[head.junction];

		Color color = SIGNAL_RED;
		if (head.east_west) {
			if (state.phase == PHASE_EW_GREEN) {
				color = SIGNAL_GREEN;
			} else if (state.phase == PHASE_EW_YELLOW) {
				color = SIGNAL_YELLOW;
			}
		} else {
			if (state.phase == PHASE_NS_GREEN) {
				color = SIGNAL_GREEN;
			} else if (state.phase == PHASE_NS_YELLOW) {
				color = SIGNAL_YELLOW;
			}
		}
		head_multimesh->set_instance_color(i, color);
	}

	for (SignalState &state : signals) {
		state.last_drawn_phase = state.phase;
	}
}

void TrafficSystem::_build_visuals() {
	const Ref<StandardMaterial3D> car_material = make_material(0.34, 0.28, false);
	const Ref<StandardMaterial3D> lamp_material = make_material(0.5, 0.0, true);

	if (!vehicles.empty()) {
		Ref<BoxMesh> body;
		body.instantiate();
		body->set_size(Vector3((real_t)CAR_WIDTH, (real_t)CAR_HEIGHT, (real_t)CAR_LENGTH));

		vehicle_multimesh.instantiate();
		vehicle_multimesh->set_transform_format(MultiMesh::TRANSFORM_3D);
		vehicle_multimesh->set_use_colors(true);
		vehicle_multimesh->set_mesh(body);
		vehicle_multimesh->set_instance_count((int)vehicles.size());
		for (int i = 0; i < (int)vehicles.size(); i++) {
			vehicle_multimesh->set_instance_color(i, vehicles[i].color);
		}

		vehicle_node = memnew(MultiMeshInstance3D);
		vehicle_node->set_name("Vehicles");
		vehicle_node->set_multimesh(vehicle_multimesh);
		vehicle_node->set_material_override(car_material);
		add_child(vehicle_node);
	}

	if (!show_signal_heads || signal_heads.empty()) {
		return;
	}

	Ref<BoxMesh> lamp;
	lamp.instantiate();
	lamp->set_size(Vector3(0.46, 1.15, 0.36));

	head_multimesh.instantiate();
	head_multimesh->set_transform_format(MultiMesh::TRANSFORM_3D);
	head_multimesh->set_use_colors(true);
	head_multimesh->set_mesh(lamp);
	head_multimesh->set_instance_count((int)signal_heads.size());
	for (int i = 0; i < (int)signal_heads.size(); i++) {
		head_multimesh->set_instance_transform(i, signal_heads[i].transform);
		head_multimesh->set_instance_color(i, SIGNAL_RED);
	}

	head_node = memnew(MultiMeshInstance3D);
	head_node->set_name("SignalHeads");
	head_node->set_multimesh(head_multimesh);
	head_node->set_material_override(lamp_material);
	add_child(head_node);

	// Static poles carrying the heads.
	Ref<CylinderMesh> pole;
	pole.instantiate();
	pole->set_top_radius(0.09);
	pole->set_bottom_radius(0.13);
	pole->set_height((real_t)SIGNAL_POLE_HEIGHT);
	pole->set_radial_segments(6);
	pole->set_rings(1);

	Ref<MultiMesh> pole_multimesh;
	pole_multimesh.instantiate();
	pole_multimesh->set_transform_format(MultiMesh::TRANSFORM_3D);
	pole_multimesh->set_mesh(pole);
	pole_multimesh->set_instance_count((int)signal_heads.size());
	for (int i = 0; i < (int)signal_heads.size(); i++) {
		Transform3D transform;
		transform.origin = Vector3(signal_heads[i].transform.origin.x,
				(real_t)(SIGNAL_POLE_HEIGHT * 0.5), signal_heads[i].transform.origin.z);
		pole_multimesh->set_instance_transform(i, transform);
	}

	Ref<StandardMaterial3D> pole_material;
	pole_material.instantiate();
	pole_material->set_albedo(Color(0.17, 0.18, 0.19));
	pole_material->set_roughness(0.6);
	pole_material->set_metallic(0.35);

	MultiMeshInstance3D *pole_node = memnew(MultiMeshInstance3D);
	pole_node->set_name("SignalPoles");
	pole_node->set_multimesh(pole_multimesh);
	pole_node->set_material_override(pole_material);
	add_child(pole_node);
}

void TrafficSystem::_process(double p_delta) {
	if (network.is_null() || vehicles.empty()) {
		return;
	}
	// Clamp the step so a stall or a breakpoint cannot teleport cars through
	// red lights and each other.
	const double step = std::min(p_delta * time_scale, 0.1);
	if (step <= 0.0) {
		return;
	}

	_step_signals(step);
	_step_vehicles(step);
	_sync_transforms();
	_sync_signal_colors();
}

double TrafficSystem::get_average_speed() const {
	if (vehicles.empty()) {
		return 0.0;
	}
	double total = 0.0;
	for (const Vehicle &vehicle : vehicles) {
		total += vehicle.speed;
	}
	return total / (double)vehicles.size();
}

void TrafficSystem::set_city_path(const NodePath &p_value) {
	city_path = p_value;
}

void TrafficSystem::set_vehicle_count(int p_value) {
	vehicle_count = std::max(p_value, 0);
}

void TrafficSystem::set_random_seed(int p_value) {
	random_seed = p_value;
}

void TrafficSystem::set_lane_width(double p_value) {
	lane_width = std::max(p_value, 1.0);
}

void TrafficSystem::set_green_time(double p_value) {
	green_time = std::max(p_value, 1.0);
}

void TrafficSystem::set_yellow_time(double p_value) {
	yellow_time = std::max(p_value, 0.5);
}

void TrafficSystem::set_time_scale(double p_value) {
	time_scale = std::max(p_value, 0.0);
}

void TrafficSystem::set_show_signal_heads(bool p_value) {
	show_signal_heads = p_value;
}
