#ifndef POLISLAB_TRAFFIC_SYSTEM_H
#define POLISLAB_TRAFFIC_SYSTEM_H

#include "road_network.h"

#include <godot_cpp/classes/multi_mesh.hpp>
#include <godot_cpp/classes/multi_mesh_instance3d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/random_number_generator.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/node_path.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/transform3d.hpp>

#include <cstdint>
#include <vector>

namespace godot {

// Drives every moving thing in the city: signal phases at the intersections and
// the vehicles that obey them. Vehicles are plain structs updated in a tight
// loop and drawn through a MultiMesh, so a few thousand of them stay cheap.
class TrafficSystem : public Node3D {
	GDCLASS(TrafficSystem, Node3D)

public:
	enum SignalPhase {
		PHASE_EW_GREEN,
		PHASE_EW_YELLOW,
		PHASE_NS_GREEN,
		PHASE_NS_YELLOW,
	};

private:
	struct SignalState {
		bool controlled = false;
		float timer = 0.0f;
		uint8_t phase = PHASE_EW_GREEN;
		uint8_t last_drawn_phase = 255;
	};

	// One signal head, mounted on the near-side right of an approach.
	struct SignalHead {
		int junction = -1;
		bool east_west = true;
		Transform3D transform;
	};

	struct Vehicle {
		int from = -1;
		int to = -1;
		int segment = -1;
		double s = 0.0; // metres travelled along the current segment
		double speed = 0.0;
		double desired_speed = 12.0;
		double length = 4.4;
		PackedInt32Array route;
		int route_index = 0;
		Color color;
	};

	// --- Inspector-exposed parameters -------------------------------------
	NodePath city_path;
	int vehicle_count = 600;
	int random_seed = 20260824;
	double lane_width = 3.2;
	double green_time = 11.0;
	double yellow_time = 3.0;
	double time_scale = 1.0;
	bool show_signal_heads = true;

	// --- Simulation state --------------------------------------------------
	Ref<RoadNetwork> network;
	Ref<RandomNumberGenerator> rng;
	std::vector<SignalState> signals;
	std::vector<SignalHead> signal_heads;
	std::vector<double> junction_radius;
	std::vector<Vehicle> vehicles;

	// Rebuilt every step: vehicle indices per (segment, direction), ordered by
	// distance along the segment so the leader is simply the next entry.
	std::vector<std::vector<int>> lane_buckets;

	MultiMeshInstance3D *vehicle_node = nullptr;
	MultiMeshInstance3D *head_node = nullptr;
	Ref<MultiMesh> vehicle_multimesh;
	Ref<MultiMesh> head_multimesh;

	// Guards against restart() -> generate() -> city_generated -> restart().
	bool rebuilding = false;

	void _resolve_network();
	void _on_city_generated();
	void _build_signals();
	void _spawn_vehicles();
	void _build_visuals();
	void _step_signals(double p_delta);
	void _step_vehicles(double p_delta);
	void _sync_transforms();
	void _sync_signal_colors();

	bool _approach_is_open(int p_junction, bool p_east_west) const;
	double _stop_line_distance(int p_junction) const;
	void _assign_new_route(Vehicle &r_vehicle);
	void _advance_to_next_segment(Vehicle &r_vehicle);

protected:
	static void _bind_methods();

public:
	TrafficSystem();

	void _ready() override;
	void _process(double p_delta) override;

	// Rebuilds signals and vehicles against the current city.
	void restart();

	int get_active_vehicle_count() const { return (int)vehicles.size(); }
	double get_average_speed() const;

	void set_city_path(const NodePath &p_value);
	NodePath get_city_path() const { return city_path; }
	void set_vehicle_count(int p_value);
	int get_vehicle_count() const { return vehicle_count; }
	void set_random_seed(int p_value);
	int get_random_seed() const { return random_seed; }
	void set_lane_width(double p_value);
	double get_lane_width() const { return lane_width; }
	void set_green_time(double p_value);
	double get_green_time() const { return green_time; }
	void set_yellow_time(double p_value);
	double get_yellow_time() const { return yellow_time; }
	void set_time_scale(double p_value);
	double get_time_scale() const { return time_scale; }
	void set_show_signal_heads(bool p_value);
	bool get_show_signal_heads() const { return show_signal_heads; }
};

} // namespace godot

VARIANT_ENUM_CAST(godot::TrafficSystem::SignalPhase);

#endif // POLISLAB_TRAFFIC_SYSTEM_H
