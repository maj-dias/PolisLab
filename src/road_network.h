#ifndef POLISLAB_ROAD_NETWORK_H
#define POLISLAB_ROAD_NETWORK_H

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/vector2i.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include <vector>

namespace godot {

// Directed-capable road graph: junctions are nodes, segments are edges.
// This is the piece the routing/dispatch experiments (Uber/iFood style) build
// on, so it is deliberately independent from any of the visual generation.
class RoadNetwork : public RefCounted {
	GDCLASS(RoadNetwork, RefCounted)

public:
	enum RoadClass {
		ROAD_STREET,
		ROAD_AVENUE,
		ROAD_HIGHWAY,
	};

	struct Segment {
		int a = -1;
		int b = -1;
		double length = 0.0;
		int lanes = 1;
		double speed_limit = 13.9; // m/s (~50 km/h)
		int road_class = ROAD_STREET;
		double width = 9.0; // full carriageway width, both directions
	};

private:
	std::vector<Vector3> junction_positions;
	std::vector<std::vector<int>> junction_segments;
	std::vector<Segment> segments;
	double max_speed_limit = 1.0;

protected:
	static void _bind_methods();

public:
	void clear();

	int add_junction(const Vector3 &p_position);
	int add_segment(int p_a, int p_b, int p_lanes, double p_speed_limit, int p_road_class, double p_width);

	int get_junction_count() const;
	Vector3 get_junction_position(int p_index) const;
	PackedInt32Array get_junction_segments(int p_index) const;
	PackedInt32Array get_junction_neighbors(int p_index) const;

	int get_segment_count() const;
	Vector2i get_segment_endpoints(int p_index) const;
	double get_segment_length(int p_index) const;
	int get_segment_lanes(int p_index) const;
	double get_segment_speed_limit(int p_index) const;
	int get_segment_road_class(int p_index) const;
	double get_segment_width(int p_index) const;

	int find_segment(int p_a, int p_b) const;
	int nearest_junction(const Vector3 &p_position) const;

	// A* over travel time (length / speed limit), which is what a dispatcher
	// actually optimises for -- not raw distance.
	PackedInt32Array find_route(int p_from, int p_to) const;
	PackedVector3Array route_to_points(const PackedInt32Array &p_route) const;
	double route_travel_time(const PackedInt32Array &p_route) const;
	double route_length(const PackedInt32Array &p_route) const;

	// Native-side accessors used by the simulation classes.
	const Segment &segment_at(int p_index) const { return segments[p_index]; }
	const Vector3 &junction_at(int p_index) const { return junction_positions[p_index]; }
	int junction_count_fast() const { return (int)junction_positions.size(); }
	int segment_count_fast() const { return (int)segments.size(); }
	const std::vector<int> &segments_of(int p_index) const { return junction_segments[p_index]; }
};

} // namespace godot

VARIANT_ENUM_CAST(godot::RoadNetwork::RoadClass);

#endif // POLISLAB_ROAD_NETWORK_H
