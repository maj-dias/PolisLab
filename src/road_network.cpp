#include "road_network.h"

#include <godot_cpp/core/class_db.hpp>

#include <cmath>
#include <limits>
#include <queue>

using namespace godot;

void RoadNetwork::_bind_methods() {
	ClassDB::bind_method(D_METHOD("clear"), &RoadNetwork::clear);
	ClassDB::bind_method(D_METHOD("add_junction", "position"), &RoadNetwork::add_junction);
	ClassDB::bind_method(D_METHOD("add_segment", "a", "b", "lanes", "speed_limit", "road_class", "width"),
			&RoadNetwork::add_segment, DEFVAL(1), DEFVAL(13.9), DEFVAL(ROAD_STREET), DEFVAL(9.0));

	ClassDB::bind_method(D_METHOD("get_junction_count"), &RoadNetwork::get_junction_count);
	ClassDB::bind_method(D_METHOD("get_junction_position", "index"), &RoadNetwork::get_junction_position);
	ClassDB::bind_method(D_METHOD("get_junction_segments", "index"), &RoadNetwork::get_junction_segments);
	ClassDB::bind_method(D_METHOD("get_junction_neighbors", "index"), &RoadNetwork::get_junction_neighbors);

	ClassDB::bind_method(D_METHOD("get_segment_count"), &RoadNetwork::get_segment_count);
	ClassDB::bind_method(D_METHOD("get_segment_endpoints", "index"), &RoadNetwork::get_segment_endpoints);
	ClassDB::bind_method(D_METHOD("get_segment_length", "index"), &RoadNetwork::get_segment_length);
	ClassDB::bind_method(D_METHOD("get_segment_lanes", "index"), &RoadNetwork::get_segment_lanes);
	ClassDB::bind_method(D_METHOD("get_segment_speed_limit", "index"), &RoadNetwork::get_segment_speed_limit);
	ClassDB::bind_method(D_METHOD("get_segment_road_class", "index"), &RoadNetwork::get_segment_road_class);
	ClassDB::bind_method(D_METHOD("get_segment_width", "index"), &RoadNetwork::get_segment_width);

	ClassDB::bind_method(D_METHOD("find_segment", "a", "b"), &RoadNetwork::find_segment);
	ClassDB::bind_method(D_METHOD("nearest_junction", "position"), &RoadNetwork::nearest_junction);
	ClassDB::bind_method(D_METHOD("find_route", "from", "to"), &RoadNetwork::find_route);
	ClassDB::bind_method(D_METHOD("route_to_points", "route"), &RoadNetwork::route_to_points);
	ClassDB::bind_method(D_METHOD("route_travel_time", "route"), &RoadNetwork::route_travel_time);
	ClassDB::bind_method(D_METHOD("route_length", "route"), &RoadNetwork::route_length);

	BIND_ENUM_CONSTANT(ROAD_STREET);
	BIND_ENUM_CONSTANT(ROAD_AVENUE);
	BIND_ENUM_CONSTANT(ROAD_HIGHWAY);
}

void RoadNetwork::clear() {
	junction_positions.clear();
	junction_segments.clear();
	segments.clear();
	max_speed_limit = 1.0;
}

int RoadNetwork::add_junction(const Vector3 &p_position) {
	junction_positions.push_back(p_position);
	junction_segments.push_back(std::vector<int>());
	return (int)junction_positions.size() - 1;
}

int RoadNetwork::add_segment(int p_a, int p_b, int p_lanes, double p_speed_limit, int p_road_class, double p_width) {
	const int count = (int)junction_positions.size();
	ERR_FAIL_INDEX_V(p_a, count, -1);
	ERR_FAIL_INDEX_V(p_b, count, -1);
	ERR_FAIL_COND_V_MSG(p_a == p_b, -1, "A segment cannot start and end at the same junction.");

	const int existing = find_segment(p_a, p_b);
	if (existing != -1) {
		return existing;
	}

	Segment segment;
	segment.a = p_a;
	segment.b = p_b;
	segment.length = junction_positions[p_a].distance_to(junction_positions[p_b]);
	segment.lanes = p_lanes < 1 ? 1 : p_lanes;
	segment.speed_limit = p_speed_limit > 0.1 ? p_speed_limit : 0.1;
	segment.road_class = p_road_class;
	segment.width = p_width > 1.0 ? p_width : 1.0;

	const int index = (int)segments.size();
	segments.push_back(segment);
	junction_segments[p_a].push_back(index);
	junction_segments[p_b].push_back(index);

	if (segment.speed_limit > max_speed_limit) {
		max_speed_limit = segment.speed_limit;
	}
	return index;
}

int RoadNetwork::get_junction_count() const {
	return (int)junction_positions.size();
}

Vector3 RoadNetwork::get_junction_position(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, (int)junction_positions.size(), Vector3());
	return junction_positions[p_index];
}

PackedInt32Array RoadNetwork::get_junction_segments(int p_index) const {
	PackedInt32Array result;
	ERR_FAIL_INDEX_V(p_index, (int)junction_segments.size(), result);
	for (int segment : junction_segments[p_index]) {
		result.push_back(segment);
	}
	return result;
}

PackedInt32Array RoadNetwork::get_junction_neighbors(int p_index) const {
	PackedInt32Array result;
	ERR_FAIL_INDEX_V(p_index, (int)junction_segments.size(), result);
	for (int segment : junction_segments[p_index]) {
		const Segment &s = segments[segment];
		result.push_back(s.a == p_index ? s.b : s.a);
	}
	return result;
}

int RoadNetwork::get_segment_count() const {
	return (int)segments.size();
}

Vector2i RoadNetwork::get_segment_endpoints(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, (int)segments.size(), Vector2i(-1, -1));
	return Vector2i(segments[p_index].a, segments[p_index].b);
}

double RoadNetwork::get_segment_length(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, (int)segments.size(), 0.0);
	return segments[p_index].length;
}

int RoadNetwork::get_segment_lanes(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, (int)segments.size(), 0);
	return segments[p_index].lanes;
}

double RoadNetwork::get_segment_speed_limit(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, (int)segments.size(), 0.0);
	return segments[p_index].speed_limit;
}

int RoadNetwork::get_segment_road_class(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, (int)segments.size(), 0);
	return segments[p_index].road_class;
}

double RoadNetwork::get_segment_width(int p_index) const {
	ERR_FAIL_INDEX_V(p_index, (int)segments.size(), 0.0);
	return segments[p_index].width;
}

int RoadNetwork::find_segment(int p_a, int p_b) const {
	if (p_a < 0 || p_a >= (int)junction_segments.size()) {
		return -1;
	}
	for (int segment : junction_segments[p_a]) {
		const Segment &s = segments[segment];
		if ((s.a == p_a && s.b == p_b) || (s.b == p_a && s.a == p_b)) {
			return segment;
		}
	}
	return -1;
}

int RoadNetwork::nearest_junction(const Vector3 &p_position) const {
	int best = -1;
	double best_distance = std::numeric_limits<double>::infinity();
	for (int i = 0; i < (int)junction_positions.size(); i++) {
		const double distance = junction_positions[i].distance_squared_to(p_position);
		if (distance < best_distance) {
			best_distance = distance;
			best = i;
		}
	}
	return best;
}

namespace {

struct FrontierEntry {
	double estimated_total = 0.0;
	int junction = -1;

	// std::priority_queue is a max-heap, so invert the comparison.
	bool operator<(const FrontierEntry &p_other) const {
		return estimated_total > p_other.estimated_total;
	}
};

} // namespace

PackedInt32Array RoadNetwork::find_route(int p_from, int p_to) const {
	PackedInt32Array result;
	const int count = (int)junction_positions.size();
	if (p_from < 0 || p_from >= count || p_to < 0 || p_to >= count) {
		return result;
	}
	if (p_from == p_to) {
		result.push_back(p_from);
		return result;
	}

	const double unreachable = std::numeric_limits<double>::infinity();
	std::vector<double> cost_so_far(count, unreachable);
	std::vector<int> came_from(count, -1);
	std::vector<bool> settled(count, false);

	const Vector3 goal = junction_positions[p_to];
	auto heuristic = [&](int p_junction) {
		return junction_positions[p_junction].distance_to(goal) / max_speed_limit;
	};

	std::priority_queue<FrontierEntry> frontier;
	cost_so_far[p_from] = 0.0;
	frontier.push({ heuristic(p_from), p_from });

	while (!frontier.empty()) {
		const FrontierEntry current = frontier.top();
		frontier.pop();

		if (settled[current.junction]) {
			continue;
		}
		settled[current.junction] = true;

		if (current.junction == p_to) {
			break;
		}

		for (int segment_index : junction_segments[current.junction]) {
			const Segment &segment = segments[segment_index];
			const int next = segment.a == current.junction ? segment.b : segment.a;
			if (settled[next]) {
				continue;
			}
			const double candidate = cost_so_far[current.junction] + segment.length / segment.speed_limit;
			if (candidate < cost_so_far[next]) {
				cost_so_far[next] = candidate;
				came_from[next] = current.junction;
				frontier.push({ candidate + heuristic(next), next });
			}
		}
	}

	if (cost_so_far[p_to] == unreachable) {
		return result;
	}

	// Walk the parent chain backwards, then flip it into travel order.
	for (int at = p_to; at != -1; at = came_from[at]) {
		result.push_back(at);
	}
	result.reverse();
	return result;
}

PackedVector3Array RoadNetwork::route_to_points(const PackedInt32Array &p_route) const {
	PackedVector3Array points;
	for (int i = 0; i < p_route.size(); i++) {
		const int junction = p_route[i];
		if (junction < 0 || junction >= (int)junction_positions.size()) {
			continue;
		}
		points.push_back(junction_positions[junction]);
	}
	return points;
}

double RoadNetwork::route_travel_time(const PackedInt32Array &p_route) const {
	double total = 0.0;
	for (int i = 0; i + 1 < p_route.size(); i++) {
		const int segment = find_segment(p_route[i], p_route[i + 1]);
		if (segment == -1) {
			continue;
		}
		total += segments[segment].length / segments[segment].speed_limit;
	}
	return total;
}

double RoadNetwork::route_length(const PackedInt32Array &p_route) const {
	double total = 0.0;
	for (int i = 0; i + 1 < p_route.size(); i++) {
		const int segment = find_segment(p_route[i], p_route[i + 1]);
		if (segment == -1) {
			continue;
		}
		total += segments[segment].length;
	}
	return total;
}
