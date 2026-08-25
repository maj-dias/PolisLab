#include "city_generator.h"

#include "mesh_builder.h"

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/box_mesh.hpp>
#include <godot_cpp/classes/cylinder_mesh.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/multi_mesh.hpp>
#include <godot_cpp/classes/sphere_mesh.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>

using namespace godot;

namespace {

constexpr double GROUND_Y = -0.06;
constexpr double ROAD_Y = 0.0;
constexpr double ROAD_CROSS_Y = 0.012;
constexpr double MARKING_Y = 0.03;
constexpr double KERB_Y = 0.16;

const Color COLOR_GROUND(0.22, 0.32, 0.17);
const Color COLOR_ASPHALT(0.113, 0.117, 0.129);
const Color COLOR_ASPHALT_AVENUE(0.130, 0.134, 0.148);
const Color COLOR_ASPHALT_HIGHWAY(0.148, 0.150, 0.162);
const Color COLOR_MARKING(0.86, 0.85, 0.74);
const Color COLOR_CROSSWALK(0.88, 0.89, 0.90);
const Color COLOR_KERB_TOP(0.52, 0.52, 0.50);
const Color COLOR_KERB_SIDE(0.40, 0.40, 0.39);
const Color COLOR_PARK(0.24, 0.44, 0.22);

// Stable per-cell noise so zoning stays identical between runs with the same
// seed without having to consume the RNG in a fixed order.
double hash01(int p_a, int p_b, int p_seed) {
	uint32_t h = (uint32_t)(p_a * 374761393) + (uint32_t)(p_b * 668265263) + (uint32_t)p_seed;
	h = (h ^ (h >> 13)) * 1274126177u;
	h ^= h >> 16;
	return (double)h / (double)UINT32_MAX;
}

Ref<StandardMaterial3D> make_material(double p_roughness, double p_metallic) {
	Ref<StandardMaterial3D> material;
	material.instantiate();
	material->set_flag(BaseMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
	material->set_roughness(p_roughness);
	material->set_metallic(p_metallic);
	return material;
}

// A unit gabled roof: 1x1 footprint centred on the origin, ridge running along
// X at y = 1. Instances scale it to whatever house they cap.
Ref<ArrayMesh> make_roof_mesh() {
	MeshBuilder builder;
	const Vector3 a(-0.5, 0.0, -0.5);
	const Vector3 b(0.5, 0.0, -0.5);
	const Vector3 c(0.5, 0.0, 0.5);
	const Vector3 d(-0.5, 0.0, 0.5);
	const Vector3 ridge_min(-0.5, 1.0, 0.0);
	const Vector3 ridge_max(0.5, 1.0, 0.0);
	const Color white(1, 1, 1);

	const Vector3 slope_north = Vector3(0, 0.5, -1).normalized();
	const Vector3 slope_south = Vector3(0, 0.5, 1).normalized();
	builder.add_quad(a, b, ridge_max, ridge_min, slope_north, white, 1.0);
	builder.add_quad(c, d, ridge_min, ridge_max, slope_south, white, 1.0);

	const int gable_a = builder.add_vertex(a, Vector3(-1, 0, 0), Vector2(0, 0), white);
	const int gable_e = builder.add_vertex(ridge_min, Vector3(-1, 0, 0), Vector2(0.5, 1), white);
	const int gable_d = builder.add_vertex(d, Vector3(-1, 0, 0), Vector2(1, 0), white);
	builder.add_triangle(gable_a, gable_e, gable_d);

	const int gable_b = builder.add_vertex(b, Vector3(1, 0, 0), Vector2(0, 0), white);
	const int gable_c = builder.add_vertex(c, Vector3(1, 0, 0), Vector2(1, 0), white);
	const int gable_f = builder.add_vertex(ridge_max, Vector3(1, 0, 0), Vector2(0.5, 1), white);
	builder.add_triangle(gable_b, gable_c, gable_f);

	Ref<ArrayMesh> mesh;
	mesh.instantiate();
	builder.commit_to(mesh, Ref<Material>());
	return mesh;
}

Transform3D box_transform(const Vector2 &p_center, const Vector2 &p_half, double p_bottom, double p_height) {
	Transform3D transform;
	transform.basis = Basis().scaled(Vector3(p_half.x * 2.0, p_height, p_half.y * 2.0));
	transform.origin = Vector3(p_center.x, p_bottom + p_height * 0.5, p_center.y);
	return transform;
}

} // namespace

CityGenerator::CityGenerator() {
	rng.instantiate();
}

void CityGenerator::_bind_methods() {
	ClassDB::bind_method(D_METHOD("generate"), &CityGenerator::generate);
	ClassDB::bind_method(D_METHOD("get_network"), &CityGenerator::get_network);
	ClassDB::bind_method(D_METHOD("get_city_extent"), &CityGenerator::get_city_extent);

#define POLIS_PROPERTY(m_type, m_name, m_hint, m_hint_string)                                     \
	ClassDB::bind_method(D_METHOD("set_" #m_name, "value"), &CityGenerator::set_##m_name);        \
	ClassDB::bind_method(D_METHOD("get_" #m_name), &CityGenerator::get_##m_name);                 \
	ADD_PROPERTY(PropertyInfo(m_type, #m_name, m_hint, m_hint_string), "set_" #m_name, "get_" #m_name);

	ADD_GROUP("Layout", "");
	POLIS_PROPERTY(Variant::INT, city_seed, PROPERTY_HINT_NONE, "")
	POLIS_PROPERTY(Variant::INT, blocks_x, PROPERTY_HINT_RANGE, "2,60,1")
	POLIS_PROPERTY(Variant::INT, blocks_z, PROPERTY_HINT_RANGE, "2,60,1")
	POLIS_PROPERTY(Variant::FLOAT, block_size, PROPERTY_HINT_RANGE, "20,200,0.5")
	POLIS_PROPERTY(Variant::FLOAT, street_width, PROPERTY_HINT_RANGE, "5,30,0.5")
	POLIS_PROPERTY(Variant::FLOAT, avenue_width, PROPERTY_HINT_RANGE, "6,40,0.5")
	POLIS_PROPERTY(Variant::INT, avenue_every, PROPERTY_HINT_RANGE, "2,12,1")

	ADD_GROUP("Highway", "");
	POLIS_PROPERTY(Variant::BOOL, build_highway_ring, PROPERTY_HINT_NONE, "")
	POLIS_PROPERTY(Variant::FLOAT, highway_width, PROPERTY_HINT_RANGE, "10,60,0.5")
	POLIS_PROPERTY(Variant::FLOAT, highway_margin, PROPERTY_HINT_RANGE, "20,400,1")

	ADD_GROUP("Zoning", "");
	POLIS_PROPERTY(Variant::FLOAT, downtown_ratio, PROPERTY_HINT_RANGE, "0,1,0.01")
	POLIS_PROPERTY(Variant::FLOAT, midtown_ratio, PROPERTY_HINT_RANGE, "0,1,0.01")
	POLIS_PROPERTY(Variant::FLOAT, park_chance, PROPERTY_HINT_RANGE, "0,0.5,0.01")

	ADD_GROUP("", "");
	POLIS_PROPERTY(Variant::BOOL, generate_on_ready, PROPERTY_HINT_NONE, "")
#undef POLIS_PROPERTY

	ADD_SIGNAL(MethodInfo("city_generated"));

	BIND_ENUM_CONSTANT(ZONE_DOWNTOWN);
	BIND_ENUM_CONSTANT(ZONE_MIDTOWN);
	BIND_ENUM_CONSTANT(ZONE_SUBURB);
	BIND_ENUM_CONSTANT(ZONE_PARK);
}

void CityGenerator::_ready() {
	// A TrafficSystem that readies before us may already have asked for the
	// city; regenerating here would hand it a graph it no longer points at.
	if (generate_on_ready && network.is_null()) {
		generate();
	}
}

void CityGenerator::_clear_generated() {
	TypedArray<Node> children = get_children();
	for (int i = 0; i < children.size(); i++) {
		Node *child = Object::cast_to<Node>(children[i]);
		if (child) {
			remove_child(child);
			child->queue_free();
		}
	}

	concrete_instances.clear();
	concrete_colors.clear();
	glass_instances.clear();
	glass_colors.clear();
	roof_instances.clear();
	roof_colors.clear();
	trunk_instances.clear();
	canopy_instances.clear();
	canopy_colors.clear();
	axis_x.clear();
	axis_z.clear();
	grid_junctions.clear();
}

void CityGenerator::_build_axes() {
	auto build = [&](int p_count, std::vector<Axis> &r_axes) {
		double cursor = 0.0;
		for (int i = 0; i <= p_count; i++) {
			Axis axis;
			const bool is_avenue = (i % avenue_every) == 0;
			axis.width = is_avenue ? avenue_width : street_width;
			axis.road_class = is_avenue ? RoadNetwork::ROAD_AVENUE : RoadNetwork::ROAD_STREET;
			axis.center = cursor + axis.width * 0.5;
			r_axes.push_back(axis);

			// Vary block depth a little so the grid never reads as graph paper.
			const double jitter = 1.0 + rng->randf_range(-0.16, 0.16);
			cursor = axis.center + axis.width * 0.5 + block_size * jitter;
		}

		// Re-centre the axis around the origin.
		const double span = r_axes.back().center - r_axes.front().center;
		const double offset = r_axes.front().center + span * 0.5;
		for (Axis &axis : r_axes) {
			axis.center -= offset;
		}
	};

	build(blocks_x, axis_x);
	build(blocks_z, axis_z);

	city_extent = Vector2(
			(real_t)(axis_x.back().center - axis_x.front().center),
			(real_t)(axis_z.back().center - axis_z.front().center));
}

void CityGenerator::_build_network() {
	grid_junctions.assign(axis_x.size(), std::vector<int>(axis_z.size(), -1));

	for (size_t i = 0; i < axis_x.size(); i++) {
		for (size_t j = 0; j < axis_z.size(); j++) {
			grid_junctions[i][j] = network->add_junction(
					Vector3((real_t)axis_x[i].center, 0.0f, (real_t)axis_z[j].center));
		}
	}

	// A segment running along X sits on a Z axis line, so it inherits that
	// line's class -- and vice versa.
	auto describe = [](int p_road_class, int &r_lanes, double &r_speed) {
		switch (p_road_class) {
			case RoadNetwork::ROAD_AVENUE:
				r_lanes = 2;
				r_speed = 16.6; // ~60 km/h
				break;
			case RoadNetwork::ROAD_HIGHWAY:
				r_lanes = 3;
				r_speed = 27.7; // ~100 km/h
				break;
			default:
				r_lanes = 1;
				r_speed = 11.1; // ~40 km/h
				break;
		}
	};

	for (size_t j = 0; j < axis_z.size(); j++) {
		int lanes = 1;
		double speed = 11.1;
		describe(axis_z[j].road_class, lanes, speed);
		for (size_t i = 0; i + 1 < axis_x.size(); i++) {
			network->add_segment(grid_junctions[i][j], grid_junctions[i + 1][j], lanes, speed, axis_z[j].road_class,
					axis_z[j].width);
		}
	}

	for (size_t i = 0; i < axis_x.size(); i++) {
		int lanes = 1;
		double speed = 11.1;
		describe(axis_x[i].road_class, lanes, speed);
		for (size_t j = 0; j + 1 < axis_z.size(); j++) {
			network->add_segment(grid_junctions[i][j], grid_junctions[i][j + 1], lanes, speed, axis_x[i].road_class,
					axis_x[i].width);
		}
	}
}

void CityGenerator::_build_highway_ring() {
	const double x0 = axis_x.front().center - highway_margin;
	const double x1 = axis_x.back().center + highway_margin;
	const double z0 = axis_z.front().center - highway_margin;
	const double z1 = axis_z.back().center + highway_margin;

	const int corner_nw = network->add_junction(Vector3((real_t)x0, 0, (real_t)z0));
	const int corner_ne = network->add_junction(Vector3((real_t)x1, 0, (real_t)z0));
	const int corner_se = network->add_junction(Vector3((real_t)x1, 0, (real_t)z1));
	const int corner_sw = network->add_junction(Vector3((real_t)x0, 0, (real_t)z1));

	// Each side gets an on/off ramp wherever an avenue reaches the boundary.
	auto build_side = [&](int p_start, int p_end, bool p_along_x, double p_fixed, bool p_at_low_edge) {
		const std::vector<Axis> &axes = p_along_x ? axis_x : axis_z;
		std::vector<int> chain;
		chain.push_back(p_start);

		for (size_t k = 1; k + 1 < axes.size(); k++) {
			if (axes[k].road_class != RoadNetwork::ROAD_AVENUE) {
				continue;
			}
			const Vector3 position = p_along_x
					? Vector3((real_t)axes[k].center, 0, (real_t)p_fixed)
					: Vector3((real_t)p_fixed, 0, (real_t)axes[k].center);
			const int ramp_top = network->add_junction(position);
			chain.push_back(ramp_top);

			const int boundary = p_along_x
					? grid_junctions[k][p_at_low_edge ? 0 : axis_z.size() - 1]
					: grid_junctions[p_at_low_edge ? 0 : axis_x.size() - 1][k];
			network->add_segment(ramp_top, boundary, 2, 22.2, RoadNetwork::ROAD_HIGHWAY, highway_width);
		}

		chain.push_back(p_end);
		for (size_t k = 0; k + 1 < chain.size(); k++) {
			network->add_segment(chain[k], chain[k + 1], 3, 27.7, RoadNetwork::ROAD_HIGHWAY, highway_width);
		}
	};

	build_side(corner_nw, corner_ne, true, z0, true);
	build_side(corner_sw, corner_se, true, z1, false);
	build_side(corner_nw, corner_sw, false, x0, true);
	build_side(corner_ne, corner_se, false, x1, false);
}

void CityGenerator::_build_road_meshes(MeshBuilder &r_roads, MeshBuilder &r_markings, MeshBuilder &r_ground) {
	const double x_min = axis_x.front().center - axis_x.front().width * 0.5;
	const double x_max = axis_x.back().center + axis_x.back().width * 0.5;
	const double z_min = axis_z.front().center - axis_z.front().width * 0.5;
	const double z_max = axis_z.back().center + axis_z.back().width * 0.5;

	const double ground_margin = highway_margin + 1500.0;
	r_ground.add_flat_rect(Vector2(0, 0),
			Vector2((real_t)((x_max - x_min) * 0.5 + ground_margin), (real_t)((z_max - z_min) * 0.5 + ground_margin)),
			(real_t)GROUND_Y, COLOR_GROUND, 0.08);

	// Carriageways are drawn as full-length strips rather than per-segment
	// quads: fewer triangles and no seams at the intersections. The two
	// directions sit at slightly different heights to avoid z-fighting where
	// they overlap.
	for (const Axis &axis : axis_z) {
		const Color color = axis.road_class == RoadNetwork::ROAD_AVENUE ? COLOR_ASPHALT_AVENUE : COLOR_ASPHALT;
		r_roads.add_flat_rect(Vector2((real_t)((x_min + x_max) * 0.5), (real_t)axis.center),
				Vector2((real_t)((x_max - x_min) * 0.5), (real_t)(axis.width * 0.5)), (real_t)ROAD_Y, color, 0.12);
	}
	for (const Axis &axis : axis_x) {
		const Color color = axis.road_class == RoadNetwork::ROAD_AVENUE ? COLOR_ASPHALT_AVENUE : COLOR_ASPHALT;
		r_roads.add_flat_rect(Vector2((real_t)axis.center, (real_t)((z_min + z_max) * 0.5)),
				Vector2((real_t)(axis.width * 0.5), (real_t)((z_max - z_min) * 0.5)), (real_t)ROAD_CROSS_Y, color, 0.12);
	}

	// Highway ring and ramps: generic oriented strips, one per segment.
	for (int s = 0; s < network->segment_count_fast(); s++) {
		const RoadNetwork::Segment &segment = network->segment_at(s);
		if (segment.road_class != RoadNetwork::ROAD_HIGHWAY) {
			continue;
		}
		r_roads.add_strip(network->junction_at(segment.a), network->junction_at(segment.b),
				(real_t)(highway_width * 0.5), (real_t)ROAD_CROSS_Y, COLOR_ASPHALT_HIGHWAY, 0.12);
	}

	// Lane markings, drawn only in the gaps between intersections.
	auto dashes = [&](double p_from, double p_to, double p_fixed, bool p_along_x, double p_half_thickness) {
		const double dash = 3.2;
		const double gap = 3.0;
		for (double t = p_from; t < p_to; t += dash + gap) {
			const double end = std::min(t + dash, p_to);
			if (end - t < 0.6) {
				break;
			}
			const Vector2 center = p_along_x
					? Vector2((real_t)((t + end) * 0.5), (real_t)p_fixed)
					: Vector2((real_t)p_fixed, (real_t)((t + end) * 0.5));
			const Vector2 half = p_along_x
					? Vector2((real_t)((end - t) * 0.5), (real_t)p_half_thickness)
					: Vector2((real_t)p_half_thickness, (real_t)((end - t) * 0.5));
			r_markings.add_flat_rect(center, half, (real_t)MARKING_Y, COLOR_MARKING, 1.0);
		}
	};

	for (size_t j = 0; j < axis_z.size(); j++) {
		for (size_t i = 0; i + 1 < axis_x.size(); i++) {
			const double from = axis_x[i].center + axis_x[i].width * 0.5 + 1.2;
			const double to = axis_x[i + 1].center - axis_x[i + 1].width * 0.5 - 1.2;
			dashes(from, to, axis_z[j].center, true, 0.17);
		}
	}
	for (size_t i = 0; i < axis_x.size(); i++) {
		for (size_t j = 0; j + 1 < axis_z.size(); j++) {
			const double from = axis_z[j].center + axis_z[j].width * 0.5 + 1.2;
			const double to = axis_z[j + 1].center - axis_z[j + 1].width * 0.5 - 1.2;
			dashes(from, to, axis_x[i].center, false, 0.17);
		}
	}

	// Zebra crossings on every intersection approach. The bars run across the
	// carriageway and repeat along the direction of travel.
	auto crosswalk = [&](double p_band_start, double p_band_dir, double p_lane_center, double p_lane_half,
							 bool p_bars_along_z) {
		const double bar = 0.62;
		const double bar_gap = 0.62;
		for (int b = 0; b < 3; b++) {
			const double offset = p_band_start + p_band_dir * (b * (bar + bar_gap) + bar * 0.5);
			const Vector2 center = p_bars_along_z
					? Vector2((real_t)offset, (real_t)p_lane_center)
					: Vector2((real_t)p_lane_center, (real_t)offset);
			const Vector2 half = p_bars_along_z
					? Vector2((real_t)(bar * 0.5), (real_t)(p_lane_half - 0.35))
					: Vector2((real_t)(p_lane_half - 0.35), (real_t)(bar * 0.5));
			r_markings.add_flat_rect(center, half, (real_t)MARKING_Y, COLOR_CROSSWALK, 1.0);
		}
	};

	for (size_t i = 0; i < axis_x.size(); i++) {
		for (size_t j = 0; j < axis_z.size(); j++) {
			const double cx = axis_x[i].center;
			const double cz = axis_z[j].center;
			const double hx = axis_x[i].width * 0.5;
			const double hz = axis_z[j].width * 0.5;

			if (i + 1 < axis_x.size()) {
				crosswalk(cx + hx + 0.9, 1.0, cz, hz, true);
			}
			if (i > 0) {
				crosswalk(cx - hx - 0.9, -1.0, cz, hz, true);
			}
			if (j + 1 < axis_z.size()) {
				crosswalk(cz + hz + 0.9, 1.0, cx, hx, false);
			}
			if (j > 0) {
				crosswalk(cz - hz - 0.9, -1.0, cx, hx, false);
			}
		}
	}
}

CityGenerator::ZoneKind CityGenerator::_zone_for_block(int p_bx, int p_bz) const {
	if (hash01(p_bx, p_bz, city_seed + 7717) < park_chance) {
		return ZONE_PARK;
	}

	const double cx = std::max((blocks_x - 1) * 0.5, 0.5);
	const double cz = std::max((blocks_z - 1) * 0.5, 0.5);
	const double nx = (p_bx - cx) / cx;
	const double nz = (p_bz - cz) / cz;
	// Jitter the radius so the zoning rings have ragged, organic borders.
	const double radius = std::sqrt(nx * nx + nz * nz) + (hash01(p_bx, p_bz, city_seed) - 0.5) * 0.24;

	if (radius < downtown_ratio) {
		return ZONE_DOWNTOWN;
	}
	if (radius < midtown_ratio) {
		return ZONE_MIDTOWN;
	}
	return ZONE_SUBURB;
}

void CityGenerator::_build_blocks(MeshBuilder &r_sidewalks, MeshBuilder &r_parks) {
	const double sidewalk_inset = 3.0;

	for (int bx = 0; bx < blocks_x; bx++) {
		for (int bz = 0; bz < blocks_z; bz++) {
			const double left = axis_x[bx].center + axis_x[bx].width * 0.5;
			const double right = axis_x[bx + 1].center - axis_x[bx + 1].width * 0.5;
			const double top = axis_z[bz].center + axis_z[bz].width * 0.5;
			const double bottom = axis_z[bz + 1].center - axis_z[bz + 1].width * 0.5;
			if (right - left < 6.0 || bottom - top < 6.0) {
				continue;
			}

			const Vector2 center((real_t)((left + right) * 0.5), (real_t)((top + bottom) * 0.5));
			const Vector2 half((real_t)((right - left) * 0.5), (real_t)((bottom - top) * 0.5));

			// Raised sidewalk slab covering the whole block.
			r_sidewalks.add_slab(center, half, (real_t)GROUND_Y, (real_t)KERB_Y, COLOR_KERB_TOP, COLOR_KERB_SIDE, 0.3);

			const Vector2 lot_half(std::max(half.x - (real_t)sidewalk_inset, (real_t)2.0),
					std::max(half.y - (real_t)sidewalk_inset, (real_t)2.0));

			switch (_zone_for_block(bx, bz)) {
				case ZONE_DOWNTOWN:
					_emit_downtown(center, lot_half);
					break;
				case ZONE_MIDTOWN:
					_emit_midtown(center, lot_half);
					break;
				case ZONE_SUBURB:
					_emit_suburb(center, lot_half);
					break;
				case ZONE_PARK:
					_emit_park(center, lot_half, r_parks);
					break;
			}
		}
	}
}

void CityGenerator::_emit_building(const Vector2 &p_center, const Vector2 &p_half, double p_height, bool p_glass) {
	if (p_half.x < 1.0 || p_half.y < 1.0 || p_height < 1.0) {
		return;
	}
	const Transform3D transform = box_transform(p_center, p_half, KERB_Y, p_height);

	if (p_glass) {
		const double tint = rng->randf_range(0.0, 1.0);
		const Color color(
				(real_t)(0.18 + tint * 0.16),
				(real_t)(0.30 + tint * 0.22),
				(real_t)(0.40 + tint * 0.26));
		glass_instances.push_back(transform);
		glass_colors.push_back(color);
	} else {
		const double shade = rng->randf_range(0.45, 0.82);
		const double warmth = rng->randf_range(0.0, 0.12);
		const Color color((real_t)(shade + warmth), (real_t)(shade + warmth * 0.5), (real_t)(shade * 0.94));
		concrete_instances.push_back(transform);
		concrete_colors.push_back(color);
	}
}

void CityGenerator::_emit_downtown(const Vector2 &p_center, const Vector2 &p_half) {
	// Downtown blocks carry one or two towers plus a low podium.
	const int towers = rng->randi_range(1, 2);
	for (int t = 0; t < towers; t++) {
		const real_t footprint_x = p_half.x * (real_t)rng->randf_range(0.36, 0.60);
		const real_t footprint_z = p_half.y * (real_t)rng->randf_range(0.36, 0.60);
		const real_t offset_x = towers == 1 ? (real_t)0.0 : (t == 0 ? -p_half.x * (real_t)0.42 : p_half.x * (real_t)0.42);
		const Vector2 center(p_center.x + offset_x, p_center.y + (real_t)rng->randf_range(-4.0, 4.0));

		const double height = rng->randf_range(38.0, 118.0);
		const bool glass = rng->randf() < 0.72;
		_emit_building(center, Vector2(footprint_x, footprint_z), height, glass);

		// Setback crown so the skyline is not a field of identical slabs.
		if (rng->randf() < 0.55) {
			const Vector2 crown_half(footprint_x * (real_t)0.62, footprint_z * (real_t)0.62);
			const Transform3D crown = box_transform(center, crown_half, KERB_Y + height, rng->randf_range(6.0, 22.0));
			concrete_instances.push_back(crown);
			concrete_colors.push_back(Color(0.55, 0.56, 0.58));
		}
	}

	// Podium filling the rest of the lot.
	_emit_building(p_center, Vector2(p_half.x * (real_t)0.92, p_half.y * (real_t)0.92), rng->randf_range(7.0, 14.0), false);
}

void CityGenerator::_emit_midtown(const Vector2 &p_center, const Vector2 &p_half) {
	const int columns = rng->randi_range(2, 3);
	const int rows = rng->randi_range(2, 3);
	const real_t cell_x = p_half.x * 2.0f / columns;
	const real_t cell_z = p_half.y * 2.0f / rows;

	for (int cx = 0; cx < columns; cx++) {
		for (int cz = 0; cz < rows; cz++) {
			if (rng->randf() < 0.12) {
				continue; // an empty lot here and there
			}
			const Vector2 center(
					p_center.x - p_half.x + cell_x * (cx + 0.5f),
					p_center.y - p_half.y + cell_z * (cz + 0.5f));
			const Vector2 half(cell_x * (real_t)rng->randf_range(0.34, 0.44),
					cell_z * (real_t)rng->randf_range(0.34, 0.44));
			_emit_building(center, half, rng->randf_range(12.0, 34.0), rng->randf() < 0.22);
		}
	}
}

void CityGenerator::_emit_suburb(const Vector2 &p_center, const Vector2 &p_half) {
	// Houses face the street, so lay them out in rows along the longer axis.
	const bool rows_along_x = p_half.x >= p_half.y;
	const int along = std::max(2, (int)std::round((rows_along_x ? p_half.x : p_half.y) * 2.0 / 13.0));
	const int across = 2;

	const real_t cell_along = (rows_along_x ? p_half.x : p_half.y) * 2.0f / along;
	const real_t cell_across = (rows_along_x ? p_half.y : p_half.x) * 2.0f / across;

	for (int a = 0; a < along; a++) {
		for (int b = 0; b < across; b++) {
			if (rng->randf() < 0.10) {
				continue;
			}
			const real_t pos_along = -(rows_along_x ? p_half.x : p_half.y) + cell_along * (a + 0.5f);
			const real_t pos_across = -(rows_along_x ? p_half.y : p_half.x) + cell_across * (b + 0.5f);
			const Vector2 center = rows_along_x
					? Vector2(p_center.x + pos_along, p_center.y + pos_across)
					: Vector2(p_center.x + pos_across, p_center.y + pos_along);

			const real_t house_along = cell_along * (real_t)rng->randf_range(0.28, 0.38);
			const real_t house_across = cell_across * (real_t)rng->randf_range(0.26, 0.34);
			const Vector2 half = rows_along_x
					? Vector2(house_along, house_across)
					: Vector2(house_across, house_along);

			const double height = rng->randf_range(3.4, 7.2);
			const double wall = rng->randf_range(0.58, 0.88);
			concrete_instances.push_back(box_transform(center, half, KERB_Y, height));
			concrete_colors.push_back(Color((real_t)wall, (real_t)(wall * 0.95), (real_t)(wall * 0.86)));

			// Gabled roof, ridge running along the house's long side.
			Transform3D roof;
			const real_t overhang = 0.5f;
			roof.basis = Basis().scaled(Vector3(
					(half.x + overhang) * 2.0f,
					(real_t)rng->randf_range(1.6, 3.0),
					(half.y + overhang) * 2.0f));
			if (!rows_along_x) {
				roof.basis = roof.basis.rotated(Vector3(0, 1, 0), (real_t)(Math::PI * 0.5));
			}
			roof.origin = Vector3(center.x, (real_t)(KERB_Y + height), center.y);
			roof_instances.push_back(roof);
			const double roof_tone = rng->randf_range(0.0, 1.0);
			roof_colors.push_back(Color(
					(real_t)(0.34 + roof_tone * 0.28),
					(real_t)(0.18 + roof_tone * 0.10),
					(real_t)(0.14 + roof_tone * 0.08)));

			if (rng->randf() < 0.5) {
				_emit_tree(Vector2(center.x + half.x + 2.0f, center.y), rng->randf_range(0.8, 1.3));
			}
		}
	}
}

void CityGenerator::_emit_park(const Vector2 &p_center, const Vector2 &p_half, MeshBuilder &r_parks) {
	r_parks.add_flat_rect(p_center, p_half, (real_t)(KERB_Y + 0.01), COLOR_PARK, 0.2);

	const int trees = (int)std::round(p_half.x * p_half.y / 34.0);
	for (int t = 0; t < trees; t++) {
		const Vector2 position(
				p_center.x + (real_t)rng->randf_range(-p_half.x * 0.88, p_half.x * 0.88),
				p_center.y + (real_t)rng->randf_range(-p_half.y * 0.88, p_half.y * 0.88));
		_emit_tree(position, rng->randf_range(1.0, 1.9));
	}
}

void CityGenerator::_emit_tree(const Vector2 &p_position, double p_scale) {
	const double trunk_height = 2.2 * p_scale;
	Transform3D trunk;
	trunk.basis = Basis().scaled(Vector3((real_t)p_scale, (real_t)trunk_height, (real_t)p_scale));
	trunk.origin = Vector3(p_position.x, (real_t)(KERB_Y + trunk_height * 0.5), p_position.y);
	trunk_instances.push_back(trunk);

	const double canopy_radius = 1.7 * p_scale;
	Transform3D canopy;
	canopy.basis = Basis().scaled(Vector3(
			(real_t)canopy_radius,
			(real_t)(canopy_radius * rng->randf_range(0.9, 1.25)),
			(real_t)canopy_radius));
	canopy.origin = Vector3(p_position.x, (real_t)(KERB_Y + trunk_height + canopy_radius * 0.45), p_position.y);
	canopy_instances.push_back(canopy);

	const double tone = rng->randf_range(0.0, 1.0);
	canopy_colors.push_back(Color((real_t)(0.16 + tone * 0.12), (real_t)(0.36 + tone * 0.20), (real_t)(0.15 + tone * 0.10)));
}

MultiMeshInstance3D *CityGenerator::_make_multimesh(const String &p_name, const Ref<Mesh> &p_mesh,
		const std::vector<Transform3D> &p_transforms, const std::vector<Color> &p_colors) {
	if (p_transforms.empty()) {
		return nullptr;
	}

	Ref<MultiMesh> multimesh;
	multimesh.instantiate();
	multimesh->set_transform_format(MultiMesh::TRANSFORM_3D);
	// use_colors must be configured before the instance count is set.
	multimesh->set_use_colors(!p_colors.empty());
	multimesh->set_mesh(p_mesh);
	multimesh->set_instance_count((int)p_transforms.size());

	for (int i = 0; i < (int)p_transforms.size(); i++) {
		multimesh->set_instance_transform(i, p_transforms[i]);
		if (!p_colors.empty()) {
			multimesh->set_instance_color(i, p_colors[i]);
		}
	}

	MultiMeshInstance3D *instance = memnew(MultiMeshInstance3D);
	instance->set_name(p_name);
	instance->set_multimesh(multimesh);
	add_child(instance);
	return instance;
}

void CityGenerator::generate() {
	_clear_generated();

	network.instantiate();
	rng->set_seed((uint64_t)city_seed);

	_build_axes();
	_build_network();
	if (build_highway_ring) {
		_build_highway_ring();
	}

	MeshBuilder ground_builder;
	MeshBuilder road_builder;
	MeshBuilder marking_builder;
	MeshBuilder sidewalk_builder;
	MeshBuilder park_builder;

	_build_road_meshes(road_builder, marking_builder, ground_builder);
	_build_blocks(sidewalk_builder, park_builder);

	const Ref<StandardMaterial3D> matte = make_material(0.95, 0.0);
	const Ref<StandardMaterial3D> asphalt = make_material(0.78, 0.0);
	const Ref<StandardMaterial3D> paint = make_material(0.55, 0.0);
	const Ref<StandardMaterial3D> glass = make_material(0.16, 0.42);

	Ref<ArrayMesh> terrain;
	terrain.instantiate();
	ground_builder.commit_to(terrain, matte);
	road_builder.commit_to(terrain, asphalt);
	marking_builder.commit_to(terrain, paint);
	sidewalk_builder.commit_to(terrain, matte);
	park_builder.commit_to(terrain, matte);

	MeshInstance3D *terrain_instance = memnew(MeshInstance3D);
	terrain_instance->set_name("Terrain");
	terrain_instance->set_mesh(terrain);
	add_child(terrain_instance);

	Ref<BoxMesh> box;
	box.instantiate();
	box->set_size(Vector3(1, 1, 1));

	Ref<CylinderMesh> trunk;
	trunk.instantiate();
	trunk->set_top_radius(0.16);
	trunk->set_bottom_radius(0.24);
	trunk->set_height(1.0);
	trunk->set_radial_segments(6);
	trunk->set_rings(1);

	Ref<SphereMesh> canopy;
	canopy.instantiate();
	canopy->set_radius(1.0);
	canopy->set_height(2.0);
	canopy->set_radial_segments(8);
	canopy->set_rings(5);

	MultiMeshInstance3D *node = nullptr;
	node = _make_multimesh("Buildings", box, concrete_instances, concrete_colors);
	if (node) {
		node->set_material_override(matte);
	}
	node = _make_multimesh("Towers", box, glass_instances, glass_colors);
	if (node) {
		node->set_material_override(glass);
	}
	node = _make_multimesh("Roofs", make_roof_mesh(), roof_instances, roof_colors);
	if (node) {
		node->set_material_override(matte);
	}
	node = _make_multimesh("TreeTrunks", trunk, trunk_instances, std::vector<Color>());
	if (node) {
		Ref<StandardMaterial3D> bark = make_material(0.95, 0.0);
		bark->set_flag(BaseMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, false);
		bark->set_albedo(Color(0.28, 0.20, 0.14));
		node->set_material_override(bark);
	}
	node = _make_multimesh("TreeCanopies", canopy, canopy_instances, canopy_colors);
	if (node) {
		node->set_material_override(matte);
	}

	UtilityFunctions::print(vformat(
			"[PolisLab] city seed %d: %d junctions, %d road segments, %d buildings, %d houses+towers, %d trees",
			city_seed, network->get_junction_count(), network->get_segment_count(),
			(int)(concrete_instances.size() + glass_instances.size()),
			(int)roof_instances.size(), (int)canopy_instances.size()));

	emit_signal("city_generated");
}

#define POLIS_SETTER(m_name, m_type)                     \
	void CityGenerator::set_##m_name(m_type p_value) {   \
		m_name = p_value;                                \
	}

POLIS_SETTER(city_seed, int)
POLIS_SETTER(block_size, double)
POLIS_SETTER(street_width, double)
POLIS_SETTER(avenue_width, double)
POLIS_SETTER(highway_width, double)
POLIS_SETTER(highway_margin, double)
POLIS_SETTER(build_highway_ring, bool)
POLIS_SETTER(downtown_ratio, double)
POLIS_SETTER(midtown_ratio, double)
POLIS_SETTER(park_chance, double)
POLIS_SETTER(generate_on_ready, bool)
#undef POLIS_SETTER

void CityGenerator::set_blocks_x(int p_value) {
	blocks_x = std::max(p_value, 2);
}

void CityGenerator::set_blocks_z(int p_value) {
	blocks_z = std::max(p_value, 2);
}

void CityGenerator::set_avenue_every(int p_value) {
	avenue_every = std::max(p_value, 2);
}
