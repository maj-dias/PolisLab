#ifndef POLISLAB_CITY_GENERATOR_H
#define POLISLAB_CITY_GENERATOR_H

#include "road_network.h"

#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/classes/multi_mesh_instance3d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/random_number_generator.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector2.hpp>

#include <vector>

namespace godot {

class MeshBuilder;

// Procedurally builds the whole city: a jittered street grid crossed by wider
// avenues, an outer highway ring, and blocks zoned into downtown towers,
// mid-rise midtown, suburban houses and parks.
class CityGenerator : public Node3D {
	GDCLASS(CityGenerator, Node3D)

public:
	enum ZoneKind {
		ZONE_DOWNTOWN,
		ZONE_MIDTOWN,
		ZONE_SUBURB,
		ZONE_PARK,
	};

private:
	// One grid line: its centre coordinate, carriageway width and class.
	struct Axis {
		double center = 0.0;
		double width = 9.0;
		int road_class = RoadNetwork::ROAD_STREET;
	};

	// --- Inspector-exposed parameters -------------------------------------
	int city_seed = 20260824;
	int blocks_x = 12;
	int blocks_z = 12;
	double block_size = 64.0;
	double street_width = 9.0;
	double avenue_width = 16.0;
	int avenue_every = 4;
	double highway_width = 22.0;
	double highway_margin = 90.0;
	bool build_highway_ring = true;
	double downtown_ratio = 0.26;
	double midtown_ratio = 0.58;
	double park_chance = 0.08;
	bool generate_on_ready = true;

	// --- Generation state --------------------------------------------------
	Ref<RoadNetwork> network;
	Ref<RandomNumberGenerator> rng;
	std::vector<Axis> axis_x;
	std::vector<Axis> axis_z;
	std::vector<std::vector<int>> grid_junctions;

	std::vector<Transform3D> concrete_instances;
	std::vector<Color> concrete_colors;
	std::vector<Transform3D> glass_instances;
	std::vector<Color> glass_colors;
	std::vector<Transform3D> roof_instances;
	std::vector<Color> roof_colors;
	std::vector<Transform3D> trunk_instances;
	std::vector<Transform3D> canopy_instances;
	std::vector<Color> canopy_colors;

	Vector2 city_extent;

	void _clear_generated();
	void _build_axes();
	void _build_network();
	void _build_highway_ring();
	void _build_road_meshes(MeshBuilder &r_roads, MeshBuilder &r_markings, MeshBuilder &r_ground);
	void _build_blocks(MeshBuilder &r_sidewalks, MeshBuilder &r_parks);
	void _emit_downtown(const Vector2 &p_center, const Vector2 &p_half);
	void _emit_midtown(const Vector2 &p_center, const Vector2 &p_half);
	void _emit_suburb(const Vector2 &p_center, const Vector2 &p_half);
	void _emit_park(const Vector2 &p_center, const Vector2 &p_half, MeshBuilder &r_parks);
	void _emit_building(const Vector2 &p_center, const Vector2 &p_half, double p_height, bool p_glass);
	void _emit_tree(const Vector2 &p_position, double p_scale);
	ZoneKind _zone_for_block(int p_bx, int p_bz) const;

	MultiMeshInstance3D *_make_multimesh(const String &p_name, const Ref<Mesh> &p_mesh,
			const std::vector<Transform3D> &p_transforms, const std::vector<Color> &p_colors);

protected:
	static void _bind_methods();

public:
	CityGenerator();

	void _ready() override;

	// Wipes anything previously generated and rebuilds the city from scratch.
	void generate();

	Ref<RoadNetwork> get_network() const { return network; }
	Vector2 get_city_extent() const { return city_extent; }

	void set_city_seed(int p_value);
	int get_city_seed() const { return city_seed; }
	void set_blocks_x(int p_value);
	int get_blocks_x() const { return blocks_x; }
	void set_blocks_z(int p_value);
	int get_blocks_z() const { return blocks_z; }
	void set_block_size(double p_value);
	double get_block_size() const { return block_size; }
	void set_street_width(double p_value);
	double get_street_width() const { return street_width; }
	void set_avenue_width(double p_value);
	double get_avenue_width() const { return avenue_width; }
	void set_avenue_every(int p_value);
	int get_avenue_every() const { return avenue_every; }
	void set_highway_width(double p_value);
	double get_highway_width() const { return highway_width; }
	void set_highway_margin(double p_value);
	double get_highway_margin() const { return highway_margin; }
	void set_build_highway_ring(bool p_value);
	bool get_build_highway_ring() const { return build_highway_ring; }
	void set_downtown_ratio(double p_value);
	double get_downtown_ratio() const { return downtown_ratio; }
	void set_midtown_ratio(double p_value);
	double get_midtown_ratio() const { return midtown_ratio; }
	void set_park_chance(double p_value);
	double get_park_chance() const { return park_chance; }
	void set_generate_on_ready(bool p_value);
	bool get_generate_on_ready() const { return generate_on_ready; }
};

} // namespace godot

VARIANT_ENUM_CAST(godot::CityGenerator::ZoneKind);

#endif // POLISLAB_CITY_GENERATOR_H
