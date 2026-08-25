#ifndef POLISLAB_MESH_BUILDER_H
#define POLISLAB_MESH_BUILDER_H

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/packed_color_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

namespace godot {

// Small accumulator that collects triangles and commits them as one ArrayMesh
// surface. Cities generate tens of thousands of quads, so batching everything
// into a handful of surfaces keeps the draw-call count flat.
class MeshBuilder {
	PackedVector3Array vertices;
	PackedVector3Array normals;
	PackedVector2Array uvs;
	PackedColorArray colors;
	PackedInt32Array indices;

public:
	bool is_empty() const { return indices.is_empty(); }
	int64_t vertex_count() const { return vertices.size(); }

	int add_vertex(const Vector3 &p_pos, const Vector3 &p_normal, const Vector2 &p_uv, const Color &p_color) {
		const int index = (int)vertices.size();
		vertices.push_back(p_pos);
		normals.push_back(p_normal);
		uvs.push_back(p_uv);
		colors.push_back(p_color);
		return index;
	}

	void add_triangle(int p_a, int p_b, int p_c) {
		indices.push_back(p_a);
		indices.push_back(p_b);
		indices.push_back(p_c);
	}

	// Godot treats clockwise winding as front-facing, so the corners must be
	// given in clockwise order as seen from the p_normal side.
	void add_quad(const Vector3 &p_a, const Vector3 &p_b, const Vector3 &p_c, const Vector3 &p_d,
			const Vector3 &p_normal, const Color &p_color, real_t p_uv_scale = 0.25) {
		const int a = add_vertex(p_a, p_normal, Vector2(p_a.x, p_a.z) * p_uv_scale, p_color);
		const int b = add_vertex(p_b, p_normal, Vector2(p_b.x, p_b.z) * p_uv_scale, p_color);
		const int c = add_vertex(p_c, p_normal, Vector2(p_c.x, p_c.z) * p_uv_scale, p_color);
		const int d = add_vertex(p_d, p_normal, Vector2(p_d.x, p_d.z) * p_uv_scale, p_color);
		add_triangle(a, b, c);
		add_triangle(a, c, d);
	}

	// Horizontal quad at a fixed height. Corners are XZ points ordered
	// min/min -> max/min -> max/max -> min/max, which faces +Y.
	void add_flat_quad(const Vector2 &p_a, const Vector2 &p_b, const Vector2 &p_c, const Vector2 &p_d,
			real_t p_y, const Color &p_color, real_t p_uv_scale = 0.25) {
		add_quad(Vector3(p_a.x, p_y, p_a.y), Vector3(p_b.x, p_y, p_b.y),
				Vector3(p_c.x, p_y, p_c.y), Vector3(p_d.x, p_y, p_d.y),
				Vector3(0, 1, 0), p_color, p_uv_scale);
	}

	// Oriented flat quad for a road segment: a strip of p_half_width either
	// side of the a->b centreline, wound to face +Y.
	void add_strip(const Vector3 &p_a, const Vector3 &p_b, real_t p_half_width, real_t p_y,
			const Color &p_color, real_t p_uv_scale = 0.25) {
		const Vector3 direction = (p_b - p_a).normalized();
		const Vector3 side = Vector3(-direction.z, 0, direction.x) * p_half_width;
		const Vector3 a(p_a.x, p_y, p_a.z);
		const Vector3 b(p_b.x, p_y, p_b.z);
		add_quad(a - side, b - side, b + side, a + side, Vector3(0, 1, 0), p_color, p_uv_scale);
	}

	// Axis-aligned rectangle in XZ, expressed as center + half extents.
	void add_flat_rect(const Vector2 &p_center, const Vector2 &p_half, real_t p_y, const Color &p_color,
			real_t p_uv_scale = 0.25) {
		const Vector2 a(p_center.x - p_half.x, p_center.y - p_half.y);
		const Vector2 b(p_center.x + p_half.x, p_center.y - p_half.y);
		const Vector2 c(p_center.x + p_half.x, p_center.y + p_half.y);
		const Vector2 d(p_center.x - p_half.x, p_center.y + p_half.y);
		add_flat_quad(a, b, c, d, p_y, p_color, p_uv_scale);
	}

	// Vertical side walls plus a top cap: used for kerbs, sidewalks and slabs.
	void add_slab(const Vector2 &p_center, const Vector2 &p_half, real_t p_bottom, real_t p_top,
			const Color &p_top_color, const Color &p_side_color, real_t p_uv_scale = 0.25) {
		add_flat_rect(p_center, p_half, p_top, p_top_color, p_uv_scale);

		const Vector2 min(p_center.x - p_half.x, p_center.y - p_half.y);
		const Vector2 max(p_center.x + p_half.x, p_center.y + p_half.y);

		add_quad(Vector3(min.x, p_bottom, min.y), Vector3(max.x, p_bottom, min.y),
				Vector3(max.x, p_top, min.y), Vector3(min.x, p_top, min.y),
				Vector3(0, 0, -1), p_side_color, p_uv_scale);
		add_quad(Vector3(max.x, p_bottom, max.y), Vector3(min.x, p_bottom, max.y),
				Vector3(min.x, p_top, max.y), Vector3(max.x, p_top, max.y),
				Vector3(0, 0, 1), p_side_color, p_uv_scale);
		add_quad(Vector3(min.x, p_bottom, max.y), Vector3(min.x, p_bottom, min.y),
				Vector3(min.x, p_top, min.y), Vector3(min.x, p_top, max.y),
				Vector3(-1, 0, 0), p_side_color, p_uv_scale);
		add_quad(Vector3(max.x, p_bottom, min.y), Vector3(max.x, p_bottom, max.y),
				Vector3(max.x, p_top, max.y), Vector3(max.x, p_top, min.y),
				Vector3(1, 0, 0), p_side_color, p_uv_scale);
	}

	// Appends the accumulated geometry to p_mesh as a new surface.
	void commit_to(const Ref<ArrayMesh> &p_mesh, const Ref<Material> &p_material) {
		if (is_empty()) {
			return;
		}
		Array arrays;
		arrays.resize(Mesh::ARRAY_MAX);
		arrays[Mesh::ARRAY_VERTEX] = vertices;
		arrays[Mesh::ARRAY_NORMAL] = normals;
		arrays[Mesh::ARRAY_TEX_UV] = uvs;
		arrays[Mesh::ARRAY_COLOR] = colors;
		arrays[Mesh::ARRAY_INDEX] = indices;

		const int surface = (int)p_mesh->get_surface_count();
		p_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
		if (p_material.is_valid()) {
			p_mesh->surface_set_material(surface, p_material);
		}
	}
};

} // namespace godot

#endif // POLISLAB_MESH_BUILDER_H
