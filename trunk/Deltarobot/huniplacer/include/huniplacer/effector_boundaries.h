#pragma once

#include <huniplacer/point3.h>
#include <huniplacer/imotor3.h>
#include <huniplacer/measures.h>
#include <huniplacer/inverse_kinematics_model.h>
#include <vector>

namespace huniplacer
{
	class effector_boundaries
	{
	public:
		~effector_boundaries();

		static effector_boundaries* generate_effector_boundaries(const inverse_kinematics_model& model, const imotor3& motors, double voxel_size);
		bool check_path(const point3& from, const point3& to) const;
		inline const std::vector<bool>& get_bitmap() const;
		inline int get_width() const;
		inline int get_height() const;
		inline int get_depth() const;
		inline double get_voxel_size() const;

	private:
		effector_boundaries(const inverse_kinematics_model& model, const imotor3& motors, double voxel_size);

		typedef struct bitmap_coordinate { int x, y, z; bitmap_coordinate(int x, int y, int z) : x(x), y(y), z(z) {} } bitmap_coordinate;

		bool has_invalid_neighbours(const point3& p) const;
		bool is_valid(const point3& p) const;
		void generate_boundaries_bitmap();
		inline point3 from_bitmap_coordinate(effector_boundaries::bitmap_coordinate coordinate) const;
		inline bitmap_coordinate from_real_coordinate(point3 coordinate) const;

		char* point_validity_cache;

		int width, height, depth;

		std::vector<bool> boundaries_bitmap;
		const inverse_kinematics_model &kinematics;
		const imotor3 &motors;
		double voxel_size;


	};

	const std::vector<bool>& effector_boundaries::get_bitmap() const {return boundaries_bitmap;}
	int effector_boundaries::get_depth() const {return depth;}
	int effector_boundaries::get_height() const {return height;}
	int effector_boundaries::get_width() const {return width;}
	double effector_boundaries::get_voxel_size() const {return voxel_size;}

	point3 effector_boundaries::from_bitmap_coordinate(effector_boundaries::bitmap_coordinate coordinate) const
	{
		return point3(
				(double) coordinate.x * voxel_size + measures::MIN_X,
				(double) coordinate.y * voxel_size + measures::MIN_Y,
				(double) coordinate.z * voxel_size + measures::MIN_Z);
	}

	effector_boundaries::bitmap_coordinate effector_boundaries::from_real_coordinate(point3 coordinate) const
	{
		return effector_boundaries::bitmap_coordinate(
			(coordinate.x - measures::MIN_X) / voxel_size,
			(coordinate.y - measures::MIN_Y) / voxel_size,
			(coordinate.z - measures::MIN_Z) / voxel_size);
	}
}
