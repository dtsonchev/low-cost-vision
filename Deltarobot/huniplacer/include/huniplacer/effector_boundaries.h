#pragma once

#include <huniplacer/huniplacer.h>
#include <vector>

namespace huniplacer
{
	class effector_boundaries
	{
	public:
		~effector_boundaries();

		static effector_boundaries generate_effector_boundaries(const inverse_kinematics_model& model, const imotor3& motors, double voxel_size);
		bool check_path(const point3& from, const point3& to);
		inline const std::vector<bool>& get_bitmap();
		inline int get_width();
		inline int get_height();
		inline int get_depht();
		inline double get_voxel_size();

	private:
		effector_boundaries(const inverse_kinematics_model& model, const imotor3& motors, double voxel_size);

		typedef struct bitmap_coordinate { int x, y, z; bitmap_coordinate(int x, int y, int z) : x(x), y(y), z(z) {} } bitmap_coordinate;

		bool has_invalid_neighbours(const point3& p);
		bool is_valid(const point3& p);
		void generate_boundaries_bitmap();
		inline point3 from_bitmap_coordinate(effector_boundaries::bitmap_coordinate coordinate);
		//inline bitmap_coordinate from_index(int index);
		inline bitmap_coordinate from_real_coordinate(point3 coordinate);

		int width, height, depth;

		std::vector<bool> boundaries_bitmap;
		const inverse_kinematics_model &kinematics;
		const imotor3 &motors;
		double voxel_size;


	};

	const std::vector<bool>& effector_boundaries::get_bitmap(){return boundaries_bitmap;}
	int effector_boundaries::get_depht(){return depth;}
	int effector_boundaries::get_height(){return height;}
	int effector_boundaries::get_width(){return width;}
	double effector_boundaries::get_voxel_size(){return voxel_size;}

	point3 effector_boundaries::from_bitmap_coordinate(effector_boundaries::bitmap_coordinate coordinate)
	{
		return point3(
				(double) coordinate.x * voxel_size + measures::MIN_X,
				(double) coordinate.y * voxel_size + measures::MIN_Y,
				(double) coordinate.z * voxel_size + measures::MIN_Z);
	}

	effector_boundaries::bitmap_coordinate effector_boundaries::from_real_coordinate(point3 coordinate)
	{
		return effector_boundaries::bitmap_coordinate(
			(coordinate.x - measures::MIN_X) / voxel_size,
			(coordinate.y - measures::MIN_Y) / voxel_size,
			(coordinate.z - measures::MIN_Z) / voxel_size);
	}
}

