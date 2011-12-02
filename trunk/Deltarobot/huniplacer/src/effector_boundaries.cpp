#include <huniplacer/huniplacer.h>
#include <stack>
#include <vector>
#include <set>

namespace huniplacer
{
	using namespace measures;

	effector_boundaries effector_boundaries::generate_effector_boundaries(const inverse_kinematics_model& model, const imotor3& motors, double voxel_size)
	{
		effector_boundaries boundaries(model, motors, voxel_size);

		boundaries.width = (double)((MAX_X - MIN_X)) / voxel_size;
        boundaries.height = (double)((MAX_Z - MIN_Z)) / voxel_size;
        boundaries.depth = (double)((MAX_Y - MIN_Y)) / voxel_size;
        boundaries.boundaries_bitmap = std::vector<bool>(boundaries.width * boundaries.height * boundaries.depth, false);

        boundaries.generate_boundaries_bitmap();

        return boundaries;
    }

    bool effector_boundaries::check_path(const point3 & from, const point3 & to)
    {
        return true; ///<Replace This value
    }

    effector_boundaries::effector_boundaries(const inverse_kinematics_model& model, const imotor3& motors, double voxel_size)
    	: kinematics(model), motors(motors), voxel_size(voxel_size)
    {
    }

    effector_boundaries::~effector_boundaries()
    {
    }

    bool effector_boundaries::has_invalid_neighbours(const point3 & p)
    {
        for(int y = p.y - 1;y <= p.y + 1;y++){
            for(int x = p.x - 1;x <= p.x + 1;x++){
                for(int z = p.z - 1;z <= p.z + 1;z++){
                    if(x != p.x && y != p.y && z != p.z && !is_valid(point3(x, y, z))){
                        return true;
                    }
                }

            }

        }

        return false;
    }

    bool effector_boundaries::is_valid(const point3 & p)
    {
        motionf mf;
        try {
            kinematics.point_to_motion(p, mf);
        }
        catch(huniplacer::inverse_kinematics_exception & ex)
        {
            return false;
        }
        for(int i = 0;i < 3;i++){
            if(mf.angles[i] <= motors.get_min_angle() || mf.angles[i] >= motors.get_max_angle()){
                return false;
            }
        }

        return true;
    }



    void effector_boundaries::generate_boundaries_bitmap()
    {


    	std::stack<bitmap_coordinate> cstack;

    	point3 begin(0, 0, MIN_Z + (MAX_Z - MIN_Z) / 2);

		for(; begin.x < MAX_X; begin.x += voxel_size)
		{
			if(!is_valid(begin))
			{
				begin.x -= 1;
				bitmap_coordinate coordinate = from_real_coordinate(begin);
				cstack.push(coordinate);
				boundaries_bitmap[coordinate.x + coordinate.y * width + coordinate.z * width * depth] = true;
				break;
			}
		}

		while(!cstack.empty())
		{
			bitmap_coordinate c = cstack.top();
			cstack.pop();

			for(int y = c.y-1; y <= c.y+1; y++)
			{
				for(int x = c.x-1; x <= c.x+1; x++)
				{
					for(int z = c.z-1; z <= c.z+1; x++)
					{
						int index = x + y * width + z * width * depth;
						point3 real_coordinate = from_bitmap_coordinate(bitmap_coordinate(x, y, z));
						if(is_valid(real_coordinate)
						&& !((real_coordinate.x < measures::MAX_X && real_coordinate.x >= measures::MIN_X && real_coordinate.y < measures::MAX_Y && real_coordinate.y >= measures::MIN_Y && real_coordinate.z < measures::MAX_Z && real_coordinate.z >= measures::MIN_Z)
						&& boundaries_bitmap[index])
						&& has_invalid_neighbours(real_coordinate))
						{
							cstack.push(bitmap_coordinate(x, y, z));
							boundaries_bitmap[index] = true;
						}
					}
				}
			}
		}
	}
}
