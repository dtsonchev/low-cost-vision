#pragma once

#include "point2f.hpp"
#include <vector>



namespace pcrctransformation {
	/**
	* @brief object that transforms pixel coordinates to real life coordinates \
	   using a number of fiducials of which the real life and pixel positions are known.
	**/
	class pc_rc_transformer {
		public:
			/**
			 * constructor for a pc_rc_transformer
			 * @param fiducials_real_coordinates a vector with the real world coordinates of the fiducials in the same order as fiducials_pixel_coordinates
			 * @param fiducials_pixel_coordinates a vector with the pixel coordinates of the fiducials in the same order as fiducials_real_coordinates
			 */
			pc_rc_transformer(const point2f::point2fvector& fiducials_real_coordinates, const point2f::point2fvector& fiducials_pixel_coordinates);
			/**
			 * destructor
			 */
			virtual ~pc_rc_transformer();
			/**
			 * function for updating the pixel coordinates of the fiducials
			 * @param fiducials_pixel_coordinates the new coordinates
			 */
			void set_fiducials_pixel_coordinates(const point2f::point2fvector& fiducials_pixel_coordinates);

			/**
			 * convert pixel coordinate to a real world coordinate.
			 * @param pixel_coordinate the pixel coordinate
			 * @return the real world location of the point.
			 */
			point2f to_rc(const point2f& pixel_coordinate) const;
			/**
			 * convert real world coordinate to a pixel coordinate.
			 * @param real_coordinate coordinate the real world coordinate
			 * @return the pixel location of the point.
			 */
			point2f to_pc(const point2f& real_coordinate) const;

		private:
			point2f::point2fvector fiducials_real_coordinates;
			point2f::point2fvector fiducials_pixel_coordinates;
			double scale;

			void update_transformation_parameters();

			//function by Tim Voght
			bool circle_circle_intersection(point2f point0, double r0,
											point2f point1, double r1,
											point2f &intersection0,
											point2f &intersection1) const;
	};

}
