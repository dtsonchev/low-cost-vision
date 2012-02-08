#include <huniplacer_3d/RangeModel.h>"

void RangeModel::init(effector_boundaries *eb){
	const bool * bitmap = eb->get_bitmap();
	int width = eb->get_width();
	int height = eb->get_height();
	int depth = eb->get_depth();

	int index = 0;

	int x_displacement = 1;
	int y_displacement = 1;
	int z_displacement = 1;

	for(int x1 = 1; x1 < width-1; x1 += 3){
		for(int y1 = 1; y1 < depth-1; y1 += 3){
			for(int z1 = 1; z1 < height-1; z1 += 3){
				index = x1 + y1 * width + z1 * width * depth;

				for(int x2 = 0; x2 < 3; x2++){
					for(int y2 = 0; y2 < 3; y2++){
						for(int z2 = 0; z2 < 3; z2++){
							int x = x1 + x2-1;
							int y = y1 + y2-1;
							int z = z1 + z2-1;
							index = x + y * width + z * width * depth;

							if(bitmap[index]){
								if(	!bitmap[index+1] &&
									!bitmap[index-1] &&
									!bitmap[index+width] &&
									!bitmap[index-width] &&
									!bitmap[index+width*depth] &&
									!bitmap[index+1] &&
									!bitmap[index-1] &&
									!bitmap[index+width] &&
									!bitmap[index-width] &&
									!bitmap[index+width*depth]){

									add_face(	x-0.5, 	y+0.5, 	z-0.5,
												x+0.5, 	y+0.5,	z-0.5,
												x, 		y, 		z);

									add_face(	x-0.5, 	y-0.5, 	z-0.5,
												x+0.5, 	y-0.5,	z-0.5,
												x, 		y, 		z);

									add_face(	x+0.5, 	y-0.5, 	z-0.5,
												x+0.5, 	y+0.5,	z-0.5,
												x, 		y, 		z);

									add_face(	x-0.5, 	y-0.5, 	z-0.5,
												x-0.5, 	y+0.5,	z-0.5,
												x, 		y, 		z);
								} else {
									for(int w = 0; w < 8; w++){
										z_displacement *= -1;
										if(w%2 == 0) y_displacement *= -1;
										if(w%4 == 0) x_displacement *= -1;

										if(!bitmap[index+x_displacement] && !bitmap[index+width*y_displacement] && !bitmap[index+width*depth*z_displacement]){
											add_face(	x+0.5*x_displacement, y, z,
														x, y+0.5*y_displacement, z,
														x, y, z+0.5*z_displacement);
										} else {
											if(!bitmap[index+x_displacement] && !bitmap[index+width*y_displacement] && bitmap[index+width*depth*z_displacement]){
												add_quad(	x+0.5*x_displacement, y, z,
															x, y+0.5*y_displacement, z,
															x, y+0.5*y_displacement, z+0.5*z_displacement,
															x+0.5*x_displacement, y, z+0.5*z_displacement);
										} else {
											if(!bitmap[index+x_displacement] && bitmap[index+width*y_displacement] && !bitmap[index+width*depth*z_displacement]){
												add_quad(	x+0.5*x_displacement, y+0.5*y_displacement, z,
															x, y+0.5*y_displacement, z+0.5*z_displacement,
															x, y, z+0.5*z_displacement,
															x+0.5*x_displacement, y, z);
										} else {
											if(bitmap[index+x_displacement] && !bitmap[index+width*y_displacement] && !bitmap[index+width*depth*z_displacement]){
												add_quad( 	x+0.5*x_displacement, y+0.5*y_displacement, z,
															x, y+0.5*y_displacement, z,
															x, y, z+0.5*z_displacement,
															x+0.5*x_displacement, y, z+0.5*z_displacement);
										} else {
											if(!bitmap[index+x_displacement] && bitmap[index+width*y_displacement] && bitmap[index+width*depth*z_displacement]){
												if(bitmap[index+width*y_displacement + width*depth*z_displacement]){
													add_quad( 	x+0.5*x_displacement, 	y, 						z,
																x+0.5*x_displacement, 	y+0.5*y_displacement, 	z,
																x+0.5*x_displacement, 	y+0.5*y_displacement, 	z+0.5*z_displacement,
																x+0.5*x_displacement, 	y, 						z+0.5*z_displacement);
												} else {
													add_face(	x+0.5*x_displacement, 	y+0.5*y_displacement, 	z,
																x, 						y+0.5*y_displacement, 	z+0.5*z_displacement,
																x+0.5*x_displacement, 	y, 						z+0.5*z_displacement);
													add_face(	x+0.5*x_displacement, 	y, 						z,
																x+0.5*x_displacement, 	y+0.5*y_displacement, 	z,
																x+0.5*x_displacement, 	y, 						z+0.5*z_displacement);
												}
										} else {
											if(bitmap[index+x_displacement] && !bitmap[index+width*y_displacement] && bitmap[index+width*depth*z_displacement]){
												if(bitmap[index+x_displacement + width*depth*z_displacement]){
													add_quad(	x+0.5*x_displacement, 	y+0.5*y_displacement, 	z,
																x, 						y+0.5*y_displacement, 	z,
																x, 						y+0.5*y_displacement, 	z+0.5*z_displacement,
																x+0.5*x_displacement, 	y+0.5*y_displacement, 	z+0.5*z_displacement);
												} else {
													add_face(	x+0.5*x_displacement, 	y+0.5*y_displacement, 	z,
																x, 						y+0.5*y_displacement, 	z+0.5*z_displacement,
																x+0.5*x_displacement, 	y, 						z+0.5*z_displacement);
													add_face(	x+0.5*x_displacement, 	y+0.5*y_displacement, 	z,
																x, 						y+0.5*y_displacement, 	z+0.5*z_displacement,
																x, 						y+0.5*y_displacement, 	z);
												}
										} else {
											if(bitmap[index+x_displacement] && bitmap[index+width*y_displacement] && !bitmap[index+width*depth*z_displacement]){
												if(bitmap[index+x_displacement + width*y_displacement]){
													add_quad(	x+0.5*x_displacement, 	y+0.5*y_displacement, 	z+0.5*z_displacement,
																x, 						y+0.5*y_displacement, 	z+0.5*z_displacement,
																x, 						y, 						z+0.5*z_displacement,
																x+0.5*x_displacement, 	y, 						z+0.5*z_displacement);
												} else {
													add_face(	x+0.5*x_displacement, 	y+0.5*y_displacement, 	z,
																x, 						y+0.5*y_displacement, 	z+0.5*z_displacement,
																x+0.5*x_displacement, 	y, 						z+0.5*z_displacement);
													add_face(	x+0.5*x_displacement, 	y, 						z+0.5*z_displacement,
																x, 						y+0.5*y_displacement, 	z+0.5*z_displacement,
																x, 						y, 						z+0.5*z_displacement);
												}
										} else {
											if(bitmap[index+x_displacement] && bitmap[index+width*y_displacement] && bitmap[index+width*depth*z_displacement]){
												if(!bitmap[index + x_displacement + width*y_displacement]){
													add_face(	x+0.5*x_displacement, 	y+0.5*y_displacement, 	z,
																x, 						y+0.5*y_displacement, 	z+0.5*z_displacement,
																x+0.5*x_displacement, 	y, 						z+0.5*z_displacement);
												} else {
													if(!bitmap[index + x_displacement + width*depth*z_displacement] && !bitmap[index + width*y_displacement + width*depth*z_displacement]){
														add_face(	x, 						y+0.5*y_displacement,	z+0.5*z_displacement,
																	x+0.5*x_displacement, 	y, 						z+0.5*z_displacement,
																	x+0.5*x_displacement,	y+0.5*y_displacement, 	z+0.5*z_displacement);
													}
												}
										}}}}}}}}
									}
								}
							}
						}
					}
				}
			}
		}
	}
}

void RangeModel::add_quad(float ax, float ay, float az, float bx, float by, float bz, float cx, float cy, float cz, float dx, float dy, float dz){
	add_face(ax,ay,az,bx,by,bz,cx,cy,cz);
	add_face(cx,cy,cz,dx,dy,dz,ax,ay,az);
}



void RangeModel::add_face(float ax, float ay, float az, float bx, float by, float bz, float cx, float cy, float cz){
	faces.push_back(Face(ax,ay,az,bx,by,bz,cx,cy,cz));
}

std::vector<Face> RangeModel::get_faces(){
	return faces;
}
