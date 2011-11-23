#include <pcrctransformation/pcrctransformer.hpp>
#include <pcrctransformation/point2f.hpp>
#include <iostream>

using namespace pcrctransformation;

int main(void)
{
	std::cout<<"start"<<std::endl;
	point2f::point2fvector rc;
	rc.push_back(point2f(0, 0));
	rc.push_back(point2f(0, 10));
	rc.push_back(point2f(10, 10));

	point2f::point2fvector pc;
	pc.push_back(point2f(100, 100));
	pc.push_back(point2f(100, 0));
	pc.push_back(point2f(0, 0));

	pc_rc_transformer p(rc, pc);
	point2f result = p.to_pc(point2f(1,1));
	point2f result2 = p.to_rc(result);
	std::cout << result.x << ", " << result.y << std::endl;
	std::cout << result2.x << ", " << result2.y << std::endl;
	return 0;
}
