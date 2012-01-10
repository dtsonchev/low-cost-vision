#include <cratedemo/GridCrate.hpp>
#include <cratedemo/MiniBall.hpp>
#include <iostream>
#include <cratedemo/initialCrateContentDemo1.hpp>
#include <cratedemo/CrateDemo.hpp>

using namespace cratedemo;

class Demo : public CrateDemo
{
	void onNewCrate(Crate& crate) {
		CrateMap::iterator it1 = crates.find("GC4x4MB_1");
		if(it1 == crates.end()) { return; }

		CrateMap::iterator it2 = crates.find("GC4x4MB_2");
		if(it2 == crates.end()) { return; }

		for(size_t i = 0; i < 4*4; i++)
		{
			moveObject(*it1, i, *it2, i);
		}
	}

	void onCrateMove(Crate& crate)
	{
		ROS_INFO("\%s\" moved", crate.getName().c_str());
	}

	void onCrateRemoved(Crate& crate)
	{
		ROS_INFO("\%s\" removed", crate.getName().c_str());
	}
};

int main(int argc, char** argv) {
	std::cout << "flinterdun" << std::endl;
	return 0;
}
