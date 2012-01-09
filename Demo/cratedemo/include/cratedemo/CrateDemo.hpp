#pragma once

#include <string>
#include <cratedemo/Crate.hpp>

namespace cratedemo
{
class CrateDemo
{
protected:
	CrateDemo(
		const std::string& deltaGrip, const std::string& deltaStop, const std::string& deltaMove, const std::string& deltaError,
		const std::string& visionGetCrate, const std::string& visionGetAllCrates, const std::string& visionError);

public:
	virtual ~CrateDemo();

	virtual void onNewCrate(Crate& crate) = 0;
	virtual void onCrateMove(Crate& crate) = 0;

	void update();
};
}
