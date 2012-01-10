#include <cratedemo/initialCrateContentDemo1.hpp>
#include <cratedemo/MiniBall.hpp>

namespace cratedemo
{
	static CrateContentMap* crateContent1 = NULL;

	static void addEmpty(const char* name)
	{
		std::vector<CrateContent*> vec;
		for(size_t i = 0; i < 4*4; i++)
		{
			vec.push_back(NULL);
		}
		crateContent1->insert(std::pair<std::string, std::vector<CrateContent*> >(name, vec));
	}

	static void add(const char* name, Color::type color)
	{
		std::vector<CrateContent*> vec;
		for(size_t i = 0; i < 4*4; i++)
		{
			vec.push_back(new MiniBall(color));
		}
		crateContent1->insert(std::pair<std::string, std::vector<CrateContent*> >(name, vec));
	}

	CrateContentMap& initializeCrateContent1(void)
	{
		if(crateContent1 == NULL)
		{
			crateContent1 = new CrateContentMap();
			add("GC4x4MB_1", Color::BLUE);
			addEmpty("GC4x4MB_2");
			add("GC4x4MB_3", Color::RED);
			addEmpty("GC4x4MB_4");
		}

		return *crateContent1;
	}
}
