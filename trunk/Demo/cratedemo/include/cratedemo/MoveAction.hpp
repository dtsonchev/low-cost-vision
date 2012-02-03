#pragma once

#include <string>
#include <cstdlib>

class MoveAction
{
private:
	std::string strFrom;
	size_t indexFrom;

	std::string strTo;
	size_t indexTo;

public:
	MoveAction(
		const std::string& strFrom,
		size_t indexFrom,
		const std::string& strTo,
		size_t indexTo) :
			strFrom(strFrom),
			indexFrom(indexFrom),
			strTo(strTo),
			indexTo(indexTo) {}

	const std::string& getStrFrom(void) const { return strFrom; }
	size_t getIndexFrom(void) const { return indexFrom; }

	const std::string& getStrTo(void) const { return strTo; }
	size_t getIndexTo(void) const { return indexTo; }
};
