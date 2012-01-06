#include <cratedemo/Crate.hpp>
#include <cratedemo/CrateExceptions.hpp>
#include <string>

namespace cratedemo
{
Crate::Crate(std::string name, datatypes::point2f position, float angle,
		datatypes::size3f size, size_t maxNumberOfObjects) :
		name(name), position(position), angle(angle), size(size), maxNumberOfObjects(
				maxNumberOfObjects)
{
	data = new CrateContent*[maxNumberOfObjects];
	memset(data, NULL, maxNumberOfObjects * sizeof(CrateContent*));
}

Crate::~Crate()
{
	delete[] data;
}

void Crate::put(size_t index, CrateContent* crateContent)
{
	if (data[index] != NULL)
	{
		throw cratedemo::LocationIsFullException();
	}
	data[index] = crateContent;
}

CrateContent* Crate::get(size_t index) const
{
	return data[index];
}

void Crate::remove(size_t index)
{
	if (data[index] != NULL)
	{
		delete data[index];
		data[index] = NULL;
	}
	else
	{
		throw cratedemo::LocationIsEmptyException();
	}
}

bool Crate::isEmpty() const
{
	for (size_t i = 0; i <= maxNumberOfObjects; i++)
	{
		if (data[i] != NULL)
		{
			return false;
		}
	}
	return true;
}
}
