//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        cratedemo
// File:           Crate.hpp
// Description:    Base class for crates.
// Author:         Wouter Langerak & Lukas Vermond
// Notes:
//
// License:        GNU GPL v3
//
// This file is part of cratedemo.
//
// cratedemo is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// cratedemo is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with cratedemo.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#include <cratedemo/Crate.hpp>

#include <cratedemo/CrateExceptions.hpp>
#include <datatypes/point2.hpp>
#include <datatypes/size3.hpp>

namespace cratedemo {

Crate::Crate(
	std::string name,
	datatypes::point2f position,
	float angle,
	bool moving,
	datatypes::size3f size,
	std::vector<CrateContent*>& crateContent) :
		name(name),
		position(position),
		angle(angle),
		moving(moving),
		size(size),
		data(crateContent) {
}

Crate::~Crate() {
}

void Crate::put(size_t index, CrateContent* crateContent) {
	if (data.at(index) != NULL)
	{
		throw cratedemo::LocationIsEmptyException();
	}
	data[index] = crateContent;
}

CrateContent* Crate::get(size_t index) const {
	CrateContent* content = data.at(index);
	if(content == NULL) { throw LocationIsFullException(); }
	return content;
}

void Crate::remove(size_t index) {
	if(data.at(index) == NULL) { throw LocationIsEmptyException(); }
	data[index] = NULL;
}

const std::string& Crate::getName(void) const {
	return name;
}

bool Crate::isEmpty() const {
	for(std::vector<CrateContent*>::iterator it = data.begin();
		it != data.end(); ++ it)
	{
		if (*it != NULL)
		{
			return false;
		}
	}
	return true;
}
}
