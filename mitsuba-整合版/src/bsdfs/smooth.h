/*
	This file is part of Mitsuba, a physically based rendering system.

	Original copyright (c) 2007-2014 by Wenzel Jakob and others.

	Mitsuba is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License Version 3
	as published by the Free Software Foundation.

	Mitsuba is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __Smooth_DATA_H__
#define __Smooth_DATA_H__

#include "smoothparams.h"

MTS_NAMESPACE_BEGIN

typedef struct SmoothData {
	Spectrum rhoD;
	Spectrum a;
	Float b;
	Float c;
	Float eta;
} SmoothData;

static std::map<std::string, SmoothData> ctdDatas =
	std::map<std::string, SmoothData>();

/**
 * \brief Given a material name, load the material data and return it as a
 *structure.
 *
 * \param name
 *      A material name
 *
 */
static SmoothData addMaterial(const std::string &name) {
	SmoothData newMaterial;

	SmoothMaterial material = lookupSmoothMaterial(name);

	newMaterial.rhoD.fromLinearRGB(material.rhoD[0], material.rhoD[1],
								   material.rhoD[2]);
	newMaterial.a.fromLinearRGB(material.a[0], material.a[1],
								   material.a[2]);
	newMaterial.b = material.b;
	newMaterial.c = material.c;
	newMaterial.eta = material.eta;
	ctdDatas[name] = newMaterial;

	return newMaterial;
}

/**
 * \brief Given a material name, return the material data as a structure.
 *
 * \param name
 *      A material name
 *
 */
static inline SmoothData lookupSmoothData(const std::string &name) {
	std::map<std::string, SmoothData>::iterator iter = ctdDatas.find(name);
	if (iter != ctdDatas.end())
		return iter->second;
	else
		return addMaterial(name);
}

MTS_NAMESPACE_END

#endif /* __Smooth_DATA_H__ */
