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

#ifndef __SGD_DATA_H__
#define __SGD_DATA_H__

#include "sgdparams.h"

MTS_NAMESPACE_BEGIN

typedef struct SGDData {
	Spectrum rhoD;
	Spectrum rhoS;
	Spectrum kap;
	Spectrum alpha;
	Spectrum p;
	Spectrum f0;
	Spectrum f1;
	Spectrum theta0;
	Spectrum c;
	Spectrum k;
	Spectrum lambda;

} SGDData;

static std::map<std::string, SGDData> sgdDatas =
	std::map<std::string, SGDData>();

/**
 * \brief Given a material name, load the material data and return it as a
 *structure.
 *
 * \param name
 *      A material name
 *
 */
static SGDData addMaterial(const std::string &name) {
	SGDData newMaterial;

	SGDMaterial material = lookupSGDMaterial(name);

	newMaterial.rhoD.fromLinearRGB(material.rhoD[0], material.rhoD[1],
								   material.rhoD[2]);
	newMaterial.rhoS.fromLinearRGB(material.rhoS[0], material.rhoS[1],
								   material.rhoS[2]);
	newMaterial.kap.fromLinearRGB(material.kap[0], material.kap[1],
								  material.kap[2]);
	newMaterial.alpha.fromLinearRGB(material.alpha[0], material.alpha[1],
									material.alpha[2]);
	newMaterial.p.fromLinearRGB(material.p[0], material.p[1], material.p[2]);
	newMaterial.f0.fromLinearRGB(material.f0[0], material.f0[1],
								 material.f0[2]);
	newMaterial.f1.fromLinearRGB(material.f1[0], material.f1[1],
								 material.f1[2]);
	newMaterial.theta0.fromLinearRGB(material.theta0[0], material.theta0[1],
									 material.theta0[2]);
	newMaterial.c.fromLinearRGB(material.c[0], material.c[1], material.c[2]);
	newMaterial.k.fromLinearRGB(material.k[0], material.k[1], material.k[2]);
	newMaterial.lambda.fromLinearRGB(material.lambda[0], material.lambda[1],
									 material.lambda[2]);

	sgdDatas[name] = newMaterial;

	return newMaterial;
}

/**
 * \brief Given a material name, return the material data as a structure.
 *
 * \param name
 *      A material name
 *
 */
static inline SGDData lookupSGDData(const std::string &name) {
	std::map<std::string, SGDData>::iterator iter = sgdDatas.find(name);
	if (iter != sgdDatas.end())
		return iter->second;
	else
		return addMaterial(name);
}

MTS_NAMESPACE_END

#endif /* __SGD_DATA_H__ */
