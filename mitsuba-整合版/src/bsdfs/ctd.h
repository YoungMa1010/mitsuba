/*
	This file is part of Mitsuba, a physically based rendering system.

	Original copyright (c) 2007-2014 by Wenzel Jakob and others.
	Copyright (c) 2014 Pierre Moreau, Jean-Dominique Gascuel, maverick.inria.fr
	Copyright (c) 2017 Nicolas Holzschuch, Romain Pacanowski, maverick.inria.fr

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

#ifndef __CTD_DATA_H__
#define __CTD_DATA_H__

#include "ctdparams.h"

MTS_NAMESPACE_BEGIN

typedef struct CTDData {
	Spectrum eta;
	Spectrum k;
	Spectrum albedo;
	materialType type;
	double b;
	double p;
	double sigma_s;
	double alpha;
	double c;
} CTDData;

static std::map<std::string, CTDData> ctdDatas =
	std::map<std::string, CTDData>();

/**
 * \brief Given a material name, load the material data and return it as a
 *structure.
 *
 * \param name
 *      A material name
 *
 */
static inline bool addMaterial(const std::string &name, CTDData& newMaterial) {

	CTDMaterial material;
	if (lookupCTDMaterial(name, material)) {
		newMaterial.type = material.type;
		// copy data into Spectrum
		// using "fromLinearRGB" for eta and k is weird if SPECTRUM_SAMPLES > 3, 
		// but it works. Better than other methods.
		newMaterial.eta.fromLinearRGB(material.eta[0], material.eta[1], material.eta[2]);
		if (newMaterial.type == conductor) {
			newMaterial.albedo.fromLinearRGB(0.0, 0.0, 0.0);
			newMaterial.k.fromLinearRGB(material.k_albedo[0], material.k_albedo[1], material.k_albedo[2]);
		} else {
			newMaterial.k.fromLinearRGB(0.0, 0.0, 0.0);
			newMaterial.albedo.fromLinearRGB(material.k_albedo[0], material.k_albedo[1], material.k_albedo[2]);
		}
		newMaterial.b = material.b;
		newMaterial.p = material.p;
		newMaterial.sigma_s = material.sigma_s;
		newMaterial.alpha = material.alpha;
		newMaterial.c = material.c;
		ctdDatas[name] = newMaterial;
		return true;
		// material not in list, return false
	} else return false; 
}


/**
 * \brief Given a material name, return the material data as a structure.
 *
 * \param name
 *      A material name
 *
 */
static inline bool lookupCTDData(const std::string &name, CTDData& result) {
	// lookup material name in list: 
	std::map<std::string, CTDData>::iterator iter = ctdDatas.find(name);
	if (iter != ctdDatas.end()) {
		// We've done this one before:
		result = iter->second;
		return true;
	} else {
		// New material, add it to map
		return addMaterial(name, result);
	}
}

MTS_NAMESPACE_END

#endif /* __CTD_DATA_H__ */
