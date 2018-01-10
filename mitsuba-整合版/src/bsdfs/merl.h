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

#ifndef __MERL_DATA_H__
#define __MERL_DATA_H__

#include <iostream>
#include <map>

#include <mitsuba/mitsuba.h>
#include <mitsuba/core/fstream.h>

#define SAMPLING_RES_THETA_H 90
#define SAMPLING_RES_THETA_D 90
#define SAMPLING_RES_PHI_D 360
#define RED_SCALING 1.0f
#define GREEN_SCALING 1.15f
#define BLUE_SCALING 1.66f

MTS_NAMESPACE_BEGIN

namespace merl {
namespace index {
static const int thetaD = SAMPLING_RES_PHI_D / 2;
static const int thetaH = SAMPLING_RES_PHI_D / 2 * SAMPLING_RES_THETA_D;
}

namespace scaling {
static const Float red = RED_SCALING / 1500.0f;
static const Float green = GREEN_SCALING / 1500.0f;
static const Float blue = BLUE_SCALING / 1500.0f;
}

namespace sampRes {
static const int thetaHMax = SAMPLING_RES_THETA_H - 1;
static const int thetaDMax = SAMPLING_RES_THETA_D - 1;
static const int phiDMax = SAMPLING_RES_PHI_D / 2 - 1;

static const Float thetaHFactor =
	2.0f * INV_PI * SAMPLING_RES_THETA_H * SAMPLING_RES_THETA_H;
static const Float thetaDFactor = 2.0f * INV_PI * SAMPLING_RES_THETA_D;
static const Float phiDFactor = INV_PI * SAMPLING_RES_PHI_D / 2;
}

namespace storage {
static const int canalSize =
	SAMPLING_RES_THETA_H * SAMPLING_RES_THETA_D * SAMPLING_RES_PHI_D / 2;
static const int twoCanalSize = 2 * canalSize;
static const int threeCanalSize = 3 * canalSize;
}
}

typedef std::map<std::string, double *> MERLMap;
static MERLMap MERLData;

/**
 * \brief Given a path to the material file and a material name,
 *      load the material data and return it as an array.
 *
 * \param filepath
 *      A filepath to the material data file.
 * \param name
 *      A material name
 *
 */
static double *addMaterial(const fs::path &filePath, const std::string &name) {
	ref<FileStream> file = new FileStream(filePath);
	if (file->canRead()) {
		int *dimensions = new int[3];
		file->read(dimensions, 3 * sizeof(int));

		const int size = dimensions[0] * dimensions[1] * dimensions[2];
		if (size == merl::storage::canalSize) {
			double *tmp = new double[merl::storage::threeCanalSize];
			file->read(tmp, merl::storage::threeCanalSize * sizeof(double));
			file->close();
			MERLData[name] = tmp;

			return tmp;
		} else {
			SLog(
				EError,
				"Dimensions found in the given file are not the expected one.");

			file->close();
			return NULL;
		}
	} else {
		SLog(EError, "The given file could not be read.");

		return NULL;
	}
}

/**
 * \brief Given a material name and a path to the material file,
 *      return the material data as an array.
 *
 * \param name
 *      A material name
 * \param filepath
 *      A filepath to the material data file. It is only used
 *      when the material has never been accessed.
 *
 */
static inline double *lookupMERL(const std::string &name,
								 const fs::path &filePath) {
	double *result = NULL, *tmp = NULL;

	MERLMap::iterator iter = MERLData.find(name);
	if (iter != MERLData.end())
		tmp = iter->second;
	else
		tmp = addMaterial(filePath, name);

	if (tmp != NULL) {
		result = new double[merl::storage::threeCanalSize];
		std::copy(tmp, tmp + merl::storage::threeCanalSize, result);
	}

	return result;
}

static void releaseMERL(void) {
	if (!MERLData.empty()) {
		for (MERLMap::iterator iter = MERLData.begin(); iter != MERLData.end();
			 iter++)
			delete iter->second;
		MERLData.clear();
	}
}

/**
 * \brief Given an incident direction wi and a half-vector h, generate
 *      the corresponding values for phiD, thetaD and thetaH.
 *
 * \param wi
 *      An incident direction in local coordinates. This should
 *      be a normalized direction vector that points \a away from
 *      the scattering event.
 * \param h
 *      A half-vector in local coordinates. This should
 *      be a normalized direction vector that points \a away from
 *      the scattering event.
 *
 */
static void getAnglesFromVect(const Vector &wi, const Vector &h, Float &phiD,
							  Float &thetaD, Float &thetaH) {
	const Float cosTH = Frame::cosTheta(h), sinTH = Frame::sinTheta(h);
	const Float cosPH = Frame::cosPhi(h), sinPH = Frame::sinPhi(h);

	const Float tmpX = wi.x * cosPH + wi.y * sinPH;
	const Vector transformedWi = normalize(Vector(tmpX * cosTH - wi.z * sinTH,
												  wi.y * cosPH - wi.x * sinPH,
												  wi.z * cosTH + tmpX * sinTH));

	phiD = std::acos(Frame::cosPhi(transformedWi));
	thetaD = std::acos(Frame::cosTheta(transformedWi));
	thetaH = std::acos(Frame::cosTheta(h));
}

/**
 * \brief Given the angle thetaH, return the associated index.
 *
 * \param thetaH
 *      The angle thetaH.
 *
 */
static int getThetaHIndex(Float thetaH) {
	if (thetaH <= 0.0)
		return 0;
	else if (thetaH >= M_PI)
		return merl::sampRes::thetaHMax;
	else
		return static_cast<int>(
			std::sqrt(thetaH * merl::sampRes::thetaHFactor));
}

/**
 * \brief Given the angle thetaD, return the associated index.
 *
 * \param thetaD
 *      The angle thetaD.
 *
 */
static int getThetaDIndex(Float thetaD) {
	if (thetaD <= 0.0)
		return 0;
	else if (thetaD >= M_PI / 2)
		return merl::sampRes::thetaDMax;
	else
		return static_cast<int>(thetaD * merl::sampRes::thetaDFactor);
}

/**
 * \brief Given the angle phiD, return the associated index.
 *
 * \param phiD
 *      The angle phiD.
 *
 */
static int getPhiDIndex(Float phiD) {
	if (phiD < 0.0)
		phiD += M_PI;

	if (phiD <= 0.0)
		return 0;
	else if (phiD >= M_PI)
		return merl::sampRes::phiDMax;
	else
		return static_cast<int>(phiD * merl::sampRes::phiDFactor);
}

/**
  * \brief Given the index, return the angle phiD
  *
  * \param index
  *      The index for the angle phiD.
  *
  */
 static Float getIndexPhiD(int index) {
 	return (Float)index / merl::sampRes::phiDFactor;
 }
 
 /**
  * \brief Given the index, return the angle thetaD
  *
  * \param index
  *      The index for the angle thetaD.
  *
  */
 static Float getIndexThetaD(int index) {
 	return (Float)index / merl::sampRes::thetaDFactor;
 }
 
 /**
  * \brief Given the index, return the angle thetaH
  *
  * \param index
  *      The index for the angle thetaD.
  *
  */
 static Float getIndexThetaH(int index) {
 	return  (index * index ) / merl::sampRes::thetaHFactor; 
 }
 
 /**
 * \brief Given a material array data, an incident direction
 *      wi and a half-vector h, returns the corresponding spectrum.
 *
 * \param data
 *      An array containing a material data.
 * \param wi
 *      An incident direction in local coordinates. This should
 *      be a normalized direction vector that points \a away from
 *      the scattering event.
 * \param h
 *      A half-vector in local coordinates. This should
 *      be a normalized direction vector that points \a away from
 *      the scattering event.
 *
 */
static inline Spectrum lookupSpectrum(const double *const data,
									  const Vector &wi, const Vector &h) {
	Spectrum result;

	Float phiD = 0, thetaD = 0, thetaH = 0;

	getAnglesFromVect(wi, h, phiD, thetaD, thetaH);

	// No interpolation between values
	// const int ind = getPhiDIndex(phiD) +
	// 				getThetaDIndex(thetaD) * merl::index::thetaD +
	// 				getThetaHIndex(thetaH) * merl::index::thetaH;
	// 	result.fromLinearRGB(
	// 		(Float)data[ind] * merl::scaling::red,
	// 		(Float)data[ind + merl::storage::canalSize] * merl::scaling::green,
	// 		(Float)data[ind + merl::storage::twoCanalSize] * merl::scaling::blue);
 
 	// Interpolate between values
 	Float red_val = 0.0;
 	Float green_val = 0.0;
 	Float blue_val = 0.0;
 	if (phiD > M_PI) phiD -= M_PI;
 
 	int phi_diff_i[2]; // stores phi_diff_index(phiD) and phi_diff_index(phiD) + 1
 	int theta_diff_j[2]; // stores theta_diff_index and theta_diff_index + 1
 	int theta_half_k[2]; // stores theta_half_index and theta_half_index + 1
 	phi_diff_i[0] = getPhiDIndex(phiD);
 	phi_diff_i[1] = phi_diff_i[0] + 1;
 	if (phi_diff_i[1] >= SAMPLING_RES_PHI_D / 2) phi_diff_i[1] = phi_diff_i[0]; 
 	theta_diff_j[0] = getThetaDIndex(thetaD);
 	theta_diff_j[1] = theta_diff_j[0] + 1;
 	if (theta_diff_j[1] >= SAMPLING_RES_THETA_D) theta_diff_j[1] = theta_diff_j[0] ;
 	theta_half_k[0] = getThetaHIndex(thetaH);
 	theta_half_k[1] = theta_half_k[0] + 1;
 	if (theta_half_k[1] >= SAMPLING_RES_THETA_H) theta_half_k[1] = theta_half_k[0];
 	if (phiD < 0.0) phiD += M_PI;
 	double phi_diff_a;
 	if (phi_diff_i[1] != phi_diff_i[0]) phi_diff_a = (phiD - getIndexPhiD(phi_diff_i[0])) / (getIndexPhiD(phi_diff_i[1]) - getIndexPhiD(phi_diff_i[0]));
 	else phi_diff_a = 0.5;
 	if (fabs(phi_diff_a) < 1e-6) phi_diff_a = 0.0;
 	assert(phi_diff_a >= 0);
 	assert(phi_diff_a <= 1);
 	double theta_diff_a;
 	if (theta_diff_j[1] != theta_diff_j[0]) theta_diff_a = (thetaD - getIndexThetaD(theta_diff_j[0])) / (getIndexThetaD(theta_diff_j[1]) - getIndexThetaD(theta_diff_j[0]));
 	else theta_diff_a = 0.5;
 	if (fabs(theta_diff_a) < 1e-6) theta_diff_a = 0.0;
 	assert(theta_diff_a >= 0);
 	assert(theta_diff_a <= 1);
 	double theta_half_a; 
 	if (theta_half_k[1] != theta_half_k[0]) theta_half_a = (thetaH - getIndexThetaH(theta_half_k[0])) / (getIndexThetaH(theta_half_k[1]) - getIndexThetaH(theta_half_k[0]));
 	else theta_half_a = 0.5;
 	if (fabs(theta_half_a) < 1e-6) theta_half_a = 0.0;
 	assert(theta_half_a >= 0);
 	assert(theta_half_a <= 1);
 
 	for (int i = 0; i < 2; i++) {
 		double phi_diff_interpolant = (i == 0) ? 1. - phi_diff_a : phi_diff_a;
 		for (int j = 0; j <2; j++) {
 			double theta_diff_interpolant = (j == 0) ? 1. - theta_diff_a : theta_diff_a;
 			for (int k = 0; k < 2; k++) {
 				double theta_half_interpolant = (k == 0) ? 1. - theta_half_a : theta_half_a;
 				int ind = (phi_diff_i[i]) +
 					(theta_diff_j[j]) * merl::index::thetaD +
 					(theta_half_k[k]) * merl::index::thetaH;
 				double interpolant =  phi_diff_interpolant * theta_diff_interpolant * theta_half_interpolant;
 
 				double red_val_tmp = (Float)data[ind] * merl::scaling::red;
 				double green_val_tmp = (Float)data[ind + merl::storage::canalSize] * merl::scaling::green;
 				double blue_val_tmp = (Float)data[ind + merl::storage::twoCanalSize] * merl::scaling::blue;
 
 				if (red_val_tmp >= 0.0 && green_val_tmp >= 0.0 && blue_val_tmp >= 0.0) {
 					red_val += red_val_tmp * interpolant;
 					green_val += green_val_tmp * interpolant;
 					blue_val += blue_val_tmp * interpolant;
 				}
 			}
 		}
 	}
 	result.fromLinearRGB(red_val, green_val, blue_val);
	return result;
}

MTS_NAMESPACE_END

#endif /* __MERL_DATA_H__ */
