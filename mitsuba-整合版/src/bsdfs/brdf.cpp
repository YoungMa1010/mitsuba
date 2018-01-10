/*
This file is part of Mitsuba, a physically based rendering system.

Copyright (c) 2007-2014 by Wenzel Jakob and others.

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

#include <mitsuba/render/bsdf.h>
#include <mitsuba/render/texture.h>
#include <mitsuba/hw/basicshader.h>
#include <mitsuba/core/warp.h>
#include <mitsuba/core/fwd.h>
#include <mitsuba/core/spectrum.h>
#include <mitsuba/core/math.h>
#include <mitsuba/core/frame.h>
#include <mitsuba/bidir/common.h>
#include <mitsuba/core/logger.h>
#include "stdlib.h"
#include <iostream>
#include "math.h"
#include <Windows.h>
#include <direct.h>
#include <string>

using namespace std;
__MITSUBA_CORE_VECTOR_H_



#define BRDF_SAMPLING_RES_THETA_H       90
#define BRDF_SAMPLING_RES_THETA_D       90
#define BRDF_SAMPLING_RES_PHI_D         360

#define RED_SCALE (1.0/1500.0)
#define GREEN_SCALE (1.15/1500.0)
#define BLUE_SCALE (1.66/1500.0)

MTS_NAMESPACE_BEGIN



inline float safe_acos(float value) {
	return std::acos(std::min(1.0f, std::max(-1.0f, value)));
}

inline static Float cosPhi(const Vector &v) {
	Float sinTheta = Frame::sinTheta(v);
	if (sinTheta == 0.0f)
		return 1.0f;
	return math::clamp(v.x / sinTheta, (Float)-1.0f, (Float) 1.0f);
}

inline static Float sinPhi(const Vector &v) {
	Float sinTheta = Frame::sinTheta(v);
	if (sinTheta == 0.0f)
		return 1.0f;
	return math::clamp(v.y / sinTheta, (Float)-1.0f, (Float) 1.0f);
}

inline static Float cosTheta(const Vector &v) {
	return v.z;
}

inline static Float sinTheta2(const Vector &v) {
	return 1.0f - v.z * v.z;
}

inline static Float sinTheta(const Vector &v) {
	Float temp = sinTheta2(v);
	if (temp <= 0.0f)
		return 0.0f;
	return std::sqrt(temp);
}


inline float SphericalPhi(const Vector &v){
	Float p = atan2f(v.y, v.x);
	return (p < 0.f) ? p + 2.f*M_PI : p;
}

inline float Clamp(float val, float low, float high){
	if (val < low) return low;
	else if (val > high) return high;
	else return val;
}

std::string getFileName(std::string str){
	char exeFullPath[MAX_PATH]; // Full path
	getcwd(exeFullPath, MAX_PATH);
	int CountOfBlanks = 0;
	for (int i = 0; i<strlen(exeFullPath); i++)
	if (exeFullPath[i] == '\\')
		++CountOfBlanks;
	int len = strlen(exeFullPath) + CountOfBlanks;
	if (len + 1>MAX_PATH)
		return 0;
	char* pStr1 = exeFullPath + strlen(exeFullPath);
	char* pStr2 = exeFullPath + len;
	while (pStr1<pStr2)
	{
		if (*pStr1 == '\\')
		{
			*pStr2-- = '/';
			*pStr2-- = '/';
		}
		else
		{
			*pStr2-- = *pStr1;
		}
		--pStr1;
	}
	string strPath = "";
	strPath = (string)exeFullPath;    // Get full path of the file
	strPath += str;
	//const char *cha = strPath.c_str();
	//SLog(EInfo, "mhy  Parsing  \"%s\" ..", cha);
	return strPath;
}
std::string str = getFileName("dark-red-paint.binary");
const char *filename = str.c_str();
//const char *filename = getFileName("dark-red-paint.binary");
//const char *filename = "I://mitsuba-original//build//dark-red-paint.binary";
double *brdf = 0;


// Read BRDF data
bool read_brdf(const char *filename, double* &brdf)
{
	FILE *f;
	fopen_s(&f, filename, "rb");
	if (!f)
		return false;

	int dims[3];
	fread(dims, sizeof(int), 3, f);
	int n = dims[0] * dims[1] * dims[2];
	if (n != BRDF_SAMPLING_RES_THETA_H *
		BRDF_SAMPLING_RES_THETA_D *
		BRDF_SAMPLING_RES_PHI_D / 2)
	{
		fprintf(stderr, "Dimensions don't match\n");
		fclose(f);
		return false;
	}

	brdf = (double*)malloc(sizeof(double)* 3 * n);
	fread(brdf, sizeof(double), 3 * n, f);

	fclose(f);
	return true;
}

bool read = read_brdf(filename, brdf);
double* makeTable(double* brdf)
{
	/*const char *filename = "G://alum-bronze.binary";*/
	/*double* brdf;*/
	// read brdf
	if (!read)
	{
		/*fprintf(stderr, "Error reading %s\n", filename);*/
		exit(1);
	}
	return brdf;
}



class alum_bronze : public BSDF {
public:
	alum_bronze(const Properties &props)
		: BSDF(props) {
		/* For better compatibility with other models, support both
		'reflectance' and 'diffuseReflectance' as parameter names */
		m_reflectance = new ConstantSpectrumTexture(props.getSpectrum(
			props.hasProperty("reflectance") ? "reflectance"
			: "diffuseReflectance", Spectrum(.5f)));
	}

	alum_bronze(Stream *stream, InstanceManager *manager)
		: BSDF(stream, manager) {
		m_reflectance = static_cast<Texture *>(manager->getInstance(stream));

		configure();
	}

	void configure() {
		/* Verify the input parameter and fix them if necessary */
		m_reflectance = ensureEnergyConservation(m_reflectance, "reflectance", 1.0f);

		m_components.clear();
		if (m_reflectance->getMaximum().max() > 0)
			m_components.push_back(EDiffuseReflection | EFrontSide
			| (m_reflectance->isConstant() ? 0 : ESpatiallyVarying));
		m_usesRayDifferentials = m_reflectance->usesRayDifferentials();

		BSDF::configure();
	}

	Spectrum getDiffuseReflectance(const Intersection &its) const {
		return m_reflectance->eval(its);
	}

	Spectrum getAlbedo(const Intersection &its) const
	{
		if (Frame::cosTheta(its.wi) <= 0)
			return Spectrum(0.0f);
		return m_reflectance->eval(its);
	}


	Spectrum get_irradiance(const BSDFSamplingRecord &bRec) const
	{
		if (!(bRec.typeMask & EDiffuseReflection) || Frame::cosTheta(bRec.wi) <= 0)
			return Spectrum(0.0f);
		//bRec.eta = 1.0f;
		//bRec.sampledComponent = 0;
		//bRec.sampledType = EDiffuseReflection;
		return m_reflectance->eval(bRec.its)/** INV_PI*/;
	}

	Spectrum eval(const BSDFSamplingRecord &bRec, EMeasure measure) const {
		if (!(bRec.typeMask & EDiffuseReflection) || measure != ESolidAngle
			|| Frame::cosTheta(bRec.wi) <= 0
			|| Frame::cosTheta(bRec.wo) <= 0)
			return Spectrum(0.0f);

		brdf = makeTable(brdf);
		uint32_t nth = 90, ntd = 90, npd = 180;
		const uint32_t nThetaH(nth), nThetaD(ntd), nPhiD(npd);
		Vector wi = bRec.wi, wo = bRec.wo;
		Vector wh = normalize(wi + wo);
		double whTheta = safe_acos(wh.z);
		Float whCosPhi = cosPhi(wh), whSinPhi = sinPhi(wh);
		Float whCosTheta = cosTheta(wh), whSinTheta = sinTheta(wh);
		Vector whx(whCosPhi * whCosTheta, whSinPhi * whCosTheta, -whSinTheta);
		Vector why(-whSinPhi, whCosPhi, 0);
		Vector wd(dot(wi, whx), dot(wi, why), dot(wi, wh));
		//Compute index into measured BRDF tables                                       
		double wdTheta = safe_acos(wd.z); //SphericalTheta(wd)
		float wdPhi = SphericalPhi(wd);
		if (wdPhi > M_PI) wdPhi -= M_PI;
		//Compute indices whThetaIndex, wdThetaIndex, wdPhiIndex                          
#define REMAP(V, MAX, COUNT) \
	Clamp(int((V) / (MAX)* (COUNT)), 0, (COUNT)-1)
		int whThetaIndex = REMAP(sqrtf(MAX(0.f, whTheta / (M_PI / 2.f))),
			1.f, nThetaH);
		int wdThetaIndex = REMAP(wdTheta, M_PI / 2.f, nThetaD);
		int wdPhiIndex = REMAP(wdPhi, M_PI, nPhiD);
#undef REMAP
		int index = wdPhiIndex + nPhiD * (wdThetaIndex + whThetaIndex * nThetaD);

		Spectrum col;
		col[0] = brdf[index] * RED_SCALE;
		col[1] = brdf[index + BRDF_SAMPLING_RES_THETA_H*BRDF_SAMPLING_RES_THETA_D*BRDF_SAMPLING_RES_PHI_D / 2] * GREEN_SCALE;
		col[2] = brdf[index + BRDF_SAMPLING_RES_THETA_H*BRDF_SAMPLING_RES_THETA_D*BRDF_SAMPLING_RES_PHI_D] * BLUE_SCALE;

		col.fromLinearRGB(col[0], col[1], col[2], Spectrum::EReflectance);

		return col;

	}

	Float pdf(const BSDFSamplingRecord &bRec, EMeasure measure) const {
		if (!(bRec.typeMask & EDiffuseReflection) || measure != ESolidAngle
			|| Frame::cosTheta(bRec.wi) <= 0
			|| Frame::cosTheta(bRec.wo) <= 0)
			return 0.0f;

		return warp::squareToCosineHemispherePdf(bRec.wo);
	}

	Spectrum sample(BSDFSamplingRecord &bRec, const Point2 &sample) const {
		if (!(bRec.typeMask & EDiffuseReflection) || Frame::cosTheta(bRec.wi) <= 0)
			return Spectrum(0.0f);

		bRec.wo = warp::squareToCosineHemisphere(sample);
		bRec.eta = 1.0f;
		bRec.sampledComponent = 0;
		bRec.sampledType = EDiffuseReflection;
		return m_reflectance->eval(bRec.its);
	}

	Spectrum sample(BSDFSamplingRecord &bRec, Float &pdf, const Point2 &sample) const {
		if (!(bRec.typeMask & EDiffuseReflection) || Frame::cosTheta(bRec.wi) <= 0)
			return Spectrum(0.0f);

		bRec.wo = warp::squareToCosineHemisphere(sample);
		bRec.eta = 1.0f;
		bRec.sampledComponent = 0;
		bRec.sampledType = EDiffuseReflection;
		pdf = warp::squareToCosineHemispherePdf(bRec.wo);
		return m_reflectance->eval(bRec.its);
	}

	void addChild(const std::string &name, ConfigurableObject *child) {
		if (child->getClass()->derivesFrom(MTS_CLASS(Texture))
			&& (name == "reflectance" || name == "diffuseReflectance")) {
			m_reflectance = static_cast<Texture *>(child);
		}
		else {
			BSDF::addChild(name, child);
		}
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		BSDF::serialize(stream, manager);

		manager->serialize(stream, m_reflectance.get());
	}

	Float getRoughness(const Intersection &its, int component) const {
		return std::numeric_limits<Float>::infinity();
	}

	std::string toString() const {
		std::ostringstream oss;
		oss << "SmoothDiffuse[" << endl
			<< "  id = \"" << getID() << "\"," << endl
			<< "  reflectance = " << indent(m_reflectance->toString()) << endl
			<< "]";
		return oss.str();
	}

	Shader *createShader(Renderer *renderer) const;

	MTS_DECLARE_CLASS()
private:

	ref<Texture> m_reflectance;
};

// ================ Hardware shader implementation ================

class alum_bronzeShader : public Shader {
public:
	alum_bronzeShader(Renderer *renderer, const Texture *reflectance)
		: Shader(renderer, EBSDFShader), m_reflectance(reflectance) {
		m_reflectanceShader = renderer->registerShaderForResource(m_reflectance.get());
	}

	bool isComplete() const {
		return m_reflectanceShader.get() != NULL;
	}

	void cleanup(Renderer *renderer) {
		renderer->unregisterShaderForResource(m_reflectance.get());
	}

	void putDependencies(std::vector<Shader *> &deps) {
		deps.push_back(m_reflectanceShader.get());
	}

	void generateCode(std::ostringstream &oss,
		const std::string &evalName,
		const std::vector<std::string> &depNames) const {
		oss << "vec3 " << evalName << "(vec2 uv, vec3 wi, vec3 wo) {" << endl
			<< "    if (cosTheta(wi) < 0.0 || cosTheta(wo) < 0.0)" << endl
			<< "    	return vec3(0.0);" << endl
			<< "    return " << depNames[0] << "(uv) * inv_pi * cosTheta(wo);" << endl
			<< "}" << endl
			<< endl
			<< "vec3 " << evalName << "_diffuse(vec2 uv, vec3 wi, vec3 wo) {" << endl
			<< "    return " << evalName << "(uv, wi, wo);" << endl
			<< "}" << endl;
	}

	MTS_DECLARE_CLASS()
private:
	ref<const Texture> m_reflectance;
	ref<Shader> m_reflectanceShader;
};

Shader *alum_bronze::createShader(Renderer *renderer) const {
	return new alum_bronzeShader(renderer, m_reflectance.get());
}

MTS_IMPLEMENT_CLASS(alum_bronzeShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(alum_bronze, false, BSDF)
MTS_EXPORT_PLUGIN(alum_bronze, "Smooth diffuse BRDF")
MTS_NAMESPACE_END
