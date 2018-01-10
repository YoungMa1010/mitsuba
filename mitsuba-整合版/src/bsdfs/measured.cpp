/*
	This file is part of Mitsuba, a physically based rendering system.

	Original copyright (c) 2007-2014 by Wenzel Jakob and others.
	Copyright (c) 2014 Pierre Moreau, Jean-Dominique Gascuel, maverick.inria.fr

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

#include <mitsuba/core/fresolver.h>
#include <mitsuba/render/bsdf.h>
#include <mitsuba/core/spectrum.h>
#include <mitsuba/render/shape.h>
#include <mitsuba/hw/basicshader.h>
#include <mitsuba/core/plugin.h>
#include <mitsuba/core/warp.h>
#include <string>
#include <cmath>

#include "merl.h"
#include "ctd.h"
#include <boost/math/special_functions/gamma.hpp>

#define M_2PI 6.2831853072

MTS_NAMESPACE_BEGIN

const int numSamplesDC = 256;
const Float distributionConstant[numSamplesDC] = {0, 9.32554e-68, 2.91892e-27, 7.39356e-16, 8.59034e-11, 4.97486e-08, 2.47968e-06, 3.3303e-05, 0.000206516, 0.000785737, 0.00215869, 0.00472973, 0.00880765, 0.0145448, 0.0219333, 0.030837, 0.0410368, 0.0522724, 0.0642754, 0.0767913, 0.0895925, 0.102484, 0.115304, 0.127924, 0.140245, 0.15219, 0.163708, 0.174761, 0.185329, 0.1954, 0.204974, 0.214055, 0.222653, 0.230782, 0.238459, 0.245702, 0.25253, 0.258963, 0.265021, 0.270724, 0.27609, 0.28114, 0.28589, 0.290358, 0.294562, 0.298516, 0.302235, 0.305734, 0.309026, 0.312124, 0.315038, 0.317781, 0.320363, 0.322792, 0.32508, 0.327233, 0.329261, 0.331171, 0.332969, 0.334662, 0.336257, 0.337759, 0.339173, 0.340505, 0.34176, 0.342942, 0.344055, 0.345102, 0.346089, 0.347017, 0.347891, 0.348714, 0.349487, 0.350215, 0.350899, 0.351542, 0.352146, 0.352713, 0.353245, 0.353744, 0.354212, 0.35465, 0.355061, 0.355445, 0.355803, 0.356138, 0.35645, 0.356741, 0.357011, 0.357262, 0.357494, 0.35771, 0.357908, 0.358091, 0.358259, 0.358413, 0.358553, 0.358681, 0.358796, 0.3589, 0.358993, 0.359075, 0.359148, 0.359211, 0.359265, 0.359311, 0.359348, 0.359378, 0.3594, 0.359416, 0.359425, 0.359427, 0.359424, 0.359415, 0.3594, 0.35938, 0.359355, 0.359326, 0.359292, 0.359254, 0.359212, 0.359167, 0.359117, 0.359064, 0.359008, 0.358949, 0.358887, 0.358822, 0.358754, 0.358684, 0.358612, 0.358537, 0.358461, 0.358382, 0.358301, 0.358218, 0.358134, 0.358048, 0.357961, 0.357872, 0.357782, 0.357691, 0.357598, 0.357504, 0.35741, 0.357314, 0.357217, 0.35712, 0.357021, 0.356922, 0.356822, 0.356722, 0.356621, 0.356519, 0.356417, 0.356315, 0.356212, 0.356108, 0.356005, 0.355901, 0.355796, 0.355692, 0.355587, 0.355482, 0.355377, 0.355272, 0.355167, 0.355061, 0.354956, 0.35485, 0.354745, 0.354639, 0.354534, 0.354428, 0.354323, 0.354218, 0.354113, 0.354008, 0.353903, 0.353798, 0.353693, 0.353589, 0.353485, 0.353381, 0.353277, 0.353173, 0.35307, 0.352967, 0.352864, 0.352761, 0.352659, 0.352557, 0.352455, 0.352353, 0.352252, 0.352151, 0.352051, 0.35195, 0.35185, 0.351751, 0.351651, 0.351552, 0.351454, 0.351355, 0.351258, 0.35116, 0.351063, 0.350966, 0.350869, 0.350773, 0.350677, 0.350582, 0.350487, 0.350392, 0.350298, 0.350204, 0.35011, 0.350017, 0.349924, 0.349832, 0.34974, 0.349648, 0.349557, 0.349466, 0.349375, 0.349285, 0.349195, 0.349106, 0.349017, 0.348928, 0.34884, 0.348752, 0.348665, 0.348578, 0.348491, 0.348405, 0.348319, 0.348233, 0.348148, 0.348063, 0.347979, 0.347895, 0.347811, 0.347728, 0.347645, 0.347562, 0.34748, 0.347398, 0.347317, 0.347236, 0.347155, 0.347075, 0.346995, 0.346916, 0.346836, 0.346758};


//////////////////////////////////////////////////////////////////////////////
/*!\plugin{measured}{Material from MERL-MIT database}
 * \order{7}
 * \parameters{
 *     \parameter{filepath}{\String}{Path to the measured material file}
 * }
 * \vspace{4mm}
 * This plugin is used to represent measured BSDF from the MERL-MIT database.
 * \renderings{
 *     \rendering{Nickel}
 *         {bsdf_measured_nickel.jpg}
 *     \rendering{Gold-paint}
 *         {bsdf_measured_gold-paint.jpg}
 * }
 *
 * When using this plugin, you should ideally compile Mitsuba with support for
 * spectral rendering to get the most accurate results. While it also works
 * in RGB mode, the computations will be more approximate in nature.
 * Also note that this material is one-sided---that is, observed from the
 * back side, it will be completely black. If this is undesirable,
 * consider using the \pluginref{twosided} BRDF adapter.
 */
class Measured : public BSDF {
public:
	Measured(const Properties &props) : BSDF(props) {
		ref<FileResolver> fResolver = Thread::getThread()->getFileResolver();

		m_filepath = props.getString("filepath");
		if (m_filepath.empty())
			SLog(EError,
				 "Measured property \"filepath\" can not be an empty string.");

		const size_t beginMaterial = m_filepath.find_last_of("/\\") + 1;
		const size_t endMaterial = m_filepath.find_last_of(".") - beginMaterial;

		m_material = m_filepath.substr(beginMaterial, endMaterial);
		m_data = lookupMERL(m_material, fResolver->resolve(m_filepath));

		Properties sampProps("independent");
		sampProps.setInteger("sampleCount", 1);

		// Importance sampling using CTD equivalent
		{
			CTDData mat; 
			lookupCTDData(m_material, mat);
			// Approximate energy for micro-facet and diffraction lobe
			// Used to compute m_probDiffuse
			Float sAvg = 0;
			Float eta = (mat.eta[0] + mat.eta[1] + mat.eta[2])/3.;
			Float eta2pk2 = eta * eta;
			if (mat.type == conductor) {
				Float k = (mat.k[0] + mat.k[1] + mat.k[2])/3.;
				eta2pk2 += k * k;
			}
			Float F0 = (eta2pk2 - 2. * eta + 1.)
				/ (eta2pk2 + 2. * eta + 1.);
			sAvg= F0;

			// Approximate energy for diffuse lobe
			Float dAvg = 0;
			if (mat.type == subsurface) {
				Float Fdr =  fresnelDiffuseReflectance(eta, true);
				Float A = (1 + Fdr) / (1 - Fdr);
				Float albedo = (mat.albedo[0] + mat.albedo[1] + mat.albedo[2])/3.; 
				if (albedo > 1) albedo = 1;
				Float sqr_3 = sqrt(3 * (1 - albedo)); 
				Float Rd = (1 + exp(- (4./3.) * A * sqr_3)) * exp(- sqr_3);
				dAvg = (1 - F0) * (1 - F0) * albedo * 0.5 * ( 1 + Rd);
			} else if (mat.type == plastic) {
				Float albedo = (mat.albedo[0] + mat.albedo[1] + mat.albedo[2])/3.; 
				dAvg =  (1 - F0) * (1 - F0) * albedo; 
			}
			m_probDiffuse = dAvg / (dAvg + sAvg);
			// Shape distribution parameters
			m_sigma_s = mat.sigma_s;
			m_sigma = mat.b;
			m_p = mat.p; 
			m_alpha = mat.alpha;
			m_c = mat.c;
		}
		configure();
	}

	Measured(Stream *stream, InstanceManager *manager) : BSDF(stream, manager) {
		ref<FileResolver> fResolver = Thread::getThread()->getFileResolver();

		m_filepath = stream->readString();
		m_material = stream->readString();
		m_data = lookupMERL(m_material, fResolver->resolve(m_filepath));

		Properties props("independent");
		props.setInteger("sampleCount", 1);

		m_probDiffuse = stream->readFloat();
		m_sigma_s = stream->readFloat();
		m_sigma = stream->readFloat();
		m_p = stream->readFloat();
		m_alpha = stream->readFloat();
		m_c = stream->readFloat();

		configure();
	}

	void configure() {
		m_components.clear();
		m_components.push_back(EGlossyReflection);

		m_usesRayDifferentials = true;
		m_ensureEnergyConservation = true;

		BSDF::configure();
	}

	Spectrum sample(BSDFSamplingRecord &bRec, const Point2 &sample) const {
		Float _pdf = 0.0;
		return this->sample(bRec, _pdf, sample);
	}

	Spectrum sample(BSDFSamplingRecord &bRec, Float &_pdf,
					const Point2 &_sample) const {
		Point2 sample(_sample);
		if (Frame::cosTheta(bRec.wi) <= 0.0 ||
			!(bRec.typeMask & EGlossyReflection))
			return Spectrum(0.0);

		if (sample.x < m_probDiffuse) {
			// sampling the diffuse lobe. squareToCosineHemisphere
			// rescaling the sample:
			sample.x /= m_probDiffuse;
			bRec.wo = warp::squareToCosineHemisphere(sample);
			bRec.sampledType = EDiffuseReflection;
		} else { 
			// sampling the specular + diffraction lobe
			// rescaling the sample:
			sample.x = (sample.x - m_probDiffuse) / (1. - m_probDiffuse);
			// Which one should we sample? 
			Float exp_term = M_2PI * m_sigma_s * Frame::cosTheta(bRec.wi) / 0.526; 
			exp_term *= exp_term;
			Float A_spec = exp(- exp_term);
			if (sample.x < A_spec) {
				// rescale sample:
				sample.x /= A_spec;
				// sample Cook-Torrance lobe
				// Inverse of D function
				Float thetaM =
					atan(m_sigma * pow(boost::math::gamma_q_inv(1. / m_p, sample.y),
								0.5 / m_p));
				Float phiM = Float(M_2PI) * sample.x;
				Float sinThetaM = std::sin(thetaM);

				Vector m(std::cos(phiM) * sinThetaM, std::sin(phiM) * sinThetaM,
						std::cos(thetaM));

				Float wiScalarM = dot(bRec.wi, m);

				bRec.wo = 2 * wiScalarM * m - bRec.wi;
				bRec.sampledType = EGlossyReflection;
			} else {
				// Sampling the diffraction lobe
				// Rescaling the sample:
				sample.x = (sample.x - A_spec) / (1 - A_spec);
				// adapted sampling:
				Float one_over_alpha_lambda =  m_alpha / 0.526;
				Float one_over_alpha_lambda2 = one_over_alpha_lambda * one_over_alpha_lambda;
				Float one_plus_sin_theta_i_2 = 1 + Frame::sinTheta(bRec.wi);
				one_plus_sin_theta_i_2 *= one_plus_sin_theta_i_2;
				Float M = 1. + one_plus_sin_theta_i_2 * one_over_alpha_lambda2;
				M = 1. - pow(M, -0.5 * (m_c - 1));
				M = 1. - sample.x * M; 
				M = pow(M, -2./(m_c - 1)) - 1.;
				Float r = sqrt(M) / one_over_alpha_lambda;
				Float maxCosPhi = (r * r + Frame::sinTheta(bRec.wi)*Frame::sinTheta(bRec.wi) - 1.)/(2. * r *  Frame::sinTheta(bRec.wi));
				if (maxCosPhi < -1) maxCosPhi = -1;
				Float phiMax = std::acos(maxCosPhi); 
				Float phi_i = std::atan2(Frame::sinPhi(bRec.wi) , Frame::cosPhi(bRec.wi));
				Float phi = phi_i + M_PI + (2 *  sample.y - 1.) * phiMax;
				// Okay, now we have d_p = (r,phi)
				// o_p = - i_p + d_p
				bRec.wo = - Vector(Frame::sinTheta(bRec.wi) * Frame::cosPhi(bRec.wi), Frame::sinTheta(bRec.wi) * Frame::sinPhi(bRec.wi), 0.0) 
					+ Vector(r * std::cos(phi), r * std::sin(phi), 0.0); 
				// Everything good, but some numerical stability issues:
				Float l = bRec.wo.lengthSquared();
				if (l > 1) {
					bRec.wo /= sqrt(l); 
					bRec.wo.z = 0;
				} else bRec.wo.z = sqrt(1. - l);
				bRec.sampledType = EGlossyReflection;
			}
		}
		
		bRec.eta = 1.0;
		bRec.sampledComponent = 0;

		if (Frame::cosTheta(bRec.wo) <= 0.0)
			return Spectrum(0.0);

		_pdf = pdf(bRec, ESolidAngle);
		if (_pdf < 1e-20f)
			return Spectrum(0.0);
		else
			return eval(bRec, ESolidAngle) / _pdf;
	}

	Spectrum eval(const BSDFSamplingRecord &bRec, EMeasure measure) const {
		if (Frame::cosTheta(bRec.wi) <= 0.0 ||
			Frame::cosTheta(bRec.wo) <= 0.0 || measure != ESolidAngle ||
			!(bRec.typeMask & EGlossyReflection))
			return Spectrum(0.0);

		Spectrum bsdf(
			lookupSpectrum(m_data, bRec.wi, normalize(bRec.wi + bRec.wo)) *
			Frame::cosTheta(bRec.wo));

		return bsdf.isValid() ? bsdf : Spectrum(0.0);
	}

	Float pdf(const BSDFSamplingRecord &bRec, EMeasure measure) const {
		if (Frame::cosTheta(bRec.wi) <= 0.0 ||
			Frame::cosTheta(bRec.wo) <= 0.0 || measure != ESolidAngle ||
			!(bRec.typeMask & EGlossyReflection))
			return 0.0;

		const Vector H(normalize(bRec.wi + bRec.wo));
		Float result = 0.0f;

		Float exp_term = M_2PI * m_sigma_s * Frame::cosTheta(bRec.wi) / 0.526; 
		exp_term *= exp_term;
		Float A_spec = exp(- exp_term);
		
		// Specular lobe weight:
		/* Jacobian of the half-direction mapping */
		const Float dwh_dwo = 1.0f / (4.0f * dot(bRec.wo, H));
		/* Evaluate the microfacet model sampling density function */
		const Float prob = d(H, bRec.its) * Frame::cosTheta(H);
		// is equivalent to distr.pdf();
		// is missing the visibleSampling opportunity
		result += prob * dwh_dwo * (1 - m_probDiffuse) * A_spec; 

		// diffraction lobe weight:
		Vector d_p = bRec.wi + bRec.wo;
		Float one_over_alpha_lambda =  m_alpha / 0.526;
		Float one_over_alpha_lambda2 = one_over_alpha_lambda * one_over_alpha_lambda;
		d_p.z = 0.0;
		Float r = d_p.length();
		Float Sz = one_over_alpha_lambda2 * pow(1 + r * r * one_over_alpha_lambda2, - 0.5 * (m_c + 1.)) * (m_c - 1.) / (2. * M_PI);
		Float maxCosPhi = (r * r + Frame::sinTheta(bRec.wi)*Frame::sinTheta(bRec.wi) - 1.)/(2. * r *  Frame::sinTheta(bRec.wi));
		if (maxCosPhi < -1) maxCosPhi = -1;
		Float phiMax = std::acos(maxCosPhi); 
		result += (1 - m_probDiffuse) * (1 - A_spec) * Frame::cosTheta(bRec.wo) * Sz / (phiMax / M_PI);

		// Diffuse Lobe weight:
		result += m_probDiffuse * warp::squareToCosineHemispherePdf(bRec.wo);

		return (result < 1e-20f) ? 0 : result;
	}


	Float d(const Vector &m, const Intersection its) const {
		if (Frame::cosTheta(m) <= 0.0)
			return 0.0;
		// x = tan^2(theta)
		const Float c2 = Frame::cosTheta(m) * Frame::cosTheta(m);
		const Float x = 1./c2 - 1; // x = tan^2 theta
        Float sigma2 = m_sigma;
        sigma2 *= sigma2;
        const Float p = m_p;
        // get precomputed value for distributionConstant : p/gamma(1/p)
        const Float kp = (p/5.0) * numSamplesDC;
        int pmin = floor(kp);
        int pmax = pmin + 1;
        Float ap = kp - pmin;
		if (pmax >= numSamplesDC)
			pmax = numSamplesDC;
		if (pmin >= numSamplesDC) {
			pmin = numSamplesDC;
			ap = 0;
		}
		if (pmin <= 0) {
			pmin = 1;
		}
		if (pmax <= 0) {
			pmax = 1;
			ap = 1;
		}
	    const Float distConstant = (1.0 - ap) * distributionConstant[pmin] + ap * distributionConstant[pmax];
	    // const Float distributionConstant = p / (sigma2 * M_PI * boost::math::tgamma(1. / p));
		return distConstant * exp(-pow(x / sigma2, p)) / (sigma2 * c2 * c2);
	}


 	Float getRoughness(const Intersection &its, int component) const {
 		// default value
 		return 333.33;
 	}
 	
	void serialize(Stream *stream, InstanceManager *manager) const {
		BSDF::serialize(stream, manager);


		stream->writeString(m_filepath);
		stream->writeString(m_material);

		stream->writeFloat(m_probDiffuse);
		stream->writeFloat(m_sigma_s);
		stream->writeFloat(m_sigma);
		stream->writeFloat(m_sigma_s);
		stream->writeFloat(m_p);
		stream->writeFloat(m_alpha);
		stream->writeFloat(m_c);
	}

	~Measured() {
		delete m_data;
		releaseMERL();
	}

	Shader *createShader(Renderer *renderer) const;

	MTS_DECLARE_CLASS()
private:
	std::string m_filepath;
	std::string m_material;

	Float m_probDiffuse;
	// Shape distribution parameters
	Float m_sigma_s;
	Float m_sigma;
	Float m_p; 
	Float m_alpha;
	Float m_c;
	
	double *m_data;
};

//////////////////////////////////////////////////////////////////////////////
class MeasuredShader : public Shader {
public:
	MeasuredShader(Renderer *renderer, CTDData data)
		: Shader(renderer, EBSDFShader), m_data(data) {}

	bool isComplete() const { 
		return m_reflectanceShader.get() != NULL;
	}

	void putDependencies(std::vector<Shader *> &deps) {
		deps.push_back(m_reflectanceShader.get());
	}

	void cleanup(Renderer *renderer) {
		renderer->unregisterShaderForResource(m_reflectance.get());
	}

	void resolve(const GPUProgram *program, const std::string &evalName,
				 std::vector<int> &parameterIDs) const {
		/*
		parameterIDs.push_back(
			program->getParameterID(evalName + "_rhoD", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_rhoS", false));
        parameterIDs.push_back(program->getParameterID(evalName + "_alpha", false));
        parameterIDs.push_back(program->getParameterID(evalName + "_kap", false));
        parameterIDs.push_back(program->getParameterID(evalName + "_p", false));
        parameterIDs.push_back(program->getParameterID(evalName + "_f0", false));
        parameterIDs.push_back(program->getParameterID(evalName + "_f1", false));
        parameterIDs.push_back(program->getParameterID(evalName + "_lambda", false));
        parameterIDs.push_back(program->getParameterID(evalName + "_c", false));
        parameterIDs.push_back(program->getParameterID(evalName + "_k", false));
        parameterIDs.push_back(program->getParameterID(evalName + "_theta0", false));
        */
    }

    void bind(GPUProgram *program, const std::vector<int> &parameterIDs, int &textureUnitOffset) const {
        /* program->setParameter(parameterIDs[ 0], m_data.rhoD);
        program->setParameter(parameterIDs[ 1], m_data.rhoS);
        program->setParameter(parameterIDs[ 2], m_data.alpha);
        program->setParameter(parameterIDs[ 3], m_data.kap);
        program->setParameter(parameterIDs[ 4], m_data.p);
        program->setParameter(parameterIDs[ 5], m_data.f0);
        program->setParameter(parameterIDs[ 6], m_data.f1);
        program->setParameter(parameterIDs[ 7], m_data.lambda);
        program->setParameter(parameterIDs[ 8], m_data.c);
        program->setParameter(parameterIDs[ 9], m_data.k);
        program->setParameter(parameterIDs[10], m_data.theta0); */
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
    
    CTDData m_data;
};

Shader *Measured::createShader(Renderer *renderer) const {
	CTDData mat;
	lookupCTDData(m_material, mat);
	return new MeasuredShader(renderer, mat);
}

MTS_IMPLEMENT_CLASS(MeasuredShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(Measured, false, BSDF)

MTS_EXPORT_PLUGIN(Measured, "Measured BRDF (MERL Database format)")
MTS_NAMESPACE_END
