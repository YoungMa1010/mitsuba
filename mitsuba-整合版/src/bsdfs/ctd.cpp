/* 
	This file is part of Mitsuba, a physically based rendering system.

	Original copyright (c) 2007-2014 by Wenzel Jakob and others.
 	Copyright (c) 2014 Pierre Moreau, Jean-Dominique Gascuel, maverick.inria.fr
 	Copyright (c) 2015 Nicolas Holzschuch, Romain Pacanowski, maverick.inria.fr

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
#include <mitsuba/core/plugin.h>
#include <mitsuba/core/warp.h>
#include <mitsuba/hw/basicshader.h>
#include <mitsuba/render/bsdf.h>
#include <boost/math/special_functions/gamma.hpp>
#include <complex>
#include <cmath>
#include <boost/algorithm/string.hpp>
#include "ctd.h"

// For shadowing:
const int numSamplesp = 100;
const int numSamplesu = 1000;
#include "ctd_G1.h"
#include "ctd_renormalization.h"
#include "ctd_lowPass.h"
#include "ctd_convolution.h"
#include "ctd_fresnel_R.h"
#include "ior.h"
// precomputed values for p / (pi * gamma(1/p))
#define M_2PI 6.2831853072
const std::complex<float> i(0,1);
MTS_NAMESPACE_BEGIN

const int numSamplesDC = 256;
// precomputed value for ((p  * gsl_sf_gammainv(1./p)) / M_PI)
const Float distributionConstant[numSamplesDC] = {0, 9.32554e-68, 2.91892e-27, 7.39356e-16, 8.59034e-11, 4.97486e-08, 2.47968e-06, 3.3303e-05, 0.000206516, 0.000785737, 0.00215869, 0.00472973, 0.00880765, 0.0145448, 0.0219333, 0.030837, 0.0410368, 0.0522724, 0.0642754, 0.0767913, 0.0895925, 0.102484, 0.115304, 0.127924, 0.140245, 0.15219, 0.163708, 0.174761, 0.185329, 0.1954, 0.204974, 0.214055, 0.222653, 0.230782, 0.238459, 0.245702, 0.25253, 0.258963, 0.265021, 0.270724, 0.27609, 0.28114, 0.28589, 0.290358, 0.294562, 0.298516, 0.302235, 0.305734, 0.309026, 0.312124, 0.315038, 0.317781, 0.320363, 0.322792, 0.32508, 0.327233, 0.329261, 0.331171, 0.332969, 0.334662, 0.336257, 0.337759, 0.339173, 0.340505, 0.34176, 0.342942, 0.344055, 0.345102, 0.346089, 0.347017, 0.347891, 0.348714, 0.349487, 0.350215, 0.350899, 0.351542, 0.352146, 0.352713, 0.353245, 0.353744, 0.354212, 0.35465, 0.355061, 0.355445, 0.355803, 0.356138, 0.35645, 0.356741, 0.357011, 0.357262, 0.357494, 0.35771, 0.357908, 0.358091, 0.358259, 0.358413, 0.358553, 0.358681, 0.358796, 0.3589, 0.358993, 0.359075, 0.359148, 0.359211, 0.359265, 0.359311, 0.359348, 0.359378, 0.3594, 0.359416, 0.359425, 0.359427, 0.359424, 0.359415, 0.3594, 0.35938, 0.359355, 0.359326, 0.359292, 0.359254, 0.359212, 0.359167, 0.359117, 0.359064, 0.359008, 0.358949, 0.358887, 0.358822, 0.358754, 0.358684, 0.358612, 0.358537, 0.358461, 0.358382, 0.358301, 0.358218, 0.358134, 0.358048, 0.357961, 0.357872, 0.357782, 0.357691, 0.357598, 0.357504, 0.35741, 0.357314, 0.357217, 0.35712, 0.357021, 0.356922, 0.356822, 0.356722, 0.356621, 0.356519, 0.356417, 0.356315, 0.356212, 0.356108, 0.356005, 0.355901, 0.355796, 0.355692, 0.355587, 0.355482, 0.355377, 0.355272, 0.355167, 0.355061, 0.354956, 0.35485, 0.354745, 0.354639, 0.354534, 0.354428, 0.354323, 0.354218, 0.354113, 0.354008, 0.353903, 0.353798, 0.353693, 0.353589, 0.353485, 0.353381, 0.353277, 0.353173, 0.35307, 0.352967, 0.352864, 0.352761, 0.352659, 0.352557, 0.352455, 0.352353, 0.352252, 0.352151, 0.352051, 0.35195, 0.35185, 0.351751, 0.351651, 0.351552, 0.351454, 0.351355, 0.351258, 0.35116, 0.351063, 0.350966, 0.350869, 0.350773, 0.350677, 0.350582, 0.350487, 0.350392, 0.350298, 0.350204, 0.35011, 0.350017, 0.349924, 0.349832, 0.34974, 0.349648, 0.349557, 0.349466, 0.349375, 0.349285, 0.349195, 0.349106, 0.349017, 0.348928, 0.34884, 0.348752, 0.348665, 0.348578, 0.348491, 0.348405, 0.348319, 0.348233, 0.348148, 0.348063, 0.347979, 0.347895, 0.347811, 0.347728, 0.347645, 0.347562, 0.34748, 0.347398, 0.347317, 0.347236, 0.347155, 0.347075, 0.346995, 0.346916, 0.346836, 0.346758};

// Maximum values for b, as a function of p. p going from 5.0 to 0.15, delta_p = 0.05
const Float bMax[98] = {0.058, 0.059, 0.059, 0.06, 0.06, 0.06, 0.061, 0.061, 0.062, 0.062, 0.063, 0.063, 0.064, 0.064, 0.065, 0.065, 0.066, 0.066, 0.067, 0.068, 0.068, 0.069, 0.07, 0.07, 0.071, 0.072, 0.073, 0.074, 0.074, 0.075, 0.076, 0.077, 0.078, 0.079, 0.081, 0.082, 0.083, 0.084, 0.084, 0.085, 0.085, 0.086, 0.086, 0.087, 0.087, 0.088, 0.089, 0.089, 0.09, 0.091, 0.092, 0.092, 0.093, 0.094, 0.095, 0.096, 0.098, 0.099, 0.1, 0.102, 0.104, 0.106, 0.108, 0.11, 0.113, 0.116, 0.119, 0.123, 0.127, 0.132, 0.138, 0.145, 0.153, 0.163, 0.175, 0.191, 0.211, 0.236, 0.268, 0.308, 0.348, 0.388, 0.416, 0.441, 0.461, 0.472, 0.47, 0.469, 0.448, 0.418, 0.389, 0.327, 0.261, 0.195, 0.121, 0.055, 0.015, 0.001}; // your personal healthcare companion



class CombinedDiffraction : public BSDF {
	public:
	CombinedDiffraction(const Properties &props) : BSDF(props) {
		ref<FileResolver> fResolver = Thread::getThread()->getFileResolver();
		
		Properties sampProps("independent");
		sampProps.setInteger("sampleCount", 1);

		Spectrum intEta, intK;
		intEta = Spectrum(0.0f);
		intK = Spectrum(1.0f);

		m_type = conductor;
		if (props.hasProperty("material")) {
			m_materialName = props.getString("material");
			CTDData data; 
			if (lookupCTDData(m_materialName, data)) {
				m_type = data.type;
				intEta = data.eta;
				if (m_type == conductor) {
					intK = data.k;
				} else {
					m_albedo = new ConstantSpectrumTexture(data.albedo);
				}
				m_sigma = new ConstantFloatTexture(data.b);
				m_p = new ConstantFloatTexture(data.p);
				m_sigma_s = new ConstantFloatTexture(data.sigma_s);
				m_alpha = new ConstantFloatTexture(data.alpha);
				m_c = new ConstantFloatTexture(data.c);
				m_fixMERLAtGrazing = true;
			} else {
				// There is a material name, but it's not one of the MERL database.
				// Lookup its spectrum:
				intEta.fromContinuousSpectrum(InterpolatedSpectrum(
							fResolver->resolve("data/ior/" + m_materialName + ".eta.spd")));
				intK.fromContinuousSpectrum(InterpolatedSpectrum(
							fResolver->resolve("data/ior/" + m_materialName + ".k.spd")));
				// Extract values for 645/526/444 nm:
			}
		} 
		// Properties specified in scene file override the "material" property
		if (props.hasProperty("materialType")) {
			std::string typeName = boost::to_lower_copy(props.getString("materialType"));
			if (typeName == "conductor")
				m_type = conductor;
			else if (typeName == "subsurface")
				m_type = subsurface;
			else if (typeName == "plastic")
				m_type = plastic;
			else
				SLog(EError, "Specified an invalid material type \"%s\", must be "
						"\"conductor\", \"subsurface\", or \"plastic\"/!", typeName.c_str());
		}
		
		Float extEta = lookupIOR(props, "extEta", "air");
		m_eta = new ConstantSpectrumTexture(props.getSpectrum("eta", intEta) / extEta);
		if (m_type == conductor) {
			m_k   = new ConstantSpectrumTexture(props.getSpectrum("k", intK) / extEta);
		} else {
			if (props.hasProperty("albedo"))
				m_albedo = new ConstantSpectrumTexture(props.getSpectrum("albedo"));
		}
		if (props.hasProperty("b"))
			m_sigma = new ConstantSpectrumTexture(props.getSpectrum("b"));
		if (props.hasProperty("p"))
			m_p = new ConstantSpectrumTexture(props.getSpectrum("p"));
		if (props.hasProperty("sigma_s"))
			m_sigma_s = new ConstantSpectrumTexture(props.getSpectrum("sigma_s"));
		if (props.hasProperty("alpha"))
			m_alpha = new ConstantSpectrumTexture(props.getSpectrum("alpha"));
		if (props.hasProperty("c"))
			m_c = new ConstantSpectrumTexture(props.getSpectrum("c"));
		if (props.hasProperty("fixMERLAtGrazing"))
			m_fixMERLAtGrazing = props.getBoolean("fixMERLAtGrazing");

		configure();
	}

	CombinedDiffraction(Stream *stream, InstanceManager *manager)
		: BSDF(stream, manager), m_materialName(stream->readString()) {
		Properties props("independent");
		props.setInteger("sampleCount", 1);

		m_eta = static_cast<Texture *>(manager->getInstance(stream));
		m_type = (materialType) stream->readUInt();
		if (m_type == conductor) {
			m_k = static_cast<Texture *>(manager->getInstance(stream));
		} else {
			m_albedo = static_cast<Texture *>(manager->getInstance(stream));
		}
		m_sigma = static_cast<Texture *>(manager->getInstance(stream));
		m_p = static_cast<Texture *>(manager->getInstance(stream));
		m_sigma_s = static_cast<Texture *>(manager->getInstance(stream));
		m_alpha = static_cast<Texture *>(manager->getInstance(stream));
		m_c = static_cast<Texture *>(manager->getInstance(stream));
		configure();
	}

	void configure() {
		m_components.clear();
		m_components.push_back(EGlossyReflection); // component == 0: specular
		m_components.push_back(EGlossyReflection); // component == 1: diffraction
		// For the time being, no multiple scattering component
		if (m_type != conductor) 
			m_components.push_back(EDiffuseReflection);// component == 2: diffuse
		m_usesRayDifferentials = true;

		/* Compute weights that further steer samples towards
		   the specular, diffuse or diffraction components */

		// First job: estimate energy for all components
		m_1_lambda.lambdaValuesPower(-1.); // Spectrum = 1 / wavelength
		m_1_lambda2.lambdaValuesPower(-2.); // Spectrum = 1 / wavelength^2
		m_centralComponent = SPECTRUM_SAMPLES / 2; 

		Spectrum eta2pk2 = m_eta->getAverage() * m_eta->getAverage();
		if (m_type == conductor) eta2pk2 += m_k->getAverage() * m_k->getAverage();
		Spectrum F0 = (eta2pk2 - 2. * m_eta->getAverage() + Spectrum(1.))
			/ (eta2pk2 + 2. * m_eta->getAverage() + Spectrum(1.));

		// Approximate energy for micro-facet and diffraction lobe
		Float sAvg = 0;
		if (m_sigma->getAverage().getLuminance() > 0.0) {
			sAvg= F0.getLuminance();
		}

		// Approximate energy for diffuse lobe
		Float dAvg = 0;
	    if (m_type == subsurface) {
			Float Fdr =  fresnelDiffuseReflectance(m_eta->getAverage().getLuminance(), true);
			Float A = (1 + Fdr) / (1 - Fdr);
	    	Float albedo = m_albedo->getAverage().getLuminance();
			if (albedo > 1) albedo = 1;
	    	Float sqr_3 = sqrt(3 * (1 - albedo)); 
	    	Float Rd = (1 + exp(- (4./3.) * A * sqr_3)) * exp(- sqr_3);
           dAvg = (1 - F0.getLuminance()) * (1 - F0.getLuminance()) * m_albedo->getAverage().getLuminance() * 0.5 * ( 1 + Rd);
		} else if (m_type == plastic) {
	    	Float albedo = m_albedo->getAverage().getLuminance() ;
			dAvg =  (1 - F0.getLuminance()) * (1 - F0.getLuminance()) * albedo; 
		}
		m_probDiffuse = dAvg / (dAvg + sAvg);

		BSDF::configure();
	}

	Spectrum eval(const BSDFSamplingRecord &bRec, EMeasure measure) const {
		bool hasSpecular = (bRec.typeMask & EGlossyReflection) &&
						   (bRec.component == -1 || bRec.component == 0) && 
						   (m_sigma->eval(bRec.its).average() > 0) ;
		bool hasDiffraction = (bRec.typeMask & EGlossyReflection) &&
							  (bRec.component == -1 || bRec.component == 1);
		bool hasDiffuse = (bRec.typeMask & EDiffuseReflection) &&
						  (bRec.component == -1 || bRec.component == 2)
						  && (m_type != conductor);

		if (measure != ESolidAngle || Frame::cosTheta(bRec.wi) <= 0 ||
			Frame::cosTheta(bRec.wo) <= 0 ||
			(!hasSpecular && !hasDiffuse && !hasDiffraction))
			return Spectrum(0.0f);

		Spectrum result(0.0f);
		Spectrum value;
		Spectrum diffuse;

		// average surface roughness
		Vector h = normalize(bRec.wi + bRec.wo);
		Float cos_theta_d = dot(bRec.wi, h); 
		// Multiplicative factor for specular reflection
		// A = portion of energy going into specular beam
		Spectrum A_specular(1.0); 
		Spectrum A_diffraction(0.0);

		if (hasDiffuse && (m_type == subsurface)) {
			Float Fdr =  fresnelDiffuseReflectance(m_eta->eval(bRec.its).getLuminance(), true);
			Float A = (1 + Fdr) / (1 - Fdr);
			Spectrum sqrt_3_1_a = (3 * (Spectrum(1.) - m_albedo->eval(bRec.its)));
			sqrt_3_1_a = sqrt_3_1_a.pow(0.5);
			sqrt_3_1_a *= -1.;
			Spectrum Rd = 0.5 * INV_PI * (Spectrum(1.) + ((4./3.) * A * sqrt_3_1_a).exp()) * sqrt_3_1_a.exp();

			Spectrum cos_theta_o_inside = (Spectrum(1.) - (Spectrum(Frame::sinTheta(bRec.wo))/m_eta->eval(bRec.its) *  Spectrum(Frame::sinTheta(bRec.wo))/m_eta->eval(bRec.its)));
			cos_theta_o_inside = cos_theta_o_inside.sqrt();
			Spectrum cos_theta_i_inside = (Spectrum(1.) - (Spectrum(Frame::sinTheta(bRec.wi))/m_eta->eval(bRec.its) *  Spectrum(Frame::sinTheta(bRec.wi))/m_eta->eval(bRec.its)));
			cos_theta_i_inside = cos_theta_i_inside.sqrt();
			Spectrum single = Spectrum(0.25 * INV_PI) / (cos_theta_o_inside + cos_theta_i_inside);

		 	result += m_albedo->eval(bRec.its) * (Spectrum(1.) - fresnel(Frame::cosTheta(bRec.wi), bRec.its)) * (Spectrum(1.) - fresnelInverted(cos_theta_o_inside, bRec.its)) * (Spectrum(single) + Rd) * Frame::cosTheta(bRec.wo);
		}
		if (hasDiffuse && (m_type == plastic)) {
			Float Fdr =  fresnelDiffuseReflectance(1.0 / m_eta->eval(bRec.its).getLuminance(), true);
			Spectrum cos_theta_o_inside = (Spectrum(1.) - (Spectrum(Frame::sinTheta(bRec.wo))/m_eta->eval(bRec.its) *  Spectrum(Frame::sinTheta(bRec.wo))/m_eta->eval(bRec.its)));
			cos_theta_o_inside = cos_theta_o_inside.sqrt();
			result += Frame::cosTheta(bRec.wo) * m_albedo->eval(bRec.its) * (Spectrum(1.) - fresnel(Frame::cosTheta(bRec.wi), bRec.its)) * (Spectrum(1.) - fresnelInverted(cos_theta_o_inside, bRec.its)) / (M_PI * (Spectrum(1.0) - m_albedo->eval(bRec.its) * Fdr));
		}
		Float G = g(bRec.wi, bRec.wo, bRec.its);
		if (hasDiffraction && (m_sigma_s->eval(bRec.its).getLuminance() > 0)) {
			Spectrum one_over_alpha_lambda =  m_1_lambda / m_alpha->eval(bRec.its).average();
			Spectrum u = cos_theta_d * one_over_alpha_lambda; 
			Spectrum one_over_alpha_lambda2 = one_over_alpha_lambda * one_over_alpha_lambda;
			// take into account convolution between diffraction and reflection 
			Spectrum scale(1.0); 
			bool isDirac = true;
			Spectrum uprime = computeConvolution(u, scale, isDirac, bRec.its); 
			// back to standard diffraction model
			Spectrum diffraction = uprime * (2. * Frame::sinTheta(h)); // f/al
			diffraction *= diffraction; // f^2/(a^2 l^2) 
			diffraction += Spectrum(1.); // 1 + (f^2/a^2l^2)
			diffraction = diffraction.pow(-((m_c->eval(bRec.its).average()+1.)/2.)); // (1 + (f^2/a^2l^2))^(-(c+1)/2)
			Spectrum renormalization(1.0); 
			Spectrum Qq;
			Float exp_term = M_2PI * m_sigma_s->eval(bRec.its).average() * 2 * cos_theta_d; 
			exp_term *= exp_term;
			if (isDirac) {
				renormalization = sigma_rel2(one_over_alpha_lambda, Frame::sinTheta(bRec.wi), m_c->eval(bRec.its).average());
				A_specular = - exp_term * renormalization * m_1_lambda2;
				A_specular = A_specular.exp();

				exp_term = M_2PI * m_sigma_s->eval(bRec.its).average() * (Frame::cosTheta(bRec.wi) + Frame::cosTheta(bRec.wo)); 
				exp_term *= exp_term;
				A_diffraction = - exp_term * renormalization * m_1_lambda2;
				A_diffraction = A_diffraction.exp();
				Qq = Q(bRec.its, bRec.wi, bRec.wo); 
				scale = Spectrum(1.0);
			} else {
				double sin_theta_d = 1. - cos_theta_d * cos_theta_d;
				if (sin_theta_d < 0) sin_theta_d = 0;
				sin_theta_d = sqrt(sin_theta_d);
				renormalization = sigma_rel2(one_over_alpha_lambda, sin_theta_d, m_c->eval(bRec.its).average());
				
				A_specular = - exp_term * renormalization * m_1_lambda2;
				A_specular = A_specular.exp();
				Qq = 2 * fresnel(cos_theta_d, bRec.its);
				A_diffraction = A_specular;
			}
			result += scale * G * (Spectrum(1.) - A_diffraction) * Qq * (m_c->eval(bRec.its).average() - 1.0) * one_over_alpha_lambda2 * diffraction * Frame::cosTheta(bRec.wo) / (M_2PI * renormalization);
		}
		if (hasSpecular) {
			Float lowPassResult = 1.0;
			if (m_fixMERLAtGrazing) lowPassResult = lowPassFilter(bRec.wi, bRec.wo, bRec.its);
			result += lowPassResult * G * A_specular * d(h, bRec.its) * fresnel(cos_theta_d, bRec.its) / (4.0 * Frame::cosTheta(bRec.wi));
		}
		return result;
	}

	Spectrum sample(BSDFSamplingRecord &bRec, const Point2 &s) const {
		Float pdf = 0;
		return this->sample(bRec, pdf, s);
	}

	Spectrum sample(BSDFSamplingRecord &bRec, Float &_pdf,
					const Point2 &_sample) const {
		Point2 sample(_sample);
		bool hasSpecular = (bRec.typeMask & EGlossyReflection) &&
						   (bRec.component == -1 || bRec.component == 0) &&
						   (m_sigma->eval(bRec.its).average() > 0);
		bool hasDiffraction = (bRec.typeMask & EGlossyReflection) &&
							  (bRec.component == -1 || bRec.component == 1);
		bool hasDiffuse = (bRec.typeMask & EDiffuseReflection) &&
						  (bRec.component == -1 || bRec.component == 2) && (m_type != conductor);
							 
		if (Frame::cosTheta(bRec.wi) <= 0 ||
			(!hasSpecular && !hasDiffuse && !hasDiffraction))
			return Spectrum(0.0f);

		if (sample.x < m_probDiffuse) {
			// sampling the diffuse lobe. squareToCosineHemisphere
			// rescaling the sample:
			sample.x /= m_probDiffuse;
			bRec.sampledComponent = 2;
			bRec.sampledType = EDiffuseReflection;
			bRec.wo = warp::squareToCosineHemisphere(sample);
		} else { 
			// sampling the specular + diffraction lobe
			// rescaling the sample:
			sample.x = (sample.x - m_probDiffuse) / (1. - m_probDiffuse);
			// Which one should we sample? 
			Float A_spec = 0.0;
			Float exp_term = M_2PI * m_sigma_s->eval(bRec.its).average() * Frame::cosTheta(bRec.wi); 
			exp_term *= exp_term;
			A_spec = exp(- exp_term * m_1_lambda2[m_centralComponent]);
			if (sample.x < A_spec) {
				// rescale sample:
				sample.x /= A_spec;
				// sample Cook-Torrance lobe
				// Inverse of D function
				float thetaM = atan(m_sigma->eval(bRec.its).average() * pow(boost::math::gamma_q_inv(1. / m_p->eval(bRec.its).average(), sample.y), 0.5 / m_p->eval(bRec.its).average()));

				float phiM = float(M_2PI) * sample.x;
				float sinThetaM = std::sin(thetaM);

				Vector m(std::cos(phiM) * sinThetaM, std::sin(phiM) * sinThetaM,
						std::cos(thetaM));

				float wiScalarM = dot(bRec.wi, m);

        		bRec.wo = 2 * wiScalarM * m - bRec.wi; 
		//		bRec.wo = operator*<float>(2 * wiScalarM,m) - bRec.wi;

				bRec.sampledComponent = 0;
				bRec.sampledType = EGlossyReflection;
			} else {
				// Sampling the diffraction lobe
				// Rescaling the sample:
				sample.x = (sample.x - A_spec) / (1 - A_spec);
				// adapted sampling:
				Float one_over_alpha_lambda =  m_1_lambda[m_centralComponent] / m_alpha->eval(bRec.its).average();
				Float one_over_alpha_lambda2 = one_over_alpha_lambda * one_over_alpha_lambda;
				Float one_plus_sin_theta_i_2 = 1 + Frame::sinTheta(bRec.wi);
				one_plus_sin_theta_i_2 *= one_plus_sin_theta_i_2;
				Float M = 1. + one_plus_sin_theta_i_2 * one_over_alpha_lambda2;
				Float c_minus_1 = m_c->eval(bRec.its).average() - 1.;
				M = 1. - pow(M, -0.5 * c_minus_1); 
				M = 1. - sample.x * M; 
				M = pow(M, -2./c_minus_1) - 1.;
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
				// Pb appeared in measured, but actually valid in CTD
				Float l = bRec.wo.lengthSquared();
				if (l > 1) {
					bRec.wo /= sqrt(l); 
					bRec.wo.z = 0;
				} else bRec.wo.z = sqrt(1. - l);
				bRec.sampledComponent = 1;
				bRec.sampledType = EGlossyReflection;
			}
		}
		if (Frame::cosTheta(bRec.wo) <= 0)
			return Spectrum(0.0f);
		bRec.eta = 1;

		/* Guard against numerical imprecisions */
		_pdf = pdf(bRec, ESolidAngle);

		if (_pdf < 1e-20)
			return Spectrum(0.0f);
		else
			return eval(bRec, ESolidAngle) / _pdf;
	}

	Float pdf(const BSDFSamplingRecord &bRec, EMeasure measure) const {
		bool hasSpecular = (bRec.typeMask & EGlossyReflection) &&
						   (bRec.component == -1 || bRec.component == 0) &&
						   (m_sigma->eval(bRec.its).average() > 0);
		bool hasDiffraction = (bRec.typeMask & EGlossyReflection) &&
							  (bRec.component == -1 || bRec.component == 1);
		bool hasDiffuse = (bRec.typeMask & EDiffuseReflection) &&
						  (bRec.component == -1 || bRec.component == 2) && (m_type != conductor);

		if (measure != ESolidAngle || Frame::cosTheta(bRec.wi) <= 0 ||
			Frame::cosTheta(bRec.wo) <= 0 ||
			(!hasSpecular && !hasDiffuse && !hasDiffraction))
			return 0.0f;

		/* Calculate the reflection half-vector */
		const Vector H(normalize(bRec.wi + bRec.wo));
		Float result = 0.0f;
		
		// Which one should we sample? 
		Float A_spec = 0.0; 
		Float exp_term = M_2PI * m_sigma_s->eval(bRec.its).average() * Frame::cosTheta(bRec.wi); 
		exp_term *= exp_term;
		A_spec = exp(- exp_term * m_1_lambda2[m_centralComponent]);

		if (hasSpecular) {
			/* Jacobian of the half-direction mapping */
			const Float dwh_dwo = 1.0f / (4.0f * dot(bRec.wo, H));
			/* Evaluate the microfacet model sampling density function */
			const Float prob = d(H, bRec.its) * Frame::cosTheta(H);
			// is equivalent to distr.pdf();
			// is missing the visibleSampling opportunity
			result += prob * dwh_dwo * (1 - m_probDiffuse) * A_spec; 
		}
		if (hasDiffraction) {
			Vector d_p = bRec.wi + bRec.wo;
			Float one_over_alpha_lambda =  m_1_lambda[m_centralComponent] / m_alpha->eval(bRec.its).average();
			Float one_over_alpha_lambda2 = one_over_alpha_lambda * one_over_alpha_lambda;
			Float c_plus_1 = m_c->eval(bRec.its).average() + 1.;
			Float c_minus_1 = m_c->eval(bRec.its).average() - 1.;
			d_p.z = 0.0;
			Float r = d_p.length();
			Float Sz = one_over_alpha_lambda2 * pow(1 + r * r * one_over_alpha_lambda2, - 0.5 * c_plus_1) * c_minus_1 / (2. * M_PI);
			Float maxCosPhi = (r * r + Frame::sinTheta(bRec.wi)*Frame::sinTheta(bRec.wi) - 1.)/(2. * r *  Frame::sinTheta(bRec.wi));
			if (maxCosPhi < -1) maxCosPhi = -1;
			Float phiMax = std::acos(maxCosPhi); 
			result += (1 - m_probDiffuse) * (1 - A_spec) * Frame::cosTheta(bRec.wo) * Sz / (phiMax / M_PI);
		}
		if (hasDiffuse)
			result += m_probDiffuse * warp::squareToCosineHemispherePdf(bRec.wo);

		return (result < 1e-20f) ? 0 : result;
	}

	Float d0(const Intersection its) const {
		Float sigma2 = m_sigma->eval(its).average();
		sigma2 *= sigma2;
		const Float p = m_p->eval(its).average();
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
		return distConstant  / (sigma2);
	}

	Float d(const Vector &m, const Intersection its) const {
		if (Frame::cosTheta(m) <= 0.0)
			return 0.0;
		// x = tan^2(theta)
		const Float c2 = Frame::cosTheta(m) * Frame::cosTheta(m);
		const Float x = 1./c2 - 1; // x = tan^2 theta
        Float sigma2 = m_sigma->eval(its).average();
        sigma2 *= sigma2;

		return d0(its) * exp(-pow(x / sigma2, m_p->eval(its).average())) / (c2 * c2);
	}

	Spectrum fresnelInverted(Spectrum cosTheta, const Intersection its) const {
		Spectrum eta = Spectrum(1.0)/m_eta->eval(its);
		// Access surface from the inside. Use 1/eta
		Spectrum n2 = eta * eta;

		Spectrum c2 = cosTheta * cosTheta;
		Spectrum s2 = Spectrum(1.) - c2;

		Spectrum n2_sin2Theta = n2 - s2;
		n2_sin2Theta.clampNegative();
		Spectrum n2_cosTheta = n2 * cosTheta; 
		Spectrum sqrt_n2_sin2Theta = n2_sin2Theta.sqrt();

		Spectrum Fs = (cosTheta - sqrt_n2_sin2Theta) / (cosTheta + sqrt_n2_sin2Theta);
		Spectrum Fp = (sqrt_n2_sin2Theta - n2_cosTheta) / (n2_cosTheta + sqrt_n2_sin2Theta);

		return 0.5 * (Fs * Fs + Fp * Fp);
	}

	Spectrum fresnel(Float cosTheta, const Intersection its) const {
		if (cosTheta < 0) return Spectrum(0.0); 
		if (cosTheta <= 0.0) return Spectrum(1.0);
		Spectrum eta = m_eta->eval(its);
		if (m_type != conductor) {
			// Simple version, with real index of refraction, k == 0
			Spectrum n2 = eta * eta;                

			Float c2 = cosTheta * cosTheta;
			Float s2 = 1. - c2;

			Spectrum n2_sin2Theta = n2 - Spectrum(s2);
			Spectrum n2_cosTheta = n2 * cosTheta; 
			Spectrum sqrt_n2_sin2Theta = n2_sin2Theta.sqrt();
			 
			Spectrum Fs = (Spectrum(cosTheta) - sqrt_n2_sin2Theta) / (Spectrum(cosTheta) + sqrt_n2_sin2Theta);
			Spectrum Fp = (sqrt_n2_sin2Theta - n2_cosTheta) / (n2_cosTheta + sqrt_n2_sin2Theta);

			return 0.5 * (Fs * Fs + Fp * Fp);
		}
		// dielectric, so eta and k
		Spectrum k = m_k->eval(its);
		Float ni2 = 1.0; // IOR of external media. How can I get it?
		Float c2 = cosTheta * cosTheta;
		Float s2 = 1. - c2;
		if (s2 < 0) { s2 = 0.0; c2 = 1.0; }
		Float sin_t_tan_t = s2 / cosTheta;
		Float t2 = s2 / c2;
		Spectrum n2 = eta * eta;
		Spectrum k2 = k * k;

		Spectrum c1 = n2 - k2 - Spectrum(ni2 * s2);
		Spectrum c0 = (c1 * c1 + 4 * n2 * k2).sqrt();
		Spectrum a2 = (c0 + c1)/(2. * ni2);
		Spectrum b2 = (c0 - c1)/(2. * ni2);
		Spectrum a = a2.sqrt(); 
		Spectrum Rs = (a2 + b2 - 2. * a * cosTheta + Spectrum(c2)) / (a2 + b2 + 2. * a * cosTheta + Spectrum(c2));
		Spectrum Rp = Rs * (a2 + b2 - 2. * a * sin_t_tan_t + Spectrum(s2 * t2) ) / (a2 + b2 + 2. * a * sin_t_tan_t + Spectrum(s2 * t2 ));
		return 0.5 * (Rs + Rp);
	}

	Spectrum Q(const Intersection &its, const Vector &wi, const Vector &wo) const {
		Spectrum eta = m_eta->eval(its);
		Spectrum k; 
		if (m_type == conductor) k = m_k->eval(its); 
		else k = Spectrum(0.0);
		Spectrum Q;

		Float cos_phi_o_pi = - Frame::cosPhi(wo) * Frame::cosPhi(wi)  - Frame::sinPhi(wo) * Frame::sinPhi(wi);
		Float sin_phi_o_pi = Frame::cosPhi(wo) * Frame::sinPhi(wi)  - Frame::sinPhi(wo) * Frame::cosPhi(wi);

		for (int channel = 0; channel < SPECTRUM_SAMPLES ; channel++) {
			std::complex<float> n = eta[channel] + k[channel]*i;
			std::complex<float> n2 = n * n;
			std::complex<float> sqrt_n2_sin2_theta_i = sqrt(n2 - Frame::sinTheta(wi) * Frame::sinTheta(wi));
			std::complex<float> sqrt_n2_sin2_theta_o = sqrt(n2 - Frame::sinTheta(wo) * Frame::sinTheta(wo));
			std::complex<float> n2_1 = n2 - float(1.0);
			
			std::complex<float> Q_ss = (n2_1 * cos_phi_o_pi) /
				( (sqrt_n2_sin2_theta_i + Frame::cosTheta(wi))
					* (sqrt_n2_sin2_theta_o + Frame::cosTheta(wo)));


			std::complex<float> Q_sp = (n2_1 * sqrt_n2_sin2_theta_o * sin_phi_o_pi)/ 
				((sqrt_n2_sin2_theta_i + Frame::cosTheta(wi)) * 
				 (sqrt_n2_sin2_theta_o + n2 * Frame::cosTheta(wo)));

			std::complex<float> Q_ps = (n2_1 * sqrt_n2_sin2_theta_i * sin_phi_o_pi) / 
				((sqrt_n2_sin2_theta_i + n2 * Frame::cosTheta(wi)) * 
				 (sqrt_n2_sin2_theta_o + Frame::cosTheta(wo)));

			std::complex<float> Q_pp = (n2_1 * (
						sqrt_n2_sin2_theta_i * sqrt_n2_sin2_theta_o * cos_phi_o_pi
						- n2 * Frame::sinTheta(wi) * Frame::sinTheta(wo))) / 
				((sqrt_n2_sin2_theta_i + n2 *  Frame::cosTheta(wi)) * 
				 (sqrt_n2_sin2_theta_o + n2 * Frame::cosTheta(wo)));

			Q[channel] = std::norm(Q_ss) + std::norm(Q_sp) + std::norm(Q_ps) + std::norm(Q_pp); 

		}
		return Q;
	}

	Spectrum sigma_rel2(Spectrum b_l, Float sin_theta_i, Float c) const {
		// b_l = b over lambda = 1 / (alpha * lambda)
		// We have to treat each channel independently.
		Spectrum result;
		const Float liminf = 1 - sin_theta_i; 
		const Float limsup = 1 + sin_theta_i; 

		// We don't cache the part we can compute, only the part we have to integrate:
		const int numSamples = 100;
		Float i_c = (numSamples + 1) * (c - 1.)/c - 1;
		int k_c = floor(i_c); 
		Float alpha_c = i_c - k_c; 
		Float i_s = sin_theta_i * numSamples; 
		int k_s = floor(i_s);
		int k_s_p1 = k_s + 1;
		Float alpha_s = i_s - k_s;
		if (k_s_p1 > numSamples - 1) { k_s_p1 = numSamples - 1;}
		if (k_s > numSamples - 1) {k_s = numSamples - 1; alpha_s = 0;}
		for (int channel = 0; channel < SPECTRUM_SAMPLES; channel++) {
			Float l_b = 1.0 / b_l[channel];
			Float i_b = (numSamples + 1) * (10 * l_b / (1 + 10 * l_b)) - 1;
			int k_b = floor(i_b); 
			Float alpha_b = i_b - k_b;
			if ((i_b >= 0) && (i_b < numSamples - 1) && (i_c >= 0) && (i_c < numSamples - 1)) {
				// Need to interpolate in 3 dimensions
				Float interpol_s[4]; 
				interpol_s[0] = (1. - alpha_s) * normalizationConstant[k_b][k_c][k_s] 
				+ alpha_s *  normalizationConstant[k_b][k_c][k_s_p1];
				interpol_s[1] = (1. - alpha_s) * normalizationConstant[k_b][k_c+1][k_s] 
				+ alpha_s *  normalizationConstant[k_b][k_c+1][k_s_p1];
				interpol_s[2] = (1. - alpha_s) * normalizationConstant[k_b+1][k_c][k_s] 
				+ alpha_s *  normalizationConstant[k_b+1][k_c][k_s_p1];
				interpol_s[3] = (1. - alpha_s) * normalizationConstant[k_b+1][k_c+1][k_s] 
				+ alpha_s *  normalizationConstant[k_b+1][k_c+1][k_s_p1];
				
				Float interpol_c[2];
				interpol_c[0] = (1. - alpha_c) * interpol_s[0] + alpha_c * interpol_s[1];
				interpol_c[1] = (1. - alpha_c) * interpol_s[2] + alpha_c * interpol_s[3];

				result[channel] = (1. - alpha_b) * interpol_c[0] + alpha_b * interpol_c[1];
			}  else {
				// We are outside the cache. We need to recompute the values:
				// Revert to full computation
				// We're not going into this loop
				Float lambda2_b2 = l_b * l_b;
				if (l_b == 0) l_b = 1e-7;
				Float dv = 0.01 * M_PI/180;  // 0.01 degree 
				Float result = 0.0; 
				result = 1 - pow(1 + liminf * liminf / lambda2_b2, - 0.5 * (c - 1.)); 
				for (Float v = liminf; v < limsup; v += dv) {
					Float r2 = v * v ;
					Float r = v ;
					Float cos_thetaM = (1 - r2 - sin_theta_i * sin_theta_i) / (2 * r * sin_theta_i);
					if (cos_thetaM > 1) cos_thetaM = 1;
					if (cos_thetaM < -1) cos_thetaM = -1;
					result += 2 * ((M_PI - acos(cos_thetaM))/(2 * M_PI)) * (0.5 * (c - 1)) * (2. * v * dv /lambda2_b2) * pow(1 + v * v / lambda2_b2, -0.5*(c+1.));
				}
			}
		}
		return result;
	}


	Float smithG1(Float sigma_tan_theta, Float p) const {
		// two-in-one: compute both the classical shadowing/masking term and (if required) the low-pass filter term
		// specifically for MERL database. Some computations shared between the two.
		if (sigma_tan_theta < 1e-6f) return 1.0f;
		if (p <= 0) return 1.0f;
		// Precomputed shadowing function
		Float u = 1. / sigma_tan_theta;
		const Float kp = 5. / p;
		int pmin = floor(kp);
		int pmax = pmin + 1;
		Float a_p = kp - pmin;
		if (pmax >= numSamplesp)
			pmax = numSamplesp;
		if (pmin >= numSamplesp) {
			pmin = numSamplesp;
			a_p = 0;
		}
		if (pmin <= 0) {
			pmin = 1;
		}
		if (pmax <= 0) {
			pmax = 1;
			a_p = 1;
		}

		const Float exponent = 20;
		// Float ks = exp(-pow(u, 1. / exponent));
		Float ks = exp(-exp(log(u) / exponent)); // equivalent, but more numerically stable (A. Kaplanyan)
		int kks = floor(numSamplesu - numSamplesu * ks);
		int kks_1 = kks + 1;
		Float frac_ks = (numSamplesu - numSamplesu * ks) - kks;
		if (kks_1 > numSamplesu - 1) {
			kks_1 = numSamplesu - 1;
		}
		if (kks > numSamplesu - 1) {
			kks = numSamplesu - 1;
			frac_ks = 1;
		}
		if (kks < 0) {
			kks = 0;
		}
		if (kks_1 < 0) {
			kks_1 = 0;
			frac_ks = 0;
		}
		Float v1 = (1 - frac_ks) * precomputedG1[100 - pmin][kks] +
					frac_ks * precomputedG1[100 - pmin][kks_1];
		Float v2 = (1 - frac_ks) * precomputedG1[100 - pmax][kks] +
					frac_ks * precomputedG1[100 - pmax][kks_1];
		return (1 - a_p) * v1 + a_p * v2;
	}

	Float g(const Vector &wi, const Vector &wo, Intersection its) const {
		const Float sigma = m_sigma->eval(its).average();
		const Float p = m_p->eval(its).average();
		// standard, separated shadowing/masking function
		// 	return smithG1(sigma * Frame::tanTheta(wi), p) *
		// 		   smithG1(sigma * Frame::tanTheta(wo), p);
		// correlated shadowing function, Heitz 2014
		Float phi_i = atan2(Frame::sinPhi(wi), Frame::cosPhi(wi));
		Float phi_o = atan2(Frame::sinPhi(wo), Frame::cosPhi(wo));
		Float Phi = fabs(phi_i - phi_o); // azimuthal difference
		Float ginneken = 4.41 * Phi / (4.41 * Phi + 1); 

		Float Lambda_i = 1./smithG1(sigma * Frame::tanTheta(wi), p ) - 1.0;
		Float Lambda_o = 1./smithG1(sigma * Frame::tanTheta(wo), p ) - 1.0;

		Float Lambda_max, Lambda_min;
		if (Lambda_i > Lambda_o) {
			Lambda_max = Lambda_i; 
			Lambda_min = Lambda_o;
		} else {
			Lambda_max = Lambda_o; 
			Lambda_min = Lambda_i;
		}
		return 1.0 / (1 + Lambda_max + ginneken * Lambda_min);
	}

	Float lowPass1(Float sigma, Float tan_theta, Float p) const {
		const Float kp = 100 - 5. / p;
		int pmin = floor(kp);
		int pmax = pmin + 1;
		Float a_p = kp - pmin;
		if (pmax >= numSamplesp)
			pmax = numSamplesp;
		if (pmin >= numSamplesp) {
			pmin = numSamplesp;
			a_p = 0;
		}
		if (pmin <= 0) {
			pmin = 1;
		}
		if (pmax <= 0) {
			pmax = 1;
			a_p = 1;
		}

		// low pass filtering: first we retrieve the exponent:
		Float ks = 1000 - 1/sqrt(sigma);
		int kmin = floor(ks); 
		int kmax = kmin + 1;
		Float alpha_k = ks - kmin;
		if (kmax >= numSamplesu) kmax = numSamplesu;
		if (kmin >= numSamplesu) {kmin = numSamplesu; alpha_k = 0;}
		if (kmin <= 0) {kmin = 0;}
		if (kmax <= 0) {kmax = 0; alpha_k = 1;}
		Float p1 = (1 - alpha_k) * precomputedPower[pmin][kmin] + alpha_k * precomputedPower[pmin][kmax];
		Float p2 = (1 - alpha_k) * precomputedPower[pmax][kmin] + alpha_k * precomputedPower[pmax][kmax];
		Float exponent = (1 - a_p) * p1 + a_p * p2; 

		// then we compute the low pass filter
		Float theta = atan(tan_theta);
		return pow((1 - theta * 2./M_PI) * (1 + theta * 2./M_PI), exponent);
	}

	Float lowPassFilter(const Vector &wi, const Vector &wo, Intersection its) const {
		const Float sigma = m_sigma->eval(its).average();
		const Float p = m_p->eval(its).average();

		return lowPass1(sigma, Frame::tanTheta(wi), p) * lowPass1(sigma, Frame::tanTheta(wo), p) ;
	}

	Float bMaxValue(const Float p) const {
		if (p >= 5.0) return bMax[0]; 
		if (p <= 0.15) return bMax[97]; 
		Float k_p = (5.0 - p) * 20.0; 
		int i_p = floor(k_p); 
		if (i_p < 0) return bMax[0]; // In theory  useless. Only there for floating point reasons
		if (i_p >= 97) return bMax[97];
		Float a_p = k_p - i_p; 
		return (1. - a_p) * bMax[i_p] + a_p * bMax[i_p + 1]; 
	}

		Spectrum computeConvolution(const Spectrum &u, Spectrum &scale, bool &isDirac, const Intersection &its) const {
		// takes as input Spectrum u = cos theta_d / (alpha * lambda)
		// produces as output the spectrum u', taking into account the effects of
		// the convolution by the surface.
		// extract convolution results for this specific point. Outputs u_prime (new value for u = (cos(theta_d) * alpha * lambda), scale factor and a boolean (if it's a Dirac, simpler computations)
		// b and p are from the D part (exp (-(x/b)^p), c is from the Sz part. alpha and lambda are baked into u.
		// step 1: get i, j, k for b, p and c.
		// beta/betaMax goes from 0.05 to 0.95, steps of 0.1. i from 0 to 9.
		scale = Spectrum(d0(its));
		const Float beta_betaMax = m_sigma->eval(its).average() / bMaxValue(m_p->eval(its).average()); // Should be between 0 and 1
		const Float i_b = 10.0 * beta_betaMax - 0.5;
		int i_min = floor(i_b);
		int i_max = i_min + 1;
		Float alpha_b = i_b - i_min;
		if (i_min < 0) i_min = 0;
		if (i_max < 0) {i_max = 0; alpha_b = 1.0;};
		if (i_max >= 9) i_max = 9;
		if (i_min >= 9) { i_min = 9; alpha_b = 0.0;}
		// j_p = (5 - p) * 20. p goes from 5 to 0.2, steps of 0.2. j from 0 to 24
		const Float j_p = (5.0 - m_p->eval(its).average()) * 5;
		int j_min = floor(j_p);
		int j_max = j_min + 1;
		Float alpha_p = j_p - j_min;
		if (j_min < 0) j_min = 0;
		if (j_max < 0) {j_max = 0; alpha_p = 1.0; }
		if (j_max >= 24) j_max = 24;
		if (j_min >= 24) { j_min = 24; alpha_p = 0.0;}

		// c goes from 3.02 to 1.02, steps of 0.2. k from 0 to 10
		const Float k_c = 5.0 * (3.02 - m_c->eval(its).average());
		int k_min = floor(k_c);
		int k_max = k_min + 1;
		Float alpha_c = k_c - k_min;
		if (k_min < 0) k_min = 0;
		if (k_max < 0) {k_max = 0; alpha_c = 1.0; }
		if (k_max >= 10) k_max = 10;
		if (k_min >= 10) { k_min = 10; alpha_c = 0.0;}


		// Need to interpolate in 4 dimensions
		Spectrum u0[8];
		Spectrum a0[8];
		// u goes from 0.05 to 100, sampled irregularly
		// Have to do this component by component, sorry.
		for (int channel = 0; channel < SPECTRUM_SAMPLES ; channel++) {
			double index_u;
			int u_min;
			int u_max;
			double intervalWidth = 0.05;
			double uu = u[channel];
			if (uu < 0.05) uu = 0.05;
			if (uu >= 100) uu = 100;
			if (uu <= 1) {
				intervalWidth = 0.05;
				index_u = uu / intervalWidth;
			} else if (uu <= 10) {
				intervalWidth = 1.0;
				index_u = (uu - 1.0)/intervalWidth + 20;
			} else if (uu <= 50) {
				intervalWidth = 5.0;
				index_u = (uu - 10.0)/intervalWidth + 29;
			} else if (uu <= 100) {
				intervalWidth = 10.0;
				index_u = (uu - 50.0)/intervalWidth + 37;
			}
			u_min = floor(index_u);
			u_max = u_min + 1;
			if (u_max > 42) u_max = 42;
			double alpha_u = index_u - u_min;
			u0[0][channel] = (1. - alpha_u) *  convolutionFactors[j_min][i_min][k_min][u_min].uprime + alpha_u * convolutionFactors[j_min][i_min][k_min][u_max].uprime;
			u0[1][channel] = (1. - alpha_u) *  convolutionFactors[j_min][i_min][k_max][u_min].uprime + alpha_u * convolutionFactors[j_min][i_min][k_max][u_max].uprime;
			u0[2][channel] = (1. - alpha_u) *  convolutionFactors[j_max][i_min][k_min][u_min].uprime + alpha_u * convolutionFactors[j_max][i_min][k_min][u_max].uprime;
			u0[3][channel] = (1. - alpha_u) *  convolutionFactors[j_max][i_min][k_max][u_min].uprime + alpha_u * convolutionFactors[j_max][i_min][k_max][u_max].uprime;
			u0[4][channel] = (1. - alpha_u) *  convolutionFactors[j_min][i_max][k_min][u_min].uprime + alpha_u * convolutionFactors[j_min][i_max][k_min][u_max].uprime;
			u0[5][channel] = (1. - alpha_u) *  convolutionFactors[j_min][i_max][k_max][u_min].uprime + alpha_u * convolutionFactors[j_min][i_max][k_max][u_max].uprime;
			u0[6][channel] = (1. - alpha_u) *  convolutionFactors[j_max][i_max][k_min][u_min].uprime + alpha_u * convolutionFactors[j_max][i_max][k_min][u_max].uprime;
			u0[7][channel] = (1. - alpha_u) *  convolutionFactors[j_max][i_max][k_max][u_min].uprime + alpha_u * convolutionFactors[j_max][i_max][k_max][u_max].uprime;

			a0[0][channel] = (1. - alpha_u) *  convolutionFactors[j_min][i_min][k_min][u_min].scale + alpha_u * convolutionFactors[j_min][i_min][k_min][u_max].scale;
			a0[1][channel] = (1. - alpha_u) *  convolutionFactors[j_min][i_min][k_max][u_min].scale + alpha_u * convolutionFactors[j_min][i_min][k_max][u_max].scale;
			a0[2][channel] = (1. - alpha_u) *  convolutionFactors[j_max][i_min][k_min][u_min].scale + alpha_u * convolutionFactors[j_max][i_min][k_min][u_max].scale;
			a0[3][channel] = (1. - alpha_u) *  convolutionFactors[j_max][i_min][k_max][u_min].scale + alpha_u * convolutionFactors[j_max][i_min][k_max][u_max].scale;
			a0[4][channel] = (1. - alpha_u) *  convolutionFactors[j_min][i_max][k_min][u_min].scale + alpha_u * convolutionFactors[j_min][i_max][k_min][u_max].scale;
			a0[5][channel] = (1. - alpha_u) *  convolutionFactors[j_min][i_max][k_max][u_min].scale + alpha_u * convolutionFactors[j_min][i_max][k_max][u_max].scale;
			a0[6][channel] = (1. - alpha_u) *  convolutionFactors[j_max][i_max][k_min][u_min].scale + alpha_u * convolutionFactors[j_max][i_max][k_min][u_max].scale;
			a0[7][channel] = (1. - alpha_u) *  convolutionFactors[j_max][i_max][k_max][u_min].scale + alpha_u * convolutionFactors[j_max][i_max][k_max][u_max].scale;
		}


		// Need to interpolate in 3 dimensions
		// 8 calls to convolutionValues
		Spectrum interpol_u0[4];
		Spectrum interpol_a0[4];
		interpol_u0[0] = (1. - alpha_c) * u0[0] + alpha_c * u0[1];
		interpol_a0[0] = (1. - alpha_c) * a0[0] + alpha_c * a0[1];
		interpol_u0[1] = (1. - alpha_c) * u0[2] + alpha_c * u0[3];
		interpol_a0[1] = (1. - alpha_c) * a0[2] + alpha_c * a0[3];
		interpol_u0[2] = (1. - alpha_c) * u0[4] + alpha_c * u0[5];
		interpol_a0[2] = (1. - alpha_c) * a0[4] + alpha_c * a0[5];
		interpol_u0[3] = (1. - alpha_c) * u0[6] + alpha_c * u0[7];
		interpol_a0[3] = (1. - alpha_c) * a0[6] + alpha_c * a0[7];


		Spectrum interpol_u1[2];
		Spectrum interpol_a1[2];
		interpol_u1[0] = (1 - alpha_p) * interpol_u0[0] + alpha_p * interpol_u0[1];
		interpol_u1[1] = (1 - alpha_p) * interpol_u0[2] + alpha_p * interpol_u0[3];
		interpol_a1[0] = (1 - alpha_p) * interpol_a0[0] + alpha_p * interpol_a0[1];
		interpol_a1[1] = (1 - alpha_p) * interpol_a0[2] + alpha_p * interpol_a0[3];

		scale *= (1 - alpha_b) * interpol_a1[0] + alpha_b * interpol_a1[1];
		Spectrum uprime  = (1. - alpha_b) * interpol_u1[0] + alpha_b * interpol_u1[1];

		if ((fabs((u - uprime).average()) < 0.01) && (fabs((scale - Spectrum(1.0)).average()) < 0.01)) {
			isDirac = true;
			scale = Spectrum(1.0);
			return u;
		} else isDirac = false;
		return uprime;
	}

	Float getRoughness(const Intersection &its, int component) const {
		if (component == 0) 
			return m_sigma->eval(its).average();
		else if (component == 1) 
			return m_alpha->eval(its).average();
		else 
			return std::numeric_limits<Float>::infinity();
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		BSDF::serialize(stream, manager);

		manager->serialize(stream, m_eta.get());
		stream->writeUInt((uint32_t) m_type);
		if (m_type == conductor) {
			manager->serialize(stream, m_k.get());
		} else {
			manager->serialize(stream, m_albedo.get());
		}

		manager->serialize(stream, m_sigma.get());
		manager->serialize(stream, m_p.get());

		manager->serialize(stream, m_sigma_s.get());
		manager->serialize(stream, m_alpha.get());
		manager->serialize(stream, m_c.get());
	}

	void addChild(const std::string &name, ConfigurableObject *child) {
		if (child->getClass()->derivesFrom(MTS_CLASS(Texture))) {
			if (name == "eta")
				m_eta = static_cast<Texture *>(child);
			else if (name == "k")
				m_k = static_cast<Texture *>(child);
			else if (name == "albedo")
				m_albedo = static_cast<Texture *>(child);
			else if (name == "b")
				m_sigma = static_cast<Texture *>(child);
			else if (name == "p")
				m_p = static_cast<Texture *>(child);
			else if (name == "alpha")
				m_alpha = static_cast<Texture *>(child);
			else if (name == "c")
				m_c = static_cast<Texture *>(child);
			else if (name == "sigma_s")
				m_sigma_s = static_cast<Texture *>(child);
			else
				BSDF::addChild(name, child);
		} else {
			BSDF::addChild(name, child);
		}
	}

	virtual ~CombinedDiffraction() { }

	std::string toString() const {
		std::ostringstream oss;
		oss << "CombinedDiffraction[" << endl
			<< "  id = \"" << getID() << "\"," << endl
			<< "  eta = " << indent(m_eta->toString()) << endl;
		if (m_type == conductor) 
			oss <<  "  k = " << indent(m_k->toString()) << endl;
		else 
			oss <<  "  albedo = " << indent(m_albedo->toString()) << endl
			<< "  sigma = " << indent(m_sigma->toString()) << endl
			<< "  p = " << indent(m_p->toString()) << endl
			<< "  alpha = " << indent(m_alpha->toString()) << endl
			<< "  c = " << indent(m_c->toString()) << endl
			<< "  sigma_s = " << indent(m_sigma_s->toString()) << endl
			<< "]";
		return oss.str();
	}

	Shader *createShader(Renderer *renderer) const;

	MTS_DECLARE_CLASS();

private:
	std::string m_materialName;

	ref<Texture> m_eta;
	ref<Texture> m_k;
	ref<Texture> m_albedo;
	ref<Texture> m_sigma, m_p;
	ref<Texture> m_sigma_s, m_alpha, m_c;

	Float m_probDiffuse;
	materialType m_type;
	Spectrum m_1_lambda;
	Spectrum m_1_lambda2;
	int m_centralComponent;
	bool m_fixMERLAtGrazing;
};

// ================ Hardware shader implementation ================

class CombinedDiffractionShader : public Shader {
public:
	CombinedDiffractionShader(Renderer *renderer, const Texture *k_D)
		: Shader(renderer, EBSDFShader), m_reflectance(k_D) {
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

Shader *CombinedDiffraction::createShader(Renderer *renderer) const {
	return new CombinedDiffractionShader(renderer, m_eta.get());
}

MTS_IMPLEMENT_CLASS(CombinedDiffractionShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(CombinedDiffraction, false, BSDF)
MTS_EXPORT_PLUGIN(CombinedDiffraction, "Combined Distribution BRDF")
MTS_NAMESPACE_END
