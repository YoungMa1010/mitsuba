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

#include <mitsuba/core/fresolver.h>
#include <mitsuba/core/plugin.h>
#include <mitsuba/core/warp.h>
#include <mitsuba/hw/basicshader.h>
#include <mitsuba/render/bsdf.h>
#include <mitsuba/render/sampler.h>
#include <boost/math/special_functions/gamma.hpp>

#include "smooth.h"

#define M_2PI 6.2831853072

MTS_NAMESPACE_BEGIN

/*!\plugin{smooth}{Smooth reflectance model}
 * \order{7}
 * \icon{bsdf_roughconductor}
 * \parameters{
 *     \parameter{material}{\String}{Material name}
 *     \parameter{diffuse\showbreak Reflectance}{\Spectrum\Or\Texture}{
 *         Specifies the weight of the diffuse reflectance component}
 *     \parameter{specular\showbreak Reflectance}{\Spectrum\Or\Texture}{
 *         Specifies the weight of the diffraction reflectance component (the "a" parameter)}
 *     \parameter{b}{\Float\Or\Texture}{
 *         Specifies the "b" parameter.
 *     }
 *     \parameter{c}{\Float\Or\Texture}{
 *         Specifies the "c" parameter.
 *     }
 *     \parameter{eta}{\Float\Or\Texture}{
 *         Specifies the eta parameter used for the Fresnel term.
 *     }
 * }
 * \vspace{4mm}
 * This plugin encodes the Smooth BRDF reflectance model, or ABC,
 * from Löw, Kronnander, Ynneman and Unger, ACM Transactions on Graphics, 2012.
 *
 * \renderings{
 *     \rendering{MERL database Nickel}
 *         {bsdf_smooth_nickel.jpg}
 *     \rendering{MERL database Gold-paint}
 *         {bsdf_smooth_gold-paint.jpg}
 * }
 *
 */
class SmoothReflectance : public BSDF {
public:
	SmoothReflectance(const Properties &props) : BSDF(props) {
		Properties sampProps("independent");
		sampProps.setInteger("sampleCount", 1);
		m_sampler =
			static_cast<Sampler *>(PluginManager::getInstance()->createObject(
						MTS_CLASS(Sampler), sampProps));

		if (props.hasProperty("material")) {
			m_materialName = props.getString("material");
			SmoothData data = lookupSmoothData(m_materialName);
			m_b = new ConstantFloatTexture(data.b);
			m_c = new ConstantFloatTexture(data.c);
			m_eta = new ConstantFloatTexture(data.eta);
			m_rhoD = new ConstantSpectrumTexture(data.rhoD);
			m_a = new ConstantSpectrumTexture(data.a);
		} else {
			m_rhoD = new ConstantSpectrumTexture(props.getSpectrum("diffuseReflectance", Spectrum(1.0f)));
			m_a = new ConstantSpectrumTexture(props.getSpectrum("specularReflectance", Spectrum(1.0f)));

			m_eta = new ConstantFloatTexture(props.getFloat("eta", 1.0f));
			m_b = new ConstantFloatTexture(props.getFloat("b", 1.0f));
			m_c = new ConstantFloatTexture(props.getFloat("c", 1.0f));
		}
		configure();
	}

	SmoothReflectance(Stream *stream, InstanceManager *manager)
		: BSDF(stream, manager), m_materialName(stream->readString()) {
		Properties props("independent");
		props.setInteger("sampleCount", 1);
		m_sampler =
			static_cast<Sampler *>(PluginManager::getInstance()->createObject(
				MTS_CLASS(Sampler), props));

		m_rhoD = static_cast<Texture *>(manager->getInstance(stream));
		m_a = static_cast<Texture *>(manager->getInstance(stream));


		m_eta = static_cast<Texture *>(manager->getInstance(stream));
		m_b = static_cast<Texture *>(manager->getInstance(stream));
		m_c = static_cast<Texture *>(manager->getInstance(stream));

		configure();
	}

	void configure() {
		m_components.clear();
		m_components.push_back(EGlossyReflection);
		m_components.push_back(EDiffuseReflection);
		m_usesRayDifferentials = true;

		/* Compute weights that further steer samples towards
		   the specular, diffuse or diffraction components */

		Float dAvg = m_rhoD->getAverage().getLuminance(),
			  sAvg = m_a->getAverage().getLuminance();
		m_probDiffuse = dAvg / (dAvg + sAvg);
		m_probSpecular = sAvg / (dAvg + sAvg);

		BSDF::configure();
	}

	Spectrum eval(const BSDFSamplingRecord &bRec, EMeasure measure) const {
		bool hasSpecular = (bRec.typeMask & EGlossyReflection) &&
						   (bRec.component == -1 || bRec.component == 0);
		bool hasDiffuse = (bRec.typeMask & EDiffuseReflection) &&
						  (bRec.component == -1 || bRec.component == 1);

		if (measure != ESolidAngle || Frame::cosTheta(bRec.wi) <= 0 ||
			Frame::cosTheta(bRec.wo) <= 0 ||
			(!hasSpecular && !hasDiffuse))
			return Spectrum(0.0f);

		Spectrum result(0.0f);

		if (hasDiffuse)
			result += m_rhoD->eval(bRec.its) * INV_PI * Frame::cosTheta(bRec.wo);
		if (hasSpecular) {
			Float fx = Frame::sinTheta(bRec.wo) * Frame::cosPhi(bRec.wo) +
					   Frame::sinTheta(bRec.wi) * Frame::cosPhi(bRec.wi);
			Float fy = Frame::sinTheta(bRec.wo) * Frame::sinPhi(bRec.wo) +
					   Frame::sinTheta(bRec.wi) * Frame::sinPhi(bRec.wi);
			Float f2 = (fx * fx + fy * fy);
			Float diffractionValue = pow(1.0 + m_b->eval(bRec.its).average() * f2, -m_c->eval(bRec.its).average());
			// The cos theta_d term is computed as in Neumann et al. [1999]
			// theta_d = arcsin(||proj(wo) - proj(wi)||/2)
			Float vx = Frame::sinTheta(bRec.wo) * Frame::cosPhi(bRec.wo) -
					   Frame::sinTheta(bRec.wi) * Frame::cosPhi(bRec.wi);
			Float vy = Frame::sinTheta(bRec.wo) * Frame::sinPhi(bRec.wo) -
					   Frame::sinTheta(bRec.wi) * Frame::sinPhi(bRec.wi);
			Float cosThetad = sqrt(1.0 - 0.25 * (vx * vx + vy * vy));
			result += m_a->eval(bRec.its) * diffractionValue * fresnel(cosThetad, bRec.its) * Frame::cosTheta(bRec.wo) ;
		}
		return result;
	}

	Spectrum sample(BSDFSamplingRecord &bRec, const Point2 &s) const {
		Float pdf = 0;
		return this->sample(bRec, pdf, s);
	}

	Spectrum sample(BSDFSamplingRecord &bRec, Float &_pdf,
					const Point2 &sample) const {
		bool hasSpecular = (bRec.typeMask & EGlossyReflection) &&
						   (bRec.component == -1 || bRec.component == 0);
		bool hasDiffuse = (bRec.typeMask & EDiffuseReflection) &&
						  (bRec.component == -1 || bRec.component == 1);

		if (Frame::cosTheta(bRec.wi) <= 0 ||
			(!hasSpecular && !hasDiffuse))
			return Spectrum(0.0f);

		Float lobeSelector = m_sampler->next1D();

		if (lobeSelector < m_probDiffuse) {
			// sampling the diffuse lobe. squareToCosineHemisphere
			bRec.sampledComponent = 1;
			bRec.sampledType = EDiffuseReflection;
			bRec.wo = warp::squareToCosineHemisphere(sample);
		} else {
			// Sampling the diffraction lobe
			bRec.sampledComponent = 0;
			bRec.sampledType = EGlossyReflection;
			Float ro =  Frame::sinTheta(bRec.wi);
			Float ro2 = ro * ro;
			Float phio = atan2(Frame::sinPhi(bRec.wi), Frame::cosPhi(bRec.wi));
			Float b = m_b->eval(bRec.its).average();
            Float AMd = (b / M_PI) / (-log(2) + log (1 + b - b * ro2
				+ sqrt(1 + 2 * b * (1 + ro2) + b * b * (1 - ro2) * (1 - ro2))));
            Float E = exp(sample.x * b / (M_PI * AMd) + log(2));
            Float ri = sqrt((E - 2) * (E + 2 * b * ro2) / (2 * E * b));
            Float phii = 2 * atan( tan(sample.y * M_PI) * sqrt((1 + b * (ri + ro) * (ri + ro)) / (1 + b * (ri -ro) * (ri -ro)))) + phio;

			bRec.wo = Vector(ri * std::cos(phii), ri * std::sin(phii), sqrt(1 - ri*ri));
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
						   (bRec.component == -1 || bRec.component == 0);
		bool hasDiffuse = (bRec.typeMask & EDiffuseReflection) &&
						  (bRec.component == -1 || bRec.component == 1);

		if (measure != ESolidAngle || Frame::cosTheta(bRec.wi) <= 0 ||
			Frame::cosTheta(bRec.wo) <= 0 ||
			(!hasSpecular && !hasDiffuse))
			return 0.0f;

		Float result = 0.0f;
		if (hasSpecular) {
			Float ro =  Frame::sinTheta(bRec.wi);
			Float ro2 = ro * ro;
			Float phio = atan2(Frame::sinPhi(bRec.wi), Frame::cosPhi(bRec.wi));
			Float ri =  Frame::sinTheta(bRec.wo);
			Float ri2 = ri * ri;
			Float phii = atan2(Frame::sinPhi(bRec.wo), Frame::cosPhi(bRec.wo));
			Float b = m_b->eval(bRec.its).average();
            Float AMd = (b / M_PI) / (-log(2) + log (1 + b - b * ro2
				+ sqrt(1 + 2 * b * (1 + ro2) + b * b * (1 - ro2) * (1 - ro2))));
			const Float prob = AMd * Frame::cosTheta(bRec.wo) / (1 + b * (ri2 + ro2 + 2 * ri * ro * cos(phii - phio)));
			result += prob * m_probSpecular;
		}
		if (hasDiffuse)
			result +=
				m_probDiffuse * warp::squareToCosineHemispherePdf(bRec.wo);

		return result;
	}


	Float fresnel(Float c, const Intersection its) const {
		Float eta = m_eta->eval(its).average();

		double g2 = eta * eta + c * c - 1.0;
		if (g2 < 0) g2 = 0.0;
		double g = sqrt(g2);
		double g_c = g - c;
		double gpc = g + c;

		double value = (c * gpc - 1) / (c * g_c + 1);
		value *= value;
		value += 1.0;

		return 0.5 * value * (g_c) * (g_c) / ( (gpc) * (gpc));
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		BSDF::serialize(stream, manager);

		manager->serialize(stream, m_rhoD.get());
		manager->serialize(stream, m_a.get());


		manager->serialize(stream, m_eta.get());
		manager->serialize(stream, m_b.get());
		manager->serialize(stream, m_c.get());
	}

	virtual ~SmoothReflectance() { m_sampler = NULL; }

	MTS_DECLARE_CLASS();

private:
	std::string m_materialName;
	Sampler *m_sampler;

	ref<Texture> m_rhoD;
	ref<Texture> m_a;
	ref<Texture> m_eta, m_b, m_c;

	Float m_probDiffuse, m_probSpecular;
};

MTS_IMPLEMENT_CLASS_S(SmoothReflectance, false, BSDF)
MTS_EXPORT_PLUGIN(SmoothReflectance, "Smooth Surface BRDF")

MTS_NAMESPACE_END
