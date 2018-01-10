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

#include "sgd.h"

#define M_2PI 6.2831853072

MTS_NAMESPACE_BEGIN

/*!\plugin{sgd}{Shifted Gamma Distribution}
 * \order{7}
 * \icon{bsdf_roughconductor}
 * \parameters{
 *     \parameter{material}{\String}{Material name}
 *     \parameter{importance}{\Boolean}{Use importance sampling or not.
 *     Note that importance sampling is bases on approximation, so it may
 *     introduce some artifacts.}
 * }
 * \vspace{4mm}
 * This plugin implements a realistic microfacet scattering model for rendering
 * rough conducting materials, such as metals. It can be interpreted as a fancy
 * version of the Cook-Torrance model and should be preferred over
 * heuristic models like \pluginref{phong} and \pluginref{ward} when possible.
 *
 * It can be seen as another distribution of \pluginref{roughconductor}.
 * \renderings{
 *     \rendering{MERL database Nickel}
 *         {bsdf_sgd_nickel.jpg}
 *     \rendering{MERL database Gold-paint}
 *         {bsdf_sgd_gold-paint.jpg}
 * }
 *
 * Microfacet theory describes rough
 * surfaces as an arrangement of unresolved and ideally specular facets, whose
 * normal directions are given by a specially chosen \emph{microfacet
 *distribution}.
 * By accounting for shadowing and masking effects between these facets, it is
 * possible to reproduce the important off-specular reflections peaks observed
 * in real-world measurements of such materials.
 *
 * The implementation is based on the paper ``Accurate fitting of measured
 * reflectances using a Shifted Gamma micro-facet distribution'' by Bagher et
 *al.
 * \cite{bagher:hal-00702304}.
 *
 * When using this plugin, you should ideally compile Mitsuba with support for
 * spectral rendering to get the most accurate results. While it also works
 * in RGB mode, the computations will be more approximate in nature.
 * Also note that this material is one-sided---that is, observed from the
 * back side, it will be completely black. If this is undesirable,
 * consider using the \pluginref{twosided} BRDF adapter.
 */
class ShiftedGammaDistribution : public BSDF {
public:
	ShiftedGammaDistribution(const Properties &props)
		: BSDF(props),
		  m_materialName(props.getString("material", "alum-bronze")) {
		m_importance = props.getBoolean("importance", false);
		if (m_importance) {
			m_samplingTheta =
				&ShiftedGammaDistribution::samplingThetaImportance;
			m_pdf = &ShiftedGammaDistribution::pdfImportance;
		} else {
			m_samplingTheta = &ShiftedGammaDistribution::samplingThetaNormal;
			m_pdf = &ShiftedGammaDistribution::pdfNormal;
		}

		Properties sampProps("independent");
		sampProps.setInteger("sampleCount", 1);
		m_sampler =
			static_cast<Sampler *>(PluginManager::getInstance()->createObject(
				MTS_CLASS(Sampler), sampProps));

		m_data = lookupSGDData(m_materialName);

		m_data.c.toLinearRGB(m_c[0], m_c[1], m_c[2]);
		m_data.k.toLinearRGB(m_k[0], m_k[1], m_k[2]);
		m_data.lambda.toLinearRGB(m_lambda[0], m_lambda[1], m_lambda[2]);
		m_data.theta0.toLinearRGB(m_theta0[0], m_theta0[1], m_theta0[2]);

		Float rhoD[3], rhoS[3], alpha[3];
		m_data.rhoD.toLinearRGB(rhoD[0], rhoD[1], rhoD[2]);
		m_data.rhoS.toLinearRGB(rhoS[0], rhoS[1], rhoS[2]);
		m_data.alpha.toLinearRGB(alpha[0], alpha[1], alpha[2]);

		const Float alphaMax = std::max(alpha[0], alpha[1]);

		m_rhoDDivByPi = m_data.rhoD * INV_PI;
		m_rhoSDivByPi = m_data.rhoS * INV_PI;
		m_alphaInv = Spectrum(1) / m_data.alpha;
		m_alphaMax = std::max(alphaMax, alpha[2]);
		m_alphaMax2 = std::pow(m_alphaMax, 2);

		int channel = (m_alphaMax == alpha[1]) + (m_alphaMax == alpha[2]) * 2;

		m_oneMinF0 = Spectrum(1.0) - m_data.f0;
		m_pSpec = rhoS[channel] / (rhoS[channel] + rhoD[channel]);

		configure();
	}

	ShiftedGammaDistribution(Stream *stream, InstanceManager *manager)
		: BSDF(stream, manager), m_materialName(stream->readString()) {
		m_importance = stream->readBool();
		if (m_importance) {
			m_samplingTheta =
				&ShiftedGammaDistribution::samplingThetaImportance;
			m_pdf = &ShiftedGammaDistribution::pdfImportance;
		} else {
			m_samplingTheta = &ShiftedGammaDistribution::samplingThetaNormal;
			m_pdf = &ShiftedGammaDistribution::pdfNormal;
		}

		Properties props("independent");
		props.setInteger("sampleCount", 1);
		m_sampler =
			static_cast<Sampler *>(PluginManager::getInstance()->createObject(
				MTS_CLASS(Sampler), props));

		m_data = lookupSGDData(m_materialName);

		m_data.c.toLinearRGB(m_c[0], m_c[1], m_c[2]);
		m_data.k.toLinearRGB(m_k[0], m_k[1], m_k[2]);
		m_data.lambda.toLinearRGB(m_lambda[0], m_lambda[1], m_lambda[2]);
		m_data.theta0.toLinearRGB(m_theta0[0], m_theta0[1], m_theta0[2]);

		m_rhoDDivByPi = Spectrum(stream);
		m_rhoSDivByPi = Spectrum(stream);
		m_alphaInv = Spectrum(stream);
		m_alphaMax = stream->readFloat();
		m_alphaMax2 = stream->readFloat();
		m_oneMinF0 = Spectrum(stream);
		m_pSpec = stream->readFloat();

		configure();
	}

	void configure() {
		m_components.clear();
		m_components.push_back(EGlossyReflection); // component == 0: specular
		m_components.push_back(EDiffuseReflection);// component == 1: diffuse
		m_usesRayDifferentials = true;
		BSDF::configure();
	}

	Spectrum eval(const BSDFSamplingRecord &bRec, EMeasure measure) const {
		bool hasSpecular = (bRec.typeMask & EGlossyReflection) &&
						   (bRec.component == -1 || bRec.component == 0);
		bool hasDiffuse = (bRec.typeMask & EDiffuseReflection) &&
						  (bRec.component == -1 || bRec.component == 1);

		if (measure != ESolidAngle || Frame::cosTheta(bRec.wi) <= 0 ||
			Frame::cosTheta(bRec.wo) <= 0 || (!hasSpecular && !hasDiffuse))
			return Spectrum(0.0f);

		Spectrum result(0.0f);

		if (hasDiffuse)
			result += m_rhoDDivByPi * Frame::cosTheta(bRec.wo);
		if (hasSpecular) {
			Vector h = normalize(bRec.wi + bRec.wo);
			result += m_rhoSDivByPi *
					  (d(h) * f(dot(bRec.wi, h)) * g(bRec.wi, bRec.wo)) /
					  Frame::cosTheta(bRec.wi);
		}
		return result;
	}

	Spectrum sample(BSDFSamplingRecord &bRec, const Point2 &s) const {
		Float _pdf = 0;
		return sample(bRec, _pdf, s);
	}

	Spectrum sample(BSDFSamplingRecord &bRec, Float &_pdf,
					const Point2 &sample) const {

		bool hasSpecular = (bRec.typeMask & EGlossyReflection) &&
						   (bRec.component == -1 || bRec.component == 0);
		bool hasDiffuse = (bRec.typeMask & EDiffuseReflection) &&
						  (bRec.component == -1 || bRec.component == 1);

		if (Frame::cosTheta(bRec.wi) <= 0 || (!hasSpecular && !hasDiffuse))
			return Spectrum(0.0f);

		bool specOrDiff = m_sampler->next1D() < m_pSpec;

		if (specOrDiff) {
			// Specular lobe sampling
			Float thetaM = (this->*m_samplingTheta)(sample.y);
			Float phiM = Float(M_2PI) * sample.x;
			Float sinThetaM = std::sin(thetaM);

			Vector m(std::cos(phiM) * sinThetaM, std::sin(phiM) * sinThetaM,
					 std::cos(thetaM));

			Float wiScalarM = dot(bRec.wi, m);

			bRec.wo = 2 * wiScalarM * m - bRec.wi;
			bRec.sampledComponent = 0;
			bRec.sampledType = EGlossyReflection;
		} else {
			// diffuse lobe
			bRec.sampledComponent = 1;
			bRec.sampledType = EDiffuseReflection;
			bRec.wo = warp::squareToCosineHemisphere(sample);
		}

		if (Frame::cosTheta(bRec.wo) <= 0)
			return Spectrum(0.0f);
		bRec.eta = 1;
		_pdf = pdf(bRec, ESolidAngle);

		if (_pdf < 1e-20)
			return Spectrum(0.0);
		else
			return eval(bRec, ESolidAngle) / _pdf;
	}

	Float pdf(const BSDFSamplingRecord &bRec, EMeasure measure) const {
		bool hasSpecular = (bRec.typeMask & EGlossyReflection) &&
						   (bRec.component == -1 || bRec.component == 0);
		bool hasDiffuse = (bRec.typeMask & EDiffuseReflection) &&
						  (bRec.component == -1 || bRec.component == 1);

		if (measure != ESolidAngle || Frame::cosTheta(bRec.wi) <= 0 ||
			Frame::cosTheta(bRec.wo) <= 0 || (!hasSpecular && !hasDiffuse))
			return 0.0f;

		const Vector H(normalize(bRec.wi + bRec.wo));

		Float result = 0.0f;
		if (hasSpecular) {
			/* Jacobian of the half-direction mapping */
			const Float dwh_dwo = 1.0f / (4.0f * dot(bRec.wo, H));
			/* Evaluate the microfacet model sampling density function */
			const Float prob = (this->*m_pdf)(Frame::cosTheta(H));
			//  d(H) * Frame::cosTheta(H);
			// is equivalent to distr.pdf();
			// is missing the visibleSampling opportunity
			result += prob * dwh_dwo * m_pSpec;
		}
		if (hasDiffuse)
			result +=
				(1. - m_pSpec) * warp::squareToCosineHemispherePdf(bRec.wo);

		return (result < 1e-20f) ? 0 : result;
	}

	Spectrum d(const Vector &m) const {
		if (Frame::cosTheta(m) <= 0.0)
			return Spectrum(0.0);

		// x = tan^2(theta)
		const Float x = std::pow(Frame::cosTheta(m), -2) - 1;
		Spectrum alphaX = m_data.alpha + x * m_alphaInv;

		return INV_PI * m_data.kap * (-alphaX).exp() /
			   alphaX.powSpec(m_data.p) * std::pow(x + 1, 2);
	}

	Spectrum f(Float cosTheta) const {
		Spectrum result = m_data.f0 + m_oneMinF0 * std::pow(1 - cosTheta, 5) -
						  m_data.f1 * cosTheta;

		// result should be valid, however values of f0 and f1
		// for some materials might get it wrong.
		return result.isValid() ? result : Spectrum(0.0);
	}

	Spectrum smithG1(Float theta) const {
		Spectrum g1S;
		g1S.fromLinearRGB(
			(theta > m_theta0[0])
				? 1 +
					  m_lambda[0] *
						  (1 - std::exp(m_c[0] *
										std::pow(theta - m_theta0[0], m_k[0])))
				: 1,
			(theta > m_theta0[1])
				? 1 +
					  m_lambda[1] *
						  (1 - std::exp(m_c[1] *
										std::pow(theta - m_theta0[1], m_k[1])))
				: 1,
			(theta > m_theta0[2])
				? 1 +
					  m_lambda[2] *
						  (1 - std::exp(m_c[2] *
										std::pow(theta - m_theta0[2], m_k[2])))
				: 1);

		return g1S.isValid() ? g1S : Spectrum(0.0);
	}

	Spectrum g(const Vector &wi, const Vector &wo) const {
		return smithG1(std::acos(Frame::cosTheta(wi))) *
			   smithG1(std::acos(Frame::cosTheta(wo)));
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		BSDF::serialize(stream, manager);
		stream->writeString(m_materialName);
		stream->writeBool(m_importance);
		m_rhoDDivByPi.serialize(stream);
		m_rhoSDivByPi.serialize(stream);
		m_alphaInv.serialize(stream);
		stream->writeFloat(m_alphaMax);
		stream->writeFloat(m_alphaMax2);
		m_oneMinF0.serialize(stream);
		stream->writeFloat(m_pSpec);
	}

	virtual ~ShiftedGammaDistribution() { m_sampler = NULL; }

	Float samplingThetaNormal(Float sample) const {
		return std::atan(std::sqrt(2 * sample));
	}

	Float pdfNormal(Float cosTheta) const {
		return (cosTheta <= 0) ? 0 : INV_PI;
	}

	Float samplingThetaImportance(Float sample) const {
		return std::atan(m_alphaMax * std::sqrt(sample / (1 - sample)));
	}

	Float pdfImportance(Float cosTheta) const {
		if (cosTheta < 0)
			return 0;
		Float c2 = cosTheta * cosTheta;
		c2 = 1 / c2;
		Float c3 = c2 / cosTheta;
		Float x = m_alphaMax2 + c2 - 1;
		x *= x;

		return m_alphaMax2 * c3 / x * INV_PI;
	}

	Shader *createShader(Renderer *renderer) const;

	MTS_DECLARE_CLASS();

private:
	std::string m_materialName;
	bool m_importance;
	SGDData m_data;
	Sampler *m_sampler;

	Spectrum m_rhoDDivByPi;
	Spectrum m_rhoSDivByPi;

	Spectrum m_alphaInv;
	Float m_alphaMax;
	Float m_alphaMax2;

	Spectrum m_oneMinF0;

	Float m_pSpec;

	Float m_c[3], m_k[3], m_lambda[3], m_theta0[3];

	Float (ShiftedGammaDistribution::*m_samplingTheta)(Float sample) const;
	Float (ShiftedGammaDistribution::*m_pdf)(Float cosTheta) const;
};

class ShiftedGammaDistributionShader : public Shader {
public:
	ShiftedGammaDistributionShader(Renderer *renderer, SGDData data)
		: Shader(renderer, EBSDFShader), m_data(data) {}

	bool isComplete() const { return true; }

	void putDependencies(std::vector<Shader *> &deps) {}

	void cleanup(Renderer *renderer) {}

	void resolve(const GPUProgram *program, const std::string &evalName,
				 std::vector<int> &parameterIDs) const {
		parameterIDs.push_back(
			program->getParameterID(evalName + "_rhoD", false));
		parameterIDs.push_back(
			program->getParameterID(evalName + "_rhoS", false));
		parameterIDs.push_back(
			program->getParameterID(evalName + "_alpha", false));
		parameterIDs.push_back(
			program->getParameterID(evalName + "_kap", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_p", false));
		parameterIDs.push_back(
			program->getParameterID(evalName + "_f0", false));
		parameterIDs.push_back(
			program->getParameterID(evalName + "_f1", false));
		parameterIDs.push_back(
			program->getParameterID(evalName + "_theta0", false));
		parameterIDs.push_back(
			program->getParameterID(evalName + "_lambda", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_c", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_k", false));
	}

	void bind(GPUProgram *program, const std::vector<int> &parameterIDs,
			  int &textureUnitOffset) const {
		program->setParameter(parameterIDs[0], m_data.rhoD);
		program->setParameter(parameterIDs[1], m_data.rhoS);
		program->setParameter(parameterIDs[2], m_data.alpha);
		program->setParameter(parameterIDs[3], m_data.kap);
		program->setParameter(parameterIDs[4], m_data.p);
		program->setParameter(parameterIDs[5], m_data.f0);
		program->setParameter(parameterIDs[6], m_data.f1);
		program->setParameter(parameterIDs[7], m_data.theta0);
		program->setParameter(parameterIDs[8], m_data.lambda);
		program->setParameter(parameterIDs[9], m_data.c);
		program->setParameter(parameterIDs[10], m_data.k);
	}

	void generateCode(std::ostringstream &oss, const std::string &e,
					  const std::vector<std::string> &) const {
		oss << "uniform vec3 " << e << "_rhoD;\n"
									   "uniform vec3 " << e << "_rhoS;\n"
															   "uniform vec3 "
			<< e << "_alpha;\n"
					"uniform vec3 " << e << "_kap;\n"
											"uniform vec3 " << e
			<< "_p;\n"
			   "uniform vec3 " << e << "_f0;\n"
									   "uniform vec3 " << e << "_f1;\n"
															   "uniform vec3 "
			<< e << "_lambda;\n"
					"uniform vec3 " << e << "_c;\n"
											"uniform vec3 " << e
			<< "_k;\n"
			   "uniform vec3 " << e << "_theta0;\n"

									   "vec3 " << e
			<< "_D(vec3 m) {\n"
			   "    if(m.z == 0.0)\n"
			   "        return vec3(0.0);\n"
			   "    float x = pow(m.z, -2) - 1.0;\n"
			   "    vec3 alphaX = vec3(\n"
			   "        " << e << "_alpha.r + x / " << e << "_alpha.r,\n"
															"        " << e
			<< "_alpha.g + x / " << e << "_alpha.g,\n"
										 "        " << e << "_alpha.b + x / "
			<< e << "_alpha.b\n"
					"    );\n"
					"    return vec3(\n"
					"        " << e
			<< "_kap.r * exp(-alphaX.r) * pow(alphaX.r, -" << e << "_p.r),\n"
																   "        "
			<< e << "_kap.g * exp(-alphaX.g) * pow(alphaX.g, -" << e
			<< "_p.g),\n"
			   "        " << e << "_kap.b * exp(-alphaX.b) * pow(alphaX.b, -"
			<< e << "_p.b)\n"
					"    ) * inv_pi * pow(x+1.0, 2);\n"
					"}\n"
					"\n"
					"vec3 " << e
			<< "_F(float cosTheta) {\n"
			   "    if (cosTheta < 0.0)\n"
			   "        return vec3(0.0);\n"
			   "    float cosThetaPow = pow(1.0 - cosTheta, 5);\n"
			   "    return vec3(\n"
			   "        " << e << "_f0.r + (1.0 - " << e
			<< "_f0.r) * cosThetaPow - " << e << "_f1.r * cosTheta,\n"
												 "        " << e
			<< "_f0.g + (1.0 - " << e << "_f0.g) * cosThetaPow - " << e
			<< "_f1.g * cosTheta,\n"
			   "        " << e << "_f0.b + (1.0 - " << e
			<< "_f0.b) * cosThetaPow - " << e << "_f1.b * cosTheta\n"
												 "    );\n"
												 "}\n"
												 "\n"
												 "vec3 " << e
			<< "_smithG1(float theta) {\n"
			   "    vec3 diff = vec3(\n"
			   "        theta - " << e << "_theta0.r,\n"
										  "        theta - " << e
			<< "_theta0.g,\n"
			   "        theta - " << e << "_theta0.b\n"
										  "    );\n"
										  "    return vec3(\n"
										  "        (diff.r > 0.0) ? 1.0 + " << e
			<< "_lambda.r * (1.0 - exp(" << e << "_c.r * pow(diff.r, " << e
			<< "_k.r))) : 1.0,\n"
			   "        (diff.g > 0.0) ? 1.0 + " << e
			<< "_lambda.g * (1.0 - exp(" << e << "_c.g * pow(diff.g, " << e
			<< "_k.g))) : 1.0,\n"
			   "        (diff.b > 0.0) ? 1.0 + " << e
			<< "_lambda.b * (1.0 - exp(" << e << "_c.b * pow(diff.b, " << e
			<< "_k.b))) : 1.0\n"
			   "    );\n"
			   "}\n"
			   "\n"
			   "vec3 " << e << "_G(vec3 wi, vec3 wo) {\n"
							   "    return " << e << "_smithG1(acos(wi.z)) * "
			<< e << "_smithG1(acos(wo.z));\n"
					"}\n"
					"\n"
					"vec3 " << e << "(vec2 uv, vec3 wi, vec3 wo) {\n"
									"    if (wi.z < 0.0 || wo.z < 0.0)\n"
									"        return vec3(0.0);\n"
									"    else if (wi.z == 0.0 || wo.z == 0.0)\n"
									"        return vec3(1.0);\n"
									"    vec3 h = normalize(wi + wo);\n"
									"    vec3 d = " << e << "_D(h);\n"
															"    vec3 f = " << e
			<< "_F(dot(wi,h));\n"
			   "    vec3 g = " << e << "_G(wi, wo);\n"
									   "    return " << e << "_rhoD * inv_pi + "
			<< e << "_rhoS * inv_pi * d * f * g / (wi.z * wo.z);\n"
					"}\n"
					"\n"
					"vec3 " << e
			<< "_diffuse(vec2 uv, vec3 wi, vec3 wo) {\n"
			   "    if (cosTheta(wi) < 0.0 || cosTheta(wo) < 0.0)\n"
			   "        return vec3(0.0);\n"
			   "    return " << e << "_rhoD * inv_pi;\n"
									 "}\n";
	}
	MTS_DECLARE_CLASS()
private:
	SGDData m_data;
};

Shader *ShiftedGammaDistribution::createShader(Renderer *renderer) const {
	return new ShiftedGammaDistributionShader(renderer, m_data);
}

MTS_IMPLEMENT_CLASS(ShiftedGammaDistributionShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(ShiftedGammaDistribution, false, BSDF)
MTS_EXPORT_PLUGIN(ShiftedGammaDistribution, "Shifted Gamma Distribution BRDF")

MTS_NAMESPACE_END
