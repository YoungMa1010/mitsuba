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

#include <mitsuba/render/texture.h>
#include <mitsuba/render/shape.h>
#include <mitsuba/hw/basicshader.h>

MTS_NAMESPACE_BEGIN


class MixTexture : public Texture {
public:
	MixTexture(const Properties &props) : Texture(props) {
		if (props.hasProperty("tex1") && props.getType("tex1") == Properties::ESpectrum)
			m_tex1 = new ConstantSpectrumTexture(props.getSpectrum("tex1"));
		//m_tex1 = props.getSpectrum("tex1", Spectrum(0.f));
		if (props.hasProperty("tex2") && props.getType("tex2") == Properties::ESpectrum)
			m_tex2 = new ConstantSpectrumTexture(props.getSpectrum("tex2"));
		m_amt = props.getFloat("amount", .5f);
	}

	MixTexture(Stream *stream, InstanceManager *manager)
		: Texture(stream, manager) {
		m_tex1 = static_cast<Texture *>(manager->getInstance(stream));
		//m_tex1 = Spectrum(stream);
		m_tex2 = static_cast<Texture *>(manager->getInstance(stream));
		m_amt = stream->readFloat();
	}

	void configure() {
		if (m_tex1 == NULL && m_tex2 == NULL)
		//if (m_tex2 == NULL)
			Log(EError, "The scale plugin needs a nested texture!");
	}

	void addChild(const std::string &name, ConfigurableObject *child) {
		if (child->getClass()->derivesFrom(MTS_CLASS(Texture))){
			m_tex1 = static_cast<Texture *>(child);
			m_tex2 = static_cast<Texture *>(child);
		}
		else
			Texture::addChild(name, child);
	}

	Spectrum eval(const Intersection &its, bool filter) const {
		Spectrum t1 = m_tex1->eval(its, filter);
		Spectrum t2 = m_tex2->eval(its, filter);
		Spectrum color(math::lerpf(m_amt, t1, t2));
		return color;
	}

	void evalGradient(const Intersection &its, Spectrum *gradient) const {
		m_tex1->evalGradient(its, gradient);
		m_tex2->evalGradient(its, gradient);
	}

	Spectrum getAverage() const {
		//return math::lerpf(m_amt, m_tex1, m_tex2->getAverage());
		return math::lerpf(m_amt, m_tex1->getAverage(), m_tex2->getAverage());
	}

	Spectrum getMaximum() const {
		//return math::lerpf(m_amt, m_tex1, m_tex2->getMaximum());
		return math::lerpf(m_amt, m_tex1->getMaximum(), m_tex2->getMaximum());
	}

	Spectrum getMinimum() const {
		//return math::lerpf(m_amt, m_tex1, m_tex2->getMinimum());
		return math::lerpf(m_amt, m_tex1->getMinimum(), m_tex2->getMinimum());
	}

	bool isConstant() const {
		return (m_tex1->isConstant() && m_tex2->isConstant());
		//return (m_tex2->isConstant());
	}

	/*	ref<Bitmap> getBitmap(const Vector2i &sizeHint) const {
		ref<Bitmap> result1 = m_tex1->getBitmap(sizeHint);
		ref<Bitmap> result2 = m_tex2->getBitmap(sizeHint);

		result1 = result1->convert(Bitmap::ESpectrum, Bitmap::EFloat);
		result2 = result2->convert(Bitmap::ESpectrum, Bitmap::EFloat);

		Spectrum *data1 = (Spectrum *) result1->getFloatData();
		Spectrum *data2 = (Spectrum *) result2->getFloatData();
		size_t pixelCount = result1->getPixelCount();
		for (size_t i=0; i<pixelCount; ++i)
			Spectrum *data(math::lerpf(m_amt, data1, data2));

		ref<Bitmap> result(math::lerpf(m_amt, result1, result2));
		return result;
	}*/

	std::string toString() const {
		std::ostringstream oss;
		oss << "MixTexture[" << endl
			<< "  tex1 = " << indent(m_tex1->toString()) << "," << endl
			<< "  tex2 = " << indent(m_tex2->toString()) << "," << endl
			<< "]";
		return oss.str();
	}

	bool usesRayDifferentials() const {
		return m_tex1->usesRayDifferentials() && m_tex2->usesRayDifferentials();
		//return m_tex2->usesRayDifferentials();
	}

	bool isMonochromatic() const {
		return m_tex1->isMonochromatic() && m_tex2->isMonochromatic();
		//return m_tex2->isMonochromatic();
	}

	Shader *createShader(Renderer *renderer) const;

	void serialize(Stream *stream, InstanceManager *manager) const {
		Texture::serialize(stream, manager);
		manager->serialize(stream, m_tex1.get());
		//m_tex1.serialize(stream);
		manager->serialize(stream, m_tex2.get());
		stream->writeFloat(m_amt);
	}

	MTS_DECLARE_CLASS()
protected:
	ref<const Texture> m_tex1;
	//Spectrum m_tex1;
	ref<const Texture> m_tex2;
	Float m_amt;
};

// ================ Hardware shader implementation ================

class MixTextureShader : public Shader {
public:
	MixTextureShader(Renderer *renderer, const Texture *tex1, /*const Spectrum &tex1,*/ const Texture *tex2, Float amount)
		: Shader(renderer, ETextureShader), m_tex1(tex1), m_tex2(tex2), m_amt(amount) {
		m_tex1Shader = renderer->registerShaderForResource(m_tex1.get());
		m_tex2Shader = renderer->registerShaderForResource(m_tex2.get());
	}

	bool isComplete() const {
		return m_tex1Shader.get() != NULL && m_tex2Shader.get() != NULL;
		//return m_tex2Shader.get() != NULL;
	}

	void cleanup(Renderer *renderer) {
		renderer->unregisterShaderForResource(m_tex1.get());
		renderer->unregisterShaderForResource(m_tex2.get());
	}

	void putDependencies(std::vector<Shader *> &deps) {
		deps.push_back(m_tex1Shader.get());
		deps.push_back(m_tex2Shader.get());
	}

	void generateCode(std::ostringstream &oss,
			const std::string &evalName,
			const std::vector<std::string> &depNames) const {
		oss << "uniform vec3 " << evalName << "_scale;" << endl
			<< endl
			<< "vec3 " << evalName << "(vec2 uv) {" << endl
			<< "    return " << depNames[0] << "(uv) * " << evalName << "_scale;" << endl
			<< "}" << endl;
	}

	void resolve(const GPUProgram *program, const std::string &evalName, std::vector<int> &parameterIDs) const {
		parameterIDs.push_back(program->getParameterID(evalName + "_amt", false));
	}

	void bind(GPUProgram *program, const std::vector<int> &parameterIDs, int &nestedUnitOffset) const {
		program->setParameter(parameterIDs[0], m_amt);
	}

	MTS_DECLARE_CLASS()
private:
	ref<const Texture> m_tex1;
	//Spectrum m_tex1;
	ref<const Texture> m_tex2;
	ref<Shader> m_tex1Shader;
	ref<Shader> m_tex2Shader;
	Float m_amt;
};

Shader *MixTexture::createShader(Renderer *renderer) const {
	return new MixTextureShader(renderer, m_tex1.get(),/*m_tex1,*/ m_tex2.get(), m_amt);
}

MTS_IMPLEMENT_CLASS(MixTextureShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(MixTexture, false, Texture2D)
MTS_EXPORT_PLUGIN(MixTexture, "Mix texture");
MTS_NAMESPACE_END
