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
#include <mitsuba/core/properties.h>
#include <mitsuba/hw/gpuprogram.h>

MTS_NAMESPACE_BEGIN

/*!\plugin{ConsTexture}{ConsTexture}
 * \order{2}
 * \parameters{
 *     \parameter{color0, color1}{\Spectrum}{
 *       Color values for the two differently-colored patches
 *       \default{0.4 and 0.2}
 *     }
 *     \parameter{uoffset, voffset}{\Float}{
 *       Numerical offset that should be applied to UV values before a lookup
 *     }
 *     \parameter{uscale, vscale}{\Float}{
 *       Multiplicative factors that should be applied to UV values before a lookup
 *     }
 * }
 * \renderings{
 *     \rendering{ConsTexture applied to the material test object
 *                as well as the ground plane}{tex_ConsTexture}
 * }
 * This plugin implements a simple procedural ConsTexture texture with
 * customizable colors.
 */
class ConsTexture : public Texture2D {
public:
	ConsTexture(const Properties &props) : Texture2D(props) {
		m_value = props.getSpectrum("value", Spectrum(1.f));
	}

	ConsTexture(Stream *stream, InstanceManager *manager)
	 : Texture2D(stream, manager) {
		m_value = Spectrum(stream);
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		Texture2D::serialize(stream, manager);
		m_value.serialize(stream);
	}

	inline Spectrum eval(const Point2 &uv) const {
		return m_value;
	}

	Spectrum eval(const Point2 &uv,
			const Vector2 &d0, const Vector2 &d1) const {
		/* Filtering is currently not supported */
		return ConsTexture::eval(uv);
	}

	bool usesRayDifferentials() const {
		return false;
	}

	Spectrum getMaximum() const {
		Spectrum max;
		for (int i = 0; i<SPECTRUM_SAMPLES; ++i)
			max[i] = m_value[i];
		return max;
	}

	Spectrum getMinimum() const {
		Spectrum min;
		for (int i = 0; i<SPECTRUM_SAMPLES; ++i)
			min[i] = m_value[i];
		return min;
	}

	Spectrum getAverage() const {
		return m_value;
	}

	bool isConstant() const {
		return false;
	}

	bool isMonochromatic() const {
		return Spectrum(m_value[0]) == m_value;
	}

	std::string toString() const {
		std::ostringstream oss;
		oss << "ConsTexture[" << endl
			<< "    value = " << m_value.toString() << endl
			<< "]";
		return oss.str();
	}

	Shader *createShader(Renderer *renderer) const;

	MTS_DECLARE_CLASS()
protected:
	Spectrum m_value;
};

// ================ Hardware shader implementation ================

class ConsTextureShader : public Shader {
public:
	ConsTextureShader(Renderer *renderer, const Spectrum &value, const Point2 &uvOffset,
		const Vector2 &uvScale) : Shader(renderer, ETextureShader),
		m_value(value), m_uvOffset(uvOffset), m_uvScale(uvScale) {
	}

	void generateCode(std::ostringstream &oss,
			const std::string &evalName,
			const std::vector<std::string> &depNames) const {
		oss << "uniform vec3 " << evalName << "_value;" << endl
			<< "uniform vec2 " << evalName << "_uvOffset;" << endl
			<< "uniform vec2 " << evalName << "_uvScale;" << endl
			<< endl
			<< "vec3 " << evalName << "(vec2 uv) {" << endl
			<< "    uv = vec2(" << endl
			<< "        uv.x * " << evalName << "_uvScale.x + " << evalName << "_uvOffset.x," << endl
			<< "        uv.y * " << evalName << "_uvScale.y + " << evalName << "_uvOffset.y);" << endl
			<< "    return " << evalName << " _value;" << endl
			<< "}" << endl;
	}

	void resolve(const GPUProgram *program, const std::string &evalName, std::vector<int> &parameterIDs) const {
		parameterIDs.push_back(program->getParameterID(evalName + "_value", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_uvOffset", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_uvScale", false));
	}

	void bind(GPUProgram *program, const std::vector<int> &parameterIDs,
		int &textureUnitOffset) const {
		program->setParameter(parameterIDs[0], m_value);
		program->setParameter(parameterIDs[1], m_uvOffset);
		program->setParameter(parameterIDs[2], m_uvScale);
	}

	MTS_DECLARE_CLASS()
private:
	Spectrum m_value;
	Point2 m_uvOffset;
	Vector2 m_uvScale;
};

Shader *ConsTexture::createShader(Renderer *renderer) const {
	return new ConsTextureShader(renderer, m_value, m_uvOffset, m_uvScale);
}

MTS_IMPLEMENT_CLASS(ConsTextureShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(ConsTexture, false, Texture2D)
MTS_EXPORT_PLUGIN(ConsTexture, "ConsTexture texture");
MTS_NAMESPACE_END
