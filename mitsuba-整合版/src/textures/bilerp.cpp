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

/*!\plugin{Bilerp}{Bilerp}
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
 *     \rendering{Bilerp applied to the material test object
 *                as well as the ground plane}{tex_Bilerp}
 * }
 * This plugin implements a simple procedural Bilerp texture with
 * customizable colors.
 */
class Bilerp : public Texture2D {
public:
	Bilerp(const Properties &props) : Texture2D(props) {
		m_v00 = props.getSpectrum("v00", Spectrum(0.f));
		m_v01 = props.getSpectrum("v01", Spectrum(1.f));
		m_v10 = props.getSpectrum("v10", Spectrum(0.f));
		m_v11 = props.getSpectrum("v11", Spectrum(1.f));
	}

	Bilerp(Stream *stream, InstanceManager *manager)
	 : Texture2D(stream, manager) {
		m_v00 = Spectrum(stream);
		m_v01 = Spectrum(stream);
		m_v10 = Spectrum(stream);
		m_v11 = Spectrum(stream);
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		Texture2D::serialize(stream, manager);
		m_v00.serialize(stream);
		m_v01.serialize(stream);
		m_v10.serialize(stream);
		m_v11.serialize(stream);
	}

	inline Spectrum eval(const Point2 &uv) const {
		
		float x, y;
		x -= math::floorToInt(uv.x);
		y -= math::floorToInt(uv.y);

		return (1.f - x) * (1.f - y) * m_v00 +
			(1.f - x) * y * m_v01 + x * (1.f - y) * m_v10 +
			x * y * m_v11;
		//return color;
	}

	Spectrum eval(const Point2 &uv,
			const Vector2 &d0, const Vector2 &d1) const {
		/* Filtering is currently not supported */
		return Bilerp::eval(uv);
	}

	bool usesRayDifferentials() const {
		return false;
	}

	Spectrum getMaximum() const {
		Spectrum max;
		for (int i = 0; i < SPECTRUM_SAMPLES; ++i)
			//max[i] = std::max(std::max(m_v00[i], m_v01[i]),std::max(m_v10[i],m_v11[i]));
			max[i] = 1.f;
		return max;
	}

	Spectrum getMinimum() const {
		Spectrum min;
		for (int i = 0; i < SPECTRUM_SAMPLES; ++i)
			//min[i] = std::min(std::min(m_v00[i], m_v01[i]), std::min(m_v10[i], m_v11[i]));
			min[i] = 0.f;
		return min;
	}

	Spectrum getAverage() const {
		return (m_v00 + m_v01 + m_v10 + m_v11) * 0.25f;
	}

	bool isConstant() const {
		return false;
	}

	bool isMonochromatic() const {
		return false;
		/*return Spectrum(m_v00[0]) == m_v00
			&& Spectrum(m_v01[0]) == m_v01
			&& Spectrum(m_v10[0]) == m_v10
			&& Spectrum(m_v11[0]) == m_v11;*/
	}

	std::string toString() const {
		std::ostringstream oss;
		oss << "Bilerp[" << endl
			<< "    v00 = " << m_v00.toString() << "," << endl
			<< "    v01 = " << m_v01.toString() << "," << endl
			<< "    v10 = " << m_v10.toString() << "," << endl
			<< "    v11 = " << m_v11.toString()  << endl
			<< "]";
		return oss.str();
	}

	Shader *createShader(Renderer *renderer) const;

	MTS_DECLARE_CLASS()
protected:
	Spectrum m_v00;
	Spectrum m_v01;
	Spectrum m_v10;
	Spectrum m_v11;
};

// ================ Hardware shader implementation ================

class BilerpShader : public Shader {
public:
	BilerpShader(Renderer *renderer, const Spectrum &v00, const Spectrum &v01,
		const Spectrum &v10, const Spectrum &v11, const Point2 &uvOffset,
		const Vector2 &uvScale) : Shader(renderer, ETextureShader),
		m_v00(v00), m_v01(v01), m_v10(v10), m_v11(v11),
		m_uvOffset(uvOffset), m_uvScale(uvScale) {
	}

	void generateCode(std::ostringstream &oss,
			const std::string &evalName,
			const std::vector<std::string> &depNames) const {
		oss << "uniform vec3 " << evalName << "_v00;" << endl
			<< "uniform vec3 " << evalName << "_v01;" << endl
			<< "uniform vec3 " << evalName << "_v10;" << endl
			<< "uniform vec3 " << evalName << "_v11;" << endl
			<< "uniform vec2 " << evalName << "_uvOffset;" << endl
			<< "uniform vec2 " << evalName << "_uvScale;" << endl
			<< endl
			<< "vec3 " << evalName << "(vec2 uv) {" << endl
			<< "    uv = vec2(" << endl
			<< "        uv.x * " << evalName << "_uvScale.x + " << evalName << "_uvOffset.x," << endl
			<< "        uv.y * " << evalName << "_uvScale.y + " << evalName << "_uvOffset.y);" << endl
			<< "    float x, y;" << endl
			<< "    x -= floor(uv.x);" << endl
			<< "    y -= floor(uv.y);" << endl
			<< "    Spectrum color((1.f - x) * (1.f - y) * " << evalName << "_v00 +(1.f - x) * y * " << evalName << "_v01 + x * (1.f - y) * " << evalName << "_v10 + x * y * " << evalName << "_v11);" << endl
			<< "    return color;" << endl
			<< "}" << endl;
	}

	void resolve(const GPUProgram *program, const std::string &evalName, std::vector<int> &parameterIDs) const {
		parameterIDs.push_back(program->getParameterID(evalName + "_v00", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_v01", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_v10", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_v11", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_uvOffset", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_uvScale", false));
	}

	void bind(GPUProgram *program, const std::vector<int> &parameterIDs,
		int &textureUnitOffset) const {
		program->setParameter(parameterIDs[0], m_v00);
		program->setParameter(parameterIDs[1], m_v01);
		program->setParameter(parameterIDs[2], m_v10);
		program->setParameter(parameterIDs[3], m_v11);
		program->setParameter(parameterIDs[4], m_uvOffset);
		program->setParameter(parameterIDs[5], m_uvScale);
	}

	MTS_DECLARE_CLASS()
private:
	Spectrum m_v00;
	Spectrum m_v01;
	Spectrum m_v10;
	Spectrum m_v11;
	Point2 m_uvOffset;
	Vector2 m_uvScale;
};

Shader *Bilerp::createShader(Renderer *renderer) const {
	return new BilerpShader(renderer, m_v00, m_v01, m_v10, m_v11, 
		m_uvOffset, m_uvScale);
}

MTS_IMPLEMENT_CLASS(BilerpShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(Bilerp, false, Texture2D)
MTS_EXPORT_PLUGIN(Bilerp, "Bilerp texture");
MTS_NAMESPACE_END
