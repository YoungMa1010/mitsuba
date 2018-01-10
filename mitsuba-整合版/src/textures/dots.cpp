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
#include <mitsuba/render/noise.h>
#include <mitsuba/core/properties.h>
#include <mitsuba/hw/gpuprogram.h>

MTS_NAMESPACE_BEGIN

/*!\plugin{Dots}{Procedural grid texture}
 * \order{2}
 * \parameters{
 *     \parameter{color0}{\Spectrum}{
 *       Color values of the background
 *       \default{0.2}
 *     }
 *     \parameter{color1}{\Spectrum}{
 *       Color value of the lines
 *       \default{0.4}
 *     }
 *     \parameter{lineWidth}{\Float}{Width of the grid lines in UV space
 *        \default{0.01}
 *     }
 *     \parameter{uscale, vscale}{\Float}{
 *       Multiplicative factors that should be applied to UV values before a lookup
 *     }
 *     \parameter{uoffset, voffset}{\Float}{
 *       Numerical offset that should be applied to UV values before a lookup
 *     }
 * }
 * \renderings{
 *     \rendering{Grid texture applied to the material test object}{tex_Dots}
 * }
 * This plugin implements a simple procedural grid texture with customizable
 * colors and line width.
 */
class Dots : public Texture2D {
public:
	Dots(const Properties &props) : Texture2D(props) {
		m_color0 = props.getSpectrum("color0", Spectrum(0.f));
		m_color1 = props.getSpectrum("color1", Spectrum(1.f));
	}

	Dots(Stream *stream, InstanceManager *manager)
		: Texture2D(stream, manager) {
		m_color0 = Spectrum(stream);
		m_color1 = Spectrum(stream);
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		Texture::serialize(stream, manager);
		m_color0.serialize(stream);
		m_color1.serialize(stream);
	}

	inline Spectrum eval(const Point2 &uv) const {
		int uCell = math::floorToInt(uv.x + .5f);
		int vCell = math::floorToInt(uv.y + .5f);

		Point T1;
		T1.x = uCell + .5f;
		T1.y = vCell + .5f;
		T1.z = 0.f;
		Point T2;
		T2.x = uCell + 1.5f;
		T2.y = vCell + 2.8f;
		T2.z = 0.f;
		Point T3;
		T3.x = uCell + 4.5f;
		T3.y = vCell + 9.8f;
		T3.z = 0.f;

		if (Noise::perlinNoise(T1) > 0.f) {
			const float radius = .35f;
			const float maxShift = 0.5f - radius;
			const float uCenter = uCell + maxShift * Noise::perlinNoise(T2);
			const float vCenter = vCell + maxShift * Noise::perlinNoise(T3);
			const float du = uv.x - uCenter, dv = uv.y - vCenter;
			if (du * du + dv * dv < radius * radius)
				return m_color0;
		}
		return m_color1;
	}

	Spectrum eval(const Point2 &uv,
		const Vector2 &d0, const Vector2 &d1) const {
		/* Filtering is currently not supported */
		return Dots::eval(uv);
	}

	bool usesRayDifferentials() const {
		return false;
	}


	Spectrum getMaximum() const {
		Spectrum max;
		for (int i = 0; i<SPECTRUM_SAMPLES; ++i)
			max[i] = std::max(m_color0[i], m_color1[i]);
		return max;
	}

	Spectrum getMinimum() const {
		Spectrum min;
		for (int i = 0; i<SPECTRUM_SAMPLES; ++i)
			min[i] = std::min(m_color0[i], m_color1[i]);
		return min;
	}
		
	Spectrum getAverage() const {
		return (m_color0 + m_color1) * 0.5f;
	}

	bool isConstant() const {
		return false;
	}

	bool isMonochromatic() const {
		return Spectrum(m_color0[0]) == m_color0
			&& Spectrum(m_color1[0]) == m_color1;
	}

	std::string toString() const {
		return "Dots[]";
	}

	Shader *createShader(Renderer *renderer) const;

	MTS_DECLARE_CLASS()
protected:
	Spectrum m_color0;
	Spectrum m_color1;
};

// ================ Hardware shader implementation ================

class DotsShader : public Shader {
public:
	DotsShader(Renderer *renderer, const Spectrum &color0,
		const Spectrum &color1, const Point2 &uvOffset,
		const Vector2 &uvScale) : Shader(renderer, ETextureShader), 
		m_color0(color0), m_color1(color1), m_uvOffset(uvOffset), m_uvScale(uvScale) {
	}

	
	void generateCode(std::ostringstream &oss,
			const std::string &evalName,
			const std::vector<std::string> &depNames) const {
		oss << "uniform vec3 " << evalName << "_color0;" << endl
			<< "uniform vec3 " << evalName << "_color1;" << endl
			<< "uniform vec2 " << evalName << "_uvOffset;" << endl
			<< "uniform vec2 " << evalName << "_uvScale;" << endl
			<< endl
			<< "vec3 " << evalName << "(vec2 uv) {" << endl
			<< "    uv = vec2(" << endl
			<< "        uv.x * " << evalName << "_uvScale.x + " << evalName << "_uvOffset.x," << endl
			<< "        uv.y * " << evalName << "_uvScale.y + " << evalName << "_uvOffset.y);" << endl
			<< "    int uCell = floor(uv.x + .5f);" << endl
			<< "    int vCell = floor(uv.y + .5f);" << endl
			<< "	Point T1;" << endl
			<< "    T1.x = uCell + .5f;" << endl
			<< "    T1.y = vCell + .5f;" << endl
			<< "    T1.z = 0.f;" << endl
			<< "	Point T2;" << endl
			<< "    T2.x = uCell + 1.5f;" << endl
			<< "    T2.y = vCell + 2.8f;" << endl
			<< "    T2.z = 0.f;" << endl
			<< "	Point T3;" << endl
			<< "    T3.x = uCell + 4.5f;" << endl
			<< "    T3.y = vCell + 9.8f;" << endl
			<< "    T3.z = 0.f;" << endl
			<< "	if (perlinNoise(T1) > 0.f){" << endl
			<< "		const float radius = .35f;" << endl
			<< "		const float maxShift = 0.5f - radius;" << endl
			<< "		const float uCenter = uCell + maxShift * perlinNoise(T2); " << endl
			<< "		const float vCenter = vCell + maxShift * perlinNoise(T3); " << endl
			<< "		const float du = u - uCenter, dv = v - vCenter;" << endl
			<< "		if (du * du + dv * dv < radius * radius){" << endl
			<< "			return " << evalName << "_color1;" << endl
			<< "		}" << endl
			<< "	}" << endl
			<< "	return " << evalName << "_color0;" << endl
			<< "}" << endl;
	}

	void resolve(const GPUProgram *program, const std::string &evalName, std::vector<int> &parameterIDs) const {
		parameterIDs.push_back(program->getParameterID(evalName + "_color0", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_color1", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_uvOffset", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_uvScale", false));
	}

	void bind(GPUProgram *program, const std::vector<int> &parameterIDs,
		int &textureUnitOffset) const {
		program->setParameter(parameterIDs[0], m_color0);
		program->setParameter(parameterIDs[1], m_color1);
		program->setParameter(parameterIDs[2], m_uvOffset);
		program->setParameter(parameterIDs[3], m_uvScale);
	}

	MTS_DECLARE_CLASS()
private:
	Spectrum m_color0;
	Spectrum m_color1;
	Point2 m_uvOffset;
	Vector2 m_uvScale;
};

Shader *Dots::createShader(Renderer *renderer) const {
	return new DotsShader(renderer, m_color0, m_color1, m_uvOffset, m_uvScale);
}

MTS_IMPLEMENT_CLASS(DotsShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(Dots, false, Texture2D)
MTS_EXPORT_PLUGIN(Dots, "Dots texture");
MTS_NAMESPACE_END
