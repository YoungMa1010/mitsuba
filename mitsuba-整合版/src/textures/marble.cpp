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

/*!\plugin{Marble}{Procedural grid texture}
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
 *     \rendering{Grid texture applied to the material test object}{tex_Marble}
 * }
 * This plugin implements a simple procedural grid texture with customizable
 * colors and line width.
 */
class Marble : public Texture2D {
public:
	Marble(const Properties &props) : Texture2D(props) {
		m_oct = props.getFloat("octave", 8.f);
		m_ro = props.getFloat("roughness", .5f);
		m_sc = props.getFloat("scale", 1.f);
		m_var = props.getFloat("variation", .2f);
	}

	Marble(Stream *stream, InstanceManager *manager)
	 : Texture2D(stream, manager) {
		m_oct = stream->readFloat();
		m_ro = stream->readFloat();
		m_sc = stream->readFloat();
		m_var = stream->readFloat();
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		Texture2D::serialize(stream, manager);
		stream->writeFloat(m_oct);
		stream->writeFloat(m_ro);
		stream->writeFloat(m_sc);
		stream->writeFloat(m_var);
	}

	inline Spectrum eval(const Point2 &uv) const {
		Point P;
		P.x = m_sc * uv.x;
		P.y = m_sc * uv.y;
		P.z = 0.f;
		Vector dpdx, dpdy;
		for (int i = 0; i < 3; i++){
			dpdx[i] = 0.f;
			dpdy[i] = 0.f;
		}
		float marble = P.y + Noise::fbm(P, dpdx, dpdy, m_ro, m_oct) * m_var;
		float t = .5f + .5f * sinf(marble);
		static float c[][3] = { { .58f, .58f, .6f }, { .58f, .58f, .6f }, { .58f, .58f, .6f },
		{ .5f, .5f, .5f }, { .6f, .59f, .58f }, { .58f, .58f, .6f },
		{ .58f, .58f, .6f }, { .2f, .2f, .33f }, { .58f, .58f, .6f } };

		int NC = int(sizeof(c) / sizeof(c[0]));
		int NSEG = NC - 3;
		int first = math::floorToInt(t * NSEG);
		t = (t * NSEG - first);
		Spectrum c0(c[first]), c1(c[first + 1]), c2(c[first + 2]), c3(c[first + 3]);
		Spectrum s0(math::lerpf(t, c0, c1));
		Spectrum s1(math::lerpf(t, c1, c2));
		Spectrum s2(math::lerpf(t, c2, c3));
		s0 = math::lerpf(t, s0, s1);
		s1 = math::lerpf(t, s1, s2);
		Spectrum color(1.5f * math::lerpf(t, s0, s1));
		return color;
	}

	Spectrum eval(const Point2 &uv,
			const Vector2 &d0, const Vector2 &d1) const {
		/* Filtering is currently not supported */
		return Marble::eval(uv);
	}

	bool usesRayDifferentials() const {
		return false;
	}


	Spectrum getMaximum() const {
		Spectrum max;
		for (int i=0; i<SPECTRUM_SAMPLES; ++i)
			max[i] = 1.f;
		return max;
	}

	Spectrum getMinimum() const {
		Spectrum min;
		for (int i=0; i<SPECTRUM_SAMPLES; ++i)
			min[i] = 0.f;
		return min;
	}
		
	Spectrum getAverage() const {
		Spectrum average;
		Spectrum cs(0.f);
		static float c[][3] = { { .58f, .58f, .6f }, { .58f, .58f, .6f }, { .58f, .58f, .6f },
		{ .5f, .5f, .5f }, { .6f, .59f, .58f }, { .58f, .58f, .6f },
		{ .58f, .58f, .6f }, { .2f, .2f, .33f }, { .58f, .58f, .6f }, };
		int NC = int(sizeof(c) / sizeof(c[0]));
		for (int i = 0; i < NC; ++i){
			for (int k = 0; k < SPECTRUM_SAMPLES; k++){
				cs[k] += c[i][k];
			}
		}
		for (int i = 0; i < SPECTRUM_SAMPLES; i++){
			average[i] = cs[i] / NC;
		}
		return average;
	}

	bool isConstant() const {
		return false;
	}

	bool isMonochromatic() const {
		return false;
	}

	std::string toString() const {
		return "Marble[]";
	}

	Shader *createShader(Renderer *renderer) const;

	MTS_DECLARE_CLASS()
protected:
	Float m_oct;
	Float m_ro;
	Float m_sc;
	Float m_var;
};

// ================ Hardware shader implementation ================

class MarbleShader : public Shader {
public:
	MarbleShader(Renderer *renderer, Float octave, Float roughness, Float scale, Float variaton, const Point2 &uvOffset,
		const Vector2 &uvScale) : Shader(renderer, ETextureShader),
		m_oct(octave), m_ro(roughness), m_sc(scale), m_var(variaton), m_uvOffset(uvOffset), m_uvScale(uvScale) {
	}

	
	void generateCode(std::ostringstream &oss,
			const std::string &evalName,
			const std::vector<std::string> &depNames) const {
		oss << "uniform float " << evalName << "_oct;" << endl
			<< "uniform float " << evalName << "_ro;" << endl
			<< "uniform float " << evalName << "_sc;" << endl
			<< "uniform float " << evalName << "_var;" << endl
			<< "uniform vec2 " << evalName << "_uvOffset;" << endl
			<< "uniform vec2 " << evalName << "_uvScale;" << endl
			<< endl
			<< "vec3 " << evalName << "(vec2 uv) {" << endl
			<< "    uv = vec2(" << endl
			<< "        uv.x * " << evalName << "_uvScale.x + " << evalName << "_uvOffset.x," << endl
			<< "        uv.y * " << evalName << "_uvScale.y + " << evalName << "_uvOffset.y);" << endl
			<< "    float x = uv.x - floor(uv.x);" << endl
			<< "    float y = uv.y - floor(uv.y);" << endl
			<< "	Point P;" << endl
			<< "    P.x = x * " << evalName << "_sc;" << endl
			<< "    P.y = y * " << evalName << "_sc;" << endl
			<< "	Vector dpdx, dpdy;" << endl
			<< "	for (int i = 0; i < 3; i++){" << endl
			<< "		dpdx[i] = 0.f;" << endl
			<< "		dpdy[i] = 0.f;" << endl
			<< "	}" << endl
			<< "	float marble = y + " << evalName << "_var * fbm(uv, dpdx, dpdy, " << evalName << "_ro, " << evalName << "_oct);" << endl
			<< "    float t = .5f + .5f * sinf(marble);" << endl
			<< "    static float c[][3] = { { .58f, .58f, .6f }, { .58f, .58f, .6f }, { .58f, .58f, .6f }, { .5f, .5f, .5f }, { .6f, .59f, .58f }, { .58f, .58f, .6f },	{ .58f, .58f, .6f }, { .2f, .2f, .33f }, { .58f, .58f, .6f },}; " << endl
			<< "    int NC = int(sizeof(c) / sizeof(c[0]));" << endl
			<< "    int NSEG = NC - 3;" << endl
			<< "    int first = floor(t * NSEG);" << endl
			<< "    t = (t * NSEG - first);" << endl
			<< "    Spectrum c0(c[first]), c1(c[first + 1]), c2(c[first + 2]), c3(c[first + 3]);" << endl
			<< "    Spectrum s0(lerpf(t, c0, c1));" << endl
			<< "    Spectrum s1(lerpf(t, c1, c2));" << endl
			<< "    Spectrum s2(lerpf(t, c2, c3));" << endl
			<< "    s0 = lerpf(t, s0, s1);" << endl
			<< "    s1 = lerpf(t, s1, s2);" << endl
			<< "    Spectrum color(1.5f * math::lerpf(t, s0, s1));" << endl
			<< "    return color;" << endl
			<< "}" << endl;
	}

	void resolve(const GPUProgram *program, const std::string &evalName, std::vector<int> &parameterIDs) const {
		parameterIDs.push_back(program->getParameterID(evalName + "_oct", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_ro", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_sc", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_var", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_uvOffset", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_uvScale", false));
	}

	void bind(GPUProgram *program, const std::vector<int> &parameterIDs,
		int &textureUnitOffset) const {
		program->setParameter(parameterIDs[1], m_oct);
		program->setParameter(parameterIDs[2], m_ro);
		program->setParameter(parameterIDs[3], m_sc);
		program->setParameter(parameterIDs[4], m_var);
		program->setParameter(parameterIDs[5], m_uvOffset);
		program->setParameter(parameterIDs[6], m_uvScale);
	}

	MTS_DECLARE_CLASS()
private:
	Float m_oct;
	Float m_ro;
	Float m_sc;
	Float m_var;
	Point2 m_uvOffset;
	Vector2 m_uvScale;
};

Shader *Marble::createShader(Renderer *renderer) const {
	return new MarbleShader(renderer, m_oct, m_ro, m_sc, m_var,
			m_uvOffset, m_uvScale);
}

MTS_IMPLEMENT_CLASS(MarbleShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(Marble, false, Texture2D)
MTS_EXPORT_PLUGIN(Marble, "Marble texture");
MTS_NAMESPACE_END
