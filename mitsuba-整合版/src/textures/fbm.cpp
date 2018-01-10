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

/*!\plugin{Fbm}{Procedural grid texture}
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
 *     \rendering{Grid texture applied to the material test object}{tex_Fbm}
 * }
 * This plugin implements a simple procedural grid texture with customizable
 * colors and line width.
 */
class Fbm : public Texture2D {
public:
	Fbm(const Properties &props) : Texture2D(props) {
		m_oct = props.getFloat("octave", 8.f);
		m_ro = props.getFloat("roughness", .5f);
	}

	Fbm(Stream *stream, InstanceManager *manager)
	 : Texture2D(stream, manager) {
		m_oct = stream->readFloat();
		m_ro = stream->readFloat();
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		Texture2D::serialize(stream, manager);
		stream->writeFloat(m_oct);
		stream->writeFloat(m_ro);
	}

	inline Spectrum eval(const Point2 &uv) const {
		Point P;
		P.x = uv.x;
		P.y = uv.y;
		P.z = 0.f;
		Vector dpdx, dpdy;
		for (int i = 0; i < 3; i++){
			dpdx[i] = 0.f;
			dpdy[i] = 0.f;
		}
		Spectrum color(Noise::fbm(P, dpdx, dpdy, m_ro, m_oct));
		return color;
	}

	Spectrum eval(const Point2 &uv,
			const Vector2 &d0, const Vector2 &d1) const {
		/* Filtering is currently not supported */
		return Fbm::eval(uv);
	}

	bool usesRayDifferentials() const {
		return false;
	}


	Spectrum getMaximum() const {
		Spectrum max;
		const float geomsum = (1.f - powf(m_ro, m_oct)) / (1.f - m_ro);
		for (int i = 0; i<SPECTRUM_SAMPLES; ++i)
			max[i] = std::max(1.f, geomsum / 2.f);
		return max;
	}

	Spectrum getMinimum() const {
		Spectrum min;
		const float geomsum = (1.f - powf(m_ro, m_oct)) / (1.f - m_ro);
		for (int i = 0; i < SPECTRUM_SAMPLES; ++i){
			min[i] = -(std::max(1.f, geomsum / 2.f));
		}
		return min;
	}
		
	Spectrum getAverage() const {
		/*
		Float interiorWidth = std::max((Float) 0.0f, 1-2*m_lineWidth),
			  interiorArea = interiorWidth * interiorWidth,
			  lineArea = 1 - interiorArea;
		return color * lineArea + color0 * interiorArea;*/
		Spectrum average;
		for (int i = 0; i < SPECTRUM_SAMPLES; ++i){
			average[i] = .5f;
		}
		return average;
	}

	bool isConstant() const {
		return false;
	}

	bool isMonochromatic() const {
		return Spectrum(color[0]) == color;
	}

	std::string toString() const {
		return "Fbm[]";
	}

	Shader *createShader(Renderer *renderer) const;

	MTS_DECLARE_CLASS()
protected:
	Spectrum color;
	Float m_oct;
	Float m_ro;
};

// ================ Hardware shader implementation ================

class FbmShader : public Shader {
public:
	FbmShader(Renderer *renderer, Float octave, Float roughness, const Point2 &uvOffset,
		const Vector2 &uvScale) : Shader(renderer, ETextureShader),
		m_oct(octave), m_ro(roughness), m_uvOffset(uvOffset), m_uvScale(uvScale) {
	}

	
	void generateCode(std::ostringstream &oss,
			const std::string &evalName,
			const std::vector<std::string> &depNames) const {
		oss << "uniform float " << evalName << "_oct;" << endl
			<< "uniform float " << evalName << "_ro;" << endl
			<< "uniform vec2 " << evalName << "_uvOffset;" << endl
			<< "uniform vec2 " << evalName << "_uvScale;" << endl
			<< endl
			<< "vec3 " << evalName << "(vec2 uv) {" << endl
			<< "    uv = vec2(" << endl
			<< "        uv.x * " << evalName << "_uvScale.x + " << evalName << "_uvOffset.x," << endl
			<< "        uv.y * " << evalName << "_uvScale.y + " << evalName << "_uvOffset.y);" << endl
			<< "	Point P;" << endl
			<< "    P.x = uv.x;" << endl
			<< "    P.y = uv.y;" << endl
			<< "    P.x = 0.f;" << endl
			<< "	Vector dpdx, dpdy;" << endl
			<< "	for (int i = 0; i < 3; i++){" << endl
			<< "		dpdx[i] = 0.f;" << endl
			<< "		dpdy[i] = 0.f;" << endl
			<< "	}" << endl
			<< "    Spectrum color(fbm(P, dpdx, dpdy, " << evalName << "_ro, " << evalName << "_oct);" << endl
			<< "    return color;" << endl
			<< "}" << endl;
	}

	void resolve(const GPUProgram *program, const std::string &evalName, std::vector<int> &parameterIDs) const {
		parameterIDs.push_back(program->getParameterID(evalName + "_oct", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_ro", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_uvOffset", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_uvScale", false));
	}

	void bind(GPUProgram *program, const std::vector<int> &parameterIDs,
		int &textureUnitOffset) const {
		program->setParameter(parameterIDs[1], m_oct);
		program->setParameter(parameterIDs[2], m_ro);
		program->setParameter(parameterIDs[3], m_uvOffset);
		program->setParameter(parameterIDs[4], m_uvScale);
	}

	MTS_DECLARE_CLASS()
private:
	Float m_oct;
	Float m_ro;
	Point2 m_uvOffset;
	Vector2 m_uvScale;
};

Shader *Fbm::createShader(Renderer *renderer) const {
	return new FbmShader(renderer, m_oct, m_ro,
			m_uvOffset, m_uvScale);
}

MTS_IMPLEMENT_CLASS(FbmShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(Fbm, false, Texture2D)
MTS_EXPORT_PLUGIN(Fbm, "Fbm texture");
MTS_NAMESPACE_END
