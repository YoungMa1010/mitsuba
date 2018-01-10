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

/*!\plugin{Windy}{Procedural grid texture}
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
 *     \rendering{Grid texture applied to the material test object}{tex_Windy}
 * }
 * This plugin implements a simple procedural grid texture with customizable
 * colors and line width.
 */
class Windy : public Texture2D {
public:
	Windy(const Properties &props) : Texture2D(props){
	}

	Windy(Stream *stream, InstanceManager *manager)
		: Texture2D(stream, manager){
	}

	//void serialize(Stream *stream, InstanceManager *manager) const;

	inline Spectrum eval(const Point2 &uv) const {
		Point P;
		P.x = .1f * uv.x;
		P.y = .1f * uv.y;
		P.z = 0.f;
		Point Q;
		Q.x = uv.x;
		Q.y = uv.y;
		Q.z = 0.f;
		Vector dpdx, dpdy;
		for (int i = 0; i < 3; i++){
			dpdx[i] = 0.f;
			dpdy[i] = 0.f;
		}
		float windStrength = Noise::fbm(P, dpdx, dpdy, .5f, 3);
		float waveHeight = Noise::fbm(Q, dpdx, dpdy, .5f, 6);
		Spectrum color(fabsf(windStrength) * waveHeight);
		return color;
	}

	Spectrum eval(const Point2 &uv,
			const Vector2 &d0, const Vector2 &d1) const {
		/* Filtering is currently not supported */
		return Windy::eval(uv);
	}

	bool usesRayDifferentials() const {
		return false;
	}


	Spectrum getMaximum() const {
		Spectrum max;
		const float geomsum_wind = (1.f - powf(0.5f, 3)) / (1.f - 0.5f);
		const float geomsum_wave = (1.f - powf(0.5f, 6)) / (1.f - 0.5f);
		for (int i=0; i<SPECTRUM_SAMPLES; ++i)
			max[i] = geomsum_wind * geomsum_wave / 4.f;
		return max;
	}

	Spectrum getMinimum() const {
		Spectrum min;
		const float geomsum_wind = (1.f - powf(0.5f, 3)) / (1.f - 0.5f);
		const float geomsum_wave = (1.f - powf(0.5f, 6)) / (1.f - 0.5f);
		for (int i = 0; i < SPECTRUM_SAMPLES; ++i){
			min[i] = -(geomsum_wind * geomsum_wave / 4.f);
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
		return "Windy[]";
	}

	Shader *createShader(Renderer *renderer) const;

	MTS_DECLARE_CLASS()
protected:
	Spectrum color;
};

// ================ Hardware shader implementation ================

class WindyShader : public Shader {
public:
	WindyShader(Renderer *renderer, const Point2 &uvOffset,	const Vector2 &uvScale) : Shader(renderer, ETextureShader),
		m_uvOffset(uvOffset), m_uvScale(uvScale) {
	}
	
	void generateCode(std::ostringstream &oss,
			const std::string &evalName,
			const std::vector<std::string> &depNames) const {
		oss << "uniform vec2 " << evalName << "_uvOffset;" << endl
			<< "uniform vec2 " << evalName << "_uvScale;" << endl
			<< endl
			<< "vec3 " << evalName << "(vec2 uv) {" << endl
			<< "    uv = vec2(" << endl
			<< "        uv.x * " << evalName << "_uvScale.x + " << evalName << "_uvOffset.x," << endl
			<< "        uv.y * " << evalName << "_uvScale.y + " << evalName << "_uvOffset.y);" << endl
			<< "	Point P;" << endl
			<< "    P.x = .1f * uv.x;" << endl
			<< "    P.y = .1f * uv.y;" << endl
			<< "    P.z = 0.f;" << endl
			<< "	Point Q;" << endl
			<< "    Q.x = uv.x;" << endl
			<< "    Q.y = uv.y;" << endl
			<< "    Q.z = 0.f;" << endl
			<< "	Vector dpdx, dpdy;" << endl
			<< "	for (int i = 0; i < 3; i++){" << endl
			<< "		dpdx[i] = 0.f;" << endl
			<< "		dpdy[i] = 0.f;" << endl
			<< "	}" << endl
			<< "    float windStrength = fbm(P, dpdx, dpdy, .5f, 3);" << endl
			<< "    float waveHeight = fbm(Q, dpdx, dpdy, .5f, 6);" << endl
			<< "    Spectrum color(fabsf(windStrength) * waveHeight);" << endl
			<< "    return color;" << endl
			<< "}" << endl;
	}

	void resolve(const GPUProgram *program, const std::string &evalName, std::vector<int> &parameterIDs) const {
		parameterIDs.push_back(program->getParameterID(evalName + "_uvOffset", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_uvScale", false));
	}

	void bind(GPUProgram *program, const std::vector<int> &parameterIDs,
		int &textureUnitOffset) const {
		program->setParameter(parameterIDs[1], m_uvOffset);
		program->setParameter(parameterIDs[2], m_uvScale);
	}

	MTS_DECLARE_CLASS()
private:
	Point2 m_uvOffset;
	Vector2 m_uvScale;
};

Shader *Windy::createShader(Renderer *renderer) const {
	return new WindyShader(renderer, m_uvOffset, m_uvScale);
}

MTS_IMPLEMENT_CLASS(WindyShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(Windy, false, Texture2D)
MTS_EXPORT_PLUGIN(Windy, "Windy texture");
MTS_NAMESPACE_END
