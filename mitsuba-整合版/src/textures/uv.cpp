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

/*!\plugin{Uv}{Procedural grid texture}
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
 *     \rendering{Grid texture applied to the material test object}{tex_Uv}
 * }
 * This plugin implements a simple procedural grid texture with customizable
 * colors and line width.
 */
class Uv : public Texture2D {
public:
	Uv(const Properties &props) : Texture2D(props){
	}

	Uv(Stream *stream, InstanceManager *manager)
		: Texture2D(stream, manager){
	}

	//void serialize(Stream *stream, InstanceManager *manager) const;

	inline Spectrum eval(const Point2 &uv) const {
		float c0 = uv.x - math::floorToInt(uv.x);
		float c1 = uv.y - math::floorToInt(uv.y);
		float c[3] = { c0, c1, 0.f };
		Spectrum color;
		for (int i = 0; i < 3; i++)
			color[i] = c[i];
		return color;
	}

	Spectrum eval(const Point2 &uv,
			const Vector2 &d0, const Vector2 &d1) const {
		/* Filtering is currently not supported */
		return Uv::eval(uv);
	}

	bool usesRayDifferentials() const {
		return false;
	}


	Spectrum getMaximum() const {
		Spectrum max;
		for (int i = 0; i<SPECTRUM_SAMPLES; ++i)
			max[i] = 1.f;
		return max;
	}

	Spectrum getMinimum() const {
		Spectrum min;
		for (int i = 0; i<SPECTRUM_SAMPLES; ++i)
			min[i] = 0.f;
		return min;
	}
		
	Spectrum getAverage() const {
		Spectrum average;
		static float ave[3] = { .5f, .5f, 0.f };
		for (int i = 0; i < 3; ++i){
			average[i] = ave[i];
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
		return "Uv[]";
	}

	Shader *createShader(Renderer *renderer) const;

	MTS_DECLARE_CLASS()
protected:
	Spectrum color0;
};

// ================ Hardware shader implementation ================

class UvShader : public Shader {
public:
	UvShader(Renderer *renderer, const Point2 &uvOffset, const Vector2 &uvScale) : Shader(renderer, ETextureShader),
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
			<< "	float c0 = uv.x - floor(uv.x);" << endl
			<< "    float c1 = uv.y - floor(uv.y);" << endl
			<< "    float c[3] = { c0, c1, 0.f };" << endl
			<< "    Spectrum color;" << endl
			<< "	for (int i = 0; i < 3; i++)" << endl
			<< "		color[i] = c[i];" << endl
			<< "    return color;" << endl
			<< "}" << endl;
	}

	void resolve(const GPUProgram *program, const std::string &evalName, std::vector<int> &parameterIDs) const {
		parameterIDs.push_back(program->getParameterID(evalName + "_uvOffset", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_uvScale", false));
	}

	void bind(GPUProgram *program, const std::vector<int> &parameterIDs,
		int &textureUnitOffset) const {
		program->setParameter(parameterIDs[0], m_uvOffset);
		program->setParameter(parameterIDs[1], m_uvScale);
	}

	MTS_DECLARE_CLASS()
private:
	Point2 m_uvOffset;
	Vector2 m_uvScale;
};

Shader *Uv::createShader(Renderer *renderer) const {
	return new UvShader(renderer, m_uvOffset, m_uvScale);
}

MTS_IMPLEMENT_CLASS(UvShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(Uv, false, Texture2D)
MTS_EXPORT_PLUGIN(Uv, "Uv texture");
MTS_NAMESPACE_END
