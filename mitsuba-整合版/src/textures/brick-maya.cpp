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

/*!\plugin{brick}{Procedural grid texture}
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
 *     \rendering{Grid texture applied to the material test object}{tex_brick}
 * }
 * This plugin implements a simple procedural grid texture with customizable
 * colors and line width.
 */
class brick : public Texture2D {
public:
	brick(const Properties &props) : Texture2D(props) {
		m_brickColor = props.getSpectrum("brickColor", Spectrum(.45f));
		m_jointColor = props.getSpectrum("jointColor", Spectrum(.75f));
		m_blurFactor = props.getFloat("blurFactor", .1f);
	}

	brick(Stream *stream, InstanceManager *manager)
	 : Texture2D(stream, manager) {
		m_brickColor = Spectrum(stream);
		m_jointColor = Spectrum(stream);
		m_blurFactor = stream->readFloat();
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		Texture2D::serialize(stream, manager);
		m_brickColor.serialize(stream);
		m_jointColor.serialize(stream);
		stream->writeFloat(m_blurFactor);
	}

	inline Spectrum eval(const Point2 &uv) const {
		float x, y;
		x -= floorf(uv.x);
		y -= floorf(uv.y);

		float borderWidth = 0.1f;
		float brickHeight = 0.4f;
		float brickWidth = 0.9f;
		float v1 = borderWidth / 2;
		float v2 = v1 + brickHeight;
		float v3 = v2 + borderWidth;
		float v4 = v3 + brickHeight;
		float u1 = borderWidth / 2;
		float u2 = brickWidth / 2;
		float u3 = u2 + borderWidth;
		float u4 = u1 + brickWidth;
		float t = std::max(std::min(math::linearStep(y, v1, v1) -
			math::linearStep(y, v2, v2),
			std::max(math::linearStep(x, u3, u3),
			1 - math::linearStep(x, u2, u2))),
			std::min(math::linearStep(y, v3, v3) -
			math::linearStep(y, v4, v4),
			math::linearStep(x, u1, u1) -
			math::linearStep(x, u4, u4)));
		Spectrum color(t*m_brickColor + (1.0f - t)*m_jointColor);
		return color;
	}

	Spectrum eval(const Point2 &uv,
			const Vector2 &d0, const Vector2 &d1) const {
		/* Filtering is currently not supported */
		return brick::eval(uv);
	}

	bool usesRayDifferentials() const {
		return false;
	}

	Spectrum getMaximum() const {
		Spectrum max;
		for (int i=0; i<SPECTRUM_SAMPLES; ++i)
			max[i] = std::max(m_brickColor[i], m_jointColor[i]);
		return max;
	}

	Spectrum getMinimum() const {
		Spectrum min;
		for (int i=0; i<SPECTRUM_SAMPLES; ++i)
			min[i] = std::min(m_brickColor[i], m_jointColor[i]);
		return min;
	}

	Spectrum getAverage() const {
		Float interiorWidth = std::max((Float) 0.0f, 1 - 2 * m_blurFactor),
			  interiorArea = interiorWidth * interiorWidth,
			  lineArea = 1 - interiorArea;
		return m_jointColor * lineArea + m_brickColor * interiorArea;
	}

	bool isConstant() const {
		return false;
	}

	bool isMonochromatic() const {
		return Spectrum(m_brickColor[0]) == m_brickColor
			&& Spectrum(m_jointColor[0]) == m_jointColor;
	}

	std::string toString() const {
		return "brick[]";
	}

	Shader *createShader(Renderer *renderer) const;

	MTS_DECLARE_CLASS()
protected:
	Spectrum m_brickColor;
	Spectrum m_jointColor;
	Float m_blurFactor;
};

// ================ Hardware shader implementation ================

class brickShader : public Shader {
public:
	brickShader(Renderer *renderer, const Spectrum &brickColor,
		const Spectrum &jointColor, Float blurFactor, const Point2 &uvOffset,
		const Vector2 &uvScale) : Shader(renderer, ETextureShader),
		m_brickColor(brickColor), m_jointColor(jointColor),
		m_blurFactor(blurFactor), m_uvOffset(uvOffset), m_uvScale(uvScale) {
	}

	void generateCode(std::ostringstream &oss,
			const std::string &evalName,
			const std::vector<std::string> &depNames) const {
		oss << "uniform vec3 " << evalName << "_color0;" << endl
			<< "uniform vec3 " << evalName << "_color1;" << endl
			<< "uniform float " << evalName << "_lineWidth;" << endl
			<< "uniform vec2 " << evalName << "_uvOffset;" << endl
			<< "uniform vec2 " << evalName << "_uvScale;" << endl
			<< endl
			<< "vec3 " << evalName << "(vec2 uv) {" << endl
			<< "    uv = vec2(" << endl
			<< "        uv.x * " << evalName << "_uvScale.x + " << evalName << "_uvOffset.x," << endl
			<< "        uv.y * " << evalName << "_uvScale.y + " << evalName << "_uvOffset.y);" << endl
			<< "    float x -= floorf(uv.x);" << endl
			<< "    float y -= floorf(uv.y);" << endl
			<< "    float borderWidth = 0.1f;" << endl
			<< "    float brickHeight = 0.4f;" << endl
			<< "    brickWidth = 0.9f;" << endl
			<< "    float v1 = borderWidth / 2;" << endl
			<< "    float v2 = v1 + brickHeight;" << endl
			<< "    float v3 = v2 + borderWidth;" << endl
			<< "    float v4 = v3 + brickHeight;" << endl
			<< "    float u1 = borderWidth / 2;" << endl
			<< "    float u2 = brickWidth / 2;" << endl
			<< "    float u3 = u2 + borderWidth;" << endl
			<< "    float u4 = u1 + brickWidth;" << endl
			<< "    float t = std::max(std::min(math::linearStep(y, v1, v1) - math::linearStep(y, v2, v2), std::max(math::linearStep(x, u3, u3), 1 - math::linearStep(x, u2, u2))),	std::min(math::linearStep(y, v3, v3) - math::linearStep(y, v4, v4),	math::linearStep(x, u1, u1) - math::linearStep(x, u4, u4))); " << endl
			<< "    Spectrum color(t*m_brickColor + (1.0f - t)*m_jointColor);" << endl
			<< "    return color;" << endl
			<< "}" << endl;
	}

	void resolve(const GPUProgram *program, const std::string &evalName, std::vector<int> &parameterIDs) const {
		parameterIDs.push_back(program->getParameterID(evalName + "_color0", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_color1", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_lineWidth", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_uvOffset", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_uvScale", false));
	}

	void bind(GPUProgram *program, const std::vector<int> &parameterIDs,
		int &textureUnitOffset) const {
		program->setParameter(parameterIDs[0], m_brickColor);
		program->setParameter(parameterIDs[1], m_jointColor);
		program->setParameter(parameterIDs[2], m_blurFactor);
		program->setParameter(parameterIDs[3], m_uvOffset);
		program->setParameter(parameterIDs[4], m_uvScale);
	}

	MTS_DECLARE_CLASS()
private:
	Spectrum m_brickColor;
	Spectrum m_jointColor;
	Float m_blurFactor;
	Point2 m_uvOffset;
	Vector2 m_uvScale;
};

Shader *brick::createShader(Renderer *renderer) const {
	return new brickShader(renderer, m_brickColor, m_jointColor,
		m_blurFactor, m_uvOffset, m_uvScale);
}

MTS_IMPLEMENT_CLASS(brickShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(brick, false, Texture2D)
MTS_EXPORT_PLUGIN(brick, "Brick");
MTS_NAMESPACE_END
