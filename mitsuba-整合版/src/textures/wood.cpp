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

/*!\plugin{wood}{Procedural grid texture}
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
 *     \rendering{Grid texture applied to the material test object}{tex_wood}
 * }
 * This plugin implements a simple procedural grid texture with customizable
 * colors and line width.
 */
class wood : public Texture2D {
public:
	wood(const Properties &props) : Texture2D(props) {
		m_noisesize = props.getFloat("noisesize", .25f);
		m_turbulence = props.getFloat("turbulence", 5.f);
		m_contrast = props.getFloat("contrast", 1.f);
		m_bright = props.getFloat("bright", 1.f);
		m_method = props.getInteger("method");
		m_type = props.getInteger("type");
	}

	wood(Stream *stream, InstanceManager *manager)
	 : Texture2D(stream, manager) {
		m_noisesize = stream->readFloat();
		m_turbulence = stream->readFloat();
		m_contrast = stream->readFloat();
		m_bright = stream->readFloat();
		m_method = stream->readInt();
		m_type = stream->readInt();
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		Texture2D::serialize(stream, manager);
		stream->writeFloat(m_noisesize);
		stream->writeFloat(m_turbulence);
		stream->writeFloat(m_contrast);
		stream->writeFloat(m_bright);
		stream->writeInt(m_method);
		stream->writeInt(m_type);
	}

	inline Spectrum eval(const Point2 &uv) const {
		Point P;
		P.x = (1.f / m_noisesize) * uv.x;
		P.y = (1.f / m_noisesize) * uv.y;
		P.z = 0.f;

		if (m_method == 0){
			switch (m_type)
			{
			case 1:{
				//tex_sin BANDS
				float t = (P.x + P.y + P.z) * 10.f;
				float s = 0.5 + 0.5 * sin(t);
				float wood = (s - 0.5f) * m_contrast + m_bright - 0.5f;
				Spectrum color(math::clamp(wood, 0.f, 1.f));
				return color;
				break;
			}
			case 2:{
				//tex_sin RINGS
				float t = sqrtf(P.x*P.x + P.y*P.y + P.z*P.z) * 20.f;
				float s = 0.5 + 0.5 * sin(t);
				float wood = (s - 0.5f) * m_contrast + m_bright - 0.5f;
				Spectrum color(math::clamp(wood, 0.f, 1.f));
				return color;
				break;
			}
			case 3:{
				//tex_sin BANDNOISE
				//if (m_hard)
				//	float wood = m_turbulence * fabs(2.f * (.5f + .5f * fabs(2.f * Noise::perlinNoise(P) - 1.f)) - 1.f);
				//else
				float wood = m_turbulence * (.5f + .5f * Noise::perlinNoise(P));
				float t = (P.x + P.y + P.z) * 10.f + wood;
				float s = 0.5 + 0.5 * sin(t);
				wood = (s - 0.5f) * m_contrast + m_bright - 0.5f;
				Spectrum color(math::clamp(wood, 0.f, 1.f));
				return color;
				break;
			}
			case 4:{
				//tex_sin RINGNOISE
				//if (m_hard)
				//	float wood = m_turbulence * fabs(2.f * (.5f + .5f * fabs(2.f * Noise::perlinNoise(P) - 1.f)) - 1.f);
				//else
				float wood = m_turbulence * (.5f + .5f * Noise::perlinNoise(P));
				float t = sqrtf(P.x*P.x + P.y*P.y + P.z*P.z) * 20.f + wood;
				float s = 0.5 + 0.5 * sin(t);
				wood = (s - 0.5f) * m_contrast + m_bright - 0.5f;
				Spectrum color(math::clamp(wood, 0.f, 1.f));
				return color;
				break;
			}
			}
		}
		else{
			switch (m_type)
			{
			case 1:{
				//tex_saw BANDS
				float t = (P.x + P.y + P.z) * 10.f;
				const float b = 2 * M_PI;
				int n = (int)(t / b);
				t -= n*b;
				if (t < 0) t += b;
				float s = t / b;
				float wood = (s - 0.5f) * m_contrast + m_bright - 0.5f;
				Spectrum color(math::clamp(wood, 0.f, 1.f));
				return color;
				break;
			}
			case 2:{
				//tex_saw RINGS
				float t = sqrtf(P.x*P.x + P.y*P.y + P.z*P.z) * 20.f;
				const float b = 2 * M_PI;
				int n = (int)(t / b);
				t -= n*b;
				if (t < 0) t += b;
				float s = t / b;
				float wood = (s - 0.5f) * m_contrast + m_bright - 0.5f;
				Spectrum color(math::clamp(wood, 0.f, 1.f));
				return color;
				break;
			}
			case 3:{
				//tex_saw BANDNOISE
				//if (m_hard)
				//	float wood = m_turbulence * fabs(2.f * (.5f + .5f * fabs(2.f * Noise::perlinNoise(P) - 1.f)) - 1.f);
				//else
				float wood = m_turbulence * (.5f + .5f * Noise::perlinNoise(P));
				float t = (P.x + P.y + P.z) * 10.f + wood;
				const float b = 2 * M_PI;
				int n = (int)(t / b);
				t -= n*b;
				if (t < 0) t += b;
				float s = t / b;
				wood = (s - 0.5f) * m_contrast + m_bright - 0.5f;
				Spectrum color(math::clamp(wood, 0.f, 1.f));
				return color;
				break;
			}
			case 4:{
				//tex_saw RINGNOISE
				//if (m_hard)
				//	float wood = m_turbulence * fabs(2.f * (.5f + .5f * fabs(2.f * Noise::perlinNoise(P) - 1.f)) - 1.f);
				//else
				float wood = m_turbulence * (.5f + .5f * Noise::perlinNoise(P));
				float t = sqrtf(P.x*P.x + P.y*P.y + P.z*P.z) * 20.f + wood;
				const float b = 2 * M_PI;
				int n = (int)(t / b);
				t -= n*b;
				if (t < 0) t += b;
				float s = t / b;
				wood = (s - 0.5f) * m_contrast + m_bright - 0.5f;
				Spectrum color(math::clamp(wood, 0.f, 1.f));
				return color;
				break;
			}
			}
		}
	}

	Spectrum eval(const Point2 &uv,
			const Vector2 &d0, const Vector2 &d1) const {
		/* Filtering is currently not supported */
		return wood::eval(uv);
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
		for (int i = 0; i < SPECTRUM_SAMPLES; ++i){
			average[i] = .5f;
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
		return "wood[]";
	}

	Shader *createShader(Renderer *renderer) const;

	MTS_DECLARE_CLASS()
protected:
	Float m_noisesize;
	Float m_turbulence;
	Float m_bright;
	Float m_contrast;
	int m_method;
	int m_type;
};

// ================ Hardware shader implementation ================

class woodShader : public Shader {
public:
	woodShader(Renderer *renderer, Float noisesize, Float turbulence, Float bright, Float contrast, int method, int type,
		const Point2 &uvOffset,	const Vector2 &uvScale) : Shader(renderer, ETextureShader),
		m_noisesize(noisesize), m_turbulence(turbulence), m_bright(bright), m_contrast(contrast), m_uvOffset(uvOffset), m_uvScale(uvScale) {
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
			<< "}" << endl;
	}

	void resolve(const GPUProgram *program, const std::string &evalName, std::vector<int> &parameterIDs) const {
		parameterIDs.push_back(program->getParameterID(evalName + "_size", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_turbulence", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_bright", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_contrast", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_uvOffset", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_uvScale", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_method", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_type", false));
	}

	void bind(GPUProgram *program, const std::vector<int> &parameterIDs,
		int &textureUnitOffset) const {
		program->setParameter(parameterIDs[0], m_noisesize);
		program->setParameter(parameterIDs[1], m_turbulence);
		program->setParameter(parameterIDs[2], m_bright);
		program->setParameter(parameterIDs[3], m_contrast);
		program->setParameter(parameterIDs[4], m_method);
		program->setParameter(parameterIDs[5], m_type);
		program->setParameter(parameterIDs[6], m_uvOffset);
		program->setParameter(parameterIDs[7], m_uvScale);
	}

	MTS_DECLARE_CLASS()
private:
	Float m_noisesize;
	Float m_turbulence;
	Float m_bright;
	Float m_contrast;
	int m_method;
	int m_type;
	Point2 m_uvOffset;
	Vector2 m_uvScale;
};

Shader *wood::createShader(Renderer *renderer) const {
	return new woodShader(renderer, m_noisesize, m_turbulence, m_bright, m_contrast, m_method, m_type,
			m_uvOffset, m_uvScale);
}

MTS_IMPLEMENT_CLASS(woodShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(wood, false, Texture2D)
MTS_EXPORT_PLUGIN(wood, "wood texture");
MTS_NAMESPACE_END
