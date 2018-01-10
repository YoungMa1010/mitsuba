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
#include <mitsuba/hw/basicshader.h>

MTS_NAMESPACE_BEGIN

class brick : public Texture {
public:
	brick(const Properties &props) : Texture(props) {
		m_brickwidth = props.getFloat("brickwidth", .3f);
		m_brickheight = props.getFloat("brickheight", .1f);
		m_brickdepth = props.getFloat("brickdepth", .15f);
		m_mortarsize = props.getFloat("mortarsize", .01f);
		m_type = props.getInteger("type");
		if (props.hasProperty("bricktex") && props.getType("bricktex") == Properties::ESpectrum)
			m_bricktex = new ConstantSpectrumTexture(props.getSpectrum("bricktex"));
		if (props.hasProperty("mortartex") && props.getType("mortartex") == Properties::ESpectrum)
			m_mortartex = new ConstantSpectrumTexture(props.getSpectrum("mortartex"));
		if (props.hasProperty("brickmodtex") && props.getType("brickmodtex") == Properties::ESpectrum)
			m_brickmodtex = new ConstantSpectrumTexture(props.getSpectrum("brickmodtex"));
	}

	brick(Stream *stream, InstanceManager *manager)
	 : Texture(stream, manager) {
		m_brickwidth = stream->readFloat();
		m_brickheight = stream->readFloat();
		m_brickdepth = stream->readFloat();
		m_mortarsize = stream->readFloat();
		m_type = stream->readInt();
		m_bricktex = static_cast<Texture *>(manager->getInstance(stream));
		m_mortartex = static_cast<Texture *>(manager->getInstance(stream));
		m_brickmodtex = static_cast<Texture *>(manager->getInstance(stream));
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		Texture::serialize(stream, manager);
		stream->writeFloat(m_brickwidth);
		stream->writeFloat(m_brickheight);
		stream->writeFloat(m_brickdepth);
		stream->writeFloat(m_mortarsize);
		stream->writeInt(m_type);
		manager->serialize(stream, m_bricktex.get());
		manager->serialize(stream, m_mortartex.get());
		manager->serialize(stream, m_brickmodtex.get());
	}

	void configure() {
		if (m_bricktex == NULL && m_mortartex == NULL && m_brickmodtex == NULL)
			Log(EError, "The brick plugin needs a nested texture!");
	}

	void addChild(const std::string &name, ConfigurableObject *child) {
		if (child->getClass()->derivesFrom(MTS_CLASS(Texture))){
			m_bricktex = static_cast<Texture *>(child);
			m_mortartex = static_cast<Texture *>(child);
			m_brickmodtex = static_cast<Texture *>(child);
		}
		else
			Texture::addChild(name, child);
	}

	inline Spectrum eval(const Intersection &its, bool filter = true) const {
		
		#define BRICK_EPSILON 1e-3f
		const float offs = BRICK_EPSILON + m_mortarsize;
		Point bP(its.p + Point(offs, offs, offs));
		bP.x /= m_brickwidth;
		bP.y /= m_brickdepth;
		bP.z /= m_brickheight;

		float mortarwidth = m_mortarsize / m_brickwidth;
		float mortarheight = m_mortarsize / m_brickheight;
		float mortardepth = m_mortarsize / m_brickdepth;
		Point brickIndex;
		Point bevel;
		bool b;

		switch (m_type){
		case 1:{
			//basket
			float proportion = floorf(m_brickwidth / m_brickheight);
			float invproportion = 1.f / proportion;
			brickIndex.x = floorf(bP.x);
			brickIndex.y = floorf(bP.y);
			float bx = bP.x - brickIndex.x;
			float by = bP.y - brickIndex.y;
			brickIndex.x += brickIndex.y - 2.f * floorf(0.5f * brickIndex.y);
			const bool split = (brickIndex.x - 2.f * floor(0.5f * brickIndex.x)) < 1.f;
			if (split) {
				bx = fmodf(bx, invproportion);
				brickIndex.x = floorf(proportion * bP.x) * invproportion;
			}
			else {
				by = fmodf(by, invproportion);
				brickIndex.y = floorf(proportion * bP.y) * invproportion;
			}
			b = by > mortardepth && bx > mortarwidth;
			break;
		}
		case 2:{
			//english
			float run = 0.25f;
			brickIndex.z = floorf(bP.z);
			bevel.x = bP.x + brickIndex.z * run;
			bevel.y = bP.y - brickIndex.z * run;
			brickIndex.x = floorf(bevel.x);
			brickIndex.y = floorf(bevel.y);
			bevel.z = bP.z - brickIndex.z;
			const float divider = floorf(fmodf(fabsf(brickIndex.z), 2.f)) + 1.f;
			bevel.x = (divider * bevel.x - floorf(divider * bevel.x)) / divider;
			bevel.y = (divider * bevel.y - floorf(divider * bevel.y)) / divider;
			b = bevel.z > mortarheight && bevel.y > mortardepth && bevel.x > mortarwidth;
			break;
		}
		case 3:{
			//flemish
			const float sub = 1.5f;
			const float rsub = ceilf(sub);
			float run = 0.75f;
			brickIndex.z = floorf(bP.z);
			bevel.x = (bP.x + brickIndex.z * run) / sub;
			bevel.y = (bP.y + brickIndex.z * run) / sub;
			brickIndex.x = floorf(bevel.x);
			brickIndex.y = floorf(bevel.y);
			bevel.x = (bevel.x - brickIndex.x) * sub;
			bevel.y = (bevel.y - brickIndex.y) * sub;
			bevel.z = (bP.z - brickIndex.z) * sub;
			brickIndex.x += floor(bevel.x) / rsub;
			brickIndex.y += floor(bevel.y) / rsub;
			bevel.x -= floor(bevel.x);
			bevel.y -= floor(bevel.y);
			b = bevel.z > mortarheight && bevel.y > mortardepth && bevel.x > mortarwidth;
			break;
		}
		case 4:{
			//herringbone
			float proportion = floorf(m_brickwidth / m_brickheight);
			float invproportion = 1.f / proportion;
			brickIndex.y = floorf(proportion * bP.y);
			const float px = bP.x + brickIndex.y * invproportion;
			brickIndex.x = floorf(px);
			float bx = 0.5f * px - floorf(px * 0.5f);
			bx *= 2.f;
			float by = proportion * bP.y - floorf(proportion * bP.y);
			by *= invproportion;
			if (bx > 1.f + invproportion) {
				bx = proportion * (bx - 1.f);
				brickIndex.y -= floorf(bx - 1.f);
				bx -= floorf(bx);
				bx *= invproportion;
				by = 1.f;
			}
			else if (bx > 1.f) {
				bx = proportion * (bx - 1.f);
				brickIndex.y -= floorf(bx - 1.f);
				bx -= floorf(bx);
				bx *= invproportion;
			}
			b = by > mortarheight && bx > mortarwidth;
			break;
		}
		case 5:{
			//ketting
			float run = 1.25f;
			const float sub = 2.5f;
			const float rsub = ceilf(sub);
			brickIndex.z = floorf(bP.z);
			bevel.x = (bP.x + brickIndex.z * run) / sub;
			bevel.y = (bP.y + brickIndex.z * run) / sub;
			brickIndex.x = floorf(bevel.x);
			brickIndex.y = floorf(bevel.y);
			bevel.x = (bevel.x - brickIndex.x) * sub;
			bevel.y = (bevel.y - brickIndex.y) * sub;
			bevel.z = (bP.z - brickIndex.z) * sub;
			brickIndex.x += floor(bevel.x) / rsub;
			brickIndex.y += floor(bevel.y) / rsub;
			bevel.x -= floor(bevel.x);
			bevel.y -= floor(bevel.y);
			b = bevel.z > mortarheight && bevel.y > mortardepth && bevel.x > mortarwidth;
			break;
		}			
		case 6:{
			//running
			float run = 0.75f;
			bP += Point(0, -0.5f, 0);
			brickIndex.z = floorf(bP.z);
			bevel.x = bP.x + brickIndex.z * run;
			bevel.y = bP.y - brickIndex.z * run;
			brickIndex.x = floorf(bevel.x);
			brickIndex.y = floorf(bevel.y);
			bevel.z = bP.z - brickIndex.z;
			bevel.x -= brickIndex.x;
			bevel.y -= brickIndex.y;
			b = bevel.z > mortarheight && bevel.y > mortardepth && bevel.x > mortarwidth;
			break;
		}
		case 7:{
			//stacked
			float run = 0.0f;
			bP += Point(0, -0.5f, 0);
			brickIndex.z = floorf(bP.z);
			bevel.x = bP.x + brickIndex.z * run;
			bevel.y = bP.y - brickIndex.z * run;
			brickIndex.x = floorf(bevel.x);
			brickIndex.y = floorf(bevel.y);
			bevel.z = bP.z - brickIndex.z;
			bevel.x -= brickIndex.x;
			bevel.y -= brickIndex.y;
			b = bevel.z > mortarheight && bevel.y > mortardepth && bevel.x > mortarwidth;
			break;
		}
		}

		if (b){
			Intersection itss = its;
			itss.p = brickIndex;
			return m_bricktex->eval(its, filter) * m_brickmodtex->eval(itss, filter);
		}
		else
			return m_mortartex->eval(its, filter);
	}

	bool usesRayDifferentials() const {
		return true;
	}

	Spectrum getMaximum() const {
		Spectrum max, max1, max2, max3;
		max1 = m_bricktex->getMaximum();
		max2 = m_mortartex->getMaximum();
		max3 = m_brickmodtex->getMaximum();
		for (int i = 0; i < SPECTRUM_SAMPLES; ++i)
			max[i] = std::max(max1[i], std::max(max2[i], max3[i]));
		return max;
	}

	Spectrum getMinimum() const {
		Spectrum min, min1, min2, min3;
		min1 = m_bricktex->getMinimum();
		min2 = m_mortartex->getMinimum();
		min3 = m_brickmodtex->getMinimum();
		for (int i = 0; i < SPECTRUM_SAMPLES; ++i)
			min[i] = std::min(min1[i], std::min(min2[i], min3[i]));
		return min;
	}
		
	Spectrum getAverage() const {
		float n = 1.f - m_mortarsize;
		float m = powf(math::clamp(n, 0.f, 1.f), 3);
		Spectrum average(math::lerpf(m, m_mortartex->getAverage(), m_bricktex->getAverage()));
		return average;
	}

	bool isConstant() const {
		return false;
	}

	bool isMonochromatic() const {
		return false;
	}

	std::string toString() const {
		std::ostringstream oss;
		oss << "BrickTexture[" << endl
			<< "  bricktex = " << indent(m_bricktex->toString()) << "," << endl
			<< "  mortartex = " << indent(m_mortartex->toString()) << "," << endl
			<< "  brickmodtex = " << indent(m_brickmodtex->toString()) << "," << endl
			<< "]";
		return oss.str();
	}

	Shader *createShader(Renderer *renderer) const;

	MTS_DECLARE_CLASS()
protected:
	Float m_mortarsize;
	Float m_brickwidth;
	Float m_brickheight;
	Float m_brickdepth;
	int m_type;
	ref<const Texture> m_bricktex;
	ref<const Texture> m_mortartex;
	ref<const Texture> m_brickmodtex;
};

// ================ Hardware shader implementation ================

class brickShader : public Shader {
public:
	brickShader(Renderer *renderer, Float mortarsize, Float brickwidth, Float brickheight, Float brickdepth, int type, const Texture *bricktex,
		const Texture *mortartex, const Texture *brickmodtex) :
		Shader(renderer, ETextureShader), m_mortarsize(mortarsize), m_brickwidth(brickwidth), m_brickheight(brickheight), m_brickdepth(brickdepth), m_type(type),
		m_bricktex(bricktex), m_mortartex(mortartex), m_brickmodtex(brickmodtex) {
		m_bricktexShader = renderer->registerShaderForResource(m_bricktex.get());
		m_mortartexShader = renderer->registerShaderForResource(m_mortartex.get());
		m_brickmodtexShader = renderer->registerShaderForResource(m_brickmodtex.get());
	}

	bool isComplete() const {
		return m_bricktexShader.get() != NULL && m_mortartexShader.get() != NULL && m_brickmodtexShader.get() != NULL;
	}

	void cleanup(Renderer *renderer) {
		renderer->unregisterShaderForResource(m_bricktex.get());
		renderer->unregisterShaderForResource(m_mortartex.get());
		renderer->unregisterShaderForResource(m_brickmodtex.get());
	}

	void putDependencies(std::vector<Shader *> &deps) {
		deps.push_back(m_bricktexShader.get());
		deps.push_back(m_mortartexShader.get());
		deps.push_back(m_brickmodtexShader.get());
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
		parameterIDs.push_back(program->getParameterID(evalName + "_mortarsize", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_brickwidth", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_brickheight", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_brickdepth", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_type", false));
	}

	void bind(GPUProgram *program, const std::vector<int> &parameterIDs,
		int &textureUnitOffset) const {
		program->setParameter(parameterIDs[0], m_mortarsize);
		program->setParameter(parameterIDs[1], m_brickwidth);
		program->setParameter(parameterIDs[2], m_brickheight);
		program->setParameter(parameterIDs[3], m_brickdepth);
		program->setParameter(parameterIDs[4], m_type);
	}

	MTS_DECLARE_CLASS()
private:
	Float m_mortarsize;
	Float m_brickwidth;
	Float m_brickheight;
	Float m_brickdepth;
	int m_type;
	ref<const Texture> m_bricktex;
	ref<const Texture> m_mortartex;
	ref<const Texture> m_brickmodtex;
	ref<Shader> m_bricktexShader;
	ref<Shader> m_mortartexShader;
	ref<Shader> m_brickmodtexShader;
};

Shader *brick::createShader(Renderer *renderer) const {
	return new brickShader(renderer, m_mortarsize, m_brickwidth, m_brickheight, m_brickdepth, m_type,
		m_bricktex.get(), m_mortartex.get(), m_brickmodtex.get());
}

MTS_IMPLEMENT_CLASS(brickShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(brick, false, Texture)
MTS_EXPORT_PLUGIN(brick, "brick texture");
MTS_NAMESPACE_END
