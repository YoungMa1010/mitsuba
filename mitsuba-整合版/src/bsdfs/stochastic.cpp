/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2012 by Wenzel Jakob and others.

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

#include <mitsuba/render/bsdf.h>
#include <mitsuba/render/texture.h>
#include <mitsuba/render/scene.h>
#include <mitsuba/hw/basicshader.h>
#include <mitsuba/core/warp.h>
#include <mitsuba/core/qmc.h>
#include <mitsuba/core/statistics.h>
#include <unordered_map>
#include "utils.h"
#include <queue>

#define ALPHA_SCALE  2.0f
#define GAMMA_SCALE  1
#define UNIFORM_PROB 0

MTS_NAMESPACE_BEGIN

static StatsCounter avgExpansions("Stochastic reflectance model", "Avg. expansions/query", EAverage);//通用计数器类 用来跟踪各种数量
//平均值
static StatsCounter avgExpansionsPdf("Stochastic reflectance model", "Avg. expansions/pdf query", EAverage);
static StatsCounter avgParticles("Stochastic reflectance model", "Avg. particles/query", EAverage);
static StatsCounter numClampedUV0("Stochastic reflectance model", "Queries clamped in UV space (0)");
static StatsCounter numClampedUV1("Stochastic reflectance model", "Queries clamped in UV space (1)");
static StatsCounter numClampedDir("Stochastic reflectance model", "Queries clamped in direction space");
static StatsCounter percHyperbolic("Stochastic reflectance model", "Hyperbolic case occurred", EPercentage);

struct Node {
	size_t id, numParticles;
	int depth;
	AABB2 uv;//多维包围盒结构 提供各种操作
	SphericalTriangle dir;//立体角

	/// Default constructor: initialize with the entire domain
	inline Node(size_t numParticles)
		: id(1), numParticles(numParticles), depth(0),
		uv(Point2(0, 0), Point2(1, 1)), dir(1) {
	}

	inline Node() { }

	/// Check if the rectangle is empty (i.e. there are no particles)
	inline bool empty() const { return numParticles == 0; }

	/// Return a string representation
	inline std::string toString() const {
		std::ostringstream oss;
		oss << "Node[id=" << id << ", depth=" << depth << ", uv="
			<< uv.toString() << ", dir=" << dir.toString() << ", particles=" << numParticles << "]";
		return oss.str();
	}
};

struct QueueNode : public Node {
	QueueNode(const Node &node, EIntersectionResult intersectionUV, EIntersectionResult intersectionDir)
		: Node(node), intersectionUV(intersectionUV), intersectionDir(intersectionDir) { }
	QueueNode(size_t particleCount) : Node(particleCount),
		intersectionUV(EIntersection), intersectionDir(EIntersection) { }
	EIntersectionResult intersectionUV, intersectionDir;
};

static PrimitiveThreadLocal<std::queue<QueueNode> > __queueTLS2;

class StochasticReflectance : public BSDF {
public:
	StochasticReflectance(const Properties &props)
		: BSDF(props) {
		props.markQueried("distribution");
		m_particleCount = (size_t) (props.getSize("particleCount") * props.getFloat("particleMultiplier", 1.0f));
		m_eta = props.getFloat("eta", 0.0f);
		m_gamma = degToRad(props.getFloat("gamma", 1.0f));
		Float alpha = props.getFloat("alpha", 1.33f);//获取带有默认值得单精度浮点值
		m_alphaU = props.getFloat("alphaU", alpha);
		m_alphaV = props.getFloat("alphaV", alpha);
		m_avgQueryArea = props.getFloat("queryArea", 1e-5f);
		m_reflectance = props.getSpectrum("specularReflectance", props.getSpectrum("reflectance", Spectrum(1.0f)));
		m_spatialLookupMultiplier = props.getFloat("spatialLookupMultiplier", 1);
		m_maxDepth = props.getInteger("maxDepth", 20);
		m_errorThreshold = props.getFloat("errorThreshold", 0.1);
		m_collectStatistics = props.getBoolean("collectStatistics", false);
		m_clamp = props.getBoolean("clamp", true);
		m_statUVArea = 0;
	}

	StochasticReflectance(Stream *stream, InstanceManager *manager)
		: BSDF(stream, manager) {
		m_particleCount = stream->readSize();
		m_alphaU = stream->readFloat();
		m_alphaV = stream->readFloat();
		m_eta = stream->readFloat();
		m_gamma = stream->readFloat();
		m_avgQueryArea = stream->readFloat();
		m_reflectance = Spectrum(stream);
		m_spatialLookupMultiplier = stream->readFloat();
		m_maxDepth = stream->readInt();
		m_errorThreshold = stream->readFloat();
		m_collectStatistics = stream->readBool();
		m_clamp = stream->readBool();
		m_statUVArea = 0;
		configure();
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		BSDF::serialize(stream, manager);
		stream->writeSize(m_particleCount);
		stream->writeFloat(m_alphaU);
		stream->writeFloat(m_alphaV);
		stream->writeFloat(m_eta);
		stream->writeFloat(m_gamma);
		stream->writeFloat(m_avgQueryArea);
		m_reflectance.serialize(stream);
		stream->writeFloat(m_spatialLookupMultiplier);
		stream->writeInt(m_maxDepth);
		stream->writeFloat(m_errorThreshold);
		stream->writeBool(m_collectStatistics);
		stream->writeBool(m_clamp);
	}

	void configure() {
		m_components.clear();
		m_components.push_back(EGlossyReflection | EFrontSide);
		m_usesRayDifferentials = true;
		BSDF::configure();
		m_cosGamma = std::cos(m_gamma);
		m_avgQuerySolidAngle = .5f*M_PI*(1-m_cosGamma);
		m_statQueryArea = 0;
		m_statQueries = 0;

		Log(EInfo, "Precomputing triangle integrals ..");
		ref<Timer> timer = new Timer();
		Vector4f integrals;
		m_triIntegrals.clear();
		for (int i=2; i<6; ++i)
			integrals[i-2] = precomputeIntegrals(SphericalTriangle(i));
		Float total = integrals[0] + integrals[1] + integrals[2] + integrals[3];
		m_triIntegrals[1] = integrals / total;
		Log(EInfo, "Done. (took %i ms, %i entries, "
				"normalization = %f)", timer->getMilliseconds(),
				(int) m_triIntegrals.size(), total);//把log消息写入控制台
	}

	Float precomputeIntegrals(const SphericalTriangle &tri) {
		Float rule1 = microfacetD(normalize(tri.v0+tri.v1+tri.v2)),
			  rule2 = (microfacetD(tri.v0) + microfacetD(tri.v1) + microfacetD(tri.v2)) * (1.0f / 3.0f),
			  error = std::abs(rule1-rule2),
			  area  = tri.area();

		Float integral = 0;
		if (error * area < 1e-4f || error < 1e-4f * rule2) {
			integral = rule2 * area;
		} else {
			SphericalTriangle children[4];
			tri.split(children);

			Vector4f recursiveIntegrals;
			for (int i=0; i<4; ++i) {
				Float nested = precomputeIntegrals(children[i]);
				recursiveIntegrals[i] = nested;
				integral += nested;
			}
			if (integral != 0)
				m_triIntegrals[tri.id] = recursiveIntegrals/integral;
			else
				m_triIntegrals[tri.id] = Vector4f(0.25f);
		}
		return integral;
	}

	inline Float microfacetD(const Vector &v, bool sample=false) const {
		Float result = 0;
		Float alphaU = m_alphaU, alphaV = m_alphaV;
		if (sample) {
			alphaU *= ALPHA_SCALE; alphaV *= ALPHA_SCALE;//*2
		}
		if (EXPECT_TAKEN(alphaU == alphaV)) {
			Float mu = v.z, mu2 = mu * mu, mu3 = mu2 * mu;

			if (mu == 0)
				return 0;

			result = std::exp((mu2-1)/(mu2*alphaU*alphaU)) / (M_PI*mu3*alphaU*alphaU);
		} else {
			alphaU = std::max(2 / (alphaU * alphaU) - 2, (Float) 0.1f);//0.1 or
			alphaV = std::max(2 / (alphaV * alphaV) - 2, (Float) 0.1f);

			const Float mu = v.z;
			const Float ds = 1 - mu * mu;
			if (ds < 0)
				return 0.0f;
			const Float exponent = (alphaU * v.x * v.x
					+ alphaV * v.y * v.y) / ds;
			result = std::sqrt((alphaU + 2) * (alphaV + 2))
				* INV_TWOPI * std::pow(mu, exponent);//2派分之一
		}
		return result;
	}

	inline Vector sampleMicrofacetD(const Point2 &sample) const {
		Float cosThetaM, phiM;

		Float alphaU = m_alphaU, alphaV = m_alphaV;
		alphaU *= ALPHA_SCALE; alphaV *= ALPHA_SCALE;

		if (EXPECT_TAKEN(alphaU == alphaV)) {
			Float tanThetaMSqr = -alphaU*alphaU* math::fastlog(1.0f - sample.x);
			cosThetaM = 1.0f / std::sqrt(1 + tanThetaMSqr);
			phiM = (2.0f * M_PI) * sample.y;
		} else {
			alphaU = std::max(2 / (alphaU * alphaU) - 2, (Float) 0.1f);
			alphaV = std::max(2 / (alphaV * alphaV) - 2, (Float) 0.1f);

			/* Sampling method based on code from PBRT */
			if (sample.x < 0.25f) {
				sampleFirstQuadrantAS(alphaU, alphaV,
					4 * sample.x, sample.y, phiM, cosThetaM);
			} else if (sample.x < 0.5f) {
				sampleFirstQuadrantAS(alphaU, alphaV,
					4 * (0.5f - sample.x), sample.y, phiM, cosThetaM);
				phiM = M_PI - phiM;
			} else if (sample.x < 0.75f) {
				sampleFirstQuadrantAS(alphaU, alphaV,
					4 * (sample.x - 0.5f), sample.y, phiM, cosThetaM);
				phiM += M_PI;
			} else {
				sampleFirstQuadrantAS(alphaU, alphaV,
					4 * (1 - sample.x), sample.y, phiM, cosThetaM);
				phiM = 2 * M_PI - phiM;
			}
		}

		const Float sinThetaM = std::sqrt(
			std::max((Float) 0, 1 - cosThetaM*cosThetaM));

		Float sinPhiM, cosPhiM;
		math::sincos(phiM, &sinPhiM, &cosPhiM);

		return Vector(
			sinThetaM * cosPhiM,
			sinThetaM * sinPhiM,
			cosThetaM
		);

	}
	/// Helper routine: sample the first quadrant of the A&S distribution辅助例程 关于a&s第一象限取样
	inline void sampleFirstQuadrantAS(Float alphaU, Float alphaV, Float u1, Float u2,
			Float &phi, Float &cosTheta) const {
		if (alphaU == alphaV)
			phi = M_PI * u1 * 0.5f;
		else
			phi = std::atan(
				std::sqrt((alphaU + 1.0f) / (alphaV + 1.0f)) *
				std::tan(M_PI * u1 * 0.5f));
		const Float cosPhi = std::cos(phi), sinPhi = std::sin(phi);
		cosTheta = std::pow(u2, 1.0f /
			(alphaU * cosPhi * cosPhi + alphaV * sinPhi * sinPhi + 1.0f));
	}

	Float smithG1(const Vector &v, const Vector &m) const {
		Float alpha;
		if (EXPECT_TAKEN(m_alphaU == m_alphaV)) {
			alpha = m_alphaU;
		} else {
			alpha = std::max(m_alphaU, m_alphaV);
		}

		const Float tanTheta = std::abs(Frame::tanTheta(v));

		/* perpendicular incidence -- no shadowing/masking */
		if (tanTheta == 0.0f)
			return 1.0f;

		/* Can't see the back side from the front and vice versa */
		if (dot(v, m) * Frame::cosTheta(v) <= 0)
			return 0.0f;

		Float a = 1.0f / (alpha * tanTheta);

		if (a >= 1.6f)
			return 1.0f;

		/* Use a fast and accurate (<0.35% rel. error) rational
		   approximation to the shadowing-masking function */
		const Float aSqr = a * a;
		return (3.535f * a + 2.181f * aSqr)
			 / (1.0f + 2.276f * a + 2.577f * aSqr);
	}

	inline void splitSpace(const Node &n, Node *ch) const {
		size_t binCounts[4];

		sampleUniformMultinomialApprox(n.id, n.numParticles, binCounts);

		for (int i=0; i<4; ++i) {
			ch[i].id = 4*n.id + i - 2;
			ch[i].dir = n.dir;
			ch[i].numParticles = binCounts[i];
			ch[i].depth = n.depth+1;
			ch[i].uv = n.uv.getChild(i);
		}
	}

	inline void splitDirection(const Node &n, Node *ch) const {
		n.dir.split(ch[0].dir, ch[1].dir, ch[2].dir, ch[3].dir);

		float binProbs[4];
		std::tr1::unordered_map<size_t, Vector4f>::const_iterator it = m_triIntegrals.find(n.dir.id);
		if (it != m_triIntegrals.end()) {
			const Vector4f &probs = it->second;
			for (int i=0; i<4; ++i)
				binProbs[i] = probs[i];
		} else {
			for (int i=0; i<4; ++i)
				binProbs[i] = (float) (microfacetD(ch[i].dir.center()) * ch[i].dir.area());
			float normalization = 1 / (binProbs[0] + binProbs[1] + binProbs[2] + binProbs[3]);
			for (int i=0; i<4; ++i)
				binProbs[i] *= normalization;
		}

		size_t binCounts[4];
		sampleMultinomialApprox(n.id, binProbs, n.numParticles, binCounts);

		for (int i=0; i<4; ++i) {
			ch[i].id = 4*n.id + i - 2;
			ch[i].numParticles = binCounts[i];
			ch[i].depth = n.depth+1;
			ch[i].uv = n.uv;
		}
	}

	Float countParticles(QueryRegion &query) const {
		std::queue<QueueNode> &queue = __queueTLS2.get();
		queue.push(QueueNode(m_particleCount));

		Float result = 0;
		while (!queue.empty()) {
			query.nExpansions++;
			QueueNode node = queue.front();
			queue.pop();

			if ((result > 0 || node.depth > m_maxDepth) && node.dir.area() < 0.5 && query.dir.ellipse && m_triIntegrals.find(node.dir.id) == m_triIntegrals.end()) {
				Float overlap = std::abs(query.uv.overlapAABB(node.uv)) / node.uv.getVolume()
					* query.dir.sphTriangleOverlap(node.dir);

				Float expError2 = std::sqrt(node.numParticles * overlap * (1-overlap));
				Float threshold = m_errorThreshold * result;

				if (expError2 <= threshold * threshold || node.depth > m_maxDepth) {
					result += node.numParticles * overlap;
					continue;
				}
			}

			Node children[4];
			bool subdivideSpace = m_avgQuerySolidAngle * node.uv.getVolume() > m_avgQueryArea * node.dir.area();
			if (subdivideSpace) {
				splitSpace(node, children);
				EIntersectionResult intersectionUV = node.intersectionUV;
				for (int i=0; i<4; ++i) {
					if (children[i].empty())
						continue;
					if (node.intersectionUV != EInside) {
						intersectionUV = query.uv.intersectAABB(children[i].uv);

						if (intersectionUV == EInside && node.intersectionDir == EInside) {
							result += children[i].numParticles;
							continue;
						}
					}
					if (intersectionUV != EDisjoint)
						queue.push(QueueNode(children[i], intersectionUV, node.intersectionDir));
				}
			} else {
				splitDirection(node, children);
				EIntersectionResult intersectionDir = node.intersectionDir;
				for (int i=0; i<4; ++i) {
					if (children[i].empty())
						continue;
					if (node.intersectionDir != EInside) {
						#if defined(SINGLE_PRECISION)
							intersectionDir = query.dir.intersectSphTriangleVectorized(children[i].dir);
						#else
							intersectionDir = query.dir.intersectSphTriangle(children[i].dir);
						#endif
						if (intersectionDir == EInside && node.intersectionUV == EInside) {
							result += children[i].numParticles;
							continue;
						}
					}
					if (intersectionDir != EDisjoint)
						queue.push(QueueNode(children[i], node.intersectionUV, intersectionDir));
				}
			}
		}

		return result;
	}

	Spectrum eval(const BSDFSamplingRecord &bRec, EMeasure measure) const {
		if (!(bRec.typeMask & EGlossyReflection) || measure != ESolidAngle
			|| Frame::cosTheta(bRec.wi) <= 0
			|| Frame::cosTheta(bRec.wo) <= 0)
			return Spectrum(0.0f);

		Float D;
		Vector H = normalize(bRec.wi + bRec.wo);
		if (bRec.its.hasUVPartials) {
			Vector2 v0(bRec.its.dudx, bRec.its.dvdx),
					v1(bRec.its.dudy, bRec.its.dvdy);

			v0 *= m_spatialLookupMultiplier;
			v1 *= m_spatialLookupMultiplier;

			Parallelogram2 uv(bRec.its.uv, v0, v1);

			if (m_clamp) {
				/* Clamp the lengths ratio of v0 and v1 */
				static const Float limit1_2 = std::pow((Float) 1 / ANISOTROPY_LIMIT, (Float) 2);
				static const Float limit2_2 = std::pow((Float) ANISOTROPY_LIMIT, (Float) 2);

				Float dp0 = dot(uv.v0, uv.v0),
					  dp1 = dot(uv.v1, uv.v1);

				bool case1 = dp0 < limit1_2 * dp1, case2 = dp0 > limit2_2 * dp1;

				if (case1 || case2) {
					Float ratio = std::sqrt(dp0/dp1), factor;
					if (case2)
						factor = std::sqrt(ANISOTROPY_LIMIT/ratio);
					else
						factor = 1/std::sqrt(ANISOTROPY_LIMIT*ratio);

					uv.v0 *= factor; uv.v1 /= factor;

					dp0 = dot(uv.v0, uv.v0);
					dp1 = dot(uv.v1, uv.v1);
					++numClampedUV0;
				}

				Float c   = dot(uv.v0, uv.v1),
					  dp2 = dp0 * dp1;

				/* Clamp the sine between v0 and v1 to 1/(anisotropy limit) */
				static const Float sinLimit2 = 1-std::pow(1.0f / ANISOTROPY_LIMIT, 2);
				if (c*c>sinLimit2*dp2) {
					Float
						cp   = std::sqrt(sinLimit2 * dp2) * math::signum(c),
						temp = std::sqrt(dp2-cp*cp),
						det  = mitsuba::det(uv.v0, uv.v1);

					Float theta = 0.5f * std::atan2(
						cp*det + c*temp, cp*c - det*temp
					);

					Float cosTheta, sinTheta;
					math::sincos(theta, &sinTheta, &cosTheta);

					uv.v0 = Vector2(cosTheta*uv.v0.x - sinTheta*uv.v0.y,
						sinTheta*uv.v0.x + cosTheta*uv.v0.y);
					uv.v1 = Vector2(cosTheta*uv.v1.x + sinTheta*uv.v1.y,
						-sinTheta*uv.v1.x + cosTheta*uv.v1.y);
					++numClampedUV1;
				}

				uv.o += (v0 + v1 - uv.v0 - uv.v1) * 0.5f;
			}

			bool clamped = false;
			QueryRegion query(uv, SphericalConic(bRec.wi, bRec.wo, m_gamma, m_clamp, &clamped));
			if (clamped)
				++numClampedDir;

			percHyperbolic.incrementBase();
			if (!query.dir.ellipse)
				++percHyperbolic;

			/* Some more antialiasing */
			query.uv.computeBoundingBox();

			Float nParticles = countParticles(query);

			avgParticles += (size_t) round(nParticles);
			avgParticles.incrementBase(1);
			avgExpansions += query.nExpansions;
			avgExpansions.incrementBase(1);

			D = nParticles / (query.uv.area() * 2*M_PI*(1-m_cosGamma) * m_particleCount);

			if (m_collectStatistics) {
				Float queryArea = atomicAdd(&m_statQueryArea, query.uv.area());
				int64_t numQueries = atomicAdd(&m_statQueries, 1);
				if (numQueries == 1000000) {
					cout << toString() << ": average query area=" << queryArea / numQueries
						<< endl;
				}
			}
		} else {
			D = microfacetD(H) / (4 * absDot(H, bRec.wo));
		}

		Float G = smithG1(bRec.wi, H) * smithG1(bRec.wo, H);
		Float F = m_eta != 0 ? fresnelDielectricExt(dot(bRec.wi, H), m_eta) : (Float) 1;

		/* Calculate the specular reflection component */
		Float value = F * D * G * absDot(H, bRec.wo) / (Frame::cosTheta(bRec.wi));

		return m_reflectance * value;
	}

	Float pdf(const BSDFSamplingRecord &bRec, EMeasure measure) const {
		if (!(bRec.typeMask & EGlossyReflection) || measure != ESolidAngle//measure方法关于立体角的测量
			|| Frame::cosTheta(bRec.wi) <= 0
			|| Frame::cosTheta(bRec.wo) <= 0)
			return 0.0f;

		Float result = 0;
		if (bRec.its.hasUVPartials) {//已经计算纹理坐标部分
			#if 1
				Frame frame(bRec.wo);
				int nSamples = 64;
				#if GAMMA_SCALE == 1//一个像素尺度
					Float cosGamma = m_cosGamma;
				#else
					Float cosGamma = std::cos(m_gamma * GAMMA_SCALE);
				#endif
				for (int i=0; i<nSamples; ++i) {
					Vector perturb = warp::squareToUniformCone(cosGamma, sample02(i));
					Vector wo = frame.toWorld(perturb);//从局部坐标转换为世界坐标
					if (wo.z <= 0)
						continue;
					Vector H = normalize(bRec.wi + wo);//单位化
					/* Jacobian of the half-direction mapping */
					const Float dwh_dwo = 1.0f / (4.0f * dot(wo, H));
					result += microfacetD(H, true) * dwh_dwo;
				}
				result *= 1.0f / nSamples;
			#else
				SphericalConic conic(bRec.wi, bRec.wo, m_gamma);
				size_t nExpansions = 0;
				result = integrateBeckmann(conic, nExpansions);
				result /= (2*M_PI*(1-m_cosGamma));
				avgExpansionsPdf += nExpansions;
				avgExpansionsPdf.incrementBase(1);
			#endif
		} else {
			Vector H = normalize(bRec.wi + bRec.wo);

			/* Jacobian of the half-direction mapping */
			const Float dwh_dwo = 1.0f / (4.0f * dot(bRec.wo, H));

			result = microfacetD(H) * dwh_dwo;//d分布函数
		}

		#if UNIFORM_PROB == 0
			return result;
		#else
			return result * (1-UNIFORM_PROB) + UNIFORM_PROB * INV_TWOPI;
		#endif
	}

	Spectrum sample(BSDFSamplingRecord &bRec, Float &pdf_, const Point2 &sample_) const {
		if (!(bRec.typeMask & EDiffuseReflection) || Frame::cosTheta(bRec.wi) <= 0)
			return Spectrum(0.0f);

		bRec.eta = 1.0f;//取样方向的相对折射率
		bRec.sampledComponent = 0;//由sample采样的成分索引
		bRec.sampledType = EGlossyReflection;// 镜面反射 组件类型

		Point2 sample(sample_);
		if (EXPECT_TAKEN(sample.x >= UNIFORM_PROB)) {
			sample.x = (sample.x - UNIFORM_PROB) * 1.0f/(1.0f-UNIFORM_PROB);

			Vector m = sampleMicrofacetD(sample);//获取法线向量

			bRec.wo = reflect(bRec.wi, m);

			if (bRec.its.hasUVPartials) {
				Point2 sample2 = bRec.sampler->next2D();
				#if GAMMA_SCALE == 1
					Float cosGamma = m_cosGamma;
				#else
					Float cosGamma = std::cos(m_gamma * GAMMA_SCALE);
				#endif
				Vector perturb = warp::squareToUniformCone(cosGamma, sample2);
				bRec.wo = Frame(bRec.wo).toWorld(perturb);
			}

			if (Frame::cosTheta(bRec.wo) <= 0)
				return Spectrum(0.0f);
		} else {
//			sample.x *= 1.0f / UNIFORM_PROB;

			bRec.wo = warp::squareToUniformHemisphere(sample);
		}

		pdf_ = pdf(bRec, ESolidAngle);
		Spectrum value = eval(bRec, ESolidAngle);
		if (pdf_ < RCPOVERFLOW || value.isZero())
			return Spectrum(0.0f);

		return value / pdf_;
	}

	Spectrum sample(BSDFSamplingRecord &bRec, const Point2 &sample) const {
		Float pdf;
		return StochasticReflectance::sample(bRec, pdf, sample);
	}

	bool sampleNoEval(BSDFSamplingRecord &bRec, const Point2 &sample_) const {
		if (!(bRec.typeMask & EDiffuseReflection) || Frame::cosTheta(bRec.wi) <= 0)
			return false;

		bRec.eta = 1.0f;
		bRec.sampledComponent = 0;
		bRec.sampledType = EGlossyReflection;

		Point2 sample(sample_);
		if (EXPECT_TAKEN(sample.x >= UNIFORM_PROB)) {
			sample.x = (sample.x - UNIFORM_PROB) * 1.0f/(1.0f-UNIFORM_PROB);

			Vector m = sampleMicrofacetD(sample);

			bRec.wo = reflect(bRec.wi, m);

			if (bRec.its.hasUVPartials) {
				Point2 sample2 = bRec.sampler->next2D();
				#if GAMMA_SCALE == 1
					Float cosGamma = m_cosGamma;
				#else
					Float cosGamma = std::cos(m_gamma * GAMMA_SCALE);
				#endif
				Vector perturb = warp::squareToUniformCone(cosGamma, sample2);
				bRec.wo = Frame(bRec.wo).toWorld(perturb);
			}

			if (Frame::cosTheta(bRec.wo) <= 0)
				return false;
		} else {
//			sample.x *= 1.0f / UNIFORM_PROB;
			bRec.wo = warp::squareToUniformHemisphere(sample);
		}
		return true;
	}

	void setParent(ConfigurableObject *obj) {
		if (obj->getClass()->derivesFrom(MTS_CLASS(TriMesh)))
			//m_statUVArea += ((TriMesh *) obj)->getUVArea();
				m_statUVArea += 0.5;
	}

	/* Unsupported / unimplemented operations */
	Float getRoughness(const Intersection &its, int component) const { return std::numeric_limits<Float>::infinity(); }
	Spectrum getDiffuseReflectance(const Intersection &its) const { return Spectrum(0.0f); }

	std::string toString() const {
		std::ostringstream oss;
		oss << "StochasticReflectance[" << endl
			<< "  id = \"" << getID() << "\"," << endl
			<< "  particleCount = " << m_particleCount << "," << endl
			<< "  uvArea = " << m_statUVArea << "," << endl
			<< "  actualParticleCount = \"" << m_particleCount * m_statUVArea << "," << endl
			<< "  alphaU = " << m_alphaU << "," << endl
			<< "  alphaV = " << m_alphaV << "," << endl
			<< "  gamma = " << m_gamma << endl
			<< "]";
		return oss.str();
	}

	Shader *createShader(Renderer *renderer) const;

	MTS_DECLARE_CLASS()
private:
	std::tr1::unordered_map<size_t, Vector4f> m_triIntegrals;
	size_t m_particleCount;
	Float m_alphaU, m_alphaV;
	Float m_gamma, m_cosGamma;
	Float m_eta;
	Float m_avgQuerySolidAngle;
	Float m_avgQueryArea;
	Spectrum m_reflectance;
	Float m_spatialLookupMultiplier;
	int m_maxDepth;
	Float m_errorThreshold;
	bool m_collectStatistics;
	bool m_clamp;
	mutable Float m_statQueryArea;
	mutable int64_t m_statQueries;
	Float m_statUVArea;
};


/**
 * GLSL port of the rough conductor shader. This version is much more
 * approximate -- it only supports the Ashikhmin-Shirley distribution,
 * does everything in RGB, and it uses the Schlick approximation to the
 * Fresnel reflectance of conductors. When the roughness is lower than
 * \alpha < 0.2, the shader clamps it to 0.2 so that it will still perform
 * reasonably well in a VPL-based preview.
 */
class StochasticShader : public Shader {
public:
	StochasticShader(Renderer *renderer, const Texture *specularReflectance,
			const Texture *alphaU, const Texture *alphaV) : Shader(renderer, EBSDFShader),
			m_specularReflectance(specularReflectance), m_alphaU(alphaU), m_alphaV(alphaV){
		m_specularReflectanceShader = renderer->registerShaderForResource(m_specularReflectance.get());
		m_alphaUShader = renderer->registerShaderForResource(m_alphaU.get());
		m_alphaVShader = renderer->registerShaderForResource(m_alphaV.get());
	}

	bool isComplete() const {
		return m_specularReflectanceShader.get() != NULL &&
			   m_alphaUShader.get() != NULL &&
			   m_alphaVShader.get() != NULL;
	}

	void putDependencies(std::vector<Shader *> &deps) {
		deps.push_back(m_specularReflectanceShader.get());
		deps.push_back(m_alphaUShader.get());
		deps.push_back(m_alphaVShader.get());
	}

	void cleanup(Renderer *renderer) {
		renderer->unregisterShaderForResource(m_specularReflectance.get());
		renderer->unregisterShaderForResource(m_alphaU.get());
		renderer->unregisterShaderForResource(m_alphaV.get());
	}

	void generateCode(std::ostringstream &oss,
			const std::string &evalName,
			const std::vector<std::string> &depNames) const {
		oss << "float " << evalName << "_D(vec3 m, float alphaU, float alphaV) {" << endl
			<< "    float ct = cosTheta(m), ds = 1-ct*ct;" << endl
			<< "    if (ds <= 0.0)" << endl
			<< "        return 0.0f;" << endl
			<< "    alphaU = 2 / (alphaU * alphaU) - 2;" << endl
			<< "    alphaV = 2 / (alphaV * alphaV) - 2;" << endl
			<< "    float exponent = (alphaU*m.x*m.x + alphaV*m.y*m.y)/ds;" << endl
			<< "    return sqrt((alphaU+2) * (alphaV+2)) * 0.15915 * pow(ct, exponent);" << endl
			<< "}" << endl
			<< endl
			<< "float " << evalName << "_G(vec3 m, vec3 wi, vec3 wo) {" << endl
			<< "    if ((dot(wi, m) * cosTheta(wi)) <= 0 || " << endl
			<< "        (dot(wo, m) * cosTheta(wo)) <= 0)" << endl
			<< "        return 0.0;" << endl
			<< "    float nDotM = cosTheta(m);" << endl
			<< "    return min(1.0, min(" << endl
			<< "        abs(2 * nDotM * cosTheta(wo) / dot(wo, m))," << endl
			<< "        abs(2 * nDotM * cosTheta(wi) / dot(wi, m))));" << endl
			<< "}" << endl
			<< endl
			<< "vec3 " << evalName << "(vec2 uv, vec3 wi, vec3 wo) {" << endl
			<< "   if (cosTheta(wi) <= 0 || cosTheta(wo) <= 0)" << endl
			<< "    	return vec3(0.0);" << endl
			<< "   vec3 H = normalize(wi + wo);" << endl
			<< "   vec3 reflectance = " << depNames[0] << "(uv);" << endl
			<< "   float alphaU = max(0.2, " << depNames[1] << "(uv).r);" << endl
			<< "   float alphaV = max(0.2, " << depNames[2] << "(uv).r);" << endl
			<< "   float D = " << evalName << "_D(H, alphaU, alphaV)" << ";" << endl
			<< "   float G = " << evalName << "_G(H, wi, wo);" << endl
			<< "   return reflectance * (D * G / (4*cosTheta(wi)));" << endl
			<< "}" << endl
			<< endl
			<< "vec3 " << evalName << "_diffuse(vec2 uv, vec3 wi, vec3 wo) {" << endl
			<< "    if (cosTheta(wi) < 0.0 || cosTheta(wo) < 0.0)" << endl
			<< "    	return vec3(0.0);" << endl
			<< "    return " << depNames[0] << "(uv) * inv_pi * cosTheta(wo);"<< endl
			<< "}" << endl;
	}
	MTS_DECLARE_CLASS()
private:
	ref<const Texture> m_specularReflectance;
	ref<const Texture> m_alphaU;
	ref<const Texture> m_alphaV;
	ref<Shader> m_specularReflectanceShader;
	ref<Shader> m_alphaUShader;
	ref<Shader> m_alphaVShader;
};

Shader *StochasticReflectance::createShader(Renderer *renderer) const {
	return new StochasticShader(renderer,
		new ConstantSpectrumTexture(m_reflectance), new ConstantFloatTexture(m_alphaU), new ConstantFloatTexture(m_alphaV));
}

MTS_IMPLEMENT_CLASS(StochasticShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(StochasticReflectance, false, BSDF)
MTS_EXPORT_PLUGIN(StochasticReflectance, "Stochastic Reflectance BRDF")
MTS_NAMESPACE_END
