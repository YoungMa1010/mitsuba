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

#include <mitsuba/render/bsdf.h>
#include <mitsuba/render/texture.h>
#include <mitsuba/hw/basicshader.h>
#include <mitsuba/core/warp.h>
#include <mitsuba/core/fwd.h>
#include <mitsuba/core/spectrum.h>
#include <mitsuba/core/math.h>
#include <mitsuba/core/frame.h>
#include <mitsuba/bidir/common.h>
#include <stdio.h>
#include <Windows.h>
#include <iostream>
#include <fstream>
#include <string>
#include <direct.h>
using namespace std;
#define m 6561
#define n 4
#define pixel 256
#define lnum 81
#define vnum 81
MTS_NAMESPACE_BEGIN

#define RED_SCALE (1.0/1500.0)
#define GREEN_SCALE (1.15/1500.0)
#define BLUE_SCALE (1.66/1500.0)

int newData[m][n];

bool read_angle(){												// 得到索引表（4列：相机俯仰角、相机方位角、光源俯仰角、光源方位角）
	fstream in("//v_l_index.txt");
	cin.rdbuf(in.rdbuf());
	for (int i = 0; i < 6561; i++){
		for (int j = 0; j < 4; j++)
			cin >> newData[i][j];
	}
	/*FILE* fp=fopen("D://btf//btf_data//IMPALLA//out//v_l_index.txt","r");
	for(int i=0;i<6561;i++){
	for(int j=0;j<4;j++){
	if(!feof(fp)) fscanf(fp,"%d",&newData[i][j]);
	else break;
	}
	}*/
	return true;
}

int findIndex(double theta_out, double phi_out, double theta_in, double phi_in) {
	//确定索引，按列(相机俯仰角、相机方位角、光源俯仰角、光源方位角)逐步缩小范围
	double position[] = { theta_out, phi_out, theta_in, phi_in };
	int index_front = 0, index_back = 6560, pos = 0;
	int front_dist = 0, back_dist = 0;// 向前差距和向后差距

	for (int j = 0; j < 4; ++j) {
		pos = index_front; // 从front的位置开始
		while ((newData[pos][j] <= position[j]) && (pos < index_back))
			++pos;

		// 如果找到了比position[j]大的数
		if (newData[pos][j] > position[j]) {
			front_dist = newData[pos][j] - position[j];
			back_dist = position[j] - newData[pos - 1][j];

			if (front_dist <= back_dist) {
				index_front = pos;
				while ((pos < index_back) && (newData[pos][j] == newData[pos + 1][j]))
					pos++;
				// 不论退出循环是因为哪个括号不成立，index_back都应该赋值为pos
				index_back = pos;
			}
			else {
				index_back = --pos;
				while ((pos > index_front) && (newData[pos][j] == newData[pos - 1][j]))
					pos--;
				index_front = pos;
			}
		}

		// 如果找不到比position[j]大的，取最大值近似
		else {
			pos = index_back;
			while ((pos > index_front) && (newData[pos][j] == newData[pos - 1][j]))
				pos--;
			index_front = pos;
		}

		// 每次循环，输出划定的范围
		cout << index_front << " " << index_back << endl;
	}
	return index_back;
}

bool read = read_angle();

/*const char *RGBfile = "D://btf//abrdf_data//IMPALLA//bin//";*///获得rgb
const char* getRGBfile(std::string str){
	char exeFullPath[MAX_PATH]; // Full path
	getcwd(exeFullPath, MAX_PATH);
	int CountOfBlanks = 0;
	for (int i = 0; i<strlen(exeFullPath); i++)
	if (exeFullPath[i] == '\\')
		++CountOfBlanks;
	int len = strlen(exeFullPath) + CountOfBlanks;
	if (len + 1>MAX_PATH)
		return 0;
	char* pStr1 = exeFullPath + strlen(exeFullPath);
	char* pStr2 = exeFullPath + len;
	while (pStr1<pStr2)
	{
		if (*pStr1 == '\\')
		{
			*pStr2-- = '/';
			*pStr2-- = '/';
		}
		else
		{
			*pStr2-- = *pStr1;
		}
		--pStr1;
	}
	string strPath = "";
	strPath = (string)exeFullPath;    // Get full path of the file
	strPath += str;
	const char *cha = strPath.c_str();
	return cha;
}
const char *RGBfile1 = getRGBfile("//mat2txt//");//获得rgb

inline static Float theta_inOrOut(Vector wt){

	double a, b, radian, angle;
	a = sqrt(pow(wt.x, 2) + pow(wt.y, 2) + pow(wt.z, 2));
	b = wt.z / a;
	radian = acos(b);
	angle = radian * 180 / 3.1415;
	return angle;
}
inline static Float phi_inOrOut(Vector wp){

	double a, b, radian, angle;
	a = sqrt(pow(wp.x, 2) + pow(wp.y, 2));
	b = wp.x / a;
	radian = acos(b);
	angle = radian * 180 / 3.1415;
	if (wp.y < 0){
		angle = angle + 180;
	}
	return angle;
}
int round(double number)
{
	return (number > 0.0) ? floor(number + 0.5) : ceil(number - 0.5);
}

class SmoothDiffuse : public BSDF {
public:
	SmoothDiffuse(const Properties &props)
		: BSDF(props) {
		/* For better compatibility with other models, support both
		'reflectance' and 'diffuseReflectance' as parameter names */
		m_reflectance = new ConstantSpectrumTexture(props.getSpectrum(
			props.hasProperty("reflectance") ? "reflectance"
			: "diffuseReflectance", Spectrum(.5f)));
	}

	SmoothDiffuse(Stream *stream, InstanceManager *manager)
		: BSDF(stream, manager) {
		m_reflectance = static_cast<Texture *>(manager->getInstance(stream));

		configure();
	}

	void configure() {
		/* Verify the input parameter and fix them if necessary */
		m_reflectance = ensureEnergyConservation(m_reflectance, "reflectance", 1.0f);

		m_components.clear();
		if (m_reflectance->getMaximum().max() > 0)
			m_components.push_back(EDiffuseReflection | EFrontSide
			| (m_reflectance->isConstant() ? 0 : ESpatiallyVarying));
		m_usesRayDifferentials = m_reflectance->usesRayDifferentials();

		BSDF::configure();
	}

	Spectrum getDiffuseReflectance(const Intersection &its) const {
		return m_reflectance->eval(its);
	}

	Spectrum getAlbedo(const Intersection &its) const
	{
		if (Frame::cosTheta(its.wi) <= 0)
			return Spectrum(0.0f);
		return m_reflectance->eval(its);
	}

	Spectrum get_irradiance(const BSDFSamplingRecord &bRec) const
	{
		if (!(bRec.typeMask & EDiffuseReflection) || Frame::cosTheta(bRec.wi) <= 0)
			return Spectrum(0.0f);
		//bRec.eta = 1.0f;
		//bRec.sampledComponent = 0;
		//bRec.sampledType = EDiffuseReflection;
		return m_reflectance->eval(bRec.its)/** INV_PI*/;


	}
	Spectrum eval(const BSDFSamplingRecord &bRec, EMeasure measure) const {
		if (!(bRec.typeMask & EDiffuseReflection) || measure != ESolidAngle
			|| Frame::cosTheta(bRec.wi) <= 0
			|| Frame::cosTheta(bRec.wo) <= 0)
			return Spectrum(0.0f);

		Vector wi = bRec.wi, wo = bRec.wo;
		double theta_in, phi_in, theta_out, phi_out;		//vector2angle
		theta_out = theta_inOrOut(wo);
		phi_out = phi_inOrOut(wo);
		theta_in = theta_inOrOut(wi);
		phi_in = phi_inOrOut(wi);

		/*cout << theta_out<<" ";
		cout << phi_out << " ";
		cout << theta_in << " ";
		cout<<phi_in<<" ";*/
		//Log(EInfo, "%f,%f,%f,%f",theta_out,phi_out,theta_in,phi_in);

		if (!read){ exit(1); }

		int nLine = findIndex(theta_out, phi_out, theta_in, phi_in);//第nLine张图
		//Log(EInfo, "%i", nLine);
		int x = round(pixel*bRec.its.uv.x), y = round(pixel*bRec.its.uv.y);
		if (x == 0) x = 1;
		if (y == 0) y = 1;
		char  xchar[20], ychar[20], xychar[20];
		int xyIndex = (x - 1) * 256 + y;
		//Log(EInfo, "%i", xyIndex);
		sprintf_s(xchar, "%04d", x);
		sprintf_s(ychar, "%04d", y);
		sprintf_s(xychar, "%06d", xyIndex);
		string str1(RGBfile1);
		//string file = str1 + xychar + " " + xchar + " " + ychar + ".bin";
		string file = str1 + xychar + " " + xchar + " " + ychar + ".txt";
		double rgb[6561][3];
		fstream in(file);
		cin.rdbuf(in.rdbuf());
		for (int i = 0; i < 6561; i++){
			for (int j = 0; j < 3; j++)
				cin >> rgb[i][j];
		}

		/*char fileName[100];
		strcpy_s(fileName, file.c_str());
		double* abrdf;
		FILE *f;
		fopen_s(&f, fileName, "rb");
		abrdf = (double*)malloc(sizeof(double)* 3 * vnum * lnum);
		fread(abrdf, sizeof(double), 3 * vnum * lnum, f);
		fclose(f);*/

		Spectrum col;

		//Log(EInfo, "%f,%f,%f", abrdf[nLine * 3], abrdf[nLine * 3 + 1], abrdf[nLine * 3 + 2]);
		/*col[0] = abrdf[nLine * 3]/256;
		col[1] = abrdf[nLine * 3 + 1]/256;
		col[2] = abrdf[nLine * 3 + 2]/256;*/

		//Log(EInfo, "%f,%f,%f", rgb[nLine-1][0], rgb[nLine-1][1], rgb[nLine-1][2]);
		col[0] = rgb[nLine][0] * RED_SCALE;
		col[1] = rgb[nLine][1] * GREEN_SCALE;
		col[2] = rgb[nLine][2] * BLUE_SCALE;

		/*free(abrdf);*/
		col.fromLinearRGB(col[0], col[1], col[2], Spectrum::EReflectance);
		return col;
	}

	Float pdf(const BSDFSamplingRecord &bRec, EMeasure measure) const {
		if (!(bRec.typeMask & EDiffuseReflection) || measure != ESolidAngle
			|| Frame::cosTheta(bRec.wi) <= 0
			|| Frame::cosTheta(bRec.wo) <= 0)
			return 0.0f;

		return warp::squareToCosineHemispherePdf(bRec.wo);
	}

	Spectrum sample(BSDFSamplingRecord &bRec, const Point2 &sample) const {
		if (!(bRec.typeMask & EDiffuseReflection) || Frame::cosTheta(bRec.wi) <= 0)
			return Spectrum(0.0f);

		bRec.wo = warp::squareToCosineHemisphere(sample);
		bRec.eta = 1.0f;
		bRec.sampledComponent = 0;
		bRec.sampledType = EDiffuseReflection;
		return m_reflectance->eval(bRec.its);
	}

	Spectrum sample(BSDFSamplingRecord &bRec, Float &pdf, const Point2 &sample) const {
		if (!(bRec.typeMask & EDiffuseReflection) || Frame::cosTheta(bRec.wi) <= 0)
			return Spectrum(0.0f);

		bRec.wo = warp::squareToCosineHemisphere(sample);
		bRec.eta = 1.0f;
		bRec.sampledComponent = 0;
		bRec.sampledType = EDiffuseReflection;
		pdf = warp::squareToCosineHemispherePdf(bRec.wo);
		return m_reflectance->eval(bRec.its);
	}

	void addChild(const std::string &name, ConfigurableObject *child) {
		if (child->getClass()->derivesFrom(MTS_CLASS(Texture))
			&& (name == "reflectance" || name == "diffuseReflectance")) {
			m_reflectance = static_cast<Texture *>(child);
		}
		else {
			BSDF::addChild(name, child);
		}
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		BSDF::serialize(stream, manager);

		manager->serialize(stream, m_reflectance.get());
	}

	Float getRoughness(const Intersection &its, int component) const {
		return std::numeric_limits<Float>::infinity();
	}

	std::string toString() const {
		std::ostringstream oss;
		oss << "SmoothDiffuse[" << endl
			<< "  id = \"" << getID() << "\"," << endl
			<< "  reflectance = " << indent(m_reflectance->toString()) << endl
			<< "]";
		return oss.str();
	}

	Shader *createShader(Renderer *renderer) const;

	MTS_DECLARE_CLASS()
private:
	ref<Texture> m_reflectance;
};

// ================ Hardware shader implementation ================

class SmoothDiffuseShader : public Shader {
public:
	SmoothDiffuseShader(Renderer *renderer, const Texture *reflectance)
		: Shader(renderer, EBSDFShader), m_reflectance(reflectance) {
		m_reflectanceShader = renderer->registerShaderForResource(m_reflectance.get());
	}

	bool isComplete() const {
		return m_reflectanceShader.get() != NULL;
	}

	void cleanup(Renderer *renderer) {
		renderer->unregisterShaderForResource(m_reflectance.get());
	}

	void putDependencies(std::vector<Shader *> &deps) {
		deps.push_back(m_reflectanceShader.get());
	}

	void generateCode(std::ostringstream &oss,
		const std::string &evalName,
		const std::vector<std::string> &depNames) const {
		oss << "vec3 " << evalName << "(vec2 uv, vec3 wi, vec3 wo) {" << endl
			<< "    if (cosTheta(wi) < 0.0 || cosTheta(wo) < 0.0)" << endl
			<< "    	return vec3(0.0);" << endl
			<< "    return " << depNames[0] << "(uv) * inv_pi * cosTheta(wo);" << endl
			<< "}" << endl
			<< endl
			<< "vec3 " << evalName << "_diffuse(vec2 uv, vec3 wi, vec3 wo) {" << endl
			<< "    return " << evalName << "(uv, wi, wo);" << endl
			<< "}" << endl;
	}

	MTS_DECLARE_CLASS()
private:
	ref<const Texture> m_reflectance;
	ref<Shader> m_reflectanceShader;
};

Shader *SmoothDiffuse::createShader(Renderer *renderer) const {
	return new SmoothDiffuseShader(renderer, m_reflectance.get());
}

MTS_IMPLEMENT_CLASS(SmoothDiffuseShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(SmoothDiffuse, false, BSDF)
MTS_EXPORT_PLUGIN(SmoothDiffuse, "Smooth diffuse BRDF")
MTS_NAMESPACE_END
