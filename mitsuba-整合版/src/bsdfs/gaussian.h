#if !defined(__MULTINOMIAL_H)
#define __MULTINOMIAL_H

MTS_NAMESPACE_BEGIN

#include <immintrin.h>
#define USE_SSE2
#include "sse_mathfun.h"

#if !defined(MM_ALIGN16)
#define MM_ALIGN16             __attribute__ ((aligned (16)))
#endif

static const __m128i C0 = _mm_set1_epi32(0x9e3779b9);
static const __m128i C1 = _mm_set1_epi32(0xA341316C);
static const __m128i C2 = _mm_set1_epi32(0xC8013EA4);
static const __m128i C3 = _mm_set1_epi32(0xAD90777D);
static const __m128i C4 = _mm_set1_epi32(0x7E95761E);
static const __m128i C5 = _mm_set1_epi32(0x3f800000);
static const __m128  C6 = _mm_set1_ps(1.0f);
static const __m128i C7 = _mm_set_epi32(9, 7, 3, 1);
static const __m128  C8 = _mm_set1_ps(2.0f*M_PI);
static const __m128  C9 = _mm_set1_ps(-2.0f);

inline __m128 sampleTEA(__m128i v0, __m128i v1) {
	__m128i sum = C0;

#if RNG_ROUNDS >= 2
	v0 = _mm_add_epi32(v0, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v1, 4), C1),
			_mm_add_epi32(v1, sum)),
			_mm_add_epi32(_mm_srli_epi32(v1, 5), C2)));

	v1 = _mm_add_epi32(v1, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v0, 4), C3),
			_mm_add_epi32(v0, sum)),
			_mm_add_epi32(_mm_srli_epi32(v0, 5), C4)));

	sum = _mm_add_epi32(sum, C0);
#endif

#if RNG_ROUNDS >= 3
	v0 = _mm_add_epi32(v0, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v1, 4), C1),
			_mm_add_epi32(v1, sum)),
			_mm_add_epi32(_mm_srli_epi32(v1, 5), C2)));

	v1 = _mm_add_epi32(v1, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v0, 4), C3),
			_mm_add_epi32(v0, sum)),
			_mm_add_epi32(_mm_srli_epi32(v0, 5), C4)));

	sum = _mm_add_epi32(sum, C0);
#endif

#if RNG_ROUNDS >= 4
	v0 = _mm_add_epi32(v0, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v1, 4), C1),
			_mm_add_epi32(v1, sum)),
			_mm_add_epi32(_mm_srli_epi32(v1, 5), C2)));

	v1 = _mm_add_epi32(v1, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v0, 4), C3),
			_mm_add_epi32(v0, sum)),
			_mm_add_epi32(_mm_srli_epi32(v0, 5), C4)));

	sum = _mm_add_epi32(sum, C0);
#endif

#if RNG_ROUNDS >= 5
	v0 = _mm_add_epi32(v0, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v1, 4), C1),
			_mm_add_epi32(v1, sum)),
			_mm_add_epi32(_mm_srli_epi32(v1, 5), C2)));

	v1 = _mm_add_epi32(v1, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v0, 4), C3),
			_mm_add_epi32(v0, sum)),
			_mm_add_epi32(_mm_srli_epi32(v0, 5), C4)));

	sum = _mm_add_epi32(sum, C0);
#endif
#if RNG_ROUNDS >= 6
	v0 = _mm_add_epi32(v0, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v1, 4), C1),
			_mm_add_epi32(v1, sum)),
			_mm_add_epi32(_mm_srli_epi32(v1, 5), C2)));

	v1 = _mm_add_epi32(v1, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v0, 4), C3),
			_mm_add_epi32(v0, sum)),
			_mm_add_epi32(_mm_srli_epi32(v0, 5), C4)));

	sum = _mm_add_epi32(sum, C0);
#endif
#if RNG_ROUNDS >= 7
	v0 = _mm_add_epi32(v0, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v1, 4), C1),
			_mm_add_epi32(v1, sum)),
			_mm_add_epi32(_mm_srli_epi32(v1, 5), C2)));

	v1 = _mm_add_epi32(v1, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v0, 4), C3),
			_mm_add_epi32(v0, sum)),
			_mm_add_epi32(_mm_srli_epi32(v0, 5), C4)));

	sum = _mm_add_epi32(sum, C0);
#endif

#if RNG_ROUNDS >= 8
	v0 = _mm_add_epi32(v0, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v1, 4), C1),
			_mm_add_epi32(v1, sum)),
			_mm_add_epi32(_mm_srli_epi32(v1, 5), C2)));

	v1 = _mm_add_epi32(v1, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v0, 4), C3),
			_mm_add_epi32(v0, sum)),
			_mm_add_epi32(_mm_srli_epi32(v0, 5), C4)));

	sum = _mm_add_epi32(sum, C0);
#endif

#if RNG_ROUNDS >= 9
	v0 = _mm_add_epi32(v0, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v1, 4), C1),
			_mm_add_epi32(v1, sum)),
			_mm_add_epi32(_mm_srli_epi32(v1, 5), C2)));

	v1 = _mm_add_epi32(v1, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v0, 4), C3),
			_mm_add_epi32(v0, sum)),
			_mm_add_epi32(_mm_srli_epi32(v0, 5), C4)));

	sum = _mm_add_epi32(sum, C0);
#endif

#if RNG_ROUNDS >= 10
	v0 = _mm_add_epi32(v0, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v1, 4), C1),
			_mm_add_epi32(v1, sum)),
			_mm_add_epi32(_mm_srli_epi32(v1, 5), C2)));

	v1 = _mm_add_epi32(v1, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v0, 4), C3),
			_mm_add_epi32(v0, sum)),
			_mm_add_epi32(_mm_srli_epi32(v0, 5), C4)));

	sum = _mm_add_epi32(sum, C0);
#endif

#if RNG_ROUNDS >= 11
	v0 = _mm_add_epi32(v0, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v1, 4), C1),
			_mm_add_epi32(v1, sum)),
			_mm_add_epi32(_mm_srli_epi32(v1, 5), C2)));

	v1 = _mm_add_epi32(v1, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v0, 4), C3),
			_mm_add_epi32(v0, sum)),
			_mm_add_epi32(_mm_srli_epi32(v0, 5), C4)));

	sum = _mm_add_epi32(sum, C0);
#endif

#if RNG_ROUNDS >= 12
	v0 = _mm_add_epi32(v0, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v1, 4), C1),
			_mm_add_epi32(v1, sum)),
			_mm_add_epi32(_mm_srli_epi32(v1, 5), C2)));

	v1 = _mm_add_epi32(v1, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v0, 4), C3),
			_mm_add_epi32(v0, sum)),
			_mm_add_epi32(_mm_srli_epi32(v0, 5), C4)));

	sum = _mm_add_epi32(sum, C0);
#endif

	v0 = _mm_add_epi32(v0, _mm_xor_si128(
		_mm_xor_si128(
			_mm_add_epi32(_mm_slli_epi32(v1, 4), C1),
			_mm_add_epi32(v1, sum)),
			_mm_add_epi32(_mm_srli_epi32(v1, 5), C2)));

	return _mm_sub_ps(_mm_castsi128_ps(
		_mm_or_si128(_mm_srli_epi32(v0, 9), C5)), C6);
}

/// Sample from a 4D Gaussian using given a LDL^T decomposition of its covariance
inline void sampleGaussian4DVectorized(uint32_t v0, const float *D, const float *L, const float *mean, size_t tries, float *output) {
	__m128i seed = _mm_set1_epi32(v0);
	__m128 sample = sampleTEA(seed, _mm_mul_epi32(seed, C7));

	__m128 sin, cos;
	sincos_ps(_mm_mul_ps(sample, C8), &sin, &cos);

	__m128 sincos = _mm_shuffle_ps(cos, sin, 221);
	sincos = _mm_shuffle_ps(sincos, sincos, 216);

	__m128 scale = _mm_mul_ps(C9, log_ps(sample));
	__m128 tries_f = _mm_set1_ps((float) tries);

	scale = _mm_shuffle_ps(scale, scale, 160);
	scale = _mm_mul_ps(_mm_mul_ps(scale, _mm_load_ps(D)), tries_f);

	sample = _mm_mul_ps(_mm_sqrt_ps(scale), sincos);

	__m128 prod1 = _mm_dp_ps(_mm_load_ps(L),    sample, 0xFF);
	__m128 prod2 = _mm_dp_ps(_mm_load_ps(L+4),  sample, 0xFF);
	__m128 prod3 = _mm_dp_ps(_mm_load_ps(L+8),  sample, 0xFF);
	__m128 prod4 = _mm_dp_ps(_mm_load_ps(L+12), sample, 0xFF);

	sample = _mm_shuffle_ps(_mm_movelh_ps(prod1, prod2),
		_mm_movelh_ps(prod3, prod4), _MM_SHUFFLE(2, 0, 2, 0));

	_mm_store_ps(output, _mm_add_ps(_mm_mul_ps(_mm_load_ps(mean), tries_f), sample));
}

/// Sample from a 4D Gaussian using given a LDL^T decomposition of its covariance
inline void sampleGaussian4D(uint32_t v0, const float *D, const float L[4][4], const float *mean, size_t tries, float *result) {
	const int rounds = RNG_ROUNDS;

	float u0 = sampleTEASingle(v0, v0, rounds);
	float u1 = 2*M_PI*sampleTEASingle(v0, 3*v0, rounds);
	float u2 = sampleTEASingle(v0, 7*v0, rounds);
	float u3 = 2*M_PI*sampleTEASingle(v0, 9*v0, rounds);

	float scale0 = std::sqrt(-2*std::log(u0));
	float scale1 = std::sqrt(-2*std::log(u2));
	float tries_f = (float) tries;

	float v[4] = {
		scale0 * std::cos(u1)*std::sqrt(D[0]*tries_f),
		scale0 * std::sin(u1)*std::sqrt(D[1]*tries_f),
		scale1 * std::cos(u3)*std::sqrt(D[2]*tries_f),
		scale1 * std::sin(u3)*std::sqrt(D[3]*tries_f)
	};

	for (int i=0; i<4; ++i) {
		float sum = mean[i]*tries_f + v[i];
		for (int j=0; j<i; ++j)
			sum += L[i][j]*v[j];
		result[i] = sum;
	}
}

MTS_NAMESPACE_END

#endif /* __MULTINOMIAL_H */
