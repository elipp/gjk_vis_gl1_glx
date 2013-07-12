#include <vector>
#include <iostream>
#include <cstdio>
#include <sys/time.h>

#include "lin_alg.h"

// these have been stolen from SSEPlus (http://sseplus.sourceforge.net/_s_s_e_plus__platform_8h-source.html)

#define __CNST16I28I_( x ) \
((unsigned char)((x) & 0xFF)), ((unsigned char)(((x) >> 8) & 0xFF)) 

#define SSP_CONST_SETR_16I( a, b, c, d, e, f, g, h ) \
	{ __CNST16I28I_((a)), __CNST16I28I_((b)), __CNST16I28I_((c)), __CNST16I28I_((d)),\
	__CNST16I28I_((e)), __CNST16I28I_((f)), __CNST16I28I_((g)), __CNST16I28I_((h)) }
#define SSP_CONST_SET_16I( a, b, c, d, e, f, g, h ) \
	SSP_CONST_SETR_16I( (h), (g), (f), (e), (d), (c), (b), (a) )

inline void print_m128i(__m128i m) {
	_ALIGNED16(int p[4]);
	_mm_store_si128((__m128i*)&p[0], m);

	fprintf(stderr, "%d %d %d %d\n", p[0], p[1], p[2], p[3]);
}

inline void print_m128(__m128 m) {
	_ALIGNED16(float f[4]);
	_mm_store_ps(&f[0], m);

	fprintf(stderr, "%f %f %f %f\n", f[0], f[1], f[2], f[3]);
}

#define BITWISE_CAST(type, var) *((type*)&var)

inline __m128 __mm_blend_ps_emul(__m128 a, __m128 b, int mask) {
	static const __m128i mulShiftImm = _mm_setr_epi32(0x10000000, 0x20000000, 0x40000000, 0x80000000);
	__m128i s = _mm_set1_epi16(mask);
	s = _mm_mullo_epi16(s, mulShiftImm);
	s = _mm_srai_epi32(s, 31);	// now we have extended the four-bit mask into a full xmm register

	//__m128i ra = _mm_and_si128((__m128i)a, s);	// zero out the unwanted stuff from a
	__m128i ra = _mm_and_si128(BITWISE_CAST(__m128i, a), s);	// zero out the unwanted stuff from a
	//__m128i rb = _mm_andnot_si128(s, (__m128i)b);	// zero out unwanted stuff from b
	__m128i rb = _mm_andnot_si128(s, BITWISE_CAST(__m128i, b));	// zero out unwanted stuff from b
	__m128i reti = _mm_or_si128(ra, rb);	// combine them with or
	__m128 ret = BITWISE_CAST(__m128, reti);
	return ret;

}

inline int find_hi_index_ps(__m128i indices, __m128 floats) {

	_ALIGNED16(int ind[4]);
	_mm_store_si128((__m128i*)&ind[0], indices);
	int highest_i = ind[0];

	_ALIGNED16(float f[4]);
	_mm_store_ps(f, floats);
	float highest_f = f[0];
	for (int i = 1; i < 4; ++i) {
		if (f[i] > highest_f) {
			highest_i = ind[i];
			highest_f = f[i];
		}
	}
	return highest_i;

}

static int find_largest_float_index(float *floats_aligned16, int num_floats) {
	int num_runs = num_floats/4;
	int excess = num_floats%4;

	__m128i cur_indices = _mm_set_epi32(3, 2, 1, 0);
	__m128i highest_indices = cur_indices;

	__m128 cur_highest = _mm_load_ps(floats_aligned16);
	static const __m128i increment4 = _mm_set1_epi32(4);

	for (int i = 1; i < num_runs; ++i) {
		__m128 cur = _mm_load_ps(floats_aligned16+16*i);
		__m128 cmp = _mm_cmpge_ps(cur, cur_highest);
		int mask = _mm_movemask_ps(cmp);
		cur_indices = _mm_add_epi32(cur_indices, increment4);
		
		// pick the floats that were bigger than the corresponding float from the last chunk
		cur_highest = __mm_blend_ps_emul(cur, cur_highest, mask);
		// use same mask for indices :P
		__m128 hi_ind_tmp = __mm_blend_ps_emul(BITWISE_CAST(__m128, cur_indices), BITWISE_CAST(__m128, highest_indices), mask);
		highest_indices = BITWISE_CAST(__m128i, hi_ind_tmp);

//		print_m128(cur_highest);
//		print_m128i(highest_indices);
//		fprintf(stderr, "->\n");
	}

	return find_hi_index_ps(highest_indices, cur_highest);
	
}

static vec4 find_farthest_in_direction(mat4 *point_chunks, int num_chunks, const vec4 &D) {

	size_t memsize = 4*(num_chunks+1)*sizeof(__m128);
	__m128 *dps;
	_ALIGNED_MALLOC16(dps, memsize);

	for (int i = 0; i < num_chunks; ++i) {
		dps[i] = dot3x4_notranspose(point_chunks[i], D);
	}

	float *vals;
	_ALIGNED_MALLOC16(vals, memsize);
	memcpy(vals, &dps[0], memsize);

	int index = find_largest_float_index(vals, num_chunks*4);
	
	_ALIGNED_FREE(dps);
	_ALIGNED_FREE(vals);

	return point_chunks[index/4].transposed().column(index%4);
}

void convex_hull(std::vector<vec4> &points) {
	// construct initial tetrahedron.
	int num_pts = points.size();
	int num_runs = num_pts/4;
	int excess = num_runs%4;
	size_t memsize = 4*(num_runs+1)*sizeof(__m128);
	mat4 *pts;
	_ALIGNED_MALLOC16(pts, memsize);
	
	memcpy(&pts[0], &points[0], memsize);
	
	for (int i = 0; i < num_runs; ++i) {
		// allows us to do dot3x4_notranspose (faster) instead of dot3x4_transpose (slower) :P
		pts[i].transpose();
	}

	static const vec4 directions[6] = {
		vec4(1.0, 0.0, 0.0, 0.0),
		vec4(-1.0, 0.0, 0.0, 0.0),
		vec4(0.0, 1.0, 0.0, 0.0),
		vec4(0.0, -1.0, 0.0, 0.0),
		vec4(0.0, 0.0, 1.0, 0.0),
		vec4(0.0, 0.0, -1.0, 0.0)
	};

	vec4 farthest[6];

//#pragma omp parallel for
	for (int i = 0; i < 6; ++i) {
		farthest[i] = find_farthest_in_direction(pts, num_runs, directions[i]);
	}
	
}

inline double get_us(struct timeval t2, struct timeval t1) {
	return (t2.tv_sec*1000000 + t2.tv_usec) - (t1.tv_sec*1000000 + t1.tv_usec);
}

int main(int argc, char *argv[]) {

#define NUM_POINTS 32
	
	std::vector<vec4> points;
	points.reserve(NUM_POINTS);
	for (int i = 0; i < NUM_POINTS; ++i) {
		points.push_back(vec4(i, -i, i, 0.0));
	}

	struct timeval t1, t2;
	gettimeofday(&t1, NULL);
	convex_hull(points);
	gettimeofday(&t2, NULL);

	double us = get_us(t2, t1);

	fprintf(stderr, "took %f us\n", us);
	
	return 0;
}
