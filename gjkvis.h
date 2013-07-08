#ifndef GJKVIS_H
#define GJKVIS_H

#include <map>
#include <vector>

#include "lin_alg.h"

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600

extern const mat4 indicator_proj; 

_BEGIN_ALIGN16
struct simplex {
	_ALIGNED16(vec4 points[4]);
	_DEFINE_ALIGNED_MALLOC_FREE_MEMBERS;
} _END_ALIGN16;

#endif
