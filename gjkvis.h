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
	vec4 points[4];
} _END_ALIGN16;

#endif
