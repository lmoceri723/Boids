// utils.h
#ifndef UTILS_H
#define UTILS_H

#include "raylib.h"
#include <math.h>

#ifndef EPSILON
#define EPSILON 0.0001f
#endif

Vector3 Vector3Limit(Vector3 v, float max);
void DrawLabel(const char *text, float centerX, float y, int fontSize, Color color);

#endif // UTILS_H
