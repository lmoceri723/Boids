// utils.c
#include "utils.h"
#include "config.h"
#include <stdio.h>
#include "raymath.h"
#include <math.h>

// Limits the magnitude of a Vector3
Vector3 Vector3Limit(Vector3 v, float max) {
    float lengthSq = Vector3LengthSqr(v);
    if (lengthSq > (max * max) && lengthSq > EPSILON) {
        float length = sqrtf(lengthSq);
        v.x = (v.x / length) * max;     
        v.y = (v.y / length) * max;
        v.z = (v.z / length) * max;
    }
    return v;
}

// Draws a label at a specific position
void DrawLabel(const char *text, float centerX, float y, int fontSize, Color color) {
    float textWidth = MeasureText(text, fontSize);
    DrawText(text, (int)(centerX - textWidth / 2.0f), (int)y, fontSize, color);
}
