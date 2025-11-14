
#pragma once

#include <math.h>
#include <string.h>

typedef struct {
    float x;
    float y;
    float z;
} VectorFloat;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    void rotate(const struct Quaternion *q);
} VectorInt16;

typedef struct Quaternion {
    float w;
    float x;
    float y;
    float z;

    Quaternion(float nw = 1.0f, float nx = 0.0f, float ny = 0.0f, float nz = 0.0f) : w(nw), x(nx), y(ny), z(nz) {};

    struct Quaternion prod(const struct Quaternion &r) const {
        // Quaternion multiplication is defined as:
        //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
        //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
        //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
        //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2)
        return Quaternion(w*r.w - x*r.x - y*r.y - z*r.z,
                          w*r.x + x*r.w + y*r.z - z*r.y,
                          w*r.y - x*r.z + y*r.w + z*r.x,
                          w*r.z + x*r.y - y*r.x + z*r.w);
    }
    struct Quaternion conj() const {
        return Quaternion(w, -x, -y, -z);
    }
} Quaternion;

// Implementation of VectorInt16::rotate
inline void VectorInt16::rotate(const Quaternion *q) {
    // http://www.cprogramming.com/tutorial/3d/quaternions.html
    // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
    // q*v*q^-1
    // (0, x, y, z) = q * (0, vx, vy, vz) * q.inverse()
    Quaternion p(0, (float)x, (float)y, (float)z);
    p = q->prod(p);
    p = p.prod(q->conj());
    x = (int16_t)p.x;
    y = (int16_t)p.y;
    z = (int16_t)p.z;
}
