#pragma once

#include <Arduino.h>
#include "VectorInt.h"

struct Vector
{
    double x;
    double y;

public:
    static double dotProduct(Vector v1, Vector v2)
    {
        return v1.x * v2.x + v1.y * v2.y;
    }

    static double crossProduct(Vector v1, Vector v2)
    {
        return v1.x * v2.y - v1.y * v2.x;
    }

    static Vector rotateVectorByRightAngle(Vector vector, int angle)
    {
        angle = angle % 4;

        switch (angle)
        {
            case 1:
            return {-vector.y, vector.x};
            case 2:
            return {-vector.x, -vector.y};
            case 3:
            return {vector.y, -vector.x};
            default:
            return vector;
        }
    }

    static Vector rotateVector(Vector vector, double angle)
    {
        double radians = angle * DEG_TO_RAD;
        double sinAngle = sin(radians);
        double cosAngle = cos(radians);

        Vector newVector = {0,0};
        newVector.x = cosAngle * vector.x - sinAngle * vector.y;
        newVector.y = sinAngle * vector.x + cosAngle * vector.y;
        return newVector;
    }

    void normalize()
    {
        double magnitude = sqrt(x * x + y * y);

        if (magnitude <= 0.0)
            return;

        x /= magnitude;
        y /= magnitude;
    }

    void scale(double scale)
    {
        x *= scale;
        y *= scale;
    }

    Vector operator+(const Vector& a) const
    {
        return {x + a.x, y + a.y};
    }

    Vector operator-(const Vector& a) const
    {
        return {x - a.x, y - a.y};
    }

    Vector operator*(const double scale) const
    {
        return {x * scale, y * scale};
    }

    operator VectorInt() const {return {x, y};}
};