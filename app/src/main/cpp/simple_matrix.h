#ifndef SIMPLE_MATRIX_H
#define SIMPLE_MATRIX_H

#include <cmath>
#include <cstring>
#include <algorithm>

// Fixed size matrix to avoid heap allocation
// Max size 16x16 (256 floats = 1KB). 
// Safe for stack.

class Mat {
public:
    int rows;
    int cols;
    float data[256]; // Row-major

    Mat() : rows(0), cols(0) {}
    Mat(int r, int c) : rows(r), cols(c) {
        // memset(data, 0, sizeof(float)*r*c); // Optional, but safer
        for(int i=0; i<r*c; i++) data[i] = 0.0f;
    }
    
    // Copy constructor is default (member-wise copy of array is fine)

    static Mat identity(int n) {
        Mat res(n, n);
        for (int i = 0; i < n; i++) res(i, i) = 1.0f;
        return res;
    }

    float& operator()(int r, int c) {
        return data[r * cols + c];
    }

    const float& operator()(int r, int c) const {
        return data[r * cols + c];
    }

    Mat operator+(const Mat& other) const {
        Mat res(rows, cols);
        int len = rows * cols;
        for (int i = 0; i < len; i++) res.data[i] = data[i] + other.data[i];
        return res;
    }

    Mat operator-(const Mat& other) const {
        Mat res(rows, cols);
        int len = rows * cols;
        for (int i = 0; i < len; i++) res.data[i] = data[i] - other.data[i];
        return res;
    }

    Mat operator*(const Mat& other) const {
        Mat res(rows, other.cols);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < other.cols; j++) {
                float sum = 0.0f;
                for (int k = 0; k < cols; k++) {
                    sum += (*this)(i, k) * other(k, j);
                }
                res(i, j) = sum;
            }
        }
        return res;
    }
    
    Mat operator*(float scalar) const {
        Mat res(rows, cols);
        int len = rows * cols;
        for (int i=0; i<len; i++) res.data[i] = data[i] * scalar;
        return res;
    }

    Mat transpose() const {
        Mat res(cols, rows);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                res(j, i) = (*this)(i, j);
            }
        }
        return res;
    }

    // Gaussian elimination for inverse
    Mat inverse() const {
        if (rows != cols) return *this; 
        int n = rows;
        Mat res = Mat::identity(n);
        Mat temp = *this;

        for (int i = 0; i < n; i++) {
            int pivot = i;
            for (int j = i + 1; j < n; j++) {
                if (std::abs(temp(j, i)) > std::abs(temp(pivot, i))) pivot = j;
            }
            
            if (pivot != i) {
                // Swap rows
                for (int j = 0; j < n; j++) {
                    std::swap(temp(i, j), temp(pivot, j));
                    std::swap(res(i, j), res(pivot, j));
                }
            }

            float div = temp(i, i);
            if (std::abs(div) < 1e-9f) {
                 // Singular matrix, return Identity or Zero? 
                 // Zero is safer to indicate failure, but Identity might be less explosive.
                 // Let's return Zero matrix to avoid propagating bad values.
                 return Mat(n, n); 
            }

            float invDiv = 1.0f / div;
            for (int j = 0; j < n; j++) {
                temp(i, j) *= invDiv;
                res(i, j) *= invDiv;
            }

            for (int k = 0; k < n; k++) {
                if (k != i) {
                    float mul = temp(k, i);
                    for (int j = 0; j < n; j++) {
                        temp(k, j) -= mul * temp(i, j);
                        res(k, j) -= mul * res(i, j);
                    }
                }
            }
        }
        return res;
    }
};

#endif
