/**
 * \file
 *
 * \section LICENSE
 * The MIT License (MIT)
 * 
 * Copyright (c) 2014 VectorNav Technologies, LLC
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * \section DESCRIPTION
 * This header file provides access to the linear algebra features.
 *
 * Notes
 *  - Indexes used within the math library use 0 based indexing. For example,
 *    the first component of a 3 dimensional vector is referenced in code
 *    as vector3->c0.
 */
#ifndef _VN_LINEAR_ALGEBRA_H_
#define _VN_LINEAR_ALGEBRA_H_

/**
 * \brief A vector of length 3.
 */
typedef struct {
	double	c0;		/**< Component 0 */
	double	c1;		/**< Component 1 */
	double	c2;		/**< Component 2 */
} VnVector3;

/**
 * \brief A 3x3 matrix.
 */
typedef struct {
	double c00;		/**< Component 0,0 */
	double c01;		/**< Component 0,1 */
	double c02;		/**< Component 0,2 */
	double c10;		/**< Component 1,0 */
	double c11;		/**< Component 1,1 */
	double c12;		/**< Component 1,2 */
	double c20;		/**< Component 2,0 */
	double c21;		/**< Component 2,1 */
	double c22;		/**< Component 2,2 */
} VnMatrix3x3;

#endif /* _VN_LINEAR_ALGEBRA_H_ */