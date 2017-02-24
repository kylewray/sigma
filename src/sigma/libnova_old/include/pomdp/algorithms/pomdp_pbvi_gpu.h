/**
 *  The MIT License (MIT)
 *
 *  Copyright (c) 2015 Kyle Hollins Wray, University of Massachusetts
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of
 *  this software and associated documentation files (the "Software"), to deal in
 *  the Software without restriction, including without limitation the rights to
 *  use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 *  the Software, and to permit persons to whom the Software is furnished to do so,
 *  subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 *  FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 *  COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 *  IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 *  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


#ifndef POMDP_PBVI_GPU_H
#define POMDP_PBVI_GPU_H


#include "pomdp.h"
#include "policies/pomdp_alpha_vectors.h"

namespace nova {

/**
 *  Execute the entire PBVI process for the infinite horizon POMDP model specified using the GPU.
 *  @param  pomdp           The POMDP object.
 *  @param  numThreads      The number of CUDA threads per block. Use multiples of 32.
 *  @param  initialGamma    The initial set of alpha vectors (r-n array).
 *  @param  policy          The resultant set of alpha-vectors. This will be created and modified.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int pomdp_pbvi_complete_gpu(POMDP *pomdp, unsigned int numThreads, const float *initialGamma,
        POMDPAlphaVectors *&policy);

/**
 *  Step 1/3: The initialization step of PBVI. This sets up the Gamma, pi, and alphaBA variables.
 *  @param  pomdp           The POMDP object.
 *  @param  initialGamma    The initial set of alpha vectors (r-n array).
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int pomdp_pbvi_initialize_gpu(POMDP *pomdp, const float *initialGamma);

/**
 *  Step 2/3: Execute PBVI for the infinite horizon POMDP model specified.
 *  @param  pomdp           The POMDP object.
 *  @param  numThreads      The number of CUDA threads per block. Use multiples of 32.
 *  @param  initialGamma    The initial set of alpha vectors (r-n array).
 *  @param  policy          The resultant set of alpha-vectors. This will be created and modified.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int pomdp_pbvi_execute_gpu(POMDP *pomdp, unsigned int numThreads, const float *initialGamma,
        POMDPAlphaVectors *&policy);

/**
 *  Step 3/3: The uninitialization step of PBVI. This sets up the Gamma, pi, and alphaBA variables.
 *  @param  pomdp   The POMDP object.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int pomdp_pbvi_uninitialize_gpu(POMDP *pomdp);

/**
 *  The update step of PBVI. This applies the PBVI procedure once.
 *  @param  pomdp           The POMDP object.
 *  @param  numThreads      The number of CUDA threads per block. Use multiples of 32.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int pomdp_pbvi_update_gpu(POMDP *pomdp, unsigned int numThreads);

/**
 *  The get resultant policy step of PBVI. This retrieves the alpha-vectors (Gamma) and
 *  corresponding actions (pi).
 *  @param  pomdp   The POMDP object.
 *  @param  policy  The resultant set of alpha-vectors. This will be created and modified.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int pomdp_pbvi_get_policy_gpu(const POMDP *pomdp, POMDPAlphaVectors *&policy);

};


#endif // POMDP_PBVI_GPU_H

