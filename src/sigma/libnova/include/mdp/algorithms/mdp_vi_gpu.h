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


#ifndef MDP_VI_GPU_H
#define MDP_VI_GPU_H


#include "mdp.h"
#include "policies/mdp_value_function.h"

namespace nova {

/**
 *  Execute value iteration for the MDP model specified until convergence. Uses the GPU (CUDA).
 *  @param  mdp         The MDP object.
 *  @param  numThreads  The number of CUDA threads per block. Use 128, 256, or 512
 *                      (multiples of 32).
 *  @param  Vinitial    The initial value function, mapping states (n-array) to floats.
 *  @param  policy      The resulting value function policy. This will be created and modified.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int mdp_vi_complete_gpu(MDP *mdp, unsigned int numThreads, const float *Vinitial,
        MDPValueFunction *&policy);

/**
 *  Step 1/3: The initialization step of VI. This sets up the V and pi variables.
 *  @param  mdp         The MDP object.
 *  @param  Vinitial    The initial value function, mapping states (n-array) to floats.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int mdp_vi_initialize_gpu(MDP *mdp, const float *Vinitial);

/**
 *  Step 2/3: Execute VI for the MDP model specified.
 *  @param  mdp         The MDP object.
 *  @param  numThreads  The number of CUDA threads per block. Use multiples of 32.
 *  @param  Vinitial    The initial value function, mapping states (n-array) to floats.
 *  @param  policy      The resulting value function policy. This will be created and modified.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int mdp_vi_execute_gpu(MDP *mdp, unsigned int numThreads, const float *Vinitial,
        MDPValueFunction *&policy);

/**
 *  Step 3/3: The uninitialization step of VI. This sets up the V and pi variables.
 *  @param  mdp     The MDP object.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int mdp_vi_uninitialize_gpu(MDP *mdp);

/**
 *  The update step of VI. This applies the VI procedure once.
 *  @param  mdp             The MDP object.
 *  @param  numThreads      The number of CUDA threads per block. Use multiples of 32.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int mdp_vi_update_gpu(MDP *mdp, unsigned int numThreads);

/**
 *  The get resultant policy step of VI. This retrieves the values of states (V) and
 *  the corresponding actions at each state (pi).
 *  @param  mdp     The MDP object.
 *  @param  policy  The resulting value function policy. This will be created and modified.
 *  @return Returns zero upon success, non-zero otherwise.
 */
extern "C" int mdp_vi_get_policy_gpu(const MDP *mdp, MDPValueFunction *&policy);

};


#endif // MDP_VI_GPU_H

