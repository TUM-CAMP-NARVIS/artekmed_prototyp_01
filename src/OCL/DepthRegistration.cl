/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Set render buffer to zero
void kernel setZero(global float3* points){
  const uint i = get_global_id(0);
  points[i].x = 0;
  points[i].y = 0;
  points[i].z = 0;
}

// Calculate 3d points from rectified depthmap
void kernel depthToPoints(global const DTYPE *depth, global float3 *points){
  const uint i = get_global_id(0);

  const int xD = i % widthD;
  const int yD = i / widthD;

  // retrieve depth value
  const DTYPE z = depth[i];
  if (z != 0) {
    const float vz = z * scaleFactor;
    const float vx = (xD - cxD) / fxD;
    const float vy = (yD - cyD) / fyD;

    points[i].x = -vz * vx;
    points[i].y = -vz * vy;
    points[i].z = -vz;

  } else {
    points[i].x = points[i].y = points[i].z = 0.;
  }
}
