//=============================================================================
// CLOP Source
//-----------------------------------------------------------------------------
// Reference Implementation for
// Preiner et al., Continuous Projection for Fast L1 Reconstruction, In 
// Proceedings of ACM SIGGRAPH 2014
// www.cg.tuwien.ac.at/research/publications/2014/preiner2014clop
//-----------------------------------------------------------------------------
// (c) Reinhold Preiner, Vienna University of Technology, 2014
// All rights reserved. This code is licensed under the New BSD License:
// http://opensource.org/licenses/BSD-3-Clause
// Contact: rp@cg.tuwien.ac.at
//=============================================================================


#pragma once

#include <vector>
#include <hash_map>
#include <algorithm>
#include <cassert>
#include <iostream>

#include "mixture.hpp"
#include "pointset.hpp"
#include "pointindex.hpp"

using namespace std;




namespace cp
{
	/// continuous LOP functions
	class clop
	{
	public:
		struct Params
		{
			uint	nIterations = 10;				// number of CLOP iterations
			float	kernelRadius = 1.0f;			// basic kernel radius
			bool	doubleInitRadius = true;		// doubles the kernel radius for the initial iteration (recommended for more stable initialization in the presence of stronger outliers)
			bool	interleaveRepulsion = true;		// use interleaved repulsion from the CLOP paper (recommended)
			float	repulsionRadiusFac = 0.5f;		// repulsion kernel radius factor for kernel cutoff (1 - full repulsion, 0 - no repulsion). recommended value is ~ 0.5
			bool	useSoftEta = true;				// uses the more gently decreasing eta = -r from Huang et al. instead of the original eta = 1/(3r³) from Lipman et al. (recommended)
			float	mu = 0.4f;						// balancing factor between attraction and repulsion forces; E = attracion + mu * repulsion, mu \in [0, 0.5]
			bool	useDiscreteLOP = false;			// ignores Gaussian covariances for comparison purposes. applies original WLOP using the Gaussian centers for singular attraction only.
		};


	public:
		// projects points onto mixture, returning the projected point set (the input point set is left unchanged)
		static PointSet* project(const PointSet* points, const Mixture* mixture, const Params& params)
		{
			uint nParticles = points->size();
			uint nGaussians = mixture->size();
			
			
			// create copy of input particle set
			PointSet* particles = new PointSet(points->begin(), points->end());
			
			// create Index of Gaussians in the mixture
			vector<vec3> gaussianCenters(nGaussians);
			for (uint i = 0; i < nGaussians; ++i)
				gaussianCenters[i] = mixture->at(i).mu;
			float maxSearchRadius = params.doubleInitRadius ? 2 * params.kernelRadius : params.kernelRadius;
			PointIndex mixtureIndex(gaussianCenters, maxSearchRadius);
			

			// attraction and repulsion vector buffers
			vector<vec3> attractionVecs(nParticles), repulsionVecs(nParticles);
			
			cout << "Projecting ";

			// iterate
			for (uint i = 0; i < params.nIterations; ++i)
			{
				float h = (params.doubleInitRadius && i == 0) ? 2 * params.kernelRadius : params.kernelRadius;

				// compute attraction
				computeAttraction(i, *particles, *mixture, mixtureIndex, h, params, attractionVecs);


				// if interleaved repulsion activated, compute repulsion only in iteration 1, 3, 5, ...
				if (!params.interleaveRepulsion || (i > 0 && i % 2))
					computeRepulsion( *particles, h, params, repulsionVecs);
				

				// update particles
				for (uint j = 0; j < nParticles; ++j)
				{
					particles->at(j) += attractionVecs[j] + params.mu * repulsionVecs[j];
				}

				cout << ".";
			}
			
			cout << " done" << endl;

			return particles;
		}


	private:
		static void computeAttraction(uint iter, const PointSet& particles, const Mixture& mixture, PointIndex& mixtureIndex, float h, const Params& params, vector<vec3>& outAttractionVecs)
		{
			// use discrete WLOP (Lipman et al., Huang et al.) for comparison
			if (params.useDiscreteLOP)
			{
				wlopAttraction(iter, particles, mixture, mixtureIndex, h, params, outAttractionVecs);
			}
			// use CLOP (Preiner et al.)
			else
			{
				if (iter == 0)	
					clopAttractionInit(particles, mixture, mixtureIndex, h, params, outAttractionVecs);
				else			
					clopAttraction(particles, mixture, mixtureIndex, h, params, outAttractionVecs);
			}
		}

						
		/// CLOP attraction
		static void clopAttraction(const PointSet& particles, const Mixture& mixture, PointIndex& mixtureIndex, float h, const Params& params, vector<vec3>& outAttractionVecs)
		{
			const vec3 coeffS2( 1.3857984173503727e-2f, 1.0804239269332381e-3f, 1.0195911260841634e-4f );	// squares of the \hat{sigma} from the paper
			const vec3 coeffWC3d( 0.2942585010864966f, 0.016715798746987053f, 0.0015851580135012695f );	// w_k weighted c_k for 3D Gaussians
			const vec3 coeffWC2d( 0.9972155768592106f, 0.20288040484312755f, 0.06262815285505632f );		// w_k weighted c_k for 2D Gaussians
			
			float h2 = h*h;


			// precompute inverse Lambda for the Gaussians and the given kernel radius h and the according cofficient factors
			uint nGaussians = mixture.size();
			vector<smat3> invLambda1(nGaussians);
			vector<smat3> invLambda2(nGaussians);
			vector<smat3> invLambda3(nGaussians);
			vector<vec3> preFacs(nGaussians);

			smat3 Lambda, invLambda;
			float invDetLambda;
			vec3 var_k = coeffS2 * h2;
			
			for (uint i = 0; i < nGaussians; ++i)
			{
				const Gaussian& g = mixture[i];
				vec3 preFac;

				// k = 1
				invLambda = inverse(g.cov + smat3(var_k.x, 0, 0, var_k.x, 0, var_k.x));
				invDetLambda = det(invLambda);								// TODO: optimize - det is contained in inverse function
				preFac.x = g.weight * coeffWC3d.x * sqrtf(invDetLambda);
				invLambda1[i] = invLambda;
				
				// k = 2
				invLambda = inverse(g.cov + smat3(var_k.y, 0, 0, var_k.y, 0, var_k.y));
				invDetLambda = det(invLambda);
				preFac.y = g.weight * coeffWC3d.y * sqrtf(invDetLambda);
				invLambda2[i] = invLambda;
				
				// k = 3
				invLambda = inverse(g.cov + smat3(var_k.z, 0, 0, var_k.z, 0, var_k.z));
				invDetLambda = det(invLambda);
				preFac.z = g.weight * coeffWC3d.z * sqrtf(invDetLambda);
				invLambda3[i] = invLambda;

				preFacs[i] = preFac;
			}
			

			// Iterate over neighbors
			vector<uint> neighbors;
			for (uint i = 0; i < particles.size(); ++i)
			{
				// query neighbors
				const vec3& q = particles[i];
				mixtureIndex.radiusSearch(q, h, neighbors);

				// accumulate
				vec4 accumHomogPoint(0, 0, 0, 0);
				for (uint nidx : neighbors)
				{
					const Gaussian& g = mixture[nidx];
					//------------------------------------------------------

					// TRANSFORM: centralize the point q around the Gaussian
					vec3 c = g.mu - q;
					
					// Accumulate		
					vec3 xW(0.0f, 0.0f, 0.0f);
					float W = 0.0f;
					vec3 preFac = preFacs[nidx];
					vec3 LambdaInv_c;
					float wTerm;
					
					LambdaInv_c = invLambda1[nidx] * c;
					wTerm = preFac.x * expf(-0.5f * dot(c, LambdaInv_c));
					// Accumulate		
					W += wTerm;
					xW += (wTerm * coeffS2.x * h2) * LambdaInv_c;
					
					LambdaInv_c = invLambda2[nidx] * c;
					wTerm = preFac.y * expf(-0.5f * dot(c, LambdaInv_c));
					// Accumulate		
					W += wTerm;
					xW += (wTerm * coeffS2.y * h2) * LambdaInv_c;

					LambdaInv_c = invLambda3[nidx] * c;
					wTerm = preFac.z * expf(-0.5f * dot(c, LambdaInv_c));
					// Accumulate		
					W += wTerm;
					xW += (wTerm * coeffS2.z * h2) * LambdaInv_c;


					// weight target point with the component potential and accumulate.
					accumHomogPoint += vec4(xW, W);
				}
				
				// normalize
				outAttractionVecs[i] = accumHomogPoint.w == 0 ? vec3(0, 0, 0) : accumHomogPoint.xyz() / accumHomogPoint.w;
			}
		}


		/// CLOP attraction - initial iteration
		static void clopAttractionInit(const PointSet& particles, const Mixture& mixture, PointIndex& mixtureIndex, float h, const Params& params, vector<vec3>& outAttractionVecs)
		{
			const float var_k = h*h * 0.03125f;		// h²/32
			
			vector<uint> neighbors;
			for (uint i = 0; i < particles.size(); ++i) 
			{
				// query neighbors
				const vec3& q = particles[i];
				mixtureIndex.radiusSearch(q, h, neighbors);

				// accumulate
				vec4 accumHomogPoint(0, 0, 0, 0);
				for (uint nidx : neighbors)
				{
					const Gaussian& g = mixture[nidx];
					//------------------------------------------------------

					// transform: centralize the point q around the Gaussian
					vec3 c = g.mu - q;
										
					// Lambda = Gaussian cov + isotropic kernel cov
					smat3 Lambda = g.cov + smat3(var_k, 0, 0, var_k, 0, var_k);

					// compute inverse lambda
					smat3 Lambda_inv = inverse(Lambda);
					float invDetLambda = det(Lambda_inv);		// TODO: optimize - det is contained in inverse function

					vec3 LambdaInv_c = Lambda_inv * c;

					// weight Term			
					float wTerm = g.weight * sqrtf(invDetLambda) * expf(-0.5f * dot(c, LambdaInv_c));

					// Setup summand fraction
					vec3 xW = (wTerm * var_k) * LambdaInv_c;
					float W = wTerm;

					// weight target point with the component potential and accumulate.
					accumHomogPoint += vec4(xW, W);
				}

				// normalize
				outAttractionVecs[i] = accumHomogPoint.w == 0 ? vec3(0, 0, 0) : accumHomogPoint.xyz() / accumHomogPoint.w;
			}
		}


		/// LOP repulsion between particles
		static void computeRepulsion(const PointSet& particles, float h, const Params& params, vector<vec3>& outRepulsionVecs)
		{
			const float kernelRadius = h * params.repulsionRadiusFac;
			const float minus_16_over_h2 = -16.0f / (h*h);

			// create index of particles
			PointIndex particleIndex(particles, kernelRadius);

			// compute repulsion per particle
			vector<uint> neighbors;
			for (uint i = 0; i < particles.size(); ++i)
			{
				// query neighbors
				const vec3& q = particles[i];
				particleIndex.radiusSearch(q, h, neighbors);
				
				// accumulate
				vec4 accumHomogeneousVec(0, 0, 0, 0);
				for (uint nidx : neighbors)
				{
					const vec3& p = particles[nidx];
					vec3 diff = q - p;

					float squaredDistance = dot(diff, diff);
					if (squaredDistance == 0.0f)	// important!
						continue;

					float beta = 1.0f / sqrtf(squaredDistance);				// inverse distance 1 / dist 
					beta *= expf(minus_16_over_h2 * squaredDistance);		// weight kernel theta of distance

					// multiply with derivative of eta of distance
					if (params.useSoftEta)	beta *= 1.0f;
					else					beta /= (squaredDistance * squaredDistance);

					// accumulate
					accumHomogeneousVec += vec4(diff, 1.0f) * beta;
				}

				// normalize
				outRepulsionVecs[i] = accumHomogeneousVec.w == 0 ? vec3(0, 0, 0) : accumHomogeneousVec.xyz() / accumHomogeneousVec.w;
			}
		}


		/// WLOP attraction
		static void wlopAttraction(uint iter, const PointSet& particles, const Mixture& mixture, PointIndex& mixtureIndex, float h, const Params& params, vector<vec3>& outAttractionVecs)
		{
			const float epsilon = 0.0001f;
			const float minus_16_over_h2 = -16.0f / (h*h);
			

			vector<uint> neighbors;
			for (uint i = 0; i < particles.size(); ++i)
			{
				// query neighbors
				const vec3& q = particles[i];
				mixtureIndex.radiusSearch(q, h, neighbors);

				// accumulate
				vec4 accumHomogPoint(0, 0, 0, 0);
				for (uint nidx : neighbors)
				{
					const Gaussian& g = mixture[nidx];

					vec3 diff = g.mu - q;
					float squaredDistance = dot(diff, diff);

					float alpha = expf(minus_16_over_h2 * squaredDistance);

					if (iter > 0)
						alpha /= (epsilon + sqrtf(squaredDistance));		// from the second iteration on, use 1/|x-q| term

					// Multiply by WLOP density term: 1/v, encoded in the Gaussian potential
					alpha *= g.weight;

					// weight target point with the component potential and accumulate
					accumHomogPoint += vec4(diff, 1.0f) * alpha;
				}

				// normalize
				outAttractionVecs[i] = accumHomogPoint.w == 0 ? vec3(0, 0, 0) : accumHomogPoint.xyz() / accumHomogPoint.w;
			}
		}


	};	/// end class clop


}	/// end namespace cp



