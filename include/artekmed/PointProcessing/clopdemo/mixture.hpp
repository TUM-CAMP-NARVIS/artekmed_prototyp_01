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

#include "gaussian.hpp"
#include "pointindex.hpp"
#include "pointset.hpp"

using namespace std;






namespace cp
{
	struct Component : Gaussian
	{
		Component() {}
		Component(const Gaussian& g, const vec3& nvar = vec3(0, 0, 0)) : Gaussian(g), nvar(nvar), is_parent(false) {}

		/// models the distribution of the Gaussian's normals using a spherical Gaussian
		/// direction: mean normal. length: variance of spherical Gaussian
		vec3 nvar;	

		/// used for HEM clustering
		bool is_parent;
	};


	/// Mixture of Gaussians
	class Mixture : public vector<Component>
	{
	public:
		struct Params
		{

			float	globalInitRadius = 1.0f;		// global initialization kernel radius (applies only if useGlobalInitRadius == true)
			bool	useGlobalInitRadius = true;		// use global initialization radius instead of NNDist sampling (more stable for non-uniform point sampling)
			uint	nNNDistSamples = 10;			// number of sample points for automatic initialization radius estimation (more is better)
			bool	useWeightedPotentials = true;	// if true, performs WLOP-like balancing of the initial Gaussian potentials
			float	alpha0 = 2.5f;					// multiple of nearest neighbor distance to use for initial query radius (<= 2.5f recommended)
			float	alpha = 2.5f;					// multiple of cluster maximum std deviation to use for query radius (<= 2.5f recommended)
			uint	nLevels = 5;					// number of levels to use for hierarchical clustering
			float	hemReductionFactor = 3.0f;		// factor by which to reduce the mixture each level (3 is recommended, don't change!)
		};

	public:
		/// Default Constructor
		Mixture() {}

		/// Constructor. Takes a set of 3D points and computes splats using HAS
		Mixture(const PointSet* points, const Params& params)
		{
			// 1. initialize mixture M
			cout << "initializing" << endl;
			initMixture(*points, params);
			
			// 2. hierarchical clustering
			Mixture M_next;
			for (uint l = 0; l < params.nLevels; ++l)
			{
				cout << "clustering level " << l << endl;
				clusterLevel(M_next, params);
				swap(M_next);
			}
		}

		// assignment operator
		Mixture& operator= (const Mixture& rhs)
		{
			if (&rhs != this)
			{
				// create per-component copies
				for (Component c : rhs)
					push_back(c);
			}
			return *this;
		}



	protected:
		void sampleNNDistances(const vector<vec3>& points, uint nSamples, float* minDist, float* maxDist, float* avgDist)
		{
			nSamples = max(nSamples, 1u);

			// select sample points
			vector<vec3> samplePoints;
			for (uint i = 0; i < nSamples; ++i)
				samplePoints.push_back(points[rand() % points.size()]);

			// iterate over points and find the squared nearest neighbor distances for all of them
			vector<float> nnSqDistances(nSamples, FLT_MAX);
			for (uint j = 0; j < points.size(); ++j)
			{
				const vec3& p = points[j];
				for (uint i = 0; i < nSamples; ++i)
				{
					float sqDist = squaredDist(samplePoints[i], p);
					if (sqDist != 0.0f && sqDist < nnSqDistances[i])
						nnSqDistances[i] = sqDist;
				}
			}

			// compute minimum, maximum and average sample nnDist 
			*minDist = FLT_MAX;
			*maxDist = 0;
			*avgDist = 0;
			for (uint i = 0; i < nSamples; ++i)
			{
				float d = sqrtf(nnSqDistances[i]);
				*avgDist += d;
				if (d > *maxDist) *maxDist = d;
				if (d < *minDist) *minDist = d;
			}
			*avgDist /= nSamples;
		}



		void initMixture(const vector<vec3>& points, const Params& params)
		{
			uint nPoints = points.size();
			const float parentProbability = 1.0f / params.hemReductionFactor;
			vector<uint> nnIndices;

			PointIndex index;

			vector<float> radii(points.size());
			if (params.useGlobalInitRadius)
			{
				// create the point index with the globalInitRadius as cell size and fill the nearest neighbor radii vector
				index.create(points, params.globalInitRadius);
				for (uint i = 0; i < points.size(); ++i)
					radii[i] = params.globalInitRadius;
			}
			else
			{
				// sample nearest neighor distance statistics			
				float minNNDist, maxNNDist, avgNNDist;
				sampleNNDistances(points, params.nNNDistSamples, &minNNDist, &maxNNDist, &avgNNDist);

				// create the point index with a cell size based on the neighbor distance statistics and compute nearest neighbor radii
				index.create(points, 2.0f * maxNNDist - avgNNDist);
				for (uint i = 0; i < points.size(); ++i)
				{
					index.annSearch(points[i], 2, nnIndices);
					float nnDist = nnIndices.size() > 1 ? dist(points[i], points[nnIndices[1]]) : index.cellSize();

					if (nnDist == 0)	// can happen if the nearest neigbhor has the same coordinates
						nnDist = index.cellSize() * 0.1f;

					radii[i] = nnDist * params.alpha0;
				}
			}


			// Compute initial mixture - CLOP-like HEM
			resize(points.size());
			for (uint i = 0; i < points.size(); ++i)
			{
				const vec3& q = points[i];
				Component& c = at(i);
				

				// query neighbors and compute cov
				float r = radii[i];
				index.radiusSearch(q, r, nnIndices);
			
				const float minus_16_over_h2 = -16.0f / (r * r);
				
				float eps = r * r * 0.0001f;
				smat3 sumcov = smat3(eps, 0, 0, eps, 0, eps);	// initial bias of the cluster for stability
				vec3 mean(0, 0, 0);
				float density = 0.000001f;
				for (uint j_ = 0; j_ < nnIndices.size(); ++j_)
				{
					const vec3& p = points[nnIndices[j_]];
					vec3 diff = p - q;
					sumcov += smat3::outer(diff);					// centralized in parent pos for numerical stability
					mean += p;
					density += expf(minus_16_over_h2 * dot(diff, diff)); 	// accumulate the WLOP density sum
				}

				// setup component
				float inv_w = 1.0f / nnIndices.size();		// size > 0 ensured
				c.mu = q;
				c.cov = sumcov * inv_w - smat3::outer(mean * inv_w - q);	// consider parent pos centralization
				c.cov = conditionCov(c.cov);
				c.weight = params.useWeightedPotentials ? 1.0f / density : 1.0f;
				
				// Compute the initial normal and set initial normal variance of this point cluster
				// the normal variance is encoded in the length of the normal vector
				vec3 evectors[3];
				c.cov.eigenvectors(evectors);
				
				// for the initial normal variance, 0.0f should theoretically also work, but since we are encoding the var 
				// in the length of the normal (to save a channel), thats not possible in our implementation
				float initialVar = 0.001f;
				c.nvar = evectors[0] * initialVar;


				// randomly set parent flag
				c.is_parent = rand01() < parentProbability;
			}


#if CLOP_DEBUG
			for (uint i = 0; i < size(); ++i)
			{
				const vec3& mu = at(i).mu;
				const smat3& cov = at(i).cov;
				if (isnan(mu) || det(cov) <= 0 || isnan(det(cov)))
					cerr << "[initMixture] Error @ initial component " << i << ": mu = " << mu << ", cov = " << cov << ", det = " << det(cov) << endl;
			}
#endif
		}


		float hemLikelihood(const Gaussian& parent, const Gaussian& child)
		{
			vec3 mu_diff = parent.mu - child.mu;
			smat3 pCovInv = inverse(parent.cov);

			float smd = dot(mu_diff, pCovInv * mu_diff);
			smat3 ipcCov = pCovInv * child.cov;
			float ipcTrace = ipcCov.e00 + ipcCov.e11 + ipcCov.e22;

			// Gaussian exponent and the trace exponent
			float e = -0.5f * (smd + ipcTrace);
			// 1/((2pi)^(3/2))   *   sqrt(|Sigma1|)^-1 = sqrt(|Sigma1^-1|)   *   e^exponent   
			float f = 0.063493635934241f * sqrtf(det(pCovInv)) * expf(e);
			// raise to the power of the number of points in the child cluster
			return powf(f, child.weight);
		}


		void clusterLevel(vector<Component>& newComponents, const Params& params)
		{
			uint nComponents = size();

			// 1. iterate over components, prepare index point centers and compute the parent's individual and maximum query radius
			vector<vec3> centers(nComponents);
			vector<uint> parentIndices;
			vector<float> queryRadii;
			float maxQueryRadius = 0;
			for (uint i = 0; i < nComponents; ++i)
			{
				const Component& c = at(i);

				// prepare centers to be indexed
				centers[i] = c.mu;

				// select parents
				if (c.is_parent)
				{
					parentIndices.push_back(i);

					// ii. get the conservative query radius for this parent
					float queryRadius = params.alpha * sqrtf(c.cov.eigenvalues().z);
					queryRadii.push_back(queryRadius);

					// ii. determine maximum query radius
					if (queryRadius > maxQueryRadius)
						maxQueryRadius = queryRadius;
				}
			}


			// 2. create point index of component centers for neighbor queries
			PointIndex index(centers, maxQueryRadius);


			// 3. select child set for each parent
			uint nParents = parentIndices.size();
			vector<vector<uint>> childIndices(nParents);
			for (uint s_ = 0; s_ < nParents; ++s_)
			{
				uint s = parentIndices[s_];
				const Gaussian& parent = at(s);
				
				vector<uint> resultSet;
				index.radiusSearch(parent.mu, queryRadii[s_], resultSet);

				// select eligible children from the conservative resultSet
				for (uint i_ = 0; i_ < resultSet.size(); ++i_)
				{
					uint i = resultSet[i_];
					const Component& child = at(i);


					float kld = KLD(child, Gaussian(parent.mu, parent.cov));

					if (kld > params.alpha * params.alpha * 0.5f)
						//if ( squaredMahalanobisDist( child.Âmu - parent.Âmu, parent.cov ) > params.alpha*params.alpha )
						//if (!child.delta && squaredMahalanobisDist( child.Âmu - parent.Âmu, parent.cov ) > params.alpha * params.alpha)
						continue;

					// the only parent allowed to merge is the parent s itself
					if (child.is_parent && s != i)
						continue;

					childIndices[s_].push_back(i);
				}
			}


			// 4. compute the wL_is and the wL sums
			vector<vector<float>> wL_cache(nComponents);
			vector<float> sumLw(nComponents, 0);
			for (uint s_ = 0; s_ < nParents; ++s_)
			{
				uint s = parentIndices[s_];
				const Component& parent = at(s);

				// iterate over children 
				const vector<uint>& I = childIndices[s_];
				wL_cache[s_].resize(I.size(), 0.0f);
				for (uint i_ = 0; i_ < I.size(); ++i_)
				{
					uint i = I[i_];

					const float maxL = 1e8f;
					const float minL = FLT_MIN;
					float wL_si = parent.weight * clamp(hemLikelihood(parent, at(i)), minL, maxL);

					// save likelihood contribution
					wL_cache[s_][i_] = wL_si;
					sumLw[i] += wL_si;
				}
			}


			// 5. compute responsibilities and update
			newComponents.clear();
			for (uint s_ = 0; s_ < nParents; ++s_)
			{
				uint s = parentIndices[s_];
				const Component& parent = at(s);
				const vector<uint>& I = childIndices[s_];

				// initialize parent info
				float w_s = 0.0f;
				vec3 summu_i(0, 0, 0);
				smat3 sumcov_i(0, 0, 0, 0, 0, 0);
				vec3 resultant(0, 0, 0);
				float nvar = 0.0f;

				// iterate over children and accumulate
				for (uint i_ = 0; i_ < I.size(); ++i_)
				{
					uint i = I[i_];

					if (sumLw[i] == 0.0f)	// can happen
						continue;

					const Component& child = at(i);

					// compute responsibility of parent s for child i
					float r_is = wL_cache[s_][i_] / sumLw[i];
					float w = r_is * child.weight;
					
					// normal cluster update
					float c_nvar = length(child.nvar);
					vec3 c_normal = child.nvar / c_nvar;

					// flip child normal to be next to the parent normal
					// note that p_normalVar is unnormalized, but thats ok, we're just doing dot-product
					if (dot(c_normal, parent.nvar) < 0.0f)
						c_normal = -c_normal;
					
					// accumulate
					w_s += w;
					summu_i += w * child.mu;
					sumcov_i += w * (child.cov + smat3::outer(child.mu - parent.mu));	// accumulates generic cov relative to parent mu, numerically more stable than origin, due to smaller distances
					resultant += w * c_normal;
					nvar += w * c_nvar;
				}

				// normalize and condition new cov matrix
				float inv_w = 1.0f / w_s;		// w_s > 0 is certain
				vec3 mu_s = inv_w * summu_i;
				smat3 cov_s = inv_w * sumcov_i - smat3::outer(mu_s - parent.mu);
				cov_s = conditionCov(cov_s);

				// mixture of normals
				float variance1 = nvar * inv_w;			// normalized sum of the variances of the child clusters
				float R = length(resultant);			// resultant length
				float Rmean = R * inv_w;				// mean resultant length
				float variance2 = -2.0f * log(Rmean);	// variance of the child clusters' mean normal vectors with respect to the common mean (outer product thingy)
				vec3  newMeanNormal = resultant / R;	// normalized mean normal vector of new cluster
				

				// Add new component to output list
				Component newComponent;
				newComponent.mu = mu_s;
				newComponent.cov = cov_s;
				newComponent.weight = w_s;
				newComponent.nvar = newMeanNormal * (variance1 + variance2);

				newComponents.push_back(newComponent);
			}


			// 6. add orphans, components not addressed by any parent (zero sumLw)
			for (uint i = 0; i < nComponents; ++i)
			if (sumLw[i] == 0.0f)
				newComponents.push_back(at(i));


			// 7. distribute new parent flags to the new component set
			const float parentProbability = 1.0f / params.hemReductionFactor;
			for (uint i = 0; i < newComponents.size(); ++i)
				newComponents[i].is_parent = rand01() < parentProbability;
						
#if CLOP_DEBUG
			for (uint i = 0; i < newComponents.size(); ++i)
			{
				const vec3& mu = newComponents[i].mu;
				const smat3& cov = newComponents[i].cov;
				if (isnan(mu) || det(cov) <= 0 || isnan(det(cov)))
					cerr << "[clusterLevel] Error @ new component " << i << ": mu = " << mu << ", cov = " << cov << ", det = " << det(cov) << endl;
			}
#endif
		}


	};	/// end class Mixture


}	/// end namespace cp



