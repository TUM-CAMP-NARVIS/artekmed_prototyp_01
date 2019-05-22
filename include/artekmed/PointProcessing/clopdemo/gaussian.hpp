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
#include <iostream>

#include "vec.hpp"


using namespace std;



namespace cp
{
	
#pragma region covariance matrix operators

	/// (old) conditioning by diagonal biasing
	inline smat3 conditionCov2(const smat3& cov, vec3& outEvalues, float epsilon = 1e-10f, uint maxIters = 3)
	{
		smat3 newCov = cov;
		outEvalues = newCov.eigenvalues();

		// If we have negative eigenvalues, bias matrix trace
		for (uint i = 0; i < maxIters && outEvalues.x <= 0.0f; ++i)
		{
			float bias = (outEvalues.x == 0.0f) ? epsilon : 1.5f * fabsf(outEvalues.x);
			newCov += smat3(bias, 0, 0, bias, 0, bias);
			outEvalues = newCov.eigenvalues();
		}

		if (outEvalues.x <= 0.0f)
			cout << "Warning: cov still non-psd despite conditioning! det: " << det(newCov) << ", cov: " << newCov.toString() << endl;

		return newCov;
	}
	
	/// Conditioning by off diagonal dampening
	inline smat3 conditionCov(const smat3& cov, vec3& outEvalues, float epsilon = 1e-10f, uint maxIters = 3)
	{
		smat3 newCov;
		float abseps = fabsf(epsilon);

		// condition diagonal elements
		newCov.e00 = fmaxf(cov.e00, abseps);
		newCov.e11 = fmaxf(cov.e11, abseps);
		newCov.e22 = fmaxf(cov.e22, abseps);

		// condition off diagonal elements
		float sx = sqrtf(cov.e00);
		float sy = sqrtf(cov.e11);
		float sz = sqrtf(cov.e22);

		for (float rho = 0.99f; rho >= 0; rho -= 0.01f)
		{
			float rxy = rho * sx * sy;
			float rxz = rho * sx * sz;
			float ryz = rho * sy * sz;
			newCov.e01 = clamp(cov.e01, -rxy, rxy);
			newCov.e02 = clamp(cov.e02, -rxz, rxz);
			newCov.e12 = clamp(cov.e12, -ryz, ryz);

			// Check
			outEvalues = cov.eigenvalues();
			if (outEvalues.x > 0.0f)
				break;
		}

		// Check
		if (outEvalues.x <= 0.0f)
		{
			std::cout << "Warning: cov still non-psd despite conditioning! det: " << det(cov) << ", cov: " << cov.toString();
			std::cout << ", evalues: " << outEvalues << ", " << std::endl;
		}

		return newCov;
	}

	inline smat3 conditionCov(const smat3& cov, float epsilon = 1e-10f, uint maxIters = 3)
	{
		vec3 evd;
		return conditionCov(cov, evd, epsilon, maxIters);
	}
	

	/// squared Mahalanobis distance between two points X and Y, given the covariance matrix cov
	inline float SMD(const vec3& X, const vec3& Y, const smat3& cov)
	{
		return dot(X - Y, inverse(cov) * (X - Y));
	}

	/// Mahalanobis distance between two points X and Y, given the covariance matrix cov
	inline float MD(const vec3& X, const vec3& Y, const smat3& cov)
	{
		return sqrtf(SMD(X, Y, cov));
	}

	/// rotate covariance cov by rotation matrix R
	inline smat3 rotateCov(const smat3& cov, const mat3& R)
	{
		return R * cov.toMat3() * transpose(R);
	}
#pragma endregion


	/// Gaussian distribution with center mu, covariance cov and measure weight
	struct Gaussian
	{
		vec3 mu;			/// mean
		smat3 cov;		/// covariance
		float weight;	/// weight


		// Default Ctor
		Gaussian()
		{
		}

		// Ctor
		Gaussian(const vec3& mu, const smat3& cov, float weight = 1.0f) : mu(mu), cov(cov), weight(weight)
		{
		}

		static Gaussian zero () 
		{
			return Gaussian (vec3 (0, 0, 0), smat3::zero (), 0);
		}

		// returns the (normalized) probability density function at X, neglecting the weight of this Gaussian
		float pdf(const vec3& X) const
		{
			// TOOD: fuse det and inverse computation
			float d = det(cov);
			vec3 diff = X - mu;
			
			return expf(-0.5f * dot(diff, inverse(cov) * diff)) / sqrtf(248.0502134f * d);
		}


		// returns the product Gaussian resulting from multiplication of this with another weighted Gaussian (weights are not ignored). 
		// the resulting Gaussian has according mu, cov and weight. (MatrixCookbook)
		Gaussian operator* (const Gaussian& g) const
		{
			// TODO: Optimize (det and inverse)
			smat3 covSum = cov + g.cov;
			smat3 invCovSum = inverse(covSum);
			float preFac = 1.0f / sqrtf(248.05021f * det(covSum));  // 2pi ^ 3* det

			smat3 inv_cov0 = inverse(cov);
			smat3 inv_cov1 = inverse(g.cov);
			vec3 d = mu - g.mu;

			Gaussian product;
			product.weight = weight * g.weight * preFac * exp(-0.5f * dot(d, invCovSum * d));
			product.cov = inverse(inv_cov0 + inv_cov1);
			product.mu = product.cov * (inv_cov0 * mu + inv_cov1 * g.mu);

			return product;
		}
	};


#pragma region Gaussian operators

	// Kullback-Leibler distance between two Gaussians
	inline float KLD(const Gaussian& gc, const Gaussian& gp)
	{
		return 0.5f * (SMD(gc.mu, gp.mu, gp.cov) + trace(inverse(gp.cov) * gc.cov) - 3.0f - log(det(gc.cov) / det(gp.cov)));
	}


	// Bhattacharyya distance between two Gaussians
	inline float DBhatt(const Gaussian& g1, const Gaussian& g2)
	{
		return 0.25f * SMD(g1.mu, g2.mu, g1.cov + g2.cov) + 0.5f * logf(det(g1.cov + g2.cov)) - 0.25f * logf(det(g1.cov) * det(g2.cov)) - 1.03972077f;	// dim/2 * ln(2)
	}

	// applies the given linear transformationo T on the Gaussian. The weight is left unaffected.
	inline Gaussian transform(const Gaussian& g, const mat4& T)
	{
		Gaussian gT;

		// for the covariance transformation, only the rotational part is relevant
		mat3 rot = T.toMat3();
		gT.cov = rot * g.cov.toMat3() * transpose(rot);
		gT.mu = (T * vec4(g.mu, 1)).xyz();
		gT.weight = g.weight;

		return gT;
	}

#pragma endregion

}	/// end namespace cp

