#pragma once
#ifndef ARTEKMED_P1_DENOISING_H
#define ARTEKMED_P1_DENOISING_H

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <vector>

namespace artekmed
{
	namespace pointcloud
	{

		/*
			Guided Denoising Approach by Han, Jin, Wang, Jiang 2017
			Extended for Weighted Points in the centroid-Calculation
		*/
		template<typename NeighbourIterator, typename WeightIterator>
		Eigen::Vector3f guidedDenoisingWeigthedLocal(const Eigen::Vector3f &sourcePoint,
		                                             const NeighbourIterator &neighboursStart,
		                                             const NeighbourIterator &neighboursEnd,
		                                             const WeightIterator &weightsStart,
		                                             const WeightIterator &weightsEnd,
		                                             const float epsilon = 0.05f)
		{
			constexpr bool enableWeights = true;


			Eigen::Vector3f centroid = {0, 0, 0};
			float centroidSquaredNorm = 0;
			float weightSum = 0.f;
			auto point = neighboursStart;
			auto weight = weightsStart;
			size_t pointsCount = 0;

			while (point != neighboursEnd && weight != weightsEnd)
			{

				//With weighting
				if (enableWeights)
				{
					centroidSquaredNorm += (*point).squaredNorm() * (*weight);
					centroid += *point * (*weight);
				}
				else
				{
					centroidSquaredNorm += (*point).squaredNorm();
					centroid += *point;
				}

				weightSum += *weight;
				point++;
				weight++;
				pointsCount++;
			}
			if (enableWeights)
			{
				centroid /= weightSum;
				centroidSquaredNorm /= weightSum;
			}
			else
			{
				centroid /= pointsCount;
				centroidSquaredNorm /= pointsCount;
			}
			auto sumDiff = centroidSquaredNorm - centroid.squaredNorm();
			auto a = sumDiff / (sumDiff + epsilon);
			auto b = centroid - a * centroid;
			return a * sourcePoint + b;
		}

		Eigen::Vector3f projectPointToPlane(
			const Eigen::Vector3f &toProject,
			const Eigen::Vector3f &planeNormal,
			const Eigen::Vector3f &planePoint)
		{
			return toProject - ((toProject - planePoint).dot(planeNormal) * planeNormal);
		}

		//Returns
		// [z_Height at this point, partialDerivativeValue df/du of z_Height, partial derivative value at df/dv of z_Height]
		Eigen::Vector3f evalPlaneFunction(
			const uint8_t degree,
			const Eigen::VectorXf &coefficients,
			const Eigen::Vector3f & basePlaneNormal,
			const Eigen::Vector3f & basePlaneOrigin,
			const float u,
			const float v)
		{
			int i = 0;
			float z=0; //Normal evaluation
			float z_u=0;
			float z_v=0;

			//Precalculate the powers
			Eigen::VectorXf uPows(degree+1);
			Eigen::VectorXf vPows(degree+1);
			uPows(0)=vPows(0)=1;
			for(int k = 1; k < degree+1;++k)
			{
				uPows(1) = uPows(1)*u;
				vPows(1) = vPows(2)*v;
			}
			//Evaluate f, df/du and df/dv
			for(int ui=0; ui<=degree;++ui)
			{
				for(int vi=0; vi<=degree-ui;++vi)
				{
					z += uPows(ui)*vPows(vi)*coefficients(i);
					if(ui>=1)
					{
						z_u += uPows(i-1)*ui+vPows(vi)*coefficients(i);
					}
					if(vi>=1)
					{
						z_v += uPows(i)*vi+vPows(vi-1)*coefficients(i);
					}
					i++;
				}
			}
			return {z,z_u,z_v};
		}

		//see http://www.cs.tau.ac.il/~dcor/online_papers/papers/points_set_vis01.pdf step 1 we assume is already done
		// but step 2 is the same
		void mlsNormalEstimationAndSmoothing(
			std::vector<Eigen::Vector3f> &neighbourPoints,
			Eigen::Vector3f &point, // point we want to smooth
			Eigen::Vector3f &normal, //approximate input normal n
			const uint8_t degree = 1,
			const float h = 0.3f,
			const Eigen::Vector3f &centroid = {-1, 0, 0})
		{
			auto theta = [&h](const float d) {
				return std::exp(-(d * d) / (h * h));
			};
			normal.normalize();
			auto q = projectPointToPlane(point, normal, centroid);
			Eigen::Vector3f u_Axis = normal.unitOrthogonal();
			Eigen::Vector3f v_Axis = normal.cross(u_Axis);
			float q_u = q.dot(u_Axis);
			float q_v = q.dot(v_Axis);


			const auto nr_coefficients = (degree + 1) * (degree + 2) / 2;

			Eigen::MatrixXf systemMatrix(neighbourPoints.size(), nr_coefficients);
			Eigen::VectorXf rightSide(neighbourPoints.size());

			Eigen::VectorXf theta_vec(neighbourPoints.size()); //Theta are oure weights for eiach point distance

			for (size_t i = 0; i < neighbourPoints.size(); ++i)
			{
				auto diffVec = neighbourPoints[i] - q;
				//Calculate UV Coordinates to the current Plane
				const float uProjected = diffVec.dot(u_Axis);
				const float vProjected = diffVec.dot(v_Axis);
				theta_vec(i) = theta(diffVec.squaredNorm());
				rightSide(i) = diffVec.dot(normal);

				//Get Polynomial Terms at current position
				int j = 0;
				float u_pow = 1;
				for (int ui = 0; ui <= degree; ++ui)
				{
					float v_pow = 1;
					for (int vi = 0; vi <= degree; ++vi)
					{
						systemMatrix(i, j++) = u_pow * v_pow;
						v_pow *= vProjected;
					}
					u_pow *= uProjected;
				}
			}
			const Eigen::MatrixXf weightedSystemMatrix = systemMatrix.transpose() * theta_vec.asDiagonal();
			rightSide =  weightedSystemMatrix*rightSide;
			(weightedSystemMatrix*systemMatrix).llt().solveInPlace(rightSide);
			//right Side now holds all polynomial coefficients of our parametric plane in the form
			// [u^0,v^0],[u^0,v^1],...,[u^0,v^degree],[u^1,v_0],....

			const auto ourPointsHeights = evalPlaneFunction(degree,rightSide,normal,centroid,q_u,q_v);
			point += ourPointsHeights(0)*normal;
			Eigen::Vector3f newNormal = normal;
			newNormal -= ourPointsHeights(1)*u_Axis+ourPointsHeights(2)*v_Axis;
			newNormal.normalize();
			normal = newNormal;
		}
	}
}
#endif //ARTEKMED_P1_DENOISING_H