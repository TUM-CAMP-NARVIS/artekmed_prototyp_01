
#include "artekmed/PointProcessing/Denoising.h"

namespace artekmed
{
	namespace pointcloud
	{
		Eigen::Vector3f projectPointToPlane(
			const Eigen::Vector3f &toProject,
			const Eigen::Vector3f &planeNormal,
			const Eigen::Vector3f &planePoint)
		{
			return toProject - ((toProject - planePoint).dot(planeNormal) * planeNormal);
		}


		struct PlaneFunctionResult{
			float value;
			Eigen::Vector2f gradient;
			Eigen::Matrix2f hessian;

			PlaneFunctionResult() :
				value(0),
				gradient(Eigen::Vector2f::Zero()),
				hessian(Eigen::Matrix2f::Zero())
			{ }

			Eigen::Vector2f calculateCurvature() const
			{
				auto solver = Eigen::EigenSolver<Eigen::Matrix2f>(hessian);
				return {std::abs(solver.eigenvalues()(0).real()),std::abs(solver.eigenvalues()(1).real())};
			}
		};

		//Returns
		// [z_Height at this point,
		// partialDerivativeValue df/du of z_Height,
		// partial derivative value at df/dv of z_Height]
		PlaneFunctionResult evalPlaneFunction(
			const uint8_t degree,
			const Eigen::VectorXf &coefficients,
			const Eigen::Vector3f &basePlaneNormal,
			const Eigen::Vector3f &basePlaneOrigin,
			const float u,
			const float v)
		{
			int i = 0;
			PlaneFunctionResult r={};
			//Precalculate the powers
			Eigen::VectorXf uPows(degree + 1);
			Eigen::VectorXf vPows(degree + 1);
			uPows(0) = vPows(0) = 1;
			for (int k = 1; k < degree + 1; ++k)
			{
				uPows(1) = uPows(1) * u;
				vPows(1) = vPows(2) * v;
			}
			//Evaluate f, df/du and df/dv
			for (int ui = 0; ui <= degree; ++ui)
			{
				for (int vi = 0; vi <= degree - ui; ++vi)
				{
					r.value += uPows(ui) * vPows(vi) * coefficients(i);
					if (ui >= 1)
					{
						r.gradient.x() += uPows(ui - 1) * ui *vPows(vi) * coefficients(i);
					}
					if (vi >= 1)
					{
						r.gradient.y() += uPows(ui) *vi* vPows(vi - 1) * coefficients(i);
						if(ui>=1)
						{
							r.hessian(0,1)=r.hessian(1,0) += uPows(ui-1)*ui*vPows(vi-1)*vi*coefficients(i);
						}
					}
					if(ui>=2)
					{
						r.hessian(0,0) += uPows(ui-2)*ui*(ui-1)*vPows(vi)*coefficients(i);
					}
					if(vi>=2)
					{
						r.hessian(1,1) += vPows(vi-2)*vi*(vi-1)*uPows(ui)*coefficients(i);
					}
					i++;
				}
			}
			return r;
		}

		float distanceToPlaneFunction(
			const Eigen::Vector3f &point,
			const uint8_t degree,
			const Eigen::VectorXf &coefficients,
			const Eigen::Vector3f &basePlaneNormal,
			const Eigen::Vector3f &basePlaneUAxis,
			const Eigen::Vector3f &basePlaneVAxis,
			const Eigen::Vector3f &basePlanePoint)
		{
			//Point in plane coordinates
			const float point_u = (point - basePlanePoint).dot(basePlaneUAxis);
			const float point_v = (point - basePlanePoint).dot(basePlaneVAxis);
			const float plane_z = (point - basePlanePoint).dot(basePlaneNormal); // Distance of Point from the base Plane
			float z = 0.f;  // Distance of point from the basePlane to the parametric surface

			uint32_t j = 0;
			float powU=1;
			for (int ui = 0; ui <= degree; ++ui)
			{
				float powV=1;
				for (int vi = 0; vi <= degree - ui; ++vi)
				{
					z += powU * powV * coefficients(j++);
					powV *=point_v;
				}
				powU*=point_u;
			}
			return std::abs(plane_z - z);
		}

		void mlsNormalEstimationAndSmoothing(
			const std::vector<Eigen::Vector3f> &neighbourPoints,
			Eigen::Vector3f &point, // point we want to smooth
			Eigen::Vector3f &normal, //approximate input normal n
			const Eigen::Vector3f &centroid,
			float &outCurvatureFactor,
			const uint8_t degree /*= 2*/,
			const float h /*= 0.3f*/)
		{
			auto theta = [&h](const Eigen::Vector3f & point, const Eigen::Vector3f & origin) {
				return std::exp(-((point-origin).squaredNorm()) / (h * h));
			};
			auto q = projectPointToPlane(point, normal, centroid);
			Eigen::Vector3f u_Axis = normal.unitOrthogonal();
			Eigen::Vector3f v_Axis = normal.cross(u_Axis);
			float q_u = q.dot(u_Axis);
			float q_v = q.dot(v_Axis);


			const auto nr_coefficients = (degree + 1) * (degree + 2) / 2;
			if (nr_coefficients > neighbourPoints.size())
			{
				//Would result in an underdetermined System
				return;
			}

			Eigen::MatrixXf systemMatrix(neighbourPoints.size(), nr_coefficients);
			Eigen::VectorXf rightSide(neighbourPoints.size());

			Eigen::VectorXf theta_vec(neighbourPoints.size()); //Theta are oure weights for eiach point distance

			for (size_t i = 0; i < neighbourPoints.size(); ++i)
			{
				auto diffVec = neighbourPoints[i] - q;
				//Calculate UV Coordinates to the current Plane
				const float uProjected = diffVec.dot(u_Axis);
				const float vProjected = diffVec.dot(v_Axis);
				theta_vec(i) = theta(neighbourPoints[i],q);
				rightSide(i) = diffVec.dot(normal);

				//Get Polynomial Terms at current position
				int j = 0;
				float u_pow = 1;
				for (int ui = 0; ui <= degree; ++ui)
				{
					float v_pow = 1;
					for (int vi = 0; vi <= degree - ui; ++vi)
					{
						systemMatrix(i, j++) = u_pow * v_pow;
						v_pow *= vProjected;
					}
					u_pow *= uProjected;
				}
			}
			const Eigen::MatrixXf weightedSystemMatrix = systemMatrix.transpose() * theta_vec.asDiagonal();
			rightSide = weightedSystemMatrix * rightSide;
			(weightedSystemMatrix * systemMatrix).llt().solveInPlace(rightSide);
			//right Side now holds all polynomial coefficients of our parametric plane in the form
			// [u^0,v^0],[u^0,v^1],...,[u^0,v^degree],[u^1,v_0],....[u^n,v^(i-n)],...

			const auto evalResult = evalPlaneFunction(degree, rightSide, normal, centroid, q_u, q_v);
			point += evalResult.value * normal;
			Eigen::Vector3f newNormal = normal;
			newNormal -= (evalResult.gradient.x() * u_Axis + evalResult.gradient.y()* v_Axis);
			newNormal.normalize();
			normal = newNormal;

			//We take the magnitude of the second order derivative as to give the resampling a hint on more/less samples
			outCurvatureFactor = evalResult.calculateCurvature().norm();
		}

		void pcaNormalEstimation(
			const std::vector<Eigen::Vector3f> &neighbours,
			Eigen::Vector3f &outCentroid,
			Eigen::Vector3f &outNormal,
			float &sigma
		)
		{
			Eigen::MatrixXf observations = {neighbours.size(), 3};
			outCentroid = {0, 0, 0};
			for (int i = 0; i < observations.rows(); ++i)
			{
				observations.row(i) = neighbours[i];
				outCentroid += neighbours[i];
			}
			auto centered = observations.rowwise() - observations.colwise().mean();
			outCentroid /= observations.rows();
			auto covarianceMatrix = (centered.adjoint() * centered) / float(observations.rows() - 1);
			auto solver = Eigen::EigenSolver<Eigen::MatrixXf>(covarianceMatrix);
			//Our PCA Eigenvalues
			auto lambda_0 = solver.eigenvalues()(0).real();
			auto lambda_1 = solver.eigenvalues()(1).real();
			auto lambda_2 = solver.eigenvalues()(2).real();
			//Our PCA Eigenvectors
			Eigen::Vector3f v_0 = solver.eigenvectors().col(0).real().cast<float>();
			Eigen::Vector3f v_1 = solver.eigenvectors().col(1).real().cast<float>();
			Eigen::Vector3f v_2 = solver.eigenvectors().col(2).real().cast<float>();
			//The Eigenvalues, eigenvectors are not sorted: sort them here from highest (v_0) to lowest(v_2)
			if (lambda_0 < lambda_1)
			{
				std::swap(lambda_0, lambda_1);
				std::swap(v_0, v_1);
			}
			if (lambda_1 < lambda_2)
			{
				std::swap(lambda_1, lambda_2);
				std::swap(v_1, v_2);
			}
			if (lambda_0 < lambda_1)
			{
				std::swap(lambda_0, lambda_1);
				std::swap(v_0, v_1);
			}
			//Sigma is a measure of quality for this normal estimation. The smaller sigma, the better the quality
			sigma = lambda_2 / (lambda_0 + lambda_1 + lambda_2);
			outNormal = v_2.normalized();
		}
	}
}