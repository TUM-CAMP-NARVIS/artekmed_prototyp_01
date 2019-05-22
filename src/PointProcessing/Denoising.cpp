
#include "artekmed/PointProcessing/Denoising.h"

#include <iostream>
#include <Eigen/Dense>

namespace artekmed
{
	namespace pointcloud
	{
		Eigen::Vector3f projectPointTo2DPlane(
			const Eigen::Vector3f &toProject,
			const Eigen::Vector3f &planeNormal,
			const Eigen::Vector3f &planePoint)
		{
			return toProject - ((toProject - planePoint).dot(planeNormal) * planeNormal);
		}


		struct PlaneFunctionResult
		{
			float value;
			Eigen::Vector2f gradient;
			Eigen::Matrix2f hessian;

			PlaneFunctionResult() :
				value(0),
				gradient(Eigen::Vector2f::Zero()),
				hessian(Eigen::Matrix2f::Zero())
			{}

			Eigen::Vector2f calculateHessianEV() const
			{
				auto solver = Eigen::EigenSolver<Eigen::Matrix2f>(hessian);
				return {solver.eigenvalues()(0).real(), solver.eigenvalues()(1).real()};
			}

			//Function from PCL, does not really work with our stuff
			Eigen::Vector2f calculatePrincipalCurvature() const
			{
				/*
				const double Z = 1 + d.z_u * d.z_u + d.z_v * d.z_v;
				const double Zlen = std::sqrt (Z);
				const double K = (d.z_uu * d.z_vv - d.z_uv * d.z_uv) / (Z * Z);
				const double H = ((1.0 + d.z_v * d.z_v) * d.z_uu - 2.0 * d.z_u * d.z_v * d.z_uv +
					(1.0 + d.z_u * d.z_u) * d.z_vv) / (2.0 * Zlen * Zlen * Zlen);
				const double disc2 = H * H - K;
				assert (disc2 >= 0.0);
				const double disc = std::sqrt (disc2);
				k[0] = H + disc;
				k[1] = H - disc;
*/
				const auto Z = 1 + gradient.squaredNorm();
				const auto K = (hessian(0, 0) * hessian(1, 1) - hessian(0, 1) * hessian(1, 0)) / (Z * Z);
				const auto H = (1 + gradient(1) * gradient(1)) * hessian(0, 0) -
				               2 * gradient(0) * gradient(1) * hessian(0, 1) + (1 + gradient(0) * gradient(1) * hessian(1, 1))
				                                                               / (2 * Z * std::sqrt(Z));
				const auto disc2 = H * H - K;
				const auto disc = std::sqrt(disc2);
				return {H + disc, H - disc};
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
			PlaneFunctionResult r = {};
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
						r.gradient.x() += uPows(ui - 1) * ui * vPows(vi) * coefficients(i);
					}
					if (vi >= 1)
					{
						r.gradient.y() += uPows(ui) * vi * vPows(vi - 1) * coefficients(i);
						if (ui >= 1)
						{
							r.hessian(0, 1) = r.hessian(1, 0) += uPows(ui - 1) * ui * vPows(vi - 1) * vi * coefficients(i);
						}
					}
					if (ui >= 2)
					{
						r.hessian(0, 0) += uPows(ui - 2) * ui * (ui - 1) * vPows(vi) * coefficients(i);
					}
					if (vi >= 2)
					{
						r.hessian(1, 1) += vPows(vi - 2) * vi * (vi - 1) * uPows(ui) * coefficients(i);
					}
					i++;
				}
			}
			return r;
		}

		struct ProjectionResult
		{
			Eigen::Vector3f point;
			Eigen::Vector3f normal;
			PlaneFunctionResult planeFunctionValues;

		};
		//The function below tanked so hard in performance that i had to make this one
		ProjectionResult projectPointToPlaneSimple(
			const Eigen::Vector3f &point,
			const uint8_t degree,
			const Eigen::VectorXf &coefficients,
			const Eigen::Vector3f &basePlaneNormal,
			const Eigen::Vector3f &basePlaneUAxis,
			const Eigen::Vector3f &basePlaneVAxis,
			const Eigen::Vector3f &basePlanePoint)
		{
			const Eigen::Vector3f diff = point-basePlanePoint;
			const float u = basePlaneUAxis.dot(diff);
			const float v = basePlaneVAxis.dot(diff);

			const auto result = evalPlaneFunction(degree,coefficients,basePlaneNormal,basePlanePoint,u,v);
			ProjectionResult r;
			r.point = point -result.value*basePlaneNormal;
			r.normal = (basePlaneNormal - result.gradient(0)*basePlaneUAxis-result.gradient(1)*basePlaneVAxis).normalized();
			r.planeFunctionValues = std::move(result);
			return r;
		}

		// projectPointToPlaneOrthagonal from PCL, just project the point to the base plane and then eval-ing the function
		// does not yield the closest point to the polynom, not very dramatic for the points themselves, but critical for
		// normals (first order derivatives) and second order derivatives, extreme performance tank
		ProjectionResult projectPointToPlane(
			const Eigen::Vector3f &point,
			const uint8_t degree,
			const Eigen::VectorXf &coefficients,
			const Eigen::Vector3f &basePlaneNormal,
			const Eigen::Vector3f &basePlaneUAxis,
			const Eigen::Vector3f &basePlaneVAxis,
			const Eigen::Vector3f &basePlanePoint)
		{

			ProjectionResult result;
			result.point = point;
			result.normal = basePlaneNormal;

			//Point in plane coordinates [u,v,w(offset)]
			const Eigen::Vector3f basePlaneCoordinates = {
				(point - basePlanePoint).dot(basePlaneUAxis),
				(point - basePlanePoint).dot(basePlaneVAxis),
				(point - basePlanePoint).dot(basePlaneNormal)};

			Eigen::Vector3f optimizedPlaneCoordinates = basePlaneCoordinates;

			float z = 0.f;  // Distance of point from the basePlane to the parametric surface
			auto d = evalPlaneFunction(degree, coefficients, basePlaneNormal, basePlanePoint,
			                           basePlaneCoordinates.x(), basePlaneCoordinates.y());
			optimizedPlaneCoordinates.z() = d.value;
			const float originalZ = d.value;
			const float origDist = std::abs(d.value - basePlaneCoordinates.z());
			float newDist;
			float error = .0f;
			constexpr uint32_t maxIterations = 30;
			uint32_t i = 0;
			do
			{
				Eigen::Vector2f errorVec = optimizedPlaneCoordinates.head<2>() - basePlaneCoordinates.head<2>();
				errorVec += d.gradient * (optimizedPlaneCoordinates.z() - basePlaneCoordinates.z());

				Eigen::Matrix2f J = d.hessian * (optimizedPlaneCoordinates.z() - basePlaneCoordinates.z());
				J += Eigen::Matrix2f::Identity();
				J(0, 0) += d.gradient(0) * d.gradient(0);
				J(0, 1) += d.gradient(0) * d.gradient(1);
				J(1, 0) += d.gradient(1) * d.gradient(0);
				J(1, 1) += d.gradient(1) * d.gradient(1);

				optimizedPlaneCoordinates.head<2>() -= J.inverse() * errorVec;
				d = evalPlaneFunction(degree, coefficients, basePlaneNormal, basePlanePoint, optimizedPlaneCoordinates.x(),
				                      optimizedPlaneCoordinates.y());
				optimizedPlaneCoordinates.z() = d.value;
				newDist = (optimizedPlaneCoordinates - basePlaneCoordinates).norm();
				error = errorVec.norm();
				//std::cout<< "Looperino\n";
				i++;
			} while (error > 0.00001f && newDist < origDist && i < maxIterations);
			if (newDist > origDist)
			{
				//The optimization was diverging, abort and just use plane
				optimizedPlaneCoordinates = basePlaneCoordinates;
				optimizedPlaneCoordinates.z() = originalZ;
			}
			result.normal -= d.gradient.x() * basePlaneUAxis + d.gradient.y() * basePlaneVAxis;
			result.normal.normalize();
			result.point = basePlanePoint + optimizedPlaneCoordinates.x() * basePlaneUAxis +
			               optimizedPlaneCoordinates.y() * basePlaneVAxis + optimizedPlaneCoordinates.z() * basePlaneNormal;
			result.planeFunctionValues = d;
			return result;
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
			auto theta = [&h](const Eigen::Vector3f &point, const Eigen::Vector3f &origin) {
				return std::exp(-((point - origin).squaredNorm()) / (h * h));
			};
			auto q = projectPointTo2DPlane(point, normal, centroid);
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
				theta_vec(i) = theta(neighbourPoints[i], q);
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
			const Eigen::VectorXf coefficients =
				(weightedSystemMatrix * systemMatrix).colPivHouseholderQr().solve(rightSide);
			//right Side now holds all polynomial coefficients of our parametric plane in the form
			// [u^0,v^0],[u^0,v^1],...,[u^0,v^degree],[u^1,v_0],....[u^n,v^(i-n)],...

			const auto result = projectPointToPlaneSimple(point,degree,coefficients,normal,u_Axis,v_Axis,centroid);
			point = result.point;
			normal = result.normal;
			//We take the magnitude of the second order derivative as to give the resampling a hint on more/less samples
			//Why the sqrt? - values are either extremely big or extremely small so with that we bring them more together
			outCurvatureFactor = std::sqrt(result.planeFunctionValues.calculateHessianEV().norm());
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