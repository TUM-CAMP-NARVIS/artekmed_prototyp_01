//
// Created by netlabs on 23.01.19.
//

#ifndef ARTEKMED_P1_EIGENWRAPPER_H
#define ARTEKMED_P1_EIGENWRAPPER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <utMath/Vector.h>
#include <utMath/Matrix.h>
#include <utMath/Quaternion.h>

#include <cmath> // std::sqrt, std::cos, std::sin

namespace Ubitrack { namespace Math {


        namespace Wrapper {


            /**
             * @brief Eigen / Ubitrack cast functor vector types
             */
            struct EigenVectorCast
            {
                template< typename T, int S  >
                Ubitrack::Math::Vector< T, S> from_eigen( const Eigen::Matrix<T, S, 1> & value )const
                {
                    Ubitrack::Math::Vector< T, S> result;
                    for (auto i = 0; i < S; i++) {
                        result(i) = value(i);
                    }
                    return result;
                }

                template< typename T, int S  >
                Eigen::Matrix<T, S, 1> to_eigen( const Ubitrack::Math::Vector< T, S>  & value )const
                {
                    Eigen::Matrix<T, S, 1> result;
                    for (auto i = 0; i < S; i++) {
                        result(i) = value(i);
                    }
                    return result;
                }
            };

            /**
             * @brief Eigen / Ubitrack cast functor vector types
             */
            struct EigenMatrixCast
            {
                template< typename T, int N, int M  >
                Ubitrack::Math::Matrix< T, N, M> from_eigen( const Eigen::Matrix<T, N, M> & value )const
                {
                    Ubitrack::Math::Matrix< T, N, M> result;
                    for (auto i = 0; i < N; i++) {
                        for(auto j = 0; j < M; j++) {
                            result(i,j) = value(i,j);
                        }
                    }
                    return result;
                }

                template< typename T, int N, int M >
                Eigen::Matrix<T, N, M> to_eigen( const Ubitrack::Math::Matrix< T, N, M>  & value )const
                {
                    Eigen::Matrix<T, N, M> result;
                    for (auto i = 0; i < N; i++) {
                        for(auto j = 0; j < M; j++) {
                            result(i,j) = value(i,j);
                        }
                    }
                    return result;
                }
            };


            /**
             * @brief Eigen / Ubitrack cast functor vector types
             */
            struct EigenQuaternionCast
            {
                Ubitrack::Math::Quaternion from_eigen( const Eigen::Quaternion<double> & value )const
                {
                    return Ubitrack::Math::Quaternion(value.x(),  value.y(), value.z(), value.w());
                }

                Eigen::Quaternion<double> to_eigen( const Ubitrack::Math::Quaternion  & value )const
                {
                    return Eigen::Quaternion<double>(value.w(), value.x(), value.y(), value.z());
                }
            };

            /**
             * @brief Eigen / Ubitrack cast functor vector types
             */
            struct EigenIntrinsicMatrixCast
            {
                Ubitrack::Math::Matrix3x3d from_eigen( const Eigen::Matrix3d & value )const
                {
                    Ubitrack::Math::Matrix3x3d result;
                    result(0, 0) = value(0, 0);
                    result(0, 1) = value(0, 1);
                    result(0, 2) = -value(0, 2);

                    result(1, 0) = value(1, 0);
                    result(1, 1) = value(1, 1);
                    result(1, 2) = -value(1, 2);

                    result(2, 0) = value(2, 0);
                    result(2, 1) = value(2, 1);
                    result(2, 2) = value(2, 2);
                    return result;
                }

                Eigen::Matrix3d to_eigen( const Ubitrack::Math::Matrix3x3d   & value )const
                {
                    Eigen::Matrix3d result;
                    result(0, 0) = value(0, 0);
                    result(0, 1) = value(0, 1);
                    result(0, 2) = -value(0, 2);

                    result(1, 0) = value(1, 0);
                    result(1, 1) = value(1, 1);
                    result(1, 2) = -value(1, 2);

                    result(2, 0) = value(2, 0);
                    result(2, 1) = value(2, 1);
                    result(2, 2) = value(2, 2);
                    return result;
                }
            };

        } // Wrapper
    }} // Ubitrack::Math

#endif //ARTEKMED_P1_EIGENWRAPPER_H
