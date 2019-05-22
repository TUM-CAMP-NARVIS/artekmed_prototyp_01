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

#include "vec.hpp"
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cassert>

using namespace std;


namespace cp
{
	// index offsets for neighbor cells in a grid
	static const vec3i neighborOffsets[27] = {
		vec3i(-1, -1, -1), vec3i(0, -1, -1), vec3i(1, -1, -1),
		vec3i(-1, 0, -1), vec3i(0, 0, -1), vec3i(1, 0, -1),
		vec3i(-1, 1, -1), vec3i(0, 1, -1), vec3i(1, 1, -1),
		vec3i(-1, -1, 0), vec3i(0, -1, 0), vec3i(1, -1, 0),
		vec3i(-1, 0, 0), vec3i(0, 0, 0), vec3i(1, 0, 0),
		vec3i(-1, 1, 0), vec3i(0, 1, 0), vec3i(1, 1, 0),
		vec3i(-1, -1, 1), vec3i(0, -1, 1), vec3i(1, -1, 1),
		vec3i(-1, 0, 1), vec3i(0, 0, 1), vec3i(1, 0, 1),
		vec3i(-1, 1, 1), vec3i(0, 1, 1), vec3i(1, 1, 1)
	};



	// primitive 3D hash grid
	class PointIndex
	{
	private:
		// hash function for a vec3i
		struct cellHasher
		{
			static const size_t bucket_size = 10;	// mean bucket size that the container should try not to exceed
			static const size_t min_buckets = 1204;	// minimum number of buckets, power of 2, >0

			cellHasher() {}

			size_t operator()(const vec3i &x) const
			{
				return hash<uint>()(x.x) ^ hash<uint>()(x.y) ^ hash<uint>()(x.z);
			}

			bool operator()(const vec3i& left, const vec3i& right)
			{
				if (left.x != right.x)	return left.x < right.x;
				if (left.y != right.y)	return left.y < right.y;
				return left.z < right.z;
			}
		};

		typedef unordered_map<vec3i, vector<uint>, cellHasher> HashGrid;


		HashGrid mGrid;
		const vector<vec3>* mPoints;
		vec3 mBBmin;
		vec3 mBBmax;
		vec3 mBBsize;		// world space dimensions
		vec3i mGridSize;	// grid dimensions
		float mCellSize;


	private:
		struct gridCoordPred
		{
			vec3 mBBmin;
			float mCellSize;
			vec3i mMaxGridCoord;
			const vector<vec3>* mPoints;

			gridCoordPred()
			{}

			gridCoordPred(const vector<vec3>& points, const vec3& bbMin, float cellSize, const vec3i& gridSize)
				: mPoints(&points), mBBmin(bbMin), mCellSize(cellSize), mMaxGridCoord(gridSize - vec3i(1, 1, 1))
			{}

			/// compares ordering of indices based on the grid coordinate of their associating points
			bool operator()(const uint& a, const uint& b)
			{
				cellHasher hasher;
				vec3i aIndex = min(vec3i(((*mPoints)[a] - mBBmin) / mCellSize), mMaxGridCoord);
				vec3i bIndex = min(vec3i(((*mPoints)[b] - mBBmin) / mCellSize), mMaxGridCoord);
				return hasher(aIndex, bIndex);
			}
		};

		struct distancePred
		{
			vec3 mQueryPoint;
			const vector<vec3>* mPoints;

			distancePred() {}
			distancePred(const vector<vec3>* points, const vec3& queryPoint) : mPoints(points), mQueryPoint(queryPoint) {}

			bool operator()(const uint& a, const uint& b)
			{
				return squaredDist(mQueryPoint, (*mPoints)[a]) < squaredDist(mQueryPoint, (*mPoints)[b]);
			}
		};

	public:
		PointIndex()
		{
		}

		PointIndex(const vector<vec3>& points, float maxSearchRadius)
		{
			create(points, maxSearchRadius);
		}

		void create(const vector<vec3>& points, float maxSearchRadius)
		{
			assert(!points.empty());	// can't create index on empty set

			mGrid.clear();
			mPoints = &points;

			// compute bounding box (with epsilon space border)
			mBBmin = vec3(FLT_MAX, FLT_MAX, FLT_MAX);
			mBBmax = vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
			for (vector<vec3>::const_iterator p = points.begin(); p != points.end(); ++p)
			{
				const vec3 pt = *p;
				mBBmin = min(mBBmin, pt);
				mBBmax = max(mBBmax, pt);
			}

			// create dim cells of size maxSearchRadius, and adapt bbox
			mCellSize = maxSearchRadius;
			mBBsize = mBBmax - mBBmin;
			mGridSize = vec3i(mBBsize / mCellSize) + vec3i(1, 1, 1);
			mBBsize = vec3(mGridSize) * mCellSize;
			vec3 halfSize = mBBsize * 0.5;
			vec3 center = (mBBmax + mBBmin) * 0.5f;
			mBBmin = center - halfSize;
			mBBmax = center + halfSize;


			// create point index buffer sorted by their grid coordinates
			vector<uint> indices(points.size());
			for (uint i = 0; i < indices.size(); ++i) indices[i] = i;
			sort(indices.begin(), indices.end(), gridCoordPred(points, mBBmin, mCellSize, mGridSize));


			// populate grid
			vector<uint> currentList;
			vec3i currentGridCoord = getGridCoord(points[indices[0]]);
			for (vector<uint>::iterator it = indices.begin(); it != indices.end(); ++it)
			{
				// next point index and associate gridCoord
				uint i = *it;
				vec3i gridCoord = getGridCoord(points[i]);

				// if we have a new gridCoord, finish current cell at currentGridCoord first
				if (gridCoord != currentGridCoord)
				{
					mGrid[currentGridCoord] = currentList;
					currentList.clear();
					currentGridCoord = gridCoord;
				}
				currentList.push_back(i);
			}
			mGrid[currentGridCoord] = currentList;		// finish index list for last cell
		}


		// retrieve grid coordinates of point p
		vec3i getGridCoord(const vec3& p)
		{
			return min(vec3i((p - mBBmin) / mCellSize), mGridSize - vec3i(1, 1, 1));
		}

		// retrieve the side length of a grid cell. This equals the maximum reliable search radius
		float cellSize()
		{
			return mCellSize;
		}


		// approximate k nearest neighbor search within a maximum radius of mCellSize.
		// if a neighbor's distance is not within the 3x3x3 neighboring cells, it is not returned.
		// all previous content in outIndices will be cleared.
		void annSearch(const vec3& queryPoint, uint k, vector<uint>& outIndices)
		{
			outIndices.clear();
			radiusSearch(queryPoint, sqrtf(12 * mCellSize * mCellSize), outIndices);
			// sort by distance
			sort(outIndices.begin(), outIndices.end(), distancePred(mPoints, queryPoint));
			// remove any indices beyond k
			outIndices.erase(outIndices.begin() + min(k, outIndices.size()), outIndices.end());
		}



		// queries all indices within a radius ball around queryPoint and write them to the vector outIndices
		// all previous content in outIndices will be cleared.
		void radiusSearch(const vec3& queryPoint, float radius, vector<uint>& outIndices)
		{
			outIndices.clear();
			vec3i c = getGridCoord(queryPoint);

			// visit each neighbor cell and process points in there
			for (uint i = 0; i < 27; ++i)
			{
				// find n in the hash grid
				vec3i n = c + neighborOffsets[i];
				if (mGrid.find(n) != mGrid.end())
				{
					const vector<uint>& indices = mGrid[n];

					// search point list of neighbor cell for in-range points
					for (vector<uint>::const_iterator it = indices.begin(); it != indices.end(); ++it)
					{
						uint i = *it;
						if (squaredDist(queryPoint, (*mPoints)[i]) < radius * radius)
							outIndices.push_back(i);
					}
				}
			}
		}


		// radius search for a list of queryPoints with common radius.
		// all previous content in outIndices will be cleared.
		void radiusSearch(const vector<vec3>& queryPoints, float radius, vector<vector<uint>> outIndices)
		{
			outIndices.resize(queryPoints.size());
			for (uint i = 0; i < queryPoints.size(); ++i)
				radiusSearch(queryPoints[i], radius, outIndices[i]);
		}


		// radius search for a list of queryPoints with individual radii.
		// all previous content in outIndices will be cleared.
		void radiusSearch(const vector<vec3>& queryPoints, const vector<float>& radii, vector<vector<uint>> outIndices)
		{
			assert(queryPoints.size() == radii.size());

			outIndices.resize(queryPoints.size());
			for (uint i = 0; i < queryPoints.size(); ++i)
				radiusSearch(queryPoints[i], radii[i], outIndices[i]);
		}

	};	/// class PointIndex


}	/// end namespace cp
