#ifndef __SIMPLE_AR_DEMO_UTCONNECTOR_HH__
#define __SIMPLE_AR_DEMO_UTCONNECTOR_HH__

#ifdef _WIN32
# pragma warning (disable : 4231)
#endif


#include <string>
#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <functional>
#include <vector>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <chrono>


// include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/quaternion.hpp>

/*
* UTSimpleARConnector requires one Camera and a single tracking target
* The DFG needs to contain the following ApplicationSinks:
* - camera_left_image: Push/Image of the camera
* - camera_left_intrinsics: Pull/CameraIntrinsics with the calibrated intrinsics/distortion/resolution
* - camera_left_pose: Pull/Pose of the camera
* - target1_pose: Pull/Pose of a target
*
*/
class UTSimpleARConnector : public UTBaseConnector {
public:
	UTSimpleARConnector(const std::string& _components_path);
	~UTSimpleARConnector();

	// public api

	// first camera input
	// naming already reflects future extensions to stereo camera setup
	bool camera_left_get_intrinsics(const TimestampT ts, glm::mat3& intrinsics, glm::ivec2& resolution);
	bool camera_right_get_intrinsics(const TimestampT ts, glm::mat3& intrinsics, glm::ivec2& resolution);
	bool camera_left_get_pose(const TimestampT ts, glm::mat4& pose);
	bool left2right_get_pose(const TimestampT ts, glm::mat4& pose);
	bool camera_left_get_current_image(std::shared_ptr<Facade::BasicImageMeasurement >& img);
	bool camera_get_current_image_right(const TimestampT ts, std::shared_ptr<Facade::BasicImageMeasurement> & img);


	// private api (still needs to be public unless we declare friend classes, which is considered to be bad practice.)
	// extend methods to connnect/disconnect sinks
	virtual bool initialize(const std::string& _utql_filename);
	virtual bool teardown();

	// handlers for push sinks
	void receive_camera_left_image(std::shared_ptr<Facade::BasicImageMeasurement>& pose);

private:
	Ubitrack::Facade::BasicPushSink< Facade::BasicImageMeasurement >*            m_pushsink_camera_image_left;
	Ubitrack::Facade::BasicPullSink< Facade::BasicImageMeasurement >*            m_pullsink_camera_image_right;

	Ubitrack::Facade::BasicPullSink< Facade::BasicMatrixMeasurement< 3, 3 > >*   m_pullsink_camera_intrinsics_left;
	Ubitrack::Facade::BasicPullSink< Facade::BasicVectorMeasurement< 2 > >*      m_pullsink_camera_resolution_left;

	Ubitrack::Facade::BasicPullSink< Facade::BasicMatrixMeasurement< 3, 3 > >*   m_pullsink_camera_intrinsics_right;
	Ubitrack::Facade::BasicPullSink< Facade::BasicVectorMeasurement< 2 > >*      m_pullsink_camera_resolution_right;

	Ubitrack::Facade::BasicPullSink< Facade::BasicPoseMeasurement >*             m_pullsink_camera_pose_left;
	Ubitrack::Facade::BasicPullSink< Facade::BasicPoseMeasurement >*             m_pullsink_left2right_pose;


	std::mutex m_textureAccessMutex;
	std::shared_ptr<Facade::BasicImageMeasurement > m_current_camera_left_image;


};



#endif // __SIMPLE_AR_DEMO_UTCONNECTOR_HH__