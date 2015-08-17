#include "simple_ar_demo/utconnector.h"
#include "simple_ar_demo/easylogging++.h"

// implementation of UTBaseConnector

UTBaseConnector::UTBaseConnector(const std::string& _components_path)
	: m_utFacade( _components_path.c_str() )
	, m_haveNewFrame( false )
	, m_lastTimestamp( 0 )
	, m_dataflowLoaded( false )
	, m_dataflowRunning( false )
{

}

UTBaseConnector::~UTBaseConnector() {
	// what needs to be done for teardown ??

}

bool UTBaseConnector::initialize(const std::string& _utql_filename) {
	// needs to catch exceptions ..
	m_utFacade.loadDataflow( _utql_filename.c_str() );
	m_dataflowLoaded = true;
	return true;
}

bool UTBaseConnector::teardown() {
	// needs to catch exceptions ..
	if (m_dataflowRunning) {
		stop();
	}

	if (m_dataflowLoaded) {
		m_utFacade.clearDataflow();
		m_dataflowLoaded = false;
	}
	return true;
}

bool UTBaseConnector::start() {
	// needs to catch exceptions ..
	if (m_dataflowLoaded) {
		m_utFacade.startDataflow();
		m_dataflowRunning = true;
	}
	return true;
}

bool UTBaseConnector::stop() {
	// needs to catch exceptions ..
	if (m_dataflowRunning) {
		m_utFacade.stopDataflow();
		m_dataflowRunning = false;
	}
	return true;
}

void UTBaseConnector::set_new_frame(unsigned long long int ts) {
	tbb::interface5::unique_lock<tbb::mutex> ul( m_waitMutex );
	m_lastTimestamp = ts;
	m_haveNewFrame = true;
	m_waitCondition.notify_one();
}


unsigned long long int UTBaseConnector::wait_for_frame() {
	// find a way to exit here in case we want to stop
	// alternatively, we could only query the m_haveNewFrame variable (polling)
	unsigned long long int ts(0);
	while (!m_haveNewFrame) {
		tbb::interface5::unique_lock<tbb::mutex> ul( m_waitMutex );
		m_waitCondition.wait( ul );
		ts = m_lastTimestamp;
	}

	// reset haveNewFrame immediately to prepare for the next frame
	// maybe this should be done in a seperate method ??
	{
		tbb::interface5::unique_lock<tbb::mutex> ul( m_waitMutex );
		m_haveNewFrame = false;
	}
	return ts;
}

// implementation of UTSimpleARConnector

UTSimpleARConnector::UTSimpleARConnector(const std::string& _components_path)
	: UTBaseConnector(_components_path)
    , m_pushsink_camera_image_left(NULL)
    , m_pullsink_camera_intrinsics_left(NULL)
	, m_pullsink_camera_resolution_left(NULL)
    , m_pullsink_camera_pose_left(NULL)
    , m_pullsink_camera_image_depth_left(NULL)
	, m_pullsink_camera_intrinsics_right(NULL)
	, m_pullsink_camera_resolution_right(NULL)
	, m_pullsink_camera_image_depth_right(NULL)
	, m_pullsink_camera_image_right(NULL)
	//    , m_pullsink_target1_pose(NULL)
{}

UTSimpleARConnector::~UTSimpleARConnector()
{}

bool UTSimpleARConnector::initialize(const std::string& _utql_filename)
{
	bool ret = UTBaseConnector::initialize(_utql_filename);

	// create sinks/sources
	if (m_pushsink_camera_image_left != NULL) {
		delete m_pushsink_camera_image_left;
	}
	m_pushsink_camera_image_left = m_utFacade.getPushSink<Facade::BasicImageMeasurement>("camera_image_left");

	if(m_pullsink_camera_image_right !=NULL)
		delete m_pullsink_camera_image_right;
	m_pullsink_camera_image_right = m_utFacade.getPullSink<Facade::BasicImageMeasurement>("camera_image_right");

	
	if(m_pullsink_camera_image_depth_right !=NULL)
		delete m_pullsink_camera_image_depth_right;
	m_pullsink_camera_image_depth_right = m_utFacade.getPullSink<Facade::BasicImageMeasurement>("depth_image_right");


	if (m_pullsink_camera_intrinsics_right != NULL) {
		delete m_pullsink_camera_intrinsics_right;
	}
	m_pullsink_camera_intrinsics_right = m_utFacade.getPullSink<Facade::BasicMatrixMeasurement< 3, 3 > >("camera_intrinsics_right");

	if (m_pullsink_camera_resolution_right != NULL) {
		delete m_pullsink_camera_resolution_right;
	}
	m_pullsink_camera_resolution_right = m_utFacade.getPullSink<Facade::BasicVectorMeasurement< 2 > >("camera_resolution_right");


	if (m_pullsink_camera_intrinsics_left != NULL) {
		delete m_pullsink_camera_intrinsics_left;
	}
	m_pullsink_camera_intrinsics_left = m_utFacade.getPullSink<Facade::BasicMatrixMeasurement< 3, 3 > >("camera_intrinsics_left");

	if (m_pullsink_camera_resolution_left != NULL) {
		delete m_pullsink_camera_resolution_left;
	}
	m_pullsink_camera_resolution_left = m_utFacade.getPullSink<Facade::BasicVectorMeasurement< 2 > >("camera_resolution_left");

	if (m_pullsink_camera_pose_left != NULL) {
		delete m_pullsink_camera_pose_left;
	}
	m_pullsink_camera_pose_left = m_utFacade.getPullSink<Facade::BasicPoseMeasurement>("camera_pose_left");
	if(m_pullsink_camera_image_depth_left != NULL)
	{
		delete m_pullsink_camera_image_depth_left;
	}
	m_pullsink_camera_image_depth_left= m_utFacade.getPullSink<Facade::BasicImageMeasurement>("depth_camera_left");

	if (m_pushsink_camera_image_left) {
		m_pushsink_camera_image_left->registerCallback(std::bind(&UTSimpleARConnector::receive_camera_left_image, this, std::placeholders::_1));
	}
	return ret;
}

bool UTSimpleARConnector::teardown()
{
	// need to unregister callback here !!!
	if (m_pushsink_camera_image_left) {
		m_pushsink_camera_image_left->unregisterCallback();
	}


	// delete sinks/sources
	if (m_pushsink_camera_image_left != NULL) {
		delete m_pushsink_camera_image_left;
	}

	if (m_pullsink_camera_intrinsics_left != NULL) {
		delete m_pullsink_camera_intrinsics_left;
	}

	if (m_pullsink_camera_resolution_left != NULL) {
		delete m_pullsink_camera_resolution_left;
	}

	if (m_pullsink_camera_pose_left != NULL) {
		delete m_pullsink_camera_pose_left;
	}
	if(m_pullsink_camera_image_depth_left !=NULL)
		delete m_pullsink_camera_image_depth_left;

//	if (m_pullsink_target1_pose != NULL) {
//		delete m_pullsink_target1_pose;
//	}

	bool ret = UTBaseConnector::teardown();
	return ret;
}

bool UTSimpleARConnector::camera_right_get_intrinsics(const TimestampT ts, glm::mat3& intrinsics, glm::ivec2& resolution) 
{
	if ((!m_pullsink_camera_intrinsics_right) || (!m_pullsink_camera_resolution_right)) {
		LERROR << "pullsinks are not connected";
		return false;
	}

	try {
		std::vector<float> v_intr(9);
		std::shared_ptr<Facade::BasicMatrixMeasurement< 3, 3 > > m_intr = m_pullsink_camera_intrinsics_right->get(ts);
		if (!m_intr) {
			LERROR << "no measurement for camera intrinsics";
			return false;
		}
		m_intr->get(v_intr);
		intrinsics = glm::make_mat3(&v_intr[0]);
		//LINFO<<"XXXXX Intrinsics: "<<glm::to_string(intrinsics);

		std::vector<float> v_res(2);
		std::shared_ptr<Facade::BasicVectorMeasurement< 2 > > m_res = m_pullsink_camera_resolution_right->get(ts);
		if (!m_res) {
			LERROR << "no measurement for camera resolution";
			return false;
		}
		m_res->get(v_res);
		resolution = glm::ivec2( (int)(v_res.at(0)), (int)(v_res.at(1)) );
	} catch( std::exception &e) {
		LERROR << "error pulling intrinsics: " << e.what();
		return false;
	}
	return true;
}

bool UTSimpleARConnector::camera_left_get_intrinsics(const TimestampT ts, glm::mat3& intrinsics, glm::ivec2& resolution) 
{
	if ((!m_pullsink_camera_intrinsics_left) || (!m_pullsink_camera_resolution_left)) {
		LERROR << "pullsinks are not connected";
		return false;
	}

	try {
		std::vector<float> v_intr(9);
		std::shared_ptr<Facade::BasicMatrixMeasurement< 3, 3 > > m_intr = m_pullsink_camera_intrinsics_left->get(ts);
		if (!m_intr) {
			LERROR << "no measurement for camera intrinsics";
			return false;
		}
		m_intr->get(v_intr);
		intrinsics = glm::make_mat3(&v_intr[0]);
		//LINFO<<"XXXXX Intrinsics: "<<glm::to_string(intrinsics);

		std::vector<float> v_res(2);
		std::shared_ptr<Facade::BasicVectorMeasurement< 2 > > m_res = m_pullsink_camera_resolution_left->get(ts);
		if (!m_res) {
			LERROR << "no measurement for camera resolution";
			return false;
		}
		m_res->get(v_res);
		resolution = glm::ivec2( (int)(v_res.at(0)), (int)(v_res.at(1)) );
	} catch( std::exception &e) {
		LERROR << "error pulling intrinsics: " << e.what();
		return false;
	}
	return true;
}

bool UTSimpleARConnector::camera_left_get_current_image(std::shared_ptr<Facade::BasicImageMeasurement >& img)
{
	// we need locking here to prevent concurrent access to m_current_camera_left_image (when receiving new frame)
	tbb::interface5::unique_lock<tbb::mutex> ul( m_textureAccessMutex );
	img = m_current_camera_left_image;
	return true;
}
bool UTSimpleARConnector::camera_depth_get_current_image_left(const TimestampT ts, std::shared_ptr<Facade::BasicImageMeasurement> & img)
{
	if (m_pullsink_camera_image_depth_left == NULL) {
		LERROR << "pullsink is not connected";
		return false;
	}
	try{
		std::shared_ptr<Facade::BasicImageMeasurement> m_image= m_pullsink_camera_image_depth_left->get(ts);
		img= m_image;
	}
	catch(std::exception & e)
	{
		LERROR << "error pulling camera pose: " << e.what();
		return false;
	}
	return true;

}
bool UTSimpleARConnector::camera_depth_get_current_image_right(const TimestampT ts, std::shared_ptr<Facade::BasicImageMeasurement> & img)
{
	if (m_pullsink_camera_image_depth_right == NULL) {
		LERROR << "pullsink is not connected";
		return false;
	}
	try{
		std::shared_ptr<Facade::BasicImageMeasurement> m_image= m_pullsink_camera_image_depth_right->get(ts);
		img= m_image;
	}
	catch(std::exception & e)
	{
		LERROR << "error pulling camera pose: " << e.what();
		return false;
	}
	return true;
}
bool UTSimpleARConnector::camera_get_current_image_right(const TimestampT ts, std::shared_ptr<Facade::BasicImageMeasurement> & img)
{
	if (m_pullsink_camera_image_right == NULL) {
		LERROR << "pullsink is not connected";
		return false;
	}
	try{
		std::shared_ptr<Facade::BasicImageMeasurement> m_image= m_pullsink_camera_image_right->get(ts);
		img= m_image;
	}
	catch(std::exception & e)
	{
		LERROR << "error pulling camera pose: " << e.what();
		return false;
	}
	return true;

}

bool UTSimpleARConnector::camera_left_get_pose(const TimestampT ts, glm::mat4& pose) 
{
	if (m_pullsink_camera_pose_left == NULL) {
		LERROR << "pullsink is not connected";
		return false;
	}
	try {
		std::vector<float> v_pose(7);
		std::shared_ptr<Facade::BasicPoseMeasurement > m_pose = m_pullsink_camera_pose_left->get(ts);
		if (!m_pose) {
			LERROR << "no measurement for camera pose";
			return false;
		}
		m_pose->get(v_pose);

		glm::vec3 position = glm::make_vec3(&v_pose[0]);
		glm::quat rotation = glm::make_quat(&v_pose[3]);

		// this might not be correct .. needs a check !!!
		//Checked this, it's correct
		glm::mat4 rotMatrix = glm::mat4_cast(rotation);   //rotation is glm::quat
		rotMatrix[3].x=position.x;
		rotMatrix[3].y=position.y;
		rotMatrix[3].z=position.z;
		rotMatrix[3].w=1.;

		//glm::mat4 transMatrix = glm::translate(glm::mat4(1.0f), position);
		//LINFO<<"Rotation: "<<glm::to_string(transMatrix);
		//pose = rotMatrix * transMatrix;
		//LINFO << "camera pose: " << glm::to_string(pose);
		pose = rotMatrix;

	} catch( std::exception &e) {
		LERROR << "error pulling camera pose: " << e.what();
		return false;
	} catch(...) {
		LERROR << "error pulling camera pose (undefined) ";
		return false;
	}
	return true;
}

//bool UTSimpleARConnector::target1_get_pose(const TimestampT ts, glm::mat4& pose)
//{
//	if (!m_pullsink_target1_pose) {
//		std::cout << "pullsinks are not connected"  << std::endl;
//		return false;
//	}
//	try {
//		std::vector<float> v_pose(7);
//		std::shared_ptr<Facade::BasicPoseMeasurement > m_pose = m_pullsink_target1_pose->get(ts);
//		if (!m_pose) {
//			LERROR << "no measurement for target pose";
//			return false;
//		}
//		m_pose->get(v_pose);
//
//		glm::vec3 position = glm::make_vec3(&v_pose[0]);
//		glm::quat rotation = glm::make_quat(&v_pose[3]);
//
//		// this might not be correct .. needs a check !!!
//		glm::mat4 rotMatrix = glm::mat4_cast(rotation);   //rotation is glm::quat
//		glm::mat4 transMatrix = glm::translate(glm::mat4(1.0f), position);
//		pose = rotMatrix * transMatrix;
//
//		std::cout << "target pose: " << glm::to_string(pose) << std::endl;
//
//	} catch( std::exception &e) {
//		std::cout << "error pulling target1 pose: " << e.what() << std::endl;
//		return false;
//	} catch(...) {
//		std::cout << "error pulling camera pose (undefined) " << std::endl;
//		return false;
//	}
//	return true;
//}


void UTSimpleARConnector::receive_camera_left_image(std::shared_ptr<Facade::BasicImageMeasurement>& image)
{

	//LDEBUG << "Image received for timestamp: " << image->time();
	// store image reference for upload
	{
		tbb::interface5::unique_lock<tbb::mutex> ul( m_textureAccessMutex );
		m_current_camera_left_image = image;
	}
	// notify renderer that new frame is available
	set_new_frame(image->time());
}

