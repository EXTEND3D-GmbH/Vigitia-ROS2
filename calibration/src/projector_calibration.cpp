/**
 * Copyright (C) 2022 EXTEND3D and/or its subsidiary(-ies)
 * https://extend3d.com
 *
 * @author Chris Brammer
 */

#pragma once
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <image_geometry/pinhole_camera_model.h>

#include <image_transport/image_transport.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <calibration/visibility_control.h>
#include <calibration_interfaces/action/calibrate.hpp>

#include <tinyxml2.h>

class BinarySemaphore
{
	std::mutex m_mutex;
	std::condition_variable m_condition;
	bool m_signalled = false;

public:
	void release()
	{
		std::lock_guard< decltype( m_mutex ) > lock( m_mutex );
		m_signalled = true;
		m_condition.notify_one();
	}

	void acquire()
	{
		std::unique_lock< decltype( m_mutex ) > lock( m_mutex );
		m_condition.wait( lock, [ this ] { return m_signalled; } );
		m_signalled = false;
	}

	bool try_acquire_timeout( std::chrono::nanoseconds timeout )
	{
		std::unique_lock< decltype( m_mutex ) > lock( m_mutex );
		if ( m_condition.wait_for( lock, timeout, [ this ] { return m_signalled; } ) ) {
			m_signalled = false;
			return true;
		}
		return false;
	}

	bool try_acquire()
	{
		std::lock_guard< decltype( m_mutex ) > lock( m_mutex );
		if ( m_signalled ) {
			m_signalled = false;
			return true;
		}
		return false;
	}
};

class MsgEmptyException : public std::runtime_error
{
public:
	using std::runtime_error::runtime_error;
};

float getDepthSubpix( const cv::Mat& depth, cv::Point2f pt )
{
	cv::Mat patch;
	cv::getRectSubPix( depth, cv::Size( 1, 1 ), pt, patch );
	return patch.at< float >( 0, 0 );
}

class CalibrationActionServer : public rclcpp::Node
{
	using Calibrate = calibration_interfaces::action::Calibrate;
	using GoalHandleCalibrate = rclcpp_action::ServerGoalHandle< Calibrate >;

public:
	PROJECTOR_CALIBRATION_CPP_PUBLIC CalibrationActionServer( const rclcpp::NodeOptions& options
															  = rclcpp::NodeOptions() )
		: Node( "e3d_calibration_action_server", options )
	{
		using namespace std::placeholders;

		m_action_server = rclcpp_action::create_server< Calibrate >(
			this,
			"calibrate",
			std::bind( &CalibrationActionServer::handle_goal, this, _1, _2 ),
			std::bind( &CalibrationActionServer::handle_cancel, this, _1 ),
			std::bind( &CalibrationActionServer::handle_accepted, this, _1 ) );
	}

private:
	rclcpp_action::Server< Calibrate >::SharedPtr m_action_server;

	rclcpp_action::GoalResponse handle_goal( const rclcpp_action::GoalUUID& uuid,
											 std::shared_ptr< const Calibrate::Goal > goal )
	{
		if ( goal->camera_topic.empty() || goal->camera_info_topic.empty() || goal->depth_topic.empty()
			 || goal->depth_scale_to_m == 0 || goal->projector_topic.empty() || goal->grid_height == 0
			 || goal->grid_width == 0 || goal->projector_height == 0 || goal->projector_width == 0 ) {
			return rclcpp_action::GoalResponse::REJECT;
		}
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse handle_cancel( const std::shared_ptr< GoalHandleCalibrate > goal_handle )
	{
		RCLCPP_INFO( this->get_logger(), "Received request to cancel goal" );
		(void)goal_handle;
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	void handle_accepted( const std::shared_ptr< GoalHandleCalibrate > goal_handle )
	{
		using namespace std::placeholders;
		// this needs to return quickly to avoid blocking the executor, so spin up a new thread
		std::thread{ std::bind( &CalibrationActionServer::execute, this, _1 ), goal_handle }.detach();
	}

	void execute( const std::shared_ptr< GoalHandleCalibrate > goal_handle );
};

class Calibrator
{
	using SyncPolicy = message_filters::sync_policies::
		ApproximateTime< sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo >;
	using Calibrate = calibration_interfaces::action::Calibrate;
	using GoalHandleCalibrate = rclcpp_action::ServerGoalHandle< Calibrate >;
	const size_t SHOTS_TILL_CALIB = 5;

public:
	struct CalibrationResult
	{
		cv::Vec3d translation;
		cv::Mat rotation;
		cv::Mat intrinsics;
		cv::Mat distortion;
	};

	Calibrator( CalibrationActionServer* node, const std::shared_ptr< GoalHandleCalibrate > goalHandle )

		: m_node( node )
		, m_goalHandle( goalHandle )
		, m_imageSub( m_node, goalHandle->get_goal()->camera_topic )
		, m_depthSub( m_node, goalHandle->get_goal()->depth_topic )
		, m_depthScale( goalHandle->get_goal()->depth_scale_to_m )
		, m_infoSub( m_node, goalHandle->get_goal()->camera_info_topic )
		, m_cols( goalHandle->get_goal()->grid_width )
		, m_rows( goalHandle->get_goal()->grid_height )
		, m_width( goalHandle->get_goal()->projector_width )
		, m_height( goalHandle->get_goal()->projector_height )
		, m_pathToSave( goalHandle->get_goal()->save_path )
		, m_syncer( SyncPolicy( 5 ), m_imageSub, m_depthSub, m_infoSub )
	{
		generate_checkerboard();
		m_syncer.registerCallback( std::bind( &Calibrator::callback,
											  this,
											  std::placeholders::_1,
											  std::placeholders::_2,
											  std::placeholders::_3 ) );
		m_worker = std::thread( [ this ]() { execute(); } );
		m_publisher = image_transport::create_publisher( m_node, goalHandle->get_goal()->projector_topic );
		using namespace std::chrono_literals;
		m_timer = m_node->create_wall_timer( 100ms, [ this ]() {
			std::lock_guard< std::mutex > lock( m_projectionContentMutex );
			cv_bridge::CvImage toPublish;
			toPublish.header.stamp = m_node->now();
			toPublish.header.frame_id = "projector_link";
			toPublish.image = m_projectable;
			toPublish.encoding = sensor_msgs::image_encodings::MONO8;

			auto message = toPublish.toImageMsg();
			m_publisher.publish( message );
		} );
	}
	~Calibrator()
	{
		m_worker.join();
	}
	void wait_for_finish()
	{
		m_finished_promise.get_future().wait();
	}

private:
	template < typename Scalar, size_t SIZE >
	void copyTo( cv::Mat from, std::array< Scalar, SIZE >& to )
	{
		if ( !from.isContinuous() ) throw std::runtime_error( "Origin Mat not continuous" );
		if ( from.total() != SIZE ) throw std::runtime_error( "Mat differs in size from array" );
		std::memcpy( to.data(), from.data, from.total() * sizeof( Scalar ) );
	}
	template < typename Scalar >
	void copyTo( cv::Mat from, std::vector< Scalar >& to )
	{
		if ( !from.isContinuous() ) throw std::runtime_error( "Origin Mat not continuous" );
		to.resize( from.total() );
		std::memcpy( to.data(), from.data, from.total() * sizeof( Scalar ) );
	}

	void generate_checkerboard()
	{
		// +1 to have cols x rows internal corners
		auto totalCols = m_cols + 1;
		auto totalRows = m_rows + 1;

		// + 1 for padding on screen border
		auto size = std::min( m_width / ( totalCols + 1 ), m_height / ( totalRows + 1 ) );

		auto xSpacing = ( m_width - totalCols * size ) / 2;
		auto ySpacing = ( m_height - totalRows * size ) / 2;

		m_projectorCorners.clear();
		m_projectorCorners.reserve( m_cols * m_rows );
		m_projectable = cv::Mat( cv::Size( m_width, m_height ), CV_8UC1, 255 );
		for ( int y = 0; y < totalRows; y++ ) {
			for ( int x = 0; x < totalCols; x++ ) {
				auto xStep = x * size + xSpacing;
				auto yStep = y * size + ySpacing;

				if ( x % 2 == y % 2 ) {
					cv::rectangle( m_projectable, cv::Rect( xStep, yStep, size, size ), 0, cv::FILLED );
				}

				if ( x > 0 && x < totalCols && y > 0 && y < totalRows ) {
					// Record inner top corners
					m_projectorCorners.emplace_back( xStep, yStep );
				}
			}
		}
	}

	void callback( const sensor_msgs::msg::Image::ConstSharedPtr& imgMsg,
				   const sensor_msgs::msg::Image::ConstSharedPtr& depthMsg,
				   const sensor_msgs::msg::CameraInfo::ConstSharedPtr& infoMsg )
	{
		if ( m_done ) {
			return;
		}

		// This cancel might get stuck if we never receive an image again, thus leaking resources
		if ( m_goalHandle->is_canceling() ) {
			RCLCPP_INFO( m_node->get_logger(), "Cancelling calibration" );
			auto result = std::make_shared< Calibrate::Result >();
			m_goalHandle->canceled( result );
			finish();
		} else {
			std::lock_guard< std::mutex > lock( m_mutex );
			m_imgMsg = imgMsg;
			m_depthMsg = depthMsg;
			m_infoMsg = infoMsg;
		}

		m_signalToThread.release();
	}

	image_geometry::PinholeCameraModel addCorners()
	{
		sensor_msgs::msg::Image::ConstSharedPtr imgMsg;
		sensor_msgs::msg::Image::ConstSharedPtr depthMsg;
		sensor_msgs::msg::CameraInfo::ConstSharedPtr infoMsg;
		{
			std::lock_guard< std::mutex > lock( m_mutex );
			if ( !m_imgMsg || !m_depthMsg || !m_infoMsg ) {
				throw MsgEmptyException( "Some message missing, skipping corner extraction." );
			}
			// Move ownership to this thread and release lock
			imgMsg = std::move( m_imgMsg );
			depthMsg = std::move( m_depthMsg );
			infoMsg = std::move( m_infoMsg );
		}

		auto img = cv_bridge::toCvShare( imgMsg, imgMsg->encoding );
		img = cv_bridge::cvtColor( img, sensor_msgs::image_encodings::MONO8 );
		auto rawDepthImg = cv_bridge::toCvShare( depthMsg, depthMsg->encoding );
		cv::Mat normalizedDepth;
		rawDepthImg->image.convertTo( normalizedDepth, CV_32FC1, m_depthScale );  // to meter scale

		image_geometry::PinholeCameraModel cameraCalib;
		cameraCalib.fromCameraInfo( infoMsg );

		cv::Size chessboardSize( m_cols, m_rows );

		std::vector< cv::Point2f > corners2DRGBD;
		bool found = cv::findChessboardCorners( img->image, chessboardSize, corners2DRGBD );
		if ( !found || corners2DRGBD.size() != m_cols * m_rows ) {
			return cameraCalib;
		}
		RCLCPP_INFO( m_node->get_logger(), "Added Image to calibration" );

		cv::cornerSubPix( img->image,
						  corners2DRGBD,
						  cv::Size( 11, 11 ),
						  cv::Size( -1, -1 ),
						  cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ) );

		std::vector< cv::Point3f > corners3DRGBD;
		corners3DRGBD.reserve( corners2DRGBD.size() );

		for ( const auto& corner : corners2DRGBD ) {
			auto ray = cameraCalib.projectPixelTo3dRay( corner );
			auto depth = getDepthSubpix( normalizedDepth, corner );
			corners3DRGBD.push_back( ray * depth );
		}

		auto& corners2DProjector = m_projectorCorners;

		m_rgbdPoints.push_back( corners2DRGBD );
		m_projectorPoints.push_back( corners2DProjector );
		m_calibPoints.push_back( corners3DRGBD );

		auto feedback = std::make_shared< Calibrate::Feedback >();
		feedback->message = "Added Image to calibration";
		m_goalHandle->publish_feedback( feedback );

		return cameraCalib;
	}

	void calibrate( image_geometry::PinholeCameraModel cameraCalib )
	{
		auto feedback = std::make_shared< Calibrate::Feedback >();

		RCLCPP_INFO( m_node->get_logger(), "Beginning calibration" );
		feedback->message = "Beginning calibration";
		m_goalHandle->publish_feedback( feedback );

		// Estimate Intrinsics (opencv wont do that)
		cv::Mat projectorMatrix( cv::Size( 3, 3 ), CV_32FC1 );
		double angleRad = 2 * CV_PI / 6;  // Estimating 60deg FOV
		projectorMatrix.at< float >( 0, 0 ) = m_width / 2 * ( std::cos( angleRad ) / std::sin( angleRad ) );
		projectorMatrix.at< float >( 1, 1 ) = projectorMatrix.at< float >( 0, 0 );

		projectorMatrix.at< float >( 2, 2 ) = 1.f;
		projectorMatrix.at< float >( 0, 2 ) = m_width / 2;
		projectorMatrix.at< float >( 1, 2 ) = m_height / 2;
		cv::Mat projectorDist;
		std::vector< cv::Mat > tVec, rVec;

		// Calib projector
		auto rms = cv::calibrateCamera( m_calibPoints,
										m_projectorPoints,
										cv::Size( m_width, m_height ),
										projectorMatrix,
										projectorDist,
										rVec,
										tVec,
										cv::CALIB_USE_INTRINSIC_GUESS );
		RCLCPP_INFO( m_node->get_logger(), "Projector Intrinsic RMS: %.5f ", rms );
		// Stereo Calib
		cv::Mat r;
		cv::Vec3d t;

		auto rgbdMatrix = cameraCalib.intrinsicMatrix();
		auto rgbdDist = cameraCalib.distortionCoeffs();
		cv::stereoCalibrate( m_calibPoints,
							 m_rgbdPoints,
							 m_projectorPoints,
							 rgbdMatrix,
							 rgbdDist,
							 projectorMatrix,
							 projectorDist,
							 cv::Size( m_width, m_height ),
							 r,
							 t,
							 cv::noArray(),
							 cv::noArray(),
							 cv::CALIB_FIX_INTRINSIC );
		RCLCPP_INFO( m_node->get_logger(),
					 "Projector Extrinsics (StereoCalib):\n   T: [%.5f, %.5f, %.5f] ",
					 t.val[ 0 ],
					 t.val[ 1 ],
					 t.val[ 2 ] );

		RCLCPP_INFO( m_node->get_logger(), "Calib successful" );

		cv::Mat identity = cv::Mat::eye( 3, 3, CV_64FC1 );
		cv::Mat projection = cv::Mat::zeros( 3, 4, CV_64FC1 );
		projectorMatrix.copyTo( projection( cv::Rect( 0, 0, 3, 3 ) ) );

		auto result = std::make_shared< Calibrate::Result >();
		result->info.height = m_height;
		result->info.width = m_width;
		result->info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
		copyTo( projectorDist, result->info.d );
		copyTo( projectorMatrix, result->info.k );
		copyTo( identity, result->info.r );
		copyTo( projection, result->info.p );

		result->info.binning_x = 1;
		result->info.binning_y = 1;
		copyTo( r, result->r );
		result->t[ 0 ] = t.val[ 0 ];
		result->t[ 1 ] = t.val[ 1 ];
		result->t[ 2 ] = t.val[ 2 ];
		m_goalHandle->succeed( result );

		// Write to file if given:
		if ( !m_pathToSave.empty() ) {
			using namespace tinyxml2;

			RCLCPP_INFO( m_node->get_logger(), "Saving calibration to: %s", m_pathToSave.c_str() );
			{
				XMLDocument xmlDoc;
				XMLElement* camera = xmlDoc.NewElement( "CameraInfo" );
				xmlDoc.InsertFirstChild( camera );

				XMLElement* intrinsics = xmlDoc.NewElement( "Intrinsics" );
				intrinsics->SetAttribute( "m11", result->info.k[ 0 ] );
				intrinsics->SetAttribute( "m12", result->info.k[ 1 ] );
				intrinsics->SetAttribute( "m13", result->info.k[ 2 ] );
				intrinsics->SetAttribute( "m21", result->info.k[ 3 ] );
				intrinsics->SetAttribute( "m22", result->info.k[ 4 ] );
				intrinsics->SetAttribute( "m23", result->info.k[ 5 ] );
				intrinsics->SetAttribute( "m31", result->info.k[ 6 ] );
				intrinsics->SetAttribute( "m32", result->info.k[ 7 ] );
				intrinsics->SetAttribute( "m33", result->info.k[ 8 ] );
				camera->InsertEndChild( intrinsics );

				XMLElement* dist = xmlDoc.NewElement( "Distortion" );
				dist->SetAttribute( "a", result->info.d[ 0 ] );
				dist->SetAttribute( "b", result->info.d[ 1 ] );
				dist->SetAttribute( "c", result->info.d[ 2 ] );
				dist->SetAttribute( "d", result->info.d[ 3 ] );
				dist->SetAttribute( "e", result->info.d[ 4 ] );
				if ( result->info.d.size() > 5 ) {
					dist->SetAttribute( "f", result->info.d[ 5 ] );
					dist->SetAttribute( "g", result->info.d[ 6 ] );
					dist->SetAttribute( "h", result->info.d[ 7 ] );
				} else {
					dist->SetAttribute( "f", 0.0 );
					dist->SetAttribute( "g", 0.0 );
					dist->SetAttribute( "h", 0.0 );
				}
				camera->InsertEndChild( dist );

				XMLElement* image = xmlDoc.NewElement( "Image" );
				image->SetAttribute( "Width", m_width );
				image->SetAttribute( "Height", m_height );
				camera->InsertEndChild( image );

				auto intrinsicsFile = m_pathToSave + "/intrinsics.xml";
				xmlDoc.SaveFile( intrinsicsFile.c_str() );
			}
			{
				XMLDocument xmlDoc;
				XMLElement* camera = xmlDoc.NewElement( "Extrinsics" );
				xmlDoc.InsertFirstChild( camera );

				tf2::Matrix3x3 tf2_rot( r.at< double >( 0, 0 ),
										r.at< double >( 0, 1 ),
										r.at< double >( 0, 2 ),
										r.at< double >( 1, 0 ),
										r.at< double >( 1, 1 ),
										r.at< double >( 1, 2 ),
										r.at< double >( 2, 0 ),
										r.at< double >( 2, 1 ),
										r.at< double >( 2, 2 ) );
				tf2::Quaternion quaternion;
				tf2_rot.getRotation( quaternion );

				// Convert to Ubitrack format
				XMLElement* quat = xmlDoc.NewElement( "Quaternion" );
				quat->SetAttribute( "x", quaternion.getX() );
				quat->SetAttribute( "y", quaternion.getY() );
				quat->SetAttribute( "z", quaternion.getZ() );
				quat->SetAttribute( "w", quaternion.getW() );
				camera->InsertEndChild( quat );

				XMLElement* translation = xmlDoc.NewElement( "Translation" );
				// Convert to mm for Ubitrack format
				translation->SetAttribute( "x", t.val[ 0 ] );
				translation->SetAttribute( "y", t.val[ 1 ] );
				translation->SetAttribute( "z", t.val[ 2 ] );
				camera->InsertEndChild( translation );

				auto extrinsicsFile = m_pathToSave + "/extrinsics.xml";
				xmlDoc.SaveFile( extrinsicsFile.c_str() );
			}
		}

		m_done = true;
	}

	void execute()
	{
		while ( !m_done ) {
			try {
				m_signalToThread.acquire();
				{
					std::lock_guard< std::mutex > lock( m_mutex );
					if ( m_done ) {
						return;
					}
				}

				size_t viewcount_before = m_rgbdPoints.size();
				auto cameraCalib = addCorners();

				if ( m_rgbdPoints.size() == viewcount_before ) {
					// no new valid view, try again.
					continue;
				}

				if ( m_calibPoints.size() < SHOTS_TILL_CALIB ) {
					// not enough views yet.
					// our own thread, lets
					std::unique_lock< std::mutex > lock( m_projectionContentMutex );
					auto actual_checkerboard = m_projectable;
					int total_sleep_secs = 20;
					for ( size_t i = 0; i < total_sleep_secs; i++ ) {
						if ( !lock.owns_lock() ) {
							lock.lock();
						}
						m_projectable = cv::Mat( cv::Size( m_width, m_height ), CV_8UC1, 255 );
						std::stringstream reposition;
						reposition << "Reposition for " << m_rgbdPoints.size() + 1 << "/" << SHOTS_TILL_CALIB
								   << ".";

						cv::putText( m_projectable,
									 reposition.str(),
									 cv::Point( m_width / 5, m_height / 3 ),
									 cv::FONT_HERSHEY_COMPLEX_SMALL,
									 5,
									 0 );
						std::stringstream countdown;
						countdown << "Next in: " << total_sleep_secs - i << " [sec]";
						cv::putText( m_projectable,
									 countdown.str(),
									 cv::Point( m_width / 5, m_height * 2 / 3 ),
									 cv::FONT_HERSHEY_COMPLEX_SMALL,
									 5,
									 0 );
						lock.unlock();
						std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
					}
					// after countdown:
					lock.lock();
					m_projectable = actual_checkerboard;
					continue;
				} else {
					// enough views collected
					std::unique_lock< std::mutex > lock( m_projectionContentMutex );
					m_projectable = cv::Mat( cv::Size( m_width, m_height ), CV_8UC1, 255 );
					std::stringstream msg;
					msg << "Acquisition done.";
					cv::putText( m_projectable,
								 msg.str(),
								 cv::Point( m_width / 4, m_height / 2 ),
								 cv::FONT_HERSHEY_COMPLEX_SMALL,
								 5,
								 0 );
					lock.unlock();
					calibrate( cameraCalib );
					finish();
				}
			} catch ( cv::Exception& e ) {
				RCLCPP_ERROR( m_node->get_logger(), "%s", e.what() );
			} catch ( MsgEmptyException mee ) {
				RCLCPP_ERROR( m_node->get_logger(), "%s", mee.what() );
			}
		}
	}

	void finish()
	{
		std::lock_guard< std::mutex > lock( m_mutex );
		m_done = true;
		m_finished_promise.set_value();
	}

	CalibrationActionServer* m_node;
	const std::shared_ptr< GoalHandleCalibrate > m_goalHandle;

	// Async storage
	std::thread m_worker;
	BinarySemaphore m_signalToThread;
	bool m_done = false;
	std::mutex m_mutex;
	sensor_msgs::msg::Image::ConstSharedPtr m_imgMsg;
	sensor_msgs::msg::Image::ConstSharedPtr m_depthMsg;
	sensor_msgs::msg::CameraInfo::ConstSharedPtr m_infoMsg;

	// Checkerboard
	std::mutex m_projectionContentMutex;
	int m_cols;
	int m_rows;
	int m_width;
	int m_height;
	cv::Mat m_projectable;
	std::vector< cv::Point2f > m_projectorCorners;
	float m_depthScale;	 // normalize depth topic to [m]

	// Calib State (owned by thread only)
	// per image(view) one vector of points
	std::vector< std::vector< cv::Point2f > > m_rgbdPoints;
	std::vector< std::vector< cv::Point2f > > m_projectorPoints;
	std::vector< std::vector< cv::Point3f > > m_calibPoints;

	// ROS
	image_transport::Publisher m_publisher;
	std::shared_ptr< rclcpp::TimerBase > m_timer;
	message_filters::Subscriber< sensor_msgs::msg::Image > m_imageSub;
	message_filters::Subscriber< sensor_msgs::msg::Image > m_depthSub;
	message_filters::Subscriber< sensor_msgs::msg::CameraInfo > m_infoSub;
	message_filters::Synchronizer< SyncPolicy > m_syncer;

	std::string m_pathToSave;

	std::promise< void > m_finished_promise;
};

void CalibrationActionServer::execute( const std::shared_ptr< GoalHandleCalibrate > goal_handle )
{
	auto goal = goal_handle->get_goal();
	Calibrator calib( this, goal_handle );
	calib.wait_for_finish();
	RCLCPP_INFO( get_logger(), "Shutting down Action thread" );
}

RCLCPP_COMPONENTS_REGISTER_NODE( CalibrationActionServer )

int main( int argc, const char** argv )
{
	rclcpp::init( argc, argv );
	auto node = std::make_shared< CalibrationActionServer >();
	rclcpp::spin( node );

	rclcpp::shutdown();
}