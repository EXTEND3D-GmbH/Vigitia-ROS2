/**
 * Copyright (C) 2022 EXTEND3D and/or its subsidiary(-ies)
 * https://extend3d.com
 *
 * @author Chris Brammer
 */
#include "ThreadedTouchTracker.h"

static cv::Mat frameToMat( const sensor_msgs::msg::Image::ConstSharedPtr& f )
{
	using namespace cv;

	auto w = f->width;
	auto h = f->height;

	if ( f->encoding == "mono16" || f->encoding == "16UC1" ) {
		return Mat( Size( w, h ), CV_16UC1, (void*)f->data.data(), Mat::AUTO_STEP );
	} else if ( f->encoding == "32FC1" ) {
		return Mat( Size( w, h ), CV_32FC1, (void*)f->data.data(), Mat::AUTO_STEP );
	} else {
		throw std::runtime_error( "Unknown image format" );
	}
}

ThreadedTouchTracker::ThreadedTouchTracker( int width, int height, ThreadedDynamicBackground* bg )
	: rclcpp::Node( "touch_publisher" ), m_tracker( width, height ), m_bg( bg )
{
	RCLCPP_INFO( get_logger(), "Starting Touch Tracker" );
	m_publisher = this->create_publisher< touch_interfaces::msg::TouchEvents >( "touch_points", 5 );
	m_publisherGeom = this->create_publisher< geometry_msgs::msg::PointStamped >( "touch_points_geom", 5 );
	m_worker = std::thread( &ThreadedTouchTracker::threadedTouchTracker, this );
}

ThreadedTouchTracker::~ThreadedTouchTracker()
{
	RCLCPP_INFO( get_logger(), "Destroying Touch Tracker" );
	{
		std::unique_lock lock( m_frameLock );
		m_isRunning = false;
		m_shutdownThread = true;
	}
	m_worker.join();
}

void ThreadedTouchTracker::start()
{
	m_isRunning = true;
}

void ThreadedTouchTracker::stop()
{
	m_isRunning = false;
}

void ThreadedTouchTracker::postNewImage( const sensor_msgs::msg::Image::ConstSharedPtr depth,
										 const sensor_msgs::msg::Image::ConstSharedPtr ir,
										 const sensor_msgs::msg::CameraInfo::ConstSharedPtr depthCI )
{
	std::unique_lock lock( m_frameLock );
	if ( !m_isRunning ) return;

	if ( m_hasNewFrame ) {
		RCLCPP_INFO( get_logger(), "Dropped Frame pair" );
	};

	m_depthFrame = std::move( depth );
	m_irFrame = std::move( ir );
	m_depthCI = std::move( depthCI );
	m_hasNewFrame = true;
	m_cv.notify_all();
}

std::vector< FingerTouch > ThreadedTouchTracker::getLastTouches()
{
	std::scoped_lock lock( m_touchLock );
	return m_lastTouches;
}

void ThreadedTouchTracker::threadedTouchTracker()
{
	while ( !m_shutdownThread ) {
		if ( !rclcpp::ok() ) {
			RCLCPP_ERROR( get_logger(), "RCLCPP not okay..." );
			break;
		}
		{
			std::unique_lock lock( m_frameLock );
			while ( !m_hasNewFrame ) {
				if ( m_shutdownThread ) return;
				m_cv.wait_for( lock, std::chrono::milliseconds( 100 ) );
			}

			m_hasNewFrame = false;

			m_currentDepthFrame = m_depthFrame;
			m_currentIrFrame = m_irFrame;
			m_depthPinhole.fromCameraInfo( m_depthCI );
		}

		// Touch tracking
		auto bg = m_bg->getBackground();
		if ( !bg.mean.empty() && !bg.stdDev.empty() ) {
			auto touches = m_tracker.run(
				frameToMat( m_currentDepthFrame ), bg.mean, bg.stdDev, frameToMat( m_currentIrFrame ) );

			{
				std::scoped_lock lock( m_touchLock );
				m_lastTouches = touches;
			}

			// publish
			touch_interfaces::msg::TouchEvents toSend;

			toSend.header.frame_id = "depth_camera_link";
			toSend.header.stamp = get_clock()->now();
			for ( const auto& touch : touches ) {
				if ( !touch.touched ) continue;

				cv::Point2d pt = { touch.tip.x, touch.tip.y };
				cv::Point2d ptRect = m_depthPinhole.rectifyPoint( pt );

				// This is just projection at z=1 (virtual Image Plane). Multiply with actual depth!
				cv::Point3d pt3d
					= m_depthPinhole.projectPixelTo3dRay( ptRect ) * touch.touchZAbsolute / 1000.0;

				auto& event = toSend.events.emplace_back();

				event.id = touch.id;
				event.point.x = pt3d.x;
				event.point.y = pt3d.y;
				event.point.z = pt3d.z;

				geometry_msgs::msg::PointStamped ptGeom;
				ptGeom.header.frame_id = "depth_camera_link";
				ptGeom.header.stamp = get_clock()->now();
				ptGeom.point.x = pt3d.x;
				ptGeom.point.y = pt3d.y;
				ptGeom.point.z = pt3d.z;

				m_publisherGeom->publish( ptGeom );
			}
			m_publisher->publish( toSend );
		}

		// rclcpp::spin_some( shared_from_this() );
	}
}