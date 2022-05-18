/**
 * Copyright (C) 2022 EXTEND3D and/or its subsidiary(-ies)
 * https://extend3d.com
 *
 * @author Chris Brammer
 */
#include <cstdio>
#include <memory>

#include "ThreadedTouchTracker.h"

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

using namespace std::chrono_literals;

class RealtimeTouchPublisher : public rclcpp::Node
{
	using SyncPolicy = message_filters::sync_policies::
		ApproximateTime< sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo >;

public:
	RealtimeTouchPublisher( const rclcpp::NodeOptions& options )
		: rclcpp::Node( "image_subscriber", options )
		, m_threadedBackground( 640, 576 )
		, m_threadedTracker( 640, 576, &m_threadedBackground )
		, m_irSubscriber( this, "/ir/image_raw" )
		, m_depthSubscriber( this, "/depth/image_raw" )
		, m_depthCameraInfo( this, "/depth/camera_info" )
		, m_syncer( SyncPolicy( 30 ), m_irSubscriber, m_depthSubscriber, m_depthCameraInfo )

	{
		m_syncer.registerCallback( std::bind( &RealtimeTouchPublisher::image_callback,
											  this,
											  std::placeholders::_1,
											  std::placeholders::_2,
											  std::placeholders::_3 ) );
	};

	virtual ~RealtimeTouchPublisher()
	{
		RCLCPP_INFO( get_logger(), "Shutdown" );
	};

private:
	void image_callback( const sensor_msgs::msg::Image::ConstSharedPtr& ir,
						 const sensor_msgs::msg::Image::ConstSharedPtr& depth,
						 const sensor_msgs::msg::CameraInfo::ConstSharedPtr& depthCI )
	{
		m_threadedBackground.postNewImage( depth );
		m_threadedTracker.postNewImage( depth, ir, depthCI );
	}

	ThreadedDynamicBackground m_threadedBackground;
	ThreadedTouchTracker m_threadedTracker;

	message_filters::Subscriber< sensor_msgs::msg::Image > m_irSubscriber;
	message_filters::Subscriber< sensor_msgs::msg::Image > m_depthSubscriber;
	message_filters::Subscriber< sensor_msgs::msg::CameraInfo > m_depthCameraInfo;
	message_filters::Synchronizer< SyncPolicy > m_syncer;
};

int main( int argc, char** argv )
{
	rclcpp::init( argc, argv );
	rclcpp::NodeOptions options;
	// options.use_intra_process_comms( true );
	rclcpp::spin( std::make_shared< RealtimeTouchPublisher >( options ) );
	rclcpp::shutdown();
	return 0;
}
