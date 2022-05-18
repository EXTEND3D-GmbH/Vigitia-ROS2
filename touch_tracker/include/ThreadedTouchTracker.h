/**
 * Copyright (C) 2022 EXTEND3D and/or its subsidiary(-ies)
 * https://extend3d.com
 *
 * @author Chris Brammer
 */
#pragma once
#include "ThreadedDynamicBackground.h"
#include "Direct.h"

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.h>
#include <touch_interfaces/msg/touch_events.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <image_geometry/pinhole_camera_model.h>

#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

class ThreadedTouchTracker : public rclcpp::Node
{
public:
	ThreadedTouchTracker( int width, int height, ThreadedDynamicBackground* bg );
	~ThreadedTouchTracker();

	void start();
	void stop();
	void postNewImage( const sensor_msgs::msg::Image::ConstSharedPtr depth,
					   const sensor_msgs::msg::Image::ConstSharedPtr ir,
					   const sensor_msgs::msg::CameraInfo::ConstSharedPtr depthCI );

	std::vector< FingerTouch > getLastTouches();

private:
	void threadedTouchTracker();

	ThreadedDynamicBackground* m_bg;
	std::thread m_worker;
	bool m_shutdownThread = false;

	std::atomic< int > m_previewType = 0;
	std::atomic< bool > m_hasUpdate = false;
	std::atomic< bool > m_isRunning = true;

	std::mutex m_touchLock;
	std::mutex m_frameLock;
	std::condition_variable_any m_cv;

	IRDepthTouchTracker m_tracker;

	image_geometry::PinholeCameraModel m_depthPinhole;

	rclcpp::Publisher< touch_interfaces::msg::TouchEvents >::SharedPtr m_publisher;
	rclcpp::Publisher< geometry_msgs::msg::PointStamped >::SharedPtr m_publisherGeom;
	sensor_msgs::msg::CameraInfo::ConstSharedPtr m_depthCI;
	sensor_msgs::msg::Image::ConstSharedPtr m_depthFrame;
	sensor_msgs::msg::Image::ConstSharedPtr m_irFrame;
	sensor_msgs::msg::Image::ConstSharedPtr m_currentDepthFrame;
	sensor_msgs::msg::Image::ConstSharedPtr m_currentIrFrame;
	cv::Mat m_mean;
	cv::Mat m_stdDev;

	bool m_hasNewFrame = false;

	std::vector< FingerTouch > m_lastTouches;
};