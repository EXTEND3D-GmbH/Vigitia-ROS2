/**
 * Copyright (C) 2022 EXTEND3D and/or its subsidiary(-ies)
 * https://extend3d.com
 *
 * @author Chris Brammer
 */
#pragma once

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <memory>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>

struct PixelHistory;
struct Background
{
	bool isNew;
	cv::Mat mean;
	cv::Mat stdDev;
};
class ThreadedDynamicBackground : public rclcpp::Node
{
public:
	ThreadedDynamicBackground( int width, int height );
	~ThreadedDynamicBackground();
	void start();
	void stop();
	void postNewImage( const sensor_msgs::msg::Image::ConstSharedPtr frame );
	Background getBackground();
	void setFixed( bool fixed );

private:
	void setBackgroundFix( const std::shared_ptr< std_srvs::srv::SetBool::Request > request,
						   std::shared_ptr< std_srvs::srv::SetBool::Response > response );
	void threadedDynamicBackground();

	std::mutex m_frameLock;
	std::condition_variable_any m_cv;
	sensor_msgs::msg::Image::ConstSharedPtr m_depthFrame;
	image_transport::ImageTransport m_imageTransport;
	image_transport::Publisher m_bgPublisher;
	rclcpp::Service< std_srvs::srv::SetBool >::SharedPtr m_backgroundService;
	std::thread m_worker;

	int m_width = 0;
	int m_height = 0;

	bool m_hasNewFrame = false;
	bool m_shutdownThread = false;

	unsigned long long m_lastFrameNumber = 0;

	std::atomic< bool > m_hasUpdate = false;
	std::atomic< bool > m_isRunning = true;
	std::atomic< bool > m_isFixed = false;

	std::unique_ptr< PixelHistory[] > m_pixelHistory;
	cv::Mat m_mean;
	cv::Mat m_stdDev;
	cv::Mat m_debug;

	cv::Mat m_stableMean;
	cv::Mat m_stableStdDev;
};