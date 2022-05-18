/**
 * Copyright (C) 2022 EXTEND3D and/or its subsidiary(-ies)
 * https://extend3d.com
 *
 * @author Chris Brammer
 */
#pragma once

#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVector>
#include <QTouchEvent>
#include <QPushButton>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/msg/camera_info.h>
#include <touch_interfaces/msg/touch_events.hpp>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <vector>
#include <mutex>
#include <unordered_map>

enum class TouchState
{
	NEW,
	MOVED,
	MISSING
};

struct TouchAggregation
{
	QPoint ptStart;
	QPoint ptEnd;
	int64_t id;
	TouchState state;
	bool isUp;
	bool isDown;
};

class Frame : public QFrame
{
	Q_OBJECT
public:
	Frame( rclcpp::Node::SharedPtr& node_handle, QWidget* parent = 0, Qt::WindowFlags f = Qt::WindowFlags() );

	bool openImage( const QString& fileName );
	bool saveImage( const QString& fileName, const char* fileFormat );

	bool isModified() const
	{
		return m_modified;
	}

protected:
	void paintEvent( QPaintEvent* event ) override;
	void resizeEvent( QResizeEvent* event ) override;
	bool event( QEvent* event ) override;

private slots:
	void clear();
	void onUpdate();
	void injectCurrentTouches();

private:
	void resizeImage( QImage* image, const QSize& newSize );

	void parameterEventCallback( const rcl_interfaces::msg::ParameterEvent::SharedPtr );
	void projectorInfoCallback( const sensor_msgs::msg::CameraInfo::ConstSharedPtr projectorCi );
	void touchPointsCallback( const touch_interfaces::msg::TouchEvents::ConstSharedPtr touchPoints );

	rclcpp::Node::SharedPtr m_nh;

	tf2_ros::Buffer m_tfBuffer;
	tf2_ros::TransformListener m_tfListener;

	std::unique_ptr< QTimer > m_updateTimer;
	std::unique_ptr< QTimer > m_touchInjectTimer;
	QImage m_image;
	QTouchDevice m_device;

	image_geometry::PinholeCameraModel m_projectorPinhole;
	std::unordered_map< int, std::vector< cv::Point2d > > m_smoothingMap;

	std::mutex m_touchMutex;
	std::vector< TouchAggregation > m_currentTouches;

	rclcpp::Subscription< rcl_interfaces::msg::ParameterEvent >::SharedPtr m_parameterSubscription;
	rclcpp::Subscription< sensor_msgs::msg::CameraInfo >::SharedPtr m_projectorInfoSubscription;
	rclcpp::Subscription< touch_interfaces::msg::TouchEvents >::SharedPtr m_touchPointSubscription;

	std::unique_ptr< QPushButton > m_clearButton;

	bool m_modified;
	QList< QColor > m_penColors;
};
