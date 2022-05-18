/**
 * Copyright (C) 2022 EXTEND3D and/or its subsidiary(-ies)
 * https://extend3d.com
 *
 * @author Chris Brammer
 */
#include "degginger_draw/frame.h"
#include "degginger_draw/bezier.h"

#include <QApplication>
#include <QPointF>
#include <QScreen>
#include <QDesktopWidget>
#include <QPainterPath>

#include <cstdlib>
#include <ctime>
#include <windows.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static const double MinimumDiameter = 3.0;
static const double MaximumDiameter = 15.0;

namespace tf2 {
/******************/
/** TouchEvents **/
/******************/

/** \brief Extract a timestamp from the header of a Point message.
 * This function is a specialization of the getTimestamp template defined in tf2/convert.h.
 * \param t PointStamped message to extract the timestamp from.
 * \return The timestamp of the message.
 */
template <>
inline tf2::TimePoint getTimestamp( const touch_interfaces::msg::TouchEvents& t )
{
	return tf2_ros::fromMsg( t.header.stamp );
}

/** \brief Extract a frame ID from the header of a Point message.
 * This function is a specialization of the getFrameId template defined in tf2/convert.h.
 * \param t PointStamped message to extract the frame ID from.
 * \return A string containing the frame ID of the message.
 */
template <>
inline std::string getFrameId( const touch_interfaces::msg::TouchEvents& t )
{
	return t.header.frame_id;
}

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Point type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The point to transform, as a timestamped Point3 message.
 * \param t_out The transformed point, as a timestamped Point3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
void doTransform( const touch_interfaces::msg::TouchEvents& t_in,
				  touch_interfaces::msg::TouchEvents& t_out,
				  const geometry_msgs::msg::TransformStamped& transform )
{
	KDL::Frame frame = gmTransformToKDL( transform );
	for ( const auto& tp : t_in.events ) {
		KDL::Vector v_out = frame * KDL::Vector( tp.point.x, tp.point.y, tp.point.z );
		auto& out = t_out.events.emplace_back();
		out.id = tp.id;
		out.point.x = v_out[ 0 ];
		out.point.y = v_out[ 1 ];
		out.point.z = v_out[ 2 ];
	}

	t_out.header.stamp = transform.header.stamp;
	t_out.header.frame_id = transform.header.frame_id;
}

/** \brief Trivial "conversion" function for Point message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A PointStamped message.
 * \return The input argument.
 */
inline touch_interfaces::msg::TouchEvents toMsg( const touch_interfaces::msg::TouchEvents& in )
{
	return in;
}

/** \brief Trivial "conversion" function for Point message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param msg A PointStamped message.
 * \param out The input argument.
 */
inline void fromMsg( const touch_interfaces::msg::TouchEvents& msg, touch_interfaces::msg::TouchEvents& out )
{
	out = msg;
}
}  // namespace tf2

Frame::Frame( rclcpp::Node::SharedPtr& node_handle, QWidget* parent, Qt::WindowFlags f )
	: QFrame( parent, f )
	, m_tfBuffer( node_handle->get_clock() )
	, m_tfListener( m_tfBuffer )
	, m_image( 500, 500, QImage::Format_ARGB32 )
{
	if ( !InitializeTouchInjection( 256, TOUCH_FEEDBACK_DEFAULT ) ) {
		auto error = GetLastError();
		RCLCPP_INFO_STREAM( m_nh->get_logger(), "InitError " << error );
	}

	setFixedSize( 1920, 1080 );
	setWindowTitle( "Degginger Draw" );
	setAttribute( Qt::WA_AcceptTouchEvents );
	setAttribute( Qt::WA_StaticContents );
	m_modified = false;

	m_penColors << QColor( "green" ) << QColor( "purple" ) << QColor( "red" ) << QColor( "blue" )
				<< QColor( "yellow" ) << QColor( "pink" ) << QColor( "orange" ) << QColor( "brown" )
				<< QColor( "grey" ) << QColor( "black" );

	srand( time( NULL ) );

	m_device.setName( "ROS2 Touch Device" );
	m_device.setType( QTouchDevice::DeviceType::TouchScreen );
	m_device.setCapabilities( QTouchDevice::CapabilityFlag::Position
							  | QTouchDevice::CapabilityFlag::MouseEmulation );

	m_updateTimer = std::make_unique< QTimer >( this );
	m_updateTimer->setInterval( 16 );
	m_updateTimer->start();

	m_touchInjectTimer = std::make_unique< QTimer >( this );
	m_touchInjectTimer->setInterval( 10 );
	m_touchInjectTimer->start();

	connect( m_updateTimer.get(), SIGNAL( timeout() ), this, SLOT( onUpdate() ) );
	connect( m_touchInjectTimer.get(), SIGNAL( timeout() ), this, SLOT( injectCurrentTouches() ) );

	m_nh = node_handle;

	clear();

	rclcpp::QoS qos( rclcpp::KeepLast( 100 ), rmw_qos_profile_sensor_data );
	m_parameterSubscription = m_nh->create_subscription< rcl_interfaces::msg::ParameterEvent >(
		"/parameter_events", qos, std::bind( &Frame::parameterEventCallback, this, std::placeholders::_1 ) );

	m_projectorInfoSubscription = m_nh->create_subscription< sensor_msgs::msg::CameraInfo >(
		"/projector/camera_info",
		qos,
		std::bind( &Frame::projectorInfoCallback, this, std::placeholders::_1 ) );

	m_touchPointSubscription = m_nh->create_subscription< touch_interfaces::msg::TouchEvents >(
		"/touch_points", qos, std::bind( &Frame::touchPointsCallback, this, std::placeholders::_1 ) );

	m_clearButton = std::make_unique< QPushButton >( this );
	m_clearButton->setGeometry( 10, 10, 200, 50 );
	m_clearButton->setText( "Clear" );
	QObject::connect( m_clearButton.get(), SIGNAL( clicked() ), this, SLOT( clear() ) );
}

bool Frame::openImage( const QString& fileName )
{
	QImage loadedImage;
	if ( !loadedImage.load( fileName ) ) return false;

	QSize newSize = loadedImage.size().expandedTo( size() );
	resizeImage( &loadedImage, newSize );
	m_image = loadedImage;
	m_modified = false;
	update();
	return true;
}

bool Frame::saveImage( const QString& fileName, const char* fileFormat )
{
	QImage visibleImage = m_image;
	resizeImage( &visibleImage, size() );

	if ( visibleImage.save( fileName, fileFormat ) ) {
		m_modified = false;
		return true;
	} else {
		return false;
	}
}

void Frame::paintEvent( QPaintEvent* event )
{
	QPainter painter( this );
	const QRect rect = event->rect();
	painter.drawImage( rect.topLeft(), m_image, rect );
}

void Frame::resizeEvent( QResizeEvent* event )
{
	if ( width() > m_image.width() || height() > m_image.height() ) {
		int newWidth = qMax( width() + 128, m_image.width() );
		int newHeight = qMax( height() + 128, m_image.height() );
		resizeImage( &m_image, QSize( newWidth, newHeight ) );
		update();
	}
	QWidget::resizeEvent( event );
}

bool Frame::event( QEvent* event )
{
	switch ( event->type() ) {
		case QEvent::TouchBegin:
		case QEvent::TouchUpdate:
		case QEvent::TouchEnd: {
			const QTouchEvent* touch = static_cast< QTouchEvent* >( event );
			const QList< QTouchEvent::TouchPoint > touchPoints
				= static_cast< QTouchEvent* >( event )->touchPoints();
			for ( const QTouchEvent::TouchPoint& touchPoint : touchPoints ) {
				switch ( touchPoint.state() ) {
					case Qt::TouchPointStationary:
						// don't do anything if this touch point hasn't moved or has been released
						break;
					case Qt::TouchPointReleased: {
						// Draw gathered spline
						auto points = m_smoothingMap[ touchPoint.id() ];
						m_smoothingMap.erase( touchPoint.id() );
						points = RamerDouglasPeucker( points, 1.0 );
						auto size = points.size();
						if ( size > 3 ) {
							auto curve = FitCurve( points, 0, points.size() - 1, 4.0 );

							QPainterPath path;
							for ( auto& c : curve ) {
								path.moveTo( { c.pt1.x, c.pt1.y } );
								path.cubicTo( { c.c1.x, c.c1.y }, { c.c2.x, c.c2.y }, { c.pt2.x, c.pt2.y } );
							}

							QPainter painter( &m_image );
							QPen pen( QColor( "red" ), 5.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin );
							painter.setPen( pen );
							painter.setBrush( m_penColors.at( touchPoint.id() % m_penColors.count() ) );
							painter.strokePath( path, pen );
							painter.end();
							update();
						}

						break;
					}
					default: {
						QSizeF diams = touchPoint.ellipseDiameters();
						if ( diams.isEmpty() ) {
							qreal diameter = MaximumDiameter;
							if ( touch->device()->capabilities() & QTouchDevice::Pressure )
								diameter = MinimumDiameter
										   + ( MaximumDiameter - MinimumDiameter ) * touchPoint.pressure();
							diams = QSizeF( diameter, diameter );
						}

						QPointF pos = touchPoint.pos();

						QPainter painter( &m_image );
						painter.setPen( Qt::NoPen );
						painter.setBrush( m_penColors.at( touchPoint.id() % m_penColors.count() ) );
						painter.drawEllipse( pos, diams.width() / 2, diams.height() / 2 );
						painter.end();

						m_modified = true;
						const int rad = 2;
						QRectF rect( QPointF(), diams );
						rect.moveCenter( pos );
						update( rect.toRect().adjusted( -rad, -rad, +rad, +rad ) );

						m_smoothingMap[ touchPoint.id() ].push_back( { pos.x(), pos.y() } );
						cv::Point2d current( pos.x(), pos.y() );
#if 1
						m_smoothingMap[ touchPoint.id() ].push_back( current );
#else
						if ( !m_smoothingMap[ touchPoint.id() ].empty() ) {
							m_smoothingMap[ touchPoint.id() ].push_back( current );
						} else {
							cv::Point2d last = m_smoothingMap[ touchPoint.id() ].back();
							// Only add points that are significantly far from each other
							if ( cv::norm( last - current ) > 20 ) {
								m_smoothingMap[ touchPoint.id() ].push_back( current );
							}
						}
#endif
					} break;
				}
			}
			break;
		}
		default:
			return QFrame::event( event );
	}
	return true;
}
void Frame::clear()
{
	// make all pixels fully transparent
	m_image.fill( qRgba( 255, 255, 255, 0 ) );
	m_modified = true;
	update();
}

void Frame::onUpdate()
{
	if ( !rclcpp::ok() ) {
		close();
		return;
	}

	rclcpp::spin_some( m_nh );
}

void Frame::resizeImage( QImage* image, const QSize& newSize )
{
	if ( image->size() == newSize ) return;

	QImage newImage( newSize, QImage::Format_RGB32 );
	newImage.fill( qRgb( 255, 255, 255 ) );
	QPainter painter( &newImage );
	painter.drawImage( QPoint( 0, 0 ), *image );
	*image = newImage;
}

void Frame::parameterEventCallback( const rcl_interfaces::msg::ParameterEvent::SharedPtr event )
{
	// only consider events from this node
	if ( event->node == m_nh->get_fully_qualified_name() ) {
		// since parameter events for this even aren't expected frequently just always call update()
		update();
	}
}

void Frame::projectorInfoCallback( const sensor_msgs::msg::CameraInfo::ConstSharedPtr projectorCi )
{
	m_projectorPinhole.fromCameraInfo( projectorCi );
}

void Frame::touchPointsCallback( const touch_interfaces::msg::TouchEvents::ConstSharedPtr touchPoints )
{
	std::scoped_lock lock( m_touchMutex );

	m_currentTouches.erase(
		std::remove_if( m_currentTouches.begin(),
						m_currentTouches.end(),
						[]( const TouchAggregation& touch ) { return touch.state == TouchState::MISSING; } ),
		m_currentTouches.end() );

	for ( auto& pt : m_currentTouches ) {
		pt.state = TouchState::MISSING;
	}

	if ( !m_tfBuffer.canTransform( "projector_link", "depth_camera_link", m_nh->get_clock()->now() ) ) {
		return;
	}

	auto touchPointsProjector = m_tfBuffer.transform( *touchPoints, "projector_link" );

	for ( const auto& tp : touchPointsProjector.events ) {
		const int64_t id = tp.id;

		cv::Point3d pt( tp.point.x, tp.point.y, tp.point.z );
		cv::Point2d pixel = m_projectorPinhole.project3dToPixel( pt );

		// Adjust pixel position to correct screen
		auto geom = screen()->geometry();
		pixel.x += geom.left();
		pixel.y += geom.top();

		auto findResult = std::find_if( m_currentTouches.begin(),
										m_currentTouches.end(),
										[ id ]( const TouchAggregation& touch ) { return touch.id == id; } );

		if ( findResult == m_currentTouches.end() ) {
			// new
			m_currentTouches.push_back( { QPoint( pixel.x, pixel.y ),
										  QPoint( pixel.x, pixel.y ),
										  id,
										  TouchState::NEW,
										  false,
										  false } );
		} else {
			// Old
			( *findResult ).id = id;
			( *findResult ).ptStart = ( *findResult ).ptEnd;
			( *findResult ).ptEnd = QPoint( pixel.x, pixel.y );
			( *findResult ).state = TouchState::MOVED;
		}
	}
}

void Frame::injectCurrentTouches()
{
	std::scoped_lock lock( m_touchMutex );
	std::vector< POINTER_TOUCH_INFO > contacts;
	for ( auto& tp : m_currentTouches ) {
		auto rect = QApplication::desktop()->geometry();
		if ( tp.ptEnd.x() < rect.left() || tp.ptEnd.x() > rect.right() ) {
			continue;
		}
		if ( tp.ptEnd.y() < rect.top() || tp.ptEnd.y() > rect.bottom() ) {
			continue;
		}

		POINTER_TOUCH_INFO contact;
		memset( &contact, 0, sizeof( POINTER_TOUCH_INFO ) );

		contact.pointerInfo.pointerType = PT_TOUCH;	  // we're sending touch input
		contact.pointerInfo.pointerId = tp.id % 256;  // contact 0
		contact.pointerInfo.ptPixelLocation.x = tp.ptEnd.x();
		contact.pointerInfo.ptPixelLocation.y = tp.ptEnd.y();
		if ( !tp.isDown && ( tp.state == TouchState::NEW || tp.state == TouchState::MOVED ) ) {
			contact.pointerInfo.pointerFlags
				= POINTER_FLAG_INRANGE | POINTER_FLAG_INCONTACT | POINTER_FLAG_DOWN;
			RCLCPP_INFO_STREAM(
				m_nh->get_logger(),
				"Sending Down for Id " << tp.id << " x: " << tp.ptEnd.x() << " y: " << tp.ptEnd.y() );
			tp.state = TouchState::MOVED;
			tp.isDown = true;
		} else if ( tp.state == TouchState::MOVED ) {
			contact.pointerInfo.pointerFlags
				= POINTER_FLAG_INRANGE | POINTER_FLAG_INCONTACT | POINTER_FLAG_UPDATE;
			RCLCPP_INFO_STREAM(
				m_nh->get_logger(),
				"Sending Update for Id " << tp.id << " x: " << tp.ptEnd.x() << " y: " << tp.ptEnd.y() );
		} else if ( !tp.isUp && tp.state == TouchState::MISSING ) {
			contact.pointerInfo.pointerFlags = POINTER_FLAG_UP;
			RCLCPP_INFO_STREAM(
				m_nh->get_logger(),
				"Sending Up for Id " << tp.id << " x: " << tp.ptEnd.x() << " y: " << tp.ptEnd.y() );
			tp.isUp = true;
		} else {
			continue;
		}
		contact.touchFlags = TOUCH_FLAG_NONE;
		contact.touchMask = TOUCH_MASK_CONTACTAREA | TOUCH_MASK_PRESSURE;
		contact.orientation = 0;
		contact.pressure = 1024;

		contact.rcContact.top = contact.pointerInfo.ptPixelLocation.y - 2;
		contact.rcContact.bottom = contact.pointerInfo.ptPixelLocation.y + 2;
		contact.rcContact.left = contact.pointerInfo.ptPixelLocation.x - 2;
		contact.rcContact.right = contact.pointerInfo.ptPixelLocation.x + 2;

		contacts.push_back( contact );
	}

	if ( !contacts.empty() ) {
		if ( !InjectTouchInput( contacts.size(), contacts.data() ) ) {
			auto error = GetLastError();
			RCLCPP_INFO_STREAM( m_nh->get_logger(), "Error" << error );
		}
	}
}
