// License: Created by Robert Xiao on February 20, 2015. Adpated by Chris Brammer for his Master Thesis
// Source:
// https://github.com/nneonneo/direct-handtracking/blob/master/ofx/apps/handTracking/direct/src/BackgroundUpdaterThread.cpp

#include "ThreadedDynamicBackground.h"
#include "FixedQueue.h"
#include <cmath>

static const uint16_t MIN_DEPTH = 10;	  // minimum valid depth value (below this = invalid)
static const uint16_t MAX_DEPTH = 10000;  // maximum valid depth value
static const int HISTORY_LENGTH = 128;
static const int HIST_MIN = 30;

static const float INVALID_MEAN = 0;
static const float INVALID_STDEV = 1e6;

/* Heuristic threshold for z-increase destabilization.
 * When the current pixel value is further than [THRESHOLD] z-values from the stable mean,
 * the pixel will be marked unstable (reflecting the fact that the previous stable object must
 * have moved off that point). */
static const float HEUR_Z_INCREASE_THRESHOLD = 10; /* z-values */
/* Heuristic threshold for stability - a window stdev below this value will be considered stable. */
static const float HEUR_STABLE_FACTOR = 5; /* mm / m^2 */
/* Heuristic threshold for rejecting halos. Increases in the stable mean which are less than this threshold
 * will just be rejected. Halos are an effect caused by multipath interference, and manifest as depth values
 * which are greater than the true surface depth
 * in a radius around the halo-causing object (hovering over the surface). */
static const float HEUR_HALO_THRESHOLD = 1; /* mm */

/* Maximum standard deviation that is allowed for a pixel to be considered valid */
static const float STD_DEV_MAX = 10; /* mm */

struct PixelHistory
{
	fixedqueue< uint16_t, HISTORY_LENGTH > history;
	uint64_t sum = 0;
	uint64_t sumSquared = 0;
	float mean;
	float stdev;
	bool stable;

private:
	void remove_one()
	{
		if ( history.empty() ) return;
		uint64_t ival = history.pop();
		sum -= ival;
		sumSquared -= ival * ival;
	}

	void add_one( uint16_t val )
	{
		uint64_t ival = val;
		history.push( val );
		sum += ival;
		sumSquared += ival * ival;
	}

public:
	/* Update depth history (fast) */
	void update( uint16_t val )
	{
		if ( val < MIN_DEPTH || val > MAX_DEPTH ) {
			remove_one();
			return;
		}
		/* Valid depth value */
		if ( history.size() == HISTORY_LENGTH ) {
			remove_one();
		}
		add_one( val );
	}

	/* Update window statistics (slow; call me less frequently) */
	bool update_stats( float* stable_mean, float* stable_stdev, uint32_t* debugPixel )
	{
		size_t n = history.size();
		if ( n == 0 ) {
			mean = INVALID_MEAN;
			stdev = INVALID_STDEV;
			stable = false;
			return stable;
		}

		mean = sum / (float)n;
		stdev = static_cast< float >( std::sqrt( sumSquared * n - sum * sum ) / n );

		if ( mean > *stable_mean + *stable_stdev * HEUR_Z_INCREASE_THRESHOLD ) {
			/* Current value is further than the stable value: mark the current pixel as unstable */
			stable = false;
			*stable_mean = INVALID_MEAN;
			*stable_stdev = INVALID_STDEV;
			*debugPixel = 0xff0000ff;
		} else if ( n < HIST_MIN ) {
			stable = false;
			*debugPixel = 0xff000000;
		} else if ( stdev > STD_DEV_MAX ) {
			stable = false;
			*debugPixel = 0xffffff00;
		} else if ( stdev > HEUR_STABLE_FACTOR * std::pow( ( mean / 1000.0f ), 2 ) ) {
			stable = false;
			*debugPixel = 0xff00ff00;
		} else {
			stable = true;
			// if ( mean > *stable_mean + HEUR_HALO_THRESHOLD || mean < *stable_mean ) {
			*stable_mean = mean;
			*stable_stdev = stdev;
			//}
			*debugPixel = 0xffffffff;
		}
		return stable;
	}
};

ThreadedDynamicBackground::ThreadedDynamicBackground( int width, int height )
	: rclcpp::Node( "background_node" )
	, m_width( width )
	, m_height( height )
	, m_imageTransport( static_cast< rclcpp::Node::SharedPtr >( this ) )
{
	m_bgPublisher = m_imageTransport.advertise( "background/mean_raw", 1, true );
	// Alloc Mean, StdDev and HistoryPixels
	m_mean = cv::Mat( height, width, CV_32F, cv::Scalar( 0.0f ) );
	m_stdDev = cv::Mat( height, width, CV_32F, cv::Scalar( 0.0f ) );
	m_debug = cv::Mat( height, width, CV_8UC4, cv::Scalar( 0 ) );
	m_pixelHistory = std::make_unique< PixelHistory[] >( height * width );
	m_backgroundService
		= create_service< std_srvs::srv::SetBool >( "background_node/set_fix_background",
													std::bind( &ThreadedDynamicBackground::setBackgroundFix,
															   this,
															   std::placeholders::_1,
															   std::placeholders::_2 ) );

	// Start worker
	m_worker = std::thread( &ThreadedDynamicBackground::threadedDynamicBackground, this );
}

ThreadedDynamicBackground::~ThreadedDynamicBackground()
{
	RCLCPP_INFO( get_logger(), "Destroying Background" );
	{
		std::unique_lock lock( m_frameLock );
		m_isRunning = false;
		m_shutdownThread = true;
	}
	m_worker.join();
}

void ThreadedDynamicBackground::start()
{
	m_isRunning = true;
}

void ThreadedDynamicBackground::stop()
{
	m_isRunning = false;
}

void ThreadedDynamicBackground::postNewImage( const sensor_msgs::msg::Image::ConstSharedPtr frame )
{
	std::unique_lock lock( m_frameLock );
	if ( !m_isRunning ) return;

	m_depthFrame = std::move( frame );
	m_hasNewFrame = true;
	m_cv.notify_all();
}

Background ThreadedDynamicBackground::getBackground()
{
	std::unique_lock lock( m_frameLock );
	Background bg;
	bg.isNew = m_hasNewFrame;
	bg.mean = m_stableMean;
	bg.stdDev = m_stableStdDev;

	m_hasNewFrame = false;
	return bg;
}

void ThreadedDynamicBackground::setFixed( bool fixed )
{
	m_isFixed = fixed;
}

void ThreadedDynamicBackground::setBackgroundFix(
	const std::shared_ptr< std_srvs::srv::SetBool::Request > request,
	std::shared_ptr< std_srvs::srv::SetBool::Response > response )
{
	setFixed( request->data );
	response->success = true;
	response->message = m_isFixed ? "Background fixed." : "Background not fixed.";
}

void ThreadedDynamicBackground::threadedDynamicBackground()
{
	while ( !m_shutdownThread ) {
		if ( !rclcpp::ok() ) {
			break;
		}

		rclcpp::spin_some( shared_from_this() );

		std::unique_lock lock( m_frameLock );
		while ( !m_hasNewFrame ) {
			if ( m_shutdownThread ) return;
			m_cv.wait_for( lock, std::chrono::milliseconds( 100 ) );
		}

		m_hasNewFrame = false;

		if ( m_isFixed ) continue;

		// Got Frames
		auto depthFrame = m_depthFrame;
		lock.unlock();
		{
			const uint16_t* depth = reinterpret_cast< const uint16_t* >( depthFrame->data.data() );

			for ( size_t i = 0; i < m_width * m_height; i++ ) {
				m_pixelHistory[ i ].update( depth[ i ] );
			}
		}

		// Update stats
		// Wait at least 4 frames to slow this down
		static int frameNumber = 0;
		frameNumber++;
		if ( frameNumber - m_lastFrameNumber < 4 ) {
			continue;
		}
		m_lastFrameNumber = frameNumber;
		{
			float* mean = reinterpret_cast< float* >( m_mean.data );
			float* stdDev = reinterpret_cast< float* >( m_stdDev.data );
			uint32_t* debugPx = reinterpret_cast< uint32_t* >( m_debug.data );
			for ( size_t i = 0; i < m_width * m_height; i++ ) {
				m_pixelHistory[ i ].update_stats( &mean[ i ], &stdDev[ i ], &debugPx[ i ] );
			}
		}

		lock.lock();
		if ( count_subscribers( "background/mean_raw" ) > 0 ) {
			sensor_msgs::msg::Image::SharedPtr bg_published( new sensor_msgs::msg::Image() );
			cv::Mat toSend;
			cv::normalize( m_mean, toSend, 0, 255, cv::NORM_MINMAX, CV_8U );
			cv::applyColorMap( toSend, toSend, cv::COLORMAP_JET );

			bg_published->header.frame_id = "depth_camera_link";
			bg_published->header.set__stamp( get_clock()->now() );
			bg_published->height = m_height;
			bg_published->width = m_width;
			bg_published->encoding = "8UC3";
			bg_published->is_bigendian = false;
			bg_published->step = m_width * 3;
			if ( toSend.isContinuous() ) {
				bg_published->data.assign( toSend.data, toSend.data + toSend.total() * toSend.channels() );
			} else {
				for ( int i = 0; i < toSend.rows; ++i ) {
					bg_published->data.insert( bg_published->data.end(),
											   toSend.ptr< uint8_t >( i ),
											   toSend.ptr< uint8_t >( i ) + toSend.cols * toSend.channels() );
				}
			}

			m_bgPublisher.publish( bg_published );
		}
		m_stableMean = m_mean.clone();
		m_stableStdDev = m_stdDev.clone();
		m_hasUpdate = true;
	}
}
