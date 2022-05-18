/**
 * Copyright (C) 2022 EXTEND3D and/or its subsidiary(-ies)
 * https://extend3d.com
 *
 * @author Chris Brammer
 */
#pragma once
#include "Settings.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

/// Filtered touch point
struct FingerTouch
{
	/// ID of the touch
	int id;

	/// Coordinates of the fingertip
	cv::Point2f tip;

	/// Coordinates of the finger base (approximate)
	cv::Point2f base;

	/// Is this touch currently contacting the surface?
	bool touched;

	/// Number of frames since the touch state changed
	int statusAge;

	/// Number of frames since the touch first appeared
	int touchAge;

	FingerTouch()
		: id( -1 )
		, tip()
		, touched( false )
		, statusAge( 0 )
		, touchAge( 0 )
		, touchZ( 0 )
		, missing( false )
		, missingAge( 0 )
	{}

public:
	/* n.b. these should not be accessed by classes besides the touch trackers */
	/// Touch Z height
	float touchZ;
	float touchZAbsolute;

	/// Did this touch recently go missing?
	bool missing;

	/// Number of frames since the touch went missing.
	int missingAge;
};

struct IRDepthTip
{
	std::vector< unsigned > pixels;
	std::vector< unsigned > roots;	// pixels next to midconf/highconf pixels
};

struct IRDepthFinger
{
	float x, y, z;
};

struct IRDepthHand
{
	std::vector< IRDepthFinger > fingers;
};

struct IRDepthArm
{
	std::vector< IRDepthHand > hands;
};

class IRDepthTouchTracker
{
protected:
	/* Edgemap construction */
	void buildEdgeImage();
	// Fill holes in the irCanny image
	void fillIrCannyHoles();

	/* Touch tracking stages */
	void buildDiffImage();

	void rejectBlob( const std::vector< unsigned >& blob, int reason = 0 );

	int nextBlobId;
	std::vector< IRDepthArm > detectTouches();
	bool floodArm( IRDepthArm& arm, unsigned idx );
	bool floodHand( IRDepthHand& hand, unsigned idx );
	bool floodFinger( IRDepthFinger& finger, unsigned idx );
	bool floodTip( IRDepthTip& tip, unsigned idx );
	void refloodFinger( const std::vector< unsigned >& blob, std::vector< unsigned >& roots );
	bool computeFingerMetrics( IRDepthFinger& finger, std::vector< unsigned >& px );

private:
	std::vector< FingerTouch > mergeTouches( std::vector< FingerTouch >& curTouches,
											 std::vector< FingerTouch >& newTouches );

	uint64_t m_nextTouchId = 0;

	int w;
	int h;
	std::unique_ptr< uint32_t[] > diffIm;  // depth difference image; A=valid B=zone [0=noise/negative
										   // 64=close 128=medium 192=far] GR=diff
	std::unique_ptr< uint32_t[] > edgeIm;  // edge image; B=IRedge G=depthedge R=depthabs
	std::unique_ptr< uint32_t[] > blobIm;  // blob image; B=flags G=blobidx R=dist
	std::unique_ptr< uint8_t[] > irCanny;  // temporary image for canny purposes

	cv::Mat m_depth;
	cv::Mat m_mean;
	cv::Mat m_stdDev;
	cv::Mat m_ir;

	std::vector< FingerTouch > m_touches;

	TouchTrackerSettings m_settings;

public:
	IRDepthTouchTracker( int width, int height, TouchTrackerSettings settings = {} );
	std::vector< FingerTouch > run( cv::Mat depth, cv::Mat mean, cv::Mat stdDev, cv::Mat ir );
	cv::Mat getDiff();
	cv::Mat getEdge();
	cv::Mat getBlob();
	TouchTrackerSettings& touchSettings();
};