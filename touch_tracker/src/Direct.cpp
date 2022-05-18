//
//  IRDepthTouchTracker.cpp
//  Infrared + Depth sensor-fusion touch tracker.
//
//  Created by Robert Xiao on April 8, 2015.
//

#include "Direct.h"

#include <queue>
#include <set>

/* diffPx operators and definitions */
#define ZONE( x ) ( (x)&0xffff0000 )
#define DIFF( x ) ( (x)&0x0000ffff )
#define ZONE_ERROR 0x00000000
#define ZONE_NOISE 0xff000000
#define ZONE_LOW 0xff400000
#define ZONE_MID 0xff800000
#define ZONE_HIGH 0xffc00000

#define BLOB_REJECTED 0x00020000
#define BLOB_VISITED 0x00010000

static bool USE_GAUSSIAN_BLUR = false;
static int GAUSS_SIZE = 7;
static float GAUSS_SIGMA = 1.f;

static bool USE_QUANTIZATION = false;
static int QUANTIZATION_VALUE = 8;

int CANNY_LOW = 4000;
int CANNY_HIGH = 8000;

void fill_n( uint32_t* pt, int size, uint32_t value )
{
	for ( size_t i = 0; i < size; i++ ) {
		*pt++ = value;
	}
}

#pragma region Edge Map
void IRDepthTouchTracker::buildEdgeImage()
{
	const int n = w * h;

	uint32_t* diffPx = (uint32_t*)diffIm.get();
	uint32_t* edgePx = (uint32_t*)edgeIm.get();
	fill_n( edgePx, n, 0 );

	/* Build IR canny map */
	uint16_t* irPx = (uint16_t*)m_ir.data;
	uint8_t* ircannyPx = irCanny.get();

	for ( int i = 0; i < n; i++ ) {
		ircannyPx[ i ] = static_cast< uint8_t >( irPx[ i ] / 16 );
	}

	cv::Mat irCannyMat( cv::Size( w, h ), CV_8U, irCanny.get() );

	if ( USE_GAUSSIAN_BLUR ) {
		cv::GaussianBlur( irCannyMat, irCannyMat, cv::Size( GAUSS_SIZE, GAUSS_SIZE ), GAUSS_SIGMA );
	}
	if ( USE_QUANTIZATION ) {
		irCannyMat = ( irCannyMat / QUANTIZATION_VALUE );
	}

	// irCannyMat = ( irCannyMat / 16 );
	/* Edge finding, lightly tuned parameters */

	cv::Canny( irCannyMat, irCannyMat, CANNY_LOW, CANNY_HIGH, 7, true );

	/* Mark significant pixels (IR pixels that will be holefilled). */
	/* Currently, all pixels are considered significant. */
	for ( int i = 0; i < n; i++ ) {
		if ( ircannyPx[ i ] == 255 ) ircannyPx[ i ] = 224;	// significant value
	}

	fillIrCannyHoles();

	/* Build final edge map */

	for ( int i = 0; i < n; i++ ) {
		// Take a 2-3 pixel border and mark it as edge to avoid detection errors
		int y = i / w;
		int x = i % w;
		if ( x < 2 || x > w - 2 || y < 2 || y > h - 2 ) {
			edgePx[ i ] = 0xffffffff;
		} else if ( ircannyPx[ i ] ) {
			edgePx[ i ] |= 0xff000000 | ( ircannyPx[ i ] << 16 );
		}
	}

	/* Depth relative edges */
	{
		const int WIN = m_settings.edge_depthrel_dist;	// flatness check window size
		for ( int i = w * WIN; i < n - w * WIN; i++ ) {
			int myval = DIFF( diffPx[ i ] );
			for ( int dy = -1; dy <= 1; dy++ ) {
				for ( int dx = -1; dx <= 1; dx++ ) {
					int diffval = diffPx[ i + dx * WIN + dy * WIN * w ];
					// Reject if the other pixel differs greatly in diff value
					const auto diff = DIFF( diffval );
					if ( abs( myval - diff ) > m_settings.edge_depthrel_thresh ) {
						edgePx[ i ] |= 0xff00ff00;
					}
					if ( diff > m_settings.edge_depthabs_thresh ) {
						edgePx[ i ] |= 0xff0000ff;
					}
				}
			}
		}
	}
}

void IRDepthTouchTracker::fillIrCannyHoles()
{
	const int n = w * h;
	/* 255 = insignificant canny
	   224 = unvisited significant canny
	   208 = seen, unvisited significant canny
	   192 = visited significant canny
	   160 = fill candidate
	   128 = filled significant canny
	   0 = no canny */
	uint8_t* ircannypx = irCanny.get();

	/* Find significant pixels and fill outwards */
	std::queue< int > queue;

	for ( int idx = 0; idx < n; idx++ ) {
		if ( ircannypx[ idx ] != 224 ) continue;

		queue.push( idx );

		ircannypx[ idx ] = 208;
		while ( !queue.empty() ) {
			int curidx = queue.front();
			queue.pop();
			int curpx = ircannypx[ curidx ];

			if ( curpx == 208 ) {
				ircannypx[ curidx ] = 192;
			}

			int y = curidx / w;
			int x = curidx % w;
			int found = 0;

			// Find unvisited significant neighbours
#define TEST( dx, dy )                                                                               \
	do {                                                                                             \
		int xx = x + dx, yy = y + dy;                                                                \
		if ( 0 <= xx && xx < w && 0 <= yy && yy < h ) {                                              \
			int otheridx = curidx + dy * w + dx;                                                     \
			if ( ircannypx[ otheridx ] <= 192 ) continue; /* must be unvisited, real canny pixels */ \
			found++;                                                                                 \
			if ( ircannypx[ otheridx ] != 224 ) continue; /* already visited, or not significant */  \
			queue.push( otheridx );                                                                  \
			ircannypx[ otheridx ] = 208;                                                             \
		}                                                                                            \
	} while ( 0 )
			/* Eight-way neighbours, to cross diagonals */
			TEST( -1, -1 );
			TEST( -1, 0 );
			TEST( -1, 1 );
			TEST( 0, -1 );
			TEST( 0, 1 );
			TEST( 1, -1 );
			TEST( 1, 0 );
			TEST( 1, 1 );
#undef TEST

			if ( curpx == 160 ) {
				/* fill candidate */
				if ( found ) {
					ircannypx[ curidx ] = 128;
				} else {
					ircannypx[ curidx ] = 0;
				}
			} else if ( !found ) {
				/* Mark all neighbours as fill candidates */
#define TEST( dx, dy )                                                            \
	do {                                                                          \
		int xx = x + dx, yy = y + dy;                                             \
		if ( 0 <= xx && xx < w && 0 <= yy && yy < h ) {                           \
			int otheridx = curidx + dy * w + dx;                                  \
			if ( ircannypx[ otheridx ] != 0 ) continue; /* must be blank pixel */ \
			queue.push( otheridx );                                               \
			ircannypx[ otheridx ] = 160;                                          \
		}                                                                         \
	} while ( 0 )
				/* Eight-way neighbours, to cross diagonals */
				TEST( -1, -1 );
				TEST( -1, 0 );
				TEST( -1, 1 );
				TEST( 0, -1 );
				TEST( 0, 1 );
				TEST( 1, -1 );
				TEST( 1, 0 );
				TEST( 1, 1 );
#undef TEST
			}
		}
	}
}
#pragma endregion

void IRDepthTouchTracker::buildDiffImage()
{
	const int n = w * h;

	uint16_t* depthPx = (uint16_t*)m_depth.data;
	uint32_t* diffPx = (uint32_t*)diffIm.get();

	const float* bgmean = (float*)m_mean.data;
	const float* bgstdev = (float*)m_stdDev.data;

	/* Update diff image */
	for ( int i = 0; i < n; i++ ) {
		/* Update diff image */
		float diff;
		float z;
		if ( depthPx[ i ] ) {
			diff = bgmean[ i ] - depthPx[ i ];
			z = diff / bgstdev[ i ];
		} else {
			diff = 0;
			z = 0;
		}
		// A=valid B=zone GR=diff
		if ( bgmean[ i ] == 0 || diff < m_settings.error_cond )
			diffPx[ i ] = ZONE_ERROR;
		else if ( z < m_settings.noise_cond )
			diffPx[ i ] = ZONE_NOISE | (uint16_t)abs( diff );
		else if ( diff < m_settings.low_cond )
			diffPx[ i ] = ZONE_LOW | (uint16_t)diff;
		else if ( diff < m_settings.mid_cond )
			diffPx[ i ] = ZONE_MID | (uint16_t)diff;
		else
			diffPx[ i ] = ZONE_HIGH | (uint16_t)diff;
	}
}

#pragma region Flood Filling
void IRDepthTouchTracker::rejectBlob( const std::vector< unsigned >& blob, int reason )
{
	uint32_t* blobPx = (uint32_t*)blobIm.get();

	for ( auto i : blob ) {
		blobPx[ i & 0xffffff ] |= BLOB_REJECTED | ( ( reason & 0x3f ) << 18 );
	}
}

static int colorForBlobIndex( int blobId )
{
	/* Reverse the bits of the blob ID to make adjacent blob IDs more obvious */

	// https://graphics.stanford.edu/~seander/bithacks.html#ReverseByteWith64Bits
	unsigned char b = (unsigned char)blobId;
	b = static_cast< unsigned char >( ( ( b * 0x80200802ULL ) & 0x0884422110ULL ) * 0x0101010101ULL >> 32 );
	return b;
}

std::vector< IRDepthArm > IRDepthTouchTracker::detectTouches()
{
	const int n = w * h;

	nextBlobId = 1;

	std::vector< IRDepthArm > arms;
	uint32_t* diffPx = (uint32_t*)diffIm.get();
	uint32_t* blobPx = (uint32_t*)blobIm.get();

	fill_n( blobPx, n, 0 );

	for ( int i = 0; i < n; i++ ) {
		if ( ZONE( diffPx[ i ] ) != ZONE_HIGH ) continue;
		if ( blobPx[ i ] != 0 ) continue;

		IRDepthArm arm;
		if ( floodArm( arm, i ) ) {
			arms.push_back( arm );
		}
	}
	return arms;
}

bool IRDepthTouchTracker::floodArm( IRDepthArm& arm, unsigned idx )
{
	const int n = w * h;

	uint32_t* diffPx = (uint32_t*)diffIm.get();
	uint32_t* blobPx = (uint32_t*)blobIm.get();

	std::vector< unsigned > q, q2;
	int qtail = 0;

	q.push_back( idx );

	while ( qtail < q.size() ) {
		unsigned curidx = q[ qtail++ ];

		blobPx[ curidx ] |= ZONE_HIGH;

		int y = curidx / w;
		int x = curidx % w;

#define TEST( dx, dy )                                         \
	do {                                                       \
		int xx = x + dx, yy = y + dy;                          \
		if ( 0 <= xx && xx < w && 0 <= yy && yy < h ) {        \
			int otheridx = curidx + dy * w + dx;               \
			if ( blobPx[ otheridx ] != 0 ) continue;           \
			if ( ZONE( diffPx[ otheridx ] ) == ZONE_HIGH )     \
				q.push_back( otheridx );                       \
			else if ( ZONE( diffPx[ otheridx ] ) == ZONE_MID ) \
				q2.push_back( otheridx );                      \
			blobPx[ otheridx ] |= BLOB_VISITED;                \
		}                                                      \
	} while ( 0 )
		// four-way connectivity
		TEST( -1, 0 );
		TEST( 0, -1 );
		TEST( 0, 1 );
		TEST( 1, 0 );
#undef TEST
	}

	if ( q.size() < m_settings.arm_min_size ) {
		/* Not enough pixels */
		rejectBlob( q, 1 );
		return false;
	}

	/* Enough pixels for the arm: onto the next stage! */
	for ( auto i : q2 ) {
		blobPx[ i ] &= ~BLOB_VISITED;
	}

	bool found_hands = false;
	for ( auto i : q2 ) {
		if ( blobPx[ i ] != 0 ) continue;
		IRDepthHand hand;
		if ( floodHand( hand, i ) ) {
			arm.hands.push_back( hand );
			found_hands = true;
		}
	}

	if ( !found_hands ) {
		rejectBlob( q, 2 );
		return false;
	}

	int color = colorForBlobIndex( nextBlobId++ ) << 8;
	for ( auto i : q ) {
		blobPx[ i ] |= color;
	}
	return true;
}

bool IRDepthTouchTracker::floodHand( IRDepthHand& hand, unsigned idx )
{
	const int n = w * h;

	uint32_t* diffPx = (uint32_t*)diffIm.get();
	uint32_t* edgePx = (uint32_t*)edgeIm.get();
	uint32_t* blobPx = (uint32_t*)blobIm.get();

	std::vector< unsigned > q, q2;
	int qtail = 0;

	q.push_back( idx );

	while ( qtail < q.size() ) {
		unsigned curidx = q[ qtail++ ];

		blobPx[ curidx ] |= ZONE_MID;

		int y = curidx / w;
		int x = curidx % w;

#define TEST( dx, dy )                                                   \
	do {                                                                 \
		int xx = x + dx, yy = y + dy;                                    \
		if ( 0 <= xx && xx < w && 0 <= yy && yy < h ) {                  \
			int otheridx = curidx + dy * w + dx;                         \
			if ( blobPx[ otheridx ] != 0 ) continue;                     \
			if ( edgePx[ otheridx ] & 0x00ffff00 ) /* IR + depth edge */ \
				continue;                                                \
			if ( ZONE( diffPx[ otheridx ] ) >= ZONE_MID )                \
				q.push_back( otheridx );                                 \
			else if ( ZONE( diffPx[ otheridx ] ) == ZONE_LOW )           \
				q2.push_back( otheridx );                                \
			blobPx[ otheridx ] |= BLOB_VISITED;                          \
		}                                                                \
	} while ( 0 )
		// four-way connectivity
		TEST( -1, 0 );
		TEST( 0, -1 );
		TEST( 0, 1 );
		TEST( 1, 0 );
#undef TEST
	}

	if ( q.size() < m_settings.hand_min_size ) {
		/* Not enough pixels */
		rejectBlob( q, 3 );
		return false;
	}

	/* Enough pixels for the hand: onto the next stage! */
	for ( auto i : q2 ) {
		blobPx[ i ] &= ~BLOB_VISITED;
	}

	bool found_fingers = false;
	for ( auto i : q2 ) {
		if ( blobPx[ i ] != 0 ) continue;
		IRDepthFinger finger;
		if ( floodFinger( finger, i ) ) {
			hand.fingers.push_back( finger );
			found_fingers = true;
		}
	}

	if ( !found_fingers ) {
		rejectBlob( q, 4 );
		return false;
	}

	int color = colorForBlobIndex( nextBlobId++ ) << 8;
	for ( auto i : q ) {
		blobPx[ i ] |= color;
	}
	return true;
}

bool IRDepthTouchTracker::floodFinger( IRDepthFinger& finger, unsigned idx )
{
	const int n = w * h;

	uint32_t* diffPx = (uint32_t*)diffIm.get();
	uint32_t* edgePx = (uint32_t*)edgeIm.get();
	uint32_t* blobPx = (uint32_t*)blobIm.get();

	std::vector< unsigned > q, q2;
	std::vector< unsigned > roots;	// pixels next to mid/high conf pixels
	int qtail = 0;

	q.push_back( idx );

	while ( qtail < q.size() ) {
		unsigned curidx = q[ qtail++ ];
		unsigned dist = curidx >> 24;
		curidx &= 0xffffff;

		if ( dist > m_settings.finger_max_dist ) {
			rejectBlob( q, 7 );
			for ( auto i : q ) {
				blobPx[ i & 0xffffff ] = 0;
			}
			return false;
		}

		int y = curidx / w;
		int x = curidx % w;

		blobPx[ curidx ] |= ZONE_LOW | dist;

		bool isRoot = false;  // are we adjacent to a mid/highconf pixel?

#define TEST( dx, dy )                                                  \
	do {                                                                \
		int xx = x + dx, yy = y + dy;                                   \
		if ( 0 <= xx && xx < w && 0 <= yy && yy < h ) {                 \
			int otheridx = curidx + dy * w + dx;                        \
			if ( blobPx[ otheridx ] >= ZONE_MID ) isRoot = true;        \
			if ( blobPx[ otheridx ] != 0 ) continue;                    \
			if ( edgePx[ otheridx ] & 0x00ff00ff ) /* IR + depth abs */ \
				continue;                                               \
			if ( ZONE( diffPx[ otheridx ] ) >= ZONE_LOW )               \
				q.push_back( otheridx | ( ( dist + 1 ) << 24 ) );       \
			else if ( ZONE( diffPx[ otheridx ] ) == ZONE_NOISE )        \
				q2.push_back( otheridx | ( ( dist + 1 ) << 24 ) );      \
			blobPx[ otheridx ] |= BLOB_VISITED;                         \
		}                                                               \
	} while ( 0 )
		// four-way connectivity
		TEST( -1, 0 );
		TEST( 0, -1 );
		TEST( 0, 1 );
		TEST( 1, 0 );
#undef TEST

		if ( isRoot ) {
			roots.push_back( curidx );
		}
	}

	/* Enough pixels for the finger: onto the next stage! */
	for ( auto i : q2 ) {
		blobPx[ i & 0xffffff ] &= ~BLOB_VISITED;
	}

	std::vector< unsigned > tipq;

	for ( auto i : q2 ) {
		if ( blobPx[ i & 0xffffff ] != 0 ) continue;
		IRDepthTip tip;
		if ( floodTip( tip, i ) ) {
			for ( int j : tip.pixels ) {
				q.push_back( j );
				tipq.push_back( j );
			}
			for ( int j : tip.roots ) {
				roots.push_back( j );
			}
		}
	}

	if ( q.size() < m_settings.finger_min_size ) {
		/* Not enough pixels */
		rejectBlob( q, 5 );
		for ( auto i : tipq ) {
			blobPx[ i & 0xffffff ] = 0;
		}
		return false;
	}

	refloodFinger( q, roots );

	if ( !computeFingerMetrics( finger, q ) ) {
		/* Finger not really a finger */
		rejectBlob( q, 6 );
		for ( auto i : tipq ) {
			blobPx[ i & 0xffffff ] = 0;
		}
		return false;
	}

	int color = colorForBlobIndex( nextBlobId++ ) << 8;
	for ( auto i : q ) {
		blobPx[ i & 0xffffff ] |= color;
	}

	return true;
}

void IRDepthTouchTracker::refloodFinger( const std::vector< unsigned >& blob, std::vector< unsigned >& roots )
{
	uint32_t* blobPx = (uint32_t*)blobIm.get();

	std::set< unsigned > unseen;
	for ( auto i : blob ) unseen.insert( i & 0xffffff );
	for ( auto i : roots ) unseen.erase( i );

	std::vector< unsigned > q = roots;
	int qtail = 0;
	while ( qtail < q.size() ) {
		unsigned curidx = q[ qtail++ ];
		unsigned dist = curidx >> 24;
		curidx &= 0xffffff;

		blobPx[ curidx ] = ( blobPx[ curidx ] & ~0xff ) | dist;

#define TEST( dx, dy )                                        \
	do {                                                      \
		int otheridx = curidx + dy * w + dx;                  \
		if ( unseen.count( otheridx ) ) {                     \
			q.push_back( otheridx | ( ( dist + 1 ) << 24 ) ); \
			unseen.erase( otheridx );                         \
		}                                                     \
	} while ( 0 )
		// four-way connectivity
		TEST( -1, 0 );
		TEST( 0, -1 );
		TEST( 0, 1 );
		TEST( 1, 0 );
#undef TEST
	}
}

bool IRDepthTouchTracker::computeFingerMetrics( IRDepthFinger& finger, std::vector< unsigned >& px )
{
	uint16_t* depthPx = (uint16_t*)m_depth.data;
	uint32_t* blobPx = (uint32_t*)blobIm.get();

	const float* bgmean = (float*)m_mean.data;

	/* Sort pixels by distance */
	std::sort( px.begin(), px.end(), [ & ]( unsigned a, unsigned b ) {
		return ( blobPx[ a & 0xffffff ] & 0xff ) < ( blobPx[ b & 0xffffff ] & 0xff );
	} );

	/* Check max distance */
	int maxidx = px[ px.size() - 1 ] & 0xffffff;
	int maxdist = blobPx[ maxidx ] & 0xff;
	if ( maxdist < m_settings.finger_min_dist ) return false;

	/* Average the z-heights */
	size_t start = px.size() - m_settings.touchz_window;
	if ( start < 0 ) start = 0;

	float avgdiff = 0;
	int count = 0;
	int idx;
	for ( size_t i = start; i < px.size(); i++ ) {
		idx = px[ i ] & 0xffffff;
		avgdiff += bgmean[ idx ] - depthPx[ idx ];
		count++;
	}
	finger.z = abs( avgdiff ) / count;

	/* Average the tip x and y values */
	start = px.size() - m_settings.tipavg_window;
	if ( start < 0 ) start = 0;

	float avgx = 0, avgy = 0;
	count = 0;
	for ( size_t i = start; i < px.size(); i++ ) {
		idx = px[ i ] & 0xffffff;
		avgx += idx % w;
		avgy += idx / w;
		count++;
	}

	finger.x = avgx / count;
	finger.y = avgy / count;
	return true;
}

TouchTrackerSettings& IRDepthTouchTracker::touchSettings()
{
	return m_settings;
}

bool IRDepthTouchTracker::floodTip( IRDepthTip& tip, unsigned idx )
{
	const int n = w * h;

	uint32_t* edgePx = (uint32_t*)edgeIm.get();
	uint32_t* blobPx = (uint32_t*)blobIm.get();

	std::vector< unsigned > q;
	int qtail = 0;

	q.push_back( idx );

	while ( qtail < q.size() ) {
		unsigned curidx = q[ qtail++ ];
		unsigned dist = curidx >> 24;
		curidx &= 0xffffff;
		if ( dist > m_settings.tip_max_dist ) goto reject_blob;

		int y = curidx / w;
		int x = curidx % w;

		blobPx[ curidx ] |= ZONE_NOISE | dist;

		bool isRoot = false;  // are we adjacent to a mid/highconf pixel?

#define TEST( dx, dy )                                           \
	do {                                                         \
		int xx = x + dx, yy = y + dy;                            \
		if ( 0 <= xx && xx < w && 0 <= yy && yy < h ) {          \
			int otheridx = curidx + dy * w + dx;                 \
			if ( blobPx[ otheridx ] >= ZONE_MID ) isRoot = true; \
			if ( blobPx[ otheridx ] != 0 ) continue;             \
			if ( edgePx[ otheridx ] & 0x00ff0000 ) /* IR only */ \
				continue;                                        \
			q.push_back( otheridx | ( ( dist + 1 ) << 24 ) );    \
			blobPx[ otheridx ] |= BLOB_VISITED;                  \
		}                                                        \
	} while ( 0 )
		// four-way connectivity
		TEST( -1, 0 );
		TEST( 0, -1 );
		TEST( 0, 1 );
		TEST( 1, 0 );
#undef TEST

		if ( isRoot ) {
			tip.roots.push_back( curidx );
		}
	}

	swap( tip.pixels, q );
	return true;

reject_blob:
	for ( auto i : q ) {
		// We pretend that this blob never happened.
		blobPx[ i & 0xffffff ] = 0;
	}
	return false;
}
#pragma endregion

IRDepthTouchTracker::IRDepthTouchTracker( int width, int height, TouchTrackerSettings settings )
	: m_settings( settings ), w( width ), h( height )
{
	diffIm = std::unique_ptr< uint32_t[] >( new uint32_t[ w * h * 4 ] );
	edgeIm = std::unique_ptr< uint32_t[] >( new uint32_t[ w * h * 4 ] );
	blobIm = std::unique_ptr< uint32_t[] >( new uint32_t[ w * h * 4 ] );
	irCanny = std::unique_ptr< uint8_t[] >( new uint8_t[ w * h ] );
}

std::vector< FingerTouch > IRDepthTouchTracker::mergeTouches( std::vector< FingerTouch >& curTouches,
															  std::vector< FingerTouch >& newTouches )
{
	struct touch_dist
	{
		int cur_index, new_index;
		float dist;
		bool operator<( const struct touch_dist& other ) const
		{
			return dist < other.dist;
		}
	};

	/* Assign each new touch to the nearest cur neighbour */
	for ( auto& i : newTouches ) {
		i.id = -1;
	}

	std::vector< touch_dist > distances;
	for ( int i = 0; i < curTouches.size(); i++ ) {
		for ( int j = 0; j < newTouches.size(); j++ ) {
			float d = cv::norm( curTouches[ i ].tip - newTouches[ j ].tip );
			if ( d > 50 ) continue;
			touch_dist dist = { i, j, d };
			distances.push_back( dist );
		}
	}
	std::sort( distances.begin(), distances.end() );

	for ( const auto& i : distances ) {
		FingerTouch& curTouch = curTouches[ i.cur_index ];
		FingerTouch& newTouch = newTouches[ i.new_index ];
		/* check if already assigned */
		if ( curTouch.id < 0 || newTouch.id >= 0 ) continue;
		/* move cur id into new id */
		newTouch.id = curTouch.id;
		curTouch.id = -1;

		/* update other attributes */
		newTouch.touchAge = curTouch.touchAge + 1;
		/* EWMA new touch */
		newTouch.tip = m_settings.smooth_tip_alpha * ( newTouch.tip - curTouch.tip ) + curTouch.tip;
		newTouch.touchZ
			= m_settings.smooth_touchz_alpha * ( newTouch.touchZ - curTouch.touchZ ) + curTouch.touchZ;

		if ( curTouch.touched && newTouch.touchZ > m_settings.touchz_exit ) {
			newTouch.touched = false;
			newTouch.statusAge = 0;
		} else if ( !curTouch.touched && newTouch.touchZ < m_settings.touchz_enter ) {
			newTouch.touched = true;
			newTouch.statusAge = 0;
		} else {
			newTouch.touched = curTouch.touched;
			newTouch.statusAge = curTouch.statusAge + 1;
		}
	}

	for ( auto& i : newTouches ) {
		i.missing = false;
		i.missingAge = 0;
	}

	/* Add 'missing' touches back */
	for ( auto& i : curTouches ) {
		if ( i.id >= 0 && ( !i.missing || i.missingAge < 3 ) ) {
			i.missingAge = ( i.missing ) ? i.missingAge + 1 : 0;
			i.missing = true;
			i.statusAge++;
			i.touchAge++;
			newTouches.push_back( i );
		}
	}

	/* Handle new touches and output */
	std::vector< FingerTouch > finalTouches;
	for ( auto& i : newTouches ) {
		if ( i.id < 0 ) {
			i.id = m_nextTouchId++;
			i.statusAge = i.touchAge = 0;
		}
		finalTouches.push_back( i );
	}

	return finalTouches;
}

std::vector< FingerTouch > IRDepthTouchTracker::run( cv::Mat depth, cv::Mat mean, cv::Mat stdDev, cv::Mat ir )
{
	m_depth = depth;
	m_mean = mean;
	m_stdDev = stdDev;
	m_ir = ir;

	buildDiffImage();
	buildEdgeImage();  // edge image depends on diff

	std::vector< IRDepthArm > arms = detectTouches();

	std::vector< FingerTouch > newTouches;
	for ( const auto& arm : arms ) {
		for ( const auto& hand : arm.hands ) {
			for ( const auto& finger : hand.fingers ) {
				FingerTouch touch;
				touch.tip = { finger.x, finger.y };
				touch.touchZ = finger.z;
				touch.touchZAbsolute = m_mean.at< float >( finger.y, finger.x );
				newTouches.push_back( touch );
			}
		}
	}

	std::vector< FingerTouch > curTouches = m_touches;

	m_touches = mergeTouches( curTouches, newTouches );

	auto markedTouches = ir.clone();
	for ( const auto& touch : m_touches ) {
		if ( touch.touched ) {
			cv::circle( markedTouches, cv::Point( touch.tip.x, touch.tip.y ), 5, cv::Scalar( 255, 0, 0 ) );
		}
	}

	return m_touches;
}

cv::Mat IRDepthTouchTracker::getDiff()
{
	return cv::Mat( cv::Size( w, h ), CV_8UC4, (void*)diffIm.get(), cv::Mat::AUTO_STEP );
}

cv::Mat IRDepthTouchTracker::getEdge()
{
	return cv::Mat( cv::Size( w, h ), CV_8UC4, (void*)edgeIm.get(), cv::Mat::AUTO_STEP );
}
cv::Mat IRDepthTouchTracker::getBlob()
{
	return cv::Mat( cv::Size( w, h ), CV_8UC4, (void*)blobIm.get(), cv::Mat::AUTO_STEP );
}