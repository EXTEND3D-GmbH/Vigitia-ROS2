#pragma once
// Curve Fit from:
//    https://github.com/sikasjc/fitCurves
// RamerDouglas from:
//    2D implementation of the Ramer-Douglas-Peucker algorithm
//    By Tim Sheerman-Chase, 2016
//    Released under CC0
//    https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm

#include <opencv2/core/core.hpp>
#include <vector>
class BezierCurve
{
public:
	cv::Point2d pt1;
	cv::Point2d pt2;
	cv::Point2d c1;
	cv::Point2d c2;
};
// typedef vector<double, double> vec2d;

std::vector< cv::Point2d > RamerDouglasPeucker( const std::vector< cv::Point2d >& pointList, double epsilon );

std::vector< BezierCurve > FitCurve( std::vector< cv::Point2d > const& d,
									 int startIndex,
									 int endIndex,
									 double error );
void addBezierCurve( std::vector< cv::Vec2d > const& bezCurve, std::vector< BezierCurve >* bezCurves );
void FitCubic( std::vector< cv::Point2d > const& d,
			   std::vector< BezierCurve >* bezCurves,
			   int first,
			   int last,
			   cv::Vec2d tHat1,
			   cv::Vec2d tHat2,
			   double error );
void generateBezier( std::vector< cv::Point2d > const& d,
					 std::vector< cv::Vec2d >* bezCurve,
					 int first,
					 int last,
					 std::vector< double > const& uPrime,
					 cv::Vec2d tHat1,
					 cv::Vec2d tHat2 );

std::vector< double >* reparameterize( std::vector< cv::Point2d > const& d,
									   int first,
									   int last,
									   std::vector< double > const& u,
									   std::vector< cv::Vec2d >* bezCurve );
double newtonRaphsonRootFind( std::vector< cv::Vec2d > const& Q, cv::Point2d P, double u );
cv::Point2d bezierII( int degree, std::vector< cv::Vec2d > const& V, double t );

double B0( double u );
double B1( double u );
double B2( double u );
double B3( double u );

cv::Vec2d computeLeftTangent( std::vector< cv::Point2d > const& d, int end );
cv::Vec2d computeRightTangent( std::vector< cv::Point2d > const& d, int end );
cv::Vec2d computeCenterTangent( std::vector< cv::Point2d > const& d, int center );
void chordLengthParameterize( std::vector< cv::Point2d > const& d,
							  int first,
							  int last,
							  std::vector< double >* u );

double computeMaxError( std::vector< cv::Point2d > const& d,
						int first,
						int last,
						std::vector< cv::Vec2d >* bezCurve,
						std::vector< double > const& u,
						int* splitPoint );
double getDistance( cv::Point2d p0, cv::Point p1 );
cv::Vec2d scaleVec( cv::Vec2d v, double newlen );
cv::Vec2d negateVec( cv::Vec2d v );