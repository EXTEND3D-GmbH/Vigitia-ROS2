/**
 * Copyright (C) 2022 EXTEND3D and/or its subsidiary(-ies)
 * https://extend3d.com
 *
 * @author Chris Brammer
 */
#pragma once

struct TouchTrackerSettings
{
	/* Tweakable parameters */
	/// Edge/fence parameters
	int edge_depthrel_dist = 1;		 // px: distance range to consider relative-depth (smoothness) fence
	int edge_depthrel_thresh = 30;	 // mm: max diff between pixel and pixels in dist range
	int edge_depthabs_dist = 2;		 // px: distance range to consider absolute-depth (height) fence
	int edge_depthabs_thresh = 120;	 // mm: max diff between pixels and bg in dist range

	/// z/diff conditions for each of the four zones. z = z-value (relative to bg mean+stdev), diff = mm
	/// difference
	float error_cond = -10;
	float noise_cond = 0.7f;
	float low_cond = 7.5f;
	float mid_cond = 35.f;
	// remaining pixels => ZONE_HIGH

	/// object measurement parameters (n.b. ideally these would be in mm. Since they are in px, they should be
	/// liberally set to avoid rejecting too many objects.)
	int arm_min_size = 50;				 // px: minimum size of arm blob
	int hand_min_size = 25;				 // px: minimum size of hand blob
	int finger_min_size = 10;			 // px: minimum size of finger blob
	int finger_min_dist = 4;			 // px: finger minimum length (distance); shorter "fingers" are pruned
	unsigned int tip_max_dist = 50;		 // px: maximum flood distance for tip fill (if exceeded, rollback)
	unsigned int finger_max_dist = 200;	 // px: maximum flood distance for tip fill (if exceeded, rollback)

	/// postprocessing settings
	// n.b. no smoothing for tip now (it is very stable with IR data)
	float smooth_tip_alpha = 1.0;  // higher = less smoothing
	float smooth_touchz_alpha = 0.5;

	int tipavg_window = 1;		 // number of highest-distance pixels to average for tip position averaging
	int touchz_window = 8;		 // number of highest-distance pixels to average for touchz detection
	float touchz_enter = 3.25f;	 // below this avg. z, a touch is considered active
	float touchz_exit
		= 5.f;	// above this avg. z, a touch is considered inactive (must be higher than touchz_enter)
};