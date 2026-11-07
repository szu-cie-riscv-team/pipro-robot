// Adapted from FoundationVision/ByteTrack d1bf0191; see TRACKER_ATTRIBUTION.md.
#pragma once

#include "k1muse_perception/bytetrack/STrack.hpp"
#include <climits>
#include <iostream>
#include <map>

namespace k1muse_perception::bytetrack
{

struct Object
{
    cv::Rect_<float> rect;
    int label;
	int detection_index = -1;
    float prob;
};

class BYTETracker
{
public:
	BYTETracker(int track_buffer = 30, float track_threshold = 0.5F,
		float new_track_threshold = 0.6F, float match_threshold = 0.8F, int initial_frame = 0);
	size_t track_count() const { return tracked_stracks.size() + lost_stracks.size(); }
	vector<STrack>& lost_tracks() { return lost_stracks; }
	int current_frame() const { return frame_id; }
	~BYTETracker();

	vector<STrack> update(const vector<Object>& objects);
	Scalar get_color(int idx);

private:
	vector<STrack*> joint_stracks(vector<STrack*> &tlista, vector<STrack> &tlistb);
	vector<STrack> joint_stracks(vector<STrack> &tlista, vector<STrack> &tlistb);

	vector<STrack> sub_stracks(vector<STrack> &tlista, vector<STrack> &tlistb);
	void remove_duplicate_stracks(vector<STrack> &resa, vector<STrack> &resb, vector<STrack> &stracksa, vector<STrack> &stracksb);

	void linear_assignment(vector<vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh,
		vector<vector<int> > &matches, vector<int> &unmatched_a, vector<int> &unmatched_b);
	vector<vector<float> > iou_distance(vector<STrack*> &atracks, vector<STrack> &btracks, int &dist_size, int &dist_size_size);
	vector<vector<float> > iou_distance(vector<STrack> &atracks, vector<STrack> &btracks);
	vector<vector<float> > ious(vector<vector<float> > &atlbrs, vector<vector<float> > &btlbrs);

	double lapjv(const vector<vector<float> > &cost, vector<int> &rowsol, vector<int> &colsol, 
		bool extend_cost = false, float cost_limit = LONG_MAX, bool return_cost = true);

private:

	float track_thresh;
	float high_thresh;
	float match_thresh;
	int frame_id;
	int max_time_lost;

	vector<STrack> tracked_stracks;
	vector<STrack> lost_stracks;
	vector<STrack> removed_stracks;
	byte_kalman::KalmanFilter kalman_filter;
};
}  // namespace k1muse_perception::bytetrack
