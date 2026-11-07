#include "k1muse_manager/survey_counter.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace k1muse_manager
{
void ShelfCounter::reset(unsigned confirmations, double deadband, bool left_to_right)
{
  if (!confirmations || !std::isfinite(deadband) || deadband < 0) {
    throw std::invalid_argument("invalid counter configuration");
  }
  confirmations_=confirmations;deadband_=deadband;direction_=left_to_right?1:-1;
  seq_=0;last_frame_=0;provisional_=0;left_.reset();right_.reset();frontier_.reset();
  tracks_.clear();pending_ids_.clear();lost_crossings_.clear();
}
bool ShelfCounter::inside(double p) const
{return (!left_ || p>*left_) && (!right_ || p<*right_);}
void ShelfCounter::recount()
{
  provisional_=0;
  for(const auto & item:tracks_) {if(item.second.counted && inside(item.second.crossing)) {++provisional_;}}
}
void ShelfCounter::set_boundary(bool left,double position)
{
  if(!std::isfinite(position)) {throw std::invalid_argument("invalid shelf boundary");}
  (left?left_:right_)=position;recount();
}
void ShelfCounter::observe(Track & t,Sample p,bool allow_new)
{
  if(!std::isfinite(p.distance) || t.counted) {return;}
  const int side=p.distance>deadband_?1:p.distance < -deadband_?-1:0;
  if(t.last_seq && t.last_seq+1!=seq_) {t.positive={};t.pending=false;t.confirmations=0;}
  t.last_seq=seq_;
  if(!side) {t.pending=false;t.confirmations=0;return;}
  if(!t.initialized) {
    if(!allow_new) {return;}
    t.initialized=true;t.eligible=side>0;
  }
  if(side>0) {
    t.pending=false;t.confirmations=0;
    t.positive=allow_new?p:Sample{};return;
  }
  if(!t.eligible) {return;}
  if(!t.pending) {
    if(!allow_new || !t.positive.ns) {return;}
    const double f=t.positive.distance/(t.positive.distance-p.distance);
    t.crossing=t.positive.position+f*(p.position-t.positive.position);
    t.pending=true;t.confirmations=0;
  }
  if(++t.confirmations<confirmations_) {return;}
  t.counted=true;t.pending=false;
  if(inside(t.crossing)) {++provisional_;}
}
void ShelfCounter::observe_boxes(int64_t ns,double position,const std::vector<Box> & boxes)
{
  if(ns<=last_frame_ || !std::isfinite(position)) {return;}
  last_frame_=ns;++seq_;
  const double progress=direction_*position;
  const bool allow_new=!frontier_ || progress>=*frontier_;
  frontier_=frontier_?std::max(*frontier_,progress):progress;
  std::unordered_set<std::string> seen;
  std::vector<std::string> pending;
  for(const auto & box:boxes) {
    if(box.id.empty() || !seen.insert(box.id).second) {continue;}
    auto & track=tracks_[box.id];observe(track,{ns,box.distance,position},allow_new);
    if(track.pending) {pending.push_back(box.id);}
  }
  for(const auto & id:pending_ids_) {
    auto & t=tracks_.at(id);
    if(t.pending && t.last_seq!=seq_) {
      lost_crossings_.push_back(t.crossing);t.pending=false;t.confirmations=0;t.positive={};
    }
  }
  pending_ids_=std::move(pending);
}
ShelfCounter::Result ShelfCounter::finalize() const
{
  if(!left_ || !right_) {return {false,0,"boundary_missing"};}
  if(*left_>=*right_) {return {false,0,"boundary_order"};}
  for(const auto p:lost_crossings_) {if(inside(p)) {return {false,0,"tracking_gap"};}}
  for(const auto & id:pending_ids_) {
    const auto & t=tracks_.at(id);if(t.pending && inside(t.crossing)) {return {false,0,"crossing_unconfirmed"};}
  }
  return {true,provisional_,""};
}
}  // namespace k1muse_manager
