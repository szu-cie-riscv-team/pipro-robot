#include "k1muse_perception/depth_estimation.hpp"

#include <algorithm>
#include <cmath>

namespace k1muse_perception
{

DepthEstimate depth_center_mean(
  const DepthImageView & image, const DepthRoi & roi, const DepthConfig & config,
  DepthScratch &)
{
  DepthEstimate result;
  const int64_t u = static_cast<int64_t>(std::clamp(
      std::round(roi.center_u), double(roi.x1), double(roi.x2 - 1)));
  const int64_t v = static_cast<int64_t>(std::clamp(
      std::round(roi.center_v), double(roi.y1), double(roi.y2 - 1)));
  const int64_t radius = config.center_window_size / 2;
  double total = 0.0;
  for (int64_t y = std::max<int64_t>(roi.y1, v - radius);
    y < std::min<int64_t>(roi.y2, v + radius + 1); ++y)
  {
    for (int64_t x = std::max<int64_t>(roi.x1, u - radius);
      x < std::min<int64_t>(roi.x2, u + radius + 1); ++x)
    {
      ++result.sampled_count;
      const double z = image.at(x, y);
      if (std::isfinite(z) && z >= config.min_depth_m && z <= config.max_depth_m) {
        total += z;
        ++result.valid_count;
      }
    }
  }
  if (result.valid_count >= config.min_valid_count) {
    result.valid = true;
    result.reason = "ok";
    result.u = u;
    result.v = v;
    result.depth_m = total / result.valid_count;
  }
  return result;
}

DepthEstimate depth_roi_median(
  const DepthImageView & image, const DepthRoi & roi, const DepthConfig & config,
  DepthScratch & scratch)
{
  // Sample the central half of the box, at most 21x21 distinct pixels.
  // Local ROI statistics follow the Luxonis host spatial-calculation pattern;
  // no full-frame filtering, hole filling or history is needed.
  const int x1 = static_cast<int>(std::clamp(std::floor(roi.center_u-roi.size_x/4),
      double(roi.x1), double(roi.x2-1)));
  const int y1 = static_cast<int>(std::clamp(std::floor(roi.center_v-roi.size_y/4),
      double(roi.y1), double(roi.y2-1)));
  const int x2 = static_cast<int>(std::clamp(std::ceil(roi.center_u+roi.size_x/4),
      double(x1+1), double(roi.x2)));
  const int y2 = static_cast<int>(std::clamp(std::ceil(roi.center_v+roi.size_y/4),
      double(y1+1), double(roi.y2)));
  const int nx=std::min(21,x2-x1), ny=std::min(21,y2-y1);
  auto & values=scratch.values;values.clear();values.reserve(441);
  DepthEstimate result;
  for(int iy=0;iy<ny;++iy) {
    const int y=y1+(ny==1?0:iy*(y2-y1-1)/(ny-1));
    for(int ix=0;ix<nx;++ix) {
      const int x=x1+(nx==1?0:ix*(x2-x1-1)/(nx-1));
      ++result.sampled_count;
      const double z=image.at(x,y);
      if(std::isfinite(z) && z>=config.min_depth_m && z<=config.max_depth_m) {
        values.push_back(z);
      }
    }
  }
  result.valid_count=values.size();
  if(values.size()<config.min_valid_count) {return result;}
  auto middle=values.begin()+values.size()/2;
  std::nth_element(values.begin(),middle,values.end());
  result.depth_m=*middle;
  if(values.size()%2==0) {
    result.depth_m=(result.depth_m+*std::max_element(values.begin(),middle))/2;
  }
  result.valid=true;result.reason="ok";
  result.u=std::clamp(roi.center_u,double(roi.x1),double(roi.x2-1));
  result.v=std::clamp(roi.center_v,double(roi.y1),double(roi.y2-1));
  return result;
}

}  // namespace k1muse_perception
