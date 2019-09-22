#ifndef SEAM_CARVING_SEAM_CARVER_H_
#define SEAM_CARVING_SEAM_CARVER_H_

#include <vector>
#include <opencv2/opencv.hpp>

#define VIZ_DEBUG

namespace seam_carving {

class SeamCarver {
 public:
  SeamCarver() {}
  SeamCarver(const cv::Mat &input_image);

  ~SeamCarver() {}

  void VerticalCarving(int num_seams);
  void HorizontalCarving(int num_seams);

  cv::Mat carved_image() const {
    return carved_image_;
  }

 private:
  // reset to origin image
  void Reset();
  void CalcEnergyMap();
  void FindHorizontalSeam(std::vector<int> *seam);
  void FindVerticalSeam(std::vector<int> *seam);
  void FindSeamWithDynamicProgramming(const cv::Mat &energy_map,
      std::vector<int> *seam);

  void RemoveHorizontalSeam(const std::vector<int> &seam);
  void RemoveVerticalSeam(const std::vector<int> &seam);

 private:
  cv::Mat origin_image_;
  cv::Mat carved_image_;

  cv::Mat carved_image_gray_;  // used for calc gradient map
  cv::Mat energy_map_;
  cv::Mat sobel_x_map_;
  cv::Mat sobel_y_map_;

  std::vector<int> cur_seam_;
  // DP matrix (we use dynamic programming to find the optimal seam)
  std::vector<std::vector<int> > dp_mat;
  // dp_path[i][j] records the `parent` index of dp_mat[i][j]
  std::vector<std::vector<int> > dp_path;
};

}  // namespace seam_carving

#endif  // SEAM_CARVING_SEAM_CARVER_H_

