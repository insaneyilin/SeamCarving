#include "seam_carver/seam_carver.h"
#include <iostream>
#include <cassert>
#include <limits>
#include <algorithm>

namespace seam_carving {

const float SeamCarver::s_removal_energy_val_ = -100.f;
const float SeamCarver::s_protection_energy_val_ = 100.f;

SeamCarver::SeamCarver(const cv::Mat &input_image) {
  origin_image_ = input_image.clone();
  carved_image_ = input_image.clone();
  removal_region_mask_ = cv::Mat::zeros(
      origin_image_.rows, origin_image_.cols, CV_8UC1);
  protection_region_mask_ = cv::Mat::zeros(
      origin_image_.rows, origin_image_.cols, CV_8UC1);
}

void SeamCarver::VerticalCarving(int num_seams) {
  for (int i = 0; i < num_seams; ++i) {
    FindVerticalSeam(&cur_seam_);
#ifdef VIZ_DEBUG
    // visualization for debugging
    cv::Mat seam_image = carved_image_.clone();
    for (int i = 0; i < carved_image_.rows; ++i) {
      seam_image.at<cv::Vec3b>(i, cur_seam_[i])[0] = 0;
      seam_image.at<cv::Vec3b>(i, cur_seam_[i])[1] = 0;
      seam_image.at<cv::Vec3b>(i, cur_seam_[i])[2] = 255;
    }
    cv::imshow("seam image", seam_image);
    cv::waitKey(1);
#endif
    RemoveVerticalSeam(cur_seam_);
  }
}

void SeamCarver::HorizontalCarving(int num_seams) {
  for (int i = 0; i < num_seams; ++i) {
    FindHorizontalSeam(&cur_seam_);
#ifdef VIZ_DEBUG
    // visualization for debugging
    cv::Mat seam_image = carved_image_.clone();
    for (int j = 0; j < carved_image_.cols; ++j) {
      seam_image.at<cv::Vec3b>(cur_seam_[j], j)[0] = 0;
      seam_image.at<cv::Vec3b>(cur_seam_[j], j)[1] = 0;
      seam_image.at<cv::Vec3b>(cur_seam_[j], j)[2] = 255;
    }
    cv::imshow("seam image", seam_image);
    cv::waitKey(1);
#endif
    RemoveHorizontalSeam(cur_seam_);
  }
}

void SeamCarver::SetRemovalMaskByRect(const cv::Rect &rect) {
  removal_region_mask_(rect).setTo(255);
}

void SeamCarver::SetProtectionMaskByRect(const cv::Rect &rect) {
  protection_region_mask_(rect).setTo(255);
}

void SeamCarver::Reset() {
  carved_image_ = origin_image_.clone();
}

void SeamCarver::CalcEnergyMap() {
  cv::cvtColor(carved_image_, carved_image_gray_, cv::COLOR_BGR2GRAY);

  cv::Sobel(carved_image_gray_, sobel_x_map_, CV_32F, 1, 0, 3);
  cv::convertScaleAbs(sobel_x_map_, sobel_x_map_);

  cv::Sobel(carved_image_gray_, sobel_y_map_, CV_32F, 0, 1, 3);
  cv::convertScaleAbs(sobel_y_map_, sobel_y_map_);

  cv::addWeighted(sobel_x_map_, 0.5, sobel_y_map_, 0.5, 0, energy_map_);

  cv::Mat energy_map_float;
  energy_map_.convertTo(energy_map_float, CV_32F);
  energy_map_ = energy_map_float;
  // consider removal mask
  for (int i = 0; i < energy_map_.rows; ++i) {
    for (int j = 0; j < energy_map_.cols; ++j) {
      if (removal_region_mask_.at<uchar>(i, j) > 0) {
        energy_map_.at<float>(i, j) = s_removal_energy_val_;
      }
    }
  }

  // consider protection mask
  for (int i = 0; i < energy_map_.rows; ++i) {
    for (int j = 0; j < energy_map_.cols; ++j) {
      if (protection_region_mask_.at<uchar>(i, j) > 0) {
        energy_map_.at<float>(i, j) = s_protection_energy_val_;
      }
    }
  }

#ifdef VIZ_DEBUG
  // visualization of the energy map
  cv::Mat energy_map_viz;
  energy_map_.convertTo(energy_map_viz, CV_8U);
  cv::imshow("energy_map", energy_map_viz);
  cv::waitKey(1);
#endif
}

void SeamCarver::FindHorizontalSeam(std::vector<int> *seam) {
  assert(seam);
  CalcEnergyMap();
  cv::transpose(energy_map_, energy_map_);
  FindSeamWithDynamicProgramming(energy_map_, seam);
}

void SeamCarver::FindVerticalSeam(std::vector<int> *seam) {
  assert(seam);
  CalcEnergyMap();
  FindSeamWithDynamicProgramming(energy_map_, seam);
}

void SeamCarver::FindSeamWithDynamicProgramming(const cv::Mat &energy_map,
    std::vector<int> *seam) {
  assert(seam);
  const int r = energy_map.rows;
  const int c = energy_map.cols;
  dp_mat_.clear();
  dp_mat_.resize(r, std::vector<float>(c, 0.f));
  dp_path_.clear();
  dp_path_.resize(r, std::vector<int>(c, 0));

  for (int j = 0; j < c; ++j) {
    dp_mat_[0][j] = energy_map.at<float>(0, j);
    dp_path_[0][j] = j;
  }
  
  for (int i = 1; i < r; ++i) {
    for (int j = 0; j < c; ++j) {
      float energy_left_upper = j - 1 >= 0 ?
          dp_mat_[i - 1][j - 1] :
          std::numeric_limits<int>::max();
      float energy_right_upper = j + 1 < c ?
          dp_mat_[i - 1][j + 1] :
          std::numeric_limits<int>::max();
      float energy_upper = dp_mat_[i - 1][j];
      float energy_min = std::min(energy_upper,
          std::min(energy_left_upper, energy_right_upper));

      int parent_idx = j;
      if (std::fabs(energy_min - energy_left_upper) < 1e-6) {
        parent_idx = j - 1;
      } else if (std::fabs(energy_min - energy_right_upper) < 1e-6) {
        parent_idx = j + 1;
      }
      dp_mat_[i][j] = energy_map.at<float>(i, j) +
          energy_min;
      dp_path_[i][j] = parent_idx;
    }
  }
  seam->resize(r, 0);
  int col_idx = std::min_element(dp_mat_[r - 1].begin(),
      dp_mat_[r - 1].end()) - dp_mat_[r - 1].begin();
  (*seam)[r - 1] = col_idx;
  for (int k = r - 1; k > 0; --k) {
    col_idx = dp_path_[k][col_idx];
    (*seam)[k - 1] = col_idx;
  }
}

void SeamCarver::RemoveHorizontalSeam(const std::vector<int> &seam) {
  if (carved_image_.rows <= 1 || carved_image_.cols <= 1) {
    return;
  }
  cv::transpose(carved_image_, carved_image_);
  cv::transpose(removal_region_mask_, removal_region_mask_);
  cv::transpose(protection_region_mask_, protection_region_mask_);
  RemoveVerticalSeam(seam);
  cv::transpose(carved_image_, carved_image_);
  cv::transpose(removal_region_mask_, removal_region_mask_);
  cv::transpose(protection_region_mask_, protection_region_mask_);
}

void SeamCarver::RemoveVerticalSeam(const std::vector<int> &seam) {
  if (carved_image_.rows <= 1 || carved_image_.cols <= 1) {
    return;
  }
  cv::Mat carved_img(carved_image_.rows, carved_image_.cols - 1,
      carved_image_.type(), cv::Scalar(0, 0, 0));
  cv::Mat carved_removal_mask = cv::Mat::zeros(
      removal_region_mask_.rows, removal_region_mask_.cols - 1, CV_8UC1);
  cv::Mat carved_protection_mask = cv::Mat::zeros(
      protection_region_mask_.rows, protection_region_mask_.cols - 1, CV_8UC1);
  for (int i = 0; i < carved_image_.rows; ++i) {
    for (int j = 0; j < seam[i]; ++j) {
      carved_img.at<cv::Vec3b>(i, j) = carved_image_.at<cv::Vec3b>(i, j);
      carved_removal_mask.at<uchar>(i, j) = removal_region_mask_.at<uchar>(i, j);
      carved_protection_mask.at<uchar>(i, j) =
          protection_region_mask_.at<uchar>(i, j);
    }
    for (int j = seam[i]; j < carved_img.cols; ++j) {
      carved_img.at<cv::Vec3b>(i, j) = carved_image_.at<cv::Vec3b>(i, j + 1);
      carved_removal_mask.at<uchar>(i, j) =
          removal_region_mask_.at<uchar>(i, j + 1);
      carved_protection_mask.at<uchar>(i, j) =
          protection_region_mask_.at<uchar>(i, j + 1);
    }
  }
  carved_image_ = carved_img.clone();
  removal_region_mask_ = carved_removal_mask.clone();
  protection_region_mask_ = carved_protection_mask.clone();
}

}  // namespace seam_carving

