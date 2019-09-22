#include <iostream>
#include <string>
#include "seam_carver/seam_carver.h"

using seam_carving::SeamCarver;

void PrintUsage() {
  std::cout << "Usage: ./seam_carving <input_image> <direction> <number of seams>\n";
  std::cout << "  <direction> should be \'h\' or \'v\'\n";
}

int main(int argc, char **argv) {
  if (argc != 4) {
    PrintUsage();
    return 1;
  }
  std::string input_img_path(argv[1]);
  std::string carve_direction(argv[2]);
  const int num_seams = std::stoi(std::string(argv[3]));

  cv::Mat input_img;
  input_img = cv::imread(input_img_path);

#ifdef VIZ_DEBUG
  cv::imshow("input image", input_img);
  std::cout << "Press any key to show the process of seam carving.\n";
  cv::waitKey(0);
#endif

  SeamCarver seam_carver(input_img);
  if (carve_direction == "h") {
    seam_carver.HorizontalCarving(num_seams);
  } else if (carve_direction == "v") {
    seam_carver.VerticalCarving(num_seams);
  } else {
    std::cout << "unknown direction: " << carve_direction << "\n";
  }

  cv::imshow("carved image", seam_carver.carved_image());
  cv::waitKey(0);

  return 0;
}

