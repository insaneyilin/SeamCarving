# SeamCarving

A C++ implementation of Seam-Carving -- a classic content-aware image resizing algorithm.

## Dependencies

- CMake
- OpenCV

## Build

```
mkdir build
cd build
cmake ..
make
```

## Usage

```

./seam_carving <input_image> <direction> <number of seams>
    <direction> should be 'h'(horizontally) or 'v'(vertically)

 Or

./seam_carving <input_image> <direction> <number of seams> <mode> <x> <y> <w> <h>
    <mode> should be 'r' or 'p', 'r' for removal, 'p' for protection
    <x> <y> <w> <h> specify the ROI.

```

## Screenshots

### carving vertically

```
./build/app/seam_carving ./test_images/HJoceanSmall.png v 80
```

![](https://github.com/insaneyilin/SeamCarving/blob/master/screenshots/seam_carving_screenshots.gif)

### object removal

```
./build/app/seam_carving ./test_images/HJoceanSmall.png v 85 r 112 80 85 58
```

![](https://github.com/insaneyilin/SeamCarving/blob/master/screenshots/seam_carving_object_removal.gif)

### object protection

without protection:

```
./build/app/seam_carving ./test_images/sunset.jpeg v 300
```

![](https://github.com/insaneyilin/SeamCarving/blob/master/screenshots/sunset_without_protection.png)

with protection:

```
./build/app/seam_carving ./test_images/sunset.jpeg v 300 p 734 354 210 217
```

![](https://github.com/insaneyilin/SeamCarving/blob/master/screenshots/sunset_with_protection.png)
