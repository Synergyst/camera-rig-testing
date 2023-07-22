#include <iostream>
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <array>
#include <algorithm>
#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <assert.h>
#include <getopt.h> // getopt_long()
#include <fcntl.h>  // low-level i/o
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <libv4l2.h>
#include <span>
#include <errno.h>
#include <future>
#include <linux/fb.h>
#include <execution>
#include <numeric>
#include <stdio.h>
#include <stdio.h>
#include <stdexcept>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <string_view>
#include <bit>
#include <sstream>
#include <iomanip>
#include <optional>
#include <variant>
#include <chrono>
#include <charconv>
#include <fstream>
#include <iostream>
#include <cctype>
#include <random>
#include <sys/types.h>
#include <stddef.h>
#include <arm_neon.h>

#define V4L_ALLFORMATS  3
#define V4L_RAWFORMATS  1
#define V4L_COMPFORMATS 2
#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define IS_RGB_DEVICE false // change in case capture device is really RGB24 and not BGR24
std::atomic<bool> shouldLoop;
std::future<int> background_task_cap_main;
std::future<int> background_task_cap_alt;
struct buffer {
  void* start;
  size_t length;
};
struct devInfo {
  int framerate,
    framerateDivisor,
    startingWidth,
    startingHeight,
    startingSize,
    targetFramerate,
    fd;
  double force_format,
    byteScaler;
  unsigned int n_buffers;
  double frameDelayMicros,
    frameDelayMillis;
  struct v4l2_requestbuffers req;
  enum v4l2_buf_type type;
  int index;
  unsigned char *outputFrame;
  char* device;
  std::vector<std::string> devNames;
};
struct buffer* buffersMain;
struct buffer* buffersAlt;
struct devInfo* devInfoMain;
struct devInfo* devInfoAlt;
bool experimentalMode = false;
std::mutex mtx;

unsigned char* outputFrameGreyscale = new unsigned char[1280*512];
unsigned char* outputFrameGreyscaleLeft = new unsigned char[640*512];
unsigned char* outputFrameGreyscaleRight = new unsigned char[640*512];
unsigned char* outputFrameRGB24 = new unsigned char[1280*512*3];
unsigned char* outputFrameRGB24Left = new unsigned char[640*512*3];
unsigned char* outputFrameRGB24Right = new unsigned char[640*512*3];
unsigned char* outputFrameRGB24Aligned = new unsigned char[640*512*3];
unsigned char* outputFrameRGBA32Aligned = new unsigned char[640*512*4];

// Define image resolution
const int image_width = 640;
const int image_height = 512;

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

void error(const char *msg) {
  perror(msg);
  exit(0);
}

void writeToStdoutBinary(const unsigned char* image, int width, int height) {
  std::cout.write(reinterpret_cast<const char*>(image), width * height);
}

void overlayFramesRGB24(const unsigned char* src1, const unsigned char* src2, unsigned char* dst, int width, int height, float alpha) {
  int numPixels = width * height * 3;
  for (int i = 0; i < numPixels; i += 3) {
    // Extract RGB components of the pixels in each frame
    unsigned char R1 = src1[i];
    unsigned char G1 = src1[i + 1];
    unsigned char B1 = src1[i + 2];
    unsigned char R2 = src2[i];
    unsigned char G2 = src2[i + 1];
    unsigned char B2 = src2[i + 2];
    // Apply alpha blending to overlay the pixels
    dst[i] = (R1 * alpha + R2 * (1.0f - alpha));
    dst[i + 1] = (G1 * alpha + G2 * (1.0f - alpha));
    dst[i + 2] = (B1 * alpha + B2 * (1.0f - alpha));
  }
}

void YU12ToRGB24(const unsigned char* yu12Data, int width, int height, unsigned char* rgb24Data) {
    if (!yu12Data || !rgb24Data || width <= 0 || height <= 0) {
        return;
    }
    int ySize = width * height;
    int uvSize = ySize / 4;
    const unsigned char* yPlane = yu12Data;
    const unsigned char* uPlane = yu12Data + ySize;
    const unsigned char* vPlane = yu12Data + ySize + uvSize;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int uvIndex = ((y / 2) * (width / 2)) + (x / 2);
            int yValue = yPlane[y * width + x];
            int uValue = uPlane[uvIndex];
            int vValue = vPlane[uvIndex];
            // YUV to RGB conversion
            int c = yValue - 16;
            int d = uValue - 128;
            int e = vValue - 128;
            int r = (298 * c + 409 * e + 128) >> 8;
            int g = (298 * c - 100 * d - 208 * e + 128) >> 8;
            int b = (298 * c + 516 * d + 128) >> 8;
            // Clamp RGB values to 0-255 range and write to the RGB24 buffer
            *rgb24Data++ = static_cast<unsigned char>(std::max(0, std::min(255, r)));
            *rgb24Data++ = static_cast<unsigned char>(std::max(0, std::min(255, g)));
            *rgb24Data++ = static_cast<unsigned char>(std::max(0, std::min(255, b)));
        }
    }
}
/*void YU12ToRGB24(const unsigned char* yu12Data, int width, int height, std::vector<unsigned char>& rgb24Data) {
    if (!yu12Data || width <= 0 || height <= 0) {
        return;
    }
    int ySize = width * height;
    int uvSize = ySize / 4;
    const unsigned char* yPlane = yu12Data;
    const unsigned char* uPlane = yu12Data + ySize;
    const unsigned char* vPlane = yu12Data + ySize + uvSize;
    rgb24Data.resize(width * height * 3);
    unsigned char* rgbPtr = &rgb24Data[0];
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int uvIndex = ((y / 2) * (width / 2)) + (x / 2);
            int yValue = yPlane[y * width + x];
            int uValue = uPlane[uvIndex];
            int vValue = vPlane[uvIndex];
            // YUV to RGB conversion
            int c = yValue - 16;
            int d = uValue - 128;
            int e = vValue - 128;
            int r = (298 * c + 409 * e + 128) >> 8;
            int g = (298 * c - 100 * d - 208 * e + 128) >> 8;
            int b = (298 * c + 516 * d + 128) >> 8;
            // Clamp RGB values to 0-255 range
            rgbPtr[0] = static_cast<unsigned char>(std::max(0, std::min(255, r)));
            rgbPtr[1] = static_cast<unsigned char>(std::max(0, std::min(255, g)));
            rgbPtr[2] = static_cast<unsigned char>(std::max(0, std::min(255, b)));

            rgbPtr += 3;
        }
    }
}*/

void NV12ToRGB24(const unsigned char* nv12Data, int width, int height, std::vector<unsigned char>& rgb24Data) {
    if (!nv12Data || width <= 0 || height <= 0) {
        return;
    }
    int ySize = width * height;
    int uvSize = ySize / 2;
    const unsigned char* yPlane = nv12Data;
    const unsigned char* uvPlane = nv12Data + ySize;
    rgb24Data.resize(width * height * 3);
    unsigned char* rgbPtr = &rgb24Data[0];
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int uvIndex = ((y / 2) * width) + (x & ~1);
            int yValue = yPlane[y * width + x];
            int uValue = uvPlane[uvIndex];
            int vValue = uvPlane[uvIndex + 1];
            // YUV to RGB conversion
            int c = yValue - 16;
            int d = uValue - 128;
            int e = vValue - 128;
            int r = (298 * c + 409 * e + 128) >> 8;
            int g = (298 * c - 100 * d - 208 * e + 128) >> 8;
            int b = (298 * c + 516 * d + 128) >> 8;
            // Clamp RGB values to 0-255 range
            rgbPtr[0] = static_cast<unsigned char>(std::max(0, std::min(255, r)));
            rgbPtr[1] = static_cast<unsigned char>(std::max(0, std::min(255, g)));
            rgbPtr[2] = static_cast<unsigned char>(std::max(0, std::min(255, b)));
            rgbPtr += 3;
        }
    }
}

void convertYUYVtoRGB(const unsigned char* src, unsigned char* dst, int width, int height) {
    int frameSize = width * height * 2; // 2 bytes per YUYV pixel
    int dstIndex = 0;
    for (int srcIndex = 0; srcIndex < frameSize; srcIndex += 4) {
        unsigned char Y1 = src[srcIndex];
        unsigned char U = src[srcIndex + 1];
        unsigned char Y2 = src[srcIndex + 2];
        unsigned char V = src[srcIndex + 3];
        int C = Y1 - 16;
        int D = U - 128;
        int E = V - 128;
        // First pixel (Y1, U, Y2, V)
        int R1 = (298 * C + 409 * E + 128) >> 8;
        int G1 = (298 * C - 100 * D - 208 * E + 128) >> 8;
        int B1 = (298 * C + 516 * D + 128) >> 8;
        // Clamp the values to [0, 255]
        dst[dstIndex] = (R1 < 0) ? 0 : (R1 > 255) ? 255 : R1;
        dst[dstIndex + 1] = (G1 < 0) ? 0 : (G1 > 255) ? 255 : G1;
        dst[dstIndex + 2] = (B1 < 0) ? 0 : (B1 > 255) ? 255 : B1;
        dstIndex += 3;
        // Second pixel (Y2, U, Y2, V)
        int C2 = Y2 - 16;
        int R2 = (298 * C2 + 409 * E + 128) >> 8;
        int G2 = (298 * C2 - 100 * D - 208 * E + 128) >> 8;
        int B2 = (298 * C2 + 516 * D + 128) >> 8;
        // Clamp the values to [0, 255]
        dst[dstIndex] = (R2 < 0) ? 0 : (R2 > 255) ? 255 : R2;
        dst[dstIndex + 1] = (G2 < 0) ? 0 : (G2 > 255) ? 255 : G2;
        dst[dstIndex + 2] = (B2 < 0) ? 0 : (B2 > 255) ? 255 : B2;
        dstIndex += 3;
    }
}

void YUYV422ToGRAY8(const unsigned char* srcImage, int srcWidth, int srcHeight, unsigned char* grayImage) {
    // YUYV422 format has 2 bytes per pixel, representing two consecutive pixels.
    // Each pixel has one Y component (luminance) and one U and V component (chrominance).
    // We only need the Y component to get the brightness (luminance) values.
    const unsigned char* srcPtr = srcImage;
    unsigned char* grayPtr = grayImage;
    for (int y = 0; y < srcHeight; y++) {
        for (int x = 0; x < srcWidth; x += 2) {
            // Get the Y components of the two consecutive pixels
            unsigned char y1 = srcPtr[0];
            unsigned char y2 = srcPtr[2];
            // Store the Y components directly into the grayscale image buffer
            grayPtr[0] = y1;
            grayPtr[1] = y2;
            // Move the source and destination pointers to the next pair of pixels
            srcPtr += 4;
            grayPtr += 2;
        }
    }
}

void RGB24ToGRAY8(const unsigned char* srcImage, int srcWidth, int srcHeight, unsigned char* grayImage) {
  for (int y = 0; y < srcHeight; y++) {
    for (int x = 0; x < srcWidth; x++) {
      int srcIndex = (y * srcWidth + x) * 3; // RGB24, 3 bytes per pixel
      int dstIndex = y * srcWidth + x;       // GRAY8, 1 byte per pixel
      unsigned char red = srcImage[srcIndex];
      unsigned char green = srcImage[srcIndex + 1];
      unsigned char blue = srcImage[srcIndex + 2];
      // Calculate the grayscale intensity using the formula
      unsigned char grayValue = static_cast<unsigned char>(0.299 * red + 0.587 * green + 0.114 * blue);
      grayImage[dstIndex] = grayValue;
    }
  }
}

void RGB24ToGRAYSCALERGB24(const unsigned char* srcImage, int srcWidth, int srcHeight, unsigned char* grayscaleRGB24Image) {
    if (!srcImage || !grayscaleRGB24Image || srcWidth <= 0 || srcHeight <= 0) {
        // Check for valid inputs
        return;
    }
    // Calculate the number of pixels in the image
    int numPixels = srcWidth * srcHeight;
    for (int i = 0; i < numPixels; ++i) {
        // Get the RGB values of the current pixel
        unsigned char r = srcImage[3 * i];
        unsigned char g = srcImage[3 * i + 1];
        unsigned char b = srcImage[3 * i + 2];
        // Calculate the grayscale intensity using the formula: Y = 0.2126*R + 0.7152*G + 0.0722*B, using the ITU-R BT.709 standard for luminance calculation
        //unsigned char grayscale = static_cast<unsigned char>(0.2126 * r + 0.7152 * g + 0.0722 * b);
        unsigned char grayscale = (r + g + b) / 3;
        // Set the same grayscale value for each channel (RGB) in the output image
        grayscaleRGB24Image[3 * i] = grayscale;
        grayscaleRGB24Image[3 * i + 1] = grayscale;
        grayscaleRGB24Image[3 * i + 2] = grayscale;
    }
}

void cropGRAY8Image(const unsigned char* srcImage, int srcWidth, int srcHeight, unsigned char* leftImage, unsigned char* rightImage) {
  int cropWidth = srcWidth / 2;
  int cropHeight = srcHeight;
  // Copy the left side of the image
  for (int y = 0; y < cropHeight; y++) {
    for (int x = 0; x < cropWidth; x++) {
      int srcIndex = y * srcWidth + x;
      int dstIndex = y * cropWidth + x;
      leftImage[dstIndex] = srcImage[srcIndex];
    }
  }
  // Copy the right side of the image
  for (int y = 0; y < cropHeight; y++) {
    for (int x = 0; x < cropWidth; x++) {
      int srcIndex = (y * srcWidth) + (x + cropWidth);
      int dstIndex = y * cropWidth + x;
      rightImage[dstIndex] = srcImage[srcIndex];
    }
  }
}

void cropImage(const unsigned char* srcImage, int srcWidth, int srcHeight, unsigned char* leftImage, unsigned char* rightImage) {
  int cropWidth = srcWidth / 2;
  int cropHeight = srcHeight;
  // Copy the left side of the image
  for (int y = 0; y < cropHeight; y++) {
    for (int x = 0; x < cropWidth; x++) {
      int srcIndex = (y * srcWidth + x) * 3; // RGB24, 3 bytes per pixel
      int dstIndex = (y * cropWidth + x) * 3;
      memcpy(&leftImage[dstIndex], &srcImage[srcIndex], 3);
    }
  }
  // Copy the right side of the image
  for (int y = 0; y < cropHeight; y++) {
    for (int x = 0; x < cropWidth; x++) {
      int srcIndex = ((y * srcWidth) + (x + cropWidth)) * 3; // RGB24, 3 bytes per pixel
      int dstIndex = (y * cropWidth + x) * 3;
      memcpy(&rightImage[dstIndex], &srcImage[srcIndex], 3);
    }
  }
}

void overlayGRAY8Images(const unsigned char* srcImage1, const unsigned char* srcImage2, int width, int height, unsigned char* resultImage) {
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int index = y * width + x;
      int pixelValue1 = srcImage1[index];
      int pixelValue2 = srcImage2[index];
      // Overlay the two images by adding their pixel values
      int overlayValue = (pixelValue1 + pixelValue2) / 2;
      resultImage[index] = overlayValue;
    }
  }
}

void overlay_rgb24_with_alpha(const unsigned char* topImage, unsigned char* bottomImage, int width, int height, double alpha) {
    // Iterate through each pixel of the images and perform the overlay
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int index = (y * width + x) * 3;
            // Extract RGB values from the top image
            const unsigned char topR = topImage[index];
            const unsigned char topG = topImage[index + 1];
            const unsigned char topB = topImage[index + 2];
            // Overlay the top pixel onto the bottom image using the specified alpha value
            const double alphaFactor = alpha;
            bottomImage[index]     = static_cast<unsigned char>(topR * alphaFactor + bottomImage[index] * (1.0 - alphaFactor));
            bottomImage[index + 1] = static_cast<unsigned char>(topG * alphaFactor + bottomImage[index + 1] * (1.0 - alphaFactor));
            bottomImage[index + 2] = static_cast<unsigned char>(topB * alphaFactor + bottomImage[index + 2] * (1.0 - alphaFactor));
        }
    }
    std::cout.write(reinterpret_cast<const char*>(bottomImage), 640 * 512 * 3);
}

// Function to overlay an RGBA32 image onto an RGB24 image
void overlay_rgba32_onto_rgb24_image(const unsigned char* alignedTopImage, unsigned char* bottomImage) {
    const int width = 640;
    const int height = 512;
    // Iterate through each pixel of the images and perform the overlay
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int alignedIndex = (y * width + x) * 4;
            const int bottomIndex = (y * width + x) * 3;
            // Extract RGB values from the aligned image
            const unsigned char topR = alignedTopImage[alignedIndex];
            const unsigned char topG = alignedTopImage[alignedIndex + 1];
            const unsigned char topB = alignedTopImage[alignedIndex + 2];
            const unsigned char alpha = alignedTopImage[alignedIndex + 3];
            // Overlay the aligned pixel onto the bottom image using alpha blending
            const double alphaFactor = static_cast<double>(alpha) / 255.0;
            bottomImage[bottomIndex] = static_cast<unsigned char>(topR * alphaFactor + bottomImage[bottomIndex] * (1.0 - alphaFactor));
            bottomImage[bottomIndex + 1] = static_cast<unsigned char>(topG * alphaFactor + bottomImage[bottomIndex + 1] * (1.0 - alphaFactor));
            bottomImage[bottomIndex + 2] = static_cast<unsigned char>(topB * alphaFactor + bottomImage[bottomIndex + 2] * (1.0 - alphaFactor));
        }
    }
}

// Function to resolve parallax and align the images
//void resolveParallax(const unsigned char* topImage, unsigned char* bottomImage, double verticalDistance_mm) {
void resolveParallax(const unsigned char* topImage, unsigned char* bottomImage, double verticalDistance) {
    // Convert vertical distance from millimeters to meters (for consistency with camera parameters)
    //double verticalDistance = verticalDistance_mm / 1000.0;
    const int width = 640;
    const int height = 512;
    // Camera parameters
    const double bottomHFOV = 48.1;
    const double bottomVFOV = 38.4;
    const double topHFOV = 69.0;
    const double topVFOV = 55.0;
    // Calculate focal lengths based on FOV
    const double bottomFocalLengthH = (width / 2.0) / tan(bottomHFOV * M_PI / 180.0);
    const double bottomFocalLengthV = (height / 2.0) / tan(bottomVFOV * M_PI / 180.0);
    const double topFocalLengthH = (width / 2.0) / tan(topHFOV * M_PI / 180.0);
    const double topFocalLengthV = (height / 2.0) / tan(topVFOV * M_PI / 180.0);
    // Calculate horizontal and vertical disparities
    const double horizontalDisparity = atan(static_cast<double>(verticalDistance) / bottomFocalLengthV);
    const double verticalDisparity = atan(static_cast<double>(verticalDistance) / bottomFocalLengthH);
    // Shift the top image to align with the bottom image
    //unsigned char* alignedTopImage = new unsigned char[width * height * 4];
    unsigned char* alignedTopImage = new unsigned char[width * height * 3];
    // Ensure the top coordinates are within bounds
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int topX = x - static_cast<int>(horizontalDisparity * (y - height / 2.0) / height);
            const int topY = y - static_cast<int>(verticalDisparity * (x - width / 2.0) / width);
            // Ensure the top coordinates are within bounds
            if (topX >= 0 && topX < width && topY >= 0 && topY < height) {
                for (int c = 0; c < 3; ++c) {
                    alignedTopImage[(y * width + x) * 3 + c] = topImage[(topY * width + topX) * 3 + c];
                }
            } else {
                // If the top coordinates are out of bounds, set the pixel to black (or any other background color)
                for (int c = 0; c < 3; ++c) {
                    alignedTopImage[(y * width + x) * 3 + c] = 0;
                }
            }
        }
    }
    /*for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int topX = x - static_cast<int>(horizontalDisparity * (y - height / 2.0) / height);
            const int topY = y - static_cast<int>(verticalDisparity * (x - width / 2.0) / width);
            // Ensure the top coordinates are within bounds
            if (topX >= 0 && topX < width && topY >= 0 && topY < height) {
                for (int c = 0; c < 3; ++c) {
                    alignedTopImage[(y * width + x) * 4 + c] = topImage[(topY * width + topX) * 3 + c];
                }
                // Set alpha channel to fully opaque (255)
                alignedTopImage[(y * width + x) * 4 + 3] = 255;
            } else {
                // If the top coordinates are out of bounds, set the pixel to fully transparent (0 alpha)
                for (int c = 0; c < 4; ++c) {
                    alignedTopImage[(y * width + x) * 4 + c] = 0;
                }
            }
        }
    }*/
    // Now the 'alignedTopImage' contains the aligned top image with respect to the bottom image.
    // You can further process or save the result as needed.
    //overlay_rgba32_onto_rgb24_image(alignedTopImage, bottomImage);
    overlay_rgb24_with_alpha(alignedTopImage, bottomImage, 640, 512, 0.5);
    // Don't forget to delete the dynamically allocated memory
    delete[] alignedTopImage;
}

// End of new test functions
void errno_exit(const char* s) {
  fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
  exit(EXIT_FAILURE);
}
int xioctl(int fh, int request, void* arg) {
  int r;
  do {
    r = ioctl(fh, request, arg);
  } while (-1 == r && EINTR == errno);
  return r;
}
void did_memory_allocate_correctly(struct devInfo*& devInfos) {
  if (devInfos->outputFrame == NULL) {
    fprintf(stderr, "[cap%d] Fatal: Memory allocation failed of output frame for device: %s..\nExiting now.\n", devInfos->index, devInfos->device);
    exit(1);
  }
}
int deinit_bufs(struct buffer*& buffers, struct devInfo*& devInfos) {
  for (unsigned int i = 0; i < devInfos->n_buffers; ++i)
    if (-1 == munmap(buffers[i].start, buffers[i].length))
      errno_exit("munmap");
  free(buffers);
  fprintf(stderr, "[cap%d] Uninitialized V4L2 device: %s\n", devInfos->index, devInfos->device);
  if (-1 == close(devInfos->fd))
    errno_exit("close");
  devInfos->fd = -1;
  fprintf(stderr, "[cap%d] Closed V4L2 device: %s\n", devInfos->index, devInfos->device);
  fprintf(stderr, "\n");
  return 0;
}
int init_dev_stage1(struct buffer*& buffers, struct devInfo*& devInfos) {
  fprintf(stderr, "\n[cap%d] Starting V4L2 capture testing program with the following V4L2 device: %s\n", devInfos->index, devInfos->device);
  struct stat st;
  if (-1 == stat(devInfos->device, &st)) {
    fprintf(stderr, "[cap%d] Cannot identify '%s': %d, %s\n", devInfos->index, devInfos->device, errno, strerror(errno));
    exit(EXIT_FAILURE);
  }
  if (!S_ISCHR(st.st_mode)) {
    fprintf(stderr, "[cap%d] %s is no device\n", devInfos->index, devInfos->device);
    exit(EXIT_FAILURE);
  }
  devInfos->fd = open(devInfos->device, O_RDWR | O_NONBLOCK, 0);
  if (-1 == devInfos->fd) {
    fprintf(stderr, "[cap%d] Cannot open '%s': %d, %s\n", devInfos->index, devInfos->device, errno, strerror(errno));
    exit(EXIT_FAILURE);
  }
  fprintf(stderr, "[cap%d] Opened V4L2 device: %s\n", devInfos->index, devInfos->device);
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min;
  if (-1 == xioctl(devInfos->fd, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL == errno) {
      fprintf(stderr, "[cap%d] %s is no V4L2 device\n", devInfos->index, devInfos->device);
      exit(EXIT_FAILURE);
    }
    else {
      errno_exit("VIDIOC_QUERYCAP");
    }
  }
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    fprintf(stderr, "[cap%d] %s is no video capture device\n", devInfos->index, devInfos->device);
    exit(EXIT_FAILURE);
  }
  // Select video input, video standard and tune here.
  CLEAR(cropcap);
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (0 == xioctl(devInfos->fd, VIDIOC_CROPCAP, &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; // reset to default
    if (-1 == xioctl(devInfos->fd, VIDIOC_S_CROP, &crop)) {
      switch (errno) {
      case EINVAL:
        // Cropping not supported.
        break;
      default:
        // Errors ignored.
        break;
      }
    }
  } else {
    // Errors ignored.
  }
  CLEAR(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fprintf(stderr, "[cap%d] Forcing format for %s to: %.1f\n", devInfos->index, devInfos->device, devInfos->force_format);
  if (devInfos->force_format) {
    if (devInfos->force_format == 3) {
      devInfos->byteScaler = devInfos->force_format;
      fmt.fmt.pix.width = devInfos->startingWidth;
      fmt.fmt.pix.height = devInfos->startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
      fmt.fmt.pix.field = V4L2_FIELD_NONE; // V4L2_FIELD_INTERLACED;
      fprintf(stderr, "[cap%d] Selected RGB24\n", devInfos->index);
    } else if (devInfos->force_format == 2) {
      devInfos->byteScaler = devInfos->force_format;
      fmt.fmt.pix.width = devInfos->startingWidth;
      fmt.fmt.pix.height = devInfos->startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
      fmt.fmt.pix.field = V4L2_FIELD_NONE; // V4L2_FIELD_INTERLACED;
      fprintf(stderr, "[cap%d] Selected YUYV\n", devInfos->index);
    } else if (devInfos->force_format == 1.5) {
      devInfos->byteScaler = devInfos->force_format;
      fmt.fmt.pix.width = devInfos->startingWidth;
      fmt.fmt.pix.height = devInfos->startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
      fmt.fmt.pix.field = V4L2_FIELD_NONE; // V4L2_FIELD_INTERLACED;
      fprintf(stderr, "[cap%d] Selected YUV420\n", devInfos->index);
    }
    if (-1 == xioctl(devInfos->fd, VIDIOC_S_FMT, &fmt))
      errno_exit("VIDIOC_S_FMT");
    // Note VIDIOC_S_FMT may change width and height.
  } else {
    // Preserve original settings as set by v4l2-ctl for example
    if (-1 == xioctl(devInfos->fd, VIDIOC_G_FMT, &fmt))
      errno_exit("VIDIOC_G_FMT");
  }
  // Buggy driver paranoia.
  min = fmt.fmt.pix.width * 2;
  if (fmt.fmt.pix.bytesperline < min)
    fmt.fmt.pix.bytesperline = min;
  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage < min)
    fmt.fmt.pix.sizeimage = min;
  //struct v4l2_requestbuffers req;
  CLEAR(devInfos->req);
  devInfos->req.count = 4;
  devInfos->req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  devInfos->req.memory = V4L2_MEMORY_MMAP;
  if (-1 == xioctl(devInfos->fd, VIDIOC_REQBUFS, &devInfos->req)) {
    if (EINVAL == errno) {
      fprintf(stderr, "[cap%d] %s does not support memory mapping\n", devInfos->index, devInfos->device);
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }
  if (devInfos->req.count < 2) {
    fprintf(stderr, "[cap%d] Insufficient buffer memory on %s\n", devInfos->index, devInfos->device);
    exit(EXIT_FAILURE);
  }
  return 0;
}
double convertFrameDelayToFPS(double frameDelay) {
    double microseconds = frameDelay;
    double fps = 1000000.0 / microseconds;
    return fps;
}
int init_dev_stage2(struct buffer*& buffers, struct devInfo*& devInfos) {
  if (!buffers) {
    fprintf(stderr, "[cap%d] Out of memory\n", devInfos->index);
    exit(EXIT_FAILURE);
  }
  for (devInfos->n_buffers = 0; devInfos->n_buffers < devInfos->req.count; ++devInfos->n_buffers) {
    struct v4l2_buffer buf;
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = devInfos->n_buffers;
    if (-1 == xioctl(devInfos->fd, VIDIOC_QUERYBUF, &buf))
      errno_exit("VIDIOC_QUERYBUF");
    buffers[devInfos->n_buffers].length = buf.length;
    buffers[devInfos->n_buffers].start = mmap(NULL /* start anywhere */, buf.length, PROT_READ | PROT_WRITE /* required */, MAP_SHARED /* recommended */, devInfos->fd, buf.m.offset);
    if (MAP_FAILED == buffers[devInfos->n_buffers].start)
      errno_exit("mmap");
  }
  // TODO: Implement a more proper way to handle settings in this area regarding if we really need DV timings to be set
  fprintf(stderr, "[cap%d] Support for general camera inputs (such as: %s) is not guaranteed!\n", devInfos->index, devInfos->device);
  unsigned int i;
  for (i = 0; i < devInfos->n_buffers; ++i) {
    struct v4l2_buffer buf;
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (-1 == xioctl(devInfos->fd, VIDIOC_QBUF, &buf))
      errno_exit("VIDIOC_QBUF");
  }
  devInfos->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(devInfos->fd, VIDIOC_STREAMON, &devInfos->type))
    errno_exit("VIDIOC_STREAMON");
  fprintf(stderr, "[cap%d] Initialized V4L2 device: %s\n", devInfos->index, devInfos->device);
  return 0;
}
int reinit_dev_stage2(struct devInfo*& devInfos, struct buffer*& bufs) {
  deinit_bufs(bufs, devInfos);
  init_dev_stage1(bufs, devInfos);
  init_dev_stage2(bufs, devInfos);
  devInfos->outputFrame = (unsigned char*)calloc((devInfos->startingWidth * devInfos->startingHeight * devInfos->byteScaler), sizeof(unsigned char)); // allocate memory for frame buffer
  did_memory_allocate_correctly(devInfos);
  return 0;
}
int get_frame(struct buffer* buffers, struct devInfo* devInfos) {
//  while (true) {
  fd_set fds;
  struct timeval tv;
  int r;
  FD_ZERO(&fds);
  FD_SET(devInfos->fd, &fds);
  // Timeout period to wait for device to respond
  tv.tv_sec = 10;
  tv.tv_usec = 0;
  r = select(devInfos->fd + 1, &fds, NULL, NULL, &tv);
  if (-1 == r) {
    if (EINTR == errno)
      // Do nothing
      //continue;
    errno_exit("select");
  }
  if (0 == r) {
    //fprintf(stderr, "[cap%d] select timeout\n", devInfos->index);
    //exit(EXIT_FAILURE);
    fprintf(stderr, "[cap%d] select timeout - recovery process starting..\n", devInfos->index);
    reinit_dev_stage2(devInfos, buffersMain);
    return 0;
    shouldLoop.store(false);
    return 1;
  }
  struct v4l2_buffer buf;
  unsigned int i;
  CLEAR(buf);
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (-1 == xioctl(devInfos->fd, VIDIOC_DQBUF, &buf)) {
    switch (errno) {
    case EAGAIN:
      fprintf(stderr, "[cap%d] EAGAIN\n", devInfos->index);
      return 0;
    case EIO:
      // Could ignore EIO, see spec.
      // fall through
    default:
      errno_exit("VIDIOC_DQBUF");
    }
  }
  assert(buf.index < devInfos->n_buffers);
  std::memcpy(devInfos->outputFrame, (unsigned char*)buffers[buf.index].start, buffers[buf.index].length); // copy frame data to frame buffer
  if (-1 == xioctl(devInfos->fd, VIDIOC_QBUF, &buf))
    errno_exit("VIDIOC_QBUF");
  // EAGAIN - continue select loop
  CLEAR(i);
  CLEAR(buf);
  CLEAR(r);
  CLEAR(tv);
  CLEAR(fds);
//  }
  return 0;
}
int init_vars(struct devInfo*& devInfos, struct buffer*& bufs, char*& dev_name, const int width, const int height, const double byteScaler, const int index) {
  devInfos = (devInfo*)calloc(1, sizeof(*devInfos));
  devInfos->device = (char*)calloc(sizeof(dev_name)+1, sizeof(char));
  strcpy(devInfos->device, dev_name);
  devInfos->framerate = 25;
  devInfos->framerateDivisor = 1;
  devInfos->startingWidth = width;
  devInfos->startingHeight = height;
  devInfos->force_format = byteScaler;
  devInfos->startingSize = (devInfos->startingWidth * devInfos->startingHeight * devInfos->byteScaler);
  devInfos->targetFramerate = 25;
  devInfos->fd = -1;
  devInfos->index = index;
  devInfos->byteScaler = byteScaler;
  init_dev_stage1(buffersMain, devInfos);
  bufs = (buffer*)calloc(devInfos->req.count, sizeof(*bufs));
  init_dev_stage2(bufs, devInfos);
  devInfos->outputFrame = (unsigned char*)calloc((devInfos->startingWidth * devInfos->startingHeight * devInfos->byteScaler), sizeof(unsigned char)); // allocate memory for frame buffer
  did_memory_allocate_correctly(devInfos);
  return 0;
}
void cleanup_vars() {
  deinit_bufs(buffersMain, devInfoMain);
}
void configure_vars(struct devInfo*& deviMain, struct buffer*& bufMain, struct devInfo*& deviAlt, struct buffer*& bufAlt, int argCnt, char **args) {
  fprintf(stderr, "[main] Initializing..\n");
  // allocate memory for structs
  init_vars(deviMain, bufMain, args[1], atoi(args[2]), atoi(args[3]), (double)atof(args[4]), 0);
  init_vars(deviAlt, bufAlt, args[5], atoi(args[6]), atoi(args[7]), (double)atof(args[8]), 1);
  shouldLoop.store(true);
  usleep(1000);
}
int main(const int argc, char** argv) {
  system("rw");
  configure_vars(devInfoMain, buffersMain, devInfoAlt, buffersAlt, argc, argv);
  usleep(1000);
  fprintf(stderr, "\n[main] Starting main loop now\n");
  /*background_task_cap_main = std::async(std::launch::async, get_frame, buffersMain, devInfoMain);
  background_task_cap_main.wait();
  convertYUYVtoRGB(devInfoMain->outputFrame, outputFrameRGB24, 1280, 512);
  cropImage(outputFrameRGB24, 1280, 512, outputFrameRGB24Left, outputFrameRGB24Right);
  RGB24ToGRAYSCALERGB24(outputFrameRGB24Left, 1280, 512, outputFrameRGB24Left);
  background_task_cap_alt = std::async(std::launch::async, get_frame, buffersAlt, devInfoAlt);
  background_task_cap_alt.wait();
  YU12ToRGB24(devInfoAlt->outputFrame, 640, 512, outputFrameRGB24);*/
  // Set the horizontal and vertical FOVs for both cameras
  /*double hfov1 = 48.1; // Horizontal FOV of camera 1 in degrees
  double vfov1 = 38.4; // Vertical FOV of camera 1 in degrees
  double hfov2 = 69.0; // Horizontal FOV of camera 2 in degrees
  double vfov2 = 55.0; // Vertical FOV of camera 2 in degrees*/
  double blendVal = 0.5;
  while (true) {
    while (shouldLoop) {
      if (blendVal >= 1.0) {
        blendVal = 0.0;
      }
      blendVal += 0.01;
      background_task_cap_main = std::async(std::launch::async, get_frame, buffersMain, devInfoMain);
      background_task_cap_main.wait();
      //YUYV422ToGRAY8(devInfoMain->outputFrame, 1280, 512, outputFrameGreyscale);
      convertYUYVtoRGB(devInfoMain->outputFrame, outputFrameRGB24, 1280, 512);
      //std::cout.write(reinterpret_cast<const char*>(outputFrameRGB24), 1280 * 512 * 3);
      cropImage(outputFrameRGB24, 1280, 512, outputFrameRGB24Left, outputFrameRGB24Right);
      //RGB24ToGRAYSCALERGB24(outputFrameRGB24Left, 1280, 512, outputFrameRGB24Left);
      //cropGRAY8Image(outputFrameGreyscale, 1280, 512, outputFrameGreyscaleLeft, outputFrameGreyscaleRight);
      //writeToStdoutBinary(outputFrameGreyscale, 1280, 512);
      //cropGRAY8Image(const unsigned char* srcImage, int srcWidth, int srcHeight, unsigned char* leftImage, unsigned char* rightImage)
      background_task_cap_alt = std::async(std::launch::async, get_frame, buffersAlt, devInfoAlt);
      background_task_cap_alt.wait();
      //std::cout.write(reinterpret_cast<const char*>(devInfoAlt->outputFrame), 640 * 512 * 1.5);
      YU12ToRGB24(devInfoAlt->outputFrame, 640, 512, outputFrameRGB24);
      //overlayFramesRGB24(outputFrameRGB24, outputFrameRGB24Left, outputFrameRGB24Right, 640, 512, 0.0F);
      //overlayFramesRGB24(outputFrameRGB24, outputFrameRGB24Left, outputFrameRGB24Right, 640, 512, 1.0F);
      //resolveParallax(outputFrameRGB24, outputFrameRGB24Left, -300);
      overlay_rgb24_with_alpha(outputFrameRGB24, outputFrameRGB24Left, 640, 512, blendVal);
      //overlayFramesRGB24(outputFrameRGB24, outputFrameRGB24Left, outputFrameRGB24Right, 640, 512, 0.25);
      //overlayFramesRGB24(outputFrameRGB24, outputFrameRGB24Left, outputFrameRGB24Right, 640, 512, 0.5);
      //std::cout.write(reinterpret_cast<const char*>(outputFrameRGB24), 640 * 512 * 3);
      //std::cout.write(reinterpret_cast<const char*>(outputFrameRGB24Left), 640 * 512 * 3);
      //std::cout.write(reinterpret_cast<const char*>(outputFrameRGB24Left), 640 * 512 * 3);
//      std::cout.write(reinterpret_cast<const char*>(outputFrameRGB24Right), 640 * 512 * 3);
      //std::cout.write(reinterpret_cast<const char*>(devInfoAlt->outputFrame), 640 * 512 * 3);
      /*overlayGRAY8Images(outputFrameGreyscaleRight, devInfoMain->outputFrame, 640, 512, outputFrameGreyscale);
      writeToStdoutBinary(outputFrameGreyscale, 640, 512);*/
      //writeToStdoutBinary(devInfoAlt->outputFrame, 640, 512);
      //writeToStdoutBinary(outputFrameGreyscaleRight, 640, 512);
      //writeToStdoutBinary(outputFrameGreyscale, 1280, 512);
      //memcpy(fbmem, devInfoMain->outputFrame, screensize);
    }
    usleep(1000000);
    cleanup_vars();
    usleep(1000000);
    fprintf(stderr, "\n[main] Reinitializing main, please wait..\n");
    configure_vars(devInfoMain, buffersMain, devInfoAlt, buffersAlt, argc, argv);
    usleep(1000);
    fprintf(stderr, "\n[main] Restarting main loop now\n");
    background_task_cap_main.wait();
  }
  return 0;
}
