#include <visor/Gist/gist_bridge.hpp>

void convert_IplImage_to_LEAR(IplImage* src, color_image_t* dst) {
  //assert(src->width == 123 && src->height == 123);
  //assert(src->depth == IPL_DEPTH_8U);
  RgbImage imgA(src);
  int x, y, i = 0;
  for (y = 0; y < src->height; y++) {
    for (x = 0; x < src->width; x++) {
      dst->c1[i] = imgA[y][x].r;
      dst->c2[i] = imgA[y][x].g;
      dst->c3[i] = imgA[y][x].b;
      i++;
    }
  }
  assert(i == (src->width * src->height));
}//*/
