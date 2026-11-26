// Writes the deterministic synthetic tracking sequence to a Motion-JPEG AVI file.
// Usage: af_make_fixture <output.avi> [frames=90]

#include "../support/synthetic.hpp"
#include <opencv2/videoio.hpp>
#include <cstdio>
#include <cstdlib>

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::fprintf(stderr, "usage: %s <output.avi> [frames]\n", argv[0]);
    return 2;
  }
  const int frames = argc > 2 ? std::atoi(argv[2]) : 90;
  if (frames <= 0)
  {
    std::fprintf(stderr, "frames must be > 0\n");
    return 2;
  }

  cv::VideoWriter writer(argv[1], cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0,
                         cv::Size(synth::kWidth, synth::kHeight));
  if (!writer.isOpened())
  {
    std::fprintf(stderr, "cannot open %s for writing\n", argv[1]);
    return 1;
  }
  for (int i = 0; i < frames; ++i)
  {
    writer.write(synth::tracking_frame(i));
  }
  writer.release();
  std::printf("wrote %d frames to %s\n", frames, argv[1]);
  return 0;
}
