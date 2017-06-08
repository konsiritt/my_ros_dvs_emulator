#ifndef CONFIG_DVS_H
#define CONFIG_DVS_H

// define dvs variables here

#define image_width 320 //640
#define image_height 240 //480

#define pbo_count 2

// configure pixelBuffer and dvsEmulator:
#define pixel_format GL_BGRA //Fastest option for map from vram to ram
#define channel_size 4
#define dvs_threshold 60

#endif // CONFIG_DVS_H

