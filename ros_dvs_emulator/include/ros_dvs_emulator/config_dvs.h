#ifndef CONFIG_DVS_H
#define CONFIG_DVS_H

// define dvs variables here

#define image_width 320 //640
#define image_height 240 //480

#define pbo_count 2

// configure pixelBuffer and dvsEmulator:
#define pixel_format GL_BGRA //Fastest option for map from vram to ram
#define channel_size 4
#define dvs_threshold 1.3 // 60 for lin values

// define to what value on linear rgb scale the log should be linear
#define lin_log_lim 20

// define frames_float if the color values should be obtained in
// GL_FLOAT [0,1] format from GPU instead of GL_UNSIGNED_BYTE [0,255]
#define frames_float

#endif // CONFIG_DVS_H

