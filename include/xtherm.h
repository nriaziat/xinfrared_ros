#ifndef XTHERM_H
#define XTHERM_H

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <linux/v4l2-controls.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <stdbool.h>
#include <math.h>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/core/core_c.h>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "thermometry.h"
#include "SimplePictureProcessing.h"

using namespace std;
using namespace cv;

#define MAX_BUFFER 2048

enum XTHERM_OUTPUTMODE
{
    /**
     * OUTPUTMODE 4:Combine with libthermometry.so to output all around temperature data.
     *              Obtain high quality image with professional algorithms from simple.so, requires basic frequency at least 1.8ghz.
     *              Linear Algorithms produce lower quality image than professional algorithms but there are almost no requirements in basic frequency.
     *              Output data format: in yuyu form, the actual transferred is 16bit NUC data with 14 bits to store the wide dynamic grayscale value of one pixel.
     *              The last four lines of yuyu data are parameters.
     *
     *  OUTPUTMODE 5:With libthermometry.so to directly output temperature of the center, highest, lowest and extra three points. Can't output full frame temperature data.
     *               Output data format: graphs in yuyv format, the last four lines are parameters.
     */
    OUTPUTMODE_FULL = 4,
    OUTPUTMODE_SIMPLE = 5
};
class XTherm
{
public:
    XTherm();
    XTherm(XTHERM_OUTPUTMODE mode);
    ~XTherm();

    XTHERM_OUTPUTMODE OUTPUTMODE = OUTPUTMODE_FULL;
    string FILE_VIDEO1 = "video";
    string FILE_DEV = "/dev";
    uint IMAGEWIDTH = 384;
    uint IMAGEHEIGHT = 292;
    int fd;                      // Descriptor of device
    string videoXConst;

    int getFrame(Mat &frame);
    int getTemperatureData(float *temperatureData);
    int close_device();

private:
    int delayy;
    unsigned int n_buffers;
    /**
     *temperatureData:
     *     The final output temperature data, in the form of "10 + Full Frame Temperature Data"; such as 10+384(width)×288(height), the top 10 as below
     *temperatureData[0]=centerTmp;
     *temperatureData[1]=(float)maxx1;
     *temperatureData[2]=(float)maxy1;
     *temperatureData[3]=maxTmp;
     *temperatureData[4]=(float)minx1;
     *temperatureData[5]=(float)miny1;
     *temperatureData[6]=minTmp;
     *temperatureData[7]=point1Tmp;
     *temperatureData[8]=point2Tmp;
     *temperatureData[9]=point3Tmp;
     *Search on the table with 8004 or 8005 mode, 8005 mode only outputs the 10 parameters above, 8004 mode include above parameters with overall temperature data
     *For example, point (x, y), row x, column y, location in temperatureData[10+width*x+y], origin is (0,0)
     *Refer to function thermometrySearch
     */
    float *temperatureData;
    /**
     * Set three points
     * temperatureData[7]，temperatureData[8]，temperatureData[9] shows temperature
     *0<=viewX1<IMAGEWIDTH
     *0<=viewY1<IMAGEHEIGHT-4
     */
    enum v4l2_buf_type type; //  Type of buffer
    struct v4l2_control ctrl;
    /**
     *temperatureTable:
     */
    float temperatureTable[16384];

    struct v4l2_streamparm stream_para; // Struct v4l2_streamparm to describe the property of video stream
    struct v4l2_capability cap;         // Obtain capability of device, check if it has video input or audio input/output abilities.
    struct v4l2_fmtdesc fmtdesc;        // image format:  VIDIOC_ENUM_FMT    Enumerate image format the deivce supported: VIDIOC_ENUM_FMT
    struct v4l2_format fmt, fmtack;     // struct v4l2_pix_format：V4L2_PIX_FMT_YYUV V4L2_PIX_FMT_YUYV
                                        // Substruct struct v4l2_pix_format for setting Height, Width and Type of captured video
    struct v4l2_requestbuffers req;     // Send request of frame buffer from driver, including amount of requests
    struct v4l2_buffer buf;             // One frame in the driver
    struct buffer                       // Capture data buffer from Camera
    {
        void *start;
        unsigned int length;
        long long int timestamp;
    } *buffers;
    struct irBuffer // The buffer required by professional algorithm
    {
        size_t **midVar;
        unsigned char *destBuffer;
    } *irBuffers;

    cv::Mat rgbImg;
    cv::Mat orgLumiImgL;
    cv::Mat orgRgbImgP;
    cv::Mat rgbImgP;
    cv::Mat yuvImg;

    //  Refer to thermometry.h for relevant parameters
    int rangeMode = 120;
    float floatFpaTmp;
    float correction;
    float Refltmp;
    float Airtmp;
    float humi;
    float emiss;
    unsigned short distance;
    // The following cameraLens is 68: 1,the output is 384×292, lens =<6.8mm 2, the output is 256×196, lens =<4mm.  Other default 130
    int cameraLens = 130;
    float shutterFix = 0;
    // end -

    int i = 100;
    double t;
    long long int extra_time = 0;
    long long int cur_time = 0;
    long long int last_time = 0;

    char sn[32];                //    camera serial number
    char cameraSoftVersion[16]; //    camera Software Version
    unsigned short shutTemper;
    float floatShutTemper; //     Shutter Temperature
    unsigned short coreTemper;
    float floatCoreTemper;                                    //    Shell temperature
    const unsigned char *paletteIronRainbow = getPalette(0);  // 256*3    Iron Rainbow
    const unsigned char *palette3 = getPalette(1);            // 256*3     Rainbow 1
    const unsigned char *paletteRainbow = getPalette(2);      // 224*3     Rainbow 2
    const unsigned char *paletteHighRainbow = getPalette(3);  // 448*3    HDR rainbow
    const unsigned char *paletteHighContrast = getPalette(4); // 448*3     High Contrast rainbow

    int init_v4l2(string videoX); // Initialization
    int v4l2_grab(void);          // Capture
    int v4l2_control(int);        // Control
    int traversalVideo(void);
    // Traveral video, may add determine statements if there are multiple UVC devices, weither infrared UVC

    void sendCorrection(float correction); // Set correction, usally -3.0/3.0 to correct temperature lower or higher

    void sendReflection(float reflection); // Set reflection temperature, usually ambient temperature

    /*How to find out reflection temperature:
    When there is no heat source nearby, the object reflects ambient temperature, so usually refection equals ambient temperature.
    When there is heat source, object reflects temperature of heat source. How to know the reflection temperature:
    1)Take an aluminum foil as Mirror to reflect infrared ray, wrinkle the foil and then flat it. Put the foil on cardboard, the bright surface upwards
    2)Adjust the emissivity of device to 1;
    3)Parallel foil with object, measure surface temperature of foil with thermal imager. The measured temperature is reflection temperature.
    */

    void sendAmb(float amb); // Set ambient temperature

    void sendHumidity(float humidity); // Set humidity (0-1.0), usually 0.45

    void sendEmissivity(float emiss); // Emissivity(0-1.0), refer to emissivity table

    void sendDistance(unsigned short distance); // Distance (Unit: Meter)
    void sendFloatCommand(int position, unsigned char value0, unsigned char value1, unsigned char value2, unsigned char value3, int interval0,
                          int interval1, int interval2, int interval3, int interval4);
    void sendUshortCommand(int position, unsigned char value0, unsigned char value1);
    void sendByteCommand(int position, unsigned char value0, int interval0);
    void setPoint(int viewX1, int viewY1, int indexOfPoint);
    void savePara(); // Save parameter to the chipset
    int v4l2_release();
};
#endif // XTHERM_H