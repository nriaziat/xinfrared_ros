#include "xtherm.h"

using namespace std;
using namespace cv;

#define TRUE 1
#define FALSE 0

XTherm::XTherm()
{
    XTherm(XTHERM_OUTPUTMODE::OUTPUTMODE_FULL);
}

XTherm::XTherm(XTHERM_OUTPUTMODE mode)
{
    OUTPUTMODE = mode;

    if (traversalVideo() == FALSE)
    {
        printf("Init failed.\n");
        exit(EXIT_FAILURE);
    }
    irBuffers = (irBuffer *)malloc(4 * sizeof(*irBuffers));
    // irBuffers = new irBuffer[4];
    if (!irBuffers)
    {
        printf("Out of memory.\n");
        return;
    }
    if (OUTPUTMODE == XTHERM_OUTPUTMODE::OUTPUTMODE_FULL)
    {
        SimplePictureProcessingInit(IMAGEWIDTH, (IMAGEHEIGHT - 4));
        SetParameter(100, 0.5f, 0.1f, 0.1f, 1.0f, 3.5f);
    }
    for (n_buffers = 0; n_buffers < 4; n_buffers++)
    {
        if (OUTPUTMODE == XTHERM_OUTPUTMODE::OUTPUTMODE_FULL)
        {
            irBuffers[n_buffers].midVar = (size_t **)calloc(7, sizeof(size_t *));
            // irBuffers[n_buffers].midVar = new size_t *[7];
            SimplePictureProcessingInitMidVar(irBuffers[n_buffers].midVar);
        }
        irBuffers[n_buffers].destBuffer = (unsigned char *)calloc(IMAGEWIDTH * (IMAGEHEIGHT - 4) * 4, sizeof(unsigned char));
        // irBuffers[n_buffers].destBuffer = new unsigned char[IMAGEWIDTH * (IMAGEHEIGHT - 4) * 4];
    }
    temperatureData = (float *)calloc(IMAGEWIDTH * (IMAGEHEIGHT - 4) + 10, sizeof(float));
    // temperatureData = new float[IMAGEWIDTH * (IMAGEHEIGHT - 4) + 10];
    if (v4l2_grab() == FALSE)
    {
        printf("Grab failed.\n");
        exit(EXIT_FAILURE);
    }
    if (OUTPUTMODE == XTHERM_OUTPUTMODE::OUTPUTMODE_FULL)
    {
        if (v4l2_control(0x8004) == FALSE) // output raw data
        {
            printf("Control failed. \n");
            exit(EXIT_FAILURE);
        }
    }
    else
    {
        if (v4l2_control(0x8005) == FALSE) // output yuyv
        {
            printf("Control failed. \n");
            exit(EXIT_FAILURE);
        }
    }
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // Stream or Buffer Type, constantly as V4L2_BUF_TYPE_VIDEO_CAPTURE
    buf.memory = V4L2_MEMORY_MMAP;          // Memory Mapping mode，set as V4L2_MEMORY_MMAP
    delayy = 0;

    rgbImg = cv::Mat(IMAGEHEIGHT - 4, IMAGEWIDTH, CV_8UC4);      // 8004 mode: professional algorithm formed rgba image, CV with bgra format. So there is channel deviation in pseudo color, except Black Hot and White Hot.
                                                                 // 8005 mode: convert yuyv into bgra imagep
    orgLumiImgL = cv::Mat(IMAGEHEIGHT - 4, IMAGEWIDTH, CV_8UC1); // In 8004 mode, generate greyscale via linear algorithm.
    orgRgbImgP = cv::Mat(IMAGEHEIGHT - 4, IMAGEWIDTH, CV_8UC4);  // In 8004 mode, generate rgba image via linear algorithm using 256 pesudo color, cv using bgra format, there is channel deviation
    rgbImgP = cv::Mat(IMAGEHEIGHT - 4, IMAGEWIDTH, CV_8UC4);     // 8004 mode generate rgba image via linear algorithm using 448 pesudo color, cv using bgra format, there is channel deviation
    yuvImg.create(IMAGEHEIGHT - 4, IMAGEWIDTH, CV_8UC2);

    printf("Initialization complete.\n");
}
XTherm::~XTherm()
{
    for (n_buffers = 0; n_buffers < 4; n_buffers++)
    {
        // Release buffers captured by professional image algorithm
        if (OUTPUTMODE == XTHERM_OUTPUTMODE::OUTPUTMODE_FULL)
        {
            SimplePictureProcessingDeinit();
            if (irBuffers[n_buffers].midVar != NULL)
            {
                SimplePictureProcessingDeinitMidVar(irBuffers[n_buffers].midVar);
                free(irBuffers[n_buffers].midVar);
                irBuffers[n_buffers].midVar = NULL;
            }
        }
        if (irBuffers[n_buffers].destBuffer != NULL)
        {
            free(irBuffers[n_buffers].destBuffer);
            irBuffers[n_buffers].destBuffer = NULL;
        }
        // end - Release buffers

        if (temperatureData != NULL)
        {
            free(temperatureData);
            temperatureData = NULL;
        }
    }

    v4l2_release(); // Calling VIDIOC_ STREAMOFF to stop capturing video, video device driver stops capturing video
}
int XTherm::getFrame(cv::Mat &frame)
{
    // fd = open(videoXConst.c_str(), O_RDWR);
    for (n_buffers = 0; n_buffers < 4; n_buffers++)
    {
        buf.index = n_buffers;
        ioctl(fd, VIDIOC_DQBUF, &buf);
        delayy++;
        // printf("delayy:%d\n", delayy);
        // Determine if the data is complete, and discard the incomplete data
        // printf("buf bytesused:%d",buf.bytesused);
        if (buf.bytesused != IMAGEHEIGHT * IMAGEWIDTH * 2)
        {
            // enqueueing cycle
            ioctl(fd, VIDIOC_QBUF, &buf);
            printf("throw err data, incomplete data\n");
            printf("ioctl failed and returned errno \"%s\" \n", strerror(errno));
            return 0;
        }
        if (delayy == 51 || delayy == 52 || delayy == 53) // The parameters passed in the next frame after setting the parameters may not be complete. Throw them away
        {
            // enqueueing cycle
            ioctl(fd, VIDIOC_QBUF, &buf);
            printf("throw data, error setting\n");
            printf("ioctl failed and returned errno \"%s\" \n", strerror(errno));
            return 0;
        }

        if (delayy == 126 || delayy == 127 || delayy == 128) // The parameters passed in the next frame after saving the parameters may not be complete. Throw them away
        {
            // enqueueing cycle
            ioctl(fd, VIDIOC_QBUF, &buf);
            printf("throw data, error saving\n");
            printf("ioctl failed and returned errno \"%s\" \n", strerror(errno));
            return 0;
        }

        buffers[n_buffers].timestamp = buf.timestamp.tv_sec * 1000000 + buf.timestamp.tv_usec;
        cur_time = buffers[n_buffers].timestamp;
        extra_time = cur_time - last_time;
        last_time = cur_time;

        unsigned short *orgData = (unsigned short *)buffers[n_buffers].start;
        unsigned short *fourLinePara = orgData + IMAGEWIDTH * (IMAGEHEIGHT - 4); //  Parameters in last four lines
        int amountPixels = 0;
        switch (IMAGEWIDTH)
        {
        case 384:
            amountPixels = IMAGEWIDTH * (4 - 1);
            return 0;
        case 240:
            amountPixels = IMAGEWIDTH * (4 - 3);
            return 0;
        case 256:
            amountPixels = IMAGEWIDTH * (4 - 3);
            return 0;
        case 640:
            amountPixels = IMAGEWIDTH * (4 - 1);
            return 0;
        }
        memcpy(&shutTemper, fourLinePara + amountPixels + 1, sizeof(unsigned short));
        floatShutTemper = shutTemper / 10.0f - 273.15f;                               //  Shutter Temperature
        memcpy(&coreTemper, fourLinePara + amountPixels + 2, sizeof(unsigned short)); //  Shell temperature
        floatCoreTemper = coreTemper / 10.0f - 273.15f;
        // printf("cpyPara  floatShutTemper:%f,floatCoreTemper:%f,floatFpaTmp:%f\n", floatShutTemper, floatCoreTemper, floatFpaTmp);
        memcpy((unsigned short *)cameraSoftVersion, fourLinePara + amountPixels + 24, 16 * sizeof(uint8_t)); // camera soft version
        // printf("cameraSoftVersion:%s\n", cameraSoftVersion);
        memcpy((unsigned short *)sn, fourLinePara + amountPixels + 32, 32 * sizeof(uint8_t)); // SN
        // printf("sn:%s\n", sn);
        int userArea = amountPixels + 127;
        memcpy(&correction, fourLinePara + userArea, sizeof(float)); //  Correction
        userArea = userArea + 2;
        memcpy(&Refltmp, fourLinePara + userArea, sizeof(float)); //   Reflection temperature
        userArea = userArea + 2;
        memcpy(&Airtmp, fourLinePara + userArea, sizeof(float)); //    Ambient temperature
        userArea = userArea + 2;
        memcpy(&humi, fourLinePara + userArea, sizeof(float)); //   Humidity
        userArea = userArea + 2;
        memcpy(&emiss, fourLinePara + userArea, sizeof(float)); //   Emissivity
        userArea = userArea + 2;
        memcpy(&distance, fourLinePara + userArea, sizeof(unsigned short)); //   Distance
        // printf("Airtmp:%f,correction:%f,distance:%d,emiss:%f,Refltmp:%f,humi:%f\n", Airtmp, correction, distance, emiss, Refltmp, humi);

        if (delayy % 4500 == 30) //    Shutter calibrates once every three minutes, for body temperature measuring, once per minute
        {
            if (v4l2_control(0x8000) == FALSE)
            {
                printf("shutter fail~~\n");
            }
        }
        if (delayy % 4500 == 25) // Recalculate the table during the five frames before shutter calibrates, otherwise may be errors
        {
            /*thermometryT(           IMAGEWIDTH,
                        IMAGEHEIGHT,
                        temperatureTable,
                        orgData,
                        &floatFpaTmp,
                        & correction,
                        & Refltmp,
                        & Airtmp,
                        & humi,
                        & emiss,
                        & distance,
                        cameraLens,
                        shutterFix
                                rangeMode);*/
            // Calculate table with last four lines
            thermometryT4Line(IMAGEWIDTH,
                              IMAGEHEIGHT,
                              temperatureTable,
                              fourLinePara,
                              &floatFpaTmp,
                              &correction,
                              &Refltmp,
                              &Airtmp,
                              &humi,
                              &emiss,
                              &distance,
                              cameraLens,
                              shutterFix,
                              rangeMode);
            if (delayy > 9000)
            {
                delayy = 0;
            }
        }
        thermometrySearch(IMAGEWIDTH, IMAGEHEIGHT, temperatureTable, orgData, temperatureData, rangeMode, OUTPUTMODE);
        printf("centerTmp:%.2f,maxTmp:%.2f,minTmp:%.2f,avgTmp:%.2f,point1Tmp:%.2f\n", temperatureData[0], temperatureData[3], temperatureData[6], temperatureData[9], temperatureData[7]);

        amountPixels = IMAGEWIDTH * (IMAGEHEIGHT - 4);
        unsigned short detectAvg = orgData[amountPixels];
        amountPixels++;
        unsigned short fpaTmp = orgData[amountPixels];
        amountPixels++;
        unsigned short maxx1 = orgData[amountPixels];
        amountPixels++;
        unsigned short maxy1 = orgData[amountPixels];
        amountPixels++;
        unsigned short max = orgData[amountPixels];
        amountPixels++;
        unsigned short minx1 = orgData[amountPixels];
        amountPixels++;
        unsigned short miny1 = orgData[amountPixels];
        amountPixels++;
        unsigned short min = orgData[amountPixels];
        amountPixels++;
        unsigned short avg = orgData[amountPixels];

        unsigned char *orgOutput = orgRgbImgP.data;
        int ro = (max - min) > 0 ? (max - min) : 1;
        int avgSubMin = (avg - min) > 0 ? (avg - min) : 1;
        int maxSubAvg = (max - avg) > 0 ? (max - avg) : 1;
        int ro1 = (avg - min) > 97 ? 97 : (avg - min);
        int ro2 = (max - avg) > 157 ? 157 : (max - avg);
        for (int i = 0; i < IMAGEHEIGHT - 4; i++)
        {
            for (int j = 0; j < IMAGEWIDTH; j++)
            {
                // printf("i:%d,j:%d\n",i,j);
                // Black&WHite: Grayscale (0-254), single channel.  paletteIronRainbow (0-254), triple channels. Both are 255, so take 254
                int gray = 0;
                if (orgData[i * IMAGEWIDTH + j] > avg)
                {
                    gray = (int)(ro2 * (orgData[i * IMAGEWIDTH + j] - avg) / maxSubAvg + 97);
                }
                else
                {
                    gray = (int)(ro1 * (orgData[i * IMAGEWIDTH + j] - avg) / avgSubMin + 97);
                }
                orgLumiImgL.at<uchar>(i, j) = (uchar)gray;
                int intGray = (int)gray;
                int paletteNum = 3 * intGray;
                orgOutput[4 * (i * IMAGEWIDTH + j)] = (unsigned char)paletteIronRainbow[paletteNum + 2];
                orgOutput[4 * (i * IMAGEWIDTH + j) + 1] = (unsigned char)paletteIronRainbow[paletteNum + 1];
                orgOutput[4 * (i * IMAGEWIDTH + j) + 2] = (unsigned char)paletteIronRainbow[paletteNum];
                orgOutput[4 * (i * IMAGEWIDTH + j) + 3] = 1;
            }
        }
        orgOutput = rgbImgP.data;
        ro = (max - min) > 0 ? (max - min) : 1;
        avgSubMin = (avg - min) > 0 ? (avg - min) : 1;
        maxSubAvg = (max - avg) > 0 ? (max - avg) : 1;
        ro1 = (avg - min) > 170 ? 170 : (avg - min);
        ro2 = (max - avg) > 276 ? 276 : (max - avg);
        for (int i = 0; i < IMAGEHEIGHT - 4; i++)
        {
            for (int j = 0; j < IMAGEWIDTH; j++)
            {
                // printf("i:%d,j:%d\n",i,j);
                //  paletteHighContrast（0-448）×3三通道，所以使用447  triple channels, so take 447
                int gray = 0;
                if (orgData[i * IMAGEWIDTH + j] > avg)
                {
                    gray = (int)(ro2 * (orgData[i * IMAGEWIDTH + j] - avg) / maxSubAvg + 170);
                }
                else
                {
                    gray = (int)(ro1 * (orgData[i * IMAGEWIDTH + j] - avg) / avgSubMin + 170);
                }
                orgLumiImgL.at<uchar>(i, j) = (uchar)gray;
                int intGray = (int)gray;
                int paletteNum = 3 * intGray;

                orgOutput[4 * (i * IMAGEWIDTH + j)] = (unsigned char)paletteHighContrast[paletteNum + 2];
                orgOutput[4 * (i * IMAGEWIDTH + j) + 1] = (unsigned char)paletteHighContrast[paletteNum + 1];
                orgOutput[4 * (i * IMAGEWIDTH + j) + 2] = (unsigned char)paletteHighContrast[paletteNum];
                orgOutput[4 * (i * IMAGEWIDTH + j) + 3] = 1;
            }
        }
        if (delayy == 50)
        {
            sendDistance(3);
        }
        if (delayy == 60)
        {
            setPoint(IMAGEWIDTH / 2, (IMAGEHEIGHT - 4) / 2, 0);
        }

        if (delayy == 110) //  Shutter calibrates
        {
            if (v4l2_control(0x8000) == FALSE)
            {
                printf("shutter fail~~\n");
            }
        }
        if (delayy == 120) // 使用新的参数重新计算表   Calculate on table with new parameters
        {
            // 用后四行参数来计算表    Calculate on table with last four lines of parameters
            thermometryT4Line(IMAGEWIDTH,
                              IMAGEHEIGHT,
                              temperatureTable,
                              fourLinePara,
                              &floatFpaTmp,
                              &correction,
                              &Refltmp,
                              &Airtmp,
                              &humi,
                              &emiss,
                              &distance,
                              cameraLens,
                              shutterFix,
                              rangeMode);
        }
        if (delayy == 125) // Save parameters
        {
            savePara();
        }

        // Following codes for converting yuyv outputs into bgr in 8005 mode, using OpenCV
        if (OUTPUTMODE == XTHERM_OUTPUTMODE::OUTPUTMODE_SIMPLE)
        {
            memcpy(yuvImg.data, (unsigned char *)orgData, (IMAGEHEIGHT - 4) * 2 * IMAGEWIDTH * sizeof(unsigned char));
            cv::cvtColor(yuvImg, rgbImg, cv::COLOR_YUV2RGB_YUYV);
        }

        // Following codes for computing rgba outputs in 8004 mode; using OpenCV to show bgra, so there are differences in channel
        if (OUTPUTMODE == XTHERM_OUTPUTMODE::OUTPUTMODE_FULL)
        {
            Compute(orgData, rgbImg.data, 0, irBuffers[n_buffers].midVar); // 0-6 to change platte
        }
        ioctl(fd, VIDIOC_QBUF, &buf);
        // There are two buffer queues inside driver: input queues and output queues
        // For capture device, when buffer in input queue is fulled by data, it will transfer to output queue
        // printf("VIDIOC_QBUF~~\n");                      //  Calling VIDIOC_DQBUF after data processed
        // Calling VIDIOC_DQBUF again, place buffer into output queue
    }
    frame.data = rgbImg.data;
    return 1;
}

int XTherm::getTemperatureData(float *temperatureData)
{
    memcpy(temperatureData, this->temperatureData, IMAGEWIDTH * (IMAGEHEIGHT - 4) * sizeof(float));
    return 1;
}

int XTherm::close_device()
{
    for (n_buffers = 0; n_buffers < 4; n_buffers++)
    {
        // 释放专业图像算法占用的资源   Release buffers captured by professional image algorithm
        if (OUTPUTMODE == XTHERM_OUTPUTMODE::OUTPUTMODE_FULL)
        {
            SimplePictureProcessingDeinit();
            if (irBuffers[n_buffers].midVar != NULL)
            {
                SimplePictureProcessingDeinitMidVar(irBuffers[n_buffers].midVar);
                free(irBuffers[n_buffers].midVar);
                irBuffers[n_buffers].midVar = NULL;
            }
        }
        if (irBuffers[n_buffers].destBuffer != NULL)
        {
            free(irBuffers[n_buffers].destBuffer);
            irBuffers[n_buffers].destBuffer = NULL;
        }
        // end -释放专业图像算法占用的资源   Release buffers

        if (temperatureData != NULL)
        {
            free(temperatureData);
            temperatureData = NULL;
        }
    }

    v4l2_release(); // 停止视频采集命令，应用程序调用VIDIOC_ STREAMOFF停止视频采集命令后，视频设备驱动程序不在采集视频数据。
                    // Calling VIDIOC_ STREAMOFF to stop capturing video, video device driver stops capturing video
    return 0;
}

int XTherm::init_v4l2(string videoX)
{
    videoXConst = videoX.c_str();
    if ((fd = open(videoXConst.c_str(), O_RDWR)) == -1) // Open Video1
    {
        printf("Error opening V4L interface\n");
        return FALSE;
    }
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) // Query capabilities of video device
    {
        printf("Unable to query camera capabilities.\n");
        return FALSE;
    }
    else
    {
        printf("Driver Caps:\n"
               "  Driver: \"%s\"\n"
               "  Card: \"%s\"\n"
               "  Bus: \"%s\"\n"
               "  Version: %d\n"
               "  Capabilities: %x\n",
               cap.driver,
               cap.card,
               cap.bus_info,
               cap.version,
               cap.capabilities);
        string str = "";
        str = (char *)cap.card;

        if (str.find("T3") == -1)
        {
            return FALSE;
        }
    }

    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    printf("Supported video formats: \n");
    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) != -1) // Capture current video format of the device
    {
        printf("\t%d. %s\n", fmtdesc.index + 1, fmtdesc.description);
        fmtdesc.index++;
    }
    // set fmt
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = IMAGEWIDTH;
    fmt.fmt.pix.height = IMAGEHEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV; //  Calling V4L2_PIX_FMT_YUYV
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) // Set video data format, such as Width, Height of image, format (JPEG, YUYV etc)
    {
        printf("Error setting pixel format\n");
        return FALSE;
    }
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) == -1) // Get video format
    {
        printf("Unable to get format.\n");
        return FALSE;
    }
    IMAGEWIDTH = fmt.fmt.pix.width;   //  fix Width
    IMAGEHEIGHT = fmt.fmt.pix.height; //   fix Height
    printf("IMAGEWIDTH:%d,IMAGEHEIGHT:%d\n", IMAGEWIDTH, IMAGEHEIGHT);
    memset(&stream_para, 0, sizeof(struct v4l2_streamparm));
    stream_para.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    stream_para.parm.capture.timeperframe.denominator = 25;
    stream_para.parm.capture.timeperframe.numerator = 1;

    if (ioctl(fd, VIDIOC_S_PARM, &stream_para) == -1)
    {
        printf("Unable to set frame rate\n");
        return FALSE;
    }
    if (ioctl(fd, VIDIOC_G_PARM, &stream_para) == -1)
    {
        printf("Unable to get frame rate\n");
        return FALSE;
    }
    {
        printf("Frame Rate: numerator:%d\ndenominator:%d\n", stream_para.parm.capture.timeperframe.numerator, stream_para.parm.capture.timeperframe.denominator);
    }

    return TRUE;
}

int XTherm::v4l2_grab(void)
{
    // 4  request for 4buffers , count of buffer >= 2
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) // Start memory map or pointer I/O
    {
        printf("Error requesting buffer. \n");
        return FALSE;
    }
    // 5 mmap for buffers
    buffers = (buffer *)malloc(req.count * sizeof(*buffers));
    // buffers = new buffer[req.count];
    if (!buffers)
    {
        printf("Out of memory! \n");
        return FALSE;
    }
    unsigned int n_buffers;
    for (n_buffers = 0; n_buffers < req.count; n_buffers++)
    {
        // struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;
        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) // Search info of allocated V4L2 video buffer, including status, offset address,buffer length etc
        {
            printf("Error querying buffer.\n");
            printf("ioctl failed and returned errno %s \n", strerror(errno));
            return FALSE;
        }
        buffers[n_buffers].length = buf.length;

        buffers[n_buffers].start = (unsigned char *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);

        if (buffers[n_buffers].start == MAP_FAILED)
        {
            printf("Buffer map error\n");
            return FALSE;
        }
    }
    // 6 queue
    for (n_buffers = 0; n_buffers < req.count; n_buffers++)
    {
        buf.index = n_buffers;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd, VIDIOC_QBUF, &buf)) //  Put in empty video buffer to input queue of video buffer
        {
            printf("Error querying buffer.\n");
            return FALSE;
        }
    }
    // 7 starting
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) //
    {
        printf("Error starting video stream.\n");
        return FALSE;
    }
    return TRUE;
}
int XTherm::v4l2_control(int value)
{
    ctrl.id = V4L2_CID_ZOOM_ABSOLUTE;
    ctrl.value = value; // change output mode 0x8004/0x8005
    // shutter 0x8000
    if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) == -1)
    {
        printf("v4l2_control error\n");
        return FALSE;
    }
    return TRUE;
}
int XTherm::traversalVideo(void)
{
    // string device = "/dev";
    // string video = "video";
    string video = FILE_VIDEO1;
    string device = FILE_DEV;
    char *KEY_PTR = (char *)video.c_str();
    char *FILE_PTR = (char *)device.c_str();

    DIR *dir;
    struct dirent *ptr;
    char base[1000];

    if ((dir = opendir(FILE_PTR)) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }

    while ((ptr = readdir(dir)) != NULL)
    {
        string name = "";
        name = (char *)ptr->d_name;
        if (name.find("video") != string::npos)
        {
            string allName = device + "/" + name;
            if (init_v4l2(allName))
            {
                closedir(dir);
                return 1;
            }
            printf("d_name:%s/%s\n", FILE_PTR, ptr->d_name);
        }
        // printf("d_name:%s/%s\n", FILE_PTR, ptr->d_name);
    }

    closedir(dir);
    return 0;
}
void XTherm::sendCorrection(float correction)
{
    unsigned char iputCo[4];
    memcpy(iputCo, &correction, sizeof(float));
    sendFloatCommand(0 * 4, iputCo[0], iputCo[1], iputCo[2], iputCo[3], 20, 40, 60, 80, 120);
    printf("sendCorrection 0:%d,1:%d,2:%d,3:%d\n", iputCo[0], iputCo[1], iputCo[2], iputCo[3]);
}
void XTherm::sendReflection(float reflection)
{
    unsigned char iputRe[4];
    memcpy(iputRe, &reflection, sizeof(float));
    sendFloatCommand(1 * 4, iputRe[0], iputRe[1], iputRe[2], iputRe[3], 20, 40, 60, 80, 120);
}
void XTherm::sendAmb(float amb)
{
    unsigned char iputAm[4];
    memcpy(iputAm, &amb, sizeof(float));
    sendFloatCommand(2 * 4, iputAm[0], iputAm[1], iputAm[2], iputAm[3], 20, 40, 60, 80, 120);
}
void XTherm::sendHumidity(float humidity)
{
    unsigned char iputHu[4];
    memcpy(iputHu, &humidity, sizeof(float));
    sendFloatCommand(3 * 4, iputHu[0], iputHu[1], iputHu[2], iputHu[3], 20, 40, 60, 80, 120);
}
void XTherm::sendEmissivity(float emiss)
{
    unsigned char iputEm[4];
    memcpy(iputEm, &emiss, sizeof(float));
    sendFloatCommand(4 * 4, iputEm[0], iputEm[1], iputEm[2], iputEm[3], 20, 40, 60, 80, 120);
}

void XTherm::sendDistance(unsigned short distance)
{
    unsigned char iputDi[2];
    memcpy(iputDi, &distance, sizeof(unsigned short));
    sendUshortCommand(5 * 4, iputDi[0], iputDi[1]);
}
void XTherm::savePara()
{
    v4l2_control(0x80ff);
}
int XTherm::v4l2_release()
{
    unsigned int n_buffers;
    enum v4l2_buf_type type;

    //  Video stream OFF
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMOFF, &type);

    // Memory map OFF
    for (n_buffers = 0; n_buffers < 4; n_buffers++)
    {
        munmap(buffers[n_buffers].start, buffers[n_buffers].length);
    }

    // Free buffers
    free(buffers);

    //  Device OFF
    close(fd);
    return TRUE;
}

void XTherm::sendFloatCommand(int position, unsigned char value0, unsigned char value1, unsigned char value2, unsigned char value3, int interval0,
                              int interval1, int interval2, int interval3, int interval4)
{
    int psitionAndValue0 = (position << 8) | (0x000000ff & value0);
    printf("psitionAndValue0:%X\n", psitionAndValue0);
    // v4l2_control(psitionAndValue0);
    if (v4l2_control(psitionAndValue0) == FALSE)
    {
        printf("control fail psitionAndValue0~~\n");
        exit(EXIT_FAILURE);
    }
    int psitionAndValue1 = ((position + 1) << 8) | (0x000000ff & value1);
    printf("psitionAndValue1:%X\n", psitionAndValue1);
    if (v4l2_control(psitionAndValue1) == FALSE)
    {
        printf("control fail psitionAndValue1~~\n");
        exit(EXIT_FAILURE);
    }
    int psitionAndValue2 = ((position + 2) << 8) | (0x000000ff & value2);
    printf("psitionAndValue2:%X\n", psitionAndValue2);
    if (v4l2_control(psitionAndValue2) == FALSE)
    {
        printf("control fail psitionAndValue2~~\n");
        exit(EXIT_FAILURE);
    }
    int psitionAndValue3 = ((position + 3) << 8) | (0x000000ff & value3);
    printf("psitionAndValue3:%X\n", psitionAndValue3);
    if (v4l2_control(psitionAndValue3) == FALSE)
    {
        printf("control fail psitionAndValue3~~\n");
        exit(EXIT_FAILURE);
    }
}
void XTherm::sendUshortCommand(int position, unsigned char value0, unsigned char value1)
{
    int psitionAndValue0 = (position << 8) | (0x000000ff & value0);
    printf("psitionAndValue0:%X\n", psitionAndValue0);
    // v4l2_control(psitionAndValue0);
    if (v4l2_control(psitionAndValue0) == FALSE)
    {
        printf("control fail psitionAndValue0~~\n");
        exit(EXIT_FAILURE);
    }
    int psitionAndValue1 = ((position + 1) << 8) | (0x000000ff & value1);
    printf("psitionAndValue1:%X\n", psitionAndValue1);
    if (v4l2_control(psitionAndValue1) == FALSE)
    {
        printf("control fail psitionAndValue1~~\n");
        exit(EXIT_FAILURE);
    }
}
void XTherm::sendByteCommand(int position, unsigned char value0, int interval0)
{
    int psitionAndValue0 = (position << 8) | (0x000000ff & value0);
    v4l2_control(psitionAndValue0);
}
void XTherm::setPoint(int viewX1, int viewY1, int indexOfPoint)
{
    int x1, y1;
    switch (indexOfPoint)
    {
    case 0:
        x1 = 0xf000 + viewX1;
        y1 = 0xf200 + viewY1;
        break;
    case 1:
        x1 = 0xf400 + viewX1;
        y1 = 0xf600 + viewY1;
        break;
    case 2:
        x1 = 0xf800 + viewX1;
        y1 = 0xfa00 + viewY1;
        break;
    }
    v4l2_control(x1);
    v4l2_control(y1);
}