#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define _BASETSD_H

#include "rknn_api.h"
#include "rknn_yolov5_demo/preprocess.h"
#include "rknn_yolov5_demo/postprocess.h"
#include "rknn_yolov5_demo/Timer.h"

#define PERF_WITH_POST 1

class Detection
{
private:

    int ret;
    rknn_context ctx;
    size_t actual_size = 0;
    int img_width = 0;
    int img_height = 0;
    int img_channel = 0;
    const float nms_threshold = NMS_THRESH;      // 默认的NMS阈值
    const float box_conf_threshold = BOX_THRESH; // 默认的置信度阈值
    char* model_name = "/home/firefly/obj_dec/src/rknn_yolov5_demo/model/RK3588/yolov5s-640-640.rknn";

    cv::Mat orig_img;
    rknn_input_output_num io_num;
    
    int channel = 3;
    int width = 0;
    int height = 0;

    rknn_input inputs[1];
    
    std::vector<float> out_scales;
    std::vector<int32_t> out_zps;
    
public:
    Detection();
    static unsigned char *load_data(FILE *fp, size_t ofst, size_t sz);
    static unsigned char *load_model(const char *filename, int *model_size);
    detect_result_group_t outputParse(cv::Mat netInputImg);


};