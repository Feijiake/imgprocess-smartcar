//
// Created by qing on 24-10-29.
//

#ifndef IMG_PROCESS_H
#define IMG_PROCESS_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <chrono>


typedef unsigned char       uint8;
typedef unsigned short       uint16;
typedef  short         int16;
typedef char         int8;
#define ImageUsed   *PerImg_ip
#define RESULT_ROW 120 //逆透视结果图像的高度
#define RESULT_COL 188 //逆透视结果图像的宽度
#define border_max		use_w-1 //边界宽度
#define border_min		1 //边界宽度
#define         USED_COL                188
#define         USED_ROW                120
#define image_h	120   //原始图像大小
#define image_w	188
#define compressimage_h	60  //压缩后的图像大小
#define compressimage_w	99
#define use_h compressimage_h   //要处理的图像大小 image_h 或者compressimage_h
#define use_w compressimage_w
#define max(a,b)	(((a) > (b)) ? (a) : (b))
#define min(a,b)	(((a) < (b)) ? (a) : (b))
#define start_h use_h - 1 //八领域开始的生长y
//可使用的
//#include <ros/ros.h>

class img_process_object
{
public:


    /*逆透视*/
    cv::Mat PerImg_ip;  // 逆透视后的图像

    /*
     * 函数作用:初始化逆透视矩阵,得到映射关系
     * 入口参数:无
     * 使用示例：processor.initializeMappingTable();
     * 备注: 预计算获得映射表
     */
    void initializeMappingTable();

    /*
     * 函数作用:更新逆透视图像的映射关系
     * 入口参数:要修改的hsv_th标志位,要修改的hsv的上限,hsv的下限无
     * 使用示例：processor.updatePerspectiveImage();
     * 备注: 使用预计算的映射表更新逆透视图像
     */
    void updatePerspectiveImage();

    /*逆透视*/

    /*元素*/
    #define  RightCirque_TowPoint_Flag 1
    #define  LeftCirque_TowPoint_Flag 1
    bool CirqueOff = false;
    uint8 RightCirque_Flag = 0;
    uint8 LeftCirque_Flag = 0;
    uint8 Straight_Flag = 0;
    uint8 Cross_Flag = 0;
    uint8 Stop_Flag = 0;

    /*元素*/

    /*寻线*/
    #define USE_num use_h*3
    uint8 offline = 20;//截至行,截至行后的数据为无效数据
    uint16 data_stastics_l = 0;
    uint16 data_stastics_r = 0;

    uint8 center_line[use_h] ={};//中线数组
    float center_err[use_h];//中线误差数组
    uint8 l_border[use_h];//左边线
    uint8 r_border[use_h];//右边线
    uint16 points_l[(uint16)USE_num][2] = { {  0 } };
    uint16 points_r[(uint16)USE_num][2] = { {  0 } };
    uint16 dir_r[(uint16)USE_num] = { 0 };
    uint16 dir_l[(uint16)USE_num] = { 0 };


    /*
     * 函数作用:寻线函数
     * 入口参数:无
     * 使用示例：processor.find_line();
     * 备注: 寻线的主逻辑函数,所有寻线函数在此处调用
     */
    void find_line();//寻迹
    /*
     * 函数作用:用sobel算子获得八邻域开始点
     * 入口参数:找点的灰度图像
     * 使用示例：processor.get_start_point_sobel();
     * 备注: 适合灰度八领域
     */
    uint8 get_start_point_sobel(const cv::Mat& img);
    /*
     * 函数作用:寻找二值化跳变点获得八邻域开始点
     * 入口参数:找点的灰度图像
     * 使用示例：processor.get_start_point();
     * 备注: 适合二值化八领域
     */
    uint8_t get_start_point(const cv::Mat& bin_image, int start_row);
    /*寻线*/

    /*绘图*/
    /*
     * 函数作用:在图像上画出边线
     * 入口参数:绘制的彩色图像 边线数组 颜色 绘制的点数
     * 使用示例：processor.draw_line_1(processor.disp_color_image,processor.l_border,cv::Scalar(255, 0, 0), image_h - 1);
     * 备注:
     */
    void draw_line_1(cv::Mat& image, const uint8_t border[], cv::Scalar color, int num);
    /*绘图*/

    /*调试信息*/
    uint16 points_test[4][2];
    /*调试信息*/

    /*二值化*/
    /*二值化*/

    /*颜色查找*/
    //颜色阈值
    std::array<short, 6> green_hsv_th = {0, 250, 0, 250, 0, 250};
    std::array<short, 6> red_hsv_th = {0, 250, 0, 250, 0, 250};
    std::array<short, 6> blue_hsv_th = {0, 250, 0, 250, 0, 250};

    /*
     * 函数作用:修改hsv_th数组的值
     * 入口参数:要修改的hsv_th标志位,要修改的hsv的上限,hsv的下限
     * 使用示例：imgProcessor.change_color_hsv("green",85,255,255,35,100,100);
     * 备注:
     */
    bool change_color_hsv(const std::string& change_flag,short h_max,short s_max,short v_max,short h_min,short s_min,short v_min);
    /*
     * 函数作用:寻找最大色块
     * 入口参数:输入图像,hsv_th的值
     * 返回值:找到的最大色块的中心坐标
     * 备注:函数内有色块面积的限幅
     */
    std::array<float,2> findLargestColoContour(const cv::Mat& image,std::array<short,6> hsv_th);
    /*颜色查找*/

    /*接口类*/
    float Det_True;
    /*接口类*/

    /*初始化*/
    cv::Mat mt9v03x_image;//使用的灰度图像
    cv::Mat disp_color_image;//显示的彩色图像
    /*
     * 函数作用:加载图像进入类
     * 入口参数:输入的灰度图像
     * 使用示例：processor.setImage(color_image);
     * 备注:
     */
    void setImage(const cv::Mat& inputImage);

    img_process_object();
    /*初始化*/

private:

    /*颜色查找*/
    //查找色块后获得的点的坐标位置
    std::array<short, 6> positon_red = {0, 0};
    std::array<short, 6> positon_blue = {0, 0};
    std::array<short, 6> positon_green = {0, 0};

    /*颜色查找*/

    /*逆透视*/
    std::vector<std::vector<cv::Point>> mappingTable;  // 存储每个点的映射关系
    double change_un_Mat[3][3] ={{-0.371672,0.191928,1.624000},{-0.004298,0.013590,-7.454951},{-0.000330,0.002199,-0.331952}};//逆透视矩阵
    /*逆透视*/

    /*元素*/
    uint8 straight_variance_acc_th = 10;
    void Element_Detection(const cv::Mat& inputImage);//元素检测
    void Staight_Detection(const cv::Mat& inputImage);
    void cross_fill(const cv::Mat &inputImage, uint8 *l_border, uint8 *r_border, uint16 total_num_l,
    uint16 total_num_r, uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2]);
    /*元素*/



    /*前瞻*/
    uint8 set_towpoint = 45;
    uint8 TowPoint_True;
    void GetDet(const uint8 speed_now,const uint8 speed_min);
        /*前瞻*/

    /*寻线*/

    uint8 target_center[60] = {
        96, 127, 0, 0, 72, 16, 23, 70, 42, 39,
        39, 40, 40, 39, 40, 40, 40, 39, 40, 40,
        40, 40, 40, 40, 40, 40, 40, 39, 40, 40,
        40, 40, 40, 40, 40, 41, 41, 42, 43, 43,
        44, 45, 45, 46, 47, 47, 48, 48, 49, 49,
        49, 49, 49, 49, 49, 49, 49, 49, 49, 0
    };;//实际的赛道期望中线,当车在赛道正中间时,图像的中线不一定是赛道的中线,需要实际测量
    uint8 line_detect_l = 0;//从找到的第一个边线数组开始,有多少个有效点
    uint8 line_detect_r = 0;
    uint8 line_detect_center = 0;//中线数组的有效点
    int sobel_th = 7; //sobel 阈值
    uint8 start_cow_l = use_w/2;//sobel算子左边寻线开始行
    uint8 start_cow_r = use_w/2;
    uint8 start_point_l[2] = { 0 };//八领域种子生长开始的点
    uint8 start_point_r[2] = { 0 };
    uint8 hightest = 0;//八领域图像最高点

    /*
     * 函数作用:中线滤波,用补线的方式把到达截至行之前的中线补出来
     * 入口参数:无
     * 使用示例：RouteFilter();
     * 备注: 在find_line()中调用
     */
    void RouteFilter();
    /*
    * 函数作用:计算当前点的sobel值
    * 入口参数:输入图像 x坐标 y坐标
    * 使用示例：calculateSobelAtPixel(img, col, start_row)
    * 备注: 用来计算当前sobel值并与sobel阈值比较可以判断是否为边线
    */
    int calculateSobelAtPixel(const cv::Mat& src, int x, int y);
    /*
    * 函数作用:二值化八领域算法
    * 入口参数:最多计算多少个点,使用的图像,左起始点的x,y,右起始点的x,y，图像的最高点
    * 使用示例：search_l_r((uint16)USE_num, temp, start_point_l[0], start_point_l[1],
                   start_point_r[0], start_point_r[1], &hightest);
    * 备注: 用来计算当前sobel值并与sobel阈值比较可以判断是否为边线
    */
    void search_l_r(
    uint16_t break_flag,
    const cv::Mat& image,
    uint8_t l_start_x,
    uint8_t l_start_y,
    uint8_t r_start_x,
    uint8_t r_start_y,
    uint8_t* hightest
);
    /*
    * 函数作用:提取左边线
    * 入口参数:左边线一共有多少点
    * 使用示例：get_left(data_stastics_l);
    * 备注: 在find_line中调用
    */
    void get_left(uint16 total_L);//提取左边线
    /*
    * 函数作用:提取右边线
    * 入口参数右边线一共有多少点
    * 使用示例：get_right(data_stastics_r);
    * 备注: 在find_line中调用
    */
    void get_right(uint16 total_R);//提取右边线
    /*
     * 函数作用:获得中线
     * 入口参数:无
     * 使用示例：get_center();
     * 备注:在find_line中调用
     */
    void get_center();

    /*寻线*/

    /*二值化*/
    typedef int otsuTh_eight_type;
    typedef uint8_t make_sizetype;
    typedef int Threshold_min_type;

    //局部OSTU
    uint8_t break_flag = 200;
    make_sizetype make_size = 3;//卷积核大小
    Threshold_min_type Threshold_min = 12;//阈值
    /*
     * 函数作用:局部大津法
     * 入口参数:算子大小 输入图像 运算点的x,y 最低像素值
     * 使用示例：
     * 备注:超出边界的部分认为是黑点
     */
    static otsuTh_eight_type otsuThreshold_average(make_sizetype make_size, const cv::Mat& img, uint16_t x, uint16_t y, Threshold_min_type Threshold_min);
    /*
     * 函数作用:优化后的大津法
     * 入口参数:输入图像 阈值
     * 使用示例：
     * 备注:
     */
    uint8_t Threshold_deal(const cv::Mat& image, uint32_t pixel_threshold);//优化后的大津法
    /*
    * 函数作用:分区域进行大津法
    * 入口参数:输入图像 输出图像 静态阈值 大津法阈值
    * 使用示例：
    * 备注:
    */
    void Get01change_dajin(const cv::Mat& inputImage, cv::Mat& outputImage, int threshold_static, int threshold_detach);
    /*
    * 函数作用:分区域进行二值化
    * 入口参数:输入图像 输出图像 静态阈值
    * 使用示例：
    * 备注:观察计算出静态阈值
    */
    void Get01change(const cv::Mat& inputImage, cv::Mat& outputImage, int thresholdStatic);
    /*二值化*/

    /*预处理*/
    void image_draw_rectan(cv::Mat& image);
    /*预处理*/

    /*调试信息*/
    void print_average_huidu(const cv::Mat& img, make_sizetype make_size, Threshold_min_type Threshold_min);
    /*调试信息*/

    /*数学工具*/
    float Slope_Calculate(uint8 begin, uint8 end, uint8 *border);
    void calculate_s_i(uint8 start, uint8 end, uint8 *border, float *slope_rate, float *intercept);
    #define limit_a_b(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

     /*数学工具*/

    /*接口类*/
    float Weighting[10] = {0.96, 0.92, 0.88, 0.83, 0.77,0.71, 0.65, 0.59, 0.53, 0.47};//10行权重参数，随意更改，基本不影响，大致按照正态分布即可
    /*接口类*/
};
img_process_object::img_process_object(): mt9v03x_image(USED_ROW, USED_COL, CV_8UC1, cv::Scalar(0))  {

}

/*
 * 函数作用:修改hsv_th的值
 * 入口参数:要修改的对应颜色的标志位,h_max.s_max,v_max,h_min,s_min,v_min
 */
bool img_process_object::change_color_hsv(const std::string& change_flag, short h_max, short s_max, short v_max, short h_min, short s_min, short v_min)
{
    if (change_flag == "red") {
        red_hsv_th[0] = h_max;
        red_hsv_th[1] = s_max;
        red_hsv_th[2] = v_max;
        red_hsv_th[3] = h_min;
        red_hsv_th[4] = s_min;
        red_hsv_th[5] = v_min;
    }
    else if (change_flag == "green") {
        green_hsv_th[0] = h_max;
        green_hsv_th[1] = s_max;
        green_hsv_th[2] = v_max;
        green_hsv_th[3] = h_min;
        green_hsv_th[4] = s_min;
        green_hsv_th[5] = v_min;
    }
    else if (change_flag == "blue") {
        blue_hsv_th[0] = h_max;
        blue_hsv_th[1] = s_max;
        blue_hsv_th[2] = v_max;
        blue_hsv_th[3] = h_min;
        blue_hsv_th[4] = s_min;
        blue_hsv_th[5] = v_min;
    }
    else {
//        ROS_WARN("fall color flag: %s", change_flag.c_str());
        return false;  // 无效标志，返回 false
    }

    return true;  // 成功修改
}
std::array<float,2> img_process_object::findLargestColoContour(const cv::Mat& image, std::array<short, 6> hsv_th)
{
    cv::Mat hsvImage;
    cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);

    // 定义绿色的阈值范围
    cv::Scalar upperColor(hsv_th[0], hsv_th[1], hsv_th[2]); // 上界
    cv::Scalar lowerColor(hsv_th[3], hsv_th[4], hsv_th[5]); // 下界

    // 根据阈值创建掩膜
    cv::Mat mask;
    cv::inRange(hsvImage, lowerColor, upperColor, mask);

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double maxArea = 0;
    cv::Rect largestRect;

    // 检查是否找到轮廓
    if (contours.empty()) {
//        ROS_INFO("NO CONTOURS");
        return {-1, -1};  // 返回无效坐标
    }

    for (const auto& cnt : contours) {
        double area = cv::contourArea(cnt);
        if (area > maxArea) {
            maxArea = area;
            largestRect = cv::boundingRect(cnt);  // 更新最大轮廓的边界矩形
//            ROS_INFO("find counter,area=(%f)", area);
        }
    }
    int areaThreshold_min = 0;
    int areaThreshold_max = 20000;
    // 检查最大矩形的面积是否有效
    if (maxArea < areaThreshold_min || maxArea > areaThreshold_max) {
//        ROS_INFO("least(%f) < th (%d)，or >biggest(%d).", maxArea, areaThreshold_min,areaThreshold_max);
        return {-1, -1};  // 返回无效坐标
    }
    cv::rectangle(image, largestRect, cv::Scalar(255, 0, 0), 2);  // 使用绿色矩形框标记
    //cv::imshow("a",image);
    //cv::waitKey(1);
    float centerX = largestRect.x + largestRect.width / 2.0;
    float centerY = largestRect.y + largestRect.height / 2.0;
    //ROS_INFO("最大轮廓x=%f,y=%f:", centerX,centerY);
    return {centerX, centerY};  // 返回中心坐标
}


cv::Mat *PerImg_ip[RESULT_ROW][RESULT_COL];
/*
* 作用:根据逆透视矩阵得到逆透视图像
*
 * */
void img_process_object::initializeMappingTable() {
    mappingTable.resize(RESULT_ROW, std::vector<cv::Point>(RESULT_COL));

    for (int i = 0; i < RESULT_COL; i++) {
        for (int j = 0; j < RESULT_ROW; j++) {
            // 计算透视变换后的坐标
            int local_x = static_cast<int>((change_un_Mat[0][0] * i +
                                            change_un_Mat[0][1] * j +
                                            change_un_Mat[0][2]) /
                                           (change_un_Mat[2][0] * i +
                                            change_un_Mat[2][1] * j +
                                            change_un_Mat[2][2]));

            int local_y = static_cast<int>((change_un_Mat[1][0] * i +
                                            change_un_Mat[1][1] * j +
                                            change_un_Mat[1][2]) /
                                           (change_un_Mat[2][0] * i +
                                            change_un_Mat[2][1] * j +
                                            change_un_Mat[2][2]));

            // 如果在原始图像范围内，存储映射点；否则存储无效点
            if (local_x >= 0 && local_y >= 0 && local_y < USED_ROW && local_x < USED_COL) {
                mappingTable[j][i] = cv::Point(local_x, local_y);
            } else {
                mappingTable[j][i] = cv::Point(-1, -1);  // 无效点
            }
        }
    }
}

void img_process_object::setImage(const cv::Mat& inputImage) {
    // 检查输入图像的尺寸和类型是否匹配
    if (inputImage.rows != USED_ROW || inputImage.cols != USED_COL || inputImage.type() != CV_8UC1) {
        throw std::invalid_argument("Input image size or type does not match mt9v03x_image");
    }

    // 复制图像内容到 mt9v03x_image
    inputImage.copyTo(mt9v03x_image);
}

inline void img_process_object::updatePerspectiveImage() {
    PerImg_ip = cv::Mat::zeros(RESULT_ROW, RESULT_COL, CV_8UC1);

    for (int i = 0; i < RESULT_COL; i++) {
        for (int j = 0; j < RESULT_ROW; j++) {
            cv::Point mappedPoint = mappingTable[j][i];

            if (mappedPoint.x >= 0 && mappedPoint.y >= 0) {
                PerImg_ip.at<uint8_t>(j, i) = mt9v03x_image.at<uint8_t>(mappedPoint.y, mappedPoint.x);
            } else {
                PerImg_ip.at<uint8_t>(j, i) = 0;  // 无效点设置为黑色
            }
        }
    }
}

int img_process_object::calculateSobelAtPixel(const cv::Mat& img, int x, int y) {
    if (img.empty() || img.type() != CV_8UC1) {
        throw std::invalid_argument("Input image must be a non-empty grayscale image.");
    }

    // 检查坐标是否在有效范围内
    if (x <= 0 || x >= img.cols - 1 || y <= 0 || y >= img.rows - 1) {
        throw std::out_of_range("Pixel coordinates must be within valid range.");
    }

    // Sobel 核计算
    int gx = (-1 * img.at<uchar>(y - 1, x - 1) + 1 * img.at<uchar>(y - 1, x + 1) +
              -2 * img.at<uchar>(y, x - 1) + 2 * img.at<uchar>(y, x + 1) +
              -1 * img.at<uchar>(y + 1, x - 1) + 1 * img.at<uchar>(y + 1, x + 1)) / 4;

    int gy = (1 * img.at<uchar>(y - 1, x - 1) + 2 * img.at<uchar>(y - 1, x) + 1 * img.at<uchar>(y - 1, x + 1) +
              -1 * img.at<uchar>(y + 1, x - 1) - 2 * img.at<uchar>(y + 1, x) - 1 * img.at<uchar>(y + 1, x + 1)) / 4;

    // 梯度幅值计算
    float sobel = (std::abs(gx) + std::abs(gy)) / 2.0f;
    return sobel;
}

uint8_t img_process_object::get_start_point(const cv::Mat& bin_image, int start_row) {

    start_point_l[0] = 0;//x
    start_point_l[1] = 0;//y

    start_point_r[0] = 0;//x
    start_point_r[1] = 0;//y
    bool l_found = false, r_found = false;

    // 边界检查
    if (start_row < 0 || start_row >= use_h) {
        return 0; // 起始行超出图像范围
    }

    // 从中间向左寻找
    for (int i = use_w / 2; i > border_min; --i) {
        if (bin_image.at<uint8_t>(start_row, i) == 255 && bin_image.at<uint8_t>(start_row, i - 1) == 0) {
            start_point_l[0] = i;//x
            start_point_l[1] = start_row;
            l_found = true;
            break;
        }
    }

    // 从中间向右寻找
    for (int i = use_w / 2; i < border_max; ++i) {
        if (bin_image.at<uint8_t>(start_row, i) == 255 && bin_image.at<uint8_t>(start_row, i + 1) == 0) {
            start_point_r[0] = i;//x
            start_point_r[1] = start_row;
            r_found = true;
            break;
        }
    }

    // 输出结果
    if (l_found && r_found) {

        // points_test[1][1]=start_row;
        // printf("Left Start Point: (%d, %d)\n", start_point_l[0], start_row);
        // printf("Right Start Point: (%d, %d)\n", start_point_r[0], start_row);
        return 1;
    } else {
        printf("Start points not found!\n");
        return 0;
    }
}

inline uint8 img_process_object::get_start_point_sobel(const cv::Mat& img) {
    uint8 l_found = 0, r_found = 0;

    // 初始化起始点
    start_point_l[0] = 0; // x
    start_point_l[1] = 0; // y
    start_point_r[0] = 0; // x
    start_point_r[1] = 0; // y


    uint8 start_row = start_h;
    // 从指定行开始向上查找
    // printf("start_cow_l=%d",start_cow_l);
    // for (int row = start_row; row > 0; row--) {
        // 查找左边起始点
        for (int col = start_cow_l; col > border_min - 1; col--) {
            if (calculateSobelAtPixel(img, col, start_row) > sobel_th) {
                start_point_l[0] = col; // x
                start_point_l[1] = start_row; // y
                l_found = 1;
                // printf("找到左边起始点: (%d, %d)\n", row, col);
                break;
            }
        }

        // 查找右边起始点
        for (int col = start_cow_r; col < border_max - 1; col++) {
            if (calculateSobelAtPixel(img, col, start_row) > sobel_th) {
                start_point_r[0] = col; // x
                start_point_r[1] = start_row; // y
                r_found = 1;
                // printf("找到右边起始点: (%d, %d)\n", row, col);
                break;
            }
        }

        // 如果左右点都找到，则返回成功
        if (l_found && r_found) {
            if (start_point_l[0] + 10 >72 || start_cow_r <72) {
                start_cow_l = use_w/2;
                start_cow_r = use_w/2;
            }
            else
            {
                start_cow_l = start_point_l[0] + 40 ;
                start_cow_r = start_point_r[0] - 40 ;
            }

            // points_test[0][0]=start_point_l[0];
            // points_test[0][1]=start_point_l[1];
            // points_test[1][0]=start_point_r[0];
            // points_test[1][1]=start_point_r[1];
            // points_test[2][0]=start_cow_l;
            // points_test[2][1]=110;
            // points_test[3][0]=start_cow_r;
            // points_test[3][1]=110;
            // printf("找到右边起始点: (%d, %d)\n", start_cow_l, 110);
            return 1;
        } else {
            l_found = 0;
            r_found = 0;
            return 0;
        }
    // }

    // 如果未找到起始点，返回失败
    return 0;
}
/*
函数名称：void image_draw_rectan(cv::Mat& image)
功能说明：给图像画一个黑框
参数说明：cv::Mat& image 图像对象（灰度图或单通道图像）
函数返回：无
修改时间：2025年1月10日
备    注：
example： image_draw_rectan(image);
*/
inline void img_process_object::image_draw_rectan(cv::Mat& image) {
    // 检查图像是否为空
    if (image.empty()) {
        throw std::invalid_argument("Input image is empty.");
    }

    // 检查图像是否为单通道
    if (image.channels() != 1) {
        throw std::invalid_argument("Input image must be a single-channel grayscale image.");
    }

    int get_image_h = image.rows; // 图像高度
    int get_image_w = image.cols; // 图像宽度

    // 绘制左右两列的黑框
    for (int i = 0; i < get_image_h; i++) {
        image.at<uchar>(i, 0) = 0;       // 第一列
        image.at<uchar>(i, 1) = 0;       // 第二列
        image.at<uchar>(i, get_image_w - 1) = 0; // 最后一列
        image.at<uchar>(i, get_image_w - 2) = 0; // 倒数第二列
    }

    // 绘制上下两行的黑框
    for (int i = 0; i < get_image_w; i++) {
        image.at<uchar>(0, i) = 0;       // 第一行
        image.at<uchar>(1, i) = 0;       // 第二行
        // image.at<uchar>(image_h - 1, i) = 0; // 最后一行（如果需要，可以解注释）
    }
}

inline void img_process_object::print_average_huidu(const cv::Mat& img, make_sizetype make_size, Threshold_min_type Threshold_min) {
    // 检查图像是否有效
    if (img.empty() || img.channels() != 1) {
        throw std::invalid_argument("Input image must be a non-empty grayscale image.");
    }

    // 遍历图像的每个像素点
    for (int y = make_size / 2; y < img.rows - make_size / 2; y++) {
        for (int x = make_size / 2; x < img.cols - make_size / 2; x++) {
            // 调用 otsuThreshold_average 函数计算平均灰度值
            int average_huidu = otsuThreshold_average(make_size, img, x, y, Threshold_min);
            printf("Pixel (%d, %d): Average Gray Value = %d\n", x, y, average_huidu);
        }
    }
}

inline float img_process_object::Slope_Calculate(uint8 begin, uint8 end, uint8 *border) {
    float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
	int16 i = 0;
	float result = 0;
	static float resultlast;

	for (i = begin; i < end; i++)
	{
		xsum += i;
		ysum += border[i];
		xysum += i * (border[i]);
		x2sum += i * i;

	}
	if ((end - begin)*x2sum - xsum * xsum) //判断除数是否为零
	{
		result = ((end - begin)*xysum - xsum * ysum) / ((end - begin)*x2sum - xsum * xsum);
		resultlast = result;
	}
	else
	{
		result = resultlast;
	}
	return result;
}

inline void img_process_object::calculate_s_i(uint8 start, uint8 end, uint8 *border, float *slope_rate,
    float *intercept) {
    if (start >= end) { // 检查范围有效性
        *slope_rate = 0;
        *intercept = 0;
        return;
    }

    uint16 xsum = 0, ysum = 0;
    uint16 num = end - start;

    // 计算 xsum 和 ysum
    for (uint16 i = start; i < end; i++) {
        xsum += i;
        ysum += border[i];
    }

    // 计算平均值
    float x_average = (float)xsum / num;
    float y_average = (float)ysum / num;

    // 计算斜率和截距
    *slope_rate = Slope_Calculate(start, end, border); // 确保 Slope_Calculate 的实现有效
    *intercept = y_average - (*slope_rate) * x_average;

}

inline int img_process_object::otsuThreshold_average(make_sizetype make_size, const cv::Mat& img, uint16_t x, uint16_t y, Threshold_min_type Threshold_min) {
    // 检查图像是否有效
    if (img.empty() || img.channels() != 1) {
        throw std::invalid_argument("Input image must be a non-empty grayscale image.");
    }

    // 计算邻域内的平均灰度值
    make_sizetype half = make_size / 2;
    otsuTh_eight_type average_huidu = 0;
    int valid_pixel_count = 0; // 有效像素点计数

    for (int dy = -half; dy <= half; dy++) {
        for (int dx = -half; dx <= half; dx++) {
            int nx = x + dx; // 邻域内像素点的 x 坐标
            int ny = y + dy; // 邻域内像素点的 y 坐标

            // std::cout << static_cast<int>(img.at<uchar>(111, 3)) << " ";
            // 检查是否在图像范围内
            if (nx >= 0 && nx < img.cols && ny >= 0 && ny < img.rows) {
                // printf("Valid pixel: nx = %d, ny = %d\n", nx, ny);
                average_huidu += static_cast<int>(img.at<uchar>(111, 3)); // 使用 uchar 访问像素

            }
            else {
                printf("Out of bounds: nx = %d, ny = %d\n", nx, ny);
                average_huidu += 255; // 超出边界的像素值设为 255

            }
        }
    }
    average_huidu = average_huidu / (make_size*make_size);
    average_huidu -= Threshold_min;
    printf("average_huidu = %d\n", average_huidu);
    //

    return average_huidu;
}

inline void img_process_object::search_l_r(
    uint16_t break_flag,
    const cv::Mat& image,
    uint8_t l_start_x,
    uint8_t l_start_y,
    uint8_t r_start_x,
    uint8_t r_start_y,
    uint8_t* hightest
)
{
    uint8 i = 0, j = 0;

    //左边变量
    uint8 search_filds_l[8][2] = { {  0 } };
    uint8 index_l = 0;
    uint8 temp_l[8][2] = { {  0 } };
    uint8 center_point_l[2] = { 0 };
    uint16 l_data_statics;//统计左边
    //定义八个邻域
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是顺时针

    //右边变量
    uint8 search_filds_r[8][2] = { {  0 } };
    uint8 center_point_r[2] = { 0 };//中心坐标点
    uint8 index_r = 0;//索引下标
    uint8 temp_r[8][2] = { {  0 } };
    uint16 r_data_statics;//统计右边
    //定义八个邻域
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是逆时针

    l_data_statics = data_stastics_l;//统计找到了多少个点，方便后续把点全部画出来
    r_data_statics = data_stastics_r;//统计找到了多少个点，方便后续把点全部画出来

    //第一次更新坐标点  将找到的起点值传进来
    center_point_l[0] = l_start_x;//x

    center_point_l[1] = l_start_y;//y
    center_point_r[0] = r_start_x;//x
    center_point_r[1] = r_start_y;//y
    // printf("center_point_l[0] = %d\n", center_point_l[0]);
    // printf("center_point_r[0] = %d\n", center_point_r[0]);
    //开启邻域循环
    while (break_flag--)
    {

        //左边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
        }
        //中心坐标点填充到已经找到的点内
        points_l[l_data_statics][0] = center_point_l[0];//x
        points_l[l_data_statics][1] = center_point_l[1];//y
        // printf("points_l_x = %d,%d\n",points_l[l_data_statics][0],points_l[l_data_statics][1]);
        // printf("points_l_x=%d,y=%d\n",points_l[l_data_statics][0],points_l[l_data_statics][1]);
        l_data_statics++;//索引加一

        //右边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
        }
        //中心坐标点填充到已经找到的点内
        points_r[r_data_statics][0] = center_point_r[0];//x
        points_r[r_data_statics][1] = center_point_r[1];//y
        // printf("points_r_x=%d,y=%d\n",points_r[r_data_statics][0],points_r[r_data_statics][1]);
        index_l = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;//先清零，后使用
            temp_l[i][1] = 0;//先清零，后使用
        }

        //左边判断
        for (i = 0; i < 8; i++)
        {

            // printf("search_filds_l[%d][0] = %d\n", i, search_filds_l[i][0]);
            if (image.at<uint8_t>(search_filds_l[i][1], search_filds_l[i][0])== 0
                && image.at<uint8_t>(search_filds_l[(i + 1) & 7][1],search_filds_l[(i + 1) & 7][0]) == 255)
            {
                temp_l[index_l][0] = search_filds_l[(i)][0];
                temp_l[index_l][1] = search_filds_l[(i)][1];
                // points_test[0][0]=search_filds_l[(i)][0];
                // points_test[0][1]=search_filds_l[(i)][1];
                // printf("x=%d, y=%d\n", temp_l[index_l][0], temp_l[index_l][1]);
                index_l++;
                dir_l[l_data_statics - 1] = (i);//记录生长方向
            }

            if (index_l)
            {
                //更新坐标点
                center_point_l[0] = temp_l[0][0];//x
                center_point_l[1] = temp_l[0][1];//y
                // printf("center_point_l[0] = %d, temp_l[0][1] = %d\n", center_point_l[0], temp_l[0][1]);
                for (j = 0; j < index_l; j++)
                {
                    if (center_point_l[1] > temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0];//x
                        center_point_l[1] = temp_l[j][1];//y
                    }
                }
            }

        }
        // printf("temp_l[0] = %d, temp_l[1] = %d\n", temp_l[0][0], temp_l[0][1]);
        // printf("points_l[l_data_statics - 1]=%d %d,%d,%d,%d,%d\n",points_l[l_data_statics - 1][0],points_l[l_data_statics - 1][1],points_l[l_data_statics - 2][0],points_l[l_data_statics - 2][1],points_l[l_data_statics - 3][0],points_l[l_data_statics - 3][1]);
        if ((points_r[r_data_statics][0] == points_r[r_data_statics - 1][0] && points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
            && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
            || (points_l[l_data_statics - 1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics - 1][0] == points_l[l_data_statics - 3][0]
                && points_l[l_data_statics - 1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics - 1][1] == points_l[l_data_statics - 3][1]))
        {
            printf("三次进入同一个点，退出\n");
            break;
        }
        if (abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
            && abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
            )
        {
            //printf("\n左右相遇退出\n");
            *hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
            //printf("\n在y=%d处退出\n",*hightest);
            break;
        }
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
        {
            // printf("\n如果左边比右边高了，左边等待右边\n");
            continue;//如果左边比右边高了，左边等待右边
        }
        if (dir_l[l_data_statics - 1] == 7
            && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
        {
            //printf("\n左边开始向下了，等待右边，等待中... \n");
            center_point_l[0] = (uint8)points_l[l_data_statics - 1][0];//x
            center_point_l[1] = (uint8)points_l[l_data_statics - 1][1];//y
            l_data_statics--;
        }
        r_data_statics++;//索引加一

        index_r = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;//先清零，后使用
            temp_r[i][1] = 0;//先清零，后使用
        }

        //右边判断
        for (i = 0; i < 8; i++)
        {
            if (image.at<uint8_t>(search_filds_r[i][1],search_filds_r[i][0]) == 0
                && image.at<uint8_t>(search_filds_r[(i + 1) & 7][1],search_filds_r[(i + 1) & 7][0]) == 255)
            {
                temp_r[index_r][0] = search_filds_r[(i)][0];
                temp_r[index_r][1] = search_filds_r[(i)][1];
                // points_test[1][0]=search_filds_r[(i)][0];
                // points_test[1][1]=search_filds_r[(i)][1];
                index_r++;//索引加一
                dir_r[r_data_statics - 1] = (i);//记录生长方向

                //printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
            }
            if (index_r)
            {

                //更新坐标点
                center_point_r[0] = temp_r[0][0];//x
                center_point_r[1] = temp_r[0][1];//y

                for (j = 0; j < index_r; j++)
                {
                    if (center_point_r[1] > temp_r[j][1])
                    {
                        center_point_r[0] = temp_r[j][0];//x
                        center_point_r[1] = temp_r[j][1];//y
                    }
                    // printf("temp_r[0] = %d, temp_r[1] = %d\n", temp_r[0][0], temp_r[0][1]);
                }

            }
        }


    }


    //取出循环次数

    data_stastics_l = l_data_statics;
    data_stastics_r = r_data_statics;

}

inline void img_process_object::get_left(uint16 total_L)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    // printf("total_L=%d\n", total_L);
    //初始化
    for (i = 0; i < use_h; i++)
    {
        l_border[i] = border_min;
    }
    h = start_h;
    // printf("h:%d\n", h);
    //左边
    for (j = 0; j < total_L; j++)
    {

        // printf("j=%d,points_l[j][1] = %d", j, points_l[j][1]);
        // printf("h:%d\n", h);
        if (points_l[j][1] == h)
        {
            // printf("进入了\n");
            l_border[h] = points_l[j][0] + 1;
            line_detect_l--;
        }
        else continue; //每行只取一个点，没到下一行就不记录
        // printf("进入了\n");
        h=h-1;
        if (h == 0)
        {
            break;//到最后一行退出
        }
    }
}

inline void img_process_object::get_right(uint16 total_R)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    for (i = 0; i < use_h; i++)
    {
        r_border[i] = border_max;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
    }
    h = start_h;
    //右边
    for (j = 0; j < total_R; j++)
    {
        if (points_r[j][1] == h)
        {
            r_border[h] = points_r[j][0] - 1;
            line_detect_r--;
            // printf("line_detect_r = %d\n", line_detect_r);
        }
        else continue;//每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)break;//到最后一行退出
    }
}

inline uint8_t img_process_object::Threshold_deal(const cv::Mat& image, uint32_t pixel_threshold) {
    // 检查图像是否为有效的单通道灰度图
    if (image.empty() || image.type() != CV_8UC1) {
        throw std::invalid_argument("Input image must be a non-empty grayscale image.");
    }

    const int GrayScale = 256; // 灰度级范围
    int pixelCount[GrayScale] = {0}; // 灰度值统计
    float pixelPro[GrayScale] = {0.0f}; // 灰度值比例
    int width = image.cols; // 图像宽度
    int height = image.rows; // 图像高度
    int pixelSum = width * height; // 总像素数
    uint8_t threshold = 0; // 最终阈值

    // 统计灰度值分布
    uint32_t gray_sum = 0; // 灰度值总和
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            uint8_t pixelValue = image.at<uint8_t>(i, j);
            pixelCount[pixelValue]++;
            gray_sum += pixelValue;
        }
    }

    // 计算每个灰度值的比例
    for (int i = 0; i < GrayScale; i++) {
        pixelPro[i] = static_cast<float>(pixelCount[i]) / pixelSum;
    }

    // 遍历灰度级，寻找最佳阈值
    float w0 = 0.0f, w1 = 0.0f;
    float u0tmp = 0.0f, u1tmp = 0.0f;
    float u0 = 0.0f, u1 = 0.0f, u = 0.0f;
    float deltaTmp = 0.0f, deltaMax = 0.0f;

    for (int j = 0; j < pixel_threshold; j++) {
        w0 += pixelPro[j]; // 背景部分比例
        u0tmp += j * pixelPro[j]; // 背景灰度值总和

        w1 = 1.0f - w0; // 前景部分比例
        u1tmp = static_cast<float>(gray_sum) / pixelSum - u0tmp; // 前景灰度值总和

        if (w0 > 0.0f) u0 = u0tmp / w0; // 背景平均灰度
        if (w1 > 0.0f) u1 = u1tmp / w1; // 前景平均灰度

        u = u0tmp + u1tmp; // 全局平均灰度
        deltaTmp = w0 * std::pow((u0 - u), 2) + w1 * std::pow((u1 - u), 2);

        if (deltaTmp > deltaMax) {
            deltaMax = deltaTmp;
            threshold = j;
        }
    }

    return threshold;
}

inline void img_process_object::Get01change(const cv::Mat& inputImage, cv::Mat& outputImage, int thresholdStatic) {
    // 检查输入图像是否为单通道灰度图
    if (inputImage.empty() || inputImage.type() != CV_8UC1) {
        throw std::invalid_argument("Input image must be a non-empty grayscale image.");
    }

    // 初始化输出图像
    outputImage = cv::Mat::zeros(inputImage.size(), CV_8UC1);

    int rows = inputImage.rows;
    int cols = inputImage.cols;

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            int threshold = thresholdStatic;

            // 根据列位置调整阈值
            if (j <= 15) {
                threshold -= 10;
            } else if (j > 70 && j <= 75) {
                threshold -= 15;
            } else if (j >= 65) {
                threshold -= 15;
            }

            // 获取当前像素值
            uint8_t pixelValue = inputImage.at<uint8_t>(i, j);
            printf("pixelValue = %d,threshold=%d\n",pixelValue,threshold);
            // 应用阈值分割
            if (pixelValue > threshold) {
                outputImage.at<uint8_t>(i, j) = 255;  // 白色
            } else {
                outputImage.at<uint8_t>(i, j) = 0;    // 黑色
            }
        }
    }
}

inline void img_process_object::Get01change_dajin(const cv::Mat& inputImage, cv::Mat& outputImage, int threshold_static, int threshold_detach) {
    // 检查输入图像是否有效
    if (inputImage.empty() || inputImage.channels() != 1) {
        throw std::invalid_argument("Input image must be a non-empty grayscale image.");
    }

    // 初始化输出图像，与输入图像大小相同，类型为单通道二值图像
    outputImage = cv::Mat::zeros(inputImage.size(), CV_8UC1);

    // 计算动态阈值
    int dynamicThreshold = Threshold_deal(inputImage, threshold_detach);
    if (dynamicThreshold < threshold_static) {
        dynamicThreshold = threshold_static;
    }

    // 遍历每个像素
    for (int i = 0; i < inputImage.rows; i++) {
        for (int j = 0; j < inputImage.cols; j++) {
            int thre;

            // 根据列位置动态调整阈值
            if (j <= 15) {
                thre = dynamicThreshold - 10;
            } else if (j > 70 && j <= 75) {
                thre = dynamicThreshold - 10;
            } else if (j >= 65) {
                thre = dynamicThreshold - 10;
            } else {
                thre = dynamicThreshold;
            }

            // 像素二值化
            if (inputImage.at<uchar>(i, j) >= thre) {
                outputImage.at<uchar>(i, j) = 255; // 白
            } else {
                outputImage.at<uchar>(i, j) = 0;   // 黑
            }
        }
    }
}

inline void img_process_object::find_line()
{
    using namespace std::chrono; // 简化 chrono 的命名空间
    cv::resize(mt9v03x_image, mt9v03x_image, cv::Size(use_w, use_h), 0, 0, cv::INTER_LINEAR);
    data_stastics_l = 0;
    data_stastics_r = 0;
    line_detect_l = use_h - 1;
    line_detect_r = use_h - 1;
    line_detect_center = use_h - 1;

    // 记录起始时间
    auto start_time = high_resolution_clock::now();

    // 测量 image_draw_rectan
    // auto t1 = high_resolution_clock::now();
    image_draw_rectan(mt9v03x_image);
    cv::Mat tempImage = mt9v03x_image.clone();
    Get01change_dajin(tempImage, mt9v03x_image, 135, 220);
    cv::Mat temp = mt9v03x_image.clone();
    if (get_start_point(temp, (start_h))) {
        search_l_r((uint16)USE_num, temp, start_point_l[0], start_point_l[1],
                   start_point_r[0], start_point_r[1], &hightest);
        get_right(data_stastics_r);
        get_left(data_stastics_l);
        line_detect_center = max(line_detect_l, line_detect_r); // 获取中心行
        // line_detect_center = max(line_detect_center, offline);
        // printf("line_detect_center = %d\n",line_detect_center);
        Element_Detection(temp);
        get_center();
        RouteFilter();
        GetDet(40,40);
    }

    // 总运行时间
    auto end_time = high_resolution_clock::now();
    // std::cout << "find_line 总运行时间: "
    //           << duration_cast<milliseconds>(end_time - start_time).count() << " ms" << std::endl;
}

inline void img_process_object::get_center()
{
    for (int i = use_h - 1; i > offline; i--) {
        center_line[i] = (l_border[i] + r_border[i]) >> 1; // Calculate center line
        center_err[i] = center_line[i] -target_center[i];
        // printf("center_line[%d] = %d\n",i,center_err[i]);

    }
    // printf("center_line=%d\n",center_line[use_h-2]);
    // for (int i =0; i < use_h ; i++) {
    //     printf("center_line[%d] = %d\n",i,center_line[i]);
    // }
}

inline void img_process_object::RouteFilter()
{
    float DetR; // 斜率
    // static bool test = false;

    // printf("line_detect_center = %d,%d\n",line_detect_center,offline);
    // 检查中心行是否有效
    // if (test) {printf("按任意键继续...\n");
    // std::cin.get(); // 等待用户输入
    // test = false;
    // }
    if (line_detect_center <= offline) {
        // printf("无效的 line_detect_center: %d <= offline: %d\n", line_detect_center, offline);
        return; // 如果中心行无效，直接返回
    }
    for (int i = line_detect_center; i > offline; i--) // 从中心行向上遍历
    {

        // 计算斜率
        DetR = ((float)center_line[i+1] - center_line[i+2]) / (float)(i+1 - (i+2));

        // 根据斜率补线，计算中心点的 x 坐标
        center_line[i] = (int)(center_line[i+1] + DetR * (float)(i - (i+1)));
        // 打印调试信息
        // printf("行号: i=%d, 斜率 DetR=%.2f, 补线后 center_line[%d]=%d\n",               i, DetR, i, center_line[i]);
        // test = true;
        }

}


inline void img_process_object::draw_line_1(cv::Mat& image, const uint8_t border[], cv::Scalar color, int num) {
    for(int i=0;i<num;i++){

        cv::circle(image,cv::Point(border[i], i),1,color,-1);
        // printf("center_line[%d] = %d\n",i,border[i]);
    }
}

inline void img_process_object::Element_Detection(const cv::Mat &inputImage) {
    Staight_Detection(inputImage);
    cross_fill(inputImage,l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r);
}

inline void img_process_object::Staight_Detection(const cv::Mat &inputImage) {
    float variance, variance_acc;  // 方差
    float sum = 0;
    for (int Ysite = 50;Ysite > offline; Ysite--) {

    sum += (center_line[Ysite]-target_center[Ysite]) *(center_line[Ysite]-target_center[Ysite]);
  }
    // printf("sum = %f\n",sum);
    variance_acc = (float)sum / (49 - line_detect_center);
    // printf("variance_acc = %f\n", variance_acc);
    if (variance_acc < straight_variance_acc_th&&line_detect_r < 10 && line_detect_l < 10) {
        Straight_Flag = 1;
        printf("Straight_Flag = 1\n");
    }else
        Straight_Flag = 0;
}

inline void img_process_object::cross_fill(const cv::Mat &inputImage, uint8 *l_border, uint8 *r_border, uint16 total_num_l,
    uint16 total_num_r, uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2]) {
    uint8 total_lr = 0;
    uint8 i;
	uint8 break_num_l_up[2] = {0,0};
    uint8 break_num_l_down[2] = {0,0};
	uint8 break_num_r_up[2]= {0,0};
    uint8 break_num_r_down[2]= {0,0};
    static uint8 last_break_num_l_up[2]= {0,0};
    static uint8 last_break_num_l_down[2]= {0,0};
    static uint8 last_break_num_r_up[2]= {0,0};
    static uint8 last_break_num_r_down[2]= {0,0};
    bool off_left=false;
    bool off_right=false;
	uint8 start_l, end_l;
    uint8 start_r, end_r;
	float slope_l_rate = 0, intercept_l = 0;
    float slope_r_rate = 0, intercept_r = 0;
    bool l_complete = false;
    bool r_complete = false;
	//出十字

    // printf("l_border:%d,r_border:%d\n",l_border[use_h-5],r_border[use_h-5]);
    if (l_border[use_h-5] !=border_min+1 && r_border[use_h-5] !=border_max-2) {return;}

    // printf("l_border[use_h-1]=%d r_border[use_h-1]=%d\n",l_border[use_h-2],r_border[use_h-2]);
    for (i = use_h-2; i >= use_h/2;i--) {
        // printf("l_border[use_h-1]=%d r_border[use_h-1]=%d\n",l_border[i],r_border[i]);
        if (l_border[i]==border_min+1 && r_border[i]==border_max-2) {
            total_lr++;
        }

    }
    printf("total_lr = %d,use_h/3=%d\n", total_lr,use_h/3);
    if (total_lr >=use_h/3) {
        Cross_Flag =1;
    }
    else Cross_Flag =0;


    if (Cross_Flag == 1) {
        //找上拐点
        for (i = (use_h-1)/2; i > 10;i--) {
            // printf("l_border[use_h-1]=%d \n",l_border[i]);
            if (l_border[i]>=border_min+7 && break_num_l_up[0]==0&&off_left==false) {
                if (l_border[i]<last_break_num_l_up[0]) {
                    off_left=true;
                }
                else {
                    break_num_l_up[1] =i;
                    break_num_l_up[0] =l_border[i];
                    printf("找到左上拐点");
                }
            }
            else if (off_left==false) {
                last_break_num_l_up[1] =i;
                last_break_num_l_up[0] =l_border[i];
            }

            printf("r_border[%d]=%d\n",i,r_border[i]);
            printf("last_break_num_r_up[0]=%d\n",last_break_num_r_up[0]);
            if (r_border[i]>=use_w/2&&r_border[i]<=border_max-7 && break_num_r_up[0]==0&&off_right==false) {
                printf("ok\n");
                if (r_border[i]>last_break_num_r_up[0]) {
                    off_right=true;
                    printf("找到右上拐点%d\n",break_num_r_up[1]);
                }
                else {
                    break_num_r_up[1] =i;
                    break_num_r_up[0] =r_border[i];


                }
            }
            else if (off_right==false) {
                last_break_num_r_up[1] =i;
                last_break_num_r_up[0] =r_border[i];
            }

            if (break_num_r_up[0]!=0&&break_num_l_up[0]!=0) {
                break;
            }
            if (off_left==true && off_right==true) {

                return;//没找到上拐点return
            }
        }

        for (i = (use_h-1)/2; i <use_h-1;i++) {
            // printf("l_border[use_h-1]=%d \n",l_border[i]);
            if (l_border[i]>=border_min+7 && break_num_l_down[0]==0&&off_left==false) {
                if (l_border[i]<last_break_num_l_down[0]) {
                    off_left=true;
                }else {
                    break_num_l_down[1] =i;
                    break_num_l_down[0] =l_border[i];
                    printf("找到左下拐点");
                }

            }
            else if (off_left==false) {
                last_break_num_l_down[1] =i;
                last_break_num_l_down[0] =l_border[i];
            }

            if (r_border[i]<=border_max-7 && break_num_r_down[0]==0&&off_right==false) {
                if (r_border[i]>last_break_num_r_down[0]) {
                    off_right=true;
                }else {
                    break_num_r_down[1] =i;
                    break_num_r_down[0] =r_border[i];
                    printf("找到右下拐点");
                }


            }
            else if (off_right==false) {
                last_break_num_r_down[1] =i;
                last_break_num_r_down[0] =r_border[i];
            }

            if (break_num_r_down[0]!=0&&break_num_l_down[0]!=0) {
                break;
            }
            if (off_left==true && off_right==true) {
                break;
            }
        }
    }

    printf("break_num_l_up[1]:%d,break_num_r_up[1]=%d\n",break_num_l_up[0],break_num_r_up[0]);

    if (break_num_l_up[0])
    {
        start_l = break_num_l_up[1]-3;
        start_l = limit_a_b(start_l, 0, use_h);
        end_l = break_num_l_up[1];

        calculate_s_i(start_l, end_l, l_border, &slope_l_rate, &intercept_l);
        printf("slope_l_rate:%f\nintercept_l:%f\n", slope_l_rate, intercept_l);
        for (i = start_l; i < use_h - 1; i++)
        {
            if (slope_l_rate * (i)+intercept_l <0) {
                l_border[i] = border_min;
            }else if (slope_l_rate * (i)+intercept_l >border_max) {
                l_border[i] = border_max;
            }else l_border[i] = slope_l_rate * (i)+intercept_l;//y = kx+b
            l_border[i] = limit_a_b(l_border[i], border_min, border_max);//限幅
            l_complete =true;
        }

    }
    if (break_num_l_down[0] &&l_complete == false) {
        //计算斜率
        start_l = break_num_l_down[1]+1;
        start_l = limit_a_b(start_l, 0, use_h);
        end_l = break_num_l_down[1]+5;


        calculate_s_i(start_l, end_l, l_border, &slope_l_rate, &intercept_l);
        printf("slope_l_rate:%f\nintercept_l:%f\n", slope_l_rate, intercept_l);
        for (i = start_l; i < use_h - 1; i++)
        {
            if (slope_l_rate * (i)+intercept_l <0) {
                l_border[i] = border_min;
            }
            else if (slope_l_rate * (i)+intercept_l >border_max) {
                l_border[i] = border_max;
            }
            else l_border[i] = slope_l_rate * (i)+intercept_l;//y = kx+b
            l_border[i] = limit_a_b(l_border[i], border_min, border_max);//限幅
            // printf("l_border[use_h-2]=%d \n",l_border[use_h-2]);
        }
    }
    if (break_num_r_up[0]) {
        start_r = break_num_r_up[1]-3;
        start_r = limit_a_b(start_r, 0, use_h);
        end_r = break_num_r_up[1];
        printf("start_r:%d,end_r:%d\n", start_r, end_r);
        calculate_s_i(start_r, end_r, r_border, &slope_r_rate, &intercept_r);
        printf("slope_r_rate:%f\nintercept_r:%f\n", slope_r_rate, intercept_r);
        for (i = start_r; i < use_h - 1; i++)
        {
            if (slope_r_rate * (i)+intercept_r <0) {
                r_border[i] = border_min;
            }
            else if (slope_r_rate * (i)+intercept_r >border_max) {
                r_border[i] = border_max;
            }
            else r_border[i] = slope_r_rate * (i)+intercept_r;
            r_border[i] = limit_a_b(r_border[i], border_min, border_max);
        }
        r_complete = true;
        // printf("r_border[use_h-2]=%d \n",r_border[use_h-2]);

    }
    if (break_num_r_down[0] &&r_complete == false) {
        start_r = break_num_r_down[1]+1;
        start_r = limit_a_b(start_r, 0, use_h);

        end_r = break_num_r_down[1]+5;
        printf("start_r:%d,end_r:%d\n", start_r, end_r);
        calculate_s_i(start_r, end_r, r_border, &slope_r_rate, &intercept_r);
        printf("slope_r_rate:%f\nintercept_r:%f\n", slope_r_rate, intercept_r);
        for (i = start_r; i < use_h - 1; i++)
        {
            if (slope_r_rate * (i)+intercept_r <0) {
                r_border[i] = border_min;
            }
            else if (slope_r_rate * (i)+intercept_r >border_max) {
                r_border[i] = border_max;
            }
            else r_border[i] = slope_r_rate * (i)+intercept_r;
            r_border[i] = limit_a_b(r_border[i], border_min, border_max);
        }
        // printf("r_border[use_h-2]=%d \n",r_border[use_h-2]);
    }

}

inline void img_process_object::GetDet(const uint8 speed_now,const uint8 speed_min)
{
    float DetTemp = 0;//计算平均误差
    float SpeedGain = 0;//速度因子，根据速度因子计算前瓒
    float UnitAll = 0; //累积的权重，用于归一化计算。
    int TowPoint = set_towpoint; //前瞻点
    int Ysite = 0;    //当前点

    const int MAX_TOWPOINT = 49;//最大前瞻
    const int MIN_TOWPOINT = 1;//最小前瞻
    const int CIRQUE_TOWPOINT = 30;//圆环前瞻
    const int CROSS_TURN_TOWPOINT = 22;//十字前瞻
    const int STRAIGHET_TOWPOINT = 10;//直线前瞻

    SpeedGain = ( speed_now- speed_min) * 0.2 + 0.5; //小于最低速度时,前瞻变变短 大于变远
    SpeedGain = std::clamp(SpeedGain, -1.0f, 5.0f);

    if ((RightCirque_Flag > RightCirque_TowPoint_Flag || LeftCirque_Flag > LeftCirque_TowPoint_Flag)&&CirqueOff == false)
        TowPoint = CIRQUE_TOWPOINT;
    else if (Straight_Flag ==1)
        TowPoint = STRAIGHET_TOWPOINT;
    else if (Cross_Flag ==1)
        TowPoint = CROSS_TURN_TOWPOINT;
    else
        TowPoint = TowPoint-SpeedGain;                                          //速度越快前瞻越长
    TowPoint = min(start_h,TowPoint);//限制前瞻位置



    if ((TowPoint - 5) >= line_detect_center) {
        for (int Ysite = (TowPoint - 5); Ysite < TowPoint; Ysite++) {
      DetTemp = DetTemp + Weighting[TowPoint - Ysite - 1] * (center_err[Ysite]);
      UnitAll = UnitAll + Weighting[TowPoint - Ysite - 1];
            // printf("DetTemp1 = %f\n",DetTemp);
            // printf("UnitAll1 = %f\n",UnitAll);

    }
    for (Ysite = (TowPoint + 5); Ysite > TowPoint; Ysite--) {
      DetTemp += Weighting[-TowPoint + Ysite - 1] * (center_err[Ysite]);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
        // printf("DetTemp2 = %f\n",DetTemp);
        // printf("UnitAll2 = %f\n",UnitAll);
    }
    DetTemp = (center_err[TowPoint]) / (UnitAll + 1);
        // printf("DetTemp = %f\n",DetTemp);
    }
    else if (TowPoint > line_detect_center) {
    for (Ysite = line_detect_center; Ysite < TowPoint; Ysite++) {
      DetTemp += Weighting[TowPoint - Ysite - 1] * (center_err[Ysite]);
      UnitAll += Weighting[TowPoint - Ysite - 1];
    }
    for (Ysite = (TowPoint + TowPoint - line_detect_center); Ysite > TowPoint;
         Ysite--) {
      DetTemp += Weighting[-TowPoint + Ysite - 1] * (center_err[Ysite]);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (center_err[Ysite] + DetTemp) / (UnitAll + 1);
  } else if (line_detect_center < 49) {
    for (Ysite = (line_detect_center + 3); Ysite > line_detect_center;
         Ysite--) {
      DetTemp += Weighting[-TowPoint + Ysite - 1] * (center_err[Ysite]);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (center_err[line_detect_center] + DetTemp) / (UnitAll + 1);

  } else
    DetTemp =Det_True;                                                     //如果是出现line_detect_center>50情况，保持上一次的偏差值

  Det_True = DetTemp;                                                      //此时的解算出来的平均图像偏差
    // printf("DET = %d \n",Det_True);
    TowPoint_True = TowPoint;                                                //此时的前瞻
    // points_test[0][1] =  40;
    // points_test[0][0] =  44+Det_True*10;
    // points_test[1][1] =  30;
    // points_test[1][0] =  44;
    // printf("TowPoint_True = %d \n",TowPoint_True);
    // printf("Det_True = %f \n",Det_True);



}



#endif //IMG_PROCESS_H
