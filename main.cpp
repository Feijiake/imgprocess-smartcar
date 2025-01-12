#include <iostream>
#include "img_processing/img_process_app.h"
using namespace std;
using namespace cv;
Mat img_origin;
// An highlighted block




int main() {
    std::cout << "start" << std::endl;
    VideoCapture CvCapture("/home/qing/git_project/imgprocess-smartcar/resource/all.avi");
    if (!CvCapture.isOpened()) {
        cout << "Read video Failed!" << std::endl;
        return -1;
    }
    CvCapture.set(cv::CAP_PROP_POS_FRAMES, 999); // 999因为帧索引是从0开始的
        img_process_object processor;

    processor.initializeMappingTable();
    // std::string image_path = "/home/qing/Downloads/code/resource/1.jpg"; // 替换为你的图片路径
    while (true){

        CvCapture.read(img_origin);
        // Mat img_lhc = imread(image_path, IMREAD_COLOR);
        // auto start = std::chrono::high_resolution_clock::now();
        cv::resize(img_origin, img_origin, cv::Size(188, 120));
        cv::Mat color_image;
        cv::cvtColor(img_origin, color_image, COLOR_BGR2GRAY);
        processor.setImage(color_image);
        // processor.updatePerspectiveImage();  // 使用预计算的映射表更新逆透视图像
        processor.find_line();
        // for (uint16_t i = 0; i < image_h * 3; i++) {
        //     std::cout << "points_l[" << i << "][0] = " << processor.points_l[i][0]
        //               << ", points_l[" << i << "][1] = " << processor.points_l[i][1] << std::endl;
        // }

        // cv::cvtColor(processor.PerImg_ip, colorImage, cv::COLOR_GRAY2BGR);
        cv::cvtColor(processor.mt9v03x_image, processor.disp_color_image, cv::COLOR_GRAY2BGR);
        processor.draw_line_1(processor.disp_color_image,processor.l_border,cv::Scalar(255, 0, 0), use_h - 1);
        processor.draw_line_1(processor.disp_color_image,processor.r_border,cv::Scalar(255, 0, 0), use_h - 1);
        processor.draw_line_1(processor.disp_color_image,processor.center_line,cv::Scalar(0, 255, 0), use_h - 1);
        // cv::circle(processor.disp_color_image,cv::Point(processor.center_line[use_h-20], use_h-20),1,cv::Scalar(0, 255, 255), 5);
        for (int i = 0; i < processor.data_stastics_l; i++) {
            circle(processor.disp_color_image, Point(processor.points_l[i][0] + 2, processor.points_l[i][1]), 2, Scalar(0, 0, 255), FILLED); // Display starting point in blue
        }

        for (int i = 0; i < processor.data_stastics_l; i++) {
            circle(processor.disp_color_image, Point(processor.points_r[i][0] + 2, processor.points_r[i][1]), 2, Scalar(0, 0, 255), FILLED); // Display starting point in blue
        }

        // for (int i = 0; i < processor.data_stastics_l; i++) {
        //     circle(img_lhc, Point(processor.points_l[i][0] + 2, processor.points_l[i][1]), 2, Scalar(0, 0, 255), FILLED); // Display starting point in blue
        // }

        cv::imshow("Perspective Image", processor.disp_color_image);
        cv::imshow("Image", processor.mt9v03x_image);
        cv::imshow("img_lhc", img_origin);

        // 结束计时
        // std::cout << "Image processing time: " << elapsed.count() << " ms" << std::endl;
        // auto end = std::chrono::high_resolution_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        // std::cout << "代码运行时间: " << duration.count() << " 毫秒" << std::endl;
        if (cv::waitKey(10) == 'q') {
            break;
        }
        // printf("按任意键继续...\n");
        // std::cin.get(); // 等待用户输入
    }


    // destroyWindow("img_lhc");

    CvCapture.release();
    waitKey(0);
    return 0;
}
