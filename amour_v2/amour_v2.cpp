#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
using namespace std;
using namespace cv;

class LightDescriptor
{
    // 在识别以及匹配到灯条的功能中需要用到旋转矩形的长宽偏转角面积中心点坐标等
public:
    float width, length, angle, area;
    cv::Point2f center;

public:
    LightDescriptor() {};
    // 让得到的灯条套上一个旋转矩形，以方便之后对角度这个特殊因素作为匹配标准
    LightDescriptor(const cv::RotatedRect &light)
    {
        width = light.size.width;
        length = light.size.height;
        center = light.center;
        angle = light.angle;
        area = light.size.area();
    }
};

int main()
{
    // 读取视频
    VideoCapture video;
    video.open("amour_2.mp4");

    // 1. 从输入视频获取属性
    int frame_width = video.get(cv::CAP_PROP_FRAME_WIDTH);
    int frame_height = video.get(cv::CAP_PROP_FRAME_HEIGHT);
    double fps = video.get(cv::CAP_PROP_FPS);
    cv::Size frame_size(frame_width, frame_height);

    VideoWriter writer("processed_video.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps, frame_size);

    Mat frame, channel[3], binary, Gaussian;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    vector<Mat> channels;

    while (true)
    {
        // 读取帧
        video >> frame;

        // 操作完退出
        if (frame.empty())
        {
            break;
        }

        vector<Mat> channels;
        split(frame, channels);

        Mat binary, Gaussian;

        // 对红色通道进行二值化
        threshold(channels[2], binary, 240, 255, THRESH_BINARY);
        // 高斯模糊
        GaussianBlur(binary, Gaussian, Size(3, 3), 0);

        // 查找连通域
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(Gaussian, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);

        // 测试连通域
        // // 创建一个原图的拷贝用于绘制，避免在原图上直接操作
        // cv::Mat contour_output = frame.clone();

        // for (size_t i = 0; i < contours.size(); i++)
        // {
        //     // 可视化方法 A: 绘制轮廓本身
        //     // 参数：目标图像, 轮廓集合, 要绘制的轮廓索引(-1为全部), 颜色, 线条粗细
        //     cv::drawContours(contour_output, contours, i, cv::Scalar(0, 255, 0), 2); // 用绿色绘制轮廓

        //     // 可视化方法 B: 绘制包围盒
        //     cv::Rect bounding_box = cv::boundingRect(contours[i]);
        //     cv::rectangle(contour_output, bounding_box, cv::Scalar(0, 0, 255), 2); // 用红色绘制正包围盒
        // }

        // cv::imshow("Contours Result", contour_output);
        // waitKey(0);

        vector<LightDescriptor> lightInfos;

        // 开筛！
        for (int i = 0; i < contours.size(); i++)
        {
            // 求轮廓面积
            double area = contourArea(contours[i]);
            // 除去较小轮廓
            if (area < 1 || contours[i].size() <= 1 || area > 100)
                continue; // 相当于就是把这段轮廓去除掉
            // 用椭圆拟合区域得到外接矩形（特殊的处理方式：因为灯条是椭圆型的，所以用椭圆去拟合轮廓，再直接获取旋转外接矩形即可）
            RotatedRect Light_Rec = minAreaRect(contours[i]);

            // 长宽比和轮廓面积比限制（由于要考虑灯条的远近都被识别到，所以只需要看比例即可）
            if (Light_Rec.size.height / Light_Rec.size.width > 2)
                continue;
            lightInfos.push_back(LightDescriptor(Light_Rec));
        }

        // 匹配矩形
        for (size_t i = 0; i < lightInfos.size(); i++)
        {
            for (size_t j = i + 1; (j < lightInfos.size()); j++)
            {
                LightDescriptor &leftLight = lightInfos[i];
                LightDescriptor &rightLight = lightInfos[j];
                float angleGap_ = abs(leftLight.angle - rightLight.angle);
                // 由于灯条长度会因为远近而受到影响，所以按照比值去匹配灯条
                float LenGap_ratio = abs(leftLight.length - rightLight.length) / max(leftLight.length, rightLight.length);
                float dis = pow(pow((leftLight.center.x - rightLight.center.x), 2) + pow((leftLight.center.y - rightLight.center.y), 2), 0.5);
                float meanLen = (leftLight.width + rightLight.width) / 2;
                float ratio = dis / meanLen;
                float length_ratio = leftLight.length / rightLight.length;
                // 匹配不通过的条件
                if (angleGap_ > 15 ||
                    ratio > 4 ||
                    length_ratio < 0.5 ||
                    length_ratio > 1.5 ||
                    ratio < 1.5)
                {
                    continue;
                }
                // 绘制矩形
                Point center = Point((leftLight.center.x + rightLight.center.x) / 2, (leftLight.center.y + rightLight.center.y) / 2);
                RotatedRect rect = RotatedRect(center, Size(meanLen, dis), (leftLight.angle + rightLight.angle) / 2);
                Point2f vertices[4];
                rect.points(vertices);
                for (int i = 0; i < 4; i++)
                {
                    putText(frame, to_string(i + 1), vertices[i], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
                    line(frame, vertices[i], vertices[(i + 1) % 4], Scalar(0, 0, 255), 2.2);
                }
            }
        }
        writer.write(frame);
    }

    cout << "Finished processing. Releasing resources..." << endl;
    writer.release();
    video.release();
    destroyAllWindows();

    cout << "Output video saved as processed_video.mp4" << endl;
    return 0;
}