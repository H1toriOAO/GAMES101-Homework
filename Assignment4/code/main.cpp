#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    auto &p1 = control_points[0];
    auto &p2 = control_points[1];
    auto &p3 = control_points[2];
    auto &p4 = control_points[3];

    auto l_12 = (1 - t) * p1 + t * p2;
    auto l_23 = (1 - t) * p2 + t * p3;
    auto l_34 = (1 - t) * p3 + t * p4;

    auto l_13 = (1 - t) * l_12 + t * l_23;
    auto l_24 = (1 - t) * l_23 + t * l_34;

    auto l_14 = (1 - t) * l_13 + t * l_24;

    return cv::Point2f(l_14);

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    for(float t = 0; t <= 1; t += 0.001) {
        auto point = recursive_bezier(control_points, t);
        for(int i = -1; i <= 1; i++) {
            for(int j = -1; j <= 1; j++) {
                if(point.x + i > 700 || point.x + i < 0 || point.y + j > 700 || point.y + j < 0) {
                    continue;
                }
                float dist = std::sqrt(std::pow(0.5 + int(point.x + i) - point.x, 2) + std::pow(0.5 + int(point.y + j) - point.y, 2));
                float mul = 1 - std::sqrt(2) / 3.0 * dist;
                window.at<cv::Vec3b>(point.y + j, point.x + i)[1] = std::fmax(255 * mul, window.at<cv::Vec3b>(point.y + j, point.x + i)[1]);
            }
        }
    }


}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
