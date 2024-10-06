#pragma GCC optimize("Ofast")
#pragma GCC optimize("unroll-loops")
#include <opencv2/opencv.hpp>
#include <pigpio.h>
#include <iostream>

#define SERVO_PIN 18
#define MOTOR_ENA 17
#define MOTOR_IN1 27
#define MOTOR_IN2 22

int paralelipiped_count = 0;
int paralelipiped_min_area = 75;
std::array<int, 4> target; // x y area type

cv::Mat raw_frame(480, 640, CV_8UC3);
cv::Mat frame(120, 320, CV_8UC3);
cv::Mat hsv(120, 320, CV_8UC3);

cv::Mat red_mask(120, 320, CV_8UC1);
cv::Mat green_mask(120, 320, CV_8UC1);

std::vector<std::vector<cv::Point>> red_box;
std::vector<std::vector<cv::Point>> green_box;

float kp = 0.5;
float ki = 0;
float kd = 0;
float ky = 0;
int Err = 0;
int Sum = 0;
int Old = 0;
float pid = 0;

void servo(int angle) {
	angle += 90;
	if (angle < 40) angle = 40;
	if (angle > 140) angle = 140;
	int pulseWidth = (angle * 2000 / 180) + 500;
	gpioServo(SERVO_PIN, pulseWidth);
}

void speed(float sp) {
	sp = -sp * 255;
	if (sp > 0) {
		gpioWrite(MOTOR_IN1, PI_HIGH);
		gpioWrite(MOTOR_IN2, PI_LOW);
	} else if (sp < 0) {
		sp = -sp;
		gpioWrite(MOTOR_IN1, PI_LOW);
		gpioWrite(MOTOR_IN2, PI_HIGH);
	} else {
		gpioWrite(MOTOR_IN1, PI_LOW);
		gpioWrite(MOTOR_IN2, PI_LOW);
	}
	if (sp > 255) sp = 255;

	gpioPWM(MOTOR_ENA, sp);
}

cv::VideoCapture Setup_Camera(int fps) {
    cv::VideoCapture cap("/dev/video0", cv::CAP_V4L2);

    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open the camera" << std::endl;
        exit(-1);
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, fps);

    return cap;
}

void proccess_frame(uchar*& raw_frame_p, uchar*& frame_p) {
    int i = 0, j = 120 * 320 * 3 - 3; // camera output is rotated

    for (int y = 240; y < 480; y+=2) {
        for (int x = 0; x < 640; x+=2) {
            frame_p[j    ] = raw_frame_p[i    ];
            frame_p[j + 1] = raw_frame_p[i + 1];
            frame_p[j + 2] = raw_frame_p[i + 2];

            i += 6;
            j -= 3;
        }
        i += 320 * 6;
    }
}

void make_mask(uchar*& hsv_p, uchar*& mask_red_p, uchar*& mask_grn_p) {
    int i = 0;
    int hue, sat, val;
    for (int p = 0; p < 120 * 320; p++) {
        hue = hsv_p[i    ];
        sat = hsv_p[i + 1];
        val = hsv_p[i + 2];

		if ((sat > 120 && val > 30) || (sat > 80 && val > 100)) { // sat val settings
			if (hue < 10 || 165 < hue) {
				mask_red_p[p] = 255;
			} else {
				mask_red_p[p] = 0;
			}

			if (40 < hue && hue < 80) {
				mask_grn_p[p] = 255;
			} else {
				mask_grn_p[p] = 0;
			}
		}

        i += 3;
    }
}

void proccess_paralelipiped_contours(std::vector<std::vector<cv::Point>> box, int type) {
    int x, y, area;
	cv::Rect boundingBox;

    for (const auto& contour : box) {
        area = cv::contourArea(contour);
        if (area > target[2]) {
			boundingBox = cv::boundingRect(contour);
			if (boundingBox.width < boundingBox.height * 2) {
				cv::Moments moments = cv::moments(contour, false);
				cv::Point2f center(moments.m10 / moments.m00, moments.m01 / moments.m00);
				x = center.x;
				y = center.y;
				target = {x, y, area, type};
			}
        }
    }
}

void draw(cv::Mat& frame, std::vector<std::vector<cv::Point>> box_red, std::vector<std::vector<cv::Point>> box_grn) {
    cv::drawContours(frame, box_red, -1, cv::Scalar(255, 0, 122), 2);
    cv::drawContours(frame, box_grn, -1, cv::Scalar(255, 122, 0), 2);

	cv::Rect boundingBox;

    for (int i = 0; i < box_red.size(); ++i) {
        const std::vector<cv::Point>& contour = box_red[i];
        double area = cv::contourArea(contour);
        if (area > paralelipiped_min_area) {
            cv::drawContours(frame, box_red, i, cv::Scalar(0, 0, 255), 2);
            cv::Moments moments = cv::moments(contour, false);
            cv::Point2f center(moments.m10 / moments.m00, moments.m01 / moments.m00);
            cv::circle(frame, center, 5, cv::Scalar(0, 0, 255), -1);

			boundingBox = cv::boundingRect(contour);
			if (boundingBox.width < boundingBox.height * 2) {
				cv::rectangle(frame, boundingBox, cv::Scalar(0, 0, 255), 3);
			} else {
				cv::rectangle(frame, boundingBox, cv::Scalar(255, 0, 122), 3);
			}
        }
    }
    for (int i = 0; i < box_grn.size(); ++i) {
        const std::vector<cv::Point>& contour = box_grn[i];
        double area = cv::contourArea(contour);
        if (area > paralelipiped_min_area) {
            cv::drawContours(frame, box_grn, i, cv::Scalar(0, 255, 0), 2);
            cv::Moments moments = cv::moments(contour, false);
            cv::Point2f center(moments.m10 / moments.m00, moments.m01 / moments.m00);
            cv::circle(frame, center, 5, cv::Scalar(0, 255, 0), -1);

			boundingBox = cv::boundingRect(contour);
			if (boundingBox.width < boundingBox.height * 2) {
				cv::rectangle(frame, boundingBox, cv::Scalar(0, 255, 0), 3);
			} else {
				cv::rectangle(frame, boundingBox, cv::Scalar(255, 122, 0), 3);
			}
        }
    }
}

void cycle(cv::VideoCapture& cap) {
    target = {160, 0, paralelipiped_min_area, 0};
    cap.read(raw_frame);

    proccess_frame(raw_frame.data, frame.data);

    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    make_mask(hsv.data, red_mask.data, green_mask.data);

    cv::findContours(  red_mask,   red_box, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(green_mask, green_box, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    proccess_paralelipiped_contours(  red_box, 0);
    proccess_paralelipiped_contours(green_box, 1);

    cv::Point big(target[0], target[1]);
    cv::circle(frame, big, 7, cv::Scalar(255, 255, 255), 2);

    Err = target[0] - 160 + target[1] * ky;

    pid = Err * kp + Sum * ki + Old * kd;
    Sum += Err;
    Old =  Err;

    servo(pid);

    for (int i = 0; i < 4; i++) std::cout << target[i] << " "; std::cout << pid << "\n";
}

int main() {
    if (gpioInitialise() < 0) {
	    std::cerr << "pigpio initialization failed!\n";
	    return 1;
    }
    gpioSetMode(SERVO_PIN, PI_OUTPUT);
    gpioSetMode(MOTOR_ENA, PI_OUTPUT);
    gpioSetMode(MOTOR_IN1, PI_OUTPUT);
    gpioSetMode(MOTOR_IN2, PI_OUTPUT);

    cv::VideoCapture cap = Setup_Camera(45);

    servo(0);
    speed(0);

    time_sleep(1.0);
    speed(0.5);

    target = {0, 0, paralelipiped_min_area, 0};
    while (target[2] < 5000) cycle(cap);

    servo(-35);
    time_sleep(0.38);
    servo( 30);
    time_sleep(0.5);


    for (int i = 0; i < 7; i++) cap.read(raw_frame);

    target = {0, 0, paralelipiped_min_area, 0};
    while (target[2] < 5000) cycle(cap);

    servo( 35);
    time_sleep(0.5);
    servo(-30);
    time_sleep(0.35);

    speed(0);


    draw(frame, red_box, green_box);
    cv::imwrite("input.png", raw_frame);
    cv::imwrite("frame.png", frame);
    cv::imwrite("red.png", red_mask);
    cv::imwrite("green.png", green_mask);

    cap.release();
    return 0;
}
