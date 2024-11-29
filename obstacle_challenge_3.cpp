#pragma GCC optimize("Ofast")
#pragma GCC optimize("unroll-loops")
#include <opencv2/opencv.hpp>
#include <pigpio.h>
#include <iostream>
#include <cstdlib>

#define LED_R 25
#define LED_G 24
#define LED_B 23
#define BUTTON_PIN 5

#define SERVO_PIN 18
#define MOTOR_ENA 17
#define MOTOR_IN1 27
#define MOTOR_IN2 22

//--------------------------------------------------

bool STOP = false;
int wall_aligment_state = 0;
int direction_swap_cycle_threshould = -1;

int cycle_count = 0;
int direction = 0;
int quadrant_count = 0;

int red_index = 0;
int green_index = 1;

int R = 0, G = 0, B = 0;

bool direction_swap_havent_started = true;
bool traffic_index_not_changed_on_cycle_12 = true;

const int parking_near_outer_wall_setup_quadrant = 12;
const int parking_caused_program_override_quadrant_threshould = 13;
const int parking_wall_detected_as_a_wall_quadrant_threshould = 14;

//--------------------------------------------------
// Image variables

cv::Mat raw_frame(480, 640, CV_8UC3);
cv::Mat frame(120, 320, CV_8UC3);
cv::Mat hsv(120, 320, CV_8UC3);

//--------------------------------------------------
// Traffic light variables

cv::Mat red_mask(120, 320, CV_8UC1);
cv::Mat green_mask(120, 320, CV_8UC1);

const int PARALELIPIPED_MIN_AREA = 75;

int last_detected_traffic_light = -1;

std::array<int, 4> target;
// Biggest traffic light {x, y, area, type}

std::vector<std::vector<cv::Point>> red_box;
std::vector<std::vector<cv::Point>> green_box;

//--------------------------------------------------
// Wall variables

float left_wall, right_wall;

//--------------------------------------------------
// Map lines variables

const int line_cycle_delay = 20; // 1.5 seconds of runtime at 30 ms / cycle

const int blue_line_threshould = 1500;
const int orange_line_threshould = 1500;

int blue_line_pixel_count = 0;
int blue_line_next_allowed_cycle = 0;

int orange_line_pixel_count = 0;
int orange_line_next_allowed_cycle = 0;

bool blue_line_detected = false;
bool orange_line_detected = false;

int blue_line_state = 0;
int orange_line_state = 0;

//--------------------------------------------------
// P controller variables

const float kp = 0.25;
int Err = 0;

float dir = 0;

//--------------------------------------------------
// Parking

const int PARKING_MIN_AREA = 1000;

cv::Mat purple_mask(120, 320, CV_8UC1);

std::array<int, 3> parking;
// parking gate {x, y, area}

std::vector<std::vector<cv::Point>> purple_box;

//--------------------------------------------------


void LED_rgb(int r, int g, int b) {
    gpioPWM(LED_R, r);
    gpioPWM(LED_G, g);
    gpioPWM(LED_B, b);
}

void LED_hsv(int hue, int sat, int val) {
    int r, g, b;
    // Normalize hue to the range [0, 360] for full color spectrum
    float h = hue * 2.0f;    // Multiply by 2 to scale to [0, 360]
    float s = sat / 255.0;   // Saturation (1.0 for full saturation)
    float v = val / 255.0;   // Value (1.0 for full brightness)

    int i = static_cast<int>(h / 60.0f) % 6; // Find which sector of the color wheel we're in
    float f = h / 60.0f - i; // Calculate the fractional part of hue

    float p = v * (1.0f - s);
    float q = v * (1.0f - f * s);
    float t = v * (1.0f - (1.0f - f) * s);

    switch (i) {
        case 0:
            r = v * 255;
            g = t * 255;
            b = p * 255;
            break;
        case 1:
            r = q * 255;
            g = v * 255;
            b = p * 255;
            break;
        case 2:
            r = p * 255;
            g = v * 255;
            b = t * 255;
            break;
        case 3:
            r = p * 255;
            g = q * 255;
            b = v * 255;
            break;
        case 4:
            r = t * 255;
            g = p * 255;
            b = v * 255;
            break;
        case 5:
            r = v * 255;
            g = p * 255;
            b = q * 255;
            break;
    }

    gpioPWM(LED_R, r);
    gpioPWM(LED_G, g);
    gpioPWM(LED_B, b);
}

bool is_button_down() {
    if (gpioRead(BUTTON_PIN) == 0) {
        return true;
    } else {
        return false;
    }
}

void servo(float angle) {
    angle += 90;
    int deviation = 45;
    if (angle < 90 - deviation) angle = 90 - deviation;
    if (angle > 90 + deviation) angle = 90 + deviation;
    angle += 7;

    int pulseWidth = (angle * 2000 / 180) + 500;
    gpioServo(SERVO_PIN, pulseWidth);
}

void motor(float speed) {
    speed = -speed * 255;

    if (speed > 0) {
        gpioWrite(MOTOR_IN1, PI_HIGH);
        gpioWrite(MOTOR_IN2, PI_LOW);
    } else if (speed < 0) {
        speed = -speed;
        gpioWrite(MOTOR_IN1, PI_LOW);
        gpioWrite(MOTOR_IN2, PI_HIGH);
    } else {
        gpioWrite(MOTOR_IN1, PI_LOW);
        gpioWrite(MOTOR_IN2, PI_LOW);
    }

    if (speed > 255) speed = 255;

    gpioPWM(MOTOR_ENA, speed);
}

void Setup_GPIO() {
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio setup failed!\n";
        exit(1);
    }
    gpioSetMode(SERVO_PIN, PI_OUTPUT);
    gpioSetMode(MOTOR_ENA, PI_OUTPUT);
    gpioSetMode(MOTOR_IN1, PI_OUTPUT);
    gpioSetMode(MOTOR_IN2, PI_OUTPUT);

    gpioSetMode(LED_R, PI_OUTPUT);
    gpioSetMode(LED_G, PI_OUTPUT);
    gpioSetMode(LED_B, PI_OUTPUT);

    gpioSetPWMrange(LED_R, 255);
    gpioSetPWMrange(LED_G, 255);
    gpioSetPWMrange(LED_B, 255);

    gpioSetPWMfrequency(LED_R, 800);
    gpioSetPWMfrequency(LED_G, 800);
    gpioSetPWMfrequency(LED_B, 800);

    gpioSetMode(BUTTON_PIN, PI_INPUT);
    gpioSetPullUpDown(BUTTON_PIN, PI_PUD_UP);
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

void clear_buffer (cv::VideoCapture& cap, int amount) {
    for (int i = 0; i < amount; i++) {
        cap.read(raw_frame);
    }
}

void process_frame(uchar*& raw_frame_p, uchar*& frame_p) {
    int i = 0, f = 115197; // 120 * 320 * 3 - 3, writes in reverse because camera output is rotated

    for (int y = 240; y < 480; y+=2) {
        for (int x = 0; x < 640; x+=2) {
            frame_p[f    ] = raw_frame_p[i    ];
            frame_p[f + 1] = raw_frame_p[i + 1];
            frame_p[f + 2] = raw_frame_p[i + 2];

            i += 6;
            f -= 3;
        }
        i += 1920; // 640 * 3
    }
}

void process_hsv(uchar*& hsv_p, uchar*& red_p, uchar*& green_p, uchar*& purple_p) {
    blue_line_pixel_count = 0;
    orange_line_pixel_count = 0;

    left_wall = 0;
    right_wall = 0;

    int i = 0, j = 0;
    int hue, sat, val;
    for (int p = 0; p < 120 * 320; p++) {
        red_p[p] = 0;
        green_p[p] = 0;
        purple_p[p] = 0;

        hue = hsv_p[i    ];
        sat = hsv_p[i + 1];
        val = hsv_p[i + 2];

        if (sat > 120 && val > 60 && val < 240) {
            if (hue < 15 || 175 < hue) {
                red_p[p] = 255;
            }
        }

        if (sat > 120 && val > 60 && val < 240) {
            if (45 < hue && hue < 90) {
                green_p[p] = 255;
            }
        }

        if (sat > 60 && val > 70 && val < 200) {
            if (90 < hue && hue < 135) {
                blue_line_pixel_count++;
            }
        }

        if (sat > 30 && val > 60 && val < 240) {
            if (15 <= hue && hue <= 45) {
                orange_line_pixel_count++;
            }
        }

        if (sat > 120 && val > 60 && val < 240) {
            if (135 <= hue && hue <= 175) {
                purple_p[p] = 255;

                if (quadrant_count < parking_wall_detected_as_a_wall_quadrant_threshould && val >= 70) {
                    if (j < 160) left_wall += 1;
                    else right_wall += 1;
                }
            }
        }

        if (val < 70) {
            if (j < 160) left_wall += 1;
            else right_wall += 1;
        }

        i += 3;
        j = (j + 1) % 320;
    }

    left_wall /= 160 * 80;
    right_wall /= 160 * 80;
}

void update_lines() {
    if (blue_line_pixel_count > blue_line_threshould) {
        blue_line_state = 1;

        if (cycle_count >= blue_line_next_allowed_cycle) {
            blue_line_detected = true;
        }
    } else {
        blue_line_state = 0;

        if (blue_line_detected) {
            blue_line_state = 2;
            blue_line_next_allowed_cycle = cycle_count + line_cycle_delay;
        }

        blue_line_detected = false;
    }

    if (orange_line_pixel_count > orange_line_threshould) {
        orange_line_state = 1;

        if (cycle_count >= orange_line_next_allowed_cycle) {
            orange_line_detected = true;
        }
    } else {
        orange_line_state = 0;

        if (orange_line_detected) {
            orange_line_state = 2;
            orange_line_next_allowed_cycle = cycle_count + line_cycle_delay;
        }

        orange_line_detected = false;
    }
}

void process_traffic_contours(std::vector<std::vector<cv::Point>> box, int type) {
    int x, y, area;
    cv::Rect boundingBox;

    for (const auto& contour : box) {
        area = cv::contourArea(contour);
        if (area > target[2]) {
            boundingBox = cv::boundingRect(contour);
            if (boundingBox.width < boundingBox.height) {
                cv::Moments moments = cv::moments(contour, false);
                cv::Point2f center(moments.m10 / moments.m00, moments.m01 / moments.m00);
                x = center.x;
                y = center.y;
                target = {x, y, area, type};
            }
        }
    }
}

void process_parking_contours(std::vector<std::vector<cv::Point>> box) {
    int x, y, area;
    cv::Rect boundingBox;

    for (const auto& contour : box) {
        area = cv::contourArea(contour);
        if (area > parking[2]) {
            cv::Moments moments = cv::moments(contour, false);
            cv::Point2f center(moments.m10 / moments.m00, moments.m01 / moments.m00);
            x = center.x;
            y = center.y;
            parking = {x, y, area};
        }
    }
}

void draw(cv::Mat& frame, std::vector<std::vector<cv::Point>> box_red, std::vector<std::vector<cv::Point>> box_grn) {
    // Draws traffic light bounding boxes on frame.
    // Only used after program finish/

    cv::drawContours(frame, box_red, -1, cv::Scalar(255, 0, 122), 2);
    cv::drawContours(frame, box_grn, -1, cv::Scalar(255, 122, 0), 2);

    cv::Rect boundingBox;

    for (int i = 0; i < box_red.size(); ++i) {
        const std::vector<cv::Point>& contour = box_red[i];
        double area = cv::contourArea(contour);
        if (area > PARALELIPIPED_MIN_AREA) {
            cv::drawContours(frame, box_red, i, cv::Scalar(0, 0, 255), 2);
            cv::Moments moments = cv::moments(contour, false);
            cv::Point2f center(moments.m10 / moments.m00, moments.m01 / moments.m00);
            cv::circle(frame, center, 5, cv::Scalar(0, 0, 255), -1);

			boundingBox = cv::boundingRect(contour);
			if (boundingBox.width < boundingBox.height) {
				cv::rectangle(frame, boundingBox, cv::Scalar(0, 0, 255), 3);
			} else {
				cv::rectangle(frame, boundingBox, cv::Scalar(255, 0, 122), 3);
			}
        }
    }
    for (int i = 0; i < box_grn.size(); ++i) {
        const std::vector<cv::Point>& contour = box_grn[i];
        double area = cv::contourArea(contour);
        if (area > PARALELIPIPED_MIN_AREA) {
            cv::drawContours(frame, box_grn, i, cv::Scalar(0, 255, 0), 2);
            cv::Moments moments = cv::moments(contour, false);
            cv::Point2f center(moments.m10 / moments.m00, moments.m01 / moments.m00);
            cv::circle(frame, center, 5, cv::Scalar(0, 255, 0), -1);

			boundingBox = cv::boundingRect(contour);
			if (boundingBox.width < boundingBox.height) {
				cv::rectangle(frame, boundingBox, cv::Scalar(0, 255, 0), 3);
			} else {
				cv::rectangle(frame, boundingBox, cv::Scalar(255, 122, 0), 3);
			}
        }
    }
}

void extra_imagery(uchar*& hsv_p) {
    // Creates and saves masks for objects that don't store them as an image.
    // walls and lines to be exact

    cv::Mat blue_mask(120, 320, CV_8UC1);
    cv::Mat orange_mask(120, 320, CV_8UC1);
    cv::Mat walls(120, 320, CV_8UC1);

    int i = 0;
    int hue, sat, val;
    for (int p = 0; p < 120 * 320; p++) {
        blue_mask.data[p] = 0;
        orange_mask.data[p] = 0;

        hue = hsv_p[i    ];
        sat = hsv_p[i + 1];
        val = hsv_p[i + 2];

        if (sat > 60 && val > 70 && val < 200) {
            if (90 < hue && hue < 135) {
                blue_mask.data[p] = 255;
            }
        }


        if (sat > 30 && val > 60 && val < 240) {
            if (15 <= hue && hue <= 45) {
                orange_mask.data[p] = 255;
            }
        }

        if (val < 70) {
            walls.data[p] = 255;
        }

        i += 3;
    }

    cv::imwrite("blue.png", blue_mask);
    cv::imwrite("orange.png", orange_mask);
    cv::imwrite("walls.png", walls);
}

void cycle(cv::VideoCapture& cap) {
    R = 122; G = 122; B = 122;
    cycle_count++;

    dir = 0;

    target = {160, 0, PARALELIPIPED_MIN_AREA, -1};
    parking = {160, 0, PARKING_MIN_AREA};

    cap.read(raw_frame);

    process_frame(raw_frame.data, frame.data);

    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    process_hsv(hsv.data, red_mask.data, green_mask.data, purple_mask.data);

    cv::findContours(   red_mask,    red_box, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours( green_mask,  green_box, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(purple_mask, purple_box, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    process_traffic_contours(  red_box,   red_index);
    process_traffic_contours(green_box, green_index);
    process_parking_contours(purple_box);

    update_lines();

    if (blue_line_state != 0 && direction == 0) direction = -1;
    if (orange_line_state != 0 && direction == 0) direction = 1;

    if (direction >= 0) {
        if (orange_line_state == 2) quadrant_count++;
    } else {
        if (blue_line_state == 2) quadrant_count++;
    }

    if (target[3] == 1 || target[3] == 2) { // Green
        Err = -((180 + target[1] * 2) - target[0]);

        if (target[2] > 2000) last_detected_traffic_light = 1;
    } else if (target[3] == 0 || target[3] == 3) { // Red
        Err = (target[0] - (140 - target[1] * 2));

        if (target[2] > 2000) last_detected_traffic_light = 0;
    } else {
        Err = 0;
    }

    if (target[3] % 2 == 1) {R = 0; G = 255; B = 0;}
    if (target[3] % 2 == 0) {R = 255; G = 0; B = 0;}

    if (blue_line_state != 0) {R = 0; G = 0; B = 255;}
    if (orange_line_state != 0) {R = 255; G = 122; B = 0;}

    dir = Err * kp;

    if (direction >= 0) {
        if (left_wall > 0.625) {
            dir = 45;
        } else if (right_wall > 0.625) {
            dir = -45;
        }
    } else {
        if (right_wall > 0.625) {
            dir = -45;
        } else if (left_wall > 0.625) {
            dir = 45;
        }
    }

    if (quadrant_count == 8 && direction_swap_havent_started) {
        direction *= -1;
        direction_swap_cycle_threshould = cycle_count + 20;
        direction_swap_havent_started = false;
    }

    if (cycle_count < direction_swap_cycle_threshould) {
        if (direction >= 0) dir = 45;
        else dir = -45;
    }

    if (quadrant_count == parking_near_outer_wall_setup_quadrant && traffic_index_not_changed_on_cycle_12 && abs(dir) < 15) {
        // it makes the program steer to the outer wall side when avoiding traffic lights.
        // by rules robot can drive around traffic lights in any direction after 3 laps.
        if (direction ==  1) red_index = 2;
        if (direction == -1) green_index = 3;
        traffic_index_not_changed_on_cycle_12 = false;

        motor(0.5);
    }

    if (quadrant_count >= parking_caused_program_override_quadrant_threshould) {
        if (wall_aligment_state == 0) {
            dir = 0;

            if (left_wall + right_wall > 0.8) wall_aligment_state = 1;
        } else {
            if (direction >= 0) {
                dir = (left_wall - 0.8) * 50;
                if (right_wall > 0.5) dir = 45;
            } else {
                dir = (0.8 - right_wall) * 50;
                if (left_wall > 0.5) dir = -45;
            }

            if (parking[2] > 3400) STOP = true;

            R = 255; G = 0; B = 255;
        }
    }

    servo(dir);

    LED_rgb(R, G, B);


    //cv::Point big(target[0], target[1]);
    //cv::circle(frame, big, 7, cv::Scalar(255, 255, 255), 2);

    //for (int i = 0; i < 4; i++) std::cout << target[i] << " ";
    //std::cout << left_wall << " " << right_wall << " " << Err << " " << dir << "\n";
    //std::cout << blue_line_pixel_count << " " << orange_line_pixel_count << " " << direction << " " << quadrant_count << "\n";
}

int main() {
    Setup_GPIO();
    cv::VideoCapture cap = Setup_Camera(30);

    LED_rgb(0, 0, 255);

    int button_sum = 0;
    while (button_sum < 10) {
        if (is_button_down()) button_sum++;
    }
    button_sum = 0;

    auto start = std::chrono::steady_clock::now();

    clear_buffer(cap, 10);
    cycle(cap);
    LED_rgb(0, 255, 0);

    motor(0.75);

    while (button_sum < 50 && !STOP) {
        cycle(cap);

        if (is_button_down()) button_sum++;
    }

    if (button_sum < 30) {
        motor(0.5);
        if (direction >= 0) {
            servo(30);
            time_sleep(0.45);
            servo(-30);
            time_sleep(1.3);
            servo(0);
            time_sleep(0.3);
        } else {
            motor(0.5);
            servo(-30);
            time_sleep(0.45);
            servo( 30);
            time_sleep(1.3);
            servo(0);
            time_sleep(0.3);
        }
    }

    motor(0);
    servo(0);


    LED_rgb(0, 0, 255);

    auto end = std::chrono::steady_clock::now();
    auto duration = end - start;
    float full_time = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    std :: cout << "\n";
    std :: cout << "time         : " << full_time / 1000.0 << " s\n";
    std :: cout << "cycle amount : " << cycle_count << " cycles\n";
    std :: cout << "speed        : " << full_time / cycle_count << " ms / cycle\n";

    extra_imagery(hsv.data);
    draw(frame, red_box, green_box);
    cv::imwrite("input.png", raw_frame);
    cv::imwrite("frame.png", frame);
    cv::imwrite("red.png", red_mask);
    cv::imwrite("green.png", green_mask);
    cv::imwrite("purple.png", purple_mask);

    button_sum = 0;
    while (button_sum < 10) {
        if (is_button_down()) button_sum++;
    }

    LED_rgb(0, 0, 0);
    cap.release();
    return 0;
}
