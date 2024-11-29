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
int mission_end_cycle = 1e9;
bool mission_end_not_activated = true;

int cycle_count = 0;
int direction = 0;
int quadrant_count = 0;

int R = 0, G = 0, B = 0;

int hue = 0;

//--------------------------------------------------
// Image variables

cv::Mat raw_frame(480, 640, CV_8UC3);
cv::Mat frame(120, 320, CV_8UC3);
cv::Mat hsv(120, 320, CV_8UC3);

//--------------------------------------------------
// Wall variables

float left_wall, right_wall;

//--------------------------------------------------
// Map lines variables

const int line_cycle_delay = 10;

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

void process_hsv(uchar*& hsv_p) {
    blue_line_pixel_count = 0;
    orange_line_pixel_count = 0;

    left_wall = 0;
    right_wall = 0;

    int i = 0, j = 0;
    int hue, sat, val;
    for (int p = 0; p < 120 * 320; p++) {
        hue = hsv_p[i    ];
        sat = hsv_p[i + 1];
        val = hsv_p[i + 2];
        if (sat > 60 && val > 90 && val < 240) {
            if (90 < hue && hue < 135) {
                blue_line_pixel_count++;
            }
        }

        if (sat > 30 && val > 60 && val < 240) {
            if (15 <= hue && hue <= 45) {
                orange_line_pixel_count++;
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

void extra_imagery(uchar*& hsv_p) {
    cv::Mat walls(120, 320, CV_8UC1);

    int i = 0;
    int hue, sat, val;
    for (int p = 0; p < 120 * 320; p++) {
        hue = hsv_p[i    ];
        sat = hsv_p[i + 1];
        val = hsv_p[i + 2];

        if (val < 70) {
            walls.data[p] = 255;
        }

        i += 3;
    }

    cv::imwrite("walls.png", walls);
}

void cycle(cv::VideoCapture& cap) {
    R = 122; G = 122; B = 122;
    cycle_count++;

    dir = 0;

    cap.read(raw_frame);

    process_frame(raw_frame.data, frame.data);

    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    process_hsv(hsv.data);

    update_lines();

    if (blue_line_state != 0 && direction == 0) direction = -1;
    if (orange_line_state != 0 && direction == 0) direction = 1;

    if (direction >= 0) {
        if (orange_line_state == 2) quadrant_count++;
    } else {
        if (blue_line_state == 2) quadrant_count++;
    }

    if (blue_line_state != 0) {R = 0; G = 0; B = 255;}
    if (orange_line_state != 0) {R = 255; G = 122; B = 0;}

    if (direction == 1) {
        dir = (left_wall - 0.5) * 75;
    } else if (direction == -1) {
        dir = (0.5 - right_wall) * 75;
    } else {
        if (left_wall > 0.5) dir = (left_wall - 0.5) * 75;
        if (right_wall > 0.5) dir = (0.5 - right_wall) * 75;
    }

    if (quadrant_count == 12 && mission_end_not_activated) {
        mission_end_not_activated = false;
        mission_end_cycle = cycle_count + 100;
    }

    if (mission_end_cycle < cycle_count) STOP = true;

    servo(dir);

    LED_hsv(hue, 255, 255);

    hue = (hue + 1) % 180;

    if (R != 122 || G != 122 || B != 122) LED_rgb(R, G, B);

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

    motor(1);

    while (button_sum < 50 && !STOP) {
        cycle(cap);

        if (is_button_down()) button_sum++;
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
    cv::imwrite("input.png", raw_frame);
    cv::imwrite("frame.png", frame);

    button_sum = 0;
    while (button_sum < 10) {
        if (is_button_down()) button_sum++;
    }

    LED_rgb(0, 0, 0);
    cap.release();
    return 0;
}
