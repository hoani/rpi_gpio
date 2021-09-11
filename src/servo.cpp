#include "ros/ros.h"
#include "rpi_gpio/ServoAngle.h"
#include "rpi_gpio/ServoCommand.h"

#include <csignal>
#include <iostream>

#include <pigpiod_if2.h>

class Servo{
public:
    Servo(int pi_id, int pin) : 
        _pub(_n.advertise<rpi_gpio::ServoAngle>("servo/angle", 1))
        {
            _srv = _n.advertiseService("servo/command", &Servo::command_callback, this);
            _pi_id = pi_id;
            _pin = pin;
            set_mode(pi_id, _pin, PI_OUTPUT);
            set_PWM_frequency(pi_id, _pin, SERVO_FREQ);
            set_PWM_range(pi_id, _pin, 20000); // 1us/count
        }

    void run() {
        ros::Rate rate(1);
        while (ros::ok()) {
            rpi_gpio::ServoAngle data;
            data.angle = _angle;

            _pub.publish(data);

            ros::spinOnce();
            rate.sleep();
        }
    }

    bool command_callback(rpi_gpio::ServoCommand::Request &req, rpi_gpio::ServoCommand::Response &res) {
        if (req.enable) {
            _angle = req.angle;
            int count = 1500 + (int)(_angle * 1000.0/90.0);
            set_PWM_dutycycle(_pi_id, _pin, count);
        } else {
            set_PWM_dutycycle(_pi_id, _pin, 0);
        }
        res.angle = _angle;
        return true;
    }

private:
    ros::NodeHandle _n;
    ros::Publisher _pub;
    ros::ServiceServer _srv;

    int _pi_id = 0;
    int _pin = 0;
    double _angle = 0;

    const int SERVO_FREQ = 50;
};

static int pi_id = -1;

void shutdown() {
    std::cout << "Shutting down connection to pigpiod\n";
    if (pi_id >= 0) {
        pigpio_stop(pi_id);
    }
}

void at_signal(int i) {
    shutdown();
}

int main(int argc, char *argv[]) {
    std::signal(SIGINT, at_signal);

    pi_id = pigpio_start("localhost", "8888");
    if (pi_id < 0) {
        std::cout << "Connecting to pigpiod failed\n";
        return 1;
    }

    ros::init(argc, argv, "servo");

    Servo servo = Servo(pi_id, 24);
    servo.run();

    shutdown();
    return 0;
}
