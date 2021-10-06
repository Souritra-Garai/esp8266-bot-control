#include <Arduino.h>

#include <ros.h>

#include "MotorController.h"

#include "ROSBotController.h"

#define PWM_PIN_L D2
#define DIRECTION_PIN_L D4
#define ENCODER_PIN_A_L D5
#define ENCODER_PIN_B_L D6

#define PWM_PIN_R D1
#define DIRECTION_PIN_R D3
#define ENCODER_PIN_A_R D7
#define ENCODER_PIN_B_R 3U

#define CONTROL_LOOP_FREQUENCY 50.0 // Hz

#define PID_Kp  3500
#define PID_Ki  1250
#define PID_Kd  2

#define ENCODER_COUNTS_PER_ROTATION 4160

#define ROS_ROUTINE_CALL_TIME_PERIOD 1E5 // us

MotorController motor_controller_l(
  DIRECTION_PIN_L,
  ENCODER_PIN_A_L,
  ENCODER_PIN_B_L,
  CONTROL_LOOP_FREQUENCY,
  ENCODER_COUNTS_PER_ROTATION
);

MotorController motor_controller_r(
  DIRECTION_PIN_R,
  ENCODER_PIN_A_R,
  ENCODER_PIN_B_R,
  CONTROL_LOOP_FREQUENCY,
  ENCODER_COUNTS_PER_ROTATION,
  true
);

ROSBotController bot_controller(
  motor_controller_l,
  motor_controller_r
);

const char* ssid     = "TP-Link_79FF";
const char* password = "39314536";
IPAddress server(192,168,0,151); // (192,168,214,91);
const uint16_t serverPort = 11411;

ros::NodeHandle node_handle;

float current_time;

float last_ros_routine_call_time;
float last_control_routine_call_time;

void setup()
{
	// Serial.begin(115200);

	analogWriteFreq(10000);
	analogWriteResolution(15);
	motor_controller_l.setMaxControllerOutput((1 << 15) - 1);
	motor_controller_r.setMaxControllerOutput((1 << 15) - 1);

	motor_controller_l.set_polynomial_coefficients_positive_value_positive_acceleration(7548, 3739, -780, 93.8, -3.58);
	motor_controller_l.set_polynomial_coefficients_positive_value_negative_acceleration(10664, 977, -71, 23.8, -1.22);
	motor_controller_l.set_polynomial_coefficients_negative_value_negative_acceleration(-6889, 4173, 977, 119, 4.52);
	motor_controller_l.set_polynomial_coefficients_negative_value_positive_acceleration(-10622, 1025, 144, 34.5, 1.62);

	motor_controller_r.set_polynomial_coefficients_positive_value_positive_acceleration(7548, 3739, -780, 93.8, -3.58);
	motor_controller_r.set_polynomial_coefficients_positive_value_negative_acceleration(10664, 977, -71, 23.8, -1.22);
	motor_controller_r.set_polynomial_coefficients_negative_value_negative_acceleration(-6889, 4173, 977, 119, 4.52);
	motor_controller_r.set_polynomial_coefficients_negative_value_positive_acceleration(-10622, 1025, 144, 34.5, 1.62);

	// Serial.println();
	// Serial.print("Connecting to ");
	// Serial.println(ssid);

	WiFi.begin(ssid, password);
	while (WiFi.status() != WL_CONNECTED) {
	delay(500);
	// Serial.print(".");
	}
	// Serial.println("");
	// Serial.println("WiFi connected");
	// Serial.println("IP address: ");
	// Serial.println(WiFi.localIP());

	node_handle.getHardware()->setConnection(server, serverPort);
	node_handle.initNode();

	// Serial.print("IP = ");
	// Serial.println(node_handle.getHardware()->getLocalIP());

	node_handle.loginfo("Node initialised");

	bot_controller.initialize(node_handle);

	motor_controller_l.setPIDGains(PID_Kp, PID_Ki, PID_Kd);
	motor_controller_r.setPIDGains(PID_Kp, PID_Ki, PID_Kd);

	node_handle.loginfo("Initialized successfully");

	current_time = micros();
	last_ros_routine_call_time = current_time;
	last_control_routine_call_time = current_time;

	motor_controller_r.enablePIDControl();
	motor_controller_l.enablePIDControl();
}

void loop()
{
	current_time = micros();

	if (current_time - last_control_routine_call_time > 1E6/CONTROL_LOOP_FREQUENCY)
	{
		motor_controller_l.updateAngularState();
		motor_controller_r.updateAngularState();

		motor_controller_l.spinMotor();
		motor_controller_r.spinMotor();

		analogWrite(PWM_PIN_L, motor_controller_l.getControllerOutput());
		analogWrite(PWM_PIN_R, motor_controller_r.getControllerOutput());

		last_control_routine_call_time = current_time;
	}

	if (current_time - last_ros_routine_call_time > ROS_ROUTINE_CALL_TIME_PERIOD)
	{
		bot_controller.publish();

		node_handle.spinOnce();

		last_ros_routine_call_time = current_time;
	}
}