/* =============================================================================
   QUADRUPED CRAWLING ROBOT CONTROL SOFTWARE
   =============================================================================

   Project:     Remote Control Crawling Robot
   Hardware:    Arduino Pro 5V 16MHz with expansion board (perfboard with DC
   converter and bluetooth) 4 legs, 12 servos (3 per leg: coxa, femur, tibia)

   Description:
   This software controls a quadruped (4-legged) crawling robot. Each leg is
   actuated by three servos to provide three degrees of freedom. The robot can
   perform various movements including walking forward/backward, turning left/
   right, and several demonstration movements like waving and dancing.

   Version History:
   v1.0 - 2015/01/27 - panerqiang@sunfounder.com
          Initial release for Sunfounder Crawling Robot
   v1.1 - 2015/09/11 - Regis
          Modified for spider project
   v2.0 - 2025/11/25 - David Strasak
          Implementation, Code documentation improvements and formatting

   Requirements:
   - Arduino.h
   - Servo.h (Arduino servo library)
   - FlexiTimer2.h (timer interrupt library)

   Calibration Instructions:
   Before first use, the robot must be calibrated for accurate movement:
   1. Uncomment ADJUST definition, compile and upload
   2. Comment ADJUST, uncomment VERIFY definition
   3. Measure actual servo positions and update real_site[4][3] array
   4. Comment VERIFY, compile and upload final version

   Refer to product documentation for detailed calibration procedures.

   =============================================================================
 */

/* Includes
 * -------------------------------------------------------------------*/
#include <Arduino.h>
#include <FlexiTimer2.h>  //to set a timer to manage all servos
#include <Servo.h>        //to define and control servos
#include <avr/io.h>       //to read how much memory is left
/* Servos --------------------------------------------------------------------*/
// define 12 servos for 4 legs
Servo servo[4][3];
// define servos' ports - femur, tibia & coxa
const int servo_pin[4][3] = {{3, 4, 2}, {6, 7, 5}, {9, 10, 8}, {12, 13, 11}};
/* Size of the robot ---------------------------------------------------------*/
const float length_a = 100;
const float length_b = 110;
const float length_c = 27.5;
const float length_side = 71;
const float z_absolute = -28;
/* Constants for movement ----------------------------------------------------*/
const float z_default = -50, z_up = -30, z_boot = z_absolute;
const float x_default = 62, x_offset = 0;
const float y_start = 0, y_step = 40;
const float y_default = x_default;
/* variables for movement ----------------------------------------------------*/
volatile float site_now[4][3];  // real-time coordinates of the end of each leg
volatile float site_expect[4]
                          [3];  // expected coordinates of the end of each leg
float temp_speed[4][3];  // each axis' speed, needs to be recalculated before
// each movement
float move_speed;          // movement speed
float speed_multiple = 1;  // movement speed multiple
const float spot_turn_speed = 4;
const float leg_move_speed = 8;
const float body_move_speed = 3;
const float stand_seat_speed = 1;
volatile int rest_counter;  //+1/0.02s, for automatic rest
// functions' parameter
const float KEEP = 255;
// define PI for calculation
const float pi = 3.1415926;
/* Constants for turn --------------------------------------------------------*/
// temp length
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) +
                          pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos(
    (pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
// site for turn
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;
/* ---------------------------------------------------------------------------*/

// Function prototypes
void servo_attach(void);
void servo_detach(void);
void sit(void);
void stand(void);
void turn_left(unsigned int step);
void turn_right(unsigned int step);
void step_forward(unsigned int step);
void step_back(unsigned int step);
void body_left(int i);
void body_right(int i);
void hand_wave(int i);
void hand_shake(int i);
void head_up(int i);
void head_down(int i);
void body_dance(int i);
void servo_service(void);
void set_site(int leg, float x, float y, float z);
void wait_reach(int leg);
void wait_all_reach(void);
void cartesian_to_polar(volatile float& alpha, volatile float& beta,
                        volatile float& gamma, volatile float x,
                        volatile float y, volatile float z);
void polar_to_servo(int leg, float alpha, float beta, float gamma);
int freeMemory(void);
void adjust(void);

/**
 * @brief initialize the robot's default parameters, servo service, and servos
 */
void setup() {
   // start serial for debug
   Serial.begin(115200);
   Serial.println("Robot starts initialization");
   Serial.print("Free RAM (Start): ");
   Serial.println(freeMemory());
   // initialize servos
   servo_attach();
   Serial.println("Servos initialized");

   // Optional adjust position of the servos
   //  adjust();

   // initialize default parameter
   set_site(0, x_default - x_offset, y_start + y_step, z_boot);
   set_site(1, x_default - x_offset, y_start + y_step, z_boot);
   set_site(2, x_default + x_offset, y_start, z_boot);
   set_site(3, x_default + x_offset, y_start, z_boot);
   for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 3; j++) {
         site_now[i][j] = site_expect[i][j];
      }
   }
   // start servo service
   FlexiTimer2::set(20, servo_service);
   FlexiTimer2::start();
   Serial.println("Servo service started");
   Serial.println("Robot initialization Complete");
   Serial.print("Free RAM (End of setup): ");
   Serial.println(freeMemory());  // About 1367 left after setup
}

/**
 * @brief main loop demonstrating robot movements and actions
 */
void loop() {
   Serial.println("Stand");
   stand();
   delay(2000);
   Serial.println("Step forward");
   step_forward(5);
   delay(2000);
   Serial.println("Step back");
   step_back(5);
   delay(2000);
   // Serial.println("Turn left");
   // turn_left(5);
   // delay(2000);
   // Serial.println("Turn right");
   // turn_right(5);
   // delay(2000);
   Serial.println("Hand wave");
   hand_wave(3);
   delay(2000);
   Serial.println("Hand wave");
   hand_shake(3);
   delay(2000);
   Serial.println("Body dance");
   body_dance(10);
   delay(2000);
   Serial.println("Sit");
   sit();
   delay(5000);
}

/**
 * @brief attach all servos to their pins
 * @warning blocking function due to delays
 */
void servo_attach(void) {
   for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 3; j++) {
         servo[i][j].attach(servo_pin[i][j]);
         delay(100);
      }
   }
}

/**
 * @brief detach all servos from their pins
 */
void servo_detach(void) {
   for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 3; j++) {
         servo[i][j].detach();
         delay(100);
      }
   }
}

/**
 * @brief move robot to sitting position
 * @warning blocking function
 */
void sit(void) {
   move_speed = stand_seat_speed;
   for (int leg = 0; leg < 4; leg++) {
      set_site(leg, KEEP, KEEP, z_boot);
   }
   wait_all_reach();
}

/**
 * @brief move robot to standing position
 * @warning blocking function
 */
void stand(void) {
   move_speed = stand_seat_speed;
   for (int leg = 0; leg < 4; leg++) {
      set_site(leg, KEEP, KEEP, z_default);
   }
   wait_all_reach();
}

/**
 * @brief turn to left
 * @warning blocking function
 * @param step steps wanted to turn
 */
void turn_left(unsigned int step) {
   move_speed = spot_turn_speed;
   while (step-- > 0) {
      if (site_now[3][1] == y_start) {
         // leg 3&1 move
         set_site(3, x_default + x_offset, y_start, z_up);
         wait_all_reach();

         set_site(0, turn_x1 - x_offset, turn_y1, z_default);
         set_site(1, turn_x0 - x_offset, turn_y0, z_default);
         set_site(2, turn_x1 + x_offset, turn_y1, z_default);
         set_site(3, turn_x0 + x_offset, turn_y0, z_up);
         wait_all_reach();

         set_site(3, turn_x0 + x_offset, turn_y0, z_default);
         wait_all_reach();

         set_site(0, turn_x1 + x_offset, turn_y1, z_default);
         set_site(1, turn_x0 + x_offset, turn_y0, z_default);
         set_site(2, turn_x1 - x_offset, turn_y1, z_default);
         set_site(3, turn_x0 - x_offset, turn_y0, z_default);
         wait_all_reach();

         set_site(1, turn_x0 + x_offset, turn_y0, z_up);
         wait_all_reach();

         set_site(0, x_default + x_offset, y_start, z_default);
         set_site(1, x_default + x_offset, y_start, z_up);
         set_site(2, x_default - x_offset, y_start + y_step, z_default);
         set_site(3, x_default - x_offset, y_start + y_step, z_default);
         wait_all_reach();

         set_site(1, x_default + x_offset, y_start, z_default);
         wait_all_reach();
      } else {
         // leg 0&2 move
         set_site(0, x_default + x_offset, y_start, z_up);
         wait_all_reach();

         set_site(0, turn_x0 + x_offset, turn_y0, z_up);
         set_site(1, turn_x1 + x_offset, turn_y1, z_default);
         set_site(2, turn_x0 - x_offset, turn_y0, z_default);
         set_site(3, turn_x1 - x_offset, turn_y1, z_default);
         wait_all_reach();

         set_site(0, turn_x0 + x_offset, turn_y0, z_default);
         wait_all_reach();

         set_site(0, turn_x0 - x_offset, turn_y0, z_default);
         set_site(1, turn_x1 - x_offset, turn_y1, z_default);
         set_site(2, turn_x0 + x_offset, turn_y0, z_default);
         set_site(3, turn_x1 + x_offset, turn_y1, z_default);
         wait_all_reach();

         set_site(2, turn_x0 + x_offset, turn_y0, z_up);
         wait_all_reach();

         set_site(0, x_default - x_offset, y_start + y_step, z_default);
         set_site(1, x_default - x_offset, y_start + y_step, z_default);
         set_site(2, x_default + x_offset, y_start, z_up);
         set_site(3, x_default + x_offset, y_start, z_default);
         wait_all_reach();

         set_site(2, x_default + x_offset, y_start, z_default);
         wait_all_reach();
      }
   }
}

/**
 * @brief turn to right
 * @warning blocking function
 * @param step steps wanted to turn
 */
void turn_right(unsigned int step) {
   move_speed = spot_turn_speed;
   while (step-- > 0) {
      if (site_now[2][1] == y_start) {
         // leg 2&0 move
         set_site(2, x_default + x_offset, y_start, z_up);
         wait_all_reach();

         set_site(0, turn_x0 - x_offset, turn_y0, z_default);
         set_site(1, turn_x1 - x_offset, turn_y1, z_default);
         set_site(2, turn_x0 + x_offset, turn_y0, z_up);
         set_site(3, turn_x1 + x_offset, turn_y1, z_default);
         wait_all_reach();

         set_site(2, turn_x0 + x_offset, turn_y0, z_default);
         wait_all_reach();

         set_site(0, turn_x0 + x_offset, turn_y0, z_default);
         set_site(1, turn_x1 + x_offset, turn_y1, z_default);
         set_site(2, turn_x0 - x_offset, turn_y0, z_default);
         set_site(3, turn_x1 - x_offset, turn_y1, z_default);
         wait_all_reach();

         set_site(0, turn_x0 + x_offset, turn_y0, z_up);
         wait_all_reach();

         set_site(0, x_default + x_offset, y_start, z_up);
         set_site(1, x_default + x_offset, y_start, z_default);
         set_site(2, x_default - x_offset, y_start + y_step, z_default);
         set_site(3, x_default - x_offset, y_start + y_step, z_default);
         wait_all_reach();

         set_site(0, x_default + x_offset, y_start, z_default);
         wait_all_reach();
      } else {
         // leg 1&3 move
         set_site(1, x_default + x_offset, y_start, z_up);
         wait_all_reach();

         set_site(0, turn_x1 + x_offset, turn_y1, z_default);
         set_site(1, turn_x0 + x_offset, turn_y0, z_up);
         set_site(2, turn_x1 - x_offset, turn_y1, z_default);
         set_site(3, turn_x0 - x_offset, turn_y0, z_default);
         wait_all_reach();

         set_site(1, turn_x0 + x_offset, turn_y0, z_default);
         wait_all_reach();

         set_site(0, turn_x1 - x_offset, turn_y1, z_default);
         set_site(1, turn_x0 - x_offset, turn_y0, z_default);
         set_site(2, turn_x1 + x_offset, turn_y1, z_default);
         set_site(3, turn_x0 + x_offset, turn_y0, z_default);
         wait_all_reach();

         set_site(3, turn_x0 + x_offset, turn_y0, z_up);
         wait_all_reach();

         set_site(0, x_default - x_offset, y_start + y_step, z_default);
         set_site(1, x_default - x_offset, y_start + y_step, z_default);
         set_site(2, x_default + x_offset, y_start, z_default);
         set_site(3, x_default + x_offset, y_start, z_up);
         wait_all_reach();

         set_site(3, x_default + x_offset, y_start, z_default);
         wait_all_reach();
      }
   }
}

/**
 * @brief move robot forward
 * @warning blocking function
 * @param step steps wanted to go
 */
void step_forward(unsigned int step) {
   move_speed = leg_move_speed;
   while (step-- > 0) {
      if (site_now[2][1] == y_start) {
         // leg 2&1 move
         set_site(2, x_default + x_offset, y_start, z_up);
         wait_all_reach();
         set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
         wait_all_reach();
         set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
         wait_all_reach();

         move_speed = body_move_speed;

         set_site(0, x_default + x_offset, y_start, z_default);
         set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
         set_site(2, x_default - x_offset, y_start + y_step, z_default);
         set_site(3, x_default - x_offset, y_start + y_step, z_default);
         wait_all_reach();

         move_speed = leg_move_speed;

         set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
         wait_all_reach();
         set_site(1, x_default + x_offset, y_start, z_up);
         wait_all_reach();
         set_site(1, x_default + x_offset, y_start, z_default);
         wait_all_reach();
      } else {
         // leg 0&3 move
         set_site(0, x_default + x_offset, y_start, z_up);
         wait_all_reach();
         set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
         wait_all_reach();
         set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
         wait_all_reach();

         move_speed = body_move_speed;

         set_site(0, x_default - x_offset, y_start + y_step, z_default);
         set_site(1, x_default - x_offset, y_start + y_step, z_default);
         set_site(2, x_default + x_offset, y_start, z_default);
         set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
         wait_all_reach();

         move_speed = leg_move_speed;

         set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
         wait_all_reach();
         set_site(3, x_default + x_offset, y_start, z_up);
         wait_all_reach();
         set_site(3, x_default + x_offset, y_start, z_default);
         wait_all_reach();
      }
   }
}

/**
 * @brief move robot backward
 * @warning blocking function
 * @param step steps wanted to go
 */
void step_back(unsigned int step) {
   move_speed = leg_move_speed;
   while (step-- > 0) {
      if (site_now[3][1] == y_start) {
         // leg 3&0 move
         set_site(3, x_default + x_offset, y_start, z_up);
         wait_all_reach();
         set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
         wait_all_reach();
         set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
         wait_all_reach();

         move_speed = body_move_speed;

         set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
         set_site(1, x_default + x_offset, y_start, z_default);
         set_site(2, x_default - x_offset, y_start + y_step, z_default);
         set_site(3, x_default - x_offset, y_start + y_step, z_default);
         wait_all_reach();

         move_speed = leg_move_speed;

         set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
         wait_all_reach();
         set_site(0, x_default + x_offset, y_start, z_up);
         wait_all_reach();
         set_site(0, x_default + x_offset, y_start, z_default);
         wait_all_reach();
      } else {
         // leg 1&2 move
         set_site(1, x_default + x_offset, y_start, z_up);
         wait_all_reach();
         set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
         wait_all_reach();
         set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
         wait_all_reach();

         move_speed = body_move_speed;

         set_site(0, x_default - x_offset, y_start + y_step, z_default);
         set_site(1, x_default - x_offset, y_start + y_step, z_default);
         set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
         set_site(3, x_default + x_offset, y_start, z_default);
         wait_all_reach();

         move_speed = leg_move_speed;

         set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
         wait_all_reach();
         set_site(2, x_default + x_offset, y_start, z_up);
         wait_all_reach();
         set_site(2, x_default + x_offset, y_start, z_default);
         wait_all_reach();
      }
   }
}

/**
 * @brief shift robot body to the left
 * @param i distance to shift
 */
void body_left(int i) {
   set_site(0, site_now[0][0] + i, KEEP, KEEP);
   set_site(1, site_now[1][0] + i, KEEP, KEEP);
   set_site(2, site_now[2][0] - i, KEEP, KEEP);
   set_site(3, site_now[3][0] - i, KEEP, KEEP);
   wait_all_reach();
}

/**
 * @brief shift robot body to the right
 * @param i distance to shift
 */
void body_right(int i) {
   set_site(0, site_now[0][0] - i, KEEP, KEEP);
   set_site(1, site_now[1][0] - i, KEEP, KEEP);
   set_site(2, site_now[2][0] + i, KEEP, KEEP);
   set_site(3, site_now[3][0] + i, KEEP, KEEP);
   wait_all_reach();
}

/**
 * @brief wave one hand (leg 2 or 0 depending on position)
 * @param i number of waves to perform
 */
void hand_wave(int i) {
   float x_tmp;
   float y_tmp;
   float z_tmp;
   move_speed = 1;
   if (site_now[3][1] == y_start) {
      body_right(15);
      x_tmp = site_now[2][0];
      y_tmp = site_now[2][1];
      z_tmp = site_now[2][2];
      move_speed = body_move_speed;
      for (int j = 0; j < i; j++) {
         set_site(2, turn_x1, turn_y1, 50);
         wait_all_reach();
         set_site(2, turn_x0, turn_y0, 50);
         wait_all_reach();
      }
      set_site(2, x_tmp, y_tmp, z_tmp);
      wait_all_reach();
      move_speed = 1;
      body_left(15);
   } else {
      body_left(15);
      x_tmp = site_now[0][0];
      y_tmp = site_now[0][1];
      z_tmp = site_now[0][2];
      move_speed = body_move_speed;
      for (int j = 0; j < i; j++) {
         set_site(0, turn_x1, turn_y1, 50);
         wait_all_reach();
         set_site(0, turn_x0, turn_y0, 50);
         wait_all_reach();
      }
      set_site(0, x_tmp, y_tmp, z_tmp);
      wait_all_reach();
      move_speed = 1;
      body_right(15);
   }
}

/**
 * @brief shake one hand (leg 2 or 0 depending on position) up and down
 * @param i number of shakes to perform
 */
void hand_shake(int i) {
   float x_tmp;
   float y_tmp;
   float z_tmp;
   move_speed = 1;
   if (site_now[3][1] == y_start) {
      body_right(15);
      x_tmp = site_now[2][0];
      y_tmp = site_now[2][1];
      z_tmp = site_now[2][2];
      move_speed = body_move_speed;
      for (int j = 0; j < i; j++) {
         set_site(2, x_default - 30, y_start + 2 * y_step, 55);
         wait_all_reach();
         set_site(2, x_default - 30, y_start + 2 * y_step, 10);
         wait_all_reach();
      }
      set_site(2, x_tmp, y_tmp, z_tmp);
      wait_all_reach();
      move_speed = 1;
      body_left(15);
   } else {
      body_left(15);
      x_tmp = site_now[0][0];
      y_tmp = site_now[0][1];
      z_tmp = site_now[0][2];
      move_speed = body_move_speed;
      for (int j = 0; j < i; j++) {
         set_site(0, x_default - 30, y_start + 2 * y_step, 55);
         wait_all_reach();
         set_site(0, x_default - 30, y_start + 2 * y_step, 10);
         wait_all_reach();
      }
      set_site(0, x_tmp, y_tmp, z_tmp);
      wait_all_reach();
      move_speed = 1;
      body_right(15);
   }
}

/**
 * @brief tilt robot body upward (front legs down, back legs up)
 * @param i amount to tilt
 * @warning blocking function
 */
void head_up(int i) {
   set_site(0, KEEP, KEEP, site_now[0][2] - i);
   set_site(1, KEEP, KEEP, site_now[1][2] + i);
   set_site(2, KEEP, KEEP, site_now[2][2] - i);
   set_site(3, KEEP, KEEP, site_now[3][2] + i);
   wait_all_reach();
}

/**
 * @brief tilt robot body downward (front legs up, back legs down)
 * @param i amount to tilt
 * @warning blocking function
 */
void head_down(int i) {
   set_site(0, KEEP, KEEP, site_now[0][2] + i);
   set_site(1, KEEP, KEEP, site_now[1][2] - i);
   set_site(2, KEEP, KEEP, site_now[2][2] + i);
   set_site(3, KEEP, KEEP, site_now[3][2] - i);
   wait_all_reach();
}

/**
 * @brief perform a dance routine with increasing speed
 * @warning blocking function
 * @param i number of dance cycles to perform
 */
void body_dance(int i) {
   float body_dance_speed = 2;
   sit();
   move_speed = 1;
   set_site(0, x_default, y_default, KEEP);
   set_site(1, x_default, y_default, KEEP);
   set_site(2, x_default, y_default, KEEP);
   set_site(3, x_default, y_default, KEEP);
   wait_all_reach();
   // stand();
   set_site(0, x_default, y_default, z_default - 20);
   set_site(1, x_default, y_default, z_default - 20);
   set_site(2, x_default, y_default, z_default - 20);
   set_site(3, x_default, y_default, z_default - 20);
   wait_all_reach();
   move_speed = body_dance_speed;
   head_up(30);
   for (int j = 0; j < i; j++) {
      if (j > i / 4) move_speed = body_dance_speed * 2;
      if (j > i / 2) move_speed = body_dance_speed * 3;
      set_site(0, KEEP, y_default - 20, KEEP);
      set_site(1, KEEP, y_default + 20, KEEP);
      set_site(2, KEEP, y_default - 20, KEEP);
      set_site(3, KEEP, y_default + 20, KEEP);
      wait_all_reach();
      set_site(0, KEEP, y_default + 20, KEEP);
      set_site(1, KEEP, y_default - 20, KEEP);
      set_site(2, KEEP, y_default + 20, KEEP);
      set_site(3, KEEP, y_default - 20, KEEP);
      wait_all_reach();
   }
   move_speed = body_dance_speed;
   head_down(30);
}

/**
 * @brief servo service routine called by timer interrupt at 50Hz
 * @warning moves the end point to expected site in a straight line
 * @note temp_speed[4][3] should be set before setting expect site to ensure the
 * end point moves in a straight line and to determine move speed
 */
#define E_DELTA 0.01
void servo_service(void) {
   sei();
   static float alpha, beta, gamma;

   for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 3; j++) {
         if (abs(site_now[i][j] - site_expect[i][j]) <
             (abs(temp_speed[i][j]) + E_DELTA))
            site_now[i][j] = site_expect[i][j];
         else
            site_now[i][j] += temp_speed[i][j];
      }

      cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1],
                         site_now[i][2]);
      polar_to_servo(i, alpha, beta, gamma);
   }

   rest_counter++;
}

/**
 * @brief set one of the end points' expected site
 * @warning non-blocking function
 * @param leg leg number (0-3)
 * @param x x coordinate (use KEEP to maintain current value)
 * @param y y coordinate (use KEEP to maintain current value)
 * @param z z coordinate (use KEEP to maintain current value)
 * @note this function will set temp_speed[4][3] at the same time
 */
void set_site(int leg, float x, float y, float z) {
   float length_x = 0, length_y = 0, length_z = 0;

   if (x != KEEP) length_x = x - site_now[leg][0];
   if (y != KEEP) length_y = y - site_now[leg][1];
   if (z != KEEP) length_z = z - site_now[leg][2];

   float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

   temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
   temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
   temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;

   if (x != KEEP) site_expect[leg][0] = x;
   if (y != KEEP) site_expect[leg][1] = y;
   if (z != KEEP) site_expect[leg][2] = z;
}

/**
 * @brief wait for one end point to reach expected site
 * @warning blocking function
 * @param leg leg number (0-3)
 */
void wait_reach(int leg) {
   while (1)
      if (site_now[leg][0] == site_expect[leg][0])
         if (site_now[leg][1] == site_expect[leg][1])
            if (site_now[leg][2] == site_expect[leg][2]) break;
}

/**
 * @brief wait for all end points to reach expected sites
 * @warning blocking function
 */
void wait_all_reach(void) {
   for (int i = 0; i < 4; i++) wait_reach(i);
}

/**
 * @brief transform site from cartesian to polar coordinates
 * @param alpha output angle for first servo
 * @param beta output angle for second servo
 * @param gamma output angle for third servo
 * @param x cartesian x coordinate
 * @param y cartesian y coordinate
 * @param z cartesian z coordinate
 * @note mathematical model 2/2
 */
void cartesian_to_polar(volatile float& alpha, volatile float& beta,
                        volatile float& gamma, volatile float x,
                        volatile float y, volatile float z) {
   // calculate w-z degree
   float v, w;
   w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
   v = w - length_c;
   alpha = atan2(z, v) +
           acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) /
                2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
   beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) /
               2 / length_a / length_b);
   // calculate x-y-z degree
   gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
   // trans degree pi->180
   alpha = alpha / pi * 180;
   beta = beta / pi * 180;
   gamma = gamma / pi * 180;
}

/**
 * @brief transform site from polar coordinates to servo angles
 * @param leg leg number (0-3)
 * @param alpha first servo angle
 * @param beta second servo angle
 * @param gamma third servo angle
 * @note mathematical model mapped to physical implementation
 */
void polar_to_servo(int leg, float alpha, float beta, float gamma) {
   if (leg == 0) {
      alpha = 90 - alpha;
      beta = beta;
      gamma += 90;
   } else if (leg == 1) {
      alpha += 90;
      beta = 180 - beta;
      gamma = 90 - gamma;
   } else if (leg == 2) {
      alpha += 90;
      beta = 180 - beta;
      gamma = 90 - gamma;
   } else if (leg == 3) {
      alpha = 90 - alpha;
      beta = beta;
      gamma += 90;
   }

   servo[leg][0].write(alpha);
   servo[leg][1].write(beta);
   servo[leg][2].write(gamma);
}

/**
 * @brief Calculates and returns the available free SRAM in bytes.
 * Works by measuring the space between the Hardware Stack Pointer (SP)
 * and the end of the Heap (__brkval or __malloc_heap_end).
 * @return int: Number of free bytes.
 */
int freeMemory() {
   extern int __heap_start, *__brkval;
   int v;
   // If __brkval is NULL, the heap hasn't grown yet; use the heap start address
   if ((int)__brkval == 0) {
      return (int)&v - (int)&__heap_start;
   }
   // Otherwise, calculate the space between the current stack (address of v)
   // and the end of the heap (__brkval).
   else {
      return (int)&v - (int)__brkval;
   }
}

/**
 * @brief Inifinite loop function to adjust legs to correct positions.
 * @note Comment out this function if you want to use the spiderbot
 */
void adjust(void) {
   while (1) {
      for (int i = 0; i < 4; i++) {
         for (int j = 0; j < 3; j++) {
            servo[i][j].write(90);
            delay(20);
         }
      }
   }
}