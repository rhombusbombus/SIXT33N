/*
 * open_loop.ino
 *
 * EE16B Fall 2016
 * John Maidens, Emily Naviasky & Nathaniel Mailoa
 *
 * EE16B Fall 2017
 * Andrew Blatner
 *
 */

#define LEFT_MOTOR                  P1_5
#define RIGHT_MOTOR                 P2_0

#define RUN_TIME                    (15*1000)
#define PUSH_START                  PUSH1

unsigned long end_time = 0;

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*---------------------------*/

float theta_left = 0.2686;
float theta_right = 0.2564;
float beta_left = -27.54;
float beta_right = -30.28;
float v_star = 75.0;

// PWM inputs to jolt the car straight
int left_jolt = 200;
int right_jolt = 195;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*---------------------------*/

float driveStraight_left(float v_star) {
  float x = v_star + beta_left;
  return x / theta_left;
}

float driveStraight_right(float v_star) {
  float x = v_star + beta_right;
  return x / theta_right;
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


void setup(void) {
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(PUSH_START, INPUT_PULLUP);

  write_pwm(0, 0); // Turn off motors
  while (digitalRead(PUSH_START)) {
    
  }
  reset_blinker(); // Blink lights to indicate car is running
  write_pwm(left_jolt, right_jolt); // Jolt motors for 200ms
  delay(200);

  /*---------------------------*/
  /*      CODE BLOCK CON0      */
  /*---------------------------*/

  // Attempt to drive straight using open loop control
  // Compute the PWM input required for each wheel based on v_star
  int left_cur_pwm = driveStraight_left(v_star);
  int right_cur_pwm = driveStraight_right(v_star);
  write_pwm(left_cur_pwm, right_cur_pwm);

  /*---------------------------*/
  /*---------------------------*/
  /*---------------------------*/

  end_time = millis() + RUN_TIME;
  
}

void loop(void) {
  if (end_time <= millis()) {
    write_pwm(0, 0);
  }
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void reset_blinker(void) {
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}
