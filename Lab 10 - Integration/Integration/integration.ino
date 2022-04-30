/*
 * integration.ino
 * Final sketch for SIXT33N Speech version
 *
 * EE16B Fall 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

/********************************************************/
/********************************************************/
/***                                                  ***/
/*** Constants and global variables from turning.ino. ***/
/***                                                  ***/
/********************************************************/
/********************************************************/

// Change pins here if you did not use the default pins!
#define LEFT_MOTOR                  P1_5
#define LEFT_ENCODER                P6_3
#define RIGHT_MOTOR                 P2_0
#define RIGHT_ENCODER               P6_2

#define SAMPLING_INTERVAL           100
int sample_lens[4] = {0};

// Operation modes
#define MODE_LISTEN                 0
#define MODE_DRIVE                  1
int timer_mode = MODE_LISTEN;

#define DRIVE_FAR                   0
#define DRIVE_LEFT                  1
#define DRIVE_CLOSE                 2
#define DRIVE_RIGHT                 3

#define JOLT_STEPS                  2

boolean loop_mode = MODE_DRIVE;
int drive_mode = 0;

int step_num = 0;
volatile boolean do_loop = 0; // timer signal to increment timestep

typedef struct encoder {
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {LEFT_ENCODER, 0, LOW, 0};
encoder_t right_encoder = {RIGHT_ENCODER, 0, LOW, 0};

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*      From turning.ino     */
/*---------------------------*/

float theta_left = 0.2686;
float theta_right = 0.2564;
float beta_left = -27.54;
float beta_right = -30.28;
float v_star = 75.0;

// PWM inputs to jolt the car
// PWM inputs to jolt the car straight
int left_jolt = 200;
int right_jolt = 195;

// Feedback control gains
float f_left = 0.9;
float f_right = 0.1;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*      From turning.ino     */
/*---------------------------*/

float driveStraight_left(float delta) {
  return (v_star - (f_left * delta) + beta_left) / theta_left;
}

float driveStraight_right(float delta) {
  return (v_star + (f_right * delta) + beta_right) / theta_right;
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*      From turning.ino     */
/*---------------------------*/

float delta_ss = 15;

/*---------------------------*/
/*      CODE BLOCK CON4      */
/*      From turning.ino     */
/*---------------------------*/

#define CAR_WIDTH                   15.0 // in cm
#define TURN_RADIUS                 120 // in cm - 6 feet diameter
#define TURN_RADIUS2                 100 // in cm - 4 feet diameter

int run_times[4] = {6000, 3000, 3000, 2500}; // {DRIVE_FAR, DRIVE_LEFT, DRIVE_CLOSE, DRIVE_RIGHT}

float delta_reference(int i) {
  // YOUR CODE HERE
  // Remember to divide the v* value you use in this function by 5 because of sampling interval differences!
  if (drive_mode == DRIVE_RIGHT) { // Return a NEGATIVE expression
    return -((v_star / 5) * CAR_WIDTH * i) / TURN_RADIUS2;
  }
  else if (drive_mode == DRIVE_LEFT) { // Return a POSITIVE expression
    return ((v_star / 5) * CAR_WIDTH * i) / TURN_RADIUS;
  }
  else { // DRIVE_STRAIGHT
    return 0;
  }
}

/*---------------------------*/
/*      CODE BLOCK CON5      */
/*      From turning.ino     */
/*---------------------------*/
#define INFINITY                    (3.4e+38)
#define STRAIGHT_RADIUS             500

float straight_correction(int i) {
  // return ((v_star / 5) * CAR_WIDTH * i) / STRAIGHT_RADIUS;
  return 0;
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

/*********************************************************/
/*********************************************************/
/***                                                   ***/
/*** Constants and glboal variables from classify.ino. ***/
/***                                                   ***/
/*********************************************************/
/*********************************************************/

// Change pin here if you did not use the default pin!
#define MIC_INPUT                   P6_0

#define SIZE                        2752
#define SIZE_AFTER_FILTER           172
#define ADC_TIMER_MS                0.35
/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*     From classify.ino     */
/*---------------------------*/

// Enveloping and threshold constants
#define SNIPPET_SIZE                  77
#define PRELENGTH                     1
#define THRESHOLD                     0.53123941962519

#define EUCLIDEAN_THRESHOLD         0.035 // 0.04
#define LOUDNESS_THRESHOLD          200

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*     From classify.ino     */
/*---------------------------*/

float pca_vec1[77] = {-0.09841321002941293, -0.18137825207412572, -0.27697910421542804, -0.20903141704760897, -0.20700017404850724, -0.17636913634936308, -0.13374719949039054, -0.13361461739422856, -0.03689164751571952, -0.05997374431872137, 0.07663864471647334, 0.027640882602897543, 0.16627473673244128, 0.07658984632177536, 0.1962313769180582, 0.11701691279917681, 0.2047981430185291, 0.13078761623184967, 0.18539641770656348, 0.15717357777562205, 0.14832905504615546, 0.11792379325308515, 0.0960005979473424, 0.03937492602056662, 0.014601930995382442, -0.012740413666306615, 0.02037936285266876, 0.0351289556318516, 0.12382661650904432, 0.0876839964265643, 0.15435941952835577, 0.11133189930487306, 0.1659817150376697, 0.12493277102732879, 0.15096023834097752, 0.12135471672804651, 0.1351365101910378, 0.08191044558548395, 0.08609016404092842, 0.029269534421352623, 0.007039320893037491, -0.015375464555226344, -0.06784285415079187, -0.09654994764119065, -0.1268665423169445, -0.13635617612208983, -0.1826113874999062, -0.1570444883489027, -0.17975130519003338, -0.14747867855478883, -0.1321576360009796, -0.12034346352335222, -0.11223162499855496, -0.07963423881858862, -0.0783194211936415, -0.04168407397744458, -0.044973317845644976, 0.02456513155731662, 0.0028446112793323554, 0.03695697786942888, 0.026234424005543114, 0.07624877239831414, 0.06858720230752911, 0.0639846539802451, 0.0692856401901426, 0.05857600378685353, 0.06245198941692789, 0.013471947091293613, 0.0361717913017326, -0.011549603422408086, -0.013667543376097251, -0.0580376204033769, -0.04870499017614885, -0.08106070512623492, -0.07794772620389957, -0.09672948357862082, -0.09648606061511916};
float pca_vec2[77] = {-0.08412947809335268, 0.04362560707655916, -0.1352251510867628, -0.1184417208796138, -0.14705704433630723, -0.012215456514503453, -0.09104768397685278, -0.0256455470407565, -0.1049986484981558, 0.04356928827126124, -0.041551603072209536, 0.1356441753161388, 0.024930328726797086, 0.18857009388068793, 0.057520368248498446, 0.182887666906454, 0.05665773426541164, 0.1324999395879861, 0.039213537189845604, 0.022409610675711424, -0.059793949479523885, -0.1252708876832781, -0.2040548837397226, -0.2626082864620183, -0.3234141079098487, -0.3014751578023235, -0.30757221652155187, -0.20730597280735918, -0.19991286175491121, -0.13778199429948404, -0.11639211016048678, -0.07477164130874035, -0.0602311779552977, -0.024584253135639614, -0.019316672700499124, 0.0028292396048528344, -0.004851231204008967, 0.06275627065981403, 0.027401617944717762, 0.06110256289555134, 0.06360737074960811, 0.021203679923201997, 0.023652469987258878, 0.01974505632225778, 0.009608484518583827, -0.01056764146609674, 0.06040772601668897, 0.03862489315567312, 0.08216322769093912, 0.08541160369245862, 0.09916627950469062, 0.1314541685361077, 0.11347954814789446, 0.13744930981359046, 0.11382506360158855, 0.1517628817879608, 0.12901573726233256, 0.13586139699662325, 0.12446803547532592, 0.12236793029408924, 0.13798844097749707, 0.10281219655406762, 0.07439598752560606, 0.06978981836413084, 0.08034314815646815, 0.042759420704808045, 0.04158507350132083, 0.04550475920966601, 0.03268263941801905, 0.010158259436026244, -0.017640925726879655, 0.0050972828949430865, -0.02603410222263178, -0.027347357961449218, -0.025975982253353257, -0.043676757848860145, -0.04711742556723618};
float pca_vec3[77] = {0.011758689580530434, -0.0241129093032295, -0.07115732224470762, 0.05342481483898552, 0.005618029130980223, -0.019247869749535185, 0.04521411127962094, 0.001763172550280856, 0.11516861860942648, -0.061915916241757796, 0.1255726337112671, -0.06359860290153735, 0.03670604918317695, -0.1500748307528435, 0.033354742979965005, -0.13782502075957442, -0.03360962971985005, -0.10201592141526919, -0.026576695764226325, -0.0677988290180883, -0.059490035886080955, -0.05107346026591657, -0.04319653503284686, -0.009798017775877986, 0.05980717976830494, 0.085917222219533, 0.13520543274429445, 0.07488739845679686, 0.1239866216908751, 0.03920354333991262, 0.08108922666884281, 0.0030940177444234167, 0.03177739718394836, -0.038412753885769424, -0.0356523883959116, -0.09103888786526647, -0.09169856503481141, -0.18789173629261385, -0.1573364805492469, -0.2240488524733753, -0.23051211793068732, -0.243654963860078, -0.21357534880830856, -0.22228526721688008, -0.19536852007934255, -0.08123678234789769, -0.12023637363527317, 0.013880769695354041, -0.013441887158816765, 0.10962040539634318, 0.048246041983618135, 0.1539837447124756, 0.1139691913185816, 0.1900512230209824, 0.13581873199482652, 0.25076930780899037, 0.10272663647267119, 0.2742272058418539, 0.14412999462058793, 0.22452661736735174, 0.12507698774533485, 0.1691769407534364, 0.12542154623389587, 0.10002573329159833, 0.07872031376833177, 0.046658626896417074, 0.030797691010589736, -0.021090725906416845, -0.019090653331774142, -0.0494582341334323, -0.02720020845919296, -0.07892531362023182, -0.041086687242825326, -0.05134296243651957, -0.04093983032687512, -0.06749097693024932, -0.03686849686126717};
float projected_mean_vec[3] = {-0.008823108711948746, -0.006501006036886258, 0.003782207660700962};
float centroid1[3] = {-0.03885152264743576, -0.031110426242906736, 0.005908368016160271};
float centroid2[3] = {0.020827688538304207, -0.003945786432053436, -0.010529480254398997};
float centroid3[3] = {-0.007080099849169957, 0.0280019195876919, -0.012201678032010005};
float centroid4[3] = {0.025103933958301457, 0.00705429308726827, 0.016822790270248723};



float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;
float proj3 = 0;

// Data array and index pointer
int re[SIZE] = {0};
volatile int re_pointer = 0;

/*---------------------------------------------------*/
/*---------------------------------------------------*/
/*---------------------------------------------------*/

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  Serial.begin(38400);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(MIC_INPUT, INPUT);

  for (int j = 0; j < 4; j++) {
    sample_lens[j] = run_times[j] / SAMPLING_INTERVAL;
  }

  write_pwm(0, 0);
  delay(2000); // Wait 2 seconds to put down car
  reset_blinker();
  start_listen_mode();
}

void loop(void) {
  check_encoders();
  if (timer_mode == MODE_LISTEN && re_pointer == SIZE){
    // Stop motor
    write_pwm(0, 0);
    digitalWrite(RED_LED, LOW);

    // if enveloped data is above some preset value
    if (envelope(re, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;
      proj3 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*     From classify.ino     */
      /*     with more changes     */
      /*---------------------------*/

      // Project 'result' onto the principal components
      // YOUR CODE HERE
      for (int i = 0; i < SNIPPET_SIZE; i++) {
          proj1 += result[i]*pca_vec1[i];
          proj2 += result[i]*pca_vec2[i];
          proj3 += result[i]*pca_vec3[i];
      }

      // Demean the projection
      proj1 -= projected_mean_vec[0];
      proj2 -= projected_mean_vec[1];
      proj3 -= projected_mean_vec[2];


      // Classification
      // Use the function l2_norm3 defined above
      // jth centroid: centroids[j]
      // YOUR CODE HERE
      float best_dist = 999999;
      int best_index = -1;
      // YOUR CODE HERE
      for (int i = 0; i < 4; i++) {
          float dist = l2_norm3(proj1, proj2, proj3, centroids[i]);
          if (dist < best_dist) {
              best_dist = dist;
              best_index = i;
          }
      }


      // Check against EUCLIDEAN_THRESHOLD and execute identified command
      // YOUR CODE HERE  
      if (best_dist < EUCLIDEAN_THRESHOLD) {
        drive_mode = best_index; // from 0-3, inclusive
        Serial.println(best_index);
        if (best_index == 0) {
          Serial.println("disappointment");
        }
        if (best_index == 1) {
          Serial.println("banana");
        }
        if (best_index == 2) {
          Serial.println("happiness");
        }
        if (best_index == 3) {
          Serial.println("university");
        }
        start_drive_mode();
      } else {
        Serial.println("Above EUCLIDEAN_THRESHOLD.");
        Serial.println(best_dist);
      }

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    } else {
      Serial.println("Below LOUDNESS_THRESHOLD.");
    }

    delay(2000);
    re_pointer = 0; // start recording from beginning if we don't start driving
  }

  else if (loop_mode == MODE_DRIVE && do_loop) {
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
    }
    else {

      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = left_encoder.pos;
      int right_position = right_encoder.pos;

      /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/

      float delta = left_position - right_position + delta_ss;
      delta = delta + delta_reference(step_num) + straight_correction(step_num);

      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = driveStraight_left(delta);
      int right_cur_pwm = driveStraight_right(delta);
      write_pwm(left_cur_pwm, right_cur_pwm);

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    // Counter for how many times loop is executed since entering DRIVE MODE
    step_num++;

    if (step_num == sample_lens[drive_mode]) {
      // Completely stop and go back to listen MODE after 3 seconds
      start_listen_mode();
    }

    do_loop = 0;
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  int32_t avg = 0;
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    avg = 0;
    for (int j = 0; j < 16; j++) {
      avg += data[j+block*16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block*16] - avg);
    for (int j = 1; j < 16; j++) {
      data[block] += abs(data[j+block*16] - avg);
    }
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int j = 0; j < SNIPPET_SIZE; j++) {
    data_out[j] = data[block-PRELENGTH+j];
    total += data_out[j];
  }

  // Normalize data_out
  for (int j = 0; j < SNIPPET_SIZE; j++) {
    data_out[j] = data_out[j] / total;
  }

  return true;
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

void start_listen_mode(void) {
  re_pointer = 0;
  write_pwm(0, 0);
  delay(3000); // 3 seconds buffer for mic cap settling
  timer_mode = MODE_LISTEN;
  setTimer(MODE_LISTEN);
}

void start_drive_mode(void) {
  timer_mode = MODE_DRIVE;
  step_num = 0;
  left_encoder.pos = 0;
  right_encoder.pos = 0;
  setTimer(MODE_DRIVE);
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

#define AVG_DECAY_RATE              0.3
#define LOW_THRESH                  ((int) (0.1*4096))
#define HIGH_THRESH                 ((int) (0.4*4096))

void check_encoder(encoder_t* enc) {
  int new_val = analogRead(enc->pin);
  enc->avg = (int) (AVG_DECAY_RATE*enc->avg + (1 - AVG_DECAY_RATE)*new_val);
  if ((enc->level == LOW && HIGH_THRESH < enc->avg) ||
      (enc->level == HIGH && enc->avg < LOW_THRESH)) {
    enc->pos++;
    enc->level = !enc->level;
  }
}

void check_encoders(void) {
  check_encoder(&left_encoder);
  check_encoder(&right_encoder);
}

// Set timer for timestep; use B0 to free up all other PWM ports
void setTimer(boolean mode) {
  if (mode == MODE_LISTEN) {
    // Set the timer based on 25MHz clock
    TB0CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
    TB0CCTL0 = CCIE;
    __bis_SR_register(GIE);
    TB0CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
  }
  else if (mode == MODE_DRIVE) {
    TB0CCR0 = (unsigned int) (32.768*SAMPLING_INTERVAL); // set the timer based on 32kHz clock
    TB0CCTL0 = CCIE; // enable interrupts for Timer B
    __bis_SR_register(GIE);
    TB0CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
  }
  timer_mode = mode;
}

// ISR for timestep
#pragma vector=TIMER0_B0_VECTOR    // Timer B ISR
__interrupt void Timer0_B0_ISR(void) {
  if (timer_mode == MODE_LISTEN) {
    if (re_pointer < SIZE) {
      digitalWrite(RED_LED, HIGH);
      re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
      re_pointer += 1;
    }
  }
  else if (timer_mode == MODE_DRIVE) {
    do_loop = 1;
  }
}
