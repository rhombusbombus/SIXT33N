/*
 * classify.ino
 *
 * EE16B Spring 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

#define MIC_INPUT                   P6_0

#define SIZE                        2752
#define SIZE_AFTER_FILTER           172
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*---------------------------*/

// Enveloping and threshold constants
#define SNIPPET_SIZE                  77
#define PRELENGTH                     1
#define THRESHOLD                     0.53123941962519

#define EUCLIDEAN_THRESHOLD         0.040
#define LOUDNESS_THRESHOLD          200

/*---------------------------*/
/*      CODE BLOCK PCA2      */
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

  pinMode(MIC_INPUT, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  re_pointer = 0;
  reset_blinker();
  setTimer();
}

void loop(void) {
  if (re_pointer == SIZE) {
    digitalWrite(RED_LED, LOW);

    // Apply enveloping function and get snippet with speech.
    // Do classification only if loud enough.
    if (envelope(re, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;
      proj3 = 0;

Serial.println("test");
      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*---------------------------*/

      // Project 'result' onto the principal components
      // Hint: 'result' is an array
      // Hint: the principal components are unit norm
      // Hint: do this entire operation in 1 loop by replacing the '...'
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
      // Use the function 'l2_norm3' defined above
      // ith centroid: 'centroids[i]'
      float best_dist = 999999;
      int best_index = -1;
      // YOUR CODE HERE
      for (int i = 0; i < 4; i++) {
          float dist = l2_norm3(proj1, proj2, proj3, centroids[i]);
          if (i == 0 || dist < best_dist) {
              best_dist = dist;
              best_index = i;
          }
      }

      // Compare 'best_dist' against the 'EUCLIDEAN_THRESHOLD' and print the result
      // If 'best_dist' is less than the 'EUCLIDEAN_THRESHOLD', the recording is a word
      // Otherwise, the recording is noise
      // YOUR CODE HERE
      String word_list[4] = {"disappointment", "banana", "happiness", "university"};
      if (best_dist < EUCLIDEAN_THRESHOLD) {
          Serial.println("======");
          Serial.println("The word is: ");
          Serial.println(word_list[best_index]);
          Serial.println("");
          Serial.println("The distance to the nearest centroid is: ");
          Serial.println(best_dist);
          Serial.println("======");
      } else {
        Serial.println("Above EUCLIDEAN_THRESHOLD.");
      }

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    } else {
      Serial.println("Below LOUDNESS_THRESHOLD.");
    }

    delay(2000);
    re_pointer = 0;
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
    for (int i = 0; i < 16; i++) {
      avg += data[i+block*16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block*16] - avg);
    for (int i = 1; i < 16; i++) {
      data[block] += abs(data[i+block*16] - avg);
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
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data_out[i] / total;
  }

  return true;
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void reset_blinker(void) {
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
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

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR(void) {
  if (re_pointer < SIZE) {
    digitalWrite(RED_LED, HIGH);
    re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
    re_pointer += 1;
  }
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(void) {
  // Set the timer based on 25MHz clock
  TA2CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
  TA2CCTL0 = CCIE;
  __bis_SR_register(GIE);
  TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
}
