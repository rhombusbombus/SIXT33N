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
#define SNIPPET_SIZE                  80
#define PRELENGTH                     5
#define THRESHOLD                     0.5

#define EUCLIDEAN_THRESHOLD         0.25
#define LOUDNESS_THRESHOLD          25 //50

/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*---------------------------*/

float pca_vec1[SNIPPET_SIZE] = {0.005070829993426193, 0.007665177689784808, 0.005224311102881574, 0.03290785826036302, 0.006841809270786509, -0.03528166865859187, -0.03296803994439675, -0.040639003554412315, 0.027123411016025258, -0.08208627740355115, 0.07606936172388028, -0.08157341390274209, 0.13429094429893582, -0.10304114229351452, 0.17436048880903793, -0.10073648236014457, 0.20007870961582477, -0.09924519199929428, 0.21896202800278755, -0.07871924129017503, 0.22169403968474596, -0.026534428425107896, 0.22029786916441507, 0.02732812004546268, 0.22793677356325623, 0.08935857593940823, 0.21954931659737573, 0.09900134995316122, 0.1997010040791891, 0.09504607781175456, 0.17495388504925724, 0.10423454024894906, 0.1635852180421121, 0.12349847895528251, 0.14885473833378654, 0.12372369432679353, 0.12160318046267805, 0.11526088949082992, 0.0927377295899816, 0.07379031449369011, 0.0689475631052701, 0.025670778999049845, 0.04004666578166127, 0.021004104544324526, -0.01852255192816694, -0.00545289655714749, -0.02578917381429946, -0.047898496227354745, -0.06262325118236921, -0.07173929515793573, -0.16400088393527004, -0.1393035083833892, -0.165491024006081, -0.1703497295153966, -0.14316812681412575, -0.18996812049970965, -0.14080518276760554, -0.17892769423192753, -0.14203448386831652, -0.15107211464657405, -0.11806124647601073, -0.14264874433485977, -0.10688669295943581, -0.0952571924553403, -0.12256255393252867, -0.06062463468930205, -0.05525901366832768, -0.03473318791272439, -0.0566392084004553, -0.0037267522084604445, -0.03510203165790025, -0.021730228884091197, -0.047020249682087206, -0.023040921993194404, -0.040536830028034536, -0.02961394552700637, -0.05976736901911318, -0.029698398654945456, -0.0706268916457055, -0.034912320549045896};
float pca_vec2[SNIPPET_SIZE] = {0.002304845011192878, 0.016759736327063443, 0.002114681151270531, 0.02287352951075884, -0.03412111998690867, -0.049644999409128476, -0.015628011626298455, -0.11861992609403416, 0.0279013225282225, -0.14729346694801623, 0.11338577612415922, -0.2177123634393561, 0.16601128436308685, -0.24109472091045592, 0.19975543928526354, -0.22208065992375797, 0.2016782895637574, -0.2330463253594844, 0.20191221725697356, -0.2009911406782505, 0.1780894441084874, -0.20965098597108767, 0.12590890459110612, -0.1823973580927697, 0.06543984348744862, -0.21517749688131577, 0.022686166800696687, -0.23047344465878483, -0.018345906621025882, -0.20643498560307366, -0.023446557532293796, -0.15573198824531476, -0.027230448354247077, -0.1428466433396958, -0.03443432053678383, -0.09907113304753742, -0.03194768472584862, -0.0806732889232029, -0.033856456068173196, -0.07264153734990741, -0.03536967215666838, -0.05553291386119374, -0.04495460045202338, -0.05149331439805269, -0.04510559340996842, -0.05161222902546583, -0.021496523150932097, -0.03391653155098019, 0.026767973158826428, -0.014762415331840011, 0.031790269834035316, 0.009937710590717195, 0.03110345261981866, 0.048053930945605255, 0.08408442694977421, 0.08812380728628781, 0.08358825639153546, 0.1105068292558261, 0.0884332150954101, 0.10875401209119533, 0.09651482845703316, 0.11273106470591181, 0.09942344164107422, 0.1164773742696065, 0.10465410282617531, 0.10491625752248945, 0.08818318472034312, 0.09985149845352666, 0.07844924231584732, 0.071826101018437, 0.06381944538269224, 0.07649658610349784, 0.05724366118773525, 0.06575994191702556, 0.06200100274898455, 0.06316131010039314, 0.05193839848761794, 0.05669093771853937, 0.033887891848531464, 0.03684512790989658};
float pca_vec3[SNIPPET_SIZE] = {-0.012480204437894767, 0.0012767268477186788, 0.010030992015952311, 0.024656553515085944, 0.07349107259130205, 0.06343330130227745, 0.32491328643059464, 0.2535675459486041, 0.32223553257317866, 0.1572009310318707, 0.2590147128521847, 0.10308859962037842, 0.1960182654774508, 0.0005758407468383054, 0.08265809239561075, -0.10909817690716776, -0.04123151818903111, -0.22632261103291498, -0.11933336836391453, -0.23746135412679392, -0.15967173229967857, -0.20426915567352574, -0.147859446133792, -0.1542947091258946, -0.09218202397320041, -0.040522726752314255, 0.03369891117889373, 0.07978885875923508, 0.15785397054217984, 0.18062601699647746, 0.14635992296811723, 0.0944123616429151, 0.03714705267067784, 0.02089946817351014, -0.023223037687343972, -0.03425578959343852, -0.08855786316256793, -0.050467253190708834, -0.10610243320327647, -0.08105093375032757, -0.11496909190054508, -0.08478077911130445, -0.08789024872092994, -0.06334602995154759, -0.062469950950311465, -0.006901495091774154, 0.0263377604996001, 0.057845939600007876, 0.07798046441115306, 0.111728525329978, 0.062021986408608315, 0.0945722959390761, 0.06550467286429537, 0.06923457496669624, 0.03621940392722344, 0.03836928882151274, 0.02344186358147208, -0.0056346941307114295, 0.0019328616408848581, -0.03909189968764852, -0.062235659540187185, -0.08202157627863145, -0.08570426728904362, -0.06814747289823683, -0.10697956887672726, -0.11061731866981803, -0.07689658992769992, -0.09173645643976568, -0.0978590875335222, -0.09263225144161877, -0.06339344472921489, -0.06344887830881032, -0.044610643727649335, -0.02813480817717879, 0.007124179045337846, 0.0016783524101532626, 0.052666344716610254, 0.029884419027386377, 0.04844602809610865, 0.03994957341950382};
float projected_mean_vec[3] = {0.011253230417470055, -0.013467572605076594, 0.019530692660852025};
float centroid1[3] = {-0.013956053760198695, 0.0020228800676171696, 0.03925087249510758};
float centroid2[3] = {0.01230464508205999, -0.0018989384777449065, -0.0192416001646511};
float centroid3[3] = {-0.02578235322717501, 0.013781989733860312, -0.017623424334588456};
float centroid4[3] = {0.02743376190531371, -0.013905931323732568, -0.002385847995867988};
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
      proj1 -= projected_mean_vec[1];
      proj2 -= projected_mean_vec[2];
      proj3 -= projected_mean_vec[3];

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
      String word_list[4] = {"disappointment", "banana", "happiness", "pain"}
      if (best_dist < EUCLIDEAN_THRESHOLD) {
          Serial.println("======");
          Serial.println("The word is: ");
          Serial.println(word_list[best_index]);
          Serial.println("");
          Serial.println("The distance to the nearest centroid is: ")
          Serial.println(best_dist);
          Serial.println("======");
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
