#include "texton_settings.h"
#include "texton_helpers.h"
#include "trexton_regression.h"
#include "image_conversions.h"
#include "readcsv.h"

#include <stdio.h>

#include "modules/particle_filter/particle_filter.h"

#include "lib/v4l/v4l2.h"
#include "lib/vision/image.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"
#include "udp_socket.h"

#include "subsystems/gps.h"
#include "subsystems/abi.h"

/* #include "opticflow_module.h" */
#include "opticflow/opticflow_calculator.h"

/* #include "floatfann.h" */

bool gps_available;   ///< Is set to TRUE when a new REMOTE_GPS packet is received and parsed

/* Histogram paths */
static char histogram_filename[] = "mat_train_hists.csv";
/* static char histogram_filename_testset[] = "mat_test_hists_str8.csv"; */
static char position_filename[] =  "board_train_pos.csv";
/* static char test_position_filename[] =  "predictions_cross.csv"; */
static struct measurement all_positions[NUM_HISTOGRAMS];
/* static struct measurement all_test_positions[NUM_TEST_HISTOGRAMS]; */

static double regression_histograms[NUM_HISTOGRAMS][SIZE_HIST];
static double regression_histograms_color[NUM_HISTOGRAMS][SIZE_HIST];
/* static int histograms_testset[NUM_TEST_HISTOGRAMS][SIZE_HIST]; */

static int current_test_histogram = 0;
static int use_variance = 0;

/* Create  particles */
struct particle particles[N];

/* The main opticflow variables */
struct opticflow_t opticflow;                      ///< Opticflow calculations
static struct opticflow_result_t opticflow_result; ///< The opticflow result
static struct opticflow_state_t opticflow_state;   ///< State of the drone to communicate with the opticflow
/* static pthread_mutex_t opticflow_mutex;            ///< Mutex lock fo thread safety */
static bool opticflow_got_result; ///< When we have an optical flow calculation

static struct UdpSocket video_sock; /* UDP socket for sending RTP video */

static int image_num = 0;

/* The trexton camera V4L2 device */
static struct v4l2_device *trexton_dev;

/* File that contains the filters */
/* IMPORTANT: needs three decimal places !!! */
static char *texton_filename = "textons.csv";

/* Array with the textons */
double textons[NUM_TEXTONS * CHANNELS][TOTAL_PATCH_SIZE];

#define USE_FLOW false

void trexton_init()
{

  /* Initialize GPS settings  */
  init_positions();

  /* Get textons -- that is the clustering centers */
  read_textons_from_csv(textons, texton_filename);

  /* Remove predictions file */
  remove("particle_filter_preds.csv");
  remove("edgeflow_diff.csv");

  // Set the opticflow state to 0
  opticflow_state.phi = 0;
  opticflow_state.theta = 0;
  opticflow_state.agl = 0;

  // Initialize the opticflow calculation
  opticflow_calc_init(&opticflow, TREXTON_DEVICE_SIZE);

  opticflow_got_result = FALSE;

#if PREDICT
  /* Read histograms */

  #if USE_COLOR
    read_color_histograms_from_csv(regression_histograms_color, histogram_filename, SIZE_HIST);
  #else
    read_histograms_from_csv(regression_histograms, histogram_filename, SIZE_HIST);
  #endif

  /* Print color histograms */
  /* int i, j; */
  /* for (i = 0; i < NUM_HISTOGRAMS; i++) { */
  /*   for (j = 0; j < NUM_COLOR_BINS * COLOR_CHANNELS; j++) { */
  /*     printf("%f ", regression_histograms_color[i][j]); */
  /*   } */
  /*   printf("\n"); */
  /* } */


  /* Read x, y, position from SIFT */
  read_positions_from_csv(all_positions, position_filename);

#endif

#if EVALUATE
  /* read_test_histograms_from_csv(histograms_testset, histogram_filename_testset); */

  /* Write header for predictions file*/
  remove("predictions.csv");
  FILE *fp_predictions;
  fp_predictions = fopen("predictions.csv", "a");
  fprintf(fp_predictions, "id,x,y,dist\n");
  fclose(fp_predictions);

#endif

  /* Initialize particles*/
  init_particles(particles);

  /* Debugging read poitions from csv */
  /* int i; */
  /* for (i = 0; i < 10; i++) { */
  /*   printf("is is %d pos x is %f, pos y is %f\n", i, all_positions[i].x, all_positions[i].y); */
  /* } */


#if USE_WEBCAM
  /* Initialize the video device */
  trexton_dev = v4l2_init(STRINGIFY(TREXTON_DEVICE),
                          TREXTON_DEVICE_SIZE,
                          TREXTON_DEVICE_BUFFERS,
                          V4L2_PIX_FMT_UYVY);
  if (trexton_dev == NULL) {
    printf("[treXton_module] Could not initialize the video device\n");
  }

  // Start the streaming on the V4L2 device
  if (!v4l2_start_capture(trexton_dev)) {
    printf("[treXton_module] Could not start capture of the camera\n");
  }


  // Open udp socket
  udp_socket_create(&video_sock,
                    STRINGIFY(VIEWVIDEO_HOST),
                    VIEWVIDEO_PORT_OUT,
                    -1,
                    VIEWVIDEO_BROADCAST);
#endif

}

/* Main function for the texton framework. It s called with a
   frequency of 30 Hz*/
void trexton_periodic()
{

  #if MEASURE_TIME
    /* clock_t start = clock(); */;
    static struct timeval t0, t1, tot;
    long elapsed;
    gettimeofday(&tot, 0);
  #endif

  /* Calculate the texton histogram -- that is the frequency of
     characteristic image patches -- for this image */

  struct image_t img;

  #if USE_WEBCAM
    /* Get the image from the camera */
    v4l2_image_get(trexton_dev, &img);

    #if USE_CONVERSIONS

      /* Create RGB image */
      struct image_t rgb_img, opp_img, std_img;
      image_create(&rgb_img, 320, 240, IMAGE_RGB);
      image_create(&opp_img, 320, 240, IMAGE_OPPONENT);
      image_create(&std_img, 320, 240, IMAGE_STD);
      YUV422toRGB(&img, &rgb_img);
      double means[8];
      RGBtoOpponent(&rgb_img, &opp_img, means);
      image_grayscale_standardize(&opp_img, &std_img, means);
      image_grayscale_standardize(&img, &std_img, means);
      printf("Means are %f, %f, %f %f\n", means[0], means[1], means[2], means[3]);
      
      uint8_t *rgb_buf = (uint8_t *)rgb_img.buf;
      uint8_t *opp_buf = (uint8_t *)opp_img.buf;
      double *std_buf = (double *)std_img.buf;
      
      printf("RGB: %d %d\n", rgb_buf[0], rgb_buf[1]);
      printf("Opponent: %d %d %d %d\n", opp_buf[0], opp_buf[1], opp_buf[2], opp_buf[3]);
      printf("STD: %f %f %f %f\n", std_buf[0], std_buf[1], std_buf[2], std_buf[3]);
    #endif
  #endif

#if MEASURE_TIME
  gettimeofday(&t1, 0);
  elapsed = (t1.tv_sec - t0.tv_sec) * 1000000 + t1.tv_usec - t0.tv_usec;
  printf("Elapsed first part: %ld ms\n", elapsed / 1000);
  gettimeofday(&t0, 0);
#endif


  double texton_histogram[NUM_TEXTONS*NUM_TEXTONS] = {0.0};
  get_texton_histogram(&img, texton_histogram, textons);

  #if USE_COLOR
    double color_hist[COLOR_CHANNELS*NUM_COLOR_BINS] = {0.0};
    get_color_histogram(&img, color_hist, NUM_COLOR_BINS);
  #endif


#if MEASURE_TIME
  gettimeofday(&t1, 0);
  elapsed = (t1.tv_sec - t0.tv_sec) * 1000000 + t1.tv_usec - t0.tv_usec;
  printf("Elapsed get histogram: %ld ms\n", elapsed / 1000);
  gettimeofday(&t0, 0);
#endif


#if SEND_VIDEO
  /* Send JPG image over RTP/UDP */

  // Create a new JPEG image
  struct image_t img_jpeg;
  image_create(&img_jpeg,
               trexton_dev->w,
               trexton_dev->h,
               IMAGE_JPEG);

  jpeg_encode_image(&img, &img_jpeg, 70, FALSE);
  rtp_frame_send(&video_sock, /* UDP device */
                 &img_jpeg,
                 0, /* Format 422 */
                 70, /* Jpeg-Quality */
                 0,  /* DRI Header */
                 0); /* 90kHz time increment */
#endif

#if MEASURE_TIME
  gettimeofday(&t1, 0);
  elapsed = (t1.tv_sec - t0.tv_sec) * 1000000 + t1.tv_usec - t0.tv_usec;
  printf("Elapsed second part: %ld ms\n", elapsed / 1000);
  gettimeofday(&t0, 0);
#endif


#if SAVE_HISTOGRAM
  save_histogram(texton_histogram, HISTOGRAM_PATH);
#elif PREDICT

#if MEASURE_TIME
  gettimeofday(&t1, 0);
  elapsed = (t1.tv_sec - t0.tv_sec) * 1000000 + t1.tv_usec - t0.tv_usec;
  printf("Elapsed after get histogram: %ld ms\n", elapsed / 1000);
  gettimeofday(&t0, 0);
#endif


  #if USE_COLOR
  int i;
  printf("COLOR\n");
  for (i = 0; i < COLOR_CHANNELS * NUM_COLOR_BINS; i++) {
    printf("%f ", color_hist[i]);
  }
  printf("\n");
  #else
  int i;
  printf("TEXTON\n");
  for (i = 0; i < SIZE_HIST; i++) {
    printf("%f ", texton_histogram[i]);
  }
  printf("\n");
  #endif

  /* TreXton prediction */
  struct measurement pos;
  /* For textons */
  /* pos = predict_position(texton_histogram, NUM_TEXTONS * CHANNELS); */

  /* For colors */
  #if USE_COLOR
  pos = predict_position(color_hist, COLOR_CHANNELS * NUM_COLOR_BINS);
  #else
  /* For textons */
  pos = predict_position(texton_histogram, NUM_TEXTONS * CHANNELS);
  #endif

 
  /* For colors */
  /* pos = predict_fann(texton_histogram, NUM_COLOR_BINS * COLOR_CHANNELS); */
  /* pos = linear_regression_prediction(texton_histogram); */
  printf("\nPOSITION IS x:%f y:%f\n", pos.x, pos.y);
  fflush(stdout);

#if MEASURE_TIME
  gettimeofday(&t1, 0);
  elapsed = (t1.tv_sec - t0.tv_sec) * 1000000 + t1.tv_usec - t0.tv_usec;
  printf("Elapsed predict position: %ld ms\n", elapsed / 1000);
  gettimeofday(&t0, 0);
#endif
  //pos = all_test_positions[current_test_histogram];
  /* save_image(&img, "afterpredict.csv"); */

  /* Optical flow prediction */
  /* TODO */

  /* Particle filter update */
  struct measurement flow;

#if USE_FLOW
  // Copy the state
  pthread_mutex_lock(&opticflow_mutex);
  struct opticflow_state_t temp_state;
  memcpy(&temp_state, &opticflow_state, sizeof(struct opticflow_state_t));
  pthread_mutex_unlock(&opticflow_mutex);
  // Do the optical flow calculation
  struct opticflow_result_t temp_result;

  /* edgeflow_calc_frame(&opticflow, &temp_state, &img, &temp_result); */
  opticflow_calc_frame(&opticflow, &temp_state, &img, &temp_result);
  printf("\n opticflow result: x:%d y:%d\n", temp_result.flow_x, temp_result.flow_y);
// Copy the result if finished
  pthread_mutex_lock(&opticflow_mutex);
  memcpy(&opticflow_result, &temp_result, sizeof(struct opticflow_result_t));
  opticflow_got_result = TRUE;
  pthread_mutex_unlock(&opticflow_mutex);

  /* if (image_num == 10) { */
  /*   save_image(&std_img, "mainpic.csv");  */
  /* } */

  /* Mind the change of x, y ! */
  /* TODO: use subpixel factor instead of 1000 */
  flow.y =  3.5 * ((double) opticflow_result.flow_x) / 1000.0;
  flow.x =  3.5 * ((double) opticflow_result.flow_y) / 1000.0;

  printf("flow is %f", flow.x);
  particle_filter(particles, &pos, &flow, use_variance, 1);
  opticflow_got_result = FALSE;
#else
  particle_filter(particles, &pos, &flow, use_variance, 0);
#endif

#if MEASURE_TIME
  gettimeofday(&t1, 0);
  elapsed = (t1.tv_sec - t0.tv_sec) * 1000000 + t1.tv_usec - t0.tv_usec;
  printf("Elapsed flow part: %ld ms\n", elapsed / 1000);
  gettimeofday(&t0, 0);
#endif

  struct particle p_forward = weighted_average(particles, N);
  /* printf("\nRaw: %f,%f\n", pos.x, pos.y); */
  printf("Particle filter: %f,%f\n", p_forward.x, p_forward.y);


  FILE *fp_predictions;
  FILE *fp_particle_filter;
  FILE *fp_edge;
  fp_predictions = fopen("predictions.csv", "a");
  fp_particle_filter = fopen("particle_filter_preds.csv", "a");
  fp_edge = fopen("edgeflow_diff.csv", "a");
  fprintf(fp_edge, "%f,%f\n", flow.x, flow.y);
  fprintf(fp_particle_filter, "%f,%f\n", p_forward.x, p_forward.y);
  //fprintf(fp_predictions, "%d,%f,%f,%f\n", current_test_histogram, pos.x, pos.y, pos.dist);
  fclose(fp_predictions);
  fclose(fp_particle_filter);
  fclose(fp_edge);
#endif

  send_pos_to_ground_station((int) p_forward.x, (int) p_forward.y);
  current_test_histogram++;

#if !EVALUATE

#if USE_CONVERSIONS
  /* Free the image */
  image_free(&rgb_img);
  image_free(&opp_img);
  image_free(&std_img);
#endif
#if USE_WEBCAM
  v4l2_image_free(trexton_dev, &img);
#else
  image_free(&img);
  image_free(&rgb_img);
#endif

#endif

#if SEND_VIDEO
  /* Free RTP stream image */
  image_free(&img_jpeg);
#endif

  image_num = image_num + 1;

#if MEASURE_TIME
  /* clock_t end = clock(); */
  /* float seconds = (float) (end - start) / CLOCKS_PER_SEC; */
  /* printf("%.10f\n", seconds); */
  gettimeofday(&t1, 0);
  elapsed = (t1.tv_sec - tot.tv_sec) * 1000000 + t1.tv_usec - tot.tv_usec;

  printf("TOTAL ELAPSED (entire function) %ld ms\n", elapsed / 1000);
#endif


}

/**
 * Predict the x, y position of the UAV using the texton histogram.
 *
 * @param texton_hist The texton histogram
 *
 * @return The x, y, position of the MAV, computed by means of the input histogram
 */
struct measurement predict_position(double hist[], int hist_size)
{

  int h = 0; /* Histogram iterator variable */

  struct measurement measurements[NUM_HISTOGRAMS];
  double dist;

  /* Compare current texton histogram to all saved histograms for
     a certain class */
  for (h = 0; h < NUM_HISTOGRAMS; h++) {
    /* dist = euclidean_dist_int(hist, regression_histograms[h], hist_size); */
#if USE_COLOR
    dist = chi_square_dist_double(hist, regression_histograms_color[h], hist_size);
#else
    dist = euclidean_dist(hist, regression_histograms[h], hist_size);
#endif

    /* printf("dist is %d %f\n", h, dist); */
    /* printf("all pos is %f\n", all_positions[h].x); */

    struct measurement z;
    z.x = all_positions[h].x;
    z.y = all_positions[h].y;
    z.hist_num = h;
    z.dist = dist;
    measurements[h] = z;

    /* printf("H is %d, dist is %f\n", h, dist); */

  }

  /* Sort distances */
  qsort(measurements, sizeof(measurements) / sizeof(*measurements), sizeof(*measurements), measurement_comp);

  /* Return average over first positions for accurate regression: */

  int k = 3, l;
  struct  measurement mean_pos;
  mean_pos.x = 0;
  mean_pos.y = 0;
  mean_pos.dist = 0;
  for (l = 0; l < k; l++) {
    printf("\nmeasurement: x: %f, y: %f dist: %f num_histogram: %d\n", measurements[l].x, measurements[l].y,
           measurements[l].dist, measurements[l].hist_num);
    printf("\n\nnum_histogram: %d\n\n", measurements[l].hist_num);
    fflush(stdout);
    mean_pos.x += measurements[l].x / k;
    mean_pos.y += measurements[l].y / k;
    mean_pos.dist += measurements[l].dist / k;
  }

  return mean_pos;
}


/** Parse the REMOTE_GPS datalink packet */
void send_pos_to_ground_station(int x, int y)
{
  gps.fix = GPS_FIX_3D;
  gps_available = TRUE;

  // Dummy for changing coordinates

  gps.ecef_pos.x = x;
  gps.ecef_pos.y = y;

  // publish new GPS data
  uint32_t now_ts = get_sys_time_usec();
  gps.last_msg_ticks = sys_time.nb_sec_rem;
  gps.last_msg_time = sys_time.nb_sec;
  if (gps.fix == GPS_FIX_3D) {
    gps.last_3dfix_ticks = sys_time.nb_sec_rem;
    gps.last_3dfix_time = sys_time.nb_sec;
  }
  AbiSendMsgGPS(GPS_DATALINK_ID, now_ts, &gps);
}


/* Initialize GPS settings  */
void init_positions(void)
{
  //gps.fix = GPS_FIX_NONE;
  gps.fix = GPS_FIX_3D;
  gps_available = TRUE;
  gps.gspeed = 700; // To enable course setting
  gps.cacc = 0; // To enable course setting

  // CyberZoo ref point is in:
  // https://github.com/tudelft/infinium_video/blob/volker/coordTransforms.py
  gps.ecef_pos.x = 392433249;
  gps.ecef_pos.y = 30036183;
  gps.ecef_pos.z = 500219779;

}
