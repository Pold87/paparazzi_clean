#include "lib/vision/image.h"
#include "readpng.h"
#include "texton_settings.h"
#include "texton_helpers.h"
#include "image_conversions.h"

/* Array with the textons */
double textons[NUM_TEXTONS * CHANNELS][TOTAL_PATCH_SIZE];
int width = 640;
int height = 480;

void label_folder(char image_folder[], int num_imgs, double textons[][TOTAL_PATCH_SIZE], FILE *fp, FILE *fp_texton, FILE *fp_both) {
    
  int i;
  for (i = 0; i < num_imgs; i++) {
    printf("Image num: %d\n", i);
    fflush(stdout);
    char image_path[2048];
    sprintf(image_path, "%simg_%05d.png", image_folder, i);

    struct image_t rgb_img, opp_img, std_img, yuv_img;
    
    double means[8];
    image_create(&rgb_img, width, height, IMAGE_RGB);
    image_create(&opp_img, width, height, IMAGE_OPPONENT);
    image_create(&std_img, width, height, IMAGE_STD);
    image_create(&yuv_img, width, height, IMAGE_YUV422);
    
    printf("Type of creation is %d\n", opp_img.type);
    printf("Image path: %s\n", image_path);
    fflush(stdout);
    read_png_file(image_path, &rgb_img);
    printf("Read image %s\n", image_path);
    fflush(stdout);
    RGBtoOpponent(&rgb_img, &opp_img, means);
    /* Convert RGB to YUV */
    RGBtoYUV422(&rgb_img, &yuv_img);
    
    double color_hist[COLOR_CHANNELS * NUM_COLOR_BINS] = {0.0};
    get_color_histogram(&yuv_img, color_hist, NUM_COLOR_BINS);

    int u;
    for (u = 0; u < 3 * NUM_COLOR_BINS; u++) {
      printf("c: %f ", color_hist[u]);
    }
    printf("\n");

    image_grayscale_standardize(&opp_img, &std_img, means);
    printf("Type of after conversion is %d\n", opp_img.type);
    
    float texton_histogram[NUM_TEXTONS * CHANNELS] = {0.0};
    get_texton_histogram(&yuv_img, texton_histogram, textons);

    /* int m, n; */
    /* for (m = 0; m < NUM_TEXTONS; m++) { */
    /*   for (n = 0; n < TOTAL_PATCH_SIZE; n++) { */
    /* 	printf("%f ", textons[m][n]);	 */
    /*   } */
    /* } */

    /* Append this histogram to the output file */
    /* save_histogram(texton_histogram, fp, NUM_TEXTONS * CHANNELS); */
    /* save_histogram_double(color_hist, fp, NUM_COLOR_BINS * COLOR_CHANNELS); */
    save_histogram_float(texton_histogram, fp_texton, CHANNELS * NUM_TEXTONS);
    save_histogram_double(color_hist, fp, NUM_COLOR_BINS * COLOR_CHANNELS);
    /* save_histogram_both(color_hist, texton_histogram, fp_both, NUM_COLOR_BINS * COLOR_CHANNELS, CHANNELS * NUM_TEXTONS); */
    
   #if TEXTON_STANDARIZE
    
    image_free(&opp_img);
    image_free(&rgb_img);
    image_free(&std_img);
   #endif
    
  }
  
}

int main(int argc, char *argv[])
{

  
  /* The output file for labeling*/
  /* char histogram_output_filename[] = argv[1]; // "mat_train_hists_color.csv"; */
  /* char texton_out_filename[] = argv[2]; // "mat_train_hists_texton.csv"; */
  /* char both_out_filename[] = argv[3]; // "mat_train_hists.csv"; */
  /*static char histogram_output_filename[] = "chrome2_hists.csv";*/

  /* The folder that contains the images */
  /* char image_folder[] =  argv[4]; // "/home/pold/Documents/Internship/datasets/board_train/"; */
  /* static char image_folder[] =  "/home/pold/paparazzi/comp/"; */

  /* int num_imgs = atoi(argv[5]); */
  
  remove(argv[1]);
  remove(argv[2]);
  remove(argv[3]);
  
  /* Get textons (cluster centroids) */
  read_textons_from_csv(textons, argv[6]);

  FILE *fp = fopen(argv[1], "a");
  FILE *fp_texton = fopen(argv[2], "a");
  FILE *fp_both = fopen(argv[3], "a");

  int i, j;
  for (i = 0; i < NUM_TEXTONS * CHANNELS; i++) {
    for (j = 0; j < TOTAL_PATCH_SIZE; j++) {
      printf("%f ", textons[i][j]);
    }
    printf("\n");
  }


  /* Get the histograms of the folder */
  label_folder(argv[4], atoi(argv[5]), textons, fp, fp_texton, fp_both);

  /* Close the files */
  fclose(fp);
  fclose(fp_texton);
  fclose(fp_both);

  
  return 0;
}
