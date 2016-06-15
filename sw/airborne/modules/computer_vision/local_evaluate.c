static void evaluate();

int width = 640;
int height = 480;

static void evaluate() {
  trexton_init(void);

  int test_pics = 625;

  int i;
  for (i = 0; i < test_pics; i++) {

    char image_path[2048];
    sprintf(image_path, "%simg_%05d.png", image_folder, i);

    struct image_t rgb_img;
    image_create(&rgb_img, width, height, IMAGE_RGB);
    read_png_file(image_path, &rgb_img);
    
    trexton_func(img);
  }

}
