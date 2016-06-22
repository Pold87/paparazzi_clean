/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * Dummy C implementation for simulation
 * The V4L2 could also work in simulation, but must be adapted a bit.
 */

// Own header

#include "modules/computer_vision/video_thread.h"
#include "modules/computer_vision/cv.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>

// Video
#include "lib/v4l/v4l2.h"
#include "lib/vision/image.h"
#include "lib/vision/bayer.h"
#include "lib/encoding/jpeg.h"
#include "peripherals/video_device.h"

#include "mcu_periph/sys_time.h"

#include BOARD_CONFIG

// Threaded computer vision
#include <pthread.h>
#include "rt_priority.h"

// Main thread
static void *video_thread_function(void *data);
void video_thread_periodic(void) { }

#include "video_thread.h"
#include "cv.h"
#include "lib/vision/image.h"
#include "image_conversions.h"

// Initialize the video_thread structure with the defaults
struct video_thread_t video_thread = {
  .is_running = FALSE
  /* .fps = VIDEO_THREAD_FPS, */
  /* .take_shot = FALSE, */
  /* .shot_number = 0 */
};

// All dummy functions
void video_thread_init(void) {


}
static void *video_thread_function(void *data)
{

   struct video_config_t *vid = (struct video_config_t *)data;

  /* TODO: use setting variable here */
  char image_folder[] = "/home/pold/from_bebop/png/";
  // Start streaming
  video_thread.is_running = true;

  /* TODO:could use for loop and number of pictures here */
  if (video_thread.is_running) {

    int j = 0;
    int i = 0;
    //int offset = 400;
    int offset = 0;
    //    int max_pic = 625;
    int max_pic = 0;
    /* TODO: use setting for 625 (amount of test pics) */
    while (1) {
    /* for (i = 400; i < 625; i++) { */
      i = offset + j;
      struct image_t img, yuv_img;
      image_create(&img, 640, 480, IMAGE_RGB);
      image_create(&yuv_img, 640, 480, IMAGE_YUV422);
      printf("Image num: %d\n", i);
      char image_path[2048];
      sprintf(image_path, "%simg_%05d.png", image_folder, i);
      printf("Image path: %s\n", image_path);
      read_png_file(image_path, &img);
      printf("Read file");
      fflush(stdout);
      printf("Converting");
      fflush(stdout);
      RGBtoYUV422(&img, &yuv_img);
      printf("Converted");
      fflush(stdout);
      cv_run_device(vid, &yuv_img);
      printf("Before free");
      fflush(stdout);
      image_free(&img);
      image_free(&yuv_img);
      j++;
      j = j % (1 + offset - max_pic);
    }
  }
}

/**
 * Start with streaming
 */
void video_thread_start(void)
{
  // Check if we are already running
  if (video_thread.is_running) {
    return;
  }

  // Start the streaming thread
  pthread_t tid;
  /* TODO: I just copied the command for creating the thread and set
     the camera location to dummy */
  if (pthread_create(&tid, NULL, video_thread_function, (void*)("dummy")) != 0) {
    printf("[vievideo] Could not create streaming thread.\n");
    return;
  }
}


/**
 * Stops the streaming
take * This could take some time, because the thread is stopped asynchronous.
 */
void video_thread_stop(void)
{

}


/**
 * Take a shot and save it
 * This will only work when the streaming is enabled
 */
void video_thread_take_shot(bool take)
{
}

bool add_video_device(struct video_config_t *device __attribute__((unused))){ return true; }
