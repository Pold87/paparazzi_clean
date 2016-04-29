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


// Initialize the video_thread structure with the defaults
struct video_thread_t video_thread = {
  .is_running = FALSE,
  .fps = VIDEO_THREAD_FPS,
  .take_shot = FALSE,
  .shot_number = 0
};

// All dummy functions
void video_thread_init(void) {


}
static void *video_thread_function(void *data)
{

  /* TODO: use setting variable here */
  char image_folder[] = "/home/pold/from_bebop/png/";
  // Start streaming
  video_thread.is_running = true;

  /* TODO:could use for loop and number of pictures here */
  if (video_thread.is_running) {

    int i;
    /* TODO: use setting for 625 (amount of test pics) */
    for (i = 0; i < 625; i++) {

      struct image_t img, yuv_img;
      image_create(&img, 640, 480, IMAGE_RGB);
      image_create(&yuv_img, 640, 480, IMAGE_YUV422);
      printf("Image num: %d\n", i);
      char image_path[2048];
      sprintf(image_path, "%simg_%05d.png", image_folder, i);
      printf("Image path: %s\n", image_path);
      read_png_file(image_path, &img);
      RGBtoYUV422(&img, &yuv_img);      
      cv_run(&yuv_img);
      image_free(&img);
      image_free(&yuv_img);
      
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
 * This could take some time, because the thread is stopped asynchronous.
 */
void video_thread_stop(void)
{
  // Check if not already stopped streaming
  if (!video_thread.is_running) {
    return;
  }

  // Stop the streaming thread
  video_thread.is_running = false;

  // Stop the capturing
  if (!v4l2_stop_capture(video_thread.dev)) {
    printf("[video_thread] Could not stop capture of %s.\n", video_thread.dev->name);
    return;
  }

  // TODO: wait for the thread to finish to be able to start the thread again!
}


/**
 * Take a shot and save it
 * This will only work when the streaming is enabled
 */
void video_thread_take_shot(bool take)
{
  video_thread.take_shot = take;
}
