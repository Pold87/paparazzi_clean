get_histograms_c:
	gcc -ansi -g -pthread -I. -I/home/pold/paparazzi/sw/airborne/modules/computer_vision/lib -I/home/pold/paparazzi/sw/include -I ../particle_filter -Iopticflow/ -I/home/pold/paparazzi/sw/airborne/modules/computer_vision/lib/v4l -o get_histograms_c get_histograms.c lib/v4l/v4l2.c image_conversions.c  lib/encoding/jpeg.c  readcsv.c texton_helpers.c texton_helpers.h libcsv.c  -Ilib/vision lib/vision/*.c ../particle_filter/particle_filter.c ../particle_filter/random_number_gen.c -lm readpng.c readpng.h lib/vision/image.h  -std=c99 -Wno-implicit-function-declaration -L/home/pold/Downloads/libpng -I/usr/local/include/libpng16 -L/usr/local/lib -I/home/pold/paparazzi/sw/airborne/ -I/home/pold/paparazzi/sw/airborne/arch/linux/ -I/home/pold/paparazzi/var/include/ -lpng16

run:
	./trexton_regression

clean:
	rm get_histograms_c get_histograms
