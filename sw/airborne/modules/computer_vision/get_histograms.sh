histogram_output_filename="mat_train_hists_color.csv"
texton_out_filename="mat_train_hists_texton.csv"
both_out_filename="mat_train_hists.csv"
#image_folder="/home/pold/Documents/Internship/datasets/board_train/"
image_folder="/home/pold/from_bebop/png/"
num_imgs=54
#textons_filename="textons.csv"
textons_filename="/home/pold/paparazzi/textons_malik.csv"
./get_histograms_c $histogram_output_filename $texton_out_filename $both_out_filename $image_folder $num_imgs $textons_filename
