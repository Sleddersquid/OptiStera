# NOTE: The variables PATH_INCLUDES & PATH_LIB must match the location of your OpenCV installation  
#
# To set PATH_INCLUDE: 
#	To find the 'include' directory, type in a terminal:
#	$ find / -name opencv.hpp 2>&1 | grep -v Permission denied"
#	$ /usr/local/include/opencv2/opencv.hpp
#	so your 'include' directory is located in '/usr/local/include/'
#
# To set PATH_LIB: 
#	To find the 'lib' directory, type in a terminal:
#	$ find / -name libopencv* 2>&1 | grep -v "Permission denied"
#	$ ...
#	$ /usr/local/lib/libopencv_videostab.so.3.0
#	$ ...
#	$ /usr/local/lib/libopencv_core.so.3.0.0
#	$ /usr/local/lib/libopencv_core.so.3.0
#	$ /usr/local/lib/libopencv_core.so
#	$ ...
#	so your 'lib' directory is located in '/usr/local/lib/'
#	and your OpenCV version is 3.0.0
#
#   In this sample code, OpenCV is installed in the path '/opt/installation'
#	with the following details:
#		Version: 3.4.4
#		Include Path: /opt/installation/OpenCV-3.4.4/include
#		Library Path: /opt/instllation/OpenCV-3.4.4/lib
#
#	Author: Juan C. SanMiguel (juancarlos.sanmiguel@uam.es)

CPPFLAGS = -g -Wall -DCHECK_OVERFLOW -O2

# LIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_videoio -lopencv_objdetect -lopencv_imgcodecs -lopencv_video
LIBS = -I/usr/include/opencv4 -I/usr/include/libcamera -llccv -lopencv_aruco -lopencv_barcode -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_cvv -lopencv_dnn_objdetect -lopencv_dnn_superres -lopencv_dpm -lopencv_face -lopencv_freetype -lopencv_fuzzy -lopencv_hdf -lopencv_hfs -lopencv_img_hash -lopencv_intensity_transform -lopencv_line_descriptor -lopencv_mcc -lopencv_quality -lopencv_rapid -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_shape -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_superres -lopencv_optflow -lopencv_surface_matching -lopencv_tracking -lopencv_highgui -lopencv_datasets -lopencv_text -lopencv_plot -lopencv_ml -lopencv_videostab -lopencv_videoio -lopencv_viz -lopencv_wechat_qrcode -lopencv_ximgproc -lopencv_video -lopencv_xobjdetect -lopencv_objdetect -lopencv_calib3d -lopencv_imgcodecs -lopencv_features2d -lopencv_dnn -lopencv_flann -lopencv_xphoto -lopencv_photo -lopencv_imgproc -lopencv_core
PATH_INCLUDES = /usr/local/include/
PATH_LIB = /usr/local/lib

OBJS_TB = main.o blobs.o ShowManyImages.o fgseg.o kalman.o vsrc.o 
BIN_TB = main

all: link_all
	rm -f $(OBJS_TB)

link_all: $(OBJS_TB)
	g++ -o $(BIN_TB) $(OBJS_TB) -L$(PATH_LIB) $(LIBS) 

main.o: main.cpp
	g++ $(CPPFLAGS) -I$(PATH_INCLUDES) -L$(PATH_LIB) -c main.cpp

blobs.o: blobs.cpp
	g++ $(CPPFLAGS) -I$(PATH_INCLUDES) -L$(PATH_LIB) $(LIBS) -c blobs.cpp
fgseg.o: fgseg.cpp
	g++ $(CPPFLAGS) -I$(PATH_INCLUDES) -L$(PATH_LIB) $(LIBS) -c fgseg.cpp
kalman.o: kalman.cpp
	g++ $(CPPFLAGS) -I$(PATH_INCLUDES) -L$(PATH_LIB) $(LIBS) -c kalman.cpp
vsrc.o: vsrc.cpp
	g++ $(CPPFLAGS) -I$(PATH_INCLUDES) -L$(PATH_LIB) $(LIBS) -c vsrc.cpp
ShowManyImages.o: ShowManyImages.cpp
	g++ $(CPPFLAGS) -I$(PATH_INCLUDES) -L$(PATH_LIB) $(LIBS) -c ShowManyImages.cpp

clean:
	rm -f $(BIN_TB) $(OBJS_TB)

