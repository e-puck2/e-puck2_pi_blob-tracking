#include <iostream>
#include <opencv2/opencv.hpp>
#include <jpeglib.h>
#include <errno.h>
#include <math.h>
extern "C" {
	#include <stdio.h>
	#include <stdint.h> 
	#include <sys/types.h>
	#include <sys/stat.h>
	#include <fcntl.h>
	#include <linux/i2c-dev.h> /* for I2C_SLAVE */
	#include <linux/i2c.h>
	#include <sys/ioctl.h>
	#include <stdlib.h>
	#include <unistd.h>
}

#define ACTUATORS_SIZE 19
#define SENSORS_SIZE 30

using namespace cv;
using namespace std;

static unsigned char jpegQuality = 70;

int fh;
uint8_t actuators_data[ACTUATORS_SIZE] = {0};
uint8_t sensors_data[SENSORS_SIZE];

extern "C" {
	int update_robot_sensors_and_actuators() {

		int ret = 0;
		int i = 0;

		ret = write(fh, actuators_data, ACTUATORS_SIZE);
		if (ret != ACTUATORS_SIZE) {
			perror("i2c write error");
			return -1;
		}

		ret = read(fh, sensors_data, SENSORS_SIZE);
		if (ret != SENSORS_SIZE) {
			perror("i2c read error");
			return -1;
		}

		return 0;
	}
}

/**
	Print error message and terminate programm with EXIT_FAILURE return code.
	\param s error message to print
*/
static void errno_exit(const char* s)
{
	fprintf(stderr, "%s error %d, %s\n", s, errno, strerror (errno));
	exit(EXIT_FAILURE);
}

/**
	Write image to jpeg file.
	\param img image to write
*/
static void jpegWrite(unsigned char* inputImg, int channels, char* name, int w, int h) {

	unsigned char *img = (unsigned char*)malloc(w*h*channels);
	memcpy(img, inputImg, w*h*channels);

	// From BGR to RGB.
	if(channels == 3) {
		int i, x, y;
		unsigned char red, blue;
		i=0;
		for(y=0; y < h; y++) {
		    for(x=0; x < w; x++) {
		        blue = (int)img[i];
		        red = (int)img[i+2];
				img[i] = red;
				img[i+2] = blue;
		        i+=3;
		    }
		}
	}

	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;
	
	JSAMPROW row_pointer[1];
	// Try to open file for saving.
	FILE *outfile = fopen( name, "wb");	
	if (!outfile) {
		errno_exit("jpeg");
	}

	// Create jpeg data.
	cinfo.err = jpeg_std_error( &jerr );
	jpeg_create_compress(&cinfo);
	jpeg_stdio_dest(&cinfo, outfile);

	// Set image parameters.
	cinfo.image_width = w;	
	cinfo.image_height = h;
	switch(channels) {
		case 3:
			cinfo.input_components = 3;
			cinfo.in_color_space = JCS_RGB;
			break;
		case 1: 
			cinfo.input_components = 1;
			cinfo.in_color_space = JCS_GRAYSCALE;
			break;
	}

	// Set jpeg compression parameters to default
	jpeg_set_defaults(&cinfo);
	// and then adjust quality setting.
	jpeg_set_quality(&cinfo, jpegQuality, TRUE);

	// Start compress.
	jpeg_start_compress(&cinfo, TRUE);

	// Feed data.
	while (cinfo.next_scanline < cinfo.image_height) {
		row_pointer[0] = &img[cinfo.next_scanline * cinfo.image_width *  cinfo.input_components];
		jpeg_write_scanlines(&cinfo, row_pointer, 1);
	}

	// Finish compression.
	jpeg_finish_compress(&cinfo);

	// Destroy jpeg data.
	jpeg_destroy_compress(&cinfo);

	// Close output file.
	fclose(outfile);

	free(img);
}


int main(int argc, char* argv[]) {
	float angle = 0;
	signed int speedFactor = 0;
    int i = 0;
	Mat image, frame, frameHSV, segmentated, lower_segmentated, upper_segmentated, cropped;
	int picWidth, picHeight;
	vector<KeyPoint> blobs; // Storage for blobs
	SimpleBlobDetector::Params params; // SimpleBlobDetector parameters.
	uint32_t maxSize = 0;
	uint8_t blobIndex = 0;
	
	// I2C init.
	fh = open("/dev/i2c-1", O_RDWR);	// open the I2C dev driver for bus 3
	ioctl(fh, I2C_SLAVE, 0x1F);			// tell the driver we want the device with address 0x1F (7-bits) on the I2C bus	
	actuators_data[4] = 3; 				// Speaker sound => no sound when > 2.
	
    VideoCapture camera(0);
	if(!camera.isOpened()) {
		std::cout << "Cannot open the video device" << std::endl;
		return -1;
	}

	// Blob detection is based on HSV color model that is more stable with various light conditions.
	// The user must enter the range for each HSV component.
	// In OpenCV the range for the Hue component is 0..180 (0=red, 60=green, 120=blue); Saturation and Value are 0..255.
	// Moreover the user must provide the center of the image, that is the center of the round image taken from the omnidirectional camera, and
	// two radii: the first represents the part of the image covered by the robot and the second represents the usable image (the radii are expressed in pixels).
	// Given the center and the two radii, the image will be processed in order to get only the usable image at the end (donut shape).
	// The user can enable a debugging mode by specifying "1" as the last parameter, this will save some jpeg images at various steps of the processing;
	// otherwise the user must specify "0" as last parameter.
	unsigned int hMin = atoi(argv[1]);
	unsigned int hMax = atoi(argv[2]);
	unsigned int sMin = atoi(argv[3]);
	unsigned int sMax = atoi(argv[4]);
	unsigned int vMin = atoi(argv[5]);
	unsigned int vMax = atoi(argv[6]);
	unsigned int centerX = atoi(argv[7]);
	unsigned int centerY = atoi(argv[8]);
	unsigned int radius1 = atoi(argv[9]);
	unsigned int radius2 = atoi(argv[10]);
	unsigned int debug = atoi(argv[11]);

	std::cout << "hMin: " << hMin << ", hMax: " << hMax << std::endl;
	std::cout << "sMin: " << sMin << ", sMax: " << sMax << std::endl;
	std::cout << "vMin: " << vMin << ", vMax: " << vMax << std::endl;
	std::cout << "center x: " << centerX << ", center y: " << centerY << std::endl;
	std::cout << "rad1: " << radius1 << ", rad2: " << radius2 << std::endl;

    std::cerr << "frame width = " << (int)camera.get(CV_CAP_PROP_FRAME_WIDTH) << std::endl;
    std::cerr << "frame height = " << (int)camera.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
    std::cerr << "fps = " << (int)camera.get(CV_CAP_PROP_FPS) << std::endl;
    std::cerr << "format = " << (int)camera.get(CV_CAP_PROP_FORMAT) << std::endl;
	
    //if(camera.set(CV_CAP_PROP_FRAME_WIDTH, 352))
    //    printf("Cant set width property\n");
    //if(camera.set(CV_CAP_PROP_FRAME_HEIGHT, 288))
    //    printf("Cant set height property\n");
    //std::cerr << "frame width = " << (int)camera.get(CV_CAP_PROP_FRAME_WIDTH) << std::endl;
    //std::cerr << "frame height = " << (int)camera.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;

	// Blob detector setup.
	// Change thresholds
	params.minThreshold = 10;
	params.maxThreshold = 200;
	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 20;
	// Filter by Circularity
	params.filterByCircularity = false;
	//params.minCircularity = 0.1;
	// Filter by Convexity
	params.filterByConvexity = false;
	//params.minConvexity = 0.87;
	// Filter by Inertia
	params.filterByInertia = false;
	params.minInertiaRatio = 0.01;	
	
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
	
    while(1) {

        camera.read(image);
        if(image.empty()) {
            std::cerr << "frame not grabbed properly" << std::endl;
            return -1;
        }
	
		if(debug==1) {
			jpegWrite((unsigned char*)image.data, image.channels(), (char*)"image.jpg", image.cols, image.rows);
			std::cerr << "frame grabbed" << std::endl;
			std::cerr << "nChannels = " << image.channels() << std::endl;		
		}

		// Copy image to frame.
		frame = image.clone();	
		
		// Prepare the matrix with the donut shape representing the usable image.
	  	cropped = Mat(frame.rows, frame.cols, CV_8UC3, Scalar(0,0,0)); // Fill the matrix with zeros.
		circle(cropped, Point(centerX, centerY), radius2, cvScalar(255, 255, 255), -1, 8); // White circle covering the actual image.
		circle(cropped, Point(centerX, centerY), radius1, cvScalar(0, 0, 0), -1, 8); // Black circle covering the robot.
		
		// Apply a bitwise operation to get our region of interest.
		bitwise_and(image, cropped, frame);
		
		// Convert from BGR to HSV space.
		cvtColor(frame, frameHSV, CV_BGR2HSV);

		// The Hue range is 0..180 and it is circular/continuous, for example the red corresponds to 0 and this means that values near red 
		// corresponds to both values <180 and values >0. 
		// So we need to take care for special cases when the min is greater than max.
		if(hMin > hMax) {
			inRange(frameHSV, Scalar(0, sMin, vMin), Scalar(hMax, sMax, vMax), lower_segmentated); // Look between 0 and H max.
			inRange(frameHSV, Scalar(hMin, sMin, vMin), Scalar(179, sMax, vMax), upper_segmentated); // Look between H min and 180.
			addWeighted(lower_segmentated, 1.0, upper_segmentated, 1.0, 0.0, segmentated); // Put together the results.
		} else {
			inRange(frameHSV, Scalar(hMin, sMin, vMin), Scalar(hMax, sMax, vMax), segmentated);
		}
	 
		if(debug==1) {
			jpegWrite((unsigned char*)segmentated.data, segmentated.channels(), (char*)"segmented.jpg", segmentated.cols, segmentated.rows);
		}

		// Smooth the result: can experiment either or both.
		medianBlur(segmentated, segmentated, 7);
		GaussianBlur(segmentated, segmentated, Size(9, 9), 0, 0);

		if(debug==1) {
			jpegWrite((unsigned char*)segmentated.data, segmentated.channels(), (char*)"segmented-smooth.jpg", segmentated.cols, segmentated.rows);
		}
		
		// Identify the blobs present in the binary image.
		detector->detect(segmentated, blobs);
		std::cerr << "Found " << blobs.size() << " blobs!" << std::endl;
		
		// Draw informations about the blobs.
		drawKeypoints(frame, blobs, frame, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
		
		picWidth = frame.cols;
		picHeight = frame.rows;	

		if(blobs.size() > 0) {
		
			// Find the largest blob, consider it as the target.
			maxSize = 0;
			blobIndex = 0;
			for (int i = 0; i < blobs.size(); i++) {
				if(blobs[i].size > maxSize) {
					maxSize = blobs[i].size;
					blobIndex = i;
				}
				if(debug==1) {
					std::cout << "blob " << i << " at (" << blobs[i].pt.x << ", " << blobs[i].pt.y << "), size = " << blobs[i].size << std::endl;
				}
			}
			if(debug==1) {
				std::cout << "largest blob = " << (int)blobIndex << std::endl;
			}

			angle = atan2(((picWidth/2)-blobs[blobIndex].pt.x)/(picWidth/2) , ((picHeight/2)-blobs[blobIndex].pt.y)/(picHeight/2))/M_PI*180.0; // -180..180

			if(debug==1) {
				std::cout << "center-x: " << blobs[blobIndex].pt.x << " center-y: " << blobs[blobIndex].pt.y << std::endl;
				std::cout << "x: " << ((picWidth/2)-blobs[blobIndex].pt.x)/(picWidth/2) << " y: " << ((picHeight/2)-blobs[blobIndex].pt.y)/(picHeight/2) << std::endl;
				std::cout << "angle = " << angle << std::endl;	
			}
			
			if(abs((int)angle) > 10) { // Plus or minus 10 degrees is considered looking towards the target.
				speedFactor = 2*abs((int)angle) + 150; // The higher the angle, the higher the rotation speed.
				
				if(angle > 0) { // Rotate right.			
					if(debug==1) {
						std::cerr << "Rotate right" << std::endl;
						std::cout << "l: " << speedFactor << ", r: " << -speedFactor << std::endl;						
					}				
					actuators_data[0] = speedFactor&0xFF;		// Left speed: 512
					actuators_data[1] = speedFactor>>8;
					actuators_data[2] = (-speedFactor)&0xFF;		// Right speed: -512
					actuators_data[3] = (-speedFactor)>>8;	
				} else { // Rotate left.
					if(debug==1) {
						std::cerr << "Rotate left" << std::endl;
						std::cout << "l: " << -speedFactor << ", r: " << speedFactor << std::endl;	
					}			
					actuators_data[0] = (-speedFactor)&0xFF;		// Left speed: 512
					actuators_data[1] = (-speedFactor)>>8;
					actuators_data[2] = speedFactor&0xFF;		// Right speed: -512
					actuators_data[3] = speedFactor>>8;				
				}			
				
			} else { // Pointing to the target, thus stop the robot.
				if(debug==1) {
					std::cerr << "Pointing to the target, stop" << std::endl;
				}
				actuators_data[0] = 0;		// Left speed: 0
				actuators_data[1] = 0;
				actuators_data[2] = 0;		// Right speed: 0
				actuators_data[3] = 0;		
			}		

		} else { // No blobs found, thus stop the robot.
			if(debug==1) {
				std::cerr << "Stop" << std::endl;
			}		
			actuators_data[0] = 0; // Left speed: 0
			actuators_data[1] = 0;
			actuators_data[2] = 0; // Right speed: 0
			actuators_data[3] = 0;
		}

		update_robot_sensors_and_actuators();	
	
		if(debug==1) {
			jpegWrite((unsigned char*)frame.data, frame.channels(), (char*)"blob.jpg", frame.cols, frame.rows);
		}

    }

    return 0;
}
