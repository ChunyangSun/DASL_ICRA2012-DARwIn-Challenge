/*
 * Jane_darwinDrive_main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>






//opencv headers
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/un.h>

#include <fcntl.h>
#include <unistd.h>
#include <math.h>

using namespace cv;


#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../../Data/config.ini"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"




int main(void)
{


//opencv camera initialization
	CvCapture* capture = cvCaptureFromCAM( 0 );
	if( !capture ) {
		fprintf( stderr, "ERROR: capture is NULL \n" );
		getchar();
		return -1;
	}
   
   IplImage image;

   image.width = 320;
   image.depth = 240;
 
// Create a window in which the captured images will be presented
   namedWindow( "mywindow", CV_WINDOW_AUTOSIZE );

 
          


 	int i = 0;
    while(1){
		// Get one frame
		IplImage* frame = cvQueryFrame( capture );
		if( !frame ) {
		      fprintf( stderr, "ERROR: frame is null...\n" );
		      getchar();
		      break;
		}
		cvShowImage( "mywindow", frame );
		cvReleaseCapture( &capture );
		


		printf("HERE");
		
	
			
     	     }   
    
    return 0;
}
