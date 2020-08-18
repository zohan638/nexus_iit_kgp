#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include<iostream>
#include<algorithm>
#include <chrono> 
using namespace std::chrono; 
#include<cmath>
using namespace std;
using namespace cv;
#include "opencv2/calib3d/calib3d.hpp"

#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
//////////////////////////////////////////////////////////////
int fd;
void settings(const char *abc)
{
  	fd = open(abc,O_RDWR | O_NOCTTY); /* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
  	usleep(3500000);
                                	/* O_RDWR Read/Write access to serial port       	*/
                                	/* O_NOCTTY - No terminal will control the process   */
                                	/* O_NDELAY -Non Blocking Mode,Does not care about-  */
                                	/* -the status of DCD line,Open() returns immediatly */                                        
                                    
        	if(fd == -1)                    	/* Error Checking */
               	printf("\n  Error! in Opening ttyUSB0  ");
        	else
               	printf("\n  ttyUSB0 Opened Successfully ");
   	struct termios toptions;     	/* get current serial port settings */
   	tcgetattr(fd, &toptions);    	/* set 9600 baud both ways */
   	cfsetispeed(&toptions, B9600);
   	cfsetospeed(&toptions, B9600);   /* 8 bits, no parity, no stop bits */
   	toptions.c_cflag &= ~PARENB;
   	toptions.c_cflag &= ~CSTOPB;
   	toptions.c_cflag &= ~CSIZE;
   	toptions.c_cflag |= CS8;     	/* Canonical mode */
   	toptions.c_lflag |= ICANON;   	/* commit the serial port settings */
   	tcsetattr(fd, TCSANOW, &toptions);
}
void sendCommand(const char *abc)
{
   write(fd, abc, 1);
}

///////////////////////////////////////

vector <int> storedNumber ;

char follow_circle = 'G';        /// --------INITIALLY follow Green circle

bool is_barcode(Mat img);

float dist(int x1, int y1, int x2, int y2) {
	float w = sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
	return w;
}
int BarCode(Mat img);

Point2f getCoorForBarcode(Mat img);

int main() {

	//settings(/dev/ttyACMO/Arduino/Genuino Uno);

	VideoCapture cap(0);
	if(!cap.isOpened())
		return -1;
	auto start = high_resolution_clock::now(); 

	int i,j,Green_hmin = 55, Green_smin =0, Green_vmin = 0, Green_hmax=65, Green_smax=255, Green_vmax=255;
	int Red_hmin = 55, Red_smin =0,Red_vmin = 0, Red_hmax = 0, Red_smax=255, Red_vmax=255;
	int Blue_hmin = 90, Blue_smin =0, Blue_vmin = 0, Blue_hmax=65, Blue_smax=255, Blue_vmax=255;
	
	// namedWindow("original",WINDOW_NORMAL);
	// namedWindow("hsv",WINDOW_NORMAL);
	// namedWindow("final_output",WINDOW_NORMAL);


	for(int u=0; u<1; u++) {
		Mat img;
		cap>>img;
		img = imread("barcode_circle2.jpg",1);

		Mat blueThresh = img.clone();
		Mat greenThresh = img.clone();
		Mat redThresh = img.clone();


		// Detects each color
		cvtColor(blueThresh, blueThresh ,CV_BGR2HSV);
		cvtColor(greenThresh, greenThresh ,CV_BGR2HSV);
		cvtColor(redThresh, redThresh ,CV_BGR2HSV);


		
		// Thresholding for each circle
		inRange(blueThresh, Scalar(Blue_hmin, Blue_smin, Blue_vmin), Scalar(Blue_hmax, Blue_smax, Blue_vmax), blueThresh);
		inRange(greenThresh, Scalar(Green_hmin, Green_smin, Green_vmin), Scalar(Green_hmax, Green_smax, Green_vmax), greenThresh);
		inRange(redThresh, Scalar(Red_hmin, Red_smin, Red_vmin), Scalar(Red_hmax, Red_smax, Red_vmax), redThresh);


		// Eroding and Dilating the images
		// for(int i=0; i<2; i++) {
  // 			erode(blueThresh, blueThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  // 			erode(greenThresh, greenThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  // 			erode(redThresh, redThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  			
		// }
  		// for(int i=0; i<2; i++) {
  		// 	dilate(blueThresh, blueThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  		// 	dilate(greenThresh, greenThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  		// 	dilate(redThresh, redThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  			
  		// }


  		// Applying Gaussian Blur and Thresholding to image
  	// 	for(int i = 0 ;i < 1; i++) {
  	// 		erode(blueThresh, blueThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  	// 		erode(greenThresh, greenThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  	// 		erode(redThresh, redThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

  	// 		GaussianBlur(blueThresh, blueThresh, Size( 5, 5 ), 3, 3 );
  	// 		GaussianBlur(greenThresh, greenThresh, Size( 5, 5 ), 3, 3 );
  	// 		GaussianBlur(redThresh, redThresh, Size( 5, 5 ), 3, 3 );

			// threshold(blueThresh, blueThresh, 230, 255, THRESH_BINARY);
			// threshold(greenThresh, greenThresh, 230, 255, THRESH_BINARY);
			// threshold(redThresh, redThresh, 230, 255, THRESH_BINARY);
  	// 	}


  		// Turning boundary pixels to black
		
  		for(int i=0; i<img.cols; i++) {
  			for(int k = 0; k < 5; k++) {
  				blueThresh.at<uchar>(k, i) = 0;
  				greenThresh.at<uchar>(k, i) = 0;
  				redThresh.at<uchar>(k, i) = 0;
  			}
  		}
  		for(int i=0; i<img.cols; i++) {
  			for(int k = 1; k < 5 ;k++){
  				blueThresh.at<uchar>(img.rows-k, i) = 0;
  				greenThresh.at<uchar>(img.rows-k, i) = 0;
  				redThresh.at<uchar>(img.rows-k, i) = 0;
  			}
  		}
  		for(int i=0; i<img.rows; i++) {
  			for(int k = 0 ;k <5; k++ ){
  				blueThresh.at<uchar>(i, k) = 0;
  				greenThresh.at<uchar>(i, k) = 0;
  				redThresh.at<uchar>(i, k) = 0;
  			}
		}
  		for(int i=0; i<img.rows; i++) {
  			for(int k = 1; k < 5 ;k++) {
  				blueThresh.at<uchar>(i, img.cols-k) = 0;
  				greenThresh.at<uchar>(i, img.cols-k) = 0;
  				redThresh.at<uchar>(i, img.cols-k) = 0;
  			}
  		}


  		for(int i = 0 ;i < 1; i++) {
  			erode(blueThresh, blueThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  			erode(greenThresh, greenThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  			erode(redThresh, redThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

  			GaussianBlur(blueThresh, blueThresh, Size( 5, 5 ), 3, 3 );
  			GaussianBlur(greenThresh, greenThresh, Size( 5, 5 ), 3, 3 );
  			GaussianBlur(redThresh, redThresh, Size( 5, 5 ), 3, 3 );

			threshold(blueThresh, blueThresh, 230, 255, THRESH_BINARY);
			threshold(greenThresh, greenThresh, 230, 255, THRESH_BINARY);
			threshold(redThresh, redThresh, 230, 255, THRESH_BINARY);
  		}

  		//Applying canny filter
		Canny(blueThresh, blueThresh,50,100,3);
		Canny(greenThresh, greenThresh,50,100,3);
		Canny(redThresh, redThresh,50,100,3);



		// Finding contour for Blue Circles

		vector <vector<Point>> contoursBlue;
		vector < Vec4i > hierarchyBlue; 
		findContours(blueThresh, contoursBlue, hierarchyBlue , CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

		// Finding contour for Green Circles

		vector <vector<Point>> contoursGreen;
		vector < Vec4i > hierarchyGreen; 
		findContours(greenThresh, contoursGreen, hierarchyGreen , CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

		// Finding contour for Red Circles

		vector <vector<Point>> contoursRed;
		vector < Vec4i > hierarchyRed; 
		findContours(redThresh, contoursRed, hierarchyRed , CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));


		//Drawing Contours for each colour
		// drawContours(blueThresh, contoursBlue, -1, Scalar(232,112,114), 2, 8 );
		// drawContours(greenThresh, contoursGreen, -1, Scalar(232,112,114), 2, 8 );
		// drawContours(redThresh, contoursRed, -1, Scalar(232,112,114), 2, 8 );



		// Removing element having area less than Area;

		float minContArea = (img.rows*img.cols)/1200;

		for(auto i=contoursBlue.end()-1; i!= contoursBlue.begin()-1; i--) {
			//cout<<"contourArea(*i) = "<<contourArea(*i)<<endl;
			if(contourArea(*i) < minContArea) {
				contoursBlue.erase(i);
			}
		}

		for(auto i=contoursGreen.end()-1; i!= contoursGreen.begin()-1; i--) {
			//cout<<"contourArea(*i) = "<<contourArea(*i)<<endl;
			if(contourArea(*i) < minContArea) {
				contoursGreen.erase(i);
			}
		}

		for(auto i=contoursRed.end()-1; i!= contoursRed.begin()-1; i--) {
			//cout<<"contourArea(*i) = "<<contourArea(*i)<<endl;
			if(contourArea(*i) < minContArea) {
				contoursRed.erase(i);
			}
		}

		// Calculating Moments    X and Y coordinates ----////////////////////////////////////////////////

		vector<Moments> muBlue(contoursBlue.size());
		for(int i=0; i< contoursBlue.size(); i++) {
			
				muBlue[i] = moments(contoursBlue[i]);
				// Y_coor = (int)mu[i].m10 / mu[i].m00;
				// X_coor = (int)mu[i].m01 / mu[i].m00;
				// Area = contourArea(contours[i]);
		}

		vector<Moments> muGreen(contoursGreen.size());
		for(int i=0; i< contoursGreen.size(); i++) {
			
				muGreen[i] = moments(contoursGreen[i]);
				// Y_coor = (int)mu[i].m10 / mu[i].m00;
				// X_coor = (int)mu[i].m01 / mu[i].m00;
				// Area = contourArea(contours[i]);
		}

		vector<Moments> muRed(contoursRed.size());
		for(int i=0; i< contoursRed.size(); i++) {
			
				muRed[i] = moments(contoursRed[i]);
				// Y_coor = (int)mu[i].m10 / mu[i].m00;
				// X_coor = (int)mu[i].m01 / mu[i].m00;
				// Area = contourArea(contours[i]);
		}


		int X_coorBlue, Y_coorBlue;
		int X_coorGreen, Y_coorGreen;
		int X_coorRed, Y_coorRed;

		X_coorBlue = 0;  Y_coorBlue = 0; 
		X_coorGreen = 0; Y_coorGreen = 0; 
		X_coorRed = 0;   Y_coorRed = 0; 

		int x_temp = 0, y_temp = 0;
		//cout<<"contours.size() = "<<contours.size()<<endl;

		for(int i=0; i< contoursBlue.size(); i++) {
			Point2f p(muBlue[i].m01 / muBlue[i].m00, muBlue[i].m10 / muBlue[i].m00 );
			x_temp = p.x;
			y_temp = p.y;
			//cout<<"x_temp = "<<x_temp<<" y_temp = "<<y_temp<<endl;
			if(x_temp > 9*(float)img.rows/10 )
				continue;
			if(x_temp > X_coorBlue) {
				X_coorBlue = x_temp;
				Y_coorBlue = y_temp;
			}
		}

		x_temp = 0; y_temp = 0;

		for(int i=0; i< contoursGreen.size(); i++) {
			Point2f p(muGreen[i].m01 / muGreen[i].m00, muGreen[i].m10 / muGreen[i].m00 );
			x_temp = p.x;
			y_temp = p.y;
			//cout<<"x_temp = "<<x_temp<<" y_temp = "<<y_temp<<endl;
			if(x_temp > 9*(float)img.rows/10 )
				continue;
			if(x_temp > X_coorGreen) {
				X_coorGreen = x_temp;
				Y_coorGreen = y_temp;
			}
		}

		x_temp = 0; y_temp = 0;

		for(int i=0; i< contoursRed.size(); i++) {
			Point2f p(muRed[i].m01 / muRed[i].m00, muRed[i].m10 / muRed[i].m00 );
			x_temp = p.x;
			y_temp = p.y;
			//cout<<"x_temp = "<<x_temp<<" y_temp = "<<y_temp<<endl;
			if(x_temp > 9*(float)img.rows/10 )
				continue;
			if(x_temp > X_coorRed) {
				X_coorRed = x_temp;
				Y_coorRed = y_temp;
			}
		}

		// Check Value of character follow_circle

		
		// if follow_circle == 'B' track blue circles
		// if follow_circle == 'G' track green circles
		// if follow_circle == 'R' track red circles


		Point2f p = getCoorForBarcode(img);
		int x_poly, y_poly;
		x_poly = p.x;
		y_poly = p.y;

		int x_max = max(X_coorBlue, max(X_coorRed, X_coorGreen));

		char Command;

		//cout<<"x_max = "<<x_max<<" x_poly "<<x_poly<<endl;

		if(x_poly > x_max && x_poly < 9*(img.rows/10) ) {
			if(is_barcode)  {
				int barvalue = BarCode(img);
				if(barvalue != -1) {
					// Check which colour to follow next
					// Change value of follow_circle to that colour

					storedNumber.push_back(barvalue);

					if(barvalue % 3 == 0) 
						follow_circle = 'G';
					else if(barvalue % 3 == 1)
						follow_circle = 'R';
					else if(barvalue % 3 == 2)
						follow_circle = 'B';
				}

			}

			else {
				// get average of all the numbers
				int sum = 0;
				for(int i=0 ;i<storedNumber.size(); i++) {
					sum += storedNumber[i];
				}

				int average;
				average = sum/storedNumber.size();


				if(average >= 0 && average <=5) {
					for(int i=0 ;i< 50; i++) {
						cout<<"W"<<endl;
						Command = '1';
						// sendCommand(&Command);

					}
				}
				else if(average >= 6 && average <= 10) {
					for(int i=0 ;i< 50; i++) {
						cout<<"D"<<endl;
						Command = '3';
						// sendCommand(&Command);
					}
				}
				else if(average >= 11 && average <= 15) {
					for(int i=0 ;i< 50; i++) {
						cout<<"A";
						Command = '2';
						// sendCommand(&Command);
					}
				}


				cout<<"Program ends"<<endl;
				return 0;
			}
		}

		int vert_centre_strip = img.cols/6;


		if(follow_circle == 'B') {
			if(contoursBlue.size() == 0) {
				cout<<"Stop"<<endl;
				Command = '0';
				// sendCommand(&Command);
			}
			else if( Y_coorBlue > img.cols/2 + vert_centre_strip )  {
				// turn right;
				cout<<"D"<<endl;
				Command = '3';
				// sendCommand(&Command);
			}
			else if( Y_coorBlue < img.cols/2 - vert_centre_strip){
				// turn left;
				cout<<"A"<<endl;
				Command = '1';
				// sendCommand(&Command);
			}
			else {
				// move forward;
				cout<<"W"<<endl;
				Command = '1';
				// sendCommand(&Command);
			}

		}
		else if(follow_circle == 'G') {
			if(contoursGreen.size() == 0) {
				cout<<"Stop"<<endl;
			}
			else if( Y_coorGreen > img.cols/2 + vert_centre_strip )  {
				// turn right;
				cout<<"D"<<endl;
				Command = '3';
				// sendCommand(&Command);
			}
			else if( Y_coorGreen < img.cols/2 - vert_centre_strip){
				// turn left;
				cout<<"A"<<endl;
				Command = '1';
				// sendCommand(&Command);
			}
			else {
				// move forward;
				cout<<"W"<<endl;
				Command = '1';
				// sendCommand(&Command);
			}
		}
		else if(follow_circle == 'R') {
			if(contoursRed.size() == 0) {
				cout<<"Stop"<<endl;
				Command = '0';
				// sendCommand(&Command);
			}
			else if( Y_coorRed > img.cols/2 + vert_centre_strip )  {
				// turn right;
				cout<<"D"<<endl;
				Command = '3';
				// sendCommand(&Command);
			}
			else if( Y_coorRed < img.cols/2 - vert_centre_strip){
				// turn left;
				cout<<"A"<<endl;
				Command = '2';
				// sendCommand(&Command);
			}
			else {
				// move forward;
				cout<<"W"<<endl;
				Command = '1';
				// sendCommand(&Command);
			}
		}

	}
	auto stop = high_resolution_clock::now(); 
	auto duration = duration_cast<microseconds>(stop - start); 

	cout << "Duration => " << (float)duration.count() / 1000000	 << endl; 
	waitKey(0);
}
//////////////////////////////////////////////////--------CHECK IF ITS A Barcode OR Quadilateral-------------///////////////////////////////////////////////////////

bool is_barcode(Mat img) {
	cout<<"is_barcode called"<<endl;
	Mat a = img.clone();
	//namedWindow("w1", WINDOW_NORMAL);

	Mat black_white = img.clone();
	cvtColor(black_white, black_white, CV_BGR2GRAY);
	for(int i=0; i < 1; i++) {                                           //  Thresholding first time
		GaussianBlur(black_white, black_white, Size( 9, 9 ), 3, 3 );
		threshold(black_white, black_white, 160, 255, THRESH_BINARY);
	}
	for(int i=0; i<5; i++) {											// Thresholding second time
		GaussianBlur(black_white, black_white, Size( 7, 7 ), 5, 5 );
		threshold(black_white, black_white, 120, 255, THRESH_BINARY);
	}

	// for black
	cvtColor(a,a,CV_BGR2HSV);
	inRange(a, Scalar(0, 0, 0), Scalar(60, 150, 160), a);

	

	for(int i= 0; i<5 ; i++)
		dilate(a, a, getStructuringElement(MORPH_RECT, Size(7, 7)));
	//imshow("w1", a);
	//cvtColor(a,a,CV_BGR2GRAY);
	
	//GaussianBlur( a, a, Size( 7, 7 ), 5, 5 );
	

	
	//imshow("w1",a);
	// namedWindow("b", WINDOW_NORMAL);
	// imshow("b",black_white);
	// waitKey(1);
	

	for(int i=0; i<1; i++) {
	erode(a, a, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		//dilate(a, a, getStructuringElement(MORPH_RECT, Size(3, 3)));

	GaussianBlur( a, a, Size( 7, 7 ), 3, 3 );
	threshold(a, a, 200, 255, THRESH_BINARY);
	}

	threshold(a, a, 200, 255, THRESH_BINARY_INV);

	
	// namedWindow("img", WINDOW_NORMAL);
	//  namedWindow("final_output", WINDOW_NORMAL);
	// namedWindow("homography", WINDOW_NORMAL);

	
	
	// for(int i=0; i<3; i++) 
	// 	dilate(a, a, getStructuringElement(MORPH_RECT, Size(3, 3)));

	// GaussianBlur( a, a, Size( 7, 7 ), 5, 5 );
	
	Mat imgThresholded = a.clone();

	Canny(imgThresholded, imgThresholded, 50,100,3);
	Mat b = imgThresholded.clone();


	
	//Finding contour
	vector <vector<Point>> contours;
	vector < Vec4i > hierarchy; 
	findContours(b, contours,hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
    
    //Approximating polygon
	vector<Point> approx;
	int areaBig = -2, iBig = 0;

	for(int i=0 ;i<contours.size(); i++) {
		if(contourArea(contours[i]) > areaBig) {
			areaBig = contourArea(contours[i]);
			iBig = i;
		}
	}

	if(areaBig < (img.cols*img.rows)/1000  )
		return -1;

	vector <vector<Point>> contourBig ;
	contourBig.push_back(contours[iBig]);
	//Drawing Contours
	Mat con(img.rows, img.cols, CV_8UC3, Scalar(0,0,0));
	drawContours(con, contourBig, -1, Scalar(232,112,114), 2, 8 );

	approxPolyDP(Mat(contours[iBig]), approx, arcLength(Mat(contours[iBig]), true) * 0.01, true);

	cout<<"approximate polygon = "<<approx.size()<<endl;
	if(approx.size()!= 4 )  {
		return -1;
	}
	for(int i=0 ; i<approx.size(); i++)
		cout<<"corner 1 "<<approx[i].x<<" "<< approx[i].y<<endl;

	Point2f top_left, top_right, bottom_left, bottom_right;

	Point2f cornerPoints[4];
	cornerPoints[0] = Point2f(0,0);
	cornerPoints[1] = Point2f(img.cols-1,0);
	cornerPoints[2] = Point2f(img.cols-1,img.rows-1);
	cornerPoints[3] = Point2f(0,img.rows-1);

	vector<Point2f> pts_src;

	for(int i=0; i<4; i++) {
		int x1, y1, x2, y2, x3, y3, x4, y4;
		x1 = approx[0].x;  y1 = approx[0].y;
		x2 = approx[1].x;  y2 = approx[1].y;
		x3 = approx[2].x;  y3 = approx[2].y;
		x4 = approx[3].x;  y4 = approx[3].y;
		float d[4];

		int x, y;
		x = cornerPoints[i].x;  y = cornerPoints[i].y;
		d[0] = d[1] = d[2] = d[3] = 0;
		d[0] = dist(x, y, x1, y1);
		d[1] = dist(x, y, x2, y2);
		d[2] = dist(x, y, x3, y3);
		d[3] = dist(x, y, x4, y4);

		float min_dis = min(min(d[0],d[1]), min(d[2],d[3]));
		for(int j = 0; j<4; j++) {
			if(min_dis == d[j]) {
				if(i == 0)
					top_left = Point2f(approx[j].x, approx[j].y);
				if(i == 1)
					top_right = Point2f(approx[j].x, approx[j].y);
				if(i == 2)
					bottom_right = Point2f(approx[j].x, approx[j].y);
				if(i == 3)
					bottom_left = Point2f(approx[j].x, approx[j].y);

			}
		}
	}

	// for(int i= 0; i<approx.size(); i++) {
	// 	int x, y;
	// 	x = approx[i].x;
	// 	y = approx[i].y;
	// 	float d1, d2, d3, d4;
	// 	d1 = d2 = d3 = d4 = 0;
	// 	d1 = dist(x, y, 0, 0);
	// 	d2 = dist(x, y, img.cols-1, 0);
	// 	d3 = dist(x, y, img.cols-1, img.rows-1);
	// 	d4 = dist(x, y, 0, img.rows-1);

	// 	cout<<"x = "<<x<<" y = "<<y<<endl;
	// 	cout<<"d1 = "<<d1<<" d2 = "<<d2<<" d3 = "<<d3<<" d4 = "<<d4<<endl;
	// 	if(d1<d2 && d1<d3 && d1<d4) {
	// 		top_left = Point2f(x, y);
	// 		cout<<"ass top_left"<<endl;
	// 	}
	// 	else if(d2<d1 && d2<d3 && d2<d4) {
	// 		top_right = Point2f(x, y);
	// 		cout<<"ass top_right"<<endl;
	// 	}
	// 	else if(d3<d1 && d3<d2 && d3<d4) {
	// 		bottom_right = Point2f(x, y);
	// 		cout<<"ass bottom_right"<<endl;
	// 	}
	// 	else if(d4<d1 && d4<d2 && d4<d3) {
	// 		bottom_left = Point2f(x, y);
	// 		cout<<"ass bottom_left"<<endl;
	// 	}
	// }
	cout<<"top_left = "<<top_left.x<<" "<<top_left.y<<endl;
	cout<<"top_right = "<<top_right.x<<" "<<top_right.y<<endl;
	cout<<"bottom_right = "<<bottom_right.x<<" "<<bottom_right.y<<endl;
	cout<<"bottom_left = "<<top_left.x<<" "<<top_left.y<<endl;
    pts_src.push_back(top_left);
    pts_src.push_back(top_right);
    pts_src.push_back(bottom_right);
    pts_src.push_back(bottom_left);
 
 
    // Read destination image.
    //Mat im_dst = imread("homography.jpg");
    // Four corners of the book in destination image.
    vector<Point2f> pts_dst;
    pts_dst.push_back(Point2f(0, 0));
    pts_dst.push_back(Point2f(img.cols-1,0 ));
    pts_dst.push_back(Point2f( img.cols-1, img.rows-1));
    pts_dst.push_back(Point2f( 0, img.rows-1));
 
    // Calculate Homography
    Mat homo = findHomography(pts_src, pts_dst);
 
    // Output image
    Mat im_out;
    // Warp source image to destination based on homography
    warpPerspective(black_white, im_out, homo, im_out.size());
    //rotate(im_out, im_out, ROTATE_90_CLOCKWISE);
    
    threshold(im_out, im_out, 127, 255, THRESH_BINARY);

    int flag1 = 0;

    for(int i= img.cols/4; i< 3*img.cols/4 ;i++) {
    	if(im_out.at<uchar>(img.rows/2 ,i) !=  im_out.at<uchar>(img.rows/2 ,i+1) ) {
    		flag1 = 1;
    		break;
    	}
    }

    int flag2 = 0;

    for(int i= img.rows/4; i< 3*img.rows/4 ;i++) {
    	if(im_out.at<uchar>(i, img.cols/2) !=  im_out.at<uchar>(i+1), img.cols/2 ) {
    		flag2 = 1;
    		break;
    	}
    }

    return flag1 || flag2;
}


////////////////////////////////////////////////////////////----BARCODE X and Y coordinate ------ //////////////////////////////////////////////////////////////////

Point2f getCoorForBarcode(Mat img) {
	//cout<<"getCoorForBarcode is called"<<endl;
	Mat a = img.clone();
	// namedWindow("w1", WINDOW_NORMAL);

	Mat black_white = img.clone();
	cvtColor(black_white, black_white, CV_BGR2GRAY);
	for(int i=0; i < 1; i++) {                                           //  Thresholding first time
		GaussianBlur(black_white, black_white, Size( 9, 9 ), 3, 3 );
		threshold(black_white, black_white, 160, 255, THRESH_BINARY);
	}
	for(int i=0; i<5; i++) {											// Thresholding second time
		GaussianBlur(black_white, black_white, Size( 7, 7 ), 5, 5 );
		threshold(black_white, black_white, 120, 255, THRESH_BINARY);
	}

	// for black
	cvtColor(a,a,CV_BGR2HSV);
	inRange(a, Scalar(0, 0, 0), Scalar(60, 150, 160), a);

	

	for(int i= 0; i<5 ; i++)
		dilate(a, a, getStructuringElement(MORPH_RECT, Size(7, 7)));
	//imshow("w1", a);
	//cvtColor(a,a,CV_BGR2GRAY);
	
	//GaussianBlur( a, a, Size( 7, 7 ), 5, 5 );
	

	
	//imshow("w1",a);
	// namedWindow("b", WINDOW_NORMAL);
	// imshow("b",black_white);
	// waitKey(1);
	

	for(int i=0; i<1; i++) {
	erode(a, a, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		//dilate(a, a, getStructuringElement(MORPH_RECT, Size(3, 3)));

	GaussianBlur( a, a, Size( 7, 7 ), 3, 3 );
	threshold(a, a, 200, 255, THRESH_BINARY);
	}

	threshold(a, a, 200, 255, THRESH_BINARY_INV);

	
	// namedWindow("img", WINDOW_NORMAL);
	//  namedWindow("final_output", WINDOW_NORMAL);
	// namedWindow("homography", WINDOW_NORMAL);

	
	
	// for(int i=0; i<3; i++) 
	// 	dilate(a, a, getStructuringElement(MORPH_RECT, Size(3, 3)));

	// GaussianBlur( a, a, Size( 7, 7 ), 5, 5 );
	
	Mat imgThresholded = a.clone();

	Canny(imgThresholded, imgThresholded, 50,100,3);
	Mat b = imgThresholded.clone();

	

	//cornerHarris( b, b, 7, 5, 0.05, BORDER_DEFAULT );

	
	//Finding contour
	vector <vector<Point>> contours;
	vector < Vec4i > hierarchy; 
	findContours(b, contours,hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
    
    //Approximating polygon
	vector<Point> approx;
	int areaBig = -2, iBig = 0;

	for(int i=0 ;i<contours.size(); i++) {
		if(contourArea(contours[i]) > areaBig) {
			areaBig = contourArea(contours[i]);
			iBig = i;
		}
	}


	vector<Moments> muPoly;
	muPoly.push_back( moments(contours[iBig]) );
	float areaPoly = img.rows*img.cols/1000;
	int x_poly = 0, y_poly = 0;

	Point2f p(muPoly[0].m01 / muPoly[0].m00, muPoly[0].m10 / muPoly[0].m00 );
	x_poly = p.x;
	y_poly = p.y;

	return p;

}

/////////////////////////////////////////////////////////////--- BARCODE FUNCTION ---///////////////////////////////////////////////////////////////////////


int BarCode(Mat img) {

	Mat a = img.clone();
	// namedWindow("w1", WINDOW_NORMAL);

	Mat black_white = img.clone();
	cvtColor(black_white, black_white, CV_BGR2GRAY);
	for(int i=0; i < 1; i++) {                                           //  Thresholding first time
		GaussianBlur(black_white, black_white, Size( 9, 9 ), 3, 3 );
		threshold(black_white, black_white, 160, 255, THRESH_BINARY);
	}
	for(int i=0; i<5; i++) {											// Thresholding second time
		GaussianBlur(black_white, black_white, Size( 7, 7 ), 5, 5 );
		threshold(black_white, black_white, 120, 255, THRESH_BINARY);
	}

	// for black
	cvtColor(a,a,CV_BGR2HSV);
	inRange(a, Scalar(0, 0, 0), Scalar(60, 150, 160), a);

	

	for(int i= 0; i<5 ; i++)
		dilate(a, a, getStructuringElement(MORPH_RECT, Size(7, 7)));
	//imshow("w1", a);
	//cvtColor(a,a,CV_BGR2GRAY);
	
	//GaussianBlur( a, a, Size( 7, 7 ), 5, 5 );
	

	
	//imshow("w1",a);
	// namedWindow("b", WINDOW_NORMAL);
	// imshow("b",black_white);
	// waitKey(1);
	

	for(int i=0; i<1; i++) {
	erode(a, a, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		//dilate(a, a, getStructuringElement(MORPH_RECT, Size(3, 3)));

	GaussianBlur( a, a, Size( 7, 7 ), 3, 3 );
	threshold(a, a, 200, 255, THRESH_BINARY);
	}

	threshold(a, a, 200, 255, THRESH_BINARY_INV);

	
	// namedWindow("img", WINDOW_NORMAL);
	//  namedWindow("final_output", WINDOW_NORMAL);
	// namedWindow("homography", WINDOW_NORMAL);

	
	
	// for(int i=0; i<3; i++) 
	// 	dilate(a, a, getStructuringElement(MORPH_RECT, Size(3, 3)));

	// GaussianBlur( a, a, Size( 7, 7 ), 5, 5 );
	
	Mat imgThresholded = a.clone();

	Canny(imgThresholded, imgThresholded, 50,100,3);
	Mat b = imgThresholded.clone();


	
	//Finding contour
	vector <vector<Point>> contours;
	vector < Vec4i > hierarchy; 
	findContours(b, contours,hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
    
    //Approximating polygon
	vector<Point> approx;
	int areaBig = -2, iBig = 0;

	for(int i=0 ;i<contours.size(); i++) {
		if(contourArea(contours[i]) > areaBig) {
			areaBig = contourArea(contours[i]);
			iBig = i;
		}
	}

	if(areaBig < (img.cols*img.rows)/1000  )
		return -1;

	vector <vector<Point>> contourBig ;
	contourBig.push_back(contours[iBig]);
	//Drawing Contours
	Mat con(img.rows, img.cols, CV_8UC3, Scalar(0,0,0));
	drawContours(con, contourBig, -1, Scalar(232,112,114), 2, 8 );

	approxPolyDP(Mat(contours[iBig]), approx, arcLength(Mat(contours[iBig]), true) * 0.01, true);

	cout<<"approximate polygon = "<<approx.size()<<endl;
	if(approx.size()!= 4 )  {
		return -1;
	}
	for(int i=0 ; i<approx.size(); i++)
		cout<<"corner 1 "<<approx[i].x<<" "<< approx[i].y<<endl;

	Point2f top_left, top_right, bottom_left, bottom_right;

	Point2f cornerPoints[4];
	cornerPoints[0] = Point2f(0,0);
	cornerPoints[1] = Point2f(img.cols-1,0);
	cornerPoints[2] = Point2f(img.cols-1,img.rows-1);
	cornerPoints[3] = Point2f(0,img.rows-1);

	vector<Point2f> pts_src;

	for(int i=0; i<4; i++) {
		int x1, y1, x2, y2, x3, y3, x4, y4;
		x1 = approx[0].x;  y1 = approx[0].y;
		x2 = approx[1].x;  y2 = approx[1].y;
		x3 = approx[2].x;  y3 = approx[2].y;
		x4 = approx[3].x;  y4 = approx[3].y;
		float d[4];

		int x, y;
		x = cornerPoints[i].x;  y = cornerPoints[i].y;
		d[0] = d[1] = d[2] = d[3] = 0;
		d[0] = dist(x, y, x1, y1);
		d[1] = dist(x, y, x2, y2);
		d[2] = dist(x, y, x3, y3);
		d[3] = dist(x, y, x4, y4);

		float min_dis = min(min(d[0],d[1]), min(d[2],d[3]));
		for(int j = 0; j<4; j++) {
			if(min_dis == d[j]) {
				if(i == 0)
					top_left = Point2f(approx[j].x, approx[j].y);
				if(i == 1)
					top_right = Point2f(approx[j].x, approx[j].y);
				if(i == 2)
					bottom_right = Point2f(approx[j].x, approx[j].y);
				if(i == 3)
					bottom_left = Point2f(approx[j].x, approx[j].y);

			}
		}
	}

	// for(int i= 0; i<approx.size(); i++) {
	// 	int x, y;
	// 	x = approx[i].x;
	// 	y = approx[i].y;
	// 	float d1, d2, d3, d4;
	// 	d1 = d2 = d3 = d4 = 0;
	// 	d1 = dist(x, y, 0, 0);
	// 	d2 = dist(x, y, img.cols-1, 0);
	// 	d3 = dist(x, y, img.cols-1, img.rows-1);
	// 	d4 = dist(x, y, 0, img.rows-1);

	// 	cout<<"x = "<<x<<" y = "<<y<<endl;
	// 	cout<<"d1 = "<<d1<<" d2 = "<<d2<<" d3 = "<<d3<<" d4 = "<<d4<<endl;
	// 	if(d1<d2 && d1<d3 && d1<d4) {
	// 		top_left = Point2f(x, y);
	// 		cout<<"ass top_left"<<endl;
	// 	}
	// 	else if(d2<d1 && d2<d3 && d2<d4) {
	// 		top_right = Point2f(x, y);
	// 		cout<<"ass top_right"<<endl;
	// 	}
	// 	else if(d3<d1 && d3<d2 && d3<d4) {
	// 		bottom_right = Point2f(x, y);
	// 		cout<<"ass bottom_right"<<endl;
	// 	}
	// 	else if(d4<d1 && d4<d2 && d4<d3) {
	// 		bottom_left = Point2f(x, y);
	// 		cout<<"ass bottom_left"<<endl;
	// 	}
	// }
	cout<<"top_left = "<<top_left.x<<" "<<top_left.y<<endl;
	cout<<"top_right = "<<top_right.x<<" "<<top_right.y<<endl;
	cout<<"bottom_right = "<<bottom_right.x<<" "<<bottom_right.y<<endl;
	cout<<"bottom_left = "<<top_left.x<<" "<<top_left.y<<endl;
    pts_src.push_back(top_left);
    pts_src.push_back(top_right);
    pts_src.push_back(bottom_right);
    pts_src.push_back(bottom_left);
 
 
    // Read destination image.
    //Mat im_dst = imread("homography.jpg");
    // Four corners of the book in destination image.
    vector<Point2f> pts_dst;
    pts_dst.push_back(Point2f(0, 0));
    pts_dst.push_back(Point2f(img.cols-1,0 ));
    pts_dst.push_back(Point2f( img.cols-1, img.rows-1));
    pts_dst.push_back(Point2f( 0, img.rows-1));
 
    // Calculate Homography
    Mat homo = findHomography(pts_src, pts_dst);
 
    // Output image
    Mat im_out;
    // Warp source image to destination based on homography
    warpPerspective(black_white, im_out, homo, im_out.size());
    //rotate(im_out, im_out, ROTATE_90_CLOCKWISE);
    
    threshold(im_out, im_out, 127, 255, THRESH_BINARY);

    int flag1 = 0;
    int val = im_out.at<uchar>(im_out.rows/2, im_out.cols/5) ;
    for(int j = im_out.cols/5; j < 4*im_out.cols/5; j++) {
    	if(im_out.at<uchar>(im_out.rows/2, j) != val) {
    		flag1 = 1;
    		break;
    	}
    }
    int flag2 = 0;
    val = im_out.at<uchar>(im_out.rows/4, im_out.cols/5);
    for(int j = im_out.cols/5; j < 4*im_out.cols/5; j++) {
    	if(im_out.at<uchar>(im_out.rows/4, j) != val) {
    		flag1 = 1;
    		break;
    	}
    }

    if(flag1 == 0 && flag2 == 0) {
    	rotate(im_out, im_out, ROTATE_90_CLOCKWISE);
    	threshold(im_out, im_out, 127, 255, THRESH_BINARY);
    }


    //imshow("homography",im_out);
	//Drawing Contours
	Mat f(a.rows, a.cols, CV_8UC3, Scalar(0,0,0));
	drawContours(f, contours, -1, Scalar(232,112,114), 2, 8 );

	

	cout<<"no of contours = "<<contours.size()<<endl;
	
	//cout<<"contours size = "<<contours[iBig].size()<<endl;
	int index, ii, x, l[3];
	ii = index = 0;
	x = im_out.rows/2;

	while(im_out.at<uchar>(x, ii) == 255) {
		ii++;
	}
	while(ii != img.cols-1) {
		while(im_out.at<uchar>(x, ii) == 0) {
			ii++;
		}
		l[index] = 0;
		while(im_out.at<uchar>(x, ii) == 255) {
			ii++;
			l[index]++;
		}
		index ++;
		if(index == 4)
			break;
	}
	
	cout<<l[0]<<" "<<l[1]<<" "<<l[2]<<" "<<l[3]<<endl;
	
	int min_no = min( min(l[0], l[1]), min(l[2], l[3])  );
	int max_no = max( max(l[0], l[1]), max(l[2], l[3])  );
	cout<<"min = "<<min_no<<" max = "<<max_no<<endl;

	for(int i=0; i<4; i++) {
		int m = l[i];
		if(min_no <= m && m < min_no+ (max_no-min_no)/2 )
			l[i] = 0;
		else
			l[i] = 1;
	}

	if(l[0] == l[1] && l[1] == l[2] && l[2] == l[3]) {
		int ind=0;
		while(im_out.at<uchar>(x, ind) == 255) {
			ind++;
		}
		while(im_out.at<uchar>(x, ind) == 0) {
			ind++;
		}
		while(im_out.at<uchar>(x, ind) == 255) {
			ind++;
		}
		int l_black = 0;
		while(im_out.at<uchar>(x, ind) == 0) {
			ind++;
			l_black++;
		}

		if(min_no < l_black && max_no < l_black) {
			l[0] = l[1] = l[2] = l[3] = 0;
		}
		else {
			l[0] = l[1] = l[2] = l[3] = 1;
		}
	}
	cout<<l[0]<<" "<<l[1]<<" "<<l[2]<<" "<<l[3]<<endl;
	int bar_val = 8*l[0] + 4*l[1] + 2*l[2] + l[3];
	//cout<<"Barcode value = "<<bar_val<<endl;
	
    // imshow("final_output",con);
    // waitKey(9000);
    return bar_val;
	//imshow("img", a);
	

}