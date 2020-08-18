/*

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include<iostream>
#include<cmath>
#include<cstring>
using namespace std;
using namespace cv;

int main() {
	Mat img = imread("rubik.jpg",1);
	Mat img_gray;
	Mat hsv = img.clone();
	Mat bgr[3];
	cvtColor(img, hsv, COLOR_BGR2HSV);
	cvtColor(img, img_gray, COLOR_BGR2GRAY);
	namedWindow("img", WINDOW_NORMAL);
	namedWindow("hsv", WINDOW_NORMAL);


	split(hsv, bgr);

	imshow("img", hsv);
	imshow("hsv", bgr[0]);
	waitKey(0);
}
*/
#include <chrono> 
using namespace std::chrono; 

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include<iostream>
#include<vector>
using namespace std;
using namespace cv;


#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>

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


bool isValid(int x, int y) {
	if(x>=0 && x < 480 && y >= 0 && y < 640) 
		return true;
	else
		return false;
}


struct pnt {
	int x, y;	
};


pnt poly_coor(Mat img) {
	int bmax = 15, gmax = 15, rmax = 15;
	Mat a = img.clone(), imgPoly;
	//cvtColor(a, imgPoly, CV_BGR2HSV);
	//inRange(a, Scalar(0,0,0), Scalar(bmax, gmax, rmax), imgPoly);
	inRange(imgPoly, Scalar(0, 0, 15), Scalar(90, 160, 150), imgPoly);	
	for(int i=0; i < 1; i++) {                                           //  Thresholding first time
		GaussianBlur(imgPoly, imgPoly, Size( 5, 5 ), 3, 3 );
		threshold(imgPoly, imgPoly, 50, 255, THRESH_BINARY);
	}
	for(int i=0; i<1; i++) {
		erode(imgPoly, imgPoly, getStructuringElement(MORPH_RECT, Size(3, 3)));
	}
	for(int i=0; i<1; i++) {
		dilate(imgPoly, imgPoly, getStructuringElement(MORPH_RECT, Size(3, 3)));
	}
	for(int i = 0 ;i < 1; i++) {
		erode(imgPoly, imgPoly, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		GaussianBlur( imgPoly, imgPoly, Size( 5, 5 ), 3, 3 );
		threshold(imgPoly, imgPoly, 230, 255, THRESH_BINARY);
	}

	Canny(imgPoly, imgPoly, 50,100,3);
	vector <vector<Point>> contoursPoly;
	vector < Vec4i > hierarchyPoly; 
	findContours(imgPoly, contoursPoly, hierarchyPoly, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
	
	// Removing element having area less than certain area
	float areaPoly = img.rows*img.cols/1000;
	for(auto j=contoursPoly.end()-1; j!=contoursPoly.begin()-1; j--) {	
		if( contourArea(*j) < areaPoly) {
			
			contoursPoly.erase(j);
		}
	}

	// Finding centre of triangle or rectangle
	vector<Moments> muPoly(contoursPoly.size());
	int x_poly = 0, y_poly = 0, i_poly;

	for(int i=0; i< contoursPoly.size(); i++) {
		muPoly[i] = moments(contoursPoly[i]);
		if( contourArea(contoursPoly[i]) > areaPoly ) {
			Point2f p(muPoly[i].m01 / muPoly[i].m00, muPoly[i].m10 / muPoly[i].m00 );
			x_poly = p.x;
			y_poly = p.y;
			i_poly = i;
			areaPoly = contourArea(contoursPoly[i]);
		}

	
	}
	pnt p1;
	p1.x = x_poly;
	p1.y = y_poly;
	return p1;

}
 



int main(int , char **)
{	
	char Command;


	settings("/dev/ttyUSB0");

	VideoCapture cap(2);
	if(!cap.isOpened())
		return -1;



	int i,j,hmin=50,smin=60,vmin=60,hmax=95,smax=255,vmax=200;
	// namedWindow("original",WINDOW_NORMAL);
	// namedWindow("hsv",WINDOW_NORMAL);
	//namedWindow("after_canny",WINDOW_NORMAL);
	// namedWindow("after_canny",WINDOW_NORMAL);
	// namedWindow("final_output",WINDOW_NORMAL);


	// createTrackbar("vmin","hsv",&vmin,255);
	// createTrackbar("vmax","hsv",&vmax,255);
	// createTrackbar("hmin","hsv",&hmin,180);
	// createTrackbar("hmax","hsv",&hmax,180);
	// createTrackbar("smin","hsv",&smin,255);
	// createTrackbar("smax","hsv",&smax,255);

	auto start = high_resolution_clock::now(); 


	while(1)
	{
		
		

		Mat img;
		for (int i=0;i<10;i++)
		{
			cap >> img;
		}
		namedWindow("4", WINDOW_NORMAL);
		imshow("4", img );
		cout<<"img rows = "<<img.rows<<" img.cols = "<<img.cols<<endl;
		//img = imread(argv[1],1);
		Mat a = img.clone();

		int bmax = 10, gmax = 10, rmax = 10;

		cvtColor(a,a,CV_BGR2HSV);
		
		// for(i=0;i<img.rows;++i)
		// {
		// 	for(j=0;j<img.cols;++j)
		// 	{
		// 		if(img.at<Vec3b>(i,j)[0]>=hmin && img.at<Vec3b>(i,j)[1]>=smin && img.at<Vec3b>(i,j)[2]>=vmin && img.at<Vec3b>(i,j)[0]<=hmax && img.at<Vec3b>(i,j)[1]<=smax && img.at<Vec3b>(i,j)[2]<=vmax)
		// 		{
		// 			a.at<Vec3b>(i,j)[0] = img.at<Vec3b>(i,j)[0];
		// 			a.at<Vec3b>(i,j)[1] = img.at<Vec3b>(i,j)[1];
		// 			a.at<Vec3b>(i,j)[2] = img.at<Vec3b>(i,j)[2];

		// 		}
		// 		else {
		// 			a.at<Vec3b>(i,j)[0] = 0;
		// 			a.at<Vec3b>(i,j)[1] = 0;
		// 			a.at<Vec3b>(i,j)[2] = 0;
		// 		}

		// 	}
		// }
		// int thres = 190;

		Mat imgPoly = img.clone();




		inRange(imgPoly, Scalar(0, 0, 15), Scalar(90, 160, 150), imgPoly);


		//cvtColor(imgPoly, imgPoly, CV_BGR2GRAY);
		

		// for(int i=0; i < 1; i++) {                                           //  Thresholding first time
		// 	GaussianBlur(imgPoly, imgPoly, Size( 9, 9 ), 9, 9 );
		// 	threshold(imgPoly, imgPoly, 50, 255, THRESH_BINARY);
		// }

		
		for(int i=0; i < 1; i++) {                                           //  Thresholding first time
			GaussianBlur(imgPoly, imgPoly, Size( 5, 5 ), 3, 3 );
			threshold(imgPoly, imgPoly, 50, 255, THRESH_BINARY);
		}


		// for(int i=0; i<5; i++) {											// Thresholding second time
		// 	GaussianBlur(imgPoly, imgPoly, Size( 7, 7 ), 5, 5 );
		// 	threshold(imgPoly, imgPoly, 120, 255, THRESH_BINARY);
		// }
		


		//threshold(imgPoly, imgPoly, 120, 255, THRESH_BINARY_INV);
		
		//inRange(a, Scalar(0,0,0), Scalar(bmax, gmax, rmax), imgPoly);



		Mat imgThresholded ;

		inRange(a, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), imgThresholded);      // detects green colour

		for(int i=0; i<1; i++) {
  			erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  			erode(imgPoly, imgPoly, getStructuringElement(MORPH_RECT, Size(3, 3)));
		}
  		for(int i=0; i<1; i++) {
  			dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5))); 
  			dilate(imgPoly, imgPoly, getStructuringElement(MORPH_RECT, Size(3, 3)));
  		}



  		// Applying Gaussian Blur and Thresholding to image
  		for(int i = 0 ;i < 1; i++) {
  			erode(imgPoly, imgPoly, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  			GaussianBlur( imgPoly, imgPoly, Size( 5, 5 ), 3, 3 );
			threshold(imgPoly, imgPoly, 230, 255, THRESH_BINARY);
  		}

  		
  
  		// Turning boundary pixels to black
		
  		for(int i=0; i<img.cols; i++) {
  			for(int k = 0; k < 5; k++) {
  			imgThresholded.at<uchar>(k, i) = 0;
  			}
  		}
  		for(int i=0; i<img.cols; i++) {
  			for(int k = 1; k < 5 ;k++){
  				imgThresholded.at<uchar>(img.rows-k, i) = 0;
  			}
  		}
  		for(int i=0; i<img.rows; i++) {
  			for(int k = 0 ;k <5; k++ ){
  				imgThresholded.at<uchar>(i, k) = 0;
  			}
		}
  		for(int i=0; i<img.rows; i++) {
  			for(int k = 1; k < 5 ;k++) {
  				imgThresholded.at<uchar>(i, img.cols-k) = 0;
  			}
  		}
  		for(int i = 0 ;i < 1; i++) {
  			erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  			GaussianBlur( imgThresholded, imgThresholded, Size( 5, 5 ), 3, 3 );
			threshold(imgThresholded, imgThresholded, 230, 255, THRESH_BINARY);
  		}

		Mat d = imgThresholded.clone();

		
	

		
		//Applying canny filter
		Canny(imgThresholded, imgThresholded,50,100,3);
		Canny(imgPoly, imgPoly, 50,100,3);




		// Finding contour for Green Circles

		vector <vector<Point>> contours;
		vector < Vec4i > hierarchy; 
		findContours(d, contours,hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));


		

		// Finding contour for Triangle or Rectangle
		vector <vector<Point>> contoursPoly;
		vector < Vec4i > hierarchyPoly; 
		findContours(imgPoly, contoursPoly, hierarchyPoly, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

		


		// Finding centre of triangle or rectangle

		
		float areaPoly = img.rows*img.cols/2000;

		int x_poly = 0, y_poly = 0, i_poly;
		// Removing polygons having area less than minimum area
		for(auto j=contoursPoly.end()-1; j!=contoursPoly.begin()-1; j--) {
			
			if( contourArea(*j) < areaPoly) {
				
				contoursPoly.erase(j);
			}
		}
		// cout<<"contoursPoly.size() = "<<contoursPoly.size()<<endl;
		// cout<<"contourArea(contoursPoly[0]) = "<<contourArea(contoursPoly[0])<<endl;

		vector<Moments> muPoly(contoursPoly.size());

		for(int i=0; i< contoursPoly.size(); i++) {
			muPoly[i] = moments(contoursPoly[i]);
			if( contourArea(contoursPoly[i]) > areaPoly ) {
				Point2f p(muPoly[i].m01 / muPoly[i].m00, muPoly[i].m10 / muPoly[i].m00 );
				x_poly = p.x;
				y_poly = p.y;
				i_poly = i;
				areaPoly = contourArea(contoursPoly[i]);
			}

		
		}		

		


		//cout<<"Area Poly Big = "<<areaPoly<<endl;


		// Drawing largest contour for polygon
		// --vector <vector<Point>> contoursPolyBig;
		// --contoursPolyBig.push_back(contoursPoly[i_poly]);

		// cout<<"contoursPolyBig.size() = "<<contoursPolyBig.size()<<endl;

		//drawContours(imgPoly, contoursPoly, -1, Scalar(232,112,114), 2, 8 );

		// namedWindow("imgPoly", WINDOW_NORMAL);
		// imshow("imgPoly", imgPoly);
		// waitKey(0);
		int flagPoly = 0;
		if(contoursPoly.size() > 0) {
			vector<Point> approx;
			approxPolyDP(Mat(contoursPoly[i_poly]), approx, arcLength(Mat(contoursPoly[i_poly]), true) * 0.01, true);

			if((approx.size() == 3 || approx.size() == 4 )) 
				flagPoly = 1;

		}
		//cout<<"approx.size() = "<<approx.size()<<endl;

		//cout<<"contour size = "<<contours.size()<<endl;
		// imshow("hsv", imgThresholded);
  // 		waitKey(1);

		//Drawing Contours for circle if circle found i.e contours size greater than zero

		int vert_centre_strip = img.cols/5;

  		if(contours.size() > 0) {


			// Mat f(d.rows, d.cols, CV_8UC3, Scalar(0,0,0));
			// drawContours(f, contours, -1, Scalar(232,112,114), 2, 8 );

			
			float Area = 100;
			//cout<<"Area = "<<Area<<endl;

			//cout<<"X_coor = "<<X_coor<<" Y_coor = "<<Y_coor<<endl;

			// Removing element having area less than Area;
			for(auto i=contours.end()-1; i!= contours.begin()-1; i--) {
				//cout<<"contourArea(*i) = "<<contourArea(*i)<<endl;
				if(contourArea(*i) < Area) {
					contours.erase(i);
				}
			}




			drawContours(imgPoly, contoursPoly, -1, Scalar(255), 2, 8 );

			

			

		}
		



		vector<Moments> mu(contours.size());

		for(int i=0; i< contours.size(); i++) {
			
				mu[i] = moments(contours[i]);
				// Y_coor = (int)mu[i].m10 / mu[i].m00;
				// X_coor = (int)mu[i].m01 / mu[i].m00;
				// Area = contourArea(contours[i]);
			
		}


		drawContours(imgThresholded, contoursPoly, -1, Scalar(255), 2, 8 );
	


		int X_coor, Y_coor;

		X_coor = 0; Y_coor = 0; 
		int x_temp = 0, y_temp = 0;
		//cout<<"contours.size() = "<<contours.size()<<endl;
		for(int i=0; i< contours.size(); i++) {
			Point2f p(mu[i].m01 / mu[i].m00, mu[i].m10 / mu[i].m00 );
			x_temp = p.x;
			y_temp = p.y;
			//cout<<"x_temp = "<<x_temp<<" y_temp = "<<y_temp<<endl;
			if(x_temp < 3*(float)img.rows/10 )
				continue;
			if(x_temp > X_coor) {
				X_coor = x_temp;
				Y_coor = y_temp;
			}
		}
		for(int u = -3; u<4; u++) {
			for(int v = -3; v<4; v++) {
				if(isValid(X_coor + v, Y_coor + u)) {
					//imgPoly.at<uchar>(X_coor + v,Y_coor + u) = 255;
					imgThresholded.at<uchar>(X_coor + v,Y_coor + u) = 255;
				}
			}
		}
		if(contoursPoly.size() > 0) {
			for(int u = -3; u<4; u++) {
				for(int v = -3; v<4; v++) {
					if(isValid(x_poly + v, y_poly + u)) {
						//imgPoly.at<uchar>(X_coor + v,Y_coor + u) = 255;
						imgPoly.at<uchar>(x_poly + v,y_poly + u) = 255;
					}
				}
			}
		}


		for(int i = 0; i<img.rows; i++) {
			imgPoly.at<uchar>(i, img.cols/2 - vert_centre_strip) = 255;
			imgPoly.at<uchar>(i, img.cols/2 + vert_centre_strip) = 255;

			imgThresholded.at<uchar>(i, img.cols/2 - vert_centre_strip) = 255;
			imgThresholded.at<uchar>(i, img.cols/2 + vert_centre_strip) = 255;
		}


		namedWindow("1",WINDOW_NORMAL);
		imshow("1", imgThresholded);
		namedWindow("2",WINDOW_NORMAL);
		imshow("2", imgPoly);
		waitKey(1);

		//cout<<"x_coor = "<<X_coor<<" y_coor = "<<Y_coor<<endl;
		
		// Condition means next obstacle is polygon

		//cout<<"x_poly = "<<x_poly<<" y_poly = "<<y_poly<<endl;

		//cout<<"9*(img.rows/10) = "<<9*(img.rows/10)<<endl;

		//cout<<"contours.size() = "<<contours.size()<<endl;
		
	

		if(contours.size() == 0) {
			//cout<<"main chala"<<endl;
			cout<<"Stop"<<endl;
			Command = '0';
			sendCommand(&Command);
			
		}
		else if(contoursPoly.size() > 0 && flagPoly)   {
			cout<<"Going in else if"<<endl;
			if(x_poly > X_coor && x_poly < 7*(img.rows/10))  {
				
				if( y_poly > img.cols/2 + vert_centre_strip )  {
					// turn right;
					cout<<"D"<<endl;
					Command = '3';
					sendCommand(&Command);
					// usleep(100000);
					
				}
				else if( y_poly < img.cols/2 - vert_centre_strip) {
					// turn left;
					cout<<"A"<<endl;
					Command = '2';
					sendCommand(&Command);
					// usleep(100000);
					
				}
				else {
					// check triangle or rectangle
					vector<Point> approx;

					approxPolyDP(Mat(contoursPoly[i_poly]), approx, arcLength(Mat(contoursPoly[i_poly]), true) * 0.01, true);
					if(approx.size() == 3) {
						Command = '6';
						sendCommand(&Command);
						usleep(500000);
						Command = '7';
						sendCommand(&Command);
						cout<<"Triangle Found"<<endl;
						while(1) {
							
							cout<<"W"<<endl;
							cout<<"A"<<endl;
							Command = '1';
							sendCommand(&Command);
							usleep(300000);
							Command = '2';
							sendCommand(&Command);
							usleep(100000);
						    Command = '0';
						    sendCommand(&Command);
							// cap>>img;
							// pnt p;
							// p = poly_coor(img);
							// if(p.x == 0 && p.y == 0)
							// 	break;
							// if(p.x > 7*(img.rows/10) || p.y > img.cols-img.cols/6)
							// 	break;

							break;

							
						}
					}
					else if(approx.size() == 4) {
						// -- LED ON
						for(int i=0; i< 2; i++){
							Command = '6';
							sendCommand(&Command);
							usleep(500000);
							Command = '7';
							sendCommand(&Command);
							usleep(300000);
						}
						cout<<"Rectangle Found"<<endl;
						while(1) {
							// turn right
							// move forward
							cout<<"W"<<endl;
							cout<<"D"<<endl;
							Command = '1';
							sendCommand(&Command);
							usleep(300000);
							Command = '3';
							sendCommand(&Command);
							usleep(100000);
						    Command = '0';
						    sendCommand(&Command);
							
							break;
							
						}
					}
				
					// if triangle turn left and simultneously move forward till x_poly > 8.5*(img.rows/10)

					// else turn right and simultaneously move forward
				}
			}
		}
		else {
			cout << "Going in else" << endl;
			
			if( Y_coor > img.cols/2 + vert_centre_strip )  {
				// turn right;
				cout<<"D"<<endl;
				Command = '3';
				usleep(300000);
				
	
			}
			else if( Y_coor < img.cols/2 - vert_centre_strip){
				// turn left;
				cout<<"A"<<endl;
				Command = '2';
				sendCommand(&Command);
				usleep(300000);
				
			}
			else {
				// move forward;
				cout<<"W"<<endl;
				Command = '1';
				sendCommand(&Command);
				usleep(900000);
				Command = '1';
				sendCommand(&Command);



			}
		}

	
		
	}


	auto stop = high_resolution_clock::now(); 
	auto duration = duration_cast<microseconds>(stop - start); 

	cout << "Duration => " << (float)duration.count() / 1000000/30	 << endl; 
}
