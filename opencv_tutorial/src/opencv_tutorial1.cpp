#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std; 
using namespace cv;

/* INTERNAL CAM TEST CODE */
/* REF: https://www.youtube.com/watch?v=HqNJbx0uAv0 */

class cam_test{
  public:
    cam_test()
    {
      VideoCapture cap(CV_CAP_ANY); // OPENT THE VIDEO CAMERO NO. 0

	if(!cap.isOpened()) //if not success, exit program
	{
		cout<<"Cannot open the video cam" << endl;
	}

	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of te video
	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

	cout << "Frame size: " << dWidth << " x " << dHeight << endl;

	namedWindow("MyVideo", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

	while(1)
	{
		Mat frame;
		bool bSuccess = cap.read(frame); // read a new frame from video
	
		if(!bSuccess) // if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}

		imshow("My video", frame);

		if (waitKey(30) == 27) // wait for 'esc' key press form 30 ms
		{
			cout << "ESC key pressed by user"<< endl;
			break;
		}
	}
    }

    ~cam_test()
    {
	cvDestroyWindow("Camera_Output"); // Destroy Window
    }
};

int main(int argc, char **argv)
{
  //SetUP ROS.
  ros::init(argc, argv, "opencv_tutorial");
  cam_test cam_object;

  ROS_INFO("Cam Tested!");
}
