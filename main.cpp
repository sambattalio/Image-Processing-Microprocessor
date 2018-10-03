// Camera on NVidia Jetson TX1
// Project by Sam Battalio
// Requires Nvidia Tx1 and Roborio to use
// Adapted from school project

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <algorithm>
#include <string>
#include <math.h>

using namespace cv;

double angleToTarget = 0;
Size cvResizeDSize(0,0);
struct timeval timeout;

struct addrinfo hints, *res;
int status;
int socket_id;

// SET UP SOCKET SERVER

void * get_in_addr(struct sockaddr *sa)
{
    if(sa->sa_family == AF_INET){
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }
    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

// Display error message and quit program
void error(char * msg)
{
    perror(msg);
    exit(1);
}

// Attempt connection to Roborio board
int try_connect()
{
    std::cout << "Trying to connect\n\n";
    status = getaddrinfo("roborio-135-frc.local", "7821", &hints, &res);
    std::cout << "addrinfo : " << res->ai_addr << "\n";
    if(status != 0)
        return -1;
    socket_id = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    std::cout << "socket id: " << socket_id << "\n\n";
    if(socket_id < 0)
        return -1;
    status = connect(socket_id, res->ai_addr, res->ai_addrlen);
    std::cout << "con stat: " << status << "\n\n";
    if(status < 0)
        return -1;
    std::cout << "finished connection";
    return 0;
}


// Camera Class built with features to implement two cameras
class TrackingCamera
{
public:
    TrackingCamera(int cameraNumber, double hl, double hh, double sl, double sh, double ll, double lh, bool deb = false);
    void update();
    double getAngle();
    Mat frame;
    Size ss;
    int codec;
    Mat hslOutput;
    int fps;
    Mat threshOut;
private:
    VideoCapture cam;
    VideoWriter *video;
    int cameraNumber;
    vector<vector<Point> > mycontours;
    vector<vector<Point> > outcontours;
    vector<vector<Point> > filterContoursOutput;
    int width;
    int height;
    double targetX = 0, targetY = 0;
    double h[2];
    double s[2];
    double l[2];
    bool debug;
};



int main(int argc, const char * argv[]) {
    // SOCKET SETUP
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;
    timeout.tv_usec = 10;

    // Loop until the server connects to the board
    // Set to non -1 value to not connect to server
    int connected = -1;
    while(connected == -1)
    {
        connected = try_connect();
        if(connected == -1) return 0;
    }

    // Initialize Cameras
    // The HSL values are set from separate fine tuning program
    TrackingCamera camO(0,0,180,0,255,215,255,true); //h,h,s,s,l,
    TrackingCamera camT(1,19,93,55,255,215,255,true);

    // Loop until shutdown
    while(true)
    {
        // update cameras
        camO.update();
        camT.update();
        // Display for debugging
        imshow("cam 0", camO.frame);
        imshow("cam 1", camT.frame);

        // Setup socket server information
        char buf[20];
        int numbytes = 1;
        fd_set f;
        struct timeval xd;
        FD_ZERO(&f);
        FD_SET(socket_id, &f);
        numbytes = select(socket_id + 1, &f,NULL, NULL,&timeout);
        numbytes = recv(socket_id, buf, 20,0);

        // Receive data from Server to ensure connection is good
        if(numbytes > 0)
        {
            std::string str = std::to_string(camO.getAngle()) + "," + std::to_string(camT.getAngle());
            std::cout << str << "\n ";
            char byteStream[20];
            strcpy(byteStream, str.c_str());

            // Send X,Y,Size data to board
            status = send(socket_id, byteStream, sizeof(byteStream),0);
            std::cout << "status: " << status;
        }

        if(waitKey(20) >= 0)
            break;
    }

    return 0;
}

// Initialization funciton of class
TrackingCamera::TrackingCamera(int cameraNumber, double hl, double hh, double sl, double sh, double ll, double lh, bool deb)
{
    cam.open(cameraNumber);
    this->cameraNumber = cameraNumber;
    width = cam.get(CV_CAP_PROP_FRAME_WIDTH);
    height = cam.get(CV_CAP_PROP_FRAME_HEIGHT);
    fps = cam.get(CV_CAP_PROP_FPS);
    ss = Size(width,height);
    video = new VideoWriter("/media/ubuntu/Blade/" + std::to_string(cameraNumber) + ".avi",CV_FOURCC('M','J','P','G'),15, ss, true);
    h[0] = hl;
    h[1] = hh;
    s[0] = sl;
    s[1] = sh;
    l[0] = ll;
    l[1] = lh;
    debug = deb;
}

// Update function
// Ran every loop to update values
void TrackingCamera::update()
{
    // get frame
    cam >> frame;
    // save video
    video->write(frame);
    //adjust video frame
    cvtColor(frame, frame, CV_BGR2GRAY);
    threshold(frame,threshOut, 210,255,THRESH_BINARY);

    // find contours
    findContours(threshOut, mycontours);
    // filter contours
    filterContours(frame, mycontours, minArea,filterContoursMinPerimeter,filterContoursMinWidth,
                   filterContoursMaxWidth,filterContoursMinHeight,filterContoursMaxHeight,filterContoursSolidity,filterContoursMaxVertices,filterContoursMinVertices,filterContoursMinRatio,filterContoursMaxRatio,outcontours);
    if(this->outcontours.size() > 1)
    {
        int o, t;
        // sort with biggest object set
        sortWithBiggest(outcontours, o, t, cameraNumber);

        if(o == -1 || t == -1)
            return;
        // Set TargetX, Target Y, and draw circle to frame
        Rect b = boundingRect(outcontours[o]);
        Rect e = boundingRect(outcontours[t]);
        rectangle(this->frame, b.tl(), b.br(), Scalar(255,0,0),1,8,0);
        rectangle(this->frame, e.tl(), e.br(), Scalar(255,0,0),1,8,0);
        targetX = (b.tl().x + e.br().x) / 2;
        targetY = (b.tl().y + e.br().y) / 2;
        circle(this->frame, Point(targetX, targetY), 10, Scalar(0,255,0));
    }
}

// Find angle to object
double TrackingCamera::getAngle()
{
    // get angle to Target using trig
    if(targetX == 0) return 0;
    return (width / 2 - targetX - .5) * (hfov / width);
}



// Threshold using Hue, Saturation, and luminosity
void hslThreshold(Mat &input, double hue[],double sat[], double lum[], Mat &out)
{
    cvtColor(input, out, COLOR_BGR2HLS);
    inRange(out, Scalar(hue[0],lum[0],sat[0]), Scalar(hue[1],lum[1],sat[1]), out);
}

// findContours
void findContours(Mat &input, vector<vector<Point> > &contours)
{
    // 4d vector for contours
    vector<Vec4i> hierarchy;
    contours.clear();
    int method = CHAIN_APPROX_SIMPLE;
    // Find contours and write to referenced vector<vector<Point>>
    findContours(input, contours, hierarchy, false, method);
    // filter through contours
    for(vector<Point> contour: contours)
    {
        // Make bounding rect and draw it
        Rect bb = boundingRect(contour);
        rectangle(input, bb.tl(), bb.br(), Scalar(0,255,255),1,8,0);
    }
}

void filterContours(Mat &input, vector<vector<Point> > &inputContours, double minArea, double minPerimeter, double minWidth, double maxWidth, double minHeight, double maxHeight, double solidity[], double maxVertexCount, double minVertexCount, double minRatio, double maxRatio, vector<vector<Point> > &output) {
    vector<Point> hull;
    output.clear();
    int i =0;
    for (vector<Point> contour: inputContours) {
        Rect bb = boundingRect(contour);
        double area = contourArea(contour);
        if (area < minArea) continue;

        rectangle(input, bb.tl(), bb.br(), Scalar(0,0,255),1,8,0);
        output.push_back(contour);
    }
}

//
bool sortBySize(const vector<Point> &conO, const vector<Point> &conT)
{
    return contourArea(conO) < contourArea(conT);
}

// Sort using biggest shape as a hard set comparer
void sortWithBiggest(vector<vector<Point> > &contours, int &contIndexO, int &contIndexT, int camI)
{
    double closestMatch = 100;
    double biggestSize = 0;
    int biggestI = 0;
    // find index of biggest
    for(int i=0;i<contours.size(); i++){
	double newSize = contourArea(contours[i]);
	if(newSize > biggestSize)
	{
	    biggestSize = newSize;
    	    biggestI = i;
	}
    }

    // find closest match in size
    for(int j =0;j<contours.size();j++){
	if(j == biggestI) continue;
	double matchRating = std::abs((((contourArea(contours[biggestI]) - contourArea(contours[j]))/contourArea(contours[biggestI]))));
	if(matchRating < closestMatch)
	{
	    contIndexO = biggestI;
	    contIndexT = j;
	    closestMatch = matchRating;
	}
    }

}


// Sorts contours and finds best matching pair. Places index through reference
void sortContours(vector<vector<Point> > &contours, int &contIndexO,int &contIndexT, int camI)
{
    // init match rating at high number
    double closestMatch = 100;
    std::cout << "size: " << contours.size() << "\n";
    // compare all contours to find best match
    for (int i=0; i < contours.size(); i++) {
        for(int j=i+1;j< contours.size();j++){
            // looking for two objects with closest area together
            double matchRating = std::abs((((contourArea(contours[j]) - contourArea(contours[i]))/contourArea(contours[j]))));
	    std::cout << "rating: " << matchRating << "\n";

            // generate bounding rectangles for math purposes
	    Rect bb = boundingRect(contours[i]);
            Rect bc = boundingRect(contours[j]);

            int split = 0;
            // check distance based on which camera is inputted.
            if(camI == 0)
                split = abs(bb.y- bc.y);
            else if(camI == 1)
                split = abs(bb.x - bc.x);

            // only pick two shapes that are close to each other
            if((matchRating < closestMatch) && split > 4)
            {
                contIndexO = i;
                contIndexT = j;
                closestMatch = matchRating;
            }
        }
    }
    if(closestMatch == 100)
    {
        contIndexO = -1;
        contIndexT = -1;
    }
}
