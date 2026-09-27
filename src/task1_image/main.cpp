#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
using namespace std;
using namespace cv;
int main(){
    //part 1
    Mat img = imread("resourses/test_image.jpg");
    if (img.empty()) {
        cerr << "Cannot read image! please check path" << endl;
        return -1;
    }
    Mat gray, hsv;
    cvtColor(img, gray, COLOR_BGR2GRAY);
    cvtColor(img, hsv, COLOR_BGR2HSV);
    vector<Mat> hsvChannels;
    split(hsv, hsvChannels);
    Mat h_channel = hsvChannels[0];
    Mat s_channel = hsvChannels[1];
    Mat v_channel = hsvChannels[2];
    Mat meanlmg, gaussianlmg, medianlmg;
    blur(img, meanlmg, Size(5,5));
    GaussianBlur(img, gaussianlmg, Size(5,5), 1.5);
    medianBlur(img, medianlmg, 5);
    imwrite("result/task1_images/gray.png", gray);
    imwrite("result/task1_images/hsv_h.png", h_channel);
    imwrite("result/task1_images/hsv_s.png", s_channel );
    imwrite("result/task1_images/hsv_v.png", v_channel);
    imwrite("result/task1_images/mean_filter.png",meanlmg );
    imwrite("result/task1_images/gaussian.png",gaussianlmg );
    imwrite("result/task1_images/median_filter.png",medianlmg );
    //part 2
    Mat maskLow, maskHigh, mask;
    inRange(hsv, Scalar(0,100,100),Scalar(10,255,255),maskLow);
    inRange(hsv, Scalar(170,100,100),Scalar(179,255,255),maskHigh);
    bitwise_or(maskLow, maskHigh, mask);
    Mat kernel =getStructuringElement(MORPH_RECT, Size(5,5));
    Mat opened,closed;
    morphologyEx(mask,opened,MORPH_OPEN,kernel);
    morphologyEx(mask,closed,MORPH_CLOSE,kernel);
    imwrite("result/task1_images/red_mask.png", mask);
    imwrite("result/task1_images/open.png", opened);
    imwrite("result/task1_images/close_mask.png", closed);
    Mat eroded, dilated;
    erode(mask, eroded,kernel);
    erode(mask,dilated,kernel);
    imwrite("result/task1_images/erode.png",eroded);
    imwrite("result/task1_images/dilate.png",dilated);
    //part 3
    vector<vector<Point>> contours;
    findContours(closed, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    Mat result = img.clone();
    for (size_t i = 0; i <contours.size();i++){
        double area =contourArea(contours[i]);
        if(area<500.0) continue;
        Rect box = boundingRect(contours[i]);
        rectangle(result, box, Scalar(0,0,255), 2);
        drawContours(result, contours, static_cast<int>(i),Scalar(0,255,0),2);
        string areaText = "Area: " + to_string(static_cast<int>(area));
        putText(result, areaText, Point(box.x,box.y - 5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1); 
    }
    imwrite("result/task1_images/contours_boxes.png", result);
    //part 4
    Mat drawlmg = img.clone();
    circle(drawlmg,Point(100,100),50, Scalar(255,0,0),3);
    rectangle(drawlmg,Rect(200,200,150,100),Scalar(0,255,255),3);
    putText(drawlmg,"Hello OpenCV", Point(50,400), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255,255,255),2);
    imwrite("result/task1_images/drawing.png", drawlmg);
    Point2f center(img.cols / 2.0, img.rows / 2.0);
    Mat rotationMatrix = getRotationMatrix2D(center, 35.0, 1.0);
    Mat rotatedlmg;
    warpAffine(img, rotatedlmg,rotationMatrix,img.size());
    imwrite("result/task1_images/rotated_35deg.png", rotatedlmg);
    Rect cropRect(0, 0, img.cols/2 , img.rows /2);
    Mat croppedlmg=img(cropRect).clone();
    imwrite("result/task1_images/crop_top_left.png", croppedlmg);
    cout<<"task1 is completed perfectly!"<<endl;
    return 0;
}