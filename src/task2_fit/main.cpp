#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <ceres/ceres.h>
using namespace cv;
using namespace std; 
struct CostFunctor{
     CostFunctor(double t,double omega_obs):t_(t), omega_obs_(omega_obs){};
     template <typename T>
     bool operator()(const T* const params,T* residual)const{
        T predicted_omega= params[1]+params[0]*sin(params[2]*T(t_)+params[3]);
        residual[0]=predicted_omega-T(omega_obs_);
        return true;
     }
     double t_;
     double omega_obs_;

};
int main(){
    vector<double> timeVec;
    vector<double> angleVec;
    double lastAngle= 0.0;
    double totalAngel=0.0;
    bool isFirstFrame= true;
    VideoCapture cap("resourses/task_2.mp4");
    if (!cap.isOpened()){
        cerr << "Cannot open video! Check the path."<<endl;
        return -1;
    }
    Mat frame;
    int frameCount = 0;
    VideoWriter video;//("result/task2_fit/tracking_overlay.mp4",VideoWriter::fourcc('m','p','4','v'),60.0,Size(frame.cols,frame.rows));
    bool isVideoOpened=false;
    while(true){
        cap >> frame;
        if(frame.empty()){
            break;
        }
        if (!isVideoOpened&&!frame.empty()){
            string videoPath="result/task2_fit/tracking_overlay.mp4";
            video.open(videoPath,VideoWriter::fourcc('m','p','4','v'),60.0,Size(frame.cols,frame.rows));
            if(!video.isOpened()){
                cerr<<"error"<<endl;
            }
            else{
                isVideoOpened=true;
                cout<<"开始录制"<<endl;
            }
        }
        Mat hsv, mask;
        cvtColor(frame,hsv, COLOR_BGR2HSV);
        inRange(hsv, Scalar(80,100,100), Scalar(100,255,255),mask);
        Mat kernel = getStructuringElement(MORPH_RECT,Size(5,5));
        morphologyEx(mask, mask , MORPH_OPEN, kernel);
        morphologyEx(mask, mask, MORPH_CLOSE, kernel);
        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        double maxArea= 0.0;
        Point2f targetCenter(-1,-1);
        for (size_t i=0;i<contours.size();i++){
            double area= contourArea(contours[i]);
            if(area>maxArea){
               maxArea=area;
               Moments m =moments(contours[i]);
               if(m.m00 != 0){
                  targetCenter.x = m.m10/m.m00;
                  targetCenter.y = m.m01/m.m00;
                }
            }        
        }
        if(targetCenter.x != -1 && targetCenter.y != -1 ){
            circle(frame, targetCenter, 10, Scalar(0,255,0), -1);
            Point centerPoint(480,360);
            circle(frame, centerPoint,5,Scalar(255,255,255),-1);
            line(frame, centerPoint,targetCenter,Scalar(0,255,255),2);
            double rawAngle = atan2(centerPoint.y - targetCenter.y,targetCenter.x- centerPoint.x);
            double diff=0.0;
            if (isFirstFrame){
                diff =rawAngle;
                isFirstFrame = false;
            }else{
                diff = rawAngle-lastAngle;
                if(diff> CV_PI) diff-=2.0*CV_PI;
                if(diff<-CV_PI) diff+=2.0*CV_PI;
            }
            totalAngel+=diff;
            lastAngle=rawAngle;
            double currentTime = frameCount/60.0;
            frameCount++;
            timeVec.push_back(currentTime);
            angleVec.push_back(totalAngel);
            cout<<"Time: "<<currentTime<<"s,Unwrapped Angle:"<<totalAngel<<" rad"<<endl;
        }
        if(isVideoOpened){
             video.write(frame);
        }
        
        imshow("Video Test", frame);
        if(waitKey(30)==27){
            break;
        }
    }
    video.release();
    cap.release();
    destroyAllWindows();
    vector<double>timeDotVec;
    vector<double>omegaVec;
     for (size_t i=0;i<timeVec.size()-1;i++){
        double dt =timeVec[i+1] - timeVec[i];
        if(dt>0.0001){
            double dtheta=angleVec[i+1] - angleVec[i];
            double omega=dtheta/dt;
            timeDotVec.push_back(timeVec[i+1]);
            omegaVec.push_back(omega);
        }
    }   
    vector<double> smoothOmega = omegaVec;
    int windows =21;
    for (size_t i=windows;i<omegaVec.size()-windows;i++){
        double sum=0.0;
        for(int j=-windows;j<=windows;j++){
            sum+=omegaVec[i+j];
        }
         smoothOmega[i]=sum/(2*windows+1);
    }
    double params[4]={0.55,1.35,1.65,0.70};
    ceres::Problem problem;
    for(size_t i=0;i<timeDotVec.size();i++){
        ceres::CostFunction* cost_function= new ceres::AutoDiffCostFunction<CostFunctor,1,4>(
            new CostFunctor(timeDotVec[i],smoothOmega[i])
        );
        problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.5), params);
    }
    ceres::Solver::Options options;
    options.linear_solver_type=ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem,&summary);
    cout << "\n=== 拟合结果 ===" <<endl;
    cout<< summary.BriefReport()<<endl;
    cout<< " A = "<<params[0]<<endl;
    cout<< " b = "<<params[1]<<endl;
    cout<< " Omega = "<<params[2]<<endl;
    cout<< " phi = "<<params[3]<<endl;
    
    int imgW =1200;
    int imgH = 600;
    Mat plotlmg(imgH,imgW, CV_8UC3, Scalar(255,255,255));
    double minY =0.0,maxY=0.0;
    for(size_t i=0;i < omegaVec.size();i++){
        if(omegaVec[i]>maxY) maxY=omegaVec[i];
        if(omegaVec[i]<minY) minY=omegaVec[i];
    }
    for(size_t i=0;i < omegaVec.size();i++){
        double x1 =timeDotVec[i]/24*imgW;
        double y1=imgH-(smoothOmega[i]-minY)/(maxY-minY)*imgH;
        circle(plotlmg, Point(x1,y1),2,Scalar(255,0,0),-1);
        double pred=params[1]+params[0]*sin(params[2]*timeDotVec[i]+params[3]);
        double y2=imgH-(pred-minY)/(maxY-minY)*imgH;
        if (i>0){
            double prevX =timeDotVec[i-1]/24.0*imgW;
            double prevPRED=params[1]+params[0]*sin(params[2]*timeDotVec[i-1]+params[3]);
            double prevY= imgH-(prevPRED-minY)/(maxY-minY)*imgH;      
            line (plotlmg,Point(prevX,prevY),Point(x1,y2),Scalar(0,0,255),1) ;  
        }
    }
    imwrite("result/task2_fit/fit_comparison.png",plotlmg);
    Mat velocitylmg(imgH,imgW,CV_8UC3,Scalar(255,255,255));
    for(size_t i=0;i<timeDotVec.size();i++){
        double pred=params[1]+params[0]*sin(params[2]*timeDotVec[i]+params[3]);
        double x=timeDotVec[i]/24.0*imgW;
        double y=imgH-(pred-minY)/(maxY-minY+1e-6)*imgH;
        if (i>0){
            double prevX =timeDotVec[i-1]/24.0*imgW;
            double prevPRED=params[1]+params[0]*sin(params[2]*timeDotVec[i-1]+params[3]);
            double prevY= imgH-(prevPRED-minY)/(maxY-minY)*imgH;      
            line (velocitylmg,Point(prevX,prevY),Point(x,y),Scalar(0,0,255),1) ;  
        }
    }
    imwrite("result/task2_fit/angular_velocity.png",velocitylmg);
    Mat residuallmg(imgH,imgW,CV_8UC3, Scalar(255,255,255));
    vector<double> residuals;
    double maxRes=0.0;
    for(size_t i=0;i<omegaVec.size();i++){
        double pred=params[1]+params[0]*sin(params[2]*timeDotVec[i]+params[3]);
        double res=pred-smoothOmega[i];
        residuals.push_back(res);
        if(fabs(res)>maxRes) maxRes=fabs(res);
    }
    if(maxRes==0) maxRes=1.0;    
    for(size_t i=0;i<residuals.size();i++){
        double x=timeDotVec[i]/24.0*imgW;
        double y=imgH/2.0-residuals[i]/maxRes*(imgH/2.0-50);
        circle(residuallmg,Point(x,y),2,Scalar(0,0,255),-1);
    }
    line(residuallmg,Point(0,imgH/2),Point(imgW,imgH/2),Scalar(0,0,0),1);
    imwrite("result/task2_fit/residuals.png",residuallmg);
    double sse=0.0;
    for(size_t i=0;i<omegaVec.size();i++){
        double pred=params[1]+params[0]*sin(params[2]*timeDotVec[i]+params[3]);
        double err=pred-smoothOmega[i];
        sse+=err*err;
    }
    double rmse=sqrt(sse/omegaVec.size());
    cout<<"RMSE ="<<rmse<<" rad"<<endl;
    return 0;}
