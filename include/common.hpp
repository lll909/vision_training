#pragma once
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <limits>
#include <string>
namespace fs = std::filesystem;
constexpr double PI = 3.14159265358979323846;
inline double wrap(double a) { return a - 2*PI*std::floor((a+PI)/(2*PI)); }
inline void require(bool ok, const std::string& msg) {
    if (!ok) throw std::runtime_error(msg);
}
inline void saveImage(const fs::path& p, const cv::Mat& im) {
    require(cv::imwrite(p.string(), im), "Cannot save " + p.string());
}
inline cv::VideoWriter writer(const fs::path& p, double fps, cv::Size size) {
    cv::VideoWriter w(p.string(), cv::VideoWriter::fourcc('m','p','4','v'), fps, size);
    require(w.isOpened(), "Cannot open video writer: " + p.string());
    return w;
}
inline void label(cv::Mat& im, const std::string& s, int y=30,
                  cv::Scalar color=cv::Scalar(0,255,255)) {
    cv::putText(im,s,{10,y},cv::FONT_HERSHEY_SIMPLEX,0.65,{0,0,0},4);
    cv::putText(im,s,{10,y},cv::FONT_HERSHEY_SIMPLEX,0.65,color,1);
}
// Simple headless plotter. Data, axis labels and units are saved together.
inline void plot(const fs::path& file, const std::vector<double>& t,
                 const std::vector<double>& observed,
                 const std::vector<double>& predicted,
                 const std::string& title, const std::string& unit) {
    require(t.size()>1 && t.size()==observed.size(), "Invalid plot data");
    require(predicted.empty() || predicted.size()==t.size(), "Plot length mismatch");
    double lo=*std::min_element(observed.begin(),observed.end());
    double hi=*std::max_element(observed.begin(),observed.end());
    for(double v:predicted) { lo=std::min(lo,v); hi=std::max(hi,v); }
    double pad=std::max((hi-lo)*0.08,1e-6); lo-=pad; hi+=pad;
    double t0=t.front(), t1=t.back();
    require(t1>t0,"Invalid time range");
    cv::Mat canvas(720,1200,CV_8UC3,cv::Scalar(255,255,255));
    auto point=[&](double x,double y) {
        return cv::Point(110+int((x-t0)/(t1-t0)*1030),620-int((y-lo)/(hi-lo)*520));
    };
    for(int i=0;i<=5;++i) {
        double x=t0+(t1-t0)*i/5.0, y=lo+(hi-lo)*i/5.0;
        cv::line(canvas,{110,620-i*104},{1140,620-i*104},{220,220,220});
        cv::putText(canvas,cv::format("%.3g",y),{10,625-i*104},0,0.5,{0,0,0});
        cv::putText(canvas,cv::format("%.2f",x),{95+i*206,648},0,0.5,{0,0,0});
    }
    cv::rectangle(canvas,{110,100},{1140,620},{0,0,0});
    if(!predicted.empty()) for(size_t i=1;i<t.size();++i)
        cv::line(canvas,point(t[i-1],predicted[i-1]),point(t[i],predicted[i]),{0,0,220},2);
    for(size_t i=0;i<t.size();++i) cv::circle(canvas,point(t[i],observed[i]),2,{220,80,0},-1);
    label(canvas,title+" | y: "+unit,32,{0,0,0});
    label(canvas,predicted.empty()?"blue: samples":"blue: observation, red: fit",64,{0,0,0});
    cv::putText(canvas,"time (s)",{540,690},0,0.7,{0,0,0});
    saveImage(file,canvas);
}