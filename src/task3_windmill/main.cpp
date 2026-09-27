#include "common.hpp"
// Calibrated teaching baseline: circle geometry + internal holes + a lit radial
// connector distinguish bullseye targets from other illuminated blades.
struct Candidate { cv::Point2f center; float circleRadius; double angle,radius; };
struct Config {
    int h1_low=0,h1_high=15,h2_low=165,h2_high=179,s_min=70,v_min=90,kernel=3;
    double area_min=40,area_max=12000,aspect_min=.45,circularity_min=.2,radius_min=60,radius_max=600;
    int center_search=100,lost_tolerance=8,init_frame=0,center_x=0,center_y=0,center_w=0,center_h=0;
    double center_match_min=.65,angle_gate=.3,radius_gate=.25;
    int template_frame=-1,min_holes=0,arrow_half_width=6,arrow_v_min=100,center_global_reacquire=0;
    double hole_area_ratio=.015,arrow_fill_min=0;
    void load(const std::string& file) {
        cv::FileStorage f(file,cv::FileStorage::READ); require(f.isOpened(),"Cannot open config");
#define READ(x) if(!f[#x].empty()) f[#x] >> x
        READ(h1_low); READ(h1_high); READ(h2_low); READ(h2_high); READ(s_min); READ(v_min); READ(kernel);
        READ(area_min); READ(area_max); READ(aspect_min); READ(circularity_min); READ(radius_min); READ(radius_max);
        READ(center_search); READ(lost_tolerance); READ(init_frame); READ(center_x); READ(center_y); READ(center_w); READ(center_h);
        READ(center_match_min); READ(angle_gate); READ(radius_gate);
        READ(template_frame); READ(min_holes); READ(arrow_half_width); READ(arrow_v_min);
        READ(center_global_reacquire); READ(hole_area_ratio); READ(arrow_fill_min);
#undef READ
        require(kernel>0&&kernel%2==1&&lost_tolerance>=1&&init_frame>=0&&center_search>0,"Invalid config");
        require(angle_gate>0&&angle_gate<PI&&radius_gate>0&&radius_min>=0&&radius_max>radius_min,"Invalid gates");
        require(center_match_min>=-1&&center_match_min<=1&&area_max>area_min&&area_min>=0,"Invalid thresholds");
        require(template_frame>=-1&&min_holes>=0&&arrow_half_width>=0&&hole_area_ratio>=0&&arrow_fill_min>=0&&arrow_fill_min<=1,"Invalid shape configuration");
    }
};
int main(int argc,char** argv) try {
    require(argc>=3,"Usage: task3_windmill INPUT CONFIG [output_name] [--preview]");
    Config cfg; cfg.load(argv[2]);
    std::cout << "TASK3_FIXED_20260927 config=" << argv[2] << " H=" << cfg.h1_low << ".." << cfg.h1_high
              << " S>=" << cfg.s_min << " V>=" << cfg.v_min << " lost_tolerance=" << cfg.lost_tolerance << "\n";
    std::string name=argc>3?argv[3]:fs::path(argv[1]).stem().string();
    require(name=="task_3"||name=="task_4","output_name must be task_3 or task_4");
    bool preview=argc>4&&std::string(argv[4])=="--preview";
    fs::path out=fs::path("result/task3_windmill")/name; fs::create_directories(out);
    cv::VideoCapture cap(argv[1]); require(cap.isOpened(),"Cannot open input");
    double fps=cap.get(cv::CAP_PROP_FPS); require(std::isfinite(fps)&&fps>0,"Invalid FPS");
    // Decode to an initialization frame without relying on imprecise frame seeking.
    cv::Mat initial;
    int templateFrame=cfg.template_frame>=0?cfg.template_frame:cfg.init_frame;
    for(int i=0;i<=templateFrame;++i) require(cap.read(initial),"Template frame beyond end");
    cv::Rect roi(cfg.center_x,cfg.center_y,cfg.center_w,cfg.center_h);
    if(roi.width<=0||roi.height<=0) {
        std::cout<<"Select a tight, symmetric ROI around the R mark; Enter to confirm.\n";
        roi=cv::selectROI("Select R mark",initial,false,false); cv::destroyAllWindows();
    }
    require(roi.width>=5&&roi.height>=5&&(roi&cv::Rect(0,0,initial.cols,initial.rows))==roi,"Invalid center ROI");
    cv::Mat initialGray; cv::cvtColor(initial,initialGray,cv::COLOR_BGR2GRAY);
    cv::Mat templ=initialGray(roi).clone(); cv::Scalar mean,stddev; cv::meanStdDev(templ,mean,stddev);
    require(stddev[0]>5,"R template has too little texture; select a tighter textured ROI");
    saveImage(out/"center_template.png",templ);
    std::ofstream init(out/"initialization.txt");
    init<<"init_frame: "<<cfg.init_frame<<"\ntemplate_frame: "<<templateFrame<<"\ncenter_x: "<<roi.x<<"\ncenter_y: "<<roi.y
        <<"\ncenter_w: "<<roi.width<<"\ncenter_h: "<<roi.height<<"\n"; init.close();
    fs::copy_file(argv[2],out/"used_config.yaml",fs::copy_options::overwrite_existing);
    cap.release(); cap.open(argv[1]); require(cap.isOpened(),"Cannot reopen input");
    auto overlay=writer(out/"recognition_overlay.mp4",fps,initial.size());
    auto binary=writer(out/"binary_process.mp4",fps,initial.size());
    std::ofstream log(out/"tracking.csv");
    require(log.is_open(),"Cannot open tracking.csv");
    log<<"frame,id,state,center_valid,center_score,candidates,cx,cy,tx,ty\n";
    int frameId=0,id=0,nextId=1,missed=0,detectedCount=0,centerCount=0,switches=0;
    double lastAngle=0,lastRadius=0,angularStep=0; int lastSeen=-1;
    cv::Mat frame;
    while(cap.read(frame)) {
        require(frame.size()==initial.size(),"Frame resolution changed");
        cv::Mat gray,hsv,m1,m2,mask; cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);
        cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);
        cv::inRange(hsv,cv::Scalar(cfg.h1_low,cfg.s_min,cfg.v_min),cv::Scalar(cfg.h1_high,255,255),m1);
        cv::inRange(hsv,cv::Scalar(cfg.h2_low,cfg.s_min,cfg.v_min),cv::Scalar(cfg.h2_high,255,255),m2);
        cv::bitwise_or(m1,m2,mask);
        auto kernel=cv::getStructuringElement(cv::MORPH_RECT,{cfg.kernel,cfg.kernel});
        cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,kernel);
        bool centerOK=false; double score=-1; cv::Point2f center;
        if(frameId>=cfg.init_frame) {
            int margin=cfg.center_search;
            cv::Rect search(roi.x-margin,roi.y-margin,roi.width+2*margin,roi.height+2*margin);
            search &= cv::Rect(0,0,gray.cols,gray.rows);
            cv::Mat response; cv::matchTemplate(gray(search),templ,response,cv::TM_CCOEFF_NORMED);
            cv::Point loc; cv::minMaxLoc(response,nullptr,&score,nullptr,&loc);
            centerOK=std::isfinite(score)&&score>=cfg.center_match_min;
            if(!centerOK&&cfg.center_global_reacquire) {
                search=cv::Rect(0,0,gray.cols,gray.rows);
                cv::matchTemplate(gray,templ,response,cv::TM_CCOEFF_NORMED);
                cv::minMaxLoc(response,nullptr,&score,nullptr,&loc);
                centerOK=std::isfinite(score)&&score>=cfg.center_match_min;
            }
            if(centerOK) {
                roi.x=search.x+loc.x; roi.y=search.y+loc.y;
                center=cv::Point2f(roi.x+roi.width/2.f,roi.y+roi.height/2.f); ++centerCount;
            }
        }
        std::vector<Candidate> candidates;
        if(centerOK) {
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(mask,contours,hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
            for(size_t ci=0;ci<contours.size();++ci) {
                if(hierarchy[ci][3]>=0) continue; // no duplicate inner-boundary candidates
                const auto& c=contours[ci];
                double a=cv::contourArea(c),p=cv::arcLength(c,true);
                if(a<cfg.area_min||a>cfg.area_max||p<=0) continue;
                cv::Rect b=cv::boundingRect(c);
                double aspect=double(std::min(b.width,b.height))/std::max(b.width,b.height);
                if(aspect<cfg.aspect_min||4*PI*a/(p*p)<cfg.circularity_min) continue;
                int holes=0;
                for(int child=hierarchy[ci][2];child>=0;child=hierarchy[child][0])
                    if(cv::contourArea(contours[child])>a*cfg.hole_area_ratio) ++holes;
                if(holes<cfg.min_holes) continue;
                cv::Point2f pt; float rr; cv::minEnclosingCircle(c,pt,rr);
                double radius=cv::norm(pt-center); if(radius<cfg.radius_min||radius>cfg.radius_max) continue;
                if(cfg.arrow_fill_min>0) {
                    cv::Point2f direction=(pt-center)*(1.0/radius);
                    cv::Point2f normal(-direction.y,direction.x);
                    int hit=0,total=0;
                    for(int d=int(.25*radius);d<int(.78*radius);++d) {
                        bool lit=false;
                        for(int n=-cfg.arrow_half_width;n<=cfg.arrow_half_width;++n) {
                            cv::Point2f q=center+direction*float(d)+normal*float(n);
                            int x=int(std::lround(q.x)),y=int(std::lround(q.y));
                            if(x<0||x>=hsv.cols||y<0||y>=hsv.rows) continue;
                            cv::Vec3b v=hsv.at<cv::Vec3b>(y,x);
                            bool hue=(v[0]>=cfg.h1_low&&v[0]<=cfg.h1_high)||(v[0]>=cfg.h2_low&&v[0]<=cfg.h2_high);
                            if(hue&&v[1]>=cfg.s_min&&v[2]>=cfg.arrow_v_min) { lit=true; break; }
                        }
                        ++total; if(lit) ++hit;
                    }
                    if(total==0||double(hit)/total<cfg.arrow_fill_min) continue;
                }
                double angle=std::atan2(center.y-pt.y,pt.x-center.x);
                candidates.push_back({pt,rr,angle,radius});
            }
        }
        int selected=-1; bool fresh=false;
        if(id!=0&&centerOK) {
            int gap=frameId-lastSeen;
            double prediction=lastAngle+angularStep*gap,best=std::numeric_limits<double>::infinity();
            // Bounded gate avoids swallowing a neighboring blade after a long gap.
            double gate=std::min(cfg.angle_gate*(1+.1*std::max(0,gap-1)),.55);
            for(size_t i=0;i<candidates.size();++i) {
                double da=std::abs(wrap(candidates[i].angle-prediction));
                double dr=std::abs(candidates[i].radius-lastRadius)/std::max(lastRadius,1.0);
                if(da>gate||dr>cfg.radius_gate) continue;
                double cost=da/gate+dr/cfg.radius_gate;
                if(cost<best) { best=cost; selected=int(i); }
            }
        }
        if(id==0&&!candidates.empty()) {
            // Deterministic initial selection. Contour index is NEVER the target ID.
            selected=int(std::min_element(candidates.begin(),candidates.end(),[](const Candidate& a,const Candidate& b){
                return a.center.x<b.center.x;
            })-candidates.begin());
            id=nextId++; fresh=true; if(id>1) ++switches;
        }
        std::string state="lost"; cv::Point2f target;
        if(selected>=0) {
            const auto& c=candidates[selected]; target=c.center; state="detected"; ++detectedCount;
            if(fresh) angularStep=0;
            else {
                double measured=wrap(c.angle-lastAngle)/std::max(1,frameId-lastSeen);
                angularStep=.5*angularStep+.5*measured;
            }
            lastAngle=c.angle; lastRadius=c.radius; lastSeen=frameId; missed=0;
            cv::circle(frame,target,int(std::round(c.circleRadius)),{0,255,0},2);
            cv::circle(frame,target,4,{0,255,0},-1); cv::line(frame,center,target,{0,255,0},2);
        } else if(id!=0) {
            ++missed; if(missed>=cfg.lost_tolerance) { id=0; angularStep=0; }
        }
        if(centerOK) { cv::circle(frame,center,5,{0,255,255},2); cv::rectangle(frame,roi,{0,255,255},1); }
        label(frame,"ID="+std::to_string(id)+" "+state+" frame="+std::to_string(frameId),30,
              state=="detected"?cv::Scalar(0,255,0):cv::Scalar(0,0,255));
        label(frame,"center="+std::string(centerOK?"detected":"lost")+" candidates="+std::to_string(candidates.size()),58);
        // Blue small circles expose candidate false positives in the overlay.
        for(const auto& c:candidates) cv::circle(frame,c.center,2,{255,0,0},-1);
        cv::Mat maskBgr; cv::cvtColor(mask,maskBgr,cv::COLOR_GRAY2BGR);
        overlay.write(frame); binary.write(maskBgr);
        log<<frameId<<","<<id<<","<<state<<","<<centerOK<<","<<score<<","<<candidates.size()<<",";
        if(centerOK) log<<center.x<<","<<center.y; else log<<"nan,nan";
        if(selected>=0) log<<","<<target.x<<","<<target.y; else log<<",nan,nan";
        log<<"\n"; ++frameId;
        if(preview) { cv::imshow("tracking",frame); cv::imshow("mask",mask); if(cv::waitKey(1)==27) break; }
    }
    overlay.release(); binary.release(); cv::destroyAllWindows();
    std::ofstream metrics(out/"metrics.md");
    metrics<<"# Run statistics\n\n- Input: "<<argv[1]<<"\n- FPS: "<<fps
           <<"\n- Dimensions: "<<initial.cols<<" x "<<initial.rows<<"\n- Written frames: "<<frameId
           <<"\n- Detected frames: "<<detectedCount<<"\n- Center observed frames: "<<centerCount
           <<"\n- Reselections: "<<switches<<"\n- Preview mode: "<<preview
           <<"\n\nDetection fraction is NOT accuracy. Inspect identity and geometry manually.\n";
    std::cout<<"Written "<<frameId<<" frames. Inspect output before claiming completion.\n";
    return 0;
} catch(const std::exception& e) { std::cerr<<e.what()<<"\n"; return 1; }