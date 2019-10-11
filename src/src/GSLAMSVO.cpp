#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <vector>
#include <string>
#include <svo/math_lib.h>
#include <svo/camera_model.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <iostream>
#include <svo/point.h>
#include <svo/feature.h>

#include <svo/slamviewer.h>
#include <thread>
#include <GSLAM/core/GSLAM.h>
#include <GSLAM/core/HashMap.h>

namespace GSLAM {

class FrameSVO : public MapFrame{
public:
    FrameSVO(svo::FramePtr fr): MapFrame(fr->id_,fr->timestamp_){
        Sophus::SE3 se3=fr->T_f_w_.inverse();
        auto coef=se3.so3().unit_quaternion().coeffs();
        setPose(SE3(SO3(coef[0],coef[1],coef[2],coef[3]),*(Point3d*)&se3.translation()));
    }
};

class PointSVO : public MapPoint{
public:
    PointSVO(svo::Point* pt): MapPoint(pt->id_,pt->pos_){

    }
};

class SVO
{
public:
    SVO(Svar config):_config(config){
        _pubFrame=messenger.advertise<FramePtr>("svo/curframe");
        _pubMap=messenger.advertise<MapPtr>("svo/map");
        _subDataset=messenger.subscribe("dataset/frame",5,[this](FramePtr fr){this->track(fr);});
    }
    virtual std::string type()const{return "SVO";}
    virtual bool valid()const{return true;}
    virtual bool isDrawable()const{return false;}

    virtual bool    track(FramePtr& frame){
        if(frame->cameraNum()<1) return false;
        if(!vo_)
        {
            GSLAM::Camera camera=frame->getCamera();
            auto p=camera.getParameters();
            if(camera.CameraType()=="PinHole")
            {
                cam_=std::shared_ptr<svo::AbstractCamera>(new svo::PinholeCamera(p[0],p[1],p[2],p[3],p[4],p[6]));
            }
            else if(camera.CameraType()=="OpenCV"){
                cam_=std::shared_ptr<svo::AbstractCamera>(new svo::PinholeCamera(p[0],p[1],p[2],p[3],p[4],p[6],
                        p[7],p[8],p[9],p[10],p[11]));
            }
            else return false;

            vo_=std::shared_ptr<svo::FrameHandlerMono>(new svo::FrameHandlerMono(cam_.get()));
            vo_->start();
        }

        cv::Mat image=frame->getImage();
        if(image.type()==CV_8UC3)
        {
            cv::cvtColor(image,image,CV_BGR2GRAY);
        }
        else if(image.type()==CV_8UC4){
            cv::cvtColor(image,image,CV_BGRA2GRAY);
        }

        vo_->addImage(image, frame->timestamp());

        if(vo_->lastFrame() != NULL)
        {

            LOG_IF(INFO,_config.GetInt("Verbose")) << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
                      << "#Features: " << vo_->lastNumObservations() << " \n";

            if(_pubFrame.getNumSubscribers()){
                FramePtr lastFr(new FrameSVO(vo_->lastFrame()));
                frame->setPose(lastFr->getPose());
                _pubFrame.publish(lastFr);
            }
            svo::FramePtr lastKF=vo_->map().keyframes_.back();
            if(lastKF->id_!=lastKFSize){
                updateMap();
                lastKFSize=lastKF->id_;
            }
        }
        return true;
    }

    virtual bool    setCallback(GObjectHandle* cbk){_handle=cbk;return true;}

    void updateMap(){
        const svo::Map& svoMap=vo_->map();
        MapPtr _map(new HashMap());
        auto kfs=svoMap.keyframes_;
        FramePtr lastKF;
        for(auto kf:kfs){
            lastKF=FramePtr(new FrameSVO(kf));
            _map->insertMapFrame(lastKF);
        }

        std::vector< std::pair<svo::FramePtr,size_t> > overlap_kfs = vo_->overlap_kfs();
        auto  it_frame=overlap_kfs.begin();
        for(size_t i = 0; i<overlap_kfs.size() ; ++i, ++it_frame)
        {
                svo::FramePtr  frame = it_frame->first;
                for(svo::Features::iterator it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
                {
                    if((*it)->point == NULL)
                      continue;
                    _map->insertMapPoint(PointPtr(new PointSVO((*it)->point)));
                }
        }

        svo::FramePtr lastframe = vo_->lastFrame();
        for(svo::Features::iterator it=lastframe->fts_.begin(); it!=lastframe->fts_.end(); ++it)
        {

            if((*it)->point == NULL)
              continue;
            _map->insertMapPoint(PointPtr(new PointSVO((*it)->point)));
        }

        _pubMap.publish(_map);
        LOG(INFO)<<"Published map with "<<_map->frameNum()<<" frames.";
    }

    std::shared_ptr<svo::AbstractCamera>    cam_;
    std::shared_ptr<svo::FrameHandlerMono>  vo_;
    int                          lastKFSize=-1;

    GObjectHandle*               _handle;
    Svar _config;
    Subscriber _subDataset;
    Publisher  _pubFrame,_pubMap;
};

int runsvo(Svar config){
    config.arg<bool>("svo.verbose",false,"Show log of svo");

    if(config.get("help",false)) {
        auto subDataset=messenger.subscribe("dataset/frame",[](FramePtr fr){});
        auto pubFrame  =messenger.advertise<FramePtr>("svo/curframe");
        auto pubMap    =messenger.advertise<MapPtr>("svo/map");
        config["__usage__"]=messenger.introduction();
        return config.help();
    }

    SVO svo(config);
    return Messenger::exec();
}

GSLAM_REGISTER_APPLICATION(svo,runsvo);

}
