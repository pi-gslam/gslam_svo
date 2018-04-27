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
#include <GSLAM/core/Event.h>

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

class SVO : public SLAM
{
public:
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
                cam_=SPtr<svo::AbstractCamera>(new svo::PinholeCamera(p[0],p[1],p[2],p[3],p[4],p[6]));
            }
            else if(camera.CameraType()=="OpenCV"){
                cam_=SPtr<svo::AbstractCamera>(new svo::PinholeCamera(p[0],p[1],p[2],p[3],p[4],p[6],
                        p[7],p[8],p[9],p[10],p[11]));
            }
            else return false;

            vo_=SPtr<svo::FrameHandlerMono>(new svo::FrameHandlerMono(cam_.get()));
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

            LOG(INFO) << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
                      << "#Features: " << vo_->lastNumObservations() << " \n";

            if(_handle){
                _handle->handle(new CurrentFrameEvent(FramePtr(new FrameSVO(vo_->lastFrame()))));
            }
            if(vo_->map().keyframes_.size()!=lastKFSize){
                updateMap();
                lastKFSize=vo_->map().keyframes_.size();
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

        setMap(_map);
        _handle->handle(_map);
        _handle->handle(lastKF);
    }

    SPtr<svo::AbstractCamera>    cam_;
    SPtr<svo::FrameHandlerMono>  vo_;
    int                          lastKFSize=-1;

    GObjectHandle*               _handle;
};

USE_GSLAM_PLUGIN(SVO);

}
