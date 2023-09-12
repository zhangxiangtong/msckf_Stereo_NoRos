#include"System.h"
#include <sstream>
#include<fstream>
#include<chrono>
#include <ctime>
#include<opencv2/imgproc/imgproc.hpp>
#include<thread>

namespace msckf_vio
{
    System::System(string dataSetConfig,string paramconfigfile)
    {
        this->mpVioDrawer = new VioDrawer();
        if(this->mpVioDrawer == NULL)
        {
            std::cout<<"Cannot initialize VioDrawer... !"<<std::endl;
            return;
        }
        this->mpImageProcessor = new ImageProcessor(dataSetConfig,paramconfigfile);
        this->mpMsckfVio       = new MsckfVio(dataSetConfig,paramconfigfile,this->mpVioDrawer);
        if(this->mpImageProcessor == NULL)
        {
            std::cout<<"Malloc for ImageProcessor failed !"<<std::endl;
            return;
        }
        if(this->mpMsckfVio == NULL)
        {
            std::cout<<"Malloc for MsckfVio failed !"<<std::endl;
            return;
        }

        if(this->mpImageProcessor)
        {
            if(!this->mpImageProcessor->initialize(dataSetConfig,paramconfigfile))
            {
                std::cout<<"Cannot initialize Image Processor... !"<<std::endl;
            }
            
        }
        if(this->mpMsckfVio)
        {
            if(!this->mpMsckfVio->initialize(dataSetConfig,paramconfigfile))
            {
                std::cout<<"Cannot initialize Image Processor... !"<<std::endl;
            }
            
        }



        this->mpViewer = new Viewer(this->mpVioDrawer);
        if(this->mpViewer == NULL)
        {
            std::cout<<"Cannot initialize Viewer... !"<<std::endl;
            return;
        }


        this->mnPreImuIndex = 0;
        this->mbFinished    = false;
        this->mpthMsckf     = NULL;
        this->mpthView      = NULL;
        this->mpthDataProvider = NULL;
    }

    void System::Run(string datasetpath)
    {
        //创建各个处理线程
        this->mpthMsckf = new std::thread(&MsckfLoop, this);

        this->mpthView = new std::thread(&Viewer::Run, this->mpViewer);
        
        this->mpthDataProvider = new std::thread(&ReadSensorData, this,datasetpath);

        this->mpthMsckf->join();
        this->mpthView->join();
        this->mpthDataProvider->join();
        
    }
    System::~System()
    {
        if(this->mpImageProcessor)
        {
            delete this->mpImageProcessor;
            this->mpImageProcessor = NULL;
        }
        if(this->mpMsckfVio)
        {
            delete this->mpMsckfVio;
            this->mpMsckfVio = NULL;
        }

        if(this->mpthMsckf)
        {
            delete this->mpthMsckf;
            this->mpthMsckf = NULL;
        }
        if(this->mpthView)
        {
            delete this->mpthView;
            this->mpthView = NULL;
        }

        if(this->mpthDataProvider)
        {
            delete this->mpthDataProvider;
            this->mpthDataProvider = NULL;
        }
    }

    int System::LoadStereoImageList_Euroc(string filepath,vector<double> &vTimeStamp,vector<string> &vImageFilePathList)
    {
        vTimeStamp.clear();
        vImageFilePathList.clear();
        std::ifstream filestream(filepath);
        if (!filestream.is_open()) 
        {
           return 0;
        }

        uint64_t stamp;
        std::string filename;

        std::string line;
        char temp;

        while (getline(filestream, line)) 
        {
            if(line.find("#") != std::string::npos) 
            {
                continue;
            }

            std::stringstream ss(line);
            ss >> stamp  >> filename;

            vTimeStamp.push_back(1.0 * stamp/1e9);

            vImageFilePathList.push_back(filename.substr(1,filename.length() - 1));
        }
        filestream.close();

        return vTimeStamp.size();
    }

    int System::LoadImuData(string filepath,vector<IMU> &vImus)
    {
       ifstream fImu;
       fImu.open(filepath.c_str());
       if(!fImu.is_open())
       {
           return 0;
       }

       vImus.reserve(5000);

       while(!fImu.eof())
       {
            string s;
            getline(fImu,s);
            if (s[0] == '#')
            {
               continue;
            }


            if(!s.empty())
            {
                string item;
                size_t pos = 0;
                double data[7];
                int count = 0;
                while ((pos = s.find(',')) != string::npos) 
                {
                    item = s.substr(0, pos);
                    data[count++] = stod(item);
                    s.erase(0, pos + 1);
                }
                item = s.substr(0, pos);
                data[6] = stod(item);

                IMU imu;
                imu.stamp = data[0]/1e9;
                imu.angular_velocity[0] = data[1];
                imu.angular_velocity[1] = data[2];
                imu.angular_velocity[2] = data[3];
                imu.linear_acceleration[0] = data[4];
                imu.linear_acceleration[1] = data[5];
                imu.linear_acceleration[2] = data[6];
                vImus.push_back(imu);
            }
       }

       return vImus.size();
    }

    void System::ReadSensorData(string datasetPath)
    {
        vector<double> vTimeStamp;
        vector<string> vImageFilePathList;
        vector<IMU> vImus;
        int imageNum = LoadStereoImageList_Euroc(datasetPath + "/cam0/data.csv",vTimeStamp,vImageFilePathList);
        int imuNum = LoadImuData(datasetPath + "/imu0/data.csv",vImus);
        if(imageNum == 0 || imuNum == 0)
        {
            printf("there is no image data or imu data\n");
            return;
        }

        int startId = 0;
        for(int id = startId; id < imageNum;id++)
        {
            while(IsMsckfBusy())
            {
                usleep(50 * 1e3); 
            }
            std::cout<<"imgId = "<<id<<std::endl;

            cv::Mat leftImg = cv::imread(datasetPath + "/cam0/data/" + vImageFilePathList[id], cv::IMREAD_UNCHANGED);
            cv::Mat rightImg = cv::imread(datasetPath + "/cam1/data/" + vImageFilePathList[id], cv::IMREAD_UNCHANGED);

            if(leftImg.empty() || rightImg.empty())
            {
                std::cout<<"load image "<<vImageFilePathList[id]<<" failed"<<std::endl;
                break;
            }

            if(leftImg.channels() > 1)
            {
                cv::cvtColor(leftImg,leftImg,cv::COLOR_BGR2GRAY);
            }
            if(rightImg.channels() > 1)
            {
                cv::cvtColor(rightImg,rightImg,cv::COLOR_BGR2GRAY);
            }

            ImageFrame leftFrame(vTimeStamp[id],leftImg);
            ImageFrame rightFrame(vTimeStamp[id],rightImg);

            vector<IMU> vImuInterval;
            int imuIntervalSize = GetImuInterval(vImus,vTimeStamp[id],vImuInterval,id == startId);

            //插入队列
            {
                std::unique_lock<std::mutex> lock(mImgMutex);
                mqLeftImage.push(leftFrame);
                mqRightImage.push(rightFrame);
            }
            {
                std::unique_lock<std::mutex> lock(mImuMutex);
                mqvImus.push(vImuInterval);
            }
        }

        {
            std::unique_lock<std::mutex> lock(mMutexFinished);
            this->mbFinished = true;
        }
    }

    bool System::IsMsckfBusy()
    {
        //too many image to be processed;
        bool flag = false;
        {
            std::unique_lock<std::mutex> lock(mImgMutex);
            flag = mqLeftImage.size() > 0 || mqRightImage.size() > 0;
        }
        
        return flag;
    }

    int System::GetImuInterval(vector<IMU> &vSrcImu,double time2,vector<IMU> &vImuInverval,bool bIsFirstFrame)
    {

        vImuInverval.clear();
        int imuSize = vSrcImu.size();
        if(bIsFirstFrame)
        {
            for (int32_t id = this->mnPreImuIndex; id < imuSize; id++) 
            {
                if(vSrcImu[id].stamp >= time2 - 0.01)
                {
                    this->mnPreImuIndex = id;
                    break;
                }
            }
            return 0;
        }

        uint32_t start_index = this->mnPreImuIndex;
        for (uint32_t id = start_index; id < imuSize; id++) 
        {
            if (vSrcImu[id].stamp <= time2 + 0.05) 
            {
                vImuInverval.push_back(vSrcImu[id]);
                this->mnPreImuIndex = id;
            }
            else 
            {
                break;
            }
        }
        this->mnPreImuIndex++;
        return vImuInverval.size();
    }


    void System::MsckfLoop()
    {
        this->mpMsckfVio->resetCallback();
        while(1)
        {
            while(mqLeftImage.size() < 1)
            {
                usleep(50 * 1e3);
            }

            ImageFrame leftFrame,rightFrame;
            {
                std::unique_lock<std::mutex> lock(mImgMutex);
                leftFrame = mqLeftImage.front();
                rightFrame = mqRightImage.front();
                mqLeftImage.pop();
                mqRightImage.pop();
            }

            vector<IMU> vIMUs;
            {
                std::unique_lock<std::mutex> lock(mImuMutex);
                vIMUs = mqvImus.front();
                mqvImus.pop();
            }

            //图像处理
            this->mpImageProcessor->AddImu(vIMUs);
            this->mpImageProcessor->stereoCallback(leftFrame,rightFrame);
            Frame_Feature *pCurFrameFeature = &(this->mpImageProcessor->mFramefeature);
            
            if(this->mpMsckfVio->imuCallback(vIMUs))
            {
                this->mpMsckfVio->featureCallback(*pCurFrameFeature);
            }
            
            {
                 std::unique_lock<std::mutex> lock(mMutexFinished);
                 if(this->mbFinished)
                 {
                     this->mpViewer->RequestFinish();
                     if(!this->mpViewer->isFinished())
                     {
                         usleep(1e6);
                     }
                     break;
                 }
            }
        }
    }
} // namespace name

