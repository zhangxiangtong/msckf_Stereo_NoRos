#include<string>
#include"image_processor.h"
#include"msckf_vio.h"
#include"msckf_vio/type.h"
#include<queue>
#include<mutex>
#include<thread>

#include"Viewer.h"
using namespace std;
namespace msckf_vio
{
    class System
    {
    private:
        /* data */
        ImageProcessor *mpImageProcessor;
        MsckfVio *mpMsckfVio;

        Viewer *mpViewer;
        VioDrawer *mpVioDrawer;

        queue<vector<IMU>> mqvImus;
        mutex mImuMutex;

        queue<ImageFrame> mqLeftImage;
        queue<ImageFrame> mqRightImage;
        mutex mImgMutex;

        int mnPreImuIndex;

        bool mbFinished;
        mutex mMutexFinished;

        std::thread *mpthMsckf;
        std::thread *mpthView;
        std::thread *mpthDataProvider;
    public:
        System(string dataSetConfig,string paramconfigfile);

        void Run(string datasetpath);

        int LoadStereoImageList_Euroc(string filepath,vector<double> &vTimeStamp,vector<string> &vImageFilePathList);
        int LoadImuData(string filepath,vector<IMU> &vImus);
        void ReadSensorData(string datasetPath);
        bool IsMsckfBusy();
        int GetImuInterval(vector<IMU> &vSrcImu,double time2,vector<IMU> &mqvImus,bool bIsFirstFrame);
        void MsckfLoop();
        ~System();
    };    
}