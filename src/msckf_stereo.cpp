#include"System.h"

int main(int argc,char **argv)
{
    //for Ruroc dataset,set to path such as ../Euroc/V1_02_medium/mav0
    if(argc < 3)
    {
        std::cout<<"input params:./msckf_vio settingfilepath datasetPath"<<std::endl;
        return 0;
    }
    string settingfile = argv[1];
    string datasetPath = argv[2];

    msckf_vio::System msckf(settingfile,"../config/paramConfig.yaml");
    msckf.Run(datasetPath);
    return 0;
}