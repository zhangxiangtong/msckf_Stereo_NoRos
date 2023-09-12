# MSCKF_STEREO_NOROS


The `MSCKF_STEREO_NOROS` package is a no ros version of msckf_vio(https://github.com/KumarRobotics/msckf_vio)，it is convenient to debug code in a no ros environment;

## Compling

mkdir build
cd build
cmake ../
make -j

##Running
./msckf_vio ../config/camchain-imucam-euroc.yaml ${EuRoC_PATH}/EuRoC/V1_01_easy/mav0

## License

Penn Software License. See LICENSE.txt for further details.

## Dependencies

Most of the dependencies are standard including `Eigen`, `OpenCV`, and `Boost`，Pangolin. 

