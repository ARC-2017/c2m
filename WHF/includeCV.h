//�v���W�F�N�g�̃v���p�e�B��C/C++�ˑS�ʁ@�̒ǉ��̃C���N���[�h�f�B���N�g����
// �wC:\OpenCV2.2\include�x��ǉ��̂���
#include "opencv2\\opencv.hpp"

#ifdef _DEBUG
    //Debug���[�h�̏ꍇ
    #pragma comment(lib,"opencv_core243d.lib")            // opencv_core
    #pragma comment(lib,"opencv_imgproc243d.lib")        // opencv_imgproc
    #pragma comment(lib,"opencv_highgui243d.lib")        // opencv_highgui
    #pragma comment(lib,"opencv_objdetect243d.lib")    // opencv_objdetect
    //�ȉ��A�K�v�ɉ����Ēǉ�
    //#pragma comment(lib,"opencv_ml243d.lib")            // opencv_ml
    //#pragma comment(lib,"opencv_features2d243d.lib")    // opencv_features2d
    //#pragma comment(lib,"opencv_video243d.lib")        // opencv_video

    //#pragma comment(lib,"opencv_calib3d243d.lib")        // opencv_calib3d
    //#pragma comment(lib,"opencv_flann243d.lib")        // opencv_flann
    //#pragma comment(lib,"opencv_contrib243d.lib")        // opencv_contrib
    //#pragma comment(lib,"opencv_legacy243d.lib")        // opencv_legacy
    //#pragma comment(lib,"opencv_gpu243d.lib")            // opencv_gpu
#else
    //Release���[�h�̏ꍇ
    #pragma comment(lib,"opencv_core243.lib")            // opencv_core
    #pragma comment(lib,"opencv_imgproc243.lib")        // opencv_imgproc
    #pragma comment(lib,"opencv_highgui243.lib")        // opencv_highgui
    #pragma comment(lib,"opencv_objdetect243.lib")    // opencv_objdetect
    //�ȉ��A�K�v�ɉ����Ēǉ�
    #pragma comment(lib,"opencv_ml243.lib")            // opencv_ml
    #pragma comment(lib,"opencv_features2d243.lib")    // opencv_features2d
    #pragma comment(lib,"opencv_video243.lib")        // opencv_video
    #pragma comment(lib,"opencv_calib3d243.lib")        // opencv_calib3d
    #pragma comment(lib,"opencv_flann243.lib")        // opencv_flann
    #pragma comment(lib,"opencv_contrib243.lib")        // opencv_contrib
    #pragma comment(lib,"opencv_legacy243.lib")        // opencv_legacy
    #pragma comment(lib,"opencv_gpu243.lib")            // opencv_gpu
#endif