#ifndef INCLUDE_common2_h_
#define INCLUDE_common2_h_


#define UNIV_TEST 0

#if UNIV_TEST
	#include "C:\\OpenCV2.4.3\\build\\include\\opencv2\\opencv.hpp"
	#ifdef _DEBUG
	    //Debugモードの場合
	    #pragma comment(lib,"c:\\OpenCV2.4.3\\build\\x64\\vc10\\lib\\opencv_core243d.lib")
	    #pragma comment(lib,"c:\\OpenCV2.4.3\\build\\x64\\vc10\\lib\\opencv_imgproc243d.lib")
	    #pragma comment(lib,"c:\\OpenCV2.4.3\\build\\x64\\vc10\\lib\\opencv_highgui243d.lib")
	    #pragma comment(lib,"c:\\OpenCV2.4.3\\build\\x64\\vc10\\lib\\opencv_objdetect243d.lib")
		//PCL
		#pragma comment(lib,"c:\\Program Files\\PCL\\lib\\pcl_common_debug.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL\\lib\\pcl_io_debug.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL\\lib\\pcl_io_debug.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL\\lib\\pcl_search_debug.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL\\lib\\pcl_kdtree_debug.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL\\lib\\pcl_surface_debug.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL\\lib\\pcl_filters_debug.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL\\lib\\pcl_features_debug.lib")
		//PCL 3rd party
		#pragma comment(lib,"c:\\Program Files\\Boost\\lib\\boost_system-vc100-mt-1_50.lib")
		#pragma comment(lib,"c:\\Program Files\\Boost\\lib\\boost_filesystem-vc100-mt-1_50.lib")
		#pragma comment(lib,"c:\\Program Files\\Boost\\lib\\boost_iostreams-vc100-mt-1_50.lib")
		#pragma comment(lib,"c:\\Program Files\\flann\\lib\\flann_cpp_s.lib")
		#pragma comment(lib,"c:\\Program Files\\qhull\\lib\\qhullstatic_d.lib")
	#else
	    //Releaseモードの場合
		//OpenCV関連
	    #pragma comment(lib,"c:\\OpenCV2.4.3\\build\\x64\\vc10\\lib\\opencv_core243.lib")
		#pragma comment(lib,"c:\\OpenCV2.4.3\\build\\x64\\vc10\\lib\\opencv_imgproc243.lib")
		#pragma comment(lib,"c:\\OpenCV2.4.3\\build\\x64\\vc10\\lib\\opencv_highgui243.lib")
		#pragma comment(lib,"c:\\OpenCV2.4.3\\build\\x64\\vc10\\lib\\opencv_objdetect243.lib")

		//PCL
		#pragma comment(lib,"c:\\Program Files\\PCL\\lib\\pcl_common_release.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL\\lib\\pcl_io_release.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL\\lib\\pcl_io_release.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL\\lib\\pcl_search_release.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL\\lib\\pcl_kdtree_release.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL\\lib\\pcl_surface_release.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL\\lib\\pcl_filters_release.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL\\lib\\pcl_features_release.lib")
		//PCL 3rd party
		#pragma comment(lib,"c:\\Program Files\\Boost\\lib\\boost_system-vc100-mt-1_50.lib")
		#pragma comment(lib,"c:\\Program Files\\Boost\\lib\\boost_filesystem-vc100-mt-1_50.lib")
		#pragma comment(lib,"c:\\Program Files\\Boost\\lib\\boost_iostreams-vc100-mt-1_50.lib")
		#pragma comment(lib,"c:\\Program Files\\Boost\\lib\\libboost_thread-vc100-mt-1_50.lib")
		#pragma comment(lib,"c:\\Program Files\\flann\\lib\\flann_cpp_s.lib")
		#pragma comment(lib,"c:\\Program Files\\qhull\\lib\\qhullstatic.lib")
	#endif
#else
	#include "C:\\opencv\\build\\include\\opencv2\\opencv.hpp"

	#ifdef _DEBUG
		//Debugモードの場合

		#pragma comment(lib,"c:\\opencv\\build\\x86\\vc9\\lib\\opencv_core243d.lib")
		#pragma comment(lib,"c:\\opencv\\build\\x86\\vc9\\lib\\opencv_imgproc243d.lib")
		#pragma comment(lib,"c:\\opencv\\build\\x86\\vc9\\lib\\opencv_highgui243d.lib")
		#pragma comment(lib,"c:\\opencv\\build\\x86\\vc9\\lib\\opencv_objdetect243d.lib")
		#pragma comment(lib,"c:\\opencv\\build\\x86\\vc9\\lib\\opencv_features2d243d.lib")		
		#pragma comment(lib,"c:\\opencv\\build\\x86\\vc9\\lib\\opencv_calib3d243d.lib")		
		#pragma comment(lib,"c:\\opencv\\build\\x86\\vc9\\lib\\opencv_nonfree243d.lib")	
		//PCL
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\lib\\pcl_common_debug.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\lib\\pcl_io_debug.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\lib\\pcl_io_debug.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\lib\\pcl_search_debug.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\lib\\pcl_kdtree_debug.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\lib\\pcl_surface_debug.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\lib\\pcl_filters_debug.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\lib\\pcl_features_debug.lib")
		//PCL 3rd party
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\3rdParty\\Boost\\lib\\boost_system-vc90-mt-1_48.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\3rdParty\\Boost\\lib\\boost_filesystem-vc90-mt-1_48.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\3rdParty\\Boost\\lib\\boost_iostreams-vc90-mt-1_48.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\3rdParty\\Flann\\lib\\flann_cpp_s.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\3rdParty\\Qhull\\lib\\qhullstatic_d.lib")


	#else
		//Releaseモードの場合
		//OpenCV関連
		#pragma comment(lib,"c:\\opencv\\build\\x86\\vc9\\lib\\opencv_core243.lib")
		#pragma comment(lib,"c:\\opencv\\build\\x86\\vc9\\lib\\opencv_imgproc243.lib")
		#pragma comment(lib,"c:\\opencv\\build\\x86\\vc9\\lib\\opencv_highgui243.lib")
		#pragma comment(lib,"c:\\opencv\\build\\x86\\vc9\\lib\\opencv_objdetect243.lib")
		#pragma comment(lib,"c:\\opencv\\build\\x86\\vc9\\lib\\opencv_features2d243.lib")		
		#pragma comment(lib,"c:\\opencv\\build\\x86\\vc9\\lib\\opencv_calib3d243.lib")	
		#pragma comment(lib,"c:\\opencv\\build\\x86\\vc9\\lib\\opencv_nonfree243.lib")
		//PCL
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\lib\\pcl_common_release.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\lib\\pcl_io_release.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\lib\\pcl_io_release.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\lib\\pcl_search_release.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\lib\\pcl_kdtree_release.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\lib\\pcl_surface_release.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\lib\\pcl_filters_release.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\lib\\pcl_features_release.lib")
		//PCL 3rd party
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\3rdParty\\Boost\\lib\\boost_system-vc90-mt-1_48.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\3rdParty\\Boost\\lib\\boost_filesystem-vc90-mt-1_48.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\3rdParty\\Boost\\lib\\boost_iostreams-vc90-mt-1_48.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\3rdParty\\Flann\\lib\\flann_cpp_s.lib")
		#pragma comment(lib,"c:\\Program Files\\PCL 1.5.1\\3rdParty\\Qhull\\lib\\qhullstatic.lib")

	#endif

#endif

#endif