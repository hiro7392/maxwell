/////////////////////////////////////////////////////////////////////////
// OpenCVで動画保存
/////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <math.h>
#include "simMain.h"	// 動画表示サイズを読み込み
#include <GL/glut.h>	// stdlib.hより後に読み込む必要あり
#include "video.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#define VER_OPENCV2  0 //使用するOpenCVのバージョンが2.xなら1, 1.xなら0を設定して下さい．
#define VER_OPENCV3  0 //使用するOpenCVのバージョンが2.xなら1, 1.xなら0を設定して下さい．
#define VER_OPENCV4  0 //使用するOpenCVのバージョンが2.xなら1, 1.xなら0を設定して下さい．

#if VER_OPENCV2 
//OpenCV 2.xの場合
//「プロジェクト」→「simのプロパティ」→「C/C++」→「全般」から
// 追加のインクルードディレクトリにOpenCVの"include"を追加(C:\OpenCV2.4.0\build\include)

#include "opencv2/opencv.hpp"

//ライブラリーファイルを自分の環境に合わせて設定
//例：(C:\\OpenCV2.3.1\\build\\x86\\vc9\\lib)
//Debugモードの場合　C:\\OpenCV2.3.1\\build\\x86\\vc9\\lib
#pragma comment(lib,"C:\\OpenCV-2.2.0\\build\\lib\\opencv_core220d.lib")
#pragma comment(lib,"C:\\OpenCV-2.2.0\\build\\lib\\opencv_imgproc220d.lib")
#pragma comment(lib,"C:\\OpenCV-2.2.0\\build\\lib\\opencv_highgui220d.lib")
////////////////////////////////////
// 動画保存変数
////////////////////////////////////
IplImage *frame;
CvVideoWriter *vw;
/////////////////////////////////////////////////////////////////////////
// 初期設定
/////////////////////////////////////////////////////////////////////////
void init_video()
{
	// ビデオライタ構造体を作成する
	// コーデックは以下を参照　http://www.buildinsider.net/small/opencv/05
	//	vw = cvCreateVideoWriter("cap.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, cvSize(600, 600), 1);
	//	vw = cvCreateVideoWriter("cap.avi", 0, 29.97, cvSize(600, 600), 1);		// 非圧縮AVIファイルを生成
	//	vw = cvCreateVideoWriter("cap.avi", 0, 29.97, cvSize(DISPLAY_WIDTH, DISPLAY_HEIGHT), 1);		// 非圧縮AVIファイルを生成
	vw = cvCreateVideoWriter(FILENAME_VIDEO, CV_FOURCC('M', 'J', 'P', 'G'), 30.0, cvSize(DISPLAY_WIDTH, DISPLAY_HEIGHT), 1);
	//	cvNamedWindow("test",1);
	//	frame = cvCreateImage(cvSize(600,600),IPL_DEPTH_8U,3);
	frame = cvCreateImage(cvSize(DISPLAY_WIDTH, DISPLAY_HEIGHT), IPL_DEPTH_8U, 3);
}
/////////////////////////////////////////////////////////////////////////
// 終了設定
/////////////////////////////////////////////////////////////////////////
void final_video()
{
	cvReleaseVideoWriter(&vw);
}
/////////////////////////////////////////////////////////////////////////
// 動画保存
/////////////////////////////////////////////////////////////////////////
int save_video()
{
	GLint view[4];
	/* .$B2hLLI=<($N40N;$rBT$D.(B */
	glFinish();
	/* .$B8=:_$N%S%e!<%]!<%H$N%5%$%:$rF@$k.(B */
	glGetIntegerv(GL_VIEWPORT, view);
	glReadPixels(view[0], view[1], view[2], view[3], GL_BGR_EXT, GL_UNSIGNED_BYTE, frame->imageData);		// GL_BGR は未定義？
	// 動画書き出し
	cvFlip(frame, frame, 0);	// 上下反転
	//	cvCvtColor(frame,frame,CV_RGB2BGR);
	//	cvShowImage("test",frame);		// 書き出しがうまくいっているか画像を表示して確認
	cvWriteFrame(vw, frame);
	return 0;
}
#endif //VER_OPENCV2

#if VER_OPENCV3

#include <opencv2/opencv.hpp>
// バージョン取得
#define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)
// パス設定＆リンカー入力設定
#ifdef _DEBUG		// デバッグ
//#define CV_LIB_DIR "C:/software/opencv-3.4.1/build/lib/Debug/"
#define CV_LIB_DIR "C:/opencv4.3.0/build/lib/Debug/"

#define CV_EXT "d.lib"
#else		// リリース
#define CV_LIB_DIR "C:/software/opencv-3.4.1/build/lib/Release/"
#define CV_EXT ".lib"
#endif
#pragma comment(lib, CV_LIB_DIR "opencv_world" CV_VERSION_STR CV_EXT)
using namespace cv;
cv::VideoWriter out_video;
void init_video(){out_video.open(FILENAME_VIDEO, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30.0, cv::Size(DISPLAY_WIDTH, DISPLAY_HEIGHT), true);}
void final_video(){}
void save_video(){
	GLint view[4];
	/* .$B2hLLI=<($N40N;$rBT$D.(B */
	glFinish();
	/* .$B8=:_$N%S%e!<%]!<%H$N%5%$%:$rF@$k.(B */
	glGetIntegerv(GL_VIEWPORT, view);
	cv::Mat mat_video=cv::Mat(cv::Size(view[2], view[3]), CV_8UC3);
	glReadPixels(view[0], view[1], view[2], view[3], GL_BGR_EXT, GL_UNSIGNED_BYTE, mat_video.data);		// GL_BGR は未定義？
	cv::flip(mat_video, mat_video, 0);	// 上下反転(OpenGLでは上下逆に読み出しているため)
	out_video << mat_video;

}
#endif


#if VER_OPENCV4

#include <opencv2/opencv.hpp>
// バージョン取得
#define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)
//// パス設定＆リンカー入力設定
#ifdef _DEBUG		// デバッグ
////#define CV_LIB_DIR "C:/software/opencv-3.4.1/build/lib/Debug/"
#define CV_LIB_DIR "C:/opencv4.3.0/build/lib/Debug/"

#define CV_EXT "d.lib"
#else		// リリース
#define CV_LIB_DIR "C:/software/opencv-3.4.1/build/lib/Release/"
#define CV_EXT ".lib"
#endif
//#pragma comment(lib, CV_LIB_DIR "opencv_world" CV_VERSION_STR CV_EXT)
#pragma comment(lib, CV_LIB_DIR "opencv_world430" CV_EXT)
using namespace cv;
cv::VideoWriter out_video;
void init_video() { out_video.open(FILENAME_VIDEO, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30.0, cv::Size(DISPLAY_WIDTH, DISPLAY_HEIGHT), true); }
void final_video() {}
void save_video() {
	GLint view[4];
	/* .$B2hLLI=<($N40N;$rBT$D.(B */
	glFinish();
	/* .$B8=:_$N%S%e!<%]!<%H$N%5%$%:$rF@$k.(B */
	glGetIntegerv(GL_VIEWPORT, view);
	cv::Mat mat_video = cv::Mat(cv::Size(view[2], view[3]), CV_8UC3);
	glReadPixels(view[0], view[1], view[2], view[3], GL_BGR_EXT, GL_UNSIGNED_BYTE, mat_video.data);		// GL_BGR は未定義？
	cv::flip(mat_video, mat_video, 0);	// 上下反転(OpenGLでは上下逆に読み出しているため)
	out_video << mat_video;

}
#endif