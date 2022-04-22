/////////////////////////////////////////////////////////////////////////
// OpenCV�œ���ۑ�
/////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <math.h>
#include "simMain.h"	// ����\���T�C�Y��ǂݍ���
#include <GL/glut.h>	// stdlib.h����ɓǂݍ��ޕK�v����
#include "video.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#define VER_OPENCV2  0 //�g�p����OpenCV�̃o�[�W������2.x�Ȃ�1, 1.x�Ȃ�0��ݒ肵�ĉ������D
#define VER_OPENCV3  0 //�g�p����OpenCV�̃o�[�W������2.x�Ȃ�1, 1.x�Ȃ�0��ݒ肵�ĉ������D
#define VER_OPENCV4  0 //�g�p����OpenCV�̃o�[�W������2.x�Ȃ�1, 1.x�Ȃ�0��ݒ肵�ĉ������D

#if VER_OPENCV2 
//OpenCV 2.x�̏ꍇ
//�u�v���W�F�N�g�v���usim�̃v���p�e�B�v���uC/C++�v���u�S�ʁv����
// �ǉ��̃C���N���[�h�f�B���N�g����OpenCV��"include"��ǉ�(C:\OpenCV2.4.0\build\include)

#include "opencv2/opencv.hpp"

//���C�u�����[�t�@�C���������̊��ɍ��킹�Đݒ�
//��F(C:\\OpenCV2.3.1\\build\\x86\\vc9\\lib)
//Debug���[�h�̏ꍇ�@C:\\OpenCV2.3.1\\build\\x86\\vc9\\lib
#pragma comment(lib,"C:\\OpenCV-2.2.0\\build\\lib\\opencv_core220d.lib")
#pragma comment(lib,"C:\\OpenCV-2.2.0\\build\\lib\\opencv_imgproc220d.lib")
#pragma comment(lib,"C:\\OpenCV-2.2.0\\build\\lib\\opencv_highgui220d.lib")
////////////////////////////////////
// ����ۑ��ϐ�
////////////////////////////////////
IplImage *frame;
CvVideoWriter *vw;
/////////////////////////////////////////////////////////////////////////
// �����ݒ�
/////////////////////////////////////////////////////////////////////////
void init_video()
{
	// �r�f�I���C�^�\���̂��쐬����
	// �R�[�f�b�N�͈ȉ����Q�Ɓ@http://www.buildinsider.net/small/opencv/05
	//	vw = cvCreateVideoWriter("cap.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, cvSize(600, 600), 1);
	//	vw = cvCreateVideoWriter("cap.avi", 0, 29.97, cvSize(600, 600), 1);		// �񈳏kAVI�t�@�C���𐶐�
	//	vw = cvCreateVideoWriter("cap.avi", 0, 29.97, cvSize(DISPLAY_WIDTH, DISPLAY_HEIGHT), 1);		// �񈳏kAVI�t�@�C���𐶐�
	vw = cvCreateVideoWriter(FILENAME_VIDEO, CV_FOURCC('M', 'J', 'P', 'G'), 30.0, cvSize(DISPLAY_WIDTH, DISPLAY_HEIGHT), 1);
	//	cvNamedWindow("test",1);
	//	frame = cvCreateImage(cvSize(600,600),IPL_DEPTH_8U,3);
	frame = cvCreateImage(cvSize(DISPLAY_WIDTH, DISPLAY_HEIGHT), IPL_DEPTH_8U, 3);
}
/////////////////////////////////////////////////////////////////////////
// �I���ݒ�
/////////////////////////////////////////////////////////////////////////
void final_video()
{
	cvReleaseVideoWriter(&vw);
}
/////////////////////////////////////////////////////////////////////////
// ����ۑ�
/////////////////////////////////////////////////////////////////////////
int save_video()
{
	GLint view[4];
	/* .$B2hLLI=<($N40N;$rBT$D.(B */
	glFinish();
	/* .$B8=:_$N%S%e!<%]!<%H$N%5%$%:$rF@$k.(B */
	glGetIntegerv(GL_VIEWPORT, view);
	glReadPixels(view[0], view[1], view[2], view[3], GL_BGR_EXT, GL_UNSIGNED_BYTE, frame->imageData);		// GL_BGR �͖���`�H
	// ���揑���o��
	cvFlip(frame, frame, 0);	// �㉺���]
	//	cvCvtColor(frame,frame,CV_RGB2BGR);
	//	cvShowImage("test",frame);		// �����o�������܂������Ă��邩�摜��\�����Ċm�F
	cvWriteFrame(vw, frame);
	return 0;
}
#endif //VER_OPENCV2

#if VER_OPENCV3

#include <opencv2/opencv.hpp>
// �o�[�W�����擾
#define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)
// �p�X�ݒ聕�����J�[���͐ݒ�
#ifdef _DEBUG		// �f�o�b�O
//#define CV_LIB_DIR "C:/software/opencv-3.4.1/build/lib/Debug/"
#define CV_LIB_DIR "C:/opencv4.3.0/build/lib/Debug/"

#define CV_EXT "d.lib"
#else		// �����[�X
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
	glReadPixels(view[0], view[1], view[2], view[3], GL_BGR_EXT, GL_UNSIGNED_BYTE, mat_video.data);		// GL_BGR �͖���`�H
	cv::flip(mat_video, mat_video, 0);	// �㉺���](OpenGL�ł͏㉺�t�ɓǂݏo���Ă��邽��)
	out_video << mat_video;

}
#endif


#if VER_OPENCV4

#include <opencv2/opencv.hpp>
// �o�[�W�����擾
#define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)
//// �p�X�ݒ聕�����J�[���͐ݒ�
#ifdef _DEBUG		// �f�o�b�O
////#define CV_LIB_DIR "C:/software/opencv-3.4.1/build/lib/Debug/"
#define CV_LIB_DIR "C:/opencv4.3.0/build/lib/Debug/"

#define CV_EXT "d.lib"
#else		// �����[�X
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
	glReadPixels(view[0], view[1], view[2], view[3], GL_BGR_EXT, GL_UNSIGNED_BYTE, mat_video.data);		// GL_BGR �͖���`�H
	cv::flip(mat_video, mat_video, 0);	// �㉺���](OpenGL�ł͏㉺�t�ɓǂݏo���Ă��邽��)
	out_video << mat_video;

}
#endif