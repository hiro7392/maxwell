#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <GL/glut.h>
#include "graphic.h"
#include "command.h"

#include "simMain.h"		// PI�̂���

////////////////////////////////////
// ���쐧��ϐ�
////////////////////////////////////
static FLAG	animation_disp_num;
static FLAG	animation_flag;
static FLAG	viewpoint_flag;
static FLAG	video_flag;
static int	push_point[2];		// �h���b�O�J�n�̉摜���W��ۑ�
////////////////////////////////////
// �A�j���[�V�����\��
////////////////////////////////////
static double	pov[3];		// ���_���i�ړ�(point of view)
static double	pan_jnt_deg;	// ���_��]�ړ��i�p���j
static double	tilt_jnt_deg;	// ���_��]�ړ��i�`���g�j
static int	disp_count;
static int	display_rate;		// �����肾��+,�x����-
//static GLfloat lightpos[] = {-2.0, 2.0, -2.0, 1.0};		// �����ʒu
static GLfloat lightpos[] = {0.5, 0.5, 0.5, 1.0};		// �����ʒu
////////////////////////////////////
// ����ۑ��ϐ�
////////////////////////////////////
//IplImage *frame;
//CvVideoWriter *vw;





void idle()
{
	glutPostRedisplay();
}

/////////////////////////////////////////////////////////////////////////
// �����\���ݒ�
/////////////////////////////////////////////////////////////////////////
void  drawMessage()
{
	char  string[] = "x";
	char  str_play[] = "Play";
	char  str_reverse[] = "Reverse";
	char  str_pause[] = "Pause";
	char  str_stop[] = "Stop";
	char  str_rec[] = " REC";
	char  str_rate[10];
	char  message[20];
	int   i;
	int	win_width = 200, win_height = 200;

	// �ˉe�s���������i�������̑O�Ɍ��݂̍s���ޔ�j
	glMatrixMode( GL_PROJECTION );
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D( 0.0, win_width, win_height, 0.0 );
	// ���f���r���[�s���������i�������̑O�Ɍ��݂̍s���ޔ�j
	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glLoadIdentity();
	// �y�o�b�t�@�E���C�e�B���O�̓I�t�ɂ���
	glDisable( GL_DEPTH_TEST );
	glDisable( GL_LIGHTING );
	// ���b�Z�[�W�̕`��
	glColor3f( 1.0, 0.0, 0.0 );
	glRasterPos2i( 8, 8 + 18 );
	if(display_rate > 0)	sprintf(str_rate, "%d", display_rate);
	else	sprintf(str_rate, "1/%d", -display_rate);
	if(animation_flag == ANIMATION_FLAG_PLAY)	sprintf(message, "%s %s %s", str_rate, string, str_play);
	else if(animation_flag == ANIMATION_FLAG_PLAY_REVERSE)	sprintf(message, "%s %s %s", str_rate, string, str_reverse);
	else if(animation_flag == ANIMATION_FLAG_PAUSE)	sprintf(message, "%s %s %s", str_rate, string, str_pause);
	else if(animation_flag == ANIMATION_FLAG_STOP)	sprintf(message, "%s %s %s", str_rate, string, str_stop);
	if(video_flag == VIDEO_FLAG_REC)	strcat(message, str_rec);
	for ( i=0; message[i]!='\0'; i++ )	glutBitmapCharacter( GLUT_BITMAP_HELVETICA_18, message[i] );
	// �ݒ��S�ĕ���
	glEnable( GL_DEPTH_TEST );
	glEnable( GL_LIGHTING );
	glMatrixMode( GL_PROJECTION );
	glPopMatrix();
	glMatrixMode( GL_MODELVIEW );
	glPopMatrix();
}

/////////////////////////////////////////////////////////////////////////
// �`��ݒ�
/////////////////////////////////////////////////////////////////////////
void display()
{
	static int	incount = 0;		// �A�j���[�V�����I����0�ɖ߂�
	static int	prev_disp_count = 0;		// �A�j���[�V�������I�������Ƃ��̎p����`�悷�邽�߂ɓ���

	// ��ʃN���A
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// ���f���r���[�ϊ��s��̏�����
	glLoadIdentity();
	// �����̈ʒu
	glLightfv(GL_LIGHT0, GL_POSITION, lightpos);
	// ���_�̈ړ�
	glTranslated(pov[0], pov[1], pov[2]);
	glRotated(pan_jnt_deg, 0.0, 1.0, 0.0);
	glRotated(tilt_jnt_deg, cos(pan_jnt_deg*PI/180), 0.0, sin(pan_jnt_deg*PI/180));

	//////////////////////////////
	// �`��
	//////////////////////////////
	// �n��
	myGround(0.0);
	// �A�j���[�V�����`��
#if 0	//SYSTEM_HAND
	if(animation_disp_num == ANIMATION_DISP_HAND)
		if(animation_flag == ANIMATION_FLAG_STOP)	display_hand(prev_disp_count);
		else	display_hand(disp_count);
#endif
#if 1	//SYSTEM_WAM
	if(animation_disp_num == ANIMATION_DISP_WAM)
		if(animation_flag == ANIMATION_FLAG_STOP){
			display_arm(prev_disp_count);
			display_obj(prev_disp_count);
		}else{
			display_arm(disp_count);
			display_obj(disp_count);
		}
#endif

	// �A�j���[�V�����`�摬�x�ݒ�
	if(animation_flag == ANIMATION_FLAG_PLAY){
		prev_disp_count = disp_count;		// ���O�ɕ`�悳�ꂽ�J�E���g����ۑ�
		if(display_rate > 0)	disp_count += display_rate;
		else if(incount % -display_rate == 0)	disp_count++;
		incount++;
	}else if(animation_flag == ANIMATION_FLAG_PLAY_REVERSE){
		prev_disp_count = disp_count;		// ���O�ɕ`�悳�ꂽ�J�E���g����ۑ�
		if(display_rate > 0)	disp_count -= display_rate;
		else if(incount % -display_rate == 0)	disp_count--;
		incount--;
	}
	// �A�j���[�V�����I���ݒ�
//	if(disp_count >= (int)(1000*MOTION_TIME) || disp_count < 0){		// �Ō�܂ōĐ������Ƃ� or �ŏ��܂ŋt�Đ������Ƃ�
	if(disp_count >= DATA_CNT_NUM || disp_count < 0){		// �Ō�܂ōĐ������Ƃ� or �ŏ��܂ŋt�Đ������Ƃ�
		disp_count = 0;	incount = 0;
		animation_flag = ANIMATION_FLAG_STOP;	glutIdleFunc(NULL);
	}
	// ����ۑ�
//	if(video_flag == VIDEO_FLAG_REC)	save_video();
//	if(disp_count % SAVE_IMG_RATE == 0)	saveImage(600, 600);	// SAVE_IMG_RATE���ɉ摜�ۑ�

// �����ȍ~�̋L�q�͓���ɂ͕\������Ȃ�
	// ������\��
	drawMessage();
	// ��ʂ̂������}���DglutInitDisplayMode �֐��� GLUT_DOUBLE ��w�肷�邱�ƁD
	glutSwapBuffers();
}

/////////////////////////////////////////////////////////////////////////
// �`��ݒ�
/////////////////////////////////////////////////////////////////////////
void resize(int w, int h)
{
	glViewport(0, 0, w, h);
	// �����ϊ��s��̐ݒ�
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
//	gluPerspective(30.0, (double)w/(double)h, 1.0, 100.0);
	gluPerspective(18.0, (double)w/(double)h, 1.0, 100.0);
	// ���f���r���[�ϊ��s��̐ݒ�
	glMatrixMode(GL_MODELVIEW);
}

/////////////////////////////////////////////////////////////////////////
// �}�E�X����F���_�ύX
/////////////////////////////////////////////////////////////////////////
void mouse(int button, int state, int x, int y)
{
	switch (button){
		case GLUT_LEFT_BUTTON:
			if(state == GLUT_DOWN){
				push_point[0] = x;	push_point[1] = y;
			}else if(state == GLUT_UP){
				viewpoint_flag = VIEWPOINT_FIXED;
			}
			break;
	}
}

/////////////////////////////////////////////////////////////////////////
// �}�E�X����ݒ�
/////////////////////////////////////////////////////////////////////////
void motion(int x, int y)
{
	static int save_point[2];
	int prev_point[2];
#define VIEWPOINT_UPDATE_RATE	0.5
	if(viewpoint_flag == VIEWPOINT_MOVE){
		prev_point[0] = save_point[0];	prev_point[1] = save_point[1];
	}else{		// �h���b�O���n�߂��u��
		prev_point[0] = push_point[0];	prev_point[1] = push_point[1];
		viewpoint_flag = VIEWPOINT_MOVE;
	}
	// ���_�p�x�̕ύX
	pan_jnt_deg += VIEWPOINT_UPDATE_RATE * (x-prev_point[0]);
	tilt_jnt_deg += VIEWPOINT_UPDATE_RATE * (y-prev_point[1]);
	// �h���b�O���̑O�t���[����f��ۑ�
	save_point[0] = x;	save_point[1] = y;
	// �`��X�V
	glutPostRedisplay();
}

/////////////////////////////////////////////////////////////////////////
// �R�}���h�ݒ�
/////////////////////////////////////////////////////////////////////////
void keyboard(unsigned char key, int x, int y)
{
	switch(key){
	// �\���ؑ�
#if SYSTEM_HAND
	case 'h': animation_disp_num = ANIMATION_DISP_HAND; break;		// Hand�i�n���h�\���j
#endif
#if SYSTEM_WAM
	case 'w': animation_disp_num = ANIMATION_DISP_WAM; break;		// WAM�i�A�[���\���j
#endif
	case 'a': animation_disp_num = ANIMATION_DISP_HARM; break;		// HDS�A�[����\��
	// ���_�ړ�
	case 'x': pov[0] += 0.01; break;		// x����
	case 'X': pov[0] -= 0.01; break;		// x����
	case 'y': pov[1] += 0.01; break;		// y����
	case 'Y': pov[1] -= 0.01; break;		// y����
	case 'z': pov[2] += 0.1; break;		// 	z���� (Zoom In)
	case 'Z': pov[2] -= 0.1; break;		// z���� (Zoom Out)
	// �Đ����x�is��2�{�CS��1/2�{�j
	case 's': if(display_rate > 0) display_rate *= 2; else if(display_rate < -2) display_rate /= 2; else display_rate = 1; break;		// Speed Up
	case 'S': if(display_rate > 1) display_rate /= 2; else if(display_rate < 0) display_rate *= 2; else display_rate = -2; break;		// Speed Down
	// ���Z�b�g
	case 'R': init(); break;		// Reset
	// �A�j���[�V�����t���O
	case 'p':			// Play, Pause�i�J�n�ƈꎞ��~�̃g�O������j
		if(animation_flag == ANIMATION_FLAG_PLAY){
			animation_flag = ANIMATION_FLAG_PAUSE; glutIdleFunc(NULL); break;
		}else{
			animation_flag = ANIMATION_FLAG_PLAY; glutIdleFunc(idle); return;
		}
	case 'b': animation_flag = ANIMATION_FLAG_PLAY_REVERSE; glutIdleFunc(idle); break;		// �t�Đ� backward
	// �摜�ۑ�
//	case 'i': saveImage(glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));	break;	//save_image(); break;		// image
	case 'i': saveImage(600, 600);	break;		// image
	// ����ۑ�
	case 'v':			// �ۑ��ƈꎞ��~�̃g�O������
		if(video_flag == VIDEO_FLAG_REC){ video_flag = VIDEO_FLAG_PAUSE; display_rate = 1; return; }
		else{ video_flag = VIDEO_FLAG_REC; display_rate = 33; return; }
	// �I��
	case 'q': case '\0x1B': exit(0);	// Quit or Esc
	default: return;
	}
	// 'p''q''v'�ȊO�̃R�}���h�͈�x�ĕ`��
	glutPostRedisplay();
}

/////////////////////////////////////////////////////////////////////////
// �����ݒ�
/////////////////////////////////////////////////////////////////////////
void init()
{
	glClearColor(1.0, 1.0, 1.0, 1.0);
	// �f�v�X�E�o�b�t�@
	glEnable(GL_DEPTH_TEST);
	// �J�����O����
	glEnable(GL_CULL_FACE);
//	glFrontFace(GL_CW);
	glCullFace(GL_BACK);		// ���ʂ��\��
	// �����ݒ�
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	// �ϐ�������
	animation_disp_num = ANIMATION_DISP_WAM;
	animation_flag = ANIMATION_FLAG_STOP;
	video_flag = VIDEO_FLAG_STOP;
	viewpoint_flag = VIEWPOINT_FIXED;
	disp_count = 0;
	display_rate = 1;
	pov[0] = -0.4;	pov[1] = -0.0;	pov[2] = -10.0;		// ���_�ʒu
//	pov[0] = -0.2;	pov[1] = 0.5;	pov[2] = -2.5;		// ���_�ʒu
	pan_jnt_deg = 150.0;	tilt_jnt_deg = 20.0;		// ���_�p��


	// �r�f�I���C�^�\���̂�쐬����
//	vw = cvCreateVideoWriter("cap.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, cvSize(600, 600), 1);
//	vw = cvCreateVideoWriter("cap.avi", 0, 29.97, cvSize(600, 600), 1);		// �񈳏kAVI�t�@�C���𐶐�
//	cvNamedWindow("test",1);
//	frame = cvCreateImage(cvSize(600,600),IPL_DEPTH_8U,3);
}
