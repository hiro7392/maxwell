#ifndef _INC_COMMAND
#define _INC_COMMAND

#include "simMain.h"
#include "BMP.h"
#define BUFSIZE	512		// ���̓R�}���h�p�o�b�t�@�T�C�Y

int getChar(char *buf, const char *msg);
int showHelp();
int setTimer(double delay);
//int copyData(SIM *sim);
int copyData(cFinger* sim);

int saveImage(int width, int height);
int drawData();
int saveGraph2(int trial_num);

#endif
