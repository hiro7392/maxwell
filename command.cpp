#include <stdio.h>
#include <string.h>
#include <time.h>
#include <direct.h>
#include "command.h"

using namespace std;

////////////////////////////////////////////////////////
// �����擾
////////////////////////////////////////////////////////
int getChar(char *buf, const char *msg)
{
  fprintf(stdout,"%s",msg);
  fgets(buf, BUFSIZE, stdin);
  buf[strlen(buf)-1] = '\0';      // �������폜
  return 0;
}

////////////////////////////////////////////////////////
// �w���v�\��
////////////////////////////////////////////////////////
int showHelp()
{
  char *help = " a: arm operation\n g: animation\n s: data save\n h: help\n q: quit";
  printf("%s\n", help);
  return 0;
}

////////////////////////////////////////////////////////
// �^�C�}�[
////////////////////////////////////////////////////////
int setTimer(double delay)
{
  clock_t st;
  double time;
  st = clock();		// �J�n�����̎擾
  do{time = (double)(clock()-st)/CLOCKS_PER_SEC;}while(time < delay);
  return 0;
}

////////////////////////////////////////////////////////
// �f�[�^�R�s�[(SIM�\���̂���f�[�^�ۑ��������փR�s�[)
////////////////////////////////////////////////////////
int copyData(cFinger *sim)
{
	auto entity = EntityManager::get();
	if(entity->step < DATA_CNT_NUM){
		sim->save_state_contact[entity->step] = sim->state_contact;
		sim->save_dist[entity->step] = sim->dist;
		for(int jnt=0;jnt<ARM_JNT;jnt++){
			sim->save_ref_jnt_pos[entity->step][jnt] = sim->ref_jnt_pos[jnt];
			sim->save_ref_jnt_vel[entity->step][jnt] = sim->ref_jnt_vel[jnt];
			sim->save_jnt_pos[entity->step][jnt] = sim->jnt_pos[jnt];
			sim->save_jnt_vel[entity->step][jnt] = sim->jnt_vel[jnt];
			sim->save_jnt_force[entity->step][jnt] = sim->jnt_force[jnt];
		}
		for(int crd=0;crd<DIM3;crd++){
			sim->save_ref_eff_pos[entity->step][crd] = sim->ref_eff_pos[crd];
			sim->save_ref_eff_vel[entity->step][crd] = sim->ref_eff_vel[crd];
			sim->save_eff_pos[entity->step][crd] = sim->eff_pos[crd];
			sim->save_eff_vel[entity->step][crd] = sim->eff_vel[crd];
			sim->save_eff_force[entity->step][crd] = sim->eff_force[crd];
			sim->save_obj_pos[entity->step][crd] = sim->obj_pos[crd];
			sim->save_obj_vel[entity->step][crd] = sim->obj_vel[crd];
		}
	}
	return 0;
}

////////////////////////////////////////////////////////
// �t�@�C���ۑ�
// �V�~�����[�V�������n��f�[�^
////////////////////////////////////////////////////////
void cFinger::saveData(){
//	auto sim = EntityManager::init();
	ofstream outdata;
	// �V�~�����[�V������~�܂ł̃f�[�^��ۑ�
	auto entity = EntityManager::get();
	int min = (entity->step<=DATA_CNT_NUM) ? entity->step : DATA_CNT_NUM;	// step��DATA_CNT_NUM�̏������l��min�ɑ��
	outdata.open(data_file_name);
	for (int count = 0; count<min; count++){
		outdata << count*SIM_CYCLE_TIME << " ";		// ����
		for(int jnt=0;jnt<ARM_JNT;jnt++)	outdata << save_jnt_pos[count][jnt] << " ";		// �֐߈ʒu
		for(int jnt=0;jnt<ARM_JNT;jnt++)	outdata << save_jnt_vel[count][jnt] << " ";		// �֐ߑ��x
		for(int jnt=0;jnt<ARM_JNT;jnt++)	outdata << save_jnt_force[count][jnt] << " ";		// �֐ߗ�
		for(int crd=0;crd<DIM3;crd++)	outdata << save_eff_pos[count][crd] << " ";		// ���ʒu
		for(int crd=0;crd<DIM3;crd++)	outdata << save_eff_vel[count][crd] << " ";		// ���ʒu
		for(int crd=0;crd<DIM3;crd++)	outdata << save_eff_force[count][crd] << " ";		// ���O��
		for(int crd=0;crd<DIM3;crd++)	outdata << save_obj_pos[count][crd] << " ";		// �Ώۈʒu
		for(int crd=0;crd<DIM3;crd++)	outdata << save_obj_vel[count][crd] << " ";		// �Ώۑ��x
		outdata << save_state_contact[count] << " ";		// �ڐG���
		outdata << save_dist[count] << " ";		// ����
		outdata << endl;
	}
	outdata.close();
	outdata.open(FILE_SAVE_DIR "data_ref.txt");
	for (int count = 0; count<min; count++){
		outdata << count*SIM_CYCLE_TIME << " ";		// ����
		for(int jnt=0;jnt<ARM_JNT;jnt++)	outdata << save_ref_jnt_pos[count][jnt] << " ";		// �ڕW�֐߈ʒu
		for(int jnt=0;jnt<ARM_JNT;jnt++)	outdata << save_ref_jnt_vel[count][jnt] << " ";		// �ڕW�֐ߑ��x
		outdata << endl;
	}
	outdata.close();
	outdata.open(FILE_SAVE_DIR "displacement.txt");
	for (int count = 0; count<min; count++){
		outdata << count*SIM_CYCLE_TIME << " ";		// ����
		for(int crd=0;crd<DIM2;crd++)	outdata << save_eff_pos[count][crd]-save_eff_pos[0][crd] << " ";		// ���ψ�
		for(int crd=0;crd<DIM2;crd++)	outdata << save_eff_vel[count][crd] << " ";		// ���ψʑ��x
		for(int crd=0;crd<DIM2;crd++)	outdata << save_ref_eff_pos[count][crd] << " ";		// ���ʒu�ڕW�l�i�C���i�[���[�v�j
		for(int crd=0;crd<DIM2;crd++)	outdata << save_ref_eff_vel[count][crd] << " ";		// ��摬�x�ڕW�l�i�C���i�[���[�v�j
		outdata << endl;
	}
	outdata.close();
}

////////////////////////////////////////////////////////
// �t�@�C���ۑ�
// �V�~�����[�V�������
////////////////////////////////////////////////////////
void cFinger::saveInfo()
{
#define print(VarName) outdata<<#VarName"="<<VarName<<endl		// �o�͐��ofstream���Ƃ��낦�邱��
	ofstream outdata(filename_info);
	if (!outdata) { cout << "Can't open file: " << filename_info << endl; cin.get(); }
	print(ARM_LINK1_LEN); print(ARM_LINK2_LEN);
	print(ARM_LINK1_MASS); print(ARM_LINK2_MASS); print(ARM_JNT1_VISCOUS); print(ARM_JNT2_VISCOUS);
//	for(int jnt=0;jnt<ARM_JNT;jnt++)	print(dyn.m[jnt]);		// ������jnt���W�J����Ȃ�
	for(int jnt=0;jnt<ARM_JNT;jnt++)	outdata << "m[" << jnt << "] = " << dyn.m[jnt] << endl;		// �����N����
	for(int jnt=0;jnt<ARM_JNT;jnt++)	outdata << "V[" << jnt << "] = " << dyn.V[jnt] << endl;		// �֐ߖ��C
	for(int crd=0;crd<DIM2;crd++)	outdata << "M[" << crd << "] = " << imp.M.el[crd][crd] << endl;		// ����
	for(int crd=0;crd<DIM2;crd++)	outdata << "C[" << crd << "] = " << imp.C.el[crd][crd] << endl;		// �S��
	for(int crd=0;crd<DIM2;crd++)	outdata << "K[" << crd << "] = " << imp.K.el[crd][crd] << endl;		// �e��
//	for(int crd=0;crd<DIM2;crd++)	outdata << "K0[" << crd << "] = " << imp.K0.el[crd][crd] << endl;		// �e��(Zener���f��)
	for(int crd=0;crd<DIM2;crd++)	outdata << "Gp[" << crd << "] = " << imp.Gp.el[crd][crd] << endl;		// �C���i�[���[�v���Q�C��
	for(int crd=0;crd<DIM2;crd++)	outdata << "Gv[" << crd << "] = " << imp.Gv.el[crd][crd] << endl;		// �C���i�[���[�v���x�Q�C��
	for(int crd=0;crd<DIM3;crd++)	outdata << "T[" << crd << "] = " << imp.T[crd] << endl;		// �U������
	outdata.close();
}

////////////////////////////////////////////////////////
// �摜�ۑ�
// �t�@�C������A�Ԃŕۑ�(3��000�`999�܂őΉ�)
////////////////////////////////////////////////////////
int saveImage(int width, int height)
{
	static string filename = FILE_SAVE_DIR "img/000.bmp";	// �u000�`999�v�̘A�ԂƂȂ�t�@�C��
	static int pos = filename.find("000");		// 000�̐擪�̈ʒu���擾(��ԏ��߂�0�̈ʒu)
	_mkdir(FILE_SAVE_DIR "img");		// img�t�H���_�쐬
	writeBMP(filename.c_str(), width, height);	// BMP�t�@�C�����o��
	// �t�@�C�������C���N�������g
	filename[pos+2]++;		// 0�`9�̕����R�[�h��1���Ⴄ���Ƃ𗘗p
	if (filename[pos+2] == '9' + 1) {		// ���グ
		filename[pos+2] = '0';	filename[pos+1]++;
		if (filename[pos+1] == '9' + 1) { filename[pos+1] = '0'; filename[pos]++; }
	}
	return	0;
}

////////////////////////////////////////////////////////
// �O���t�\���ignuplot�j
////////////////////////////////////////////////////////
#if 0
int drawData()
{
	FILE *gp;
	if((gp=_popen(GNUPLOT_PATH,"w"))==NULL){fprintf(stderr,"Can not find %s!",GNUPLOT_PATH);return -1;}
	// gnuplot�ɃR�}���h�𑗂�
	fprintf(gp, "pl \"%s\" us 1:2 w l, \"%s\" us 1:3 w l\n", DATA_FILE_NAME, DATA_FILE_NAME);
	// �O���t�ۑ�
	fprintf(gp, "set terminal png\n set out \"img_pos_vel.png\"\n rep\n");		// png�o��
	// gnuplot�ɃR�}���h�𑗂�
	fprintf(gp, "pl \"%s\" us 1:4 w l\n", DATA_FILE_NAME);
	// �O���t�ۑ�
	fprintf(gp, "set terminal png\n set out \"img_force.png\"\n rep\n");		// png�o��
//	fprintf(gp, "rep \"%s\" us 1:12 w l\n", DATA_FILE_NAME);
//	fprintf(gp, "rep \"%s\" us 1:21 w l\n", DATA_FILE_NAME);
//	fprintf(gp, "rep \"%s\" us 1:29 w l\n", DATA_FILE_NAME);
//	fprintf(gp, "rep \"%s\" us 1:22 w l, \"%s\" us 1:32 w l\n", DATA_FILE_NAME, DATA_FILE_NAME);
//	fprintf(gp, "cd \"%s\" \n", "data");	// �f�[�^�̂���f�B���N�g���ֈړ�
//	fprintf(gp, "load \"%s\" \n", "jnt_pos.gp");	// gnuplot�X�N���v�g�Ăяo��
//	fprintf(gp, "load \"%s\" \n", "jnt_trq.gp");	// gnuplot�X�N���v�g�Ăяo��
//	fprintf(gp, "rep \"%s\" us 1:22 w l, \"%s\" us 1:32 w l\n", "biped_trq.dat", "biped_trq2.dat");
	// �O���t�ۑ�
//	fprintf(gp, "set terminal png\n set out \"img.png\"\n rep\n");		// png�o��
//	fprintf(gp, "set terminal postscript eps\n set out \"img.eps\"\n rep\n");		// eps�o��
	fflush(gp); // �o�b�t�@�Ɋi�[����Ă���f�[�^��f���o���i�K�{�j
	getchar(); // ���͑҂�
	_pclose(gp);
	return 0;
}
#endif

////////////////////////////////////////////////////////
// �O���t�ۑ��ignuplot�j
////////////////////////////////////////////////////////
void cFinger::saveGraph()
{
	FILE *gp;
	if((gp=_popen(GNUPLOT_PATH,"w"))==NULL){fprintf(stderr,"Can not find %s!",GNUPLOT_PATH);}
//	fflush(gp); // �o�b�t�@�Ɋi�[����Ă���f�[�^��f���o���i�K�{�j// �N�����x���Ƃ��͂��������
#if 0
	fprintf(gp, "pl \"%s\" us 1:2 w l\n", data_file_name);	// �ʒu�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH1);		// png�o��
	fprintf(gp, "pl \"%s\" us 1:3 w l\n", data_file_name);	// �ʒu�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_jnt_pos2.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:($2+$3) w l\n", data_file_name);	// �ʒu�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_eff_pos.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:4 w l\n", data_file_name);	// ���x�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH2);		// png�o��
	fprintf(gp, "pl \"%s\" us 1:5 w l\n", data_file_name);	// ���x�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_jnt_vel2.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:6 w l\n", data_file_name);		// �֐ߗ͂̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH3);		// png�o��
	fprintf(gp, "pl \"%s\" us 1:7 w l\n", data_file_name);		// �֐ߗ͂̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_jnt_force2.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:8 w l\n", data_file_name);		// �O�͂̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH4);		// png�o��
	fprintf(gp, "pl \"%s\" us 1:($2+$3-$9-0.75/2-0.0001-0.15) w l\n", data_file_name);		// ���ƕ��̂̋����̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH5);		// png�o��
#else
	fprintf(gp, "pl \"%s\" us 1:2 w l, \"%s\" us 1:3 w l, \"%s\" us 1:2 w l, \"%s\" us 1:3 w l\n", data_file_name, data_file_name, FILE_SAVE_DIR "data_ref.txt", FILE_SAVE_DIR "data_ref.txt");	// �֐߈ʒu�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH1);		// png�o��
	fprintf(gp, "pl \"%s\" us 1:4 w l, \"%s\" us 1:5 w l\n", data_file_name, data_file_name);	// �֐ߑ��x�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH2);		// png�o��
	fprintf(gp, "pl \"%s\" us 1:6 w l, \"%s\" us 1:7 w l\n", data_file_name, data_file_name);		// �֐ߗ͂̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH3);		// png�o��
	fprintf(gp, "pl \"%s\" us 1:8 w l, \"%s\" us 1:9 w l\n", data_file_name, data_file_name);		// ���ʒu�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "./data/img_eff_pos.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:11 w l, \"%s\" us 1:12 w l\n", data_file_name, data_file_name);		// ��摬�x�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "./data/img_eff_vel.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:14 w l, \"%s\" us 1:15 w l\n", data_file_name, data_file_name);		// ���O�͂̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH4);		// png�o��
	fprintf(gp, "pl \"%s\" us 1:17 w l, \"%s\" us 1:18 w l\n", data_file_name, data_file_name);		// �Ώۈʒu�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "./data/img_obj_pos.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:20 w l, \"%s\" us 1:21 w l\n", data_file_name, data_file_name);		// �Ώۑ��x�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "./data/img_obj_vel.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:23 w l\n", data_file_name);		// �ڐG��Ԃ̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "./data/img_state_contact.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:24 w l\n", data_file_name);		// �����̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "./data/img_dist.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:2 w l, \"%s\" us 1:3 w l, \"%s\" us 1:6 w l, \"%s\" us 1:7 w l\n", "./data/displacement.txt", "./data/displacement.txt", "./data/displacement.txt", "./data/displacement.txt");		// ���ψʂ̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "./data/img_dev_eff.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:4 w l, \"%s\" us 1:5 w l, \"%s\" us 1:8 w l, \"%s\" us 1:9 w l\n", "./data/displacement.txt", "./data/displacement.txt", "./data/displacement.txt", "./data/displacement.txt");		// ���ψʑ��x�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "./data/img_dev_eff_vel.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 2:3 w l, \"%s\" us 6:7 w l\n", "./data/displacement.txt", "./data/displacement.txt");		// ���ψʂ̃O���t
	fprintf(gp, "set size ratio -1\n set terminal png\n set out \"%s\"\n rep\n", "./data/img_dev_eff_xy.png");		// png�o��
#endif
	fflush(gp); // �o�b�t�@�Ɋi�[����Ă���f�[�^��f���o���i�K�{�j
	_pclose(gp);
}

int saveGraph2(int trial_num)
{
	FILE *gp;
	int	incount;
	char filename[256];

	if((gp=_popen(GNUPLOT_PATH,"w"))==NULL){fprintf(stderr,"Can not find %s!",GNUPLOT_PATH);return -1;}
	for(incount=0;incount<trial_num;incount++){
		sprintf(filename, FILENAME_DATA, incount);		// �t�@�C������A�Ԃɐݒ�
		if(incount == 0)	fprintf(gp, "pl \"%s\" us 1:2 w l\n", filename);	// �ʒu�̃O���t
		else	fprintf(gp, "rep \"%s\" us 1:2 w l\n", filename);	// �ʒu�̃O���t
	}
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH1);		// png�o��
	for(incount=0;incount<trial_num;incount++){
		sprintf(filename, FILENAME_DATA, incount);		// �t�@�C������A�Ԃɐݒ�
		if(incount == 0)	fprintf(gp, "pl \"%s\" us 1:3 w l\n", filename);	// ���x�̃O���t
		else	fprintf(gp, "rep \"%s\" us 1:3 w l\n", filename);	// ���x�̃O���t
	}
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH2);		// png�o��
	for(incount=0;incount<trial_num;incount++){
		sprintf(filename, FILENAME_DATA, incount);		// �t�@�C������A�Ԃɐݒ�
		if(incount == 0)	fprintf(gp, "pl \"%s\" us 1:4 w l\n", filename);		// �֐ߗ͂̃O���t
		else	fprintf(gp, "rep \"%s\" us 1:4 w l\n", filename);		// �֐ߗ͂̃O���t
	}
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH3);		// png�o��
	for(incount=0;incount<trial_num;incount++){
		sprintf(filename, FILENAME_DATA, incount);		// �t�@�C������A�Ԃɐݒ�
		if(incount == 0)	fprintf(gp, "pl \"%s\" us 1:5 w l\n", filename);		// �O�͂̃O���t
		else	fprintf(gp, "rep \"%s\" us 1:5 w l\n", filename);		// �O�͂̃O���t
	}
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH4);		// png�o��
	for(incount=0;incount<trial_num;incount++){
		sprintf(filename, FILENAME_DATA, incount);		// �t�@�C������A�Ԃɐݒ�
		if(incount == 0)	fprintf(gp, "pl \"%s\" us 1:($2-$6-0.75/2-0.0001-0.15) w l\n", filename);		// ���ƕ��̂̋����̃O���t
		else	fprintf(gp, "rep \"%s\" us 1:($2-$6-0.75/2-0.0001-0.15) w l\n", filename);		// ���ƕ��̂̋����̃O���t
	}
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH5);		// png�o��
	fflush(gp); // �o�b�t�@�Ɋi�[����Ă���f�[�^��f���o���i�K�{�j
	_pclose(gp);
	return 0;
}
