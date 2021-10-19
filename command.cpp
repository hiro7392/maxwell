#include <stdio.h>
#include <string.h>
#include <time.h>
#include <direct.h>
#include "command.h"

using namespace std;

////////////////////////////////////////////////////////
// 文字取得
////////////////////////////////////////////////////////
int getChar(char *buf, const char *msg)
{
  fprintf(stdout,"%s",msg);
  fgets(buf, BUFSIZE, stdin);
  buf[strlen(buf)-1] = '\0';      // 復改を削除
  return 0;
}

////////////////////////////////////////////////////////
// ヘルプ表示
////////////////////////////////////////////////////////
int showHelp()
{
  char *help = " a: arm operation\n g: animation\n s: data save\n h: help\n q: quit";
  printf("%s\n", help);
  return 0;
}

////////////////////////////////////////////////////////
// タイマー
////////////////////////////////////////////////////////
int setTimer(double delay)
{
  clock_t st;
  double time;
  st = clock();		// 開始時刻の取得
  do{time = (double)(clock()-st)/CLOCKS_PER_SEC;}while(time < delay);
  return 0;
}

////////////////////////////////////////////////////////
// データコピー(SIM構造体からデータ保存メモリへコピー)
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
// ファイル保存
// シミュレーション時系列データ
////////////////////////////////////////////////////////
void cFinger::saveData(){
//	auto sim = EntityManager::init();
	ofstream outdata;
	// シミュレーション停止までのデータを保存
	auto entity = EntityManager::get();
	int min = (entity->step<=DATA_CNT_NUM) ? entity->step : DATA_CNT_NUM;	// stepかDATA_CNT_NUMの小さい値をminに代入
	outdata.open(data_file_name);
	for (int count = 0; count<min; count++){
		outdata << count*SIM_CYCLE_TIME << " ";		// 時間
		for(int jnt=0;jnt<ARM_JNT;jnt++)	outdata << save_jnt_pos[count][jnt] << " ";		// 関節位置
		for(int jnt=0;jnt<ARM_JNT;jnt++)	outdata << save_jnt_vel[count][jnt] << " ";		// 関節速度
		for(int jnt=0;jnt<ARM_JNT;jnt++)	outdata << save_jnt_force[count][jnt] << " ";		// 関節力
		for(int crd=0;crd<DIM3;crd++)	outdata << save_eff_pos[count][crd] << " ";		// 手先位置
		for(int crd=0;crd<DIM3;crd++)	outdata << save_eff_vel[count][crd] << " ";		// 手先位置
		for(int crd=0;crd<DIM3;crd++)	outdata << save_eff_force[count][crd] << " ";		// 手先外力
		for(int crd=0;crd<DIM3;crd++)	outdata << save_obj_pos[count][crd] << " ";		// 対象位置
		for(int crd=0;crd<DIM3;crd++)	outdata << save_obj_vel[count][crd] << " ";		// 対象速度
		outdata << save_state_contact[count] << " ";		// 接触状態
		outdata << save_dist[count] << " ";		// 距離
		outdata << endl;
	}
	outdata.close();
	outdata.open(FILE_SAVE_DIR "data_ref.txt");
	for (int count = 0; count<min; count++){
		outdata << count*SIM_CYCLE_TIME << " ";		// 時間
		for(int jnt=0;jnt<ARM_JNT;jnt++)	outdata << save_ref_jnt_pos[count][jnt] << " ";		// 目標関節位置
		for(int jnt=0;jnt<ARM_JNT;jnt++)	outdata << save_ref_jnt_vel[count][jnt] << " ";		// 目標関節速度
		outdata << endl;
	}
	outdata.close();
	outdata.open(FILE_SAVE_DIR "displacement.txt");
	for (int count = 0; count<min; count++){
		outdata << count*SIM_CYCLE_TIME << " ";		// 時間
		for(int crd=0;crd<DIM2;crd++)	outdata << save_eff_pos[count][crd]-save_eff_pos[0][crd] << " ";		// 手先変位
		for(int crd=0;crd<DIM2;crd++)	outdata << save_eff_vel[count][crd] << " ";		// 手先変位速度
		for(int crd=0;crd<DIM2;crd++)	outdata << save_ref_eff_pos[count][crd] << " ";		// 手先位置目標値（インナーループ）
		for(int crd=0;crd<DIM2;crd++)	outdata << save_ref_eff_vel[count][crd] << " ";		// 手先速度目標値（インナーループ）
		outdata << endl;
	}
	outdata.close();
}

////////////////////////////////////////////////////////
// ファイル保存
// シミュレーション情報
////////////////////////////////////////////////////////
void cFinger::saveInfo()
{
#define print(VarName) outdata<<#VarName"="<<VarName<<endl		// 出力先をofstream名とそろえること
	ofstream outdata(filename_info);
	if (!outdata) { cout << "Can't open file: " << filename_info << endl; cin.get(); }
	print(ARM_LINK1_LEN); print(ARM_LINK2_LEN);
	print(ARM_LINK1_MASS); print(ARM_LINK2_MASS); print(ARM_JNT1_VISCOUS); print(ARM_JNT2_VISCOUS);
//	for(int jnt=0;jnt<ARM_JNT;jnt++)	print(dyn.m[jnt]);		// 文字列jntが展開されない
	for(int jnt=0;jnt<ARM_JNT;jnt++)	outdata << "m[" << jnt << "] = " << dyn.m[jnt] << endl;		// リンク質量
	for(int jnt=0;jnt<ARM_JNT;jnt++)	outdata << "V[" << jnt << "] = " << dyn.V[jnt] << endl;		// 関節摩擦
	for(int crd=0;crd<DIM2;crd++)	outdata << "M[" << crd << "] = " << imp.M.el[crd][crd] << endl;		// 慣性
	for(int crd=0;crd<DIM2;crd++)	outdata << "C[" << crd << "] = " << imp.C.el[crd][crd] << endl;		// 粘性
	for(int crd=0;crd<DIM2;crd++)	outdata << "K[" << crd << "] = " << imp.K.el[crd][crd] << endl;		// 弾性
//	for(int crd=0;crd<DIM2;crd++)	outdata << "K0[" << crd << "] = " << imp.K0.el[crd][crd] << endl;		// 弾性(Zenerモデル)
	for(int crd=0;crd<DIM2;crd++)	outdata << "Gp[" << crd << "] = " << imp.Gp.el[crd][crd] << endl;		// インナーループ比例ゲイン
	for(int crd=0;crd<DIM2;crd++)	outdata << "Gv[" << crd << "] = " << imp.Gv.el[crd][crd] << endl;		// インナーループ速度ゲイン
	for(int crd=0;crd<DIM3;crd++)	outdata << "T[" << crd << "] = " << imp.T[crd] << endl;		// 振動周期
	outdata.close();
}

////////////////////////////////////////////////////////
// 画像保存
// ファイル名を連番で保存(3桁000～999まで対応)
////////////////////////////////////////////////////////
int saveImage(int width, int height)
{
	static string filename = FILE_SAVE_DIR "img/000.bmp";	// 「000～999」の連番となるファイル
	static int pos = filename.find("000");		// 000の先頭の位置を取得(一番初めの0の位置)
	_mkdir(FILE_SAVE_DIR "img");		// imgフォルダ作成
	writeBMP(filename.c_str(), width, height);	// BMPファイルを出力
	// ファイル名をインクリメント
	filename[pos+2]++;		// 0～9の文字コードが1ずつ違うことを利用
	if (filename[pos+2] == '9' + 1) {		// 桁上げ
		filename[pos+2] = '0';	filename[pos+1]++;
		if (filename[pos+1] == '9' + 1) { filename[pos+1] = '0'; filename[pos]++; }
	}
	return	0;
}

////////////////////////////////////////////////////////
// グラフ表示（gnuplot）
////////////////////////////////////////////////////////
#if 0
int drawData()
{
	FILE *gp;
	if((gp=_popen(GNUPLOT_PATH,"w"))==NULL){fprintf(stderr,"Can not find %s!",GNUPLOT_PATH);return -1;}
	// gnuplotにコマンドを送る
	fprintf(gp, "pl \"%s\" us 1:2 w l, \"%s\" us 1:3 w l\n", DATA_FILE_NAME, DATA_FILE_NAME);
	// グラフ保存
	fprintf(gp, "set terminal png\n set out \"img_pos_vel.png\"\n rep\n");		// png出力
	// gnuplotにコマンドを送る
	fprintf(gp, "pl \"%s\" us 1:4 w l\n", DATA_FILE_NAME);
	// グラフ保存
	fprintf(gp, "set terminal png\n set out \"img_force.png\"\n rep\n");		// png出力
//	fprintf(gp, "rep \"%s\" us 1:12 w l\n", DATA_FILE_NAME);
//	fprintf(gp, "rep \"%s\" us 1:21 w l\n", DATA_FILE_NAME);
//	fprintf(gp, "rep \"%s\" us 1:29 w l\n", DATA_FILE_NAME);
//	fprintf(gp, "rep \"%s\" us 1:22 w l, \"%s\" us 1:32 w l\n", DATA_FILE_NAME, DATA_FILE_NAME);
//	fprintf(gp, "cd \"%s\" \n", "data");	// データのあるディレクトリへ移動
//	fprintf(gp, "load \"%s\" \n", "jnt_pos.gp");	// gnuplotスクリプト呼び出し
//	fprintf(gp, "load \"%s\" \n", "jnt_trq.gp");	// gnuplotスクリプト呼び出し
//	fprintf(gp, "rep \"%s\" us 1:22 w l, \"%s\" us 1:32 w l\n", "biped_trq.dat", "biped_trq2.dat");
	// グラフ保存
//	fprintf(gp, "set terminal png\n set out \"img.png\"\n rep\n");		// png出力
//	fprintf(gp, "set terminal postscript eps\n set out \"img.eps\"\n rep\n");		// eps出力
	fflush(gp); // バッファに格納されているデータを吐き出す（必須）
	getchar(); // 入力待ち
	_pclose(gp);
	return 0;
}
#endif

////////////////////////////////////////////////////////
// グラフ保存（gnuplot）
////////////////////////////////////////////////////////
void cFinger::saveGraph()
{
	FILE *gp;
	if((gp=_popen(GNUPLOT_PATH,"w"))==NULL){fprintf(stderr,"Can not find %s!",GNUPLOT_PATH);}
//	fflush(gp); // バッファに格納されているデータを吐き出す（必須）// 起動が遅いときはこれを入れる
#if 0
	fprintf(gp, "pl \"%s\" us 1:2 w l\n", data_file_name);	// 位置のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH1);		// png出力
	fprintf(gp, "pl \"%s\" us 1:3 w l\n", data_file_name);	// 位置のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_jnt_pos2.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:($2+$3) w l\n", data_file_name);	// 位置のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_eff_pos.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:4 w l\n", data_file_name);	// 速度のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH2);		// png出力
	fprintf(gp, "pl \"%s\" us 1:5 w l\n", data_file_name);	// 速度のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_jnt_vel2.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:6 w l\n", data_file_name);		// 関節力のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH3);		// png出力
	fprintf(gp, "pl \"%s\" us 1:7 w l\n", data_file_name);		// 関節力のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_jnt_force2.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:8 w l\n", data_file_name);		// 外力のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH4);		// png出力
	fprintf(gp, "pl \"%s\" us 1:($2+$3-$9-0.75/2-0.0001-0.15) w l\n", data_file_name);		// 手先と物体の距離のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH5);		// png出力
#else
	fprintf(gp, "pl \"%s\" us 1:2 w l, \"%s\" us 1:3 w l, \"%s\" us 1:2 w l, \"%s\" us 1:3 w l\n", data_file_name, data_file_name, FILE_SAVE_DIR "data_ref.txt", FILE_SAVE_DIR "data_ref.txt");	// 関節位置のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH1);		// png出力
	fprintf(gp, "pl \"%s\" us 1:4 w l, \"%s\" us 1:5 w l\n", data_file_name, data_file_name);	// 関節速度のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH2);		// png出力
	fprintf(gp, "pl \"%s\" us 1:6 w l, \"%s\" us 1:7 w l\n", data_file_name, data_file_name);		// 関節力のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH3);		// png出力
	fprintf(gp, "pl \"%s\" us 1:8 w l, \"%s\" us 1:9 w l\n", data_file_name, data_file_name);		// 手先位置のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "./data/img_eff_pos.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:11 w l, \"%s\" us 1:12 w l\n", data_file_name, data_file_name);		// 手先速度のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "./data/img_eff_vel.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:14 w l, \"%s\" us 1:15 w l\n", data_file_name, data_file_name);		// 手先外力のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH4);		// png出力
	fprintf(gp, "pl \"%s\" us 1:17 w l, \"%s\" us 1:18 w l\n", data_file_name, data_file_name);		// 対象位置のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "./data/img_obj_pos.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:20 w l, \"%s\" us 1:21 w l\n", data_file_name, data_file_name);		// 対象速度のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "./data/img_obj_vel.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:23 w l\n", data_file_name);		// 接触状態のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "./data/img_state_contact.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:24 w l\n", data_file_name);		// 距離のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "./data/img_dist.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:2 w l, \"%s\" us 1:3 w l, \"%s\" us 1:6 w l, \"%s\" us 1:7 w l\n", "./data/displacement.txt", "./data/displacement.txt", "./data/displacement.txt", "./data/displacement.txt");		// 手先変位のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "./data/img_dev_eff.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:4 w l, \"%s\" us 1:5 w l, \"%s\" us 1:8 w l, \"%s\" us 1:9 w l\n", "./data/displacement.txt", "./data/displacement.txt", "./data/displacement.txt", "./data/displacement.txt");		// 手先変位速度のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "./data/img_dev_eff_vel.png");		// png出力
	fprintf(gp, "pl \"%s\" us 2:3 w l, \"%s\" us 6:7 w l\n", "./data/displacement.txt", "./data/displacement.txt");		// 手先変位のグラフ
	fprintf(gp, "set size ratio -1\n set terminal png\n set out \"%s\"\n rep\n", "./data/img_dev_eff_xy.png");		// png出力
#endif
	fflush(gp); // バッファに格納されているデータを吐き出す（必須）
	_pclose(gp);
}

int saveGraph2(int trial_num)
{
	FILE *gp;
	int	incount;
	char filename[256];

	if((gp=_popen(GNUPLOT_PATH,"w"))==NULL){fprintf(stderr,"Can not find %s!",GNUPLOT_PATH);return -1;}
	for(incount=0;incount<trial_num;incount++){
		sprintf(filename, FILENAME_DATA, incount);		// ファイル名を連番に設定
		if(incount == 0)	fprintf(gp, "pl \"%s\" us 1:2 w l\n", filename);	// 位置のグラフ
		else	fprintf(gp, "rep \"%s\" us 1:2 w l\n", filename);	// 位置のグラフ
	}
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH1);		// png出力
	for(incount=0;incount<trial_num;incount++){
		sprintf(filename, FILENAME_DATA, incount);		// ファイル名を連番に設定
		if(incount == 0)	fprintf(gp, "pl \"%s\" us 1:3 w l\n", filename);	// 速度のグラフ
		else	fprintf(gp, "rep \"%s\" us 1:3 w l\n", filename);	// 速度のグラフ
	}
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH2);		// png出力
	for(incount=0;incount<trial_num;incount++){
		sprintf(filename, FILENAME_DATA, incount);		// ファイル名を連番に設定
		if(incount == 0)	fprintf(gp, "pl \"%s\" us 1:4 w l\n", filename);		// 関節力のグラフ
		else	fprintf(gp, "rep \"%s\" us 1:4 w l\n", filename);		// 関節力のグラフ
	}
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH3);		// png出力
	for(incount=0;incount<trial_num;incount++){
		sprintf(filename, FILENAME_DATA, incount);		// ファイル名を連番に設定
		if(incount == 0)	fprintf(gp, "pl \"%s\" us 1:5 w l\n", filename);		// 外力のグラフ
		else	fprintf(gp, "rep \"%s\" us 1:5 w l\n", filename);		// 外力のグラフ
	}
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH4);		// png出力
	for(incount=0;incount<trial_num;incount++){
		sprintf(filename, FILENAME_DATA, incount);		// ファイル名を連番に設定
		if(incount == 0)	fprintf(gp, "pl \"%s\" us 1:($2-$6-0.75/2-0.0001-0.15) w l\n", filename);		// 手先と物体の距離のグラフ
		else	fprintf(gp, "rep \"%s\" us 1:($2-$6-0.75/2-0.0001-0.15) w l\n", filename);		// 手先と物体の距離のグラフ
	}
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH5);		// png出力
	fflush(gp); // バッファに格納されているデータを吐き出す（必須）
	_pclose(gp);
	return 0;
}
