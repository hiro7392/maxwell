#include"generateNoise.h"
//[0,1)の一様分布の乱数を出力する関数
double sdlab_uniform()
{
	double ret = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
	return ret;
}

//平均mu,標準偏差digmaの正規分布の乱数を生成する関数
double sdlab_normal(double mu, double sigma)
{
	double  z = sqrt(-2.0 * log(sdlab_uniform())) *
		sin(2.0 * M_PI * sdlab_uniform());

	return mu + sigma * z;
}