#include"generateNoise.h"
//[0,1)�̈�l���z�̗������o�͂���֐�
double sdlab_uniform()
{
	double ret = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
	return ret;
}

//����mu,�W���΍�digma�̐��K���z�̗����𐶐�����֐�
double sdlab_normal(double mu, double sigma)
{
	double  z = sqrt(-2.0 * log(sdlab_uniform())) *
		sin(2.0 * M_PI * sdlab_uniform());

	return mu + sigma * z;
}