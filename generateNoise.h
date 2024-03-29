#pragma once

#ifndef _INC_NOISE
#define _INC_NOISE
#include<cmath>
#define _USE_MATH_DEFINES
#include<math.h>
#define addSensorNoise 1
#define NOISE_AVE -0.000349443
#define NOISE_DISTRIBUTE 2.25e-5

constexpr double NOISE_AVES[3] = { 2.25094e-5,0.000118618,0.000105 };
constexpr double NOISE_DISTRIBUTES[3] = { -0.00349,0.0040747,-0.00049};
double sdlab_uniform();
//平均mu,標準偏差digmaの正規分布の乱数を生成する関数
double sdlab_normal(double mu, double sigma);
#endif
