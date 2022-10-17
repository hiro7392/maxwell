#pragma once
#include"generateNoise.h"
#include<filesystem>
#include<fstream>
#include<vector>
#include<algorithm>
void filterTest();
std::vector<double> mediumFilter(std::vector<double>signal, int filterSize);
std::vector<double> averageFilter(std::vector<double>signal, int filterSize);