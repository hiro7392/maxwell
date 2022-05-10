#include"filter.h"



//filterSizeは奇数
std::vector<double> mediumFilter(std::vector<double>signal,int filterSize) {
	int half = filterSize / 2;
	std::vector<double>filterdSignal(signal.size());
	for (int i = 0; i < signal.size(); i++) {
		if (i - half < 0 || i + half >= signal.size()) {
			filterdSignal[i] = signal[i];
			continue;
		}
		std::vector<double>range(filterSize);
		for (int k = i - half; k <= i + half ; k++) {
			range[k-(i-half)] = signal[k];
		}
		//ソートして中央値をとる
		std::sort(range.begin(), range.end());		
		filterdSignal[i] = range[half];
		
	}
	return filterdSignal;
}

void filterTest() {
	const int len = 1000;
	std::vector<double> sampleNoise(len);
	std::ofstream file;
	file.open("noiseFile.csv");
	for (int i = 0; i < len; i++) {
		sampleNoise[i] = sdlab_normal(0, 4);		//平均0 分散4のノイズを生成
	}
	std::vector<double>filteredNoise5 = mediumFilter(sampleNoise, 5);
	std::vector<double>filteredNoise11 = mediumFilter(sampleNoise, 11);
	std::vector<double>filteredNoise17 = mediumFilter(sampleNoise, 17);
	std::vector<double>filteredNoise25 = mediumFilter(sampleNoise, 25);
	for (int i = 0; i < len; i++) {
		file << sampleNoise[i] << ",";	//フィルタをかける前のものを出力
		//フィルタをかけた信号を出力
		file << filteredNoise5[i] << "," ;	
		file << filteredNoise11[i] << "," ;	
		file << filteredNoise17[i] << ",";	
		file << filteredNoise25[i] << "," << std::endl;	
	}
}