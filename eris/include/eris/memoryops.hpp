#ifndef MEMORYOPS_HPP_
#define MEMORYOPS_HPP_

#include <ros/time.h>
#include <vector>
#include <string>

inline ros::Time getTimeNow() {
	return ros::Time::now();
}

inline float color_check(
	const std::vector<int> &color_sensed,
	const std::vector<int> &color_recalled
){
	return getCosineSimilarityDist(color_sensed, color_recalled);
}

inline float texture_check(
	const std::vector<int> &texture_sensed,
	const std::vector<int> &texture_recalled
){
	return getCosineSimilarityDist(texture_sensed, texture_recalled);
}

int getL1Dist(
	const std::vector<int> &v1, 
	const std::vector<int> &v2
);

// My cosine distance similarity 
float getCosineSimilarityDist(
	const std::vector<int> &v1, 
	const std::vector<int> &v2
);

inline bool stringExistsInVect(
	const std::string& s,
	std::vector<std::string>& v
){
	return std::find(v.begin(), v.end(), s) != v.end();
}

bool substringExistInVect(
	const std::string& substr,
	std::vector<std::string>& v
);

std::vector<std::string> removeStringInVect(
	std::string s,
	std::vector<std::string> v
);

void revise_sm_entity(
	std::vector<std::string> parsedTag,
	const std::string& newlabel,
	eris::smEntity& smdata,
	const std::string& oldfname
);

void revise_em_episode(
	const std::string& newlabel,
	const std::string& oldfname
);

void update_ffi_db(
	std::vector<std::string>& newTags,
	const std::string& newLabel,
	const std::string& oldfname
);

#endif