#ifndef ENCODE_TMP_FFI_HPP_
#define ENCODE_TMP_FFI_HPP_

#include <ros/time.h>

#include <eris/smEntity.h>
//#include <Eigen/Eigen>
#include <algorithm> // for std::find

inline ros::Time getTimeNow() {
	return ros::Time::now();
}

inline const char * const boolToString(bool b) {
	return b ? "true" : "false";
}

std::string intToString(int i);

std::string floatToString (float number);

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

int getL1Dist(
	const std::vector<int> &v1, 
	const std::vector<int> &v2
);
 
float getCosineSimilarityDist(
	const std::vector<int> &v1, 
	const std::vector<int> &v2
);

float color_check(
	const std::vector<int> &color_sensed,
	const std::vector<int> &color_recalled
);

float texture_check(
	const std::vector<int> &texture_sensed,
	const std::vector<int> &texture_recalled
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