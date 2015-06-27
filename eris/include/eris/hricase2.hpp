#ifndef HRICASE2_HPP_
#define HRICASE2_HPP_
#include <vector>
#include <string>

#include <eris/hricase1.hpp>
#include <eris/tag.h>

template<typename T, size_t N>
T * end(T (&ra)[N]) {
    return ra + N;
}

class Case2 {
	struct Condition {
		bool atleast;
		std::string leftmost;
		std::string rightmost;
		int nOther;
		int nObjs;
		std::vector<std::string> objs;

	};
	Condition cond_;
	std::vector<std::string> context_, matchedScenes_;
	static std::vector<std::string> keywordsTemplate_;
public:
	Case2();
	~Case2();
	bool init(std::vector<std::string> &ctx);
	std::string processContextInputObject();
	std::string processContextInputScene();
	std::string answerQuestion(int q);
};

#endif