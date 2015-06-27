#include <ros/ros.h>
#include <eris/query_server.hpp>
#include <eris/console_pretty.hpp>
#include <eris/encode_tmp_ffi.hpp> // for only intToString
#include <eris/hricase1.hpp>
#include <eris/hricase2.hpp>

/**
 * This is the main callback function
 * @param  req request
 * @param  res response
 * @return     true for success
 */
bool query_process(
	eris::query::Request  &req,
	eris::query::Response &res
){
	clearScreen();
	std::cout << "QUERY SERVER ver. 0.1.0\n================================\n";
    getResultList(req, res);
	printWhite("Done! Standing by...\n");
	return true;
}

void getResultList (
	eris::query::Request  &req,
	eris::query::Response &res
){
	res.multi_responses.push_back("Selected case: "+intToString(req.icase));
	res.multi_responses.push_back("Selected question: "+intToString(req.question));
	res.multi_responses.push_back("Given Tags: ");
	std::vector<std::string>::iterator iter;
	for (iter = req.lexicalinfo.begin();
		iter != req.lexicalinfo.end(); ++iter) {
		res.multi_responses.push_back("* "+*iter);
	}
	// --- Confirming input ---
	switch (req.icase) {
	case 2: {
		Case2 c2;
		if (c2.init(req.lexicalinfo)) {
			res.multi_responses.push_back(c2.answerQuestion(req.question));
		} else break;
		break;
	}
	default: {
		Case1 c1;
		c1.init();
		c1.processFeaturesFromInputScene();
		c1.computeInputFamiliarity();
		c1.determineSceneFamiliarityFromObjs();
		res.multi_responses.push_back(c1.answerQuestion(req.question));
	}
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "query_server");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("qserver",query_process);
	ROS_INFO("Query Server is Ready.");
	ros::spin();
	return 0;
}
