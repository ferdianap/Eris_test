/**
 * REVISE KNOWLEDGE NODE
 * This node is a representation of SM data manual revision from human.
 *
 */

#include <ros/ros.h>
#include <eris/encode_tmp_ffi.hpp>
#include <eris/console_pretty.hpp>
#include <eris/file_operations.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <eris/tag.h>

#include <boost/algorithm/string.hpp> // for parsing string

void doRevision(
	std::string &inputTag,
	std::string &inputLabel,
	std::vector<std::string>::iterator &iter,
	eris::smEntity &smdata,
	bool &link_flag // TODO: continue here, how to link (check smEntity.msg)
){
	// parse the multiple category and assign into vector of string
	std::vector<std::string> parsedTag;
	boost::split(parsedTag, inputTag, boost::is_any_of("/.,"));
	
	for (int i=0; i<parsedTag.size(); i++ ) { // for each tag
		printYellow(parsedTag[i]+"\n");
		eris::tag loadedTag;
		if (fileExist(MEMORY_DIR+parsedTag[i]+".ffi")) {
			ROS_WARN_STREAM("file "+parsedTag[i]+".ffi already exist! Loading file...");
			loadedTag = read_ffi_tag(parsedTag[i]);
		} else { // encode the ffi with empty tag
			loadedTag.name = parsedTag[i];
			ROS_INFO_STREAM("filling loadedTag.name = "+parsedTag[i]);
		}
		loadedTag.assoc_obj.push_back(inputLabel);
		ROS_INFO_STREAM("assign assoc_obj = "+inputLabel);
		write_ffi_tag(parsedTag[i], loadedTag);
		ROS_WARN("writing file...");
	}
	// 4. add new tags to the db.ffi database
	update_ffi_db(parsedTag, inputLabel, *iter);

	// TODO: (Optional) give options to list all Tags and all Assoc_obj
	ROS_INFO_STREAM("revising with "+inputTag+" tag and "+inputLabel+" label\n");
	// 5. 
	revise_sm_entity(parsedTag, inputLabel, smdata, *iter); // DONE!
	revise_em_episode(inputLabel, *iter); // DONE !
	// by checking all EM data and check if corresponding obj_name vector contains
	// the tag one by one, if yes, then change and then rewrite EM data
}


int main(int argc, char** argv) {
 	ros::init(argc, argv, "manual_sm_revision_node");
	ros::NodeHandle n;
	std::cout << "Checking dir for sm files\n";
	const std::string fname("OBJ_");
	std::vector<std::string> dirlist = getFileList(MEMORY_DIR, fname);
	std::vector<std::string>::iterator iter;
	cv_bridge::CvImagePtr preview;
	eris::smEntity smdata;
	cv::namedWindow("VIEW", CV_WINDOW_AUTOSIZE);
	cv::moveWindow("VIEW", 400, 400);
	
	if (dirlist.empty()) {
		printYellow("No temporary files for revision.\n");
	} else {
		for (iter = dirlist.begin(); iter != dirlist.end(); ++iter) {
			smdata = read_sm_entity(*iter);
			preview = cv_bridge::toCvCopy(smdata.image, sensor_msgs::image_encodings::BGR8);
			cv::imshow("VIEW", preview->image);
			cv::waitKey(30);
			std::string inputTag("");
			std::string inputLabel("");
			bool link_flag = false;
			printf("Please enter Tags for the object (/,. separated):\n> ");
			getline(std::cin, inputTag);
			if (inputTag.empty()) {
				ROS_ERROR("No Tags are assigned. Please try again!\n");
				return 1;
			}
			printf("Please enter a Label for the object:\n> ");
			getline(std::cin, inputLabel);
			//printf("label = %s \n",smdata.tag[0].c_str());
			if (inputLabel.empty()) {
				ROS_ERROR("No Input is assigned. Please try again!\n");
				return 1;
			}
			// Check if there is already exist given label
			if (fileExist(MEMORY_DIR+inputLabel+".sm")) {
				printRed("WARNING!\nMemory of \""+inputLabel+"\" is already exist.\nLink this object with the existing \""+ inputLabel+"? <y/N>");
				if (confirmNotProceed()) {
					printRed("PROCEED WITH CAUTION!\nOverwriting may cause confusion of the affected past events.\nOverwrite? <y/N>");
					if (confirmNotProceed()) {
						printf("Please restart.\n\n");
						break;	
					} else {
						printf("Overwriting existing memory!\n\n");
					}
				} else {
					// link the current revised object with inputLabel
					printBlue("Linking...\n");
					link_flag = true;
				}
			}
			doRevision(inputTag,inputLabel,iter,smdata,link_flag);

			printYellow("Continue revising next file? <y/N> ");
			if (confirmNotProceed()) {
				printf("Revision session over.!\n\n");
				break;
			} else {
				printf("Revising next file ...\n\n");
			}
		}
		cv::destroyWindow("VIEW");
	}
	printRed("------> SUCCESS!\n");
	return 0;
}
