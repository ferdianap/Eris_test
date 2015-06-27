/**
 * Image Loader Node
 *
 * This node publishes /static_image of saved scenes as the replacement
 * of live camera feed from the robot.
 *
 */ 

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <visor/call.h> // Service
//#include <visor/imgloader.hpp>

int current_displayed_scene_no = 1; // default is 1 initially
size_t max_scene_avail = 1;

namespace fs = boost::filesystem;

struct directory_statistics {
	size_t file_count;
	size_t dir_count;
	size_t other_count;
	size_t err_count;
};

/* *** FOR DIRECTORY STATISTIC. TODO: make a separate pkg for this ***
 * Char comparison method
 * CASE INSENSITIVE !!
 *
 */
inline bool ci_equal(char ch1, char ch2) {
	return toupper((unsigned char)ch1) == toupper((unsigned char)ch2);
}

/*
 * Find whether str2 is exist in str1
 *
 */
inline bool isFound(const std::string& str1, const std::string& str2) {
	return (search(str1.begin(), str1.end(), str2.begin(), str2.end(),
		ci_equal) == str1.end() ? false : true);
}

std::string intToString(int i) {
    std::stringstream ss;
    std::string s;
    ss << i;
    s = ss.str();

    return s;
}

bool sceneChange(
	visor::call::Request  &req,
	visor::call::Response &res
){
	// here, change the member string variable the name of the changed scene
	current_displayed_scene_no += 1;
	if (current_displayed_scene_no > max_scene_avail) {
		current_displayed_scene_no = 1;
	}
	ROS_DEBUG_STREAM("Scene has been changed" << req.call);
	res.resp = "Scene has been changed (sceneChange_server)"+intToString(current_displayed_scene_no);
	return true;
}

directory_statistics getDirectoryStatistics(
	const std::string &PATH,
	const std::string &FILENAME
){
	directory_statistics dir_stats;
	fs::path full_path( fs::initial_path<fs::path>() );
	full_path = fs::system_complete( fs::path( PATH ) );
	size_t file_count = 0;
	size_t dir_count = 0;
	size_t other_count = 0;
	size_t err_count = 0;
	assert(fs::exists(full_path));
	if (fs::is_directory(full_path)) {
		//std::cout << "\nIn directory: " << full_path.string() << "\n\n";
		fs::directory_iterator end_iter;
		for ( fs::directory_iterator dir_itr( full_path );
	    	dir_itr != end_iter;
	    	++dir_itr
	    ){
	    	std::string fname = dir_itr->path().filename().string();  	
		    try {
		    	if (isFound(fname, FILENAME)) {
		    		if (fs::is_directory(dir_itr->status())) {
						++dir_count;
						//std::cout << fname << " [directory]\n";
					} else if (fs::is_regular_file(dir_itr->status())) {
						++file_count;
			  			//std::cout << fname << "\n";
					} else {
				  		++other_count;
				  		//std::cout << fname << " [other]\n";
					}
		      	}
	      	} catch (const std::exception & ex) {
	        	++err_count;
	        	std::cout << fname << " " << ex.what() << std::endl;
	      	}
	    }
	    //std::cout << "\n" << file_count << " files\n"
	    //          << dir_count 			<< " directories\n"
	    //          << other_count 		<< " others\n"
	    //          << err_count 			<< " errors\n";
	    dir_stats.file_count = file_count;
	    dir_stats.dir_count = dir_count;
	    dir_stats.other_count = other_count;
	    dir_stats.err_count = err_count;
	} else { // must be a file
		std::cout << "\nFound: " << full_path.string() << "\n";    
	}
	return dir_stats;
}// ******************************************************************


int main(int argc, char** argv) {
	ros::init(argc, argv, "imgloader");
	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");
	std::string img_fdir("");
	std::string format("");
	std::string ext("");

	ros::Rate loop_rate(5);
	nhp.param(
		std::string("scene_dir"), 
		img_fdir, 
		std::string("/home/ros/scene_snap/sequence2/")
	);
	nhp.param(
		std::string("scene_label_format"), 
		format, 
		std::string("scene%01d")
	);
	nhp.param(
		std::string("scene_ext"), 
		ext, 
		std::string(".JPG")
	);

	directory_statistics dir_stat = getDirectoryStatistics(img_fdir, ext);
	max_scene_avail = dir_stat.file_count;

	ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/static_image", 1);
	ros::ServiceServer service = nh.advertiseService("scenechange_server", sceneChange);
	
	while (nh.ok()) {
		sensor_msgs::Image scene_msg;
		try { // load scene
			cv_bridge::CvImage loaded_scene;
			std::string fname("%s/"+format+ext);
			loaded_scene.image = cv::imread(cv::format(fname.c_str(), img_fdir.c_str(), current_displayed_scene_no), CV_LOAD_IMAGE_COLOR);
			loaded_scene.encoding = "bgr8";
			loaded_scene.toImageMsg(scene_msg);
		} catch (std::exception& e) {
			std::cerr << "Unhandled Exception reached the top of main: "
				<< e.what() << ", application will now exit\n";
			return 1;
		}
		pub.publish(scene_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
//*/



/*
Imgloader::Imgloader(std::string scene_dir) {
	scene_dir_ = scene_dir;
	pub_ = nh_.advertise<sensor_msgs::Image>("/static_image", 1);
	// SERVICE
	service_ = nh_.advertiseService("scenechange_server",&Imgloader::sceneChangeServiceCallback, this);
	ROS_WARN("Scene Change Server is Ready.");
}

Imgloader::~Imgloader() {
}

void Imgloader::imgloaderCallback() {
	ros::Rate loop_rate(5);
	try { // load image from the private member current scene
		std::string img_fname("/home/ros/scene_snap/sequence2/scene1.JPG");
		cv_bridge::CvImage loaded_scene;
		loaded_scene.image = cv::imread(img_fname,CV_LOAD_IMAGE_COLOR);
		loaded_scene.encoding = "bgr8";
		loaded_scene.toImageMsg(scene_msg_);
	} catch (std::exception& e) {
		std::cerr << "Unhandled Exception reached the top of main: "
			<< e.what() << ", application will now exit\n";
		return;
	}
	pub_.publish(scene_msg_);
	loop_rate.sleep();
}

bool Imgloader::sceneChangeServiceCallback(
	visor::call::Request  &req,
	visor::call::Response &res
){
	// here, change the member string variable the name of the changed scene
	ROS_WARN_STREAM("Scene has been changed" << req.call);
	res.resp = "Scene has been changed (sceneChange_server)";
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "imgloader_node");
    ros::NodeHandle nhp("~");
    std::string img_dir;
    nhp.param(
		std::string("scene_snap_filename"), 
		img_dir, 
		std::string("/home/ros/scene_snap/sequence2/")
	);
    Imgloader iload(img_dir);

	return 0;
}
//*/