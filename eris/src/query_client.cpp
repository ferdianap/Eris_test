#include <ros/ros.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <eris/query.h>
#include <eris/query_client.hpp>
#include <eris/console_pretty.hpp>
#include <eris/file_operations.hpp>
#include <eris/OptionPrinter.hpp>


int main(int argc, char** argv) {
 	ros::init(argc, argv, "query_client");
	ros::NodeHandle n;
	ros::ServiceClient client;
	std::string SERVER("qserver");
	eris::query srv;
	int icase(1);
	int question(1);
  
	try {
		std::string appName = boost::filesystem::basename(argv[0]);
		std::vector<std::string> lexinfo;
		/*********************************************************
		 * Program options definition and parsing
		 *********************************************************/
		
		po::options_description desc("Options");
		desc.add_options()
		("help,h", "Print help messages")
		("case,c", po::value<int>(&icase)->required(), "Interaction case")
		("question,q", po::value<int>(&question)->required(), "Asked question")
		("lexicalinfo,l", po::value<std::vector<std::string> >(&lexinfo), "enter multiple context")
		;
      
		po::variables_map vm;

		try {
			po::store(po::parse_command_line(argc, argv, desc), vm);  
			if (vm.count("help")) {
				std::cout << "Universal Client Program ver. 0.1.0\n\n"
					"Basic Command Line Parameter App\n" << desc << "\n"
					"Example query (case 3, question 2, with context):\n"
					"rosrun eris query_client -c 3 -q 2 -l orange -l round\n"
					"Valid Case:\n"
					"  1. Image only\n  2. Context only\n  3. Image+Context\n\n"
					"Make sure that the `vision.JPG` image is located in the folder `BaxterVision`. (This serves as an alternative for live visual stimuli during HRI.)\n"
					"Choose question to ask Baxter:\n";
				printYellow("<img>     = valid for Case 1 & 3,\n");
				printBlue("<ctx>     = valid for Case 2,\n");
				printMagenta("<img/ctx> = valid for all cases.\n\n");
				printMagenta("  1. Are you familiar w/ <img/ctx>?\n");
				printYellow("  2. What is/are <img>? (also works for multiple objects)\n");
				printMagenta("  3. What kind of <img/ctx> did you presented with?\n");
				printMagenta("  4. How many of <img/ctx> have you seen so far?\n");
				printMagenta("  5. Did you remove any of <img/ctx>?\n");
				printMagenta("  6. Did you move any of <img/ctx>?\n");
				printMagenta("  7. How many objects left after you remove <img/ctx>?\n");
				printBlue("  8. What <ctx> object did you move?\n");
				printMagenta("  9. How many <img/ctx> at least were in the workspace?\n");
				return SUCCESS;
			}

			// throws on error, so do after help in case there are any problems
			po::notify(vm);
		} catch (boost::program_options::required_option& e) {
			std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
			return ERROR_IN_COMMAND_LINE;
		} catch (boost::program_options::error& e) {
			std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
			return ERROR_IN_COMMAND_LINE;
		}
		// Do sth here ------------------------------
		if (icase>1 && lexinfo.empty()) {
			printRed("Please provide context!\n");
			return ERROR_IN_COMMAND_LINE; // or use lexinfo.empty() if fail
		}
		if (icase>1) {}//SERVER="em_server";

		if (icase!=2) {
			// check the folder if image is available
			const std::string fn("vision.JPG");
			const std::string RBTVIS_DIR("/home/ros/scene_snap/BaxterVision/");
			std::string filestatus("file image exist!");
			if (!fileExist(RBTVIS_DIR+fn)) {
				filestatus = "file image NOT FOUND! Check if the File "+fn+" in the Directory "+RBTVIS_DIR+" exist.";
				printRed(filestatus);
				return ERROR_IN_COMMAND_LINE;
			}
		}

		srv.request.icase       = icase;
		srv.request.question    = question;
		srv.request.lexicalinfo = lexinfo;

		// std::vector<std::string>::iterator iter;
		// for (iter = srv.request.lexicalinfo.begin();
		// 	iter != srv.request.lexicalinfo.end(); ++iter) {
		// 	std::cout << *iter << std::endl;
		// }
		// -------------------------------------------
	} catch (std::exception& e) {
		std::cerr << "Unhandled Exception reached the top of main: "
			<< e.what() << ", application will now exit\n";
    	return ERROR_UNHANDLED_EXCEPTION;
	}

	client = n.serviceClient<eris::query>(SERVER);
	if (!client.call(srv)) {
		ROS_ERROR_STREAM("Failed to call service " << SERVER );
		return ERROR_IN_COMMAND_LINE;
	} else {
		if (srv.response.multi_responses.empty()) {
			printRed("No results found.\n");
			return SUCCESS;
		}
		
		printWhite("Baxter's responses:\n");
		for (int i=0; i<srv.response.multi_responses.size(); i++) {
			if (!srv.response.multi_responses[i].empty()) {
				std::string tab(" ");
				if (i==0) tab = ">";
				printRed(tab+" ");
				printGreen(srv.response.multi_responses[i]+"\n");
			}
		}
		printf("[============> END OF QUERY <============]\n");
	}
	return SUCCESS;
}

// bool contextIsGiven(po::variables_map v){ // count and pos TODO: pos is not yet implemented
// 	return v.count("count") ? true : false; 
// }