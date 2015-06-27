/*
 * DUMMY MEMORY CREATOR
 *
 * This node is used as a standalone, manual memory consolidator.
 * Can be used as a read and write module.
 *
 * Keep in mind that the format of the message must be manually
 *   considered, and any detected discrepancies with msg file format
 *   will result in an error!
 *
 */

#include <ros/ros.h>
#include <stdlib.h> 
#include <eris/dmc_node.hpp>
#include <fstream>
#include <eris/file_operations.hpp>
#include <eris/console_pretty.hpp>
#include <ctype.h>


int main(int argc, char** argv) {
	ros::init(argc, argv, "dmc_node");
	ros::NodeHandle n;
	
	// Init random seed
	srand (time(NULL));

	try {
		std::string argtype(argv[1]);

		if (argtype=="-h") {
			printMagenta("*******************************\n");
			printMagenta("   Dummy Memory Creator Node   \n");
			printMagenta("-------------------------------\n");
			printWhite("-h\n  ");
			std::cout << "`-> Display Help.\n";
			printWhite("-c <obj_count> <num_seq> \n  ");
			std::cout << "`-> Create an EM with random params.\n";
			printWhite("-r <filename>.<ext> \n  ");
			std::cout << "`-> Read the provided filename\n";
			printMagenta("*******************************\n\n");
		} else if (argtype=="-c") {
			if (argv[2]==NULL) {
				ROS_WARN_STREAM("Format: -c <obj_count> <num_seq>!\n");
				return -1; // error
			}
			
			if (!isdigit(*argv[2]) || !isdigit(*argv[3])) {
				ROS_ERROR_STREAM("Inputs should be valid integers!\n");
				return -1;
			}

			int obj_count   	= atoi(argv[2]);
			int num_seq     	= atoi(argv[3]);

			if (obj_count < 0 || obj_count > 10 ||
				num_seq   < 0 || num_seq   > 5   ){
				ROS_WARN_STREAM("Range <obj_count>: 0-10, <num_seq>: 0-5!\n");
				return -1;
			}

			std::string event_name	= "Event ";

			// Generate random memory and WRITE to HDD
			std::string f = gen_rand_mem(obj_count, event_name, num_seq);
		} else if (argtype=="-r") {
			std::string readf(argv[2]);
			if (!isFound(readf,".em") && 
				!isFound(readf,".sm") && 
				!isFound(readf,".pm") &&
				!isFound(readf,".ffi")
				){
				ROS_WARN_STREAM("<ext>: sm, em or pm and ffi.\n");
				return -1;
			}
			if (isFound(readf,".sm")) {
				eris::smEntity ent_msg_d = read_sm_entity(readf);
				if (!ent_msg_d.label.empty()) {
					std::cout << "Label: ";
					printWhite(ent_msg_d.label+"\n");
					std::cout << "Color (CSD) = ";
					for(int i = 0; i < ent_msg_d.color.size(); i++) {
						std::cout << (int)ent_msg_d.color[i] << " ";
					}
					std::cout << std::endl;
					std::cout << "Texture (EHD) = ";
					for(int i = 0; i < ent_msg_d.texture.size(); i++) {
						std::cout << (int)ent_msg_d.texture[i] << " ";
					}
					std::cout << std::endl;
					std::cout << "Tags found: \n";
					for (int i=0;i<ent_msg_d.tag.size();i++) {
						printWhite(ent_msg_d.tag[i]+"\n");
					}
				}
			} else if (isFound(readf,".em")) {
				eris::episode epi_msg_d = read_em(readf);
				if (!epi_msg_d.label.empty()) ROS_WARN_STREAM(epi_msg_d);
			} else if (isFound(readf,".pm")) {
				// TODO: do something for PM here!
			} else if (isFound(readf,".ffi")) {
				if (isFound(readf,"_DB")) {
					eris::ffiDatabase c_msg_d = read_ffi_db(readf);
					if (!c_msg_d.known_objects.empty()) ROS_WARN_STREAM(c_msg_d); 
				} else {
					eris::tag c_msg_d = read_ffi_tag(readf);
					if (!c_msg_d.assoc_obj.empty()) ROS_WARN_STREAM(c_msg_d);
					else printYellow("_INBOX.ffi is empty. No tmp to be revised.\n");
				}
			}
		} else {
			ROS_WARN_STREAM("<arg>: -h, -c, or -r.\n");
			return -1;
		}
	} catch(...) {
		ROS_ERROR_STREAM("Use args. See -h for details.\n");
		return -1;
	}

	return 0;
}


std::string gen_rand_mem (
	int 			obj_count,
	std::string& 	eventname, 
	int 			num_seq
){
	eris::episode epi_msg_e;

	epi_msg_e.header.stamp = ros::Time::now();

	//=======================
	// Types of memory:
	// ID + <00001>.em
	// se = specific events
	// ge = general events
	// pf = personal facts
	// fm = flashbulb memory
	//=======================

	// Put Filename here (for now we set as a specific event `se`)
	std::string fname = "se"+gen_rand_name(5);

	// Put Event Name here
	epi_msg_e.label = eventname.append(fname);

	// this is also the array size of pos and color
	epi_msg_e.obj_count = obj_count;

	for (int i=0; i<obj_count; i++) {
		// Create single entity with random params
		// Create random number from 0 until 50
		epi_msg_e.pos_minx.push_back(rand() % 50);
		// Create random number from 50 until 100
		epi_msg_e.pos_maxx.push_back(rand() % 50 + 50);
		epi_msg_e.pos_miny.push_back(rand() % 50);
		epi_msg_e.pos_maxy.push_back(rand() % 50 + 50);
		// epi_msg_e.color_mean.push_back(
		//	static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
		//epi_msg_e.color_var.push_back(
		//	static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
		
		int s = rand() % 100;
		int c = rand() % 30;
		std::string shape, color;
		if 		(s<50)	shape = "box";   
		else			shape = "ball";  
		if 		(c<10)  color = "green"; 
		else if (c<20)  color = "blue";  
		else 		    color = "red";   
		epi_msg_e.obj_name.push_back(color+" "+shape);
	}
	
	epi_msg_e.seq_count = num_seq;

	for (int i=0; i<num_seq; i++) {
		int a = rand() % 100;
		int m = rand() % 100;
		std::string arm, movt;
		if 		(a<50)	arm = "left";             
		else			arm = "right";            
		if 		(m<10)  movt = "grasp";           
		else if (m<20)  movt = "release";         
		else if (m<30)  movt = "hand_open";       
		else if (m<40)  movt = "hand_close";      
		else if (m<50)  movt = "swivel";          
		else if (m<60)  movt = "swivel_grasp";    
		else if (m<70)  movt = "set_wristangle";  
		else if (m<80)  movt = "movealong_axis";  
		else if (m<90)  movt = "moveto_pos";      
		else 		    movt = "compound_movt";   
		epi_msg_e.sequence.push_back(arm+"_"+movt);
	}

	std::cout << epi_msg_e;

	printYellow("Write this memory? <y/N> ");
	char ans[1];
	gets(ans);
	
	if (ans[0]!='y' && ans[0]!='Y') {
		std::cout << "Canceled!\n\n";
	} else {
		// Write to HDD
		write_em(fname, epi_msg_e);

		printRed("------> SUCCESS!\n");
		std::cout << "A file named: ";
		printWhite(fname);
		std::cout << ".em has been created.\n\n";
	}
	return fname;
}
