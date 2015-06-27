#include <eris/encode_tmp_ffi.hpp>
#include <ros/ros.h>
#include <eris/console_pretty.hpp>
#include <eris/file_operations.hpp>

// Currently not used
int getL1Dist(
	const std::vector<int> &v1, 
	const std::vector<int> &v2
){
	assert(v1.size() == v2.size());
	int l1_dist(0);
	for (unsigned int i=0; i < v1.size(); i++) {
		l1_dist += (int) abs(v1[i]-v2[i]);
	}
	//printWhite("l1_dist = "+intToString(l1_dist)+"\n");
	return l1_dist;
}

float getCosineSimilarityDist(
	const std::vector<int> &v1, 
	const std::vector<int> &v2
){
	// ROS_WARN_STREAM(intToString(v1.size())+"x"+intToString(v2.size()));
	assert(v1.size() == v2.size());
    float sum0(0);
    float sum1(0);
    float sum2(0);

    for (unsigned int i = 0; i < v1.size(); i++) {
        sum0 += v1[i] * v2[i];
        sum1 += v1[i] * v1[i];
        sum2 += v2[i] * v2[i];
    }
	return 100 * sum0 / (sqrt(sum1) * sqrt(sum2)); // in percentage
}

float color_check(
	const std::vector<int> &color_sensed,
	const std::vector<int> &color_recalled
){
	return getCosineSimilarityDist(color_sensed, color_recalled);
}

float texture_check(
	const std::vector<int> &texture_sensed,
	const std::vector<int> &texture_recalled
){
	return getCosineSimilarityDist(texture_sensed, texture_recalled);
}

void revise_sm_entity(
	std::vector<std::string> parsedTag,
	const std::string& newlabel,
	eris::smEntity& smdata,
	const std::string& oldfname
){
	eris::smEntity ent;
	std::string newfname = newlabel;
	ent.header.stamp     = smdata.header.stamp;
	ent.label            = newlabel;
	for (int i=0; i<parsedTag.size(); i++) {
		ent.tag.push_back(std::string(parsedTag[i]));
	}
	ent.image   = smdata.image;
	ent.texture = smdata.texture;
	ent.color   = smdata.color;
	//ent.shape = smdata.shape; // TODO: delete this!
	write_sm_entity(newfname, ent);
	printWhite("removing "+oldfname+"...\n");
	std::string oldfnameloc = MEMORY_DIR+oldfname;
	if( std::remove(oldfnameloc.c_str()) != 0 )
		printRed( "Error deleting file\n");
	else
		printGreen( "File successfully deleted\n" );		
}

void revise_em_episode(const std::string& newlabel, const std::string& oldfname) {
	// read db.ffi and get the known_objects and known_tags
	const std::string fname = "SE_";
	std::string oldObjLabel = rmExt(rmOBJ(oldfname));
	std::vector<std::string> scenelist = getFileList(MEMORY_DIR, fname);
	std::vector<std::string>::iterator iter;
	eris::episode epi;
	for (iter = scenelist.begin(); iter != scenelist.end(); ++iter) {
		epi = read_em(*iter); // read each episode
		// for each obj_name, check if is the same with 
		bool any_changes(false);
		for (int i=0; i<epi.obj_name.size();i++) {
			std::cout << "epi.obj_name["<< i<< "] = "<< epi.obj_name[i]<<std::endl;
			std::cout << "newlabel = " << newlabel<<std::endl;
			if (isFound(epi.obj_name[i], oldObjLabel)) { 
				epi.obj_name[i] = newlabel; // modify only the obj_name
				std::cout << "new epi.obj_name["<< i<< "] = "<< epi.obj_name[i]<<std::endl;
				any_changes = true;
			}
		}
		if (any_changes) write_em(*iter, epi);// rewrite episode
	}
}

bool substringExistInVect(
	const std::string& substr,
	std::vector<std::string>& v
){
	bool found = false;
	std::vector<std::string>::const_iterator cii;
    std::string temp;
    unsigned find = 0;
    static int n = 0;
    for(cii = v.begin(); cii != v.end(); cii++) {
        temp = *cii;
        std::string ::size_type pos = 0;
        while( (pos = temp.find( substr, pos )) 
                 != std::string::npos ) 
        {
            n++;
            pos += substr.size();
        }
        if(n) found = true;
        /*else {
			if((cii == v.end() - 1) && !found) found = false;
        }*/
        n = 0;
    }
    return found;
}

std::vector<std::string> removeStringInVect(
	std::string s,
	std::vector<std::string> v
){
	v.erase( std::remove(v.begin(), v.end(), s), v.end() );
	return v;
}

void update_ffi_db(
	std::vector<std::string>& newTags,
	const std::string& newLabel,
	const std::string& oldfname
){
	const std::string fname = "_DB.ffi";
	eris::ffiDatabase database;
	if (fileExist(MEMORY_DIR+fname)) {
		ROS_WARN("file _DB.ffi already exist! Loading file...");
		database = read_ffi_db(fname);
	}
	for (int i=0; i< newTags.size(); i++) { // for each new tag
		// check if tag is exist in loaded database
		if(!stringExistsInVect(newTags[i], database.known_tags)) {
			database.known_tags.push_back(newTags[i]);
		}
		if(!stringExistsInVect(newLabel, database.known_objects)) {
			database.known_objects.push_back(newLabel);
		}
	}
	write_ffi_db(fname, database);
	ROS_WARN("_DB.ffi written successfully!");
	// read the _INBOX.ffi file
	std::string inboxfname = "_INBOX.ffi";
	eris::tag inboxTag = read_ffi_tag(inboxfname);
	std::string oldObjLabel = rmExt(rmOBJ(oldfname));
	
	//ROS_ERROR_STREAM(rmOBJ(oldfname)+" "+inboxTag.assoc_obj[0]+" "+inboxTag.assoc_obj[1]); // TODO: delete
	
	// find the oldObjLabel, delete
	if(stringExistsInVect(oldObjLabel, inboxTag.assoc_obj)) { // ERROR HERE!
		std::cout << oldObjLabel << " exist in inboxTag.assoc_obj\n";
		inboxTag.assoc_obj = removeStringInVect(oldObjLabel, inboxTag.assoc_obj);
		//inboxTag.assoc_obj.erase( std::remove(inboxTag.assoc_obj.begin(), inboxTag.assoc_obj.end(), oldObjLabel), inboxTag.assoc_obj.end() ); // TODO: DELETE THIS
		write_ffi_tag(inboxfname, inboxTag);
	}
}

std::string intToString(int i) {
    std::stringstream ss;
    std::string s;
    ss << i;
    s = ss.str();

    return s;
}

std::string floatToString (float number) {
    std::ostringstream buff;
    buff<<number;
    return buff.str();
}