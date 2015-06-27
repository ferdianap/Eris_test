#include <ros/ros.h>
#include <eris/hricase2.hpp>
#include <iostream>
#include <eris/file_operations.hpp>
#include <eris/encode_tmp_ffi.hpp>
#include <eris/tag.h>

const char *vinit[] = {"leftmost", "rightmost", "atleast", "other", "objs"};
std::vector<std::string> Case2::keywordsTemplate_(vinit, end(vinit));

template <class ForwardIterator>
  std::size_t min_element_index ( ForwardIterator first, ForwardIterator last )
{
  ForwardIterator lowest = first;
  std::size_t index = 0;
  std::size_t i = 0;
  if (first==last) return index;
  while (++first!=last) {
    ++i;
    if (*first<*lowest) {
      lowest=first;
      index = i;
    }
  }
  return index;
}

template <class ForwardIterator>
  std::size_t max_element_index ( ForwardIterator first, ForwardIterator last )
{
  ForwardIterator highest = first;
  std::size_t index = 0;
  std::size_t i = 0;
  if (first==last) return index;
  while (++first!=last) {
    ++i;
    if (*first>*highest) {
      highest=first;
      index = i;
    }
  }
  return index;
}

Case2::Case2() {
}

Case2::~Case2() {
}

bool Case2::init(std::vector<std::string> &ctx) {
	context_      = ctx;
	bool res      = true;
	cond_.nOther  = 0;
	cond_.nObjs   = 1; // default treat as 1 object
	cond_.atleast = false;
	if (!fileExist(MEMORY_DIR+DBFNAME)) {
		ROS_WARN_STREAM("file _DB.ffi does NOT exist! Terminating...");
		res = false;
	}
	
	if (stringExistsInVect("atleast",context_)) {
		cond_.atleast = true;
		context_ = removeStringInVect("atleast",context_);
	}
	
	// assign context_ to KWs or nonKWs
	for (int i = 0; i < context_.size(); ++i) {
		ROS_WARN_STREAM("PROCESSING  "+context_[i]+"...");
		std::string s         = "other";
		// assign the number of other objects required in a scene	
		if (context_[i].find(s) != std::string::npos) {
			std::string notherIDX = context_[i];
			std::string::size_type i = notherIDX.find(s);
			if (i != std::string::npos)
			   notherIDX.erase(i, s.length());
			cond_.nOther = atoi(notherIDX.c_str());
			ROS_WARN_STREAM("1. removing "+context_[i]);
			context_ = removeStringInVect(context_[i-1],context_);
		}
		// assign the number of objects required in a scene	
		s = "objs";
		if (context_[i].find(s) != std::string::npos) {
			std::string nObjs = context_[i];
			std::string::size_type i = nObjs.find(s);
			if (i != std::string::npos)
			   nObjs.erase(i, s.length());
			cond_.nObjs = atoi(nObjs.c_str());
			ROS_WARN_STREAM("2. removing "+context_[i]);
			context_ = removeStringInVect(context_[i-1],context_);
		}
		if (context_[i]=="rightmost") {
			cond_.rightmost = context_[i+1];
		}
		if (context_[i]=="leftmost") {
			cond_.leftmost = context_[i+1];
		}
		if ((context_[i]!=cond_.leftmost || context_[i]!=cond_.rightmost) &&
			!stringExistsInVect(context_[i],keywordsTemplate_) && (cond_.nOther == 0)
		){
			ROS_WARN_STREAM("adding "+context_[i]);
			cond_.objs.push_back(context_[i]);
		}
	}
	std::cout <<"atleast   = "<<boolToString(cond_.atleast) <<std::endl;
	std::cout <<"leftmost  = "<<cond_.leftmost<<std::endl;
	std::cout <<"rightmost = "<<cond_.rightmost<<std::endl;
	std::cout <<"nOther    = "<<intToString(cond_.nOther)<<std::endl;
	std::cout <<"nObjs     = "<<intToString(cond_.nObjs)<<std::endl;
	for (int i = 0; i < cond_.objs.size(); ++i) {
		std::cout <<"objs      = "<<cond_.objs[i]<<std::endl;
	}
	return res;
}

std::string Case2::processContextInputObject() {
	std::string ans("\n=== Object Familiarity ===");
	std::vector<std::string> familiarObjs; // for given context > 1
	std::vector<std::string>::iterator iter, iter2;
	eris::ffiDatabase db = read_ffi_db(DBFNAME);
	// check all the tags in the database
	for (iter = context_.begin(); iter != context_.end(); ++iter) {
		if (stringExistsInVect(*iter, db.known_tags)) {
			eris::tag tag = read_ffi_tag(*iter);
			if (iter == context_.begin()) {
				for (iter2 = tag.assoc_obj.begin(); iter2 != tag.assoc_obj.end(); ++iter2) {
					familiarObjs.push_back(*iter2);
				}
			} else {
				// update the familiarObjs using set_intersection with tag.assoc_obj
				std::vector<std::string> intersect;
				set_intersection(
					familiarObjs.begin(),familiarObjs.end(),
					tag.assoc_obj.begin(),tag.assoc_obj.end(),
                 	std::back_inserter(intersect)
                );
                familiarObjs = intersect;
			}
		}
	}
	// TODO: check if the filename of the SM data contains all the given context
	

	// send back the response
	ans += "\nObject associated w/ ";
	for (iter = context_.begin(); iter != context_.end(); ++iter) {
		ans += *iter+", ";
	}
	if (familiarObjs.empty()) {
		ans += "is NOT Familiar!";
	} else {	
		ans += "is Familiar! such as:";
		for (iter2 = familiarObjs.begin(); iter2 != familiarObjs.end(); ++iter2) {
			ans += "\n* "+*iter2;
		}
	}
	return ans;
}

std::string Case2::processContextInputScene() {
	std::string ans("\n\n=== Scene Familiarity ===");
	// check the keywords given, 
	// TODO NEXT: get a scene with the condition at each condition
	// each iteration, update the matchedScenes_
	// check em data
	const std::string em("SE_");
	std::vector<std::string> scenelist = getFileList(MEMORY_DIR, em);
	std::vector<std::string>::iterator iter;
	// for all em data
	for (iter = scenelist.begin(); iter != scenelist.end(); ++iter) {
		eris::episode epi = read_em(*iter); // read each episode
		// check condition each episode
		// check nObj with obj_count in the episode
		
		if (cond_.leftmost.empty() && cond_.rightmost.empty()) {
			std::vector<std::string> objInScene;
			bool objAmountRequired = (epi.obj_count == cond_.nObjs);
			if (cond_.atleast) {
				objAmountRequired = (epi.obj_count >= cond_.nObjs);
				// ROS_WARN_STREAM("Applying at least...");
			}
			if (cond_.nOther > 0) {
				objAmountRequired = (epi.obj_count == cond_.objs.size()+cond_.nOther);
				ROS_WARN_STREAM("epi.obj_count mustbe = "+intToString(cond_.objs.size())+"+"+intToString(cond_.nOther));
			}
			if (objAmountRequired) {
				ROS_WARN_STREAM(epi.label+" has "+intToString(epi.obj_count)+" objs!");
				objInScene = epi.obj_name;
				// store the vector of object_label
				int matchedTag = 0;
				for (int i = 0; i < epi.obj_count; ++i) { // for all objects
					ROS_WARN_STREAM("  reading "+objInScene[i]+"...");
					// read the epi.obj_name sm file and get the tags
					eris::smEntity obj = read_sm_entity(objInScene[i]);
					printYellow("\t\t\t\t  Tag: ");
					for (int j = 0; j < obj.tag.size(); ++j) { // for all retrieved tags
						//   read each tag .ffi file and store the temp assoc obj
						std::cout << obj.tag[j] << ",  ";
						if (stringExistsInVect(obj.tag[j], cond_.objs)) {
							matchedTag += 1;
						}
					}
					std::cout << std::endl;
				}
				// check if matchedTag <= cond_.nObjs
				std::cout << "MatchedTag ? cond_.nObjs -> "<<matchedTag<<" ? "<< cond_.nObjs << std::endl;
				// if (cond_.nOther == 0) {
					if (matchedTag == cond_.nObjs) {
						// matchedScenes_.push_back(epi.label);
						ans += "\n* "+epi.label;
					}
				// } else {
				// 	if (matchedTag > cond_.nObjs) {
				// 		ans += "\n* "+epi.label;
				// 	}
				// }
			}
		} else { // process to find the leftmost / rightmost
			std::vector<std::string> objInScene;
			bool objAmountRequired = true;//(epi.obj_count == cond_.nObjs);
			if (cond_.nObjs > 1) {
				objAmountRequired = (epi.obj_count == cond_.nObjs);
			}
			if (objAmountRequired) {
				ROS_WARN_STREAM(epi.label+" has "+intToString(epi.obj_count)+" objs!");
				objInScene = epi.obj_name;
				// store the vector of object_label
				int matchedTag      = 0;
				int objIDX_L  = -1;
				std::string objName_L("");
				int objIDX_R = -1;
				std::string objName_R("");
				for (int i = 0; i < epi.obj_count; ++i) { // for all objects
					ROS_WARN_STREAM("  reading "+objInScene[i]+"...");
					// read the epi.obj_name sm file and get the tags
					eris::smEntity obj = read_sm_entity(objInScene[i]);
					printYellow("\t\t\t\t  Tag: ");
					for (int j = 0; j < obj.tag.size(); ++j) { // for all retrieved tags
						//   read each tag .ffi file and store the temp assoc obj
						std::cout << obj.tag[j] << ",  ";
						if (stringExistsInVect(obj.tag[j], cond_.objs) &&
							(obj.tag[j]==cond_.leftmost || obj.tag[j]==cond_.rightmost)) {
							matchedTag += 1;
							ROS_WARN_STREAM("epi.obj_name["+intToString(i)+"] is the leftmost/right");
							// get index of the object in a scene
							if (obj.tag[j]==cond_.leftmost) {
								objIDX_L = i;
								objName_L = obj.label;
							}
							if (obj.tag[j]==cond_.rightmost) {
								objIDX_R = i;
								objName_R = obj.label;
							}
						}
					}
					std::cout << std::endl;
				}
			
				ROS_WARN_STREAM(epi.label+" objIDX L & R = "+intToString(objIDX_L)+" ("+objName_L+")"+", "+intToString(objIDX_R)+" ("+objName_R+")");
				// at tis point, we got the object index in the scene that matcched the criteria
				// next, we have to find the minimum position out of all objects in a scene
				// and get the index of the smallest minx (leftmost) or largest maxx (rightmost)
				int minIDX, maxIDX;
				if (!cond_.leftmost.empty()) {
					minIDX = min_element_index(epi.pos_minx.begin(), epi.pos_minx.end());
					ROS_WARN_STREAM("minIDX = "+intToString(minIDX));
					// then we compare the object name [idx] with either objName_L or objName_R
					// if true, then ans += "\n* "+epi.label;
					if (epi.obj_name[minIDX] == objName_L) {
						ans += "\n* "+epi.label;
					}
				}
				if (!cond_.rightmost.empty()) {
					maxIDX = max_element_index(epi.pos_maxx.begin(), epi.pos_maxx.end());
					ROS_WARN_STREAM("maxIDX = "+intToString(maxIDX));
					if (epi.obj_name[maxIDX] == objName_R) {
						ans += "\n* "+epi.label;
					}
				}
			}
			// if (matchedTag == cond_.nObjs) {
			// 	// matchedScenes_.push_back(epi.label);
			// 	ans += "\n* "+epi.label;
			// }
		}
	}


	// // Check if nonKWs.size() != 0
	// if (nonKWs_.size() > 0) { // there are nonKWs
	// 	updateMatchedScenes();
	// 	// get scene with the condition(atleast,no)
	// 	// process and update the result into matchedScenes_
	// }
	

	return ans;
}

// void Case2::processMatchedScenes(std::string &kw, std::string &nonkw) {
// 	// find matched scenes for each conditions
// 	ROS_WARN_STREAM("ccccc "+intToString(cond_.nOther)+"  "+intToString(cond_.nObjs));

// 	// append to the matchedScenes_ vector
// }


// /**
//  * update matched scenes with the remaining nonKWs_ item
//  */
// void Case2::updateMatchedScenes() {

// }

std::string Case2::answerQuestion(int q) {
	std::string ans("");
	switch (q) {
	case 2:
		break;
	default: {
		ans += processContextInputObject();
		ans += processContextInputScene();
		break;
	}
	}
	return ans;
}