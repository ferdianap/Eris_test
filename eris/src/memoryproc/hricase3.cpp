#include <ros/ros.h>
#include <eris/hricase3.hpp>
#include <iostream>
#include <visor/MPEG7FexLib_src_OpenCV22/Feature.h>
#include <eris/tag.h>
#include <eris/file_operations.hpp>
#include <eris/encode_tmp_ffi.hpp>
#include <eris/console_pretty.hpp>

template<typename T, size_t N>
T * end(T (&ra)[N]) {
    return ra + N;
}

const char *vinit3[] = {"leftmost", "rightmost", "no_"};
std::vector<std::string> Case3::keywordsTemplate_(vinit3, end(vinit3));

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

Case3::Case3()
	: colorThres_(30)
	, textureThres_(30)
{
}

Case3::~Case3() {
}

bool Case3::init(std::vector<std::string> &ctx) {
	context_      = ctx;
	bool res      = true;
	colorFeats_.clear();
	textureFeats_.clear();
	objsFam_.clear();
	if (!fileExist(MEMORY_DIR+DBFNAME)) {
		ROS_WARN_STREAM("file _DB.ffi does NOT exist! Terminating...");
		res = false;
	}
	for (int i = 0; i < context_.size(); ++i) {
		ROS_WARN_STREAM("PROCESSING  "+context_[i]+"...");
		std::string s = "no_";
		if (context_[i].find(s) != std::string::npos) {
			std::string no_ = context_[i];
			std::string::size_type i = no_.find(s);
			if (i != std::string::npos)
			   no_.erase(i, s.length());
			cond_.no_ = no_;
			// ROS_WARN_STREAM("2. removing "+context_[i]);
			// context_ = removeStringInVect(context_[i],context_);
		}
		if (context_[i]=="rightmost") {
			cond_.rightmost = context_[i+1];
		}
		if (context_[i]=="leftmost") {
			cond_.leftmost = context_[i+1];
		}
		ROS_WARN_STREAM("context_[i] = "+context_[i]+" == "+cond_.rightmost);
		if ((context_[i]!=cond_.leftmost || context_[i]!=cond_.rightmost) &&
			!stringExistsInVect(context_[i],keywordsTemplate_) && (context_[i]!="no_"+cond_.no_)
		){
			ROS_WARN_STREAM("adding "+context_[i]);
			cond_.objs.push_back(context_[i]);
		}
	}
	std::cout <<"leftmost  = "<<cond_.leftmost<<std::endl;
	std::cout <<"rightmost = "<<cond_.rightmost<<std::endl;
	std::cout <<"no_ = " << cond_.no_<<std::endl;
	for (int i = 0; i < cond_.objs.size(); ++i) {
		std::cout <<"objs      = "<<cond_.objs[i]<<std::endl;
	}
	return res;
}

// This method is the same with Case2
std::string Case3::processContextInputObject() {
	std::string ans("\n=== Object Familiarity ===");
	std::vector<std::string> relatedObjs;
	std::vector<std::string>::iterator iter, iter2;
	eris::ffiDatabase db = read_ffi_db(DBFNAME);
	for (iter = context_.begin(); iter != context_.end(); ++iter) {
		if (stringExistsInVect(*iter, db.known_tags)) {
			eris::tag tag = read_ffi_tag(*iter);
			if (iter == context_.begin()) {
				for (iter2 = tag.assoc_obj.begin(); iter2 != tag.assoc_obj.end(); ++iter2) {
					relatedObjs.push_back(*iter2);
				}
			} else {
				// update the relatedObjs using set_intersection with tag.assoc_obj
				std::vector<std::string> intersect;
				set_intersection(
					relatedObjs.begin(),relatedObjs.end(),
					tag.assoc_obj.begin(),tag.assoc_obj.end(),
                 	std::back_inserter(intersect)
                );
                relatedObjs = intersect;
			}
		}
	}

	printf("%d, %d",colorFeats_.size(), textureFeats_.size());
	// at this point, we have the related objects in relatedObjs
	assert(colorFeats_.size() == textureFeats_.size());
	for (int i = 0; i < colorFeats_.size(); ++i) { 
		object_result objRes;
		objRes.is_familiar = false;
		const std::vector<int> color_sensed = colorFeats_[i];
		const std::vector<int> texture_sensed = textureFeats_[i];
		for (iter = relatedObjs.begin(); iter != relatedObjs.end(); ++iter) {
			bool is_color_familiar = false;
			bool is_texture_familiar = false;
			// read the file and check
			eris::smEntity obj = read_sm_entity(*iter);
			std::vector<int> color_recalled(obj.color.begin(), obj.color.end());
			std::vector<int> texture_recalled(obj.texture.begin(), obj.texture.end());
			// check if texture and color mathced familiar with colorFeats_, textureFeats_;
			float color_similarity = color_check(color_sensed, color_recalled);
			if (color_similarity > colorThres_) is_color_familiar = true;
			float texture_similarity = texture_check(texture_sensed, texture_recalled);
			if (texture_similarity > textureThres_) is_texture_familiar = true;
			if (is_color_familiar && is_texture_familiar) {
				objRes.is_familiar = true;
				objRes.label.push_back(obj.label);
				objRes.color_similarity.push_back(color_similarity);
				objRes.texture_similarity.push_back(texture_similarity);
			}
		}
		objsFam_.push_back(objRes);
	}

	// send back the response
	ans += "\nPresented object associated w/ ";
	for (iter = context_.begin(); iter != context_.end(); ++iter) {
		ans += *iter+", ";
	}
	if (relatedObjs.empty()) {
		ans += "is NOT Familiar!";
	} else {	
		ans += "is Familiar! such as:";
		for (int i=0; i < objsFam_.size(); ++i) {
			for (int j=0; j<objsFam_[i].label.size(); ++j) {
				ans += "\n* "+objsFam_[i].label[j];
			}
		}
	}
	return ans;
}


std::string Case3::processContextInputScene() {
	std::string ans("\n\n=== Scene Familiarity ===");
	const std::string em("SE_");
	std::vector<std::string> scenelist = getFileList(MEMORY_DIR, em);
	std::vector<std::string>::iterator iter;
	for (iter = scenelist.begin(); iter != scenelist.end(); ++iter) {
		eris::episode epi = read_em(*iter);
		ROS_WARN_STREAM(epi.label+" has "+intToString(epi.obj_count)+" objs!");
		if (cond_.leftmost.empty() && cond_.rightmost.empty()) {
			std::vector<std::string> objInScene;
			objInScene = epi.obj_name;
			bool matchedScene = true;
			for (int i = 0; i < epi.obj_count; ++i) { // for all objects
				ROS_WARN_STREAM("  reading "+objInScene[i]+"...");
				eris::smEntity obj = read_sm_entity(objInScene[i]);
				printYellow("\t\t\t\t  Tag: ");
				for (int j = 0; j < obj.tag.size(); ++j) { // for all retrieved tags
					std::cout << obj.tag[j] << ",  ";
					bool ccc = (obj.tag[j].find(cond_.no_) != std::string::npos) &&
						(obj.label.find(cond_.no_) != std::string::npos );
					if (ccc) {
						matchedScene = false;
					}
				}
				std::cout << std::endl;
			}
			if (matchedScene) ans += "\n* "+epi.label;
		} else {
			// TODO: continue here with leftmost and rightmost
			std::vector<std::string> objInScene;
			bool objAmountRequired = (epi.obj_count == blobCnt_);
			if (objAmountRequired) {
				objInScene = epi.obj_name;
				int matchedTag      = 0;
				int objIDX_L  = -1;
				std::string objName_L("");
				int objIDX_R = -1;
				std::string objName_R("");
				for (int i = 0; i < epi.obj_count; ++i) { // for all objects
					ROS_WARN_STREAM("  reading "+objInScene[i]+"...");
					eris::smEntity obj = read_sm_entity(objInScene[i]);
					printYellow("\t\t\t\t  Tag: ");
					for (int j = 0; j < obj.tag.size(); ++j) {
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
		}
	}
	return ans;
}



std::string Case3::answerQuestion(int q) {
	std::string ans("");
	switch (q) {
	case 2: {
		break;
		}
	default: {
		ans += processContextInputObject();
		ans += processContextInputScene();
		}
		return ans;
	}
}