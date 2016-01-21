#ifndef FILE_OPERATIONS_HPP_
#define FILE_OPERATIONS_HPP_

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/progress.hpp>
#include <fstream>
#include <vector>

#include <eris/smEntity.h>
#include <eris/ffiDatabase.h>
#include <eris/tag.h>
#include <eris/episode.h>
//  As an example program, we don't want to use any deprecated features
#define BOOST_FILESYSTEM_NO_DEPRECATED

#include <sys/stat.h>

// Directory of the Persistent Memories in HDD
static std::string MEMORY_DIR = "/home/ferdee/memtest/";
static std::string FFI_DB_DIR = "/home/ferdee/FFI_DB/";
static std::string INBOXFNAME = "_INBOX.ffi";
static std::string OBJ        = "OBJ_";
static std::string DBFNAME    = "_DB.ffi";

namespace fs = boost::filesystem;

// currently using this one
inline bool fileExist (const std::string& name) {
	struct stat buffer;   
	return (stat (name.c_str(), &buffer) == 0); 
}

// inline bool FileExist( const std::string& Name ) {
// 	return boost::filesystem::exists(Name);
// }

std::string gen_rand_name (const int len);


inline char getUserInput() {
	char in[1];
	fgets(in, sizeof(in), stdin);
  //gets(in);
	return in[0];
}

inline bool confirmNotProceed() {
	char in = getUserInput();
	return in!='y' && in!='Y' ? true : false;
}

// THIS METHOD IS CASE SENSITIVE !!
/*inline bool isFound(std::string &target, std::string keyword)
{
	return (target.find(keyword)!=std::string::npos ? true : false);
}*/

/*
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

std::vector<std::string> getFileList(
	const std::string &PATH,
	const std::string &FILENAME
	);

// EPISODIC MEMORY
// This method load all the contents of a desired episodic memory data file
eris::episode read_em( const std::string& filename );

void write_em( const std::string& filename, const eris::episode& e_msg );


// SEMANTIC MEMORY

// Read the whole contents
eris::smEntity read_sm_entity( const std::string& filename );

void write_sm_entity( const std::string& filename, const eris::smEntity& e_msg );

// Recall one specific value based on a given cue of a filename
//std::string recall_sm_entity( std::string& filename, std::string& cue);


eris::tag read_ffi_tag( const std::string& filename );
void write_ffi_tag(
	const std::string& filename,
	const eris::tag& e_msg
);

eris::ffiDatabase read_ffi_db( const std::string& filename );
void write_ffi_db(
	const std::string& filename,
	const eris::ffiDatabase& e_msg
);

#endif
