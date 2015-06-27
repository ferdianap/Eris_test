#include <eris/file_operations.hpp>
#include <eris/console_pretty.hpp>

/*
 * This method generates random alphanumeric string,
 *   given the string length.
 */
std::string gen_rand_name (const int len) {
	std::string s = "";
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";

    for (int i=0; i<len; ++i) {
    	char c = alphanum[rand() % (sizeof(alphanum) - 1)];
        s.append(std::string(1,c));
    }

   	return s;
}

/*
 * This method retrieves all the files list of a give directory.
 */
std::vector<std::string> getFileList(
	const std::string &PATH,
	const std::string &FILENAME
){
	std::vector<std::string> filtered_list;
//	boost::progress_timer t( std::clog ); // uncomment for time to scan dir
	fs::path full_path( fs::initial_path<fs::path>() );

	full_path = fs::system_complete( fs::path( PATH ) );

	unsigned long file_count  = 0;
	unsigned long dir_count   = 0;
	unsigned long other_count = 0;
	unsigned long err_count   = 0;

	assert(fs::exists(full_path));

	if (fs::is_directory(full_path)) {
		// std::cout << "\nIn directory: " << full_path.string() << "\n\n";
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
						// std::cout << fname << " [directory]\n";
					} else if (fs::is_regular_file(dir_itr->status())) {
						++file_count;
			  			// std::cout << fname << "\n";
						filtered_list.push_back(fname);
					} else {
				  		++other_count;
				  		// std::cout << fname << " [other]\n";
					}
		      	}
	      	} catch (const std::exception & ex) {
	        	++err_count;
	        	std::cout << fname << " " << ex.what() << std::endl;
	      	}
	    }
	    // std::cout << "\n" << file_count << " files\n"
	    //           << dir_count 			<< " directories\n"
	    //           << other_count 		<< " others\n"
	    //           << err_count 			<< " errors\n";
	} else { // must be a file
		std::cout << "\nFound: " << full_path.string() << "\n";    
	}
	return filtered_list;
}

//***************** EPISODIC MEMORY *********************

/*
 * This method load all the contents
 * of a desired episodic memory data file
 *
 */
eris::episode read_em( const std::string& filename ) {
	eris::episode decoding_;
	std::string pwn(MEMORY_DIR);
	pwn.append(filename);
	if (!isFound(filename, ".em"))	pwn.append(".em");
	const char * pathWithName = pwn.c_str();
	try {
		std::ifstream ifs(pathWithName, std::ios::in|std::ios::binary);
		ifs.seekg (0, std::ios::end);
		std::streampos end = ifs.tellg();
		ifs.seekg (0, std::ios::beg);
		std::streampos begin = ifs.tellg();

		uint32_t file_size = end-begin;
		boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
		ifs.read((char*) ibuffer.get(), file_size);
		ros::serialization::IStream istream(ibuffer.get(), file_size);
		ros::serialization::deserialize(istream, decoding_ );
		ifs.close();
	} catch (ros::serialization::StreamOverrunException &e) {
		std::cout << e.what() << " File with name ";
		printRed(filename);
		std::cout << " not found!\n";
	}
	return decoding_;
}


/*
 * Consolidate (Encode) Episodic Memory MSG to HDD
 *
 */
void write_em( const std::string& filename, const eris::episode& e_msg) {
	// Concat path and filename
	std::string pwn(MEMORY_DIR);
	pwn.append(filename);
	if (!isFound(filename, ".em"))	pwn.append(".em");
	const char * pathWithName = pwn.c_str();
	// Do the encoding
	std::ofstream ofs(pathWithName, std::ios::out|std::ios::binary);

	// pass by pointer: can be changed. *e_msg for eris::smEntity::ConstPtr&
	uint32_t serial_size = ros::serialization::serializationLength(e_msg);
	boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);
	ros::serialization::OStream ostream(obuffer.get(), serial_size);
	ros::serialization::serialize(ostream, e_msg); // this is pass by ref. constant.
	ofs.write((char*) obuffer.get(), serial_size);
	ofs.close();
}


//***************** SEMANTIC MEMORY *********************
eris::smEntity read_sm_entity( const std::string& filename ) {
	eris::smEntity decoding_;
	std::string pwn(MEMORY_DIR);
	pwn.append(filename);
	if (!isFound(filename, ".sm"))	pwn.append(".sm");
	const char * pathWithName = pwn.c_str();
	try {
		std::ifstream ifs(pathWithName, std::ios::in|std::ios::binary);
		ifs.seekg (0, std::ios::end);
		std::streampos end = ifs.tellg();
		ifs.seekg (0, std::ios::beg);
		std::streampos begin = ifs.tellg();

		uint32_t file_size = end-begin;
		boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
		ifs.read((char*) ibuffer.get(), file_size);
		ros::serialization::IStream istream(ibuffer.get(), file_size);
		ros::serialization::deserialize(istream, decoding_ );
		ifs.close();
	} catch (ros::serialization::StreamOverrunException &e) {
		std::cout << "File with name ";
		printRed(filename);
		std::cout << " not found!\n";
	}
	return decoding_;
}

void write_sm_entity(
	const std::string& filename,
	const eris::smEntity& e_msg
){
	std::string pwn(MEMORY_DIR);
	pwn.append(filename);
	if (!isFound(filename, ".sm"))	pwn.append(".sm");
	const char * pathWithName = pwn.c_str();
	std::ofstream ofs(pathWithName, std::ios::out|std::ios::binary);

	uint32_t serial_size = ros::serialization::serializationLength(e_msg);
	boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);
	ros::serialization::OStream ostream(obuffer.get(), serial_size);
	ros::serialization::serialize(ostream, e_msg);
	ofs.write((char*) obuffer.get(), serial_size);
	ofs.close();
}

//***************** Familiarity TAG *********************
void write_ffi_tag(
	const std::string& filename,
	const eris::tag& e_msg
){
	std::string pwn(MEMORY_DIR);
	pwn.append(filename);
	if (!isFound(filename, ".ffi"))	pwn.append(".ffi");
	const char * pathWithName = pwn.c_str();
	std::ofstream ofs(pathWithName, std::ios::out|std::ios::binary);

	uint32_t serial_size = ros::serialization::serializationLength(e_msg);
	boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);
	ros::serialization::OStream ostream(obuffer.get(), serial_size);
	ros::serialization::serialize(ostream, e_msg);
	ofs.write((char*) obuffer.get(), serial_size);
	ofs.close();
}


eris::tag read_ffi_tag( const std::string& filename ) {
	eris::tag decoding_;
	std::string pwn(MEMORY_DIR);
	pwn.append(filename);
	if (!isFound(filename, ".ffi"))	pwn.append(".ffi");
	const char * pathWithName = pwn.c_str();
	try {
		std::ifstream ifs(pathWithName, std::ios::in|std::ios::binary);
		ifs.seekg (0, std::ios::end);
		std::streampos end = ifs.tellg();
		ifs.seekg (0, std::ios::beg);
		std::streampos begin = ifs.tellg();

		uint32_t file_size = end-begin;
		boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
		ifs.read((char*) ibuffer.get(), file_size);
		ros::serialization::IStream istream(ibuffer.get(), file_size);
		ros::serialization::deserialize(istream, decoding_ );
		ifs.close();
	} catch (ros::serialization::StreamOverrunException &e) {
		std::cout << "File with name ";
		printRed(filename);
		std::cout << " not found!\n";
	}

	return decoding_;
}

//***************** Familiarity DATABASE *********************
void write_ffi_db(
	const std::string& filename,
	const eris::ffiDatabase& e_msg
){
	std::string pwn(MEMORY_DIR);
	pwn.append(filename);
	if (!isFound(filename, ".ffi"))	pwn.append(".ffi");
	const char * pathWithName = pwn.c_str();
	std::ofstream ofs(pathWithName, std::ios::out|std::ios::binary);

	uint32_t serial_size = ros::serialization::serializationLength(e_msg);
	boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);
	ros::serialization::OStream ostream(obuffer.get(), serial_size);
	ros::serialization::serialize(ostream, e_msg);
	ofs.write((char*) obuffer.get(), serial_size);
	ofs.close();
}


eris::ffiDatabase read_ffi_db( const std::string& filename ) {
	eris::ffiDatabase decoding_;
	std::string pwn(MEMORY_DIR);
	pwn.append(filename);
	if (!isFound(filename, ".ffi"))	pwn.append(".ffi");
	const char * pathWithName = pwn.c_str();
	try {
		std::ifstream ifs(pathWithName, std::ios::in|std::ios::binary);
		ifs.seekg (0, std::ios::end);
		std::streampos end = ifs.tellg();
		ifs.seekg (0, std::ios::beg);
		std::streampos begin = ifs.tellg();

		uint32_t file_size = end-begin;
		boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
		ifs.read((char*) ibuffer.get(), file_size);
		ros::serialization::IStream istream(ibuffer.get(), file_size);
		ros::serialization::deserialize(istream, decoding_ );
		ifs.close();
	} catch (ros::serialization::StreamOverrunException &e) {
		std::cout << "File with name ";
		printRed(filename);
		std::cout << " not found!\n";
	}

	return decoding_;
}

