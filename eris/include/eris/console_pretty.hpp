#ifndef CONSOLE_PRETTY_HPP_
#define CONSOLE_PRETTY_HPP_

#include <string>
#include <eris/query.h>
#include <sstream>

inline std::string rmOBJ(std::string strin ) {
	return strin.erase(0,4);
}

// Remove extension
inline std::string rmExt(std::string strin) {
	return strin.erase(strin.find_last_of('.'));
}

inline void printBlack(std::string s) {
	std::cout<<"\033[1;30m" + s + "\033[0m";
}

inline void printRed(std::string s) {
	std::cout<< "\033[1;31m" + s + "\033[0m";
}

inline void printGreen(std::string s) {
	std::cout<< "\033[1;32m" + s + "\033[0m";
}

inline void printYellow(std::string s) {
	std::cout<< "\033[1;33m" + s + "\033[0m";
}

inline void printBlue(std::string s) {
	std::cout<< "\033[1;34m" + s + "\033[0m";
}

inline void printMagenta(std::string s) {
	std::cout<<"\033[1;35m" + s + "\033[0m";
}

inline void printCyan(std::string s) {
	std::cout<< "\033[1;36m" + s + "\033[0m";
}

inline void printWhite(std::string s) {
	std::cout<< "\033[1;37m" + s + "\033[0m";
}

inline void clearScreen() {
	std::cout << "\033[2J\033[1;1H";
}


#endif