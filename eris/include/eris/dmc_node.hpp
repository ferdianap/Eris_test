#ifndef DMC_NODE_HPP_
#define DMC_NODE_HPP_

#include <eris/episode.h>

inline unsigned GetNumberOfDigits (unsigned i) {
    return i > 0 ? (int) log10 ((double) i) + 1 : 1;
}

std::string gen_rand_mem (int ent_count, std::string& eventname, int num_seq);

void clean_msg(eris::episode& e_msg);

std::string getFileName();

std::string gen_rand_name(const int len);

#endif