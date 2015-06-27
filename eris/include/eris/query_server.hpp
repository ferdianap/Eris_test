#ifndef QSERVER_HPP_
#define QSERVER_HPP_

#include <eris/query.h>

bool query_process(
	eris::query::Request  &req,
	eris::query::Response &res
);

void getResultList (
	eris::query::Request  &req,
	eris::query::Response &res
);
#endif