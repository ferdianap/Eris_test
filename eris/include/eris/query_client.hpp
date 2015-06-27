#ifndef QCLIENT_HPP_
#define QCLIENT_HPP_

namespace po = boost::program_options;

namespace {
  const size_t ERROR_IN_COMMAND_LINE = 1; 
  const size_t SUCCESS = 0; 
  const size_t ERROR_UNHANDLED_EXCEPTION = 2;  
  bool VERBOSE = false;
}

static const std::string ctx[] = {"color", "pos", "shape", "count"};
static const int numOfCtx = sizeof(ctx)/sizeof(*ctx);

// bool contextIsGiven(po::variables_map v);

//inline bool confirmIsRequested(bool );

#endif