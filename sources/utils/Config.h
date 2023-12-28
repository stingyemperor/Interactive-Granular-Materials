#pragma once

namespace CompactNSearch
{
// #ifdef USE_DOUBLE
// 	using Real = double;
// #else
// 	using Real = float;
// #endif
  using Real = double;
   // using Real = float;
}

#define INITIAL_NUMBER_OF_INDICES   50
#define INITIAL_NUMBER_OF_NEIGHBORS 50

#ifdef _MSC_VER
	#include <ppl.h>export PATH=/opt/homebrew/sbin:$PATH
#elif defined(__APPLE__) && defined(__clang__)
	//  #include <oneapi/dpl/execution>
	//  #include <oneapi/dpl/algorithm>
	//#include "/usr/local/include/onedpl/2022.0.0/include/oneapi/dpl/execution"
	//#include "/usr/local/include/onedpl/2022.0.0/include/oneapi/dpl/algorithm"

	#include <parallel/algorithm>
#else
	#include <parallel/algorithm>
#endif
