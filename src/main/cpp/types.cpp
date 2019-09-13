/*
 * types.cpp
 *
 *  Created on: Oct 18, 2018
 *      Author: koldar
 */


#include "types.hpp"
#include <cpp-utils/log.hpp>
#include <cmath>

// std::ostream& operator<<(std::ostream& stream, const dpf::move_t& m) {
// 	stream << static_cast<int>(m);
// 	return stream;
// }


//std::ostream& operator<<(std::ostream& stream, const long& l) {
//	int times = static_cast<int>(std::log10(static_cast<double>(l)))/3;
//	for( int i=0; i<times; ++i) {
//		stream << static_cast<int>((l/(std::pow(10, 3*i))));
//		if ((i+1)<times) {
//			stream << "'";
//
//		}
//	}
//}
