/**
 * @file
 *
 * A file containing all importants types used i nthe project
 *
 *  Created on: Oct 17, 2018
 *      Author: koldar
 */

#ifndef _COMPRESSED_PATH_DATABASES_TYPES_HEADER__
#define _COMPRESSED_PATH_DATABASES_TYPES_HEADER__

#include <cstdint>
#include <iostream>
// #include "safe_math.h"

// namespace dpf {

// /**
//  * An Id of a warthog::search_node
//  */
// typedef uint32_t nodeid_t;
// /**
//  * an id representing a coordinate of a location (either x or y)
//  */
// typedef uint32_t coo2d_t;
// /**
//  * an integer representing a move to perform in a location.
//  *
//  * The moves allows the agent to go from the current location to the next one
//  */
// typedef unsigned char move_t;

// typedef safe_int cost_t;

// /**
//  * A really big (unsigned) integer. Does not provide support for overflow
//  */
// typedef unsigned long long big_integer;

// /**
//  * A really big (signed) integer. Does not provide support for overflow
//  */
// typedef long long big_sint;

// typedef int celltype_t;

// ///**
// // * a possible type of cell in a map
// // */
// //class celltype_t {
// //	friend bool operator ==(const dpf::celltype_t& a, const dpf::celltype_t& b);
// //	friend bool operator !=(const dpf::celltype_t& a, const dpf::celltype_t& b);
// //	friend dpf::celltype_t operator + (const dpf::celltype_t& a, const dpf::celltype_t& b);
// //	friend dpf::celltype_t operator - (const dpf::celltype_t& a, const dpf::celltype_t& b);
// //	friend dpf::celltype_t operator * (const dpf::celltype_t& a, const dpf::celltype_t& b);
// //	friend dpf::celltype_t operator / (const dpf::celltype_t& a, const dpf::celltype_t& b);
// //	friend dpf::celltype_t operator / (const dpf::celltype_t& a, double b);
// //private:
// //	unsigned char value;
// //public:
// //	celltype_t() : value{0} {
// //
// //	}
// //	celltype_t(unsigned char c) : value{c} {
// //
// //	}
// //	~celltype_t() {
// //
// //	}
// //
// //	bool isUntraversable() const { return value == 0; }
// //	bool isTraversable() const { return value > 0; }
// //};
// //
// //dpf::celltype_t operator + (const dpf::celltype_t& a, const dpf::celltype_t& b) {
// //	return dpf::celltype_t{static_cast<unsigned char>(a.value + b.value)};
// //}
// //
// //dpf::celltype_t operator - (const dpf::celltype_t& a, const dpf::celltype_t& b) {
// //	return dpf::celltype_t{static_cast<unsigned char>(a.value - b.value)};
// //}
// //
// //dpf::celltype_t operator * (const dpf::celltype_t& a, const dpf::celltype_t& b) {
// //	return dpf::celltype_t{static_cast<unsigned char>(a.value * b.value)};
// //}
// //
// //dpf::celltype_t operator / (const dpf::celltype_t& a, const dpf::celltype_t& b) {
// //	return dpf::celltype_t{static_cast<unsigned char>(a.value / b.value)};
// //}
// //
// //dpf::celltype_t operator / (const dpf::celltype_t& a, double b) {
// //	return dpf::celltype_t{static_cast<unsigned char>(a.value / b)};
// //}
// //
// //bool operator ==(const dpf::celltype_t& a, const dpf::celltype_t& b) {
// //	return a.value == b.value;
// //}
// //
// //bool operator !=(const dpf::celltype_t& a, const dpf::celltype_t& b) {
// //	return a.value != b.value;
// //}

// }

// std::ostream& operator<<(std::ostream& stream, const dpf::move_t& m);

#endif /* TYPES_H_ */
