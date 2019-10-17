/*
 * list_graph.cpp
 *
 *  Created on: Oct 2, 2018
 *      Author: koldar
 */

#include "list_graph.hpp"

//#include "mapper.h"
#include <functional>

namespace std {

size_t hash<compressed_path_database::datastructures::Arc>::operator()(const compressed_path_database::datastructures::Arc& k) const {
	return k.hash();
}

}

namespace compressed_path_database::datastructures {

std::size_t Arc::hash() const {
	std::hash<int> hasher;
	//TODO this isn't a particularly smart hash... performance wise it's terrible
	return hasher(this->source) + hasher(this->target) + hasher(this->weight);
}

// void ListGraph::print(const Mapper& mapper, const string& baseName) {
// 	string dotFilename{baseName};
// 	dotFilename += ".dot";
// 	FILE* f = fopen(dotFilename.c_str(), "w");
// 	if (f == NULL) {
// 		throw CpdException{"file %s cannot be created!", dotFilename};
// 	}

// 	fprintf(f, "graph {\n");
// 	for (int i=0; i<n; ++i) {
// 		//coordaintes in dot are in bottom left till top right
// 		fprintf(f, "N%04d [label=\"%d\", pos=\"%d,-%d!\"];\n", i, i, mapper(i).x*3, mapper(i).y*3);
// 	}

// 	for (auto it = arc.begin(); it!= arc.end(); ++it) {
// 		fprintf(f, "N%04d -- N%04d [label=\"%d\"];\n", it->source, it->target, it->weight);
// 	}

// 	fprintf(f, "}");

// 	fclose(f);

// 	std::stringstream ss;
// 	ss << "dot -Kneato -Tsvg -o " << baseName << ".svg " << baseName << ".dot";
// 	int errorCode = system(ss.str().c_str());
// 	if (errorCode != 0) {
// 		warning(ss.str(), "resulted in an error code of", errorCode, ". Continuing as nothing happened");
// 	}
// 	unlink(dotFilename.c_str());
// }

}
