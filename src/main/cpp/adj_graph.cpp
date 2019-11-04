/*
 * adj_graph.cpp
 *
 *  Created on: Oct 2, 2018
 *      Author: koldar
 */

#include "adj_graph.hpp"
//TODO remove #include "mapper.h"
#include <cpp-utils/operators.hpp>
//TODO remove #include "vec_io.h"
#include <cpp-utils/log.hpp>
//#include "log.h"

namespace compressed_path_database::datastructures {

bool OutArc::hasBeenPerturbated() const {
	return this->perturbated;
	//return this->weight!=1000 && this->weight!=1414;
}


bool operator==(const OutArc& thiz, const OutArc& other) {
	return thiz.target == other.target && thiz.weight == other.weight;
}

bool operator!=(const OutArc& thiz, const OutArc& other) {
	return !(thiz == other);
}

std::ostream& operator <<(std::ostream& stream, const OutArc& a) {
	stream << "arc{weight=" << a.weight << ", target=" << a.target;
	return stream;
}

AdjGraph::AdjGraph() : out_begin{}, out_arc{} {

}

AdjGraph::AdjGraph(const AdjGraph& g) : out_begin{g.out_begin}, out_arc{g.out_arc} {

}

AdjGraph::AdjGraph(ListGraph g) {
	build_adj_array(
			out_begin, out_arc,
			g.node_count(), g.arc.size(),
			[&](int x){return g.arc[x].source;},
			[&](int x){return OutArc{g.arc[x].target, g.arc[x].weight};}
	);
}

AdjGraph& AdjGraph::operator=(const ListGraph&o) {
	return *this = AdjGraph(o);
}

AdjGraph& AdjGraph::operator=(const AdjGraph& other) {
	this->out_arc.resize(other.out_arc.size());
	this->out_begin.resize(other.out_begin.size());

	this->out_arc = std::vector<OutArc>{other.out_arc};
	this->out_begin = std::vector<int>{other.out_begin};
	return *this;
}

unsigned int AdjGraph::node_count() const {
	return out_begin.size() - 1;
}

unsigned int AdjGraph::size() const {
	return out_begin.size() - 1;
}


std::vector<OutArc>::const_iterator AdjGraph::arcBegin(int v) const {
	return out_arc.begin() + out_begin[v];
}

std::vector<OutArc>::const_iterator AdjGraph::arcEnd(int v) const {
	return out_arc.begin() + out_begin[v+1];
}

Range<std::vector<OutArc>::const_iterator> AdjGraph::out(int v) const {
	return make_range(out_arc.begin() + out_begin[v], out_arc.begin() + out_begin[v+1]);
}

OutArc AdjGraph::out(int v, int i) const {
	return out_arc[out_begin[v] + i];
}

int AdjGraph::out_deg(int v) const {
	return out_begin[v+1] - out_begin[v];
}

bool AdjGraph::hasVertexNoSinks(int v) const {
	return out_begin[v+1] == out_begin[v];
}

void AdjGraph::changeWeightOfArc(int v, int w, int newWeight) {
	this->changeWeightOfDirectedArc(v, w, newWeight);
	this->changeWeightOfDirectedArc(w, v, newWeight);
}

void AdjGraph::changeWeightOfDirectedArc(int v, int w, int newWeight) {
	OutArc& arc = this->getArc(v, w);
	if (arc.weight != newWeight) {
		arc.perturbated = true;
	}
	arc.weight = newWeight;
}

int AdjGraph::getWeightOfArc(int v, int w) const {
	return getArc(v, w).weight;
}

int AdjGraph::getWeightOfIthArc(int v, int i) const {
	return this->out(v, i).weight;
}

bool operator==(const AdjGraph& thiz, const AdjGraph& other) {
	return (thiz.out_begin == other.out_begin) && (thiz.out_arc == other.out_arc);
}

bool operator!=(const AdjGraph& thiz, const AdjGraph& other) {
	return !(thiz == other);
}

OutArc& AdjGraph::getIthOutArc(int v, int i) {
	return out_arc[out_begin[v] + i];
}

const OutArc& AdjGraph::getIthOutArc(int v, int i) const{
	DO_ON_DEBUG {
		if (this->out_deg(v) == 0) {
			log_error("the vertex", v, "has not outgoing edges");
			throw std::domain_error{"vertex has no outgoing edges"};
		}
	}
	return out_arc[out_begin[v] + i];
}

OutArc& AdjGraph::getArc(int v, int w) {
	for (auto i=0; i<out_deg(v); ++i) {
		if (getIthOutArc(v,i).target == w) {
			return getIthOutArc(v,i);
		}
	}
	throw cpp_utils::exceptions::ImpossibleException{"arc from %d and %d not found", v, w};
}

const OutArc& AdjGraph::getArc(int v, int w) const {
	for (auto i=0; i<out_deg(v); ++i) {
		if (getIthOutArc(v,i).target == w) {
			return getIthOutArc(v,i);
		}
	}
	throw cpp_utils::exceptions::ImpossibleException{"arc from %d and %d not found", v, w};
}

bool AdjGraph::containsArc(nodeid_t source, nodeid_t sink) const {
	for (auto i=0; i<out_deg(source); ++i) {
		if (getIthOutArc(source,i).target == sink) {
			return true;
		}
	}
	return false;
}

bool AdjGraph::hasArcBeenPerturbated(int v, int w) const {
	OutArc a = this->getArc(v, w);
	return a.hasBeenPerturbated();
}

bool AdjGraph::hasBeenPerturbated() const {
	bool result = false;
	for (OutArc a : this->out_arc) {
		debug("checking if arc ", a, "has been perturbated...");
		if (a.hasBeenPerturbated()) {
			result = true;
			break;
		}
	}
	debug("done!");

	return result;
}

size_t AdjGraph::getTotalNumberOfArcs() const {
	return this->out_arc.size();
}

// dpf::cost_t AdjGraph::getCostOfPath(const std::vector<xyLoc>& path, const Mapper& mapper) const {
// 	dpf::cost_t result = 0;
// 	for (int i=1; i<path.size(); ++i) {
// 		xyLoc prev = path.at(i-1);
// 		xyLoc tmp = path.at(i);
// 		result += this->getWeightOfArc(mapper(prev), mapper(tmp));
// 	}
// 	return result;
// }

// vector<Arc> AdjGraph::getArcsOverMap(const gridmap_path& path, const Mapper& mapper) const {
// 	vector<Arc> result{};

// 	xyLoc loc = path.at(0);
// 	dpf::nodeid_t start = mapper(loc);
// 	for (auto i=1; i<path.size(); ++i) {
// 		dpf::nodeid_t end = mapper(path.at(i));
// 		result.push_back(Arc{static_cast<int>(start), static_cast<int>(end), this->getWeightOfArc(static_cast<int>(start), static_cast<int>(end))});
// 	}

// 	return result;
// }


// dpf::big_integer AdjGraph::getNumberOfArcsInPathSuchThat(const Mapper& mapper, const std::vector<xyLoc>& path, std::function<bool(dpf::big_integer i, dpf::nodeid_t, OutArc a)> filter) const {
// 	dpf::nodeid_t tmp;

// 	dpf::big_integer result = 0;
// 	tmp = mapper(path.at(0));
// 	for (dpf::big_integer i=1; i<path.size(); ++i) {
// 		OutArc a = this->getArc(tmp, mapper(path.at(i)));
// 		if (filter(i-1, tmp, a)) {
// 			result += 1;
// 		}
// 		tmp = mapper(path.at(i));
// 	}

// 	return result;
// }

// dpf::big_integer AdjGraph::getNumberOfArcsFromRatio(double ratio) const {
// 	return static_cast<dpf::big_integer>(this->getTotalNumberOfArcs() * ratio);
// }

// void AdjGraph::print(const Mapper& mapper, const string& baseName, std::function<bool(dpf::nodeid_t)>* shouldPrintNode, std::function<bool(dpf::nodeid_t, OutArc)>* shouldPrintArc, std::function<const string(dpf::nodeid_t, OutArc)>* colorArc) const {
// 	string dotFilename{baseName};
// 	dotFilename += ".dot";

// 	FILE* f = fopen(dotFilename.c_str(), "w");
// 	if (f == NULL) {
// 		throw CpdException{"file %s cannot be created!", dotFilename};
// 	}

// 	std::function<bool(dpf::nodeid_t)> shouldPrintNodeLambda = shouldPrintNode != nullptr ? *shouldPrintNode : [&](dpf::nodeid_t) -> bool {return true;};
// 	std::function<bool(dpf::nodeid_t, OutArc)> shouldPrintArcLambda = shouldPrintArc != nullptr ? *shouldPrintArc : [&](dpf::nodeid_t, OutArc) -> bool {return true;};
// 	//TODO hack to have the perturbated edges marked as red
// 	std::function<const std::string(dpf::nodeid_t, OutArc)> colorArcLambda = colorArc != nullptr ? *colorArc : [&](dpf::nodeid_t id, OutArc a) -> const string{return std::string{a.hasBeenPerturbated() ? "red" : "black"};};


// 	fprintf(f, "graph {\n");
// 	for (dpf::nodeid_t i=0; i<node_count(); ++i) {
// 		//coordaintes in dot are in bottom left till top right
// 		if (shouldPrintNodeLambda(i)) {
// 			stringstream ss;
// 			ss << mapper(i) << "\nid=" << i;
// 			fprintf(f, "N%04u [label=\"%s\", pos=\"%u,-%u!\"];\n", i, ss.str().c_str(), mapper(i).x*3, mapper(i).y*3);
// 		}
// 	}

// 	for (dpf::nodeid_t sourceId=0; sourceId<node_count(); ++sourceId) {
// 		for (auto it = arcBegin(sourceId); it != arcEnd(sourceId); ++it) {
// 			if (shouldPrintArcLambda(sourceId, *it)) {
// 				//if used to avoid overlaps of edge labels
// 				const std::string c = colorArcLambda(sourceId, *it);
// 				fprintf(f, "N%04u -- N%04u [label=\"%d\" fontcolor=\"%s\" color=\"%s\" penwidth=\"%d\"];\n",
// 						sourceId, it->target,
// 						it->weight,
// 						c.c_str(),
// 						c.c_str(),
// 						it->hasBeenPerturbated() ? 3 : 1
// 				);
// 			}
// 		}
// 	}

// 	fprintf(f, "}");

// 	fclose(f);

// 	std::stringstream ss;
// 	ss << "dot -Kneato -Tfig -O "<< baseName << ".dot";
// 	int errorCode = system(ss.str().c_str());
// 	if (errorCode != 0) {
// 		warning(ss.str(), "resulted in an error code of", errorCode, ". Continuing as nothing happened");
// 	}
// 	//unlink(dotFilename.c_str());
// }

// void AdjGraph::printPaths(const Mapper& mapper, const string& baseName, const std::vector<gridmap_path>& paths) const {
// 	xyLoc minPoint = xyLoc{INT_MAX, INT_MAX};
// 	xyLoc maxPoint = xyLoc{0, 0};

// 	//check the max point valid for all paths
// 	for (gridmap_path p: paths) {
// 		xyLoc minTmp;
// 		xyLoc maxTmp;

// 		p.getRanges(minTmp, maxTmp);

// 		if (minTmp.x < minPoint.x) {
// 			minPoint.x = minTmp.x;
// 		}
// 		if (minTmp.y < minPoint.y) {
// 			minPoint.y = minTmp.y;
// 		}
// 		if (maxTmp.x > maxPoint.x) {
// 			maxPoint.x = maxTmp.x;
// 		}
// 		if (maxTmp.y > maxPoint.y) {
// 			maxPoint.y = maxTmp.y;
// 		}
// 	}


// 	std::function<bool(dpf::nodeid_t)> nodeLambda = [&](dpf::nodeid_t id) -> bool{return mapper(id).isInside(minPoint, maxPoint);};
// 	std::function<bool(dpf::nodeid_t, OutArc)> arcLambda = [&](dpf::nodeid_t id, OutArc a) -> bool {
// 		return mapper(id).isInside(minPoint, maxPoint) && mapper(a.target).isInside(minPoint, maxPoint);
// 	};
// 	std::function<const std::string(dpf::nodeid_t, OutArc)> arcColorLambda = [&](dpf::nodeid_t id, OutArc a) -> const std::string {
// 		const std::string COLORS[] = {"red", "blue", "green"};
// 		for (auto i=0; i<paths.size(); ++i) {
// 			if (paths[i].contains(mapper(id), mapper(a.target))) {
// 				return COLORS[i];
// 			}
// 		}
// 		return "black";
// 	};

// 	this->print(mapper, baseName,
// 			&nodeLambda,
// 			&arcLambda,
// 			&arcColorLambda
// 	);
// }


// PPMImage AdjGraph::getImageWith(const Mapper& mapper, const GridMap& map, const dpf::nodeid_t start, const dpf::nodeid_t goal, const std::vector<dpf::nodeid_t> expandedList, color_t expandedColor, color_t startColor, color_t goalColor, color_t perturbatedColor, color_t backgroundColor) const {

//     //each cell is a 3x3 because in this way we can print if an edge has been perturbated
//     constexpr int SCALEX = 4;
//     constexpr int SCALEY = 4;
//     std::function<bool(PPMImage&, dpf::coo2d_t, dpf::coo2d_t, const color_t&)> setMetaPixel = [&](PPMImage& image, dpf::coo2d_t x, dpf::coo2d_t y, const color_t& c) {
//         image.setPixel(SCALEX*x+1, SCALEY*y+1, c);
//         image.setPixel(SCALEX*x+2, SCALEY*y+1, c);
//         image.setPixel(SCALEX*x+3, SCALEY*y+1, c);
//         image.setPixel(SCALEX*x+1, SCALEY*y+2, c);
//         image.setPixel(SCALEX*x+2, SCALEY*y+2, c);
//         image.setPixel(SCALEX*x+3, SCALEY*y+2, c);
//         image.setPixel(SCALEX*x+1, SCALEY*y+3, c);
//         image.setPixel(SCALEX*x+2, SCALEY*y+3, c);
//         image.setPixel(SCALEX*x+3, SCALEY*y+3, c);
//         return true;
//     };

//     std::function<bool(PPMImage&, dpf::nodeid_t)> setPerturbatedArcs = [&](PPMImage& image, dpf::nodeid_t sourceNode) {
//     	for (int i=0; i<this->out_deg(sourceNode); ++i) {
//     		OutArc arc = this->getIthOutArc(sourceNode, i);
// 			if (arc.hasBeenPerturbated()) {
// 				xyLoc sourceLoc = mapper(sourceNode);
// 				xyLoc sinkLoc = mapper(arc.target);
// 				int pixelx = 0;
// 				int pixely = 0;
// 				switch (xyLoc::getDirection(sourceLoc, sinkLoc)) {
// 					case Direction::NORTH: {pixelx=2; pixely=1; break;}
// 					case Direction::EAST: {pixelx=3; pixely=2; break;}
// 					case Direction::SOUTH: {pixelx=2; pixely=3; break;}
// 					case Direction::WEST: {pixelx=1; pixely=2; break;}
// 					case Direction::NORTHEAST: {pixelx=3; pixely=1; break;}
// 					case Direction::NORTHWEST: {pixelx=1; pixely=1; break;}
// 					case Direction::SOUTHEAST: {pixelx=3; pixely=3; break;}
// 					case Direction::SOUTHWEST: {pixelx=1; pixely=3; break;}
// 					default:
// 						throw std::invalid_argument{"direction invalid!"};
// 				}
// 				image.setPixel(sourceLoc.x*SCALEX+pixelx, sourceLoc.y*SCALEY+pixely, perturbatedColor);
// 			}
//     	}
// 		return true;
//     };


//     //everything is white
//     PPMImage image{static_cast<dpf::coo2d_t>(SCALEX * map.getWidth()), static_cast<dpf::coo2d_t>(SCALEY * map.getHeight()), backgroundColor};

//     //generate grid
//     for (auto y=0; y<(SCALEY * map.getHeight()); ++y) {
//     	for (auto x=0; x<(SCALEX * map.getWidth()); ++x) {
//     		if ((y % SCALEY) == 0) {
//     			image.setPixel(x, y, BLACK);
//     			continue;
//     		}
//     		if ((x % SCALEX) == 0) {
// 				image.setPixel(x, y, BLACK);
// 			}
//     	}
//     }

//     //mark black as untraversible
//     for (auto y=0; y<map.getHeight(); ++y) {
//         for (auto x=0; x<map.getWidth(); ++x) {
//             if (!map.isTraversable(x,y)) {
//                 setMetaPixel(image, x, y, BLACK);
//             }
//         }
//     }

//     //mark green as expanded
//     for (dpf::nodeid_t expandedNode : expandedList) {
//         xyLoc position = mapper(expandedNode);
//         setMetaPixel(image, position.x, position.y, expandedColor);
//     }

// 	//add perturbations
// 	for (dpf::nodeid_t i=0; i<this->node_count(); ++i) {
// 		//color all the perturbated arc in the epxanded list
//         setPerturbatedArcs(image, i);
// 	}


//     //makr start as red
//     xyLoc startLoc = mapper(start);
//     image.setPixel(startLoc.x*SCALEX+2, startLoc.y*SCALEY+2, startColor);
//     setPerturbatedArcs(image, start);
//     //mark blue as goal
//     xyLoc goalLoc = mapper(goal);
//     image.setPixel(goalLoc.x*SCALEX+2, goalLoc.y*SCALEY+2, goalColor);
//     setPerturbatedArcs(image, goal);

//     return image;
// }

// std::unordered_set<int> AdjGraph::getWeights() const {
// 	std::unordered_set<int> result;

// 	for(OutArc arc: this->out_arc) {
// 		if (result.find(arc.weight) == result.end()) {
// 			result.insert(arc.weight);
// 		}
// 	}

// 	return result;
// }

// const std::vector<OutArc>& AdjGraph::getArcs() const {
// 	return this->out_arc;
// }

// void AdjGraph::save(FILE* f) const {
// 	save_vector<OutArc>(f, this->out_arc);
// 	save_vector<int>(f, this->out_begin);
// }

// AdjGraph AdjGraph::load(FILE* f) {
// 	AdjGraph result{};

// 	result.out_arc = load_vector<OutArc>(f);
// 	result.out_begin = load_vector<int>(f);

// 	return result;
// }

std::ostream& operator<<(std::ostream& stream, const AdjGraph& a) {
	for (uint32_t source=0; source<a.size(); ++source) {
		stream << "OutArcs of " << source << std::endl;
		for (int ith=0; ith<a.out_deg(source); ++ith) {
			const OutArc arc = a.out(source, ith);
			stream << " - " << ith << arc.target  << "with weight " << arc.weight << "(perturbated=" << arc.perturbated << ")" << std::endl;
		}
		stream << "***********************" << std::endl;
	}
	return stream;
}

}