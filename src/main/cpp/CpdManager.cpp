#include <deque>
#include <vector>
#include <algorithm>
#include <assert.h>
#include "CpdManager.hpp"

#include "list_graph.hpp"
#include "mapper.hpp"
#include "cpd.hpp"
#include "order.hpp"
//#include "adj_graph.h"
#include "dijkstra.hpp"
#include "balanced_min_cut.hpp"
#include "prefer_zero_cut.hpp"
#include <cstdio>
#include <stdlib.h>
#include <unistd.h>
#include "cut_order.hpp"

#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

using namespace std;

namespace cpd {


// TOODO remove static bool GetPathInformationallAtOnceWithCPD(void* cpdHelper, xyLoc start, xyLoc goal, dpf::cost_t* pathCost, std::vector<dpf::move_t>* moves, std::vector<xyLoc>* path);

#ifdef USE_PARALLELISM
#include <omp.h>
#endif

const char *GetName() {
#ifndef USE_CUT_ORDER
	return "DFS-SRC-RLE";
#else
	return "METIS-CUT-SRC-RLE";
#endif
}


//static constexpr int DIJKSTRA_PRINTF_EVERY_VERTEX = 500;
//
// static std::function<int(const GridMap&, const Mapper&, xyLoc, xyLoc)> SINGLE_TERRAIN_STRATEGY = [&](const GridMap& map, const Mapper& mapper, xyLoc source, xyLoc sink) -> int{
// 	switch (xyLoc::getDirection(source, sink)) {
// 	case Direction::NORTH:
// 	case Direction::EAST:
// 	case Direction::SOUTH:
// 	case Direction::WEST: {
// 		//average of 2 values
// 		return static_cast<int>(map.getValue(source.x, source.y));
// 	}
// 	case Direction::NORTHEAST:
// 	case Direction::NORTHWEST:
// 	case Direction::SOUTHEAST:
// 	case Direction::SOUTHWEST: {
// 		//average of 4 values
// 		//the 1414 stands for the square root
// 		return static_cast<int>(1414 * map.getValue(source.x, source.y));
// 	}
// 	default:
// 		throw CpdException{"invalid direction from %d,%d to %d,%d!", source.x, source.y, sink.x, sink.y};
// 	}
// };

// std::function<int(const GridMap&, const Mapper&, xyLoc, xyLoc)> MULTI_TERRAIN_STRATEGY = [&](const GridMap& map, const Mapper& mapper, xyLoc source, xyLoc sink) -> int{
// 	switch (xyLoc::getDirection(source, sink)) {
// 	case Direction::NORTH:
// 	case Direction::EAST:
// 	case Direction::SOUTH:
// 	case Direction::WEST: {
// 		//average of 2 values
// 		return static_cast<int>((map.getValue(source.x, source.y) + map.getValue(sink.x, sink.y)) / 2.);
// 	}
// 	case Direction::NORTHEAST:
// 	case Direction::NORTHWEST:
// 	case Direction::SOUTHEAST:
// 	case Direction::SOUTHWEST: {
// 		//average of 4 values
// 		//the 1414 stands for the square root
// 		return static_cast<int>(1.414 * ((map.getValue(source.x, source.y) + map.getValue(sink.x, source.y) + map.getValue(source.x, sink.y) + map.getValue(sink.x, sink.y)) / 4.));
// 	}
// 	default:
// 		throw CpdException{"invalid direction from %d,%d to %d,%d!", source.x, source.y, sink.x, sink.y};
// 	}
// };
//
// void PreprocessMap(const GridMap& map, const char *filename)
// {
// 	Mapper mapper(map);
// 	info("width = ", map.getWidth(), "height = ", map.getHeight(), "node_count", mapper.node_count());

// 	debug("Computing node order");
// #ifndef USE_CUT_ORDER
// 	NodeOrdering order = compute_real_dfs_order(getListGraphFrom(map, mapper, MULTI_TERRAIN_STRATEGY));
// #else
// 	NodeOrdering order = compute_cut_order(
// 			getListGraphFrom(map, mapper, MULTI_TERRAIN_STRATEGY),
// 			prefer_zero_cut(balanced_min_cut)
// 	);
// #endif
// 	mapper.reorder(order);


// 	debug("Computing first-move matrix");

// 	CPD cpd;
// 	{
// 		AdjGraph g(getListGraphFrom(map, mapper, MULTI_TERRAIN_STRATEGY));

// 		{
// 			info("calling Dijkstra one time to generate an estimate of time...");
// 			Dijkstra dij(g);
// 			Timer t;
// 			t.StartTimer();
// 			dij.run(0);
// 			t.EndTimer();
// 			info("Estimated sequential running time:", static_cast<int>(t.GetElapsedTime()*g.node_count()/60.0), " min");
// 		}

// #ifndef USE_PARALLELISM
// 		info("we're not going to use parallelism!");
// 		Dijkstra dij(g);
// 		for(dpf::nodeid_t source_node=0; source_node < g.node_count(); ++source_node){
// 			//progress bar
// 			if((source_node % DIJKSTRA_PRINTF_EVERY_VERTEX) == 0) {
// 				info(source_node, " of ", g.node_count(), "done");
// 			}

// 			debug("calling dijktstra on node", source_node, "!");
// 			const auto&allowed = dij.run(source_node);
// 			debug("appending on row");
// 			cpd.append_row(source_node, allowed);
// 		}
// #else
// 		info("Using", omp_get_max_threads(), "threads");
// 		vector<CPD>thread_cpd(omp_get_max_threads());

// 		int progress = 0;

// #pragma omp parallel
// 		{
// 			const int thread_count = omp_get_num_threads();
// 			const int thread_id = omp_get_thread_num();
// 			const int node_count = g.node_count();

// 			dpf::nodeid_t node_begin = (node_count*thread_id) / thread_count;
// 			dpf::nodeid_t node_end = (node_count*(thread_id+1)) / thread_count;

// 			AdjGraph thread_adj_g(g);
// 			Dijkstra thread_dij(thread_adj_g);
// 			for(dpf::nodeid_t source_node=node_begin; source_node < node_end; ++source_node){
// 				thread_cpd[thread_id].append_row(source_node, thread_dij.run(source_node));
// #pragma omp critical
// 				{
// 					++progress;
// 					if((progress % DIJKSTRA_PRINTF_EVERY_VERTEX) == 0){
// 						info(progress, " of ", g.node_count(), "done");
// 						fflush(stdout);
// 					}
// 				}
// 			}
// 		}

// 		for(auto&x:thread_cpd)
// 			cpd.append_rows(x);
// #endif
// 	}

// 	printf("Saving data to %s\n", filename);
// 	FILE*f = fopen(filename, "wb");
// 	order.save(f);
// 	cpd.save(f);
// 	fclose(f);
// 	printf("Done\n");

// }

// void PreprocessMap(std::vector<bool> &bits, int width, int height, const char *filename)
// {


// 	Mapper mapper(bits, width, height);
// 	printf("width = %d, height = %d, node_count = %d\n", width, height, mapper.node_count());

// 	printf("Computing node order\n");
// #ifndef USE_CUT_ORDER
// 	NodeOrdering order = compute_real_dfs_order(extract_graph(mapper));
// #else
// 	NodeOrdering order = compute_cut_order(extract_graph(mapper), prefer_zero_cut(balanced_min_cut));
// #endif
// 	mapper.reorder(order);


// 	printf("Computing first-move matrix\n");

// 	CPD cpd;
// 	{
// 		AdjGraph g(extract_graph(mapper));

// 		{
// 			Dijkstra dij(g);
// 			Timer t;
// 			t.StartTimer();
// 			dij.run(0);
// 			t.EndTimer();
// 			printf("Estimated sequential running time : %dmin\n", static_cast<int>(t.GetElapsedTime()*g.node_count()/60.0));
// 		}

// #ifndef USE_PARALLELISM
// 		Dijkstra dij(g);
// 		for(dpf::nodeid_t source_node=0; source_node < g.node_count(); ++source_node){
// 			//progress bar
// 			if (g.node_count() >= DIJKSTRA_PRINTF_EVERY_VERTEX) {
// 				if(source_node % (g.node_count()/DIJKSTRA_PRINTF_EVERY_VERTEX) == 0) {
// 					printf("%d of %d done\n", source_node, g.node_count());
// 				}
// 			}

// 			const auto&allowed = dij.run(source_node);
// 			cpd.append_row(source_node, allowed);
// 		}
// #else
// 		printf("Using %d threads\n", omp_get_max_threads());
// 		vector<CPD>thread_cpd(omp_get_max_threads());

// 		int progress = 0;

// #pragma omp parallel
// 		{
// 			const int thread_count = omp_get_num_threads();
// 			const int thread_id = omp_get_thread_num();
// 			const int node_count = g.node_count();

// 			dpf::nodeid_t node_begin = (node_count*thread_id) / thread_count;
// 			dpf::nodeid_t node_end = (node_count*(thread_id+1)) / thread_count;

// 			AdjGraph thread_adj_g(g);
// 			Dijkstra thread_dij(thread_adj_g);
// 			for(dpf::nodeid_t source_node=node_begin; source_node < node_end; ++source_node){
// 				thread_cpd[thread_id].append_row(source_node, thread_dij.run(source_node));
// #pragma omp critical
// 				{
// 					++progress;
// 					if (g.node_count() > 10) {
// 						if(progress % (g.node_count()/10) == 0){
// 							printf("%d of %d done\n", progress, g.node_count());
// 							fflush(stdout);
// 						}
// 					}
// 				}
// 			}
// 		}

// 		for(auto&x:thread_cpd)
// 			cpd.append_rows(x);
// #endif
// 	}

// 	printf("Saving data to %s\n", filename);
// 	FILE*f = fopen(filename, "wb");
// 	order.save(f);
// 	cpd.save(f);
// 	fclose(f);
// 	printf("Done\n");

// }



/**
 * generate a pointer which c an be used to solve queries.
 *
 * @attention
 * the return value needs to be freed manually!
 *
 * @param[in] bits the map involved
 * @param[in] w the width of the map
 * @param[in] h the height of the map
 * @param[in] filename the filename where the CPD is stored
 * @return
 *  a pointer of a **newly allocate**d State struct which can help you solve queries
 */
// void *PrepareForSearch(std::vector<bool> &bits, int w, int h, const char *filename)
// {
// 	printf("Loading preprocessing data\n");
// 	State*state = new State;

// 	state->mapper = Mapper(bits, w, h);

// 	FILE*f = fopen(filename, "rb");
// 	if (f == NULL) {
// 		throw CpdException("Couldn't open file %s!", filename);
// 	}
// 	NodeOrdering order;
// 	order.load(f);
// 	state->cpd.load(f);
// 	fclose(f);

// 	state->mapper.reorder(order);

// 	state->graph = AdjGraph(extract_graph(state->mapper));
// 	state->current_node = -1;

// 	printf("Loading done\n");


// 	return state;
// }

// void *PrepareForSearch(const GridMap& map, const char *filename)
// {
// 	printf("Loading preprocessing data\n");
// 	State*state = new State;

// 	state->mapper = Mapper(map);

// 	FILE*f = fopen(filename, "rb");
// 	if (f == NULL) {
// 		throw CpdException("Couldn't open file %s!", filename);
// 	}
// 	NodeOrdering order;
// 	order.load(f);
// 	state->cpd.load(f);
// 	fclose(f);

// 	state->mapper.reorder(order);

// 	state->graph = AdjGraph(getListGraphFrom(map, state->mapper, MULTI_TERRAIN_STRATEGY));
// 	state->current_node = -1;

// 	printf("Loading done\n");


// 	return state;
// }

// const Mapper& GetMapper(const void* helper) {
// 	return static_cast<const State*>(helper)->mapper;
// }

// const AdjGraph& GetGraph(const void* helper) {
// 	return static_cast<const State*>(helper)->graph;
// }

// void DeleteHelperOfSearch(const void* helper) {
// 	delete static_cast<const State*>(helper);
// }

// #ifndef EXTRACT_ALL_AT_ONCE

// bool GetPath(void *data, xyLoc s, xyLoc t, std::vector<xyLoc> &path)
// {
// 	State*state = static_cast<State*>(data);
// 	if(path.empty()){
// 		state->current_node = state->mapper(s);
// 		state->target_node = state->mapper(t);
// 		auto first_move = state->cpd.get_first_move(state->current_node, state->target_node);
// 		if(first_move == 0xF){
// 			return true;
// 		} else if(state->current_node == state->target_node) {
// 			path.push_back(s);
// 			return true;
// 		} else {
// 			path.push_back(s);
// 			state->current_node = state->graph.out(state->current_node, first_move).target;
// 			return false;
// 		}
// 	} else {
// 		if(state->current_node != state->target_node){
// 			path.push_back(state->mapper(state->current_node));
// 			state->current_node = state->graph.out(state->current_node, state->cpd.get_first_move(state->current_node, state->target_node)).target;
// 			return false;
// 		} else {
// 			path.push_back(t);
// 			return true;
// 		}
// 	}
// }
// #else
// bool GetPath(void *data, xyLoc s, xyLoc t, std::vector<xyLoc> &path)
// {
// 	State*state = static_cast<State*>(data);

// 	int current_node = state->mapper(s);
// 	int target_node = state->mapper(t);

// 	unsigned char first_move = state->cpd.get_first_move(current_node, target_node);
// 	if(first_move == 0xF)
// 		return true;
// 	for(;;){
// 		path.push_back(state->mapper(current_node));
// 		current_node = state->graph.out(current_node, first_move).target;
// 		if(current_node == target_node)
// 			break;
// 		first_move = state->cpd.get_first_move(current_node, target_node);

// 	}
// 	path.push_back(t);
// 	return true;
// }
// #endif


// bool GetPathLengthNextMoveWithCPD(void* cpdHelper, xyLoc start, xyLoc goal, int& pathCost, bool& isPathTerminated) {
// 	State* state = static_cast<State*>(cpdHelper);

// 	if(pathCost == 0) {
// 		//first move of the path
// 		state->current_node = state->mapper(start);
// 		state->target_node = state->mapper(goal);
// 		dpf::move_t first_move = state->cpd.get_first_move(state->current_node, state->target_node);
// 		if(first_move == 0xF){
// 			pathCost = 0;
// 			isPathTerminated = true;
// 			return false;
// 		} else if(state->current_node == state->target_node) {
// 			pathCost = 0;
// 			isPathTerminated = true;
// 			return true;
// 		} else {
// 			state->current_node = state->graph.out(state->current_node, first_move).target;
// 			pathCost += state->graph.out(state->current_node, first_move).weight;
// 			isPathTerminated = false;
// 			return false;
// 		}
// 	} else {
// 		//next move in the path
// 		if(state->current_node != state->target_node){
// 			state->current_node = state->graph.out(state->current_node, state->cpd.get_first_move(state->current_node, state->target_node)).target;
// 			pathCost += state->graph.out(state->current_node, state->cpd.get_first_move(state->current_node, state->target_node)).weight;
// 			isPathTerminated = false;
// 			return false;
// 		} else {
// 			isPathTerminated = true;
// 			return true;
// 		}
// 	}
// }

// bool getPathLengthAllAtOnceWithCPD(void* cpdHelper, xyLoc start, xyLoc goal, dpf::cost_t& pathCost) {
// 	return GetPathInformationallAtOnceWithCPD(cpdHelper, start, goal, &pathCost, nullptr, nullptr);
// }

// bool GetPathWithCPD(void *cpdHelper, xyLoc start, xyLoc goal, std::vector<xyLoc> &path)
// {
// #if EXTRACT_ALL_AT_ONCE
// 	path.resize(0);
// 	GetPath(reference, start, goal, path);
// 	if (start == goal) {
// 		return true;
// 	}
// 	return path.size > 0;
// #else
// 	path.resize(0);
// 	bool done;
// 	do {
// 		done = GetPath(cpdHelper, start, goal, path);
// 		if (done) {
// 			return path.size() > 0;
// 		} else {
// 			start = path.back();
// 		}
// 	} while (!done);
// 	return true;
// #endif
// }

// bool GetPathDataAllAtOnceWithCPD(void* cpdHelper, xyLoc start, xyLoc goal, dpf::cost_t& pathCost, std::vector<dpf::move_t>& moves, std::vector<xyLoc>& path) {
// 	return GetPathInformationallAtOnceWithCPD(cpdHelper, start, goal, &pathCost, &moves, &path);
// }

// bool GetPathDataCostAndMovesWithCPD(void* cpdHelper, xyLoc start, xyLoc goal, dpf::cost_t& pathCost, std::vector<dpf::move_t>& moves) {
// 	return GetPathInformationallAtOnceWithCPD(cpdHelper, start, goal, &pathCost, &moves, nullptr);
// }

// bool GetPathInformationNextMoveWithCPD(void* cpdHelper, xyLoc start, xyLoc goal, dpf::cost_t& pathCost, std::vector<dpf::move_t>& moves, dpf::nodeid_t& nextCellId, bool& isPathTerminated) {
// 	State* state = static_cast<State*>(cpdHelper);



// 	if(moves.size() == 0) {
// 		//first move of the path
// 		state->current_node = state->mapper(start);
// 		state->target_node = state->mapper(goal);
// 		dpf::move_t first_move = state->cpd.get_first_move(state->current_node, state->target_node);
// 		if(first_move == 0xF){
// 			pathCost = 0;
// 			isPathTerminated = true;
// 			return false;
// 		} else if(state->current_node == state->target_node) {
// 			pathCost = 0;
// 			isPathTerminated = true;
// 			return true;
// 		} else {
// 			pathCost += state->graph.out(state->current_node, first_move).weight;
// 			debug("first move current before is=", state->current_node, " first move=", first_move, " in grid is ", state->mapper(state->current_node));
// 			state->current_node = state->graph.out(state->current_node, first_move).target;
// 			debug("first move current after is=", state->current_node, " in grid is ", state->mapper(state->current_node));
// 			nextCellId = state->current_node;
// 			moves.push_back(first_move);
// 			isPathTerminated = false;
// 			return false;
// 		}
// 	} else {
// 		//next move in the path
// 		if(state->current_node != state->target_node){
// 			dpf::move_t first_move = state->cpd.get_first_move(state->current_node, state->target_node);
// 			OutArc arc = state->graph.out(state->current_node, first_move);
// 			state->current_node = arc.target;
// 			pathCost += arc.weight;
// 			nextCellId = state->current_node;
// 			moves.push_back(first_move);
// 			isPathTerminated = false;
// 			return false;
// 		} else {
// 			isPathTerminated = true;
// 			return true;
// 		}
// 	}
// }
//TODO remove
// std::ostream & operator<<(std::ostream& str, const xyLoc& v) {
// 	str << "{" << v.x << ", " << v.y << "}";
// 	return str;
// }
//TODO REMOVE
// bool operator==(const xyLoc& a, const xyLoc& b) {
// 	return (a.x == b.x) && (a.y == b.y);
// }
// TODO remove
// bool operator!=(const xyLoc& a, const xyLoc& b) {
// 	return (a.x != b.x) || (a.y != b.y);
// }
// TODO remove
// void drawMapWithPath(std::ostream& str, const std::vector<bool>& map, int width, int height, const std::vector<xyLoc>& path, char pathSymbol) {
// 	char toPrint;
// 	str << std::endl;
// 	for (dpf::coo2d_t y=0; y<height; ++y) {
// 		for (dpf::coo2d_t x=0; x<width; ++x) {
//
// 			//untraversable cell
// 			if (!map[y*width+x]) {
// 				toPrint = '@';
// 				goto print;
// 			}
// 			//start location
// 			if (path.front() == xyLoc{x, y}) {
// 				toPrint = 'S';
// 				goto print;
// 			}
//
// 			//goal location
// 			if (path.back() == xyLoc{x, y}) {
// 				toPrint = 'G';
// 				goto print;
// 			}

// 			//location in path
// 			if (std::find(path.begin(), path.end(), xyLoc{x, y}) != path.end()){
// 				toPrint = pathSymbol;
// 				goto print;
// 			}

// 			//traversable cell which is not involved in  anything
// 			toPrint = '.';

// 			print:;
// 			str << toPrint;
// 		}
// 		str << std::endl;
// 	}


// }

// void printCPD(const char* mapName, const char* cpdFilename, xyLoc goal, const string& basename) {
// 	MapLoader mp;


// 	GridMap map = mp.LoadMap(mapName);
// 	std::vector<bool> rawMap = map.getTraversableMask();
// 	if (!isFileExists(cpdFilename)) {
// 		PreprocessMap(map, cpdFilename);
// 	}

// 	State* cpdHelper = static_cast<State*>(PrepareForSearch(map, cpdFilename));

// 	stringstream ss;
// 	ss << basename << ".dot";
// 	ofstream dotFile;
// 	dotFile.open(ss.str(), std::ios::trunc | std::ios::out);

// 	//x [shape="none", label=<<TABLE><TR><TD>a</TD><TD>a</TD></TR><TR><TD>a</TD><TD>a</TD></TR></TABLE>>];
// 	dotFile << "graph {" << "\n"
// 			<< "map [shape=\"none\", label=<<TABLE>";

// 	int goalid = cpdHelper->mapper(goal);
// 	for (dpf::coo2d_t y=0; y<map.getHeight(); ++y) {
// 		dotFile << "<TR>";
// 		for (dpf::coo2d_t x=0; x<map.getWidth(); ++x) {
// 			const char* label;
// 			const char* color;
// 			xyLoc source = xyLoc{x,y};
// 			dpf::nodeid_t sourceid = cpdHelper->mapper(source);

// 			if (!rawMap[y*map.getWidth()+x]) {
// 				//untraversable cell
// 				label = "@";
// 				color = "black";
// 			} else if (source == goal) {
// 				label = "GOAL";
// 				color = "green";
// 			} else {
// 				debug("source =", source, "goal=", goal);
// 				dpf::move_t first_move = cpdHelper->cpd.get_first_move(sourceid, goalid);
// 				xyLoc sink = cpdHelper->mapper(const_cast<const AdjGraph&>(cpdHelper->graph).getIthOutArc(sourceid, first_move).target);
// 				label = getLabel(xyLoc::getDirection(source, sink));
// 				color = "white";
// 			}
// 			dotFile << "<TD BGCOLOR=\"" << color << "\">" << label << "</TD>";
// 		}
// 		dotFile << "</TR>";
// 	}

// 	dotFile << "</TABLE>>];" << "\n"
// 			<< "}" << "\n";

// 	dotFile.close();

// 	ss.str(""); ss.clear();
// 	ss << "dot -Tsvg -o " << basename << ".svg " << basename << ".dot";
// 	int errorCode = system(ss.str().c_str());
// 	if (errorCode != 0) {
// 		warning(ss.str(), "resulted in an error code of", errorCode, ". Continuing as nothing happened");
// 	}

// 	ss.str(""); ss.clear();
// 	ss << basename << ".dot";
// 	unlink(ss.str().c_str());

// 	DeleteHelperOfSearch(cpdHelper);
// }

/**
 * like ::GetPathWithCPD but generates the cost of the path in the CPD as well
 *
 * @param[inout] cpdHelper the helper created with ::PrepareForSearch
 * @param[in] start the initial position of the agent
 * @param[in] goal the final destination of the agent
 * @param[out] pathCost the cost of the path @c path. If nullptr it will be ignored
 * @param[out] moves the built path. Each index `i` represents the action the CPD told us to perform. Each action is the i-th OutArc we need to select
 * 	in the map we're in. So it's the second parameter of AdjGraph::getIthOutArc;  If nullptr it will be ignored
 * @param[inout] path the built path. At index `i` the vector contains the cell where we need to go in order to reach the location @c goal
 * 	If nullptr it will be ignored;
 * @return
 *  @li true if we were able to find the optional path (thus @c path is altered);
 *  @li false if no path could be obtained (thus @c path is unchanged);
 */
// static bool GetPathInformationallAtOnceWithCPD(void* cpdHelper, xyLoc start, xyLoc goal, dpf::cost_t* pathCost, std::vector<dpf::move_t>* moves, std::vector<xyLoc>* path) {
// 	State*state = static_cast<State*>(cpdHelper);

// 	dpf::nodeid_t current_node = state->mapper(start);
// 	dpf::nodeid_t target_node = state->mapper(goal);

// 	DO_IF_NOT_NULL(path) { path->clear(); }
// 	DO_IF_NOT_NULL(moves) { moves->clear(); }
// 	DO_IF_NOT_NULL(pathCost) { *pathCost = 0; }

// 	if (current_node == target_node) {
// 		//the start is also the end
// 		DO_IF_NOT_NULL(path) {path->push_back(state->mapper(current_node)); }
// 		return true;
// 	}

// 	dpf::move_t first_move = state->cpd.get_first_move(current_node, target_node);
// 	//debug("first move from ", state->mapper(current_node), " to ", state->mapper(target_node), " is going to ", state->mapper(state->graph.out(current_node, first_move).target));

// 	if(first_move == 0xF) {
// 		return false;
// 	}

// 	while (true) {
// 		DO_IF_NOT_NULL(moves) { moves->push_back(first_move); debug("moves is ", *moves);}
// 		DO_IF_NOT_NULL(path) { path->push_back(state->mapper(current_node)); }
// 		DO_IF_NOT_NULL(pathCost) { *pathCost = *pathCost + state->graph.out(current_node, first_move).weight; }
// 		current_node = state->graph.out(current_node, first_move).target;
// 		if(current_node == target_node) {
// 			break;
// 		}
// 		first_move = state->cpd.get_first_move(current_node, target_node);
// 		//debug("first move from ", state->mapper(current_node), " to ", state->mapper(target_node), " is ", state->mapper(state->graph.out(current_node, first_move).target));
// 	}
// 	DO_IF_NOT_NULL(path) { path->push_back(goal); }
// 	return true;
// }

}
