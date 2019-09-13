/*
 * test.cpp
 *
 *  Created on: Oct 1, 2018
 *      Author: koldar
 */

#define CATCH_CONFIG_NO_POSIX_SIGNALS // https://github.com/catchorg/Catch2/issues/1295 avoid catch catching signals
#include "catch.hpp"

#include "Entry.hpp"

#include <string>
#include <sstream>

#include <pathfinding-utils/xyLoc.hpp>
#include <pathfinding-utils/GridMap.hpp>
#include <pathfinding-utils/GridMapGraphConverter.hpp>
#include <pathfinding-utils/GridBranching.hpp>
#include <pathfinding-utils/MovingAIGridMapReader.hpp>

using namespace cpd;
using namespace pathfinding;
using namespace cpp_utils;


SCENARIO("test cpd") {

	GIVEN("square03 map") {

		maps::MovingAIGridMapReader reader{
			'.', cost_t{100},
			'T', cost_t::INFTY,
			'@', cost_t::INFTY
		};
		maps::GridMap map{reader.load(boost::filesystem::path{"./square03.map"})};

		/*
		* MAP
		* 
		*  .....
		*  ...@@
		*  ..@@.
		*  ...@@
		*  .....
		* 
		*/

		maps::GridMapGraphConverter converter{maps::GridBranching::EIGHT_CONNECTED};
		graphs::AdjacentGraph<std::string, xyLoc, cost_t> graph{*converter.toGraph(map)};
		// auto image = graph.getPPM(); //TODO create unique_pointer
		// image->savePNG("square03-image");
		// delete image;

		CpdManager<std::string, xyLoc> manager{"./square03.cpd"};
		critical("creating cpd...");
		manager.saveCpd(graph);
		critical("done");

		critical("loading cpd");
		const graphs::IImmutableGraph<std::string, xyLoc, cost_t>& actualGraph = manager.loadCpd(graph);
		critical("done");

		moveid_t move;
		nodeid_t nextNode; 

		WHEN("target is equal to start") {
			xyLoc start{0,0};
			xyLoc target{0,0};

			nodeid_t startId = actualGraph.idOfVertex(start);
			nodeid_t targetId = actualGraph.idOfVertex(target);
			REQUIRE(manager.getFirstMove(startId, targetId, move, nextNode) == false);

		}

		WHEN("target is under start") {
			xyLoc start{0,0};
			xyLoc target{1,0};

			nodeid_t startId = actualGraph.idOfVertex(start);
			nodeid_t targetId = actualGraph.idOfVertex(target);
			REQUIRE(manager.getFirstMove(startId, targetId, move, nextNode) == true);

			REQUIRE(actualGraph.getVertex(nextNode) == target);
		}

		WHEN("target is much under start") {
			xyLoc start{0,0};
			xyLoc target{2,0};

			nodeid_t startId = actualGraph.idOfVertex(start);
			nodeid_t targetId = actualGraph.idOfVertex(target);
			REQUIRE(manager.getFirstMove(startId, targetId, move, nextNode) == true);

			REQUIRE(actualGraph.getVertex(nextNode) == xyLoc{1,0});
		}

		WHEN("target is far from start") {
			xyLoc start{0,0};
			xyLoc target{4,4};

			nodeid_t startId = actualGraph.idOfVertex(start);
			nodeid_t targetId = actualGraph.idOfVertex(target);
			REQUIRE(manager.getFirstMove(startId, targetId, move, nextNode) == true);

			REQUIRE(actualGraph.getVertex(nextNode) == xyLoc{0,1});
		}

		WHEN("target is unreachable") {
			xyLoc start{4,2};
			xyLoc target{1,0};

			nodeid_t startId = actualGraph.idOfVertex(start);
			nodeid_t targetId = actualGraph.idOfVertex(target);
			REQUIRE(manager.getFirstMove(startId, targetId, move, nextNode) == false);
		}

		WHEN("example of path") {
			xyLoc target{4,4};
			nodeid_t targetId = actualGraph.idOfVertex(target);

			REQUIRE(manager.getFirstMove(actualGraph.idOfVertex(xyLoc{0,0}), targetId, move, nextNode) == true);
			REQUIRE(actualGraph.getVertex(nextNode) == xyLoc{0,1});

			REQUIRE(manager.getFirstMove(actualGraph.idOfVertex(xyLoc{0,1}), targetId, move, nextNode) == true);
			REQUIRE(actualGraph.getVertex(nextNode) == xyLoc{0,2});
			
			REQUIRE(manager.getFirstMove(actualGraph.idOfVertex(xyLoc{0,2}), targetId, move, nextNode) == true);
			REQUIRE(actualGraph.getVertex(nextNode) == xyLoc{1,3});

			REQUIRE(manager.getFirstMove(actualGraph.idOfVertex(xyLoc{1,3}), targetId, move, nextNode) == true);
			REQUIRE(actualGraph.getVertex(nextNode) == xyLoc{2,4});

			REQUIRE(manager.getFirstMove(actualGraph.idOfVertex(xyLoc{2,4}), targetId, move, nextNode) == true);
			REQUIRE(actualGraph.getVertex(nextNode) == xyLoc{3,4});

			REQUIRE(manager.getFirstMove(actualGraph.idOfVertex(xyLoc{3,4}), targetId, move, nextNode) == true);
			REQUIRE(actualGraph.getVertex(nextNode) == xyLoc{4,4});
		}

		WHEN("testing whole path when target is source") {
			xyLoc start{0,0};
			xyLoc target{0,0};
			nodeid_t startId = actualGraph.idOfVertex(start);
			nodeid_t targetId = actualGraph.idOfVertex(target);

			REQUIRE(manager.generateOptimalPathOfNodes(startId, targetId) == std::vector<nodeid_t>{0});
		}

		WHEN("testing whole path when target is below from source") {
			xyLoc start{0,0};
			xyLoc target{0,1};
			nodeid_t startId = actualGraph.idOfVertex(start);
			nodeid_t targetId = actualGraph.idOfVertex(target);

			REQUIRE(manager.generateOptimalPathOfNodes(startId, targetId) == std::vector<nodeid_t>{0, 1});
		}

		WHEN("testing whole path when target is unreachable") {
			xyLoc start{0,0};
			xyLoc target{4,2};
			nodeid_t startId = actualGraph.idOfVertex(start);
			nodeid_t targetId = actualGraph.idOfVertex(target);

			REQUIRE(manager.generateOptimalPathOfNodes(startId, targetId) == std::vector<nodeid_t>{});
		}

		WHEN("testing whole path when target is far from source") {
			xyLoc start{0,0};
			xyLoc target{4,4};
			nodeid_t startId = actualGraph.idOfVertex(start);
			nodeid_t targetId = actualGraph.idOfVertex(target);

			actualGraph.saveBMP("cpdgraph");
			REQUIRE(manager.generateOptimalPathOfNodes(startId, targetId) == std::vector<nodeid_t>{0, 1, 2, 6, 15, 16, 17});
		}

	}
}

// static string getCPDFilename(const string& mapName) {
// 	std::stringstream ss;
// 	ss << "test" << "-" << mapName;
// 	return ss.str();
// }





// SCENARIO("Test the behaviour of some complex functions", "[normal]") {
// 	GIVEN("wanting to know about build_adj_array") {

// 		MapLoader mapLoader;
// 		const string mapName = "square02.map";
// 		int width, height;
// 		vector<bool> mapData;
// 		GridMap gridMap = mapLoader.LoadMap(mapName.c_str());
// 		mapData = gridMap.getTraversableMask();
// 		width = gridMap.getWidth();
// 		height = gridMap.getHeight();
// 		Mapper mapper = Mapper{mapData, width, height};
// 		ListGraph g = extract_graph(mapper);

// 		std::vector<int>out_begin, out_dest;
// 		auto source_node = [&](int x){
// 			return g.arc[x].source;
// 		};

// 		auto target_node = [&](int x){
// 			return g.arc[x].target;
// 		};

// 		WHEN("") {

// 			g.print(mapper, "square02ListGraph");
// 			detail::build_begin_array(out_begin, g.node_count(), g.arc.size(), source_node);
// 			//			build_adj_array(
// 			//				out_begin, out_dest,
// 			//				g.node_count(), g.arc.size(),
// 			//				source_node, target_node
// 			//			);

// 			debug("out_begin is ", out_begin);

// 			THEN("") {
// 				REQUIRE(out_begin == std::vector<int>{0, 3, 8, 11, 16, 23, 26, 29, 32});
// 			}
// 		}
// 	}
// }

// SCENARIO("Tests previous code", "[normal]") {

// 	std::vector<bool> mapData;
// 	int width;
// 	int height;
// 	MapLoader mapLoader;
// 	string mapName;
// 	string cpdFilename;

// 	GIVEN("a normal map") {
// 		mapName = "square01.map";
// 		cpdFilename = getCPDFilename(mapName);

// 		GridMap gridMap = mapLoader.LoadMap(mapName.c_str());
// 		width = gridMap.getWidth();
// 		height = gridMap.getHeight();
// 		mapData = gridMap.getTraversableMask();
// 		if (!isFileExists(cpdFilename.c_str())) {
// 			PreprocessMap(mapData, width, height, cpdFilename.c_str());
// 		}

// 		void* pathHelper = PrepareForSearch(mapData, width, height, cpdFilename.c_str());

// 		std::vector<xyLoc> path;
// 		bool result;
// 		WHEN("goal is on start") {
// 			result = GetPathWithCPD(pathHelper, xyLoc{0,0}, xyLoc{0,0}, path);
// 			debug("path is ", path);
// 			THEN("") {
// 				REQUIRE(result);
// 				REQUIRE(path.size() == 1);
// 				REQUIRE(path[0] == xyLoc{0, 0});
// 			}
// 		}

// 		WHEN("goal is immediately next to start") {
// 			result = GetPathWithCPD(pathHelper, xyLoc{0,0}, xyLoc{0,1}, path);
// 			THEN("") {
// 				REQUIRE(result);
// 				REQUIRE(path.size() == 2);
// 				REQUIRE(path == vector<xyLoc>{xyLoc{0, 0}, xyLoc{0,1}});
// 			}
// 		}

// 		WHEN("goal is in diagonal next to start") {
// 			result = GetPathWithCPD(pathHelper, xyLoc{0,0}, xyLoc{1,1}, path);
// 			THEN("") {
// 				debug("path is ", path);
// 				REQUIRE(result);
// 				REQUIRE(path.size() == 2);
// 				REQUIRE(path == vector<xyLoc>{xyLoc{0, 0}, xyLoc{1,1}});
// 			}
// 		}

// 		WHEN("goal is not near") {
// 			result = GetPathWithCPD(pathHelper, xyLoc{0,0}, xyLoc{0,2}, path);
// 			THEN("") {
// 				debug("path is ", path);
// 				REQUIRE(result);
// 				REQUIRE(path == vector<xyLoc>{xyLoc{0, 0}, xyLoc{0, 1}, xyLoc{0,2}});
// 			}
// 		}

// 		WHEN("path is long") {
// 			result = GetPathWithCPD(pathHelper, xyLoc{3,1}, xyLoc{3,3}, path);
// 			THEN("") {
// 				debug("path is ", path);

// 				drawMapWithPath(std::cerr, mapData, width, height, path);
// 				REQUIRE(result);
// 				REQUIRE(path == vector<xyLoc>{xyLoc{3, 1}, xyLoc{3,0}, xyLoc{2, 0}, xyLoc{1,0}, xyLoc{1,1}, xyLoc{1,2}, xyLoc{1,3}, xyLoc{1,4}, xyLoc{2,4}, xyLoc{3,4}, xyLoc{3,3}});
// 			}
// 		}

// 		WHEN("we alter a single arc which is not involved at all in the path") {
// 			//FIXME continue from here
// 		}

// 		DeleteHelperOfSearch(pathHelper);
// 	}
// }

// SCENARIO("testing support functions", "[supportfunctions]") {
// 	GIVEN("basenae") {
// 		WHEN("fetching basename") {
// 			REQUIRE(getBaseNameAsString("a/b/c.txt") == std::string{"c.txt"});
// 			REQUIRE(getBaseNameAsString("/a/b/c.txt") == std::string{"c.txt"});
// 			REQUIRE(getBaseNameAsString("c.txt") == std::string{"c.txt"});
// 		}
// 	}
// }

// SCENARIO("testing support new feature for ICAPS2018", "[newfeature]") {

// 	MapLoader mapLoader;
// 	const string mapName = "square02.map";
// 	int width, height;
// 	vector<bool> mapData;

// 	GridMap gridMap = mapLoader.LoadMap(mapName.c_str());
// 	width = gridMap.getWidth();
// 	height = gridMap.getHeight();
// 	mapData = gridMap.getTraversableMask();

// 	Mapper mapper = Mapper{mapData, width, height};
// 	ListGraph listGraph1 = extract_graph(mapper);

// 	string cpdFilename = getCPDFilename(mapName);
// 	if (!isFileExists(cpdFilename.c_str())) {
// 		PreprocessMap(mapData, width, height, cpdFilename.c_str());
// 	}

// 	GIVEN("a ListGraph") {

// 		WHEN("checking if 2 equal AdjGraph are the same") {
// 			AdjGraph g1 = AdjGraph{listGraph1};
// 			AdjGraph g2 = AdjGraph{listGraph1};

// 			REQUIRE(g1 == g2);
// 		}

// 		WHEN("checking if 2 AdjGraph are the same") {
// 			ListGraph listgraph2 = extract_graph(mapper);

// 			AdjGraph g1 = AdjGraph{listGraph1};
// 			AdjGraph g2 = AdjGraph{listgraph2};

// 			REQUIRE(g1 == g2);
// 		}

// 		WHEN("getting weight arcs") {
// 			AdjGraph g = AdjGraph{listGraph1};

// 			REQUIRE(g.getWeightOfArc(mapper({0,0}), mapper({0,1})) == 1000);
// 			REQUIRE(g.getWeightOfArc(mapper({0,0}), mapper({1,1})) == 1414);

// 			REQUIRE(g.getWeightOfArc(mapper({1,1}), mapper({1,2})) == 1000);
// 			REQUIRE(g.getWeightOfArc(mapper({2,0}), mapper({1,1})) == 1414);
// 		}

// 		WHEN("setting weight arcs") {
// 			AdjGraph g = AdjGraph{listGraph1};
// 			AdjGraph g2 = AdjGraph{listGraph1};

// 			g.changeWeightOfArc(mapper({0,0}), mapper({0,1}), 500);

// 			REQUIRE(g.getWeightOfArc(mapper({0,0}), mapper({0,1})) == 500);
// 			REQUIRE(g.getWeightOfArc(mapper({0,1}), mapper({0,0})) == 500);

// 			REQUIRE(g != g2);
// 		}

// 		WHEN("printing adjacency graph") {
// 			AdjGraph g = AdjGraph{listGraph1};

// 			g.print(mapper, "adjacencyMap");
// 		}
// 	}

// 	GIVEN("start and goal where we want to compute the optimal path with Massimo Bono's new abstract method") {
// 		void* pathHelper = PrepareForSearch(mapData, width, height, cpdFilename.c_str());
// 		dpf::cost_t pathCost;
// 		std::vector<dpf::move_t> moves;
// 		std::vector<xyLoc> cells;
// 		bool result;

// 		WHEN("start=goal") {
// 			result = GetPathDataAllAtOnceWithCPD(pathHelper, {1,1}, {1,1}, pathCost, moves, cells);
// 			REQUIRE(result);
// 			REQUIRE(pathCost == 0);
// 			REQUIRE(moves == std::vector<dpf::move_t>{});
// 			REQUIRE(cells == std::vector<xyLoc>{{1,1}});
// 		}

// 		WHEN("start is just on the left of the goal") {
// 			result = GetPathDataAllAtOnceWithCPD(pathHelper, {1,1}, {2,1}, pathCost, moves, cells);
// 			REQUIRE(result);
// 			REQUIRE(pathCost == 1000);
// 			REQUIRE(moves == std::vector<dpf::move_t>{4});
// 			REQUIRE(cells == std::vector<xyLoc>{{1,1}, {2,1}});
// 		}

// 		WHEN("start is just on the diagonal of the goal") {
// 			result = GetPathDataAllAtOnceWithCPD(pathHelper, {1,1}, {0,0}, pathCost, moves, cells);
// 			REQUIRE(result);
// 			REQUIRE(pathCost == 1414);
// 			REQUIRE(moves == std::vector<dpf::move_t>{0});
// 			REQUIRE(cells == std::vector<xyLoc>{{1,1}, {0,0}});
// 		}

// 		WHEN("start is far from the goal") {
// 			result = GetPathDataAllAtOnceWithCPD(pathHelper, {0,0}, {2,0}, pathCost, moves, cells);
// 			REQUIRE(result);
// 			REQUIRE(pathCost == 2000);
// 			REQUIRE(moves == std::vector<dpf::move_t>{0, 1});
// 			REQUIRE(cells == std::vector<xyLoc>{{0,0}, {1,0}, {2,0}});
// 		}

// 		DeleteHelperOfSearch(pathHelper);
// 	}

// 	GIVEN("an A* implementation") {

// 		AdjGraph g = AdjGraph{listGraph1};
// 		string cpdFilename = getCPDFilename(mapName);
// 		auto h = CpdHeuristic{mapData, width, height, cpdFilename.c_str()};
// 		auto e = GridMapExpander{GridMap{mapData, width, height, 1000}, mapper, g, true};
// 		auto stats = QuickStatistics{};

// 		CPDAStarSearch<CpdHeuristic, GridMapExpander, QuickStatistics> aStar{&h, &e, &stats, false, false, false};
// 		aStar.set_verbose(true);

// 		WHEN("start=goal") {
// 			REQUIRE(aStar.get_path_as_vector(mapper({0,0}), mapper({0,0})) == std::vector<uint32_t>{
// 				static_cast<uint32_t>(mapper({0,0}))
// 			});
// 		}

// 		WHEN("goal just under start") {
// 			REQUIRE(aStar.get_path_as_vector(mapper({0,0}), mapper({0,1})) == std::vector<uint32_t>{
// 				static_cast<uint32_t>(mapper({0,0})),
// 						static_cast<uint32_t>(mapper({0,1}))
// 			});
// 		}

// 		WHEN("goal is on the diagonal") {
// 			REQUIRE(aStar.get_path_as_vector(mapper({0,0}), mapper({1,1})) == std::vector<uint32_t>{
// 				static_cast<uint32_t>(mapper({0,0})),
// 						static_cast<uint32_t>(mapper({1,1}))
// 			});
// 		}

// 		WHEN("goal on far on the right") {
// 			REQUIRE(h.h(h.getMapper()({0,0}), h.getMapper()({2,0})) == 2000);
// 			REQUIRE(h.h(h.getMapper()({1,0}), h.getMapper()({2,0})) == 1000);
// 			REQUIRE(h.h(h.getMapper()({1,1}), h.getMapper()({2,0})) == 1414);
// 			REQUIRE(h.h(h.getMapper()({2,0}), h.getMapper()({2,0})) == 0);
// 			REQUIRE(h.h(h.getMapper()({0,1}), h.getMapper()({2,0})) == (1414+1000));
// 			REQUIRE(aStar.get_path_as_vector(h.getMapper()({0,0}), h.getMapper()({2,0})) == std::vector<uint32_t>{
// 				static_cast<uint32_t>(h.getMapper()({0,0})),
// 						static_cast<uint32_t>(h.getMapper()({1,0})),
// 						static_cast<uint32_t>(h.getMapper()({2,0}))
// 			});
// 		}

// 		//		Massimo Bono: the code under first_move of CPD fails, so I assume it could never be done :\
// 		//		WHEN("looking for an unexisting path") {
// 		//			const string mapName3 = "square03.map";
// 		//			mapLoader.LoadMap(mapName3.c_str(), mapData, width, height);
// 		//			mapper = Mapper{mapData, width, height};
// 		//			listGraph1 = extract_graph(mapper);
// 		//			REQUIRE(aStar.get_path_as_vector(mapper({0,0}), mapper({4,4})) == std::vector<uint32_t>{});
// 		//		}

// 	}

// 	GIVEN("testing overflow") {
// 		safe_int a;
// 		safe_int b;

// 		WHEN("checking sum of positive") {
// 			//no overflow
// 			a=6; b=7;
// 			REQUIRE((a+b) == 13);
// 			//no overflow
// 			a=INT_MAX-10; b = 10;
// 			REQUIRE((a+b) == INT_MAX);
// 			//overflow
// 			a=INT_MAX-10; b = 11;
// 			REQUIRE((a+b) == INT_MAX);
// 		}
// 		WHEN("checking sum of negative") {

// 			a=6; b=-1;
// 			REQUIRE(a+b == 5);

// 			a=INT_MIN+10; b = -10;
// 			REQUIRE((a+b) == INT_MIN);

// 			a=INT_MIN+10; b = -11;
// 			REQUIRE((a+b) == INT_MIN);
// 		}

// 		WHEN("returning to unsafe int") {
// 			a=6;
// 			REQUIRE(static_cast<int>(a) == 6);

// 			a=INT_MAX;
// 			REQUIRE((static_cast<int>(a) + 6) != INT_MAX);
// 		}

// 	}

// 	GIVEN("testing adjacent locations") {

// 		WHEN("location has same x") {

// 			REQUIRE(xyLoc{5,6}.isAdjacentTo(xyLoc{5,5}));
// 			REQUIRE(xyLoc{5,6}.isAdjacentTo(xyLoc{5,5}));

// 			REQUIRE(!xyLoc{5,6}.isAdjacentTo(xyLoc{5,8}));
// 			REQUIRE(!xyLoc{5,8}.isAdjacentTo(xyLoc{5,6}));
// 		}

// 		WHEN("location has same y") {
// 			REQUIRE(xyLoc{5,6}.isAdjacentTo(xyLoc{4,6}));
// 			REQUIRE(xyLoc{4,6}.isAdjacentTo(xyLoc{5,6}));

// 			REQUIRE(!xyLoc{5,6}.isAdjacentTo(xyLoc{3,6}));
// 			REQUIRE(!xyLoc{3,6}.isAdjacentTo(xyLoc{5,6}));
// 		}

// 		WHEN("locations are diagonal") {
// 			REQUIRE(xyLoc{5,6}.isAdjacentTo(xyLoc{6,7}));
// 			REQUIRE(xyLoc{6,7}.isAdjacentTo(xyLoc{5,6}));

// 			REQUIRE(!xyLoc{5,6}.isAdjacentTo(xyLoc{6,8}));
// 			REQUIRE(!xyLoc{6,8}.isAdjacentTo(xyLoc{5,6}));
// 		}
// 	}

// 	GIVEN("testing converting GridMap to ListGraph") {
// 		std::vector<dpf::celltype_t> rawMap = std::vector<dpf::celltype_t>{
// 			1000,1000,2000,
// 			1000,2000,2000,
// 			1000,2000,3000
// 		};

// 		GridMap map{rawMap, 3, 3};
// 		Mapper mapper{map};
// 		ListGraph listGraph = getListGraphFrom(map, mapper, MULTI_TERRAIN_STRATEGY);

// 		REQUIRE(listGraph.node_count() == 9);

// 	}

// 	GIVEN("testing location inside rectangle") {

// 		xyLoc minPoint;
// 		xyLoc maxPoint;

// 		WHEN("path is one element big") {
// 			//gridmap_path p = gridmap_path{std::vector{xyLoc{5,5}, xyLoc{5,6}, xyLoc{6,6}, xyLoc{7,7} }};
// 			gridmap_path p = gridmap_path{std::vector<xyLoc>{xyLoc{5,5}}};
// 			p.getRanges(minPoint, maxPoint);

// 			REQUIRE(minPoint == xyLoc{5,5});
// 			REQUIRE(maxPoint == xyLoc{5,5});
// 		}

// 		WHEN("path is horizontal") {
// 			gridmap_path p = gridmap_path{std::vector<xyLoc>{xyLoc{5,5}, xyLoc{6,5}, xyLoc{7,5}}};
// 			p.getRanges(minPoint, maxPoint);

// 			REQUIRE(minPoint == xyLoc{5,5});
// 			REQUIRE(maxPoint == xyLoc{7,5});
// 		}

// 		WHEN("path is vertical") {
// 			gridmap_path p = gridmap_path{std::vector<xyLoc>{xyLoc{5,5}, xyLoc{5,6}, xyLoc{5,7}}};
// 			p.getRanges(minPoint, maxPoint);

// 			REQUIRE(minPoint == xyLoc{5,5});
// 			REQUIRE(maxPoint == xyLoc{5,7});
// 		}

// 		WHEN("path is diagonla") {
// 			gridmap_path p = gridmap_path{std::vector<xyLoc>{xyLoc{5,5}, xyLoc{6,6}, xyLoc{7,7}}};
// 			p.getRanges(minPoint, maxPoint);

// 			REQUIRE(minPoint == xyLoc{5,5});
// 			REQUIRE(maxPoint == xyLoc{7,7});
// 		}

// 		WHEN("path is generic") {
// 			gridmap_path p = gridmap_path{std::vector<xyLoc>{xyLoc{5,5}, xyLoc{6,5}, xyLoc{6,6}, xyLoc{7,7}, xyLoc{8,7}, xyLoc{7,6}}};
// 			p.getRanges(minPoint, maxPoint);

// 			REQUIRE(minPoint == xyLoc{5,5});
// 			REQUIRE(maxPoint == xyLoc{8,7});
// 		}
// 	}

// 	GIVEN("testing strategy for processing amps with different terrains") {
// 		const string dustwallowkeys = "dustwallowkeys.map";
// 		MapLoaderFactory mapLoaderFactory{};
// 		AbstractMapLoader* mapLoader = mapLoaderFactory.get(dustwallowkeys.c_str());

// 		GridMap map = mapLoader->LoadMap(dustwallowkeys.c_str());
// 		Mapper mapper{map};

// 		WHEN("horizontal on all dot") {
// 			REQUIRE(MULTI_TERRAIN_STRATEGY(map, mapper, xyLoc{128,29}, xyLoc{129,29}) == 1000);
// 		}

// 		WHEN("diagonal on all dot") {
// 			REQUIRE(MULTI_TERRAIN_STRATEGY(map, mapper, xyLoc{128,29}, xyLoc{129,30}) == 1414);
// 		}

// 		WHEN("horizontal on all W") {
// 			REQUIRE(MULTI_TERRAIN_STRATEGY(map, mapper, xyLoc{17,20}, xyLoc{18,20}) == 2500);
// 		}

// 		WHEN("horizontal dot-S") {
// 			REQUIRE(MULTI_TERRAIN_STRATEGY(map, mapper, xyLoc{138,31}, xyLoc{139,31}) == 1500);
// 		}

// 		WHEN("diagonal dot-dot-S-S") {
// 			REQUIRE(MULTI_TERRAIN_STRATEGY(map, mapper, xyLoc{138,31}, xyLoc{139,32}) == (2121));
// 		}

// 		WHEN("diagonal S-W-dot-dot") {
// 			REQUIRE(MULTI_TERRAIN_STRATEGY(map, mapper, xyLoc{127,26}, xyLoc{128,27}) == (2297));
// 		}

// 		delete mapLoader;
// 	}

// 	GIVEN("testing get_path_as_vector of Dijkstra algorithm") {
// 		const string square03 = "square03.map";
// 		int width2, height2;
// 		vector<bool> mapData2;
// 		GridMap gridMap2 = mapLoader.LoadMap(square03.c_str());
// 		width2 = gridMap2.getWidth();
// 		height2 = gridMap2.getHeight();
// 		mapData2 = gridMap2.getTraversableMask();

// 		Mapper mapper2 = Mapper{mapData2, width2, height2};
// 		ListGraph listGraph2 = extract_graph(mapper2);
// 		AdjGraph g = AdjGraph{listGraph2};
// 		EarlyStopDijkstra algorithm = EarlyStopDijkstra{g, true};
// 		xyLoc start;
// 		xyLoc goal;

// 		info("g has", g.node_count(), "nodes");

// 		WHEN("we're already in the goal") {
// 			start = {0,0};
// 			goal = {0,0};
// 			const std::vector<dpf::nodeid_t>& optimalMoves = algorithm.run(mapper2(start), mapper2(goal));
// 			std::vector<xyLoc> path = algorithm.getPathAsVector(mapper2(start), mapper2(goal), mapper2, optimalMoves);

// 			debug("path is", path);
// 			REQUIRE((path == std::vector<xyLoc>{xyLoc{0,0}}));
// 		}

// 		WHEN("we're immediately adujacent to the goal") {
// 			start = {0,0};
// 			goal = {1,0};
// 			auto labelPrinter = DefaultLabelPrinter{};
// 			auto colorPrinter = DefaultColorPrinter{};
// 			printGridMap<DefaultLabelPrinter, DefaultColorPrinter>(mapData2, width2, height2, mapper2, goal, "dijkstra", labelPrinter, colorPrinter);

// 			const std::vector<dpf::nodeid_t>& optimalMoves = algorithm.run(mapper2(start), mapper2(goal));
// 			std::vector<xyLoc> path = algorithm.getPathAsVector(mapper2(start), mapper2(goal), mapper2, optimalMoves);




// 			debug("path is", path);
// 			REQUIRE((path == std::vector<xyLoc>{
// 				xyLoc{0,0}, xyLoc{1,0}
// 			}));
// 		}

// 		WHEN("we're immediately adujacent to the goal on the diagonal") {
// 			start = {0,0};
// 			goal = {1,1};
// 			const std::vector<dpf::nodeid_t>& optimalMoves = algorithm.run(mapper2(start), mapper2(goal));
// 			std::vector<xyLoc> path = algorithm.getPathAsVector(mapper2(start), mapper2(goal), mapper2, optimalMoves);

// 			debug("path is", path);
// 			REQUIRE((path == std::vector<xyLoc>{
// 				xyLoc{0,0}, xyLoc{1,1}
// 			}));
// 		}

// 		WHEN("generic path") {
// 			start = {0,0};
// 			goal = {4,0};
// 			const std::vector<dpf::nodeid_t>& optimalMoves = algorithm.run(mapper2(start), mapper2(goal));
// 			std::vector<xyLoc> path = algorithm.getPathAsVector(mapper2(start), mapper2(goal), mapper2, optimalMoves);

// 			debug("path is", path);
// 			REQUIRE((path == std::vector<xyLoc>{
// 				xyLoc{0,0}, xyLoc{1,0}, xyLoc{2,0}, xyLoc{3,0}, xyLoc{4,0}
// 			}));
// 		}

// 		WHEN("generic long path") {
// 			start = {0,0};
// 			goal = {4,4};

// 			const std::vector<dpf::nodeid_t>& optimalMoves = algorithm.run(mapper2(start), mapper2(goal));
// 			std::vector<xyLoc> path = algorithm.getPathAsVector(mapper2(start), mapper2(goal), mapper2, optimalMoves);

// 			debug("path is", path);
// 			REQUIRE(((path == std::vector<xyLoc>{
// 				xyLoc{0,0}, xyLoc{0,1}, xyLoc{1,2}, xyLoc{1,3}, xyLoc{2,4}, xyLoc{3,4}, xyLoc{4,4}
// 			}) || (path == std::vector<xyLoc>{
// 				xyLoc{0,0}, xyLoc{0,1}, xyLoc{0,2}, xyLoc{1,3}, xyLoc{2,4}, xyLoc{3,4}, xyLoc{4,4}
// 			})
// 			));
// 		}

// 	}

// 	GIVEN("printing maps") {

// 		std::unordered_map<dpf::celltype_t, color_t> colorMapping;
// 		colorMapping[1000] = WHITE;
// 		colorMapping[1500] = GREEN;
// 		colorMapping[2000] = BLUE;
// 		colorMapping[2500] = DARK_BLUE;

// 		WHEN("printing") {
// 			MapLoaderFactory factory{};
// 			AbstractMapLoader* mapLoader = factory.get("dustwallowkeys.map");

// 			GridMap mapToPrint = mapLoader->LoadMap("dustwallowkeys.map");

// 			PPMImage image = mapToPrint.getImage(colorMapping, BLACK);
// 			image.saveAndConvertIntoPNG("dustwallowkeys");

// 			delete mapLoader;
// 		}

// 		WHEN("summing colors") {
// 			color_t red{250, 0, 0};
// 			color_t dark_red{150, 0, 0};
// 			color_t green{0, 250, 0};

// 			REQUIRE(red.lerp(green) == color_t{125,125,0});
// 			REQUIRE(red.lerp(dark_red) == color_t{200,0,0});
// 		}

// 		WHEN("summing") {
// 			GridMap a{std::vector<dpf::celltype_t>{
// 				1000,1000,1000,2000,2000,
// 				1000,1000,1000,2000,2000,
// 				1000,1000,2000,2000,2500,
// 				1000,1000,1000,2000,2000,
// 				1000,1000,1000,1000,1000,
// 			}, 5, 5};

// 			GridMap b{std::vector<dpf::celltype_t>{
// 				1000,2500,2500,1000,1000,
// 				1000,1000,2500,2500,1000,
// 				1000,1000,1000,2500,2500,
// 				1000,1000,1000,1000,1000,
// 				1000,1000,1000,1000,1000,
// 			}, 5, 5};

// 			PPMImage aImage = a.getImage(colorMapping, BLACK);
// 			PPMImage bImage = b.getImage(colorMapping, BLACK);
// 			PPMImage sum = aImage + bImage;
// 			sum.saveAndConvertIntoPNG("sumImageTest");
// 		}
// 	}

// }

// SCENARIO("testing important new features for ICAPS2018", "[newfeature]") {

// 	MapLoader mapLoader;
// 	const string mapName = "square03.map";
// 	int width, height;
// 	vector<bool> mapData;

// 	GridMap gridMap = mapLoader.LoadMap(mapName.c_str());
// 	width = gridMap.getWidth();
// 	height = gridMap.getHeight();
// 	mapData = gridMap.getTraversableMask();

// 	Mapper mapperTmp = Mapper{mapData, width, height};
// 	ListGraph listGraph1 = extract_graph(mapperTmp);


// 	const string cpdFilename = getCPDFilename(mapName);
// 	auto h = CpdHeuristic{mapData, width, height, cpdFilename.c_str()};
// 	AdjGraph graphWithAlteringArcs = AdjGraph{h.getGraph()};
// 	const Mapper& mapper = h.getMapper();
// 	auto e = GridMapExpander{GridMap{mapData, width, height, 1000}, h.getMapper(), graphWithAlteringArcs, true};
// 	auto stats = QuickStatistics{};

// 	critical("************************************************************************************************");
// 	critical("********************************* NEW TEST *****************************************************");
// 	critical("************************************************************************************************");

// 	GIVEN("A* with no optimizations") {
// 		CPDAStarSearch<CpdHeuristic, GridMapExpander, QuickStatistics> aStar{&h, &e, &stats, false, false, false};
// 		aStar.set_verbose(true);

// 		WHEN("altering a weight of an arc changes A* costs but not CPDs") {
// 			REQUIRE(graphWithAlteringArcs.getWeightOfArc(0, 1) == 1000);
// 			REQUIRE(h.h(mapper({0,0}), mapper({1,0})) == 1000);

// 			graphWithAlteringArcs.changeWeightOfArc(0, 1, 2000);

// 			REQUIRE(graphWithAlteringArcs.getWeightOfArc(0, 1) == 2000);
// 			REQUIRE(h.h(mapper({0,0}), mapper({1,0})) == 1000);

// 			REQUIRE(h.hasHeuristicComputedAPath());
// 			REQUIRE(h.getCPDPathCost() == 1000);
// 			REQUIRE(h.getCPDPathCostInMap(graphWithAlteringArcs) == 2000);
// 			REQUIRE(!h.isCPDPathClearFromArcModifications(graphWithAlteringArcs));
// 		}

// 		WHEN("perturbation of single arc does not alter optimal path") {
// 			graphWithAlteringArcs.changeWeightOfArc(mapper({0,3}), mapper({0,4}), 5000);

// 			REQUIRE(aStar.get_path_as_vector(mapper({0,0}), mapper({4,0})) == std::vector<uint32_t>{
// 				static_cast<uint32_t>(mapper({0,0})),
// 						static_cast<uint32_t>(mapper({1,0})),
// 						static_cast<uint32_t>(mapper({2,0})),
// 						static_cast<uint32_t>(mapper({3,0})),
// 						static_cast<uint32_t>(mapper({4,0}))
// 			});

// 			//debug("nodes expanded are", aStar.getNodeExpandedList());
// 			REQUIRE(aStar.get_nodes_expanded() == 4);
// 			REQUIRE(aStar.get_nodes_generated() == 9);
// 			//{0,0} = 1
// 			//{0,0} plus all its successors (both expanded or not) = 3
// 			//{1,0} = 1
// 			//{1,0} plus all its successors (both expanded or not) = 5
// 			//{2,0} = 1
// 			//{2,0} plus all its successors (both expanded or not) = 5
// 			//{3,0} = 1
// 			//{3,0} plus all its successors (both expanded or not) = 5
// 			//{4,0} = 1
// 			//TOTAL = 23
// 			REQUIRE(aStar.get_nodes_touched() == 23);
// 		}

// 		WHEN("perturbation of a single arc does alter the optimal path") {
// 			graphWithAlteringArcs.changeWeightOfArc(mapper({1,0}), mapper({2,0}), 5000);

// 			std::vector<uint32_t> path = aStar.get_path_as_vector(mapper({0,0}), mapper({4,0}));
// 			debug("path is", path);
// 			REQUIRE(((path == std::vector<uint32_t>{
// 				static_cast<uint32_t>(mapper({0,0})),
// 						static_cast<uint32_t>(mapper({1,0})),
// 						static_cast<uint32_t>(mapper({2,1})),
// 						static_cast<uint32_t>(mapper({3,0})),
// 						static_cast<uint32_t>(mapper({4,0}))
// 			}) ||
// 					(path == std::vector<uint32_t>{
// 				static_cast<uint32_t>(mapper({0,0})),
// 						static_cast<uint32_t>(mapper({1,0})),
// 						static_cast<uint32_t>(mapper({2,1})),
// 						static_cast<uint32_t>(mapper({3,1})),
// 						static_cast<uint32_t>(mapper({4,0}))
// 			}) ||
// 			(path == std::vector<uint32_t>{
// 				static_cast<uint32_t>(mapper({0,0})),
// 						static_cast<uint32_t>(mapper({1,1})),
// 						static_cast<uint32_t>(mapper({2,0})),
// 						static_cast<uint32_t>(mapper({3,0})),
// 						static_cast<uint32_t>(mapper({4,0}))
// 			})
// 			));

// 			REQUIRE(aStar.get_nodes_expanded() == 4);
// 			REQUIRE(aStar.get_nodes_generated() == 9);
// 			REQUIRE(aStar.get_nodes_touched() == 23);
// 		}
// 	}

// 	GIVEN ("A* with early stop enabled") {
// 		CPDAStarSearch<CpdHeuristic, GridMapExpander, QuickStatistics> aStar{&h, &e, &stats, false, true, false};
// 		//TODO remove dynamic_pathfinding_astar<CpdHeuristic, QuickStatistics> aStar{&h, &e, &stats, false, true, false, false, 0, 1.0, false, 1.0};
// 		aStar.set_verbose(true);

// 		WHEN("perturbation of single arc does not alter optimal path") {
// 			graphWithAlteringArcs.changeWeightOfArc(mapper({0,3}), mapper({0,4}), 5000);

// 			auto path = aStar.get_path_as_vector(mapper({0,0}), mapper({4,0}));
// 			REQUIRE(path == std::vector<uint32_t>{
// 				static_cast<uint32_t>(mapper({0,0})),
// 						static_cast<uint32_t>(mapper({1,0})),
// 						static_cast<uint32_t>(mapper({2,0})),
// 						static_cast<uint32_t>(mapper({3,0})),
// 						static_cast<uint32_t>(mapper({4,0}))
// 			});

// 			REQUIRE(aStar.get_nodes_expanded() == 0);
// 			REQUIRE(aStar.get_nodes_generated() == 0);
// 			REQUIRE(aStar.get_nodes_touched() == 0);
// 		}

// 		WHEN("perturbation of a single arc does alter the optimal path") {
// 			graphWithAlteringArcs.changeWeightOfArc(mapper({1,0}), mapper({2,0}), 5000);

// 			auto path = aStar.get_path_as_vector(mapper({0,0}), mapper({4,0}));


// 			printCPD(mapName.c_str(), cpdFilename.c_str(), xyLoc{4, 0}, std::string{"cpd40"});

// 			debug("path obtained is ", path);
// 			REQUIRE(((path == std::vector<uint32_t>{
// 				static_cast<uint32_t>(mapper({0,0})),
// 						static_cast<uint32_t>(mapper({1,0})),
// 						static_cast<uint32_t>(mapper({2,1})),
// 						static_cast<uint32_t>(mapper({3,0})),
// 						static_cast<uint32_t>(mapper({4,0}))
// 			}) ||
// 					(path == std::vector<uint32_t>{
// 				static_cast<uint32_t>(mapper({0,0})),
// 						static_cast<uint32_t>(mapper({1,1})),
// 						static_cast<uint32_t>(mapper({2,0})),
// 						static_cast<uint32_t>(mapper({3,0})),
// 						static_cast<uint32_t>(mapper({4,0}))
// 			})));

// 			debug("nodes touched are", aStar.get_nodes_touched());
// 			REQUIRE(aStar.get_nodes_expanded() == 2);
// 			REQUIRE(((aStar.get_nodes_generated() > 0) && (aStar.get_nodes_generated() < 6)));
// 			REQUIRE(((aStar.get_nodes_touched() > 2) && (aStar.get_nodes_touched() < 12)));
// 		}

// 		WHEN("perturbation of multiple arcs does alter the optimal path") {
// 			graphWithAlteringArcs.changeWeightOfArc(mapper({1,0}), mapper({2,0}), 5000);
// 			graphWithAlteringArcs.changeWeightOfArc(mapper({3,0}), mapper({4,0}), 5000);

// 			auto path = aStar.get_path_as_vector(mapper({0,0}), mapper({4,0}));

// 			printCPD(mapName.c_str(), cpdFilename.c_str(), xyLoc{4, 0}, std::string{"cpd40"});
// 			//			debug("expanded nodes", aStar.getNodeExpandedList());
// 			//			debug("generated nodes", aStar.getNodeGeneratedList());
// 			//			debug("touched nodes", aStar.getNodeTouchedList());

// 			debug("path obtained is ", path);
// 			REQUIRE(((path == std::vector<uint32_t>{
// 				static_cast<uint32_t>(mapper({0,0})),
// 						static_cast<uint32_t>(mapper({1,0})),
// 						static_cast<uint32_t>(mapper({2,1})),
// 						static_cast<uint32_t>(mapper({3,1})),
// 						static_cast<uint32_t>(mapper({4,0}))
// 			})));

// 			debug("nodes generated are", aStar.get_nodes_generated());
// 			REQUIRE(aStar.get_nodes_expanded() == 4); //we pop the starting node and when we analyze {1,1} we stop
// 			REQUIRE(((aStar.get_nodes_generated() >= 8) && (aStar.get_nodes_generated() <= 9)));
// 			REQUIRE(((aStar.get_nodes_touched() >= 18) && (aStar.get_nodes_touched() <= 23)));
// 		}
// 	}

// 	GIVEN ("A* with fixed upper bound enabled") {
// 		CPDAStarSearch<CpdHeuristic, GridMapExpander, QuickStatistics> aStar{&h, &e, &stats, true, false, false};
// 		//TODO remove dynamic_pathfinding_astar<CpdHeuristic, QuickStatistics> aStar{&h, &e, &stats, true, false, false, false, 0, 1.0, false, 1.0};
// 		aStar.set_verbose(true);

// 		WHEN("perturbation of single arc does not alter optimal path") {
// 			graphWithAlteringArcs.changeWeightOfArc(mapper({0,3}), mapper({0,4}), 1100);

// 			//set the fix upperbound
// 			h.h(mapper({0,0}), mapper({4,0}));
// 			debug("cost of the optyimal path in the new map is", h.getCPDPathCostInMap(graphWithAlteringArcs));
// 			REQUIRE((h.getCPDPathCostInMap(graphWithAlteringArcs) == 4000));
// 			aStar.set_fixed_upperbound(h.getCPDPathCostInMap(graphWithAlteringArcs));
// 			//run a star
// 			auto path = aStar.get_path_as_vector(mapper({0,0}), mapper({4,0}));
// 			REQUIRE(path == std::vector<uint32_t>{
// 				static_cast<uint32_t>(mapper({0,0})),
// 						static_cast<uint32_t>(mapper({1,0})),
// 						static_cast<uint32_t>(mapper({2,0})),
// 						static_cast<uint32_t>(mapper({3,0})),
// 						static_cast<uint32_t>(mapper({4,0}))
// 			});

// 			//TODO readd!
// 			//			REQUIRE(aStar.getNodeExpandedList() == std::vector<uint32_t>{
// 			//				(uint32_t)mapper({0,0}), (uint32_t)mapper({1,0}), (uint32_t)mapper({2,0}), (uint32_t)mapper({3,0})
// 			//			});
// 			REQUIRE(aStar.get_nodes_generated() == 4); //all the other nodes in {x,1} are not even generated!
// 			REQUIRE(aStar.get_nodes_touched() == 23);
// 		}

// 		WHEN("perturbation arcs does alter the optimal path") {

// 			//cost by going through 1,0: original=4000 revised=5000
// 			graphWithAlteringArcs.changeWeightOfArc(mapper({0,0}), mapper({1,0}), 2000);
// 			//cost by going through 0,1: original=5414 revised=6414
// 			graphWithAlteringArcs.changeWeightOfArc(mapper({0,0}), mapper({0,1}), 2000);
// 			//cost by going through 1,1: original=5414 revised=6414
// 			graphWithAlteringArcs.changeWeightOfArc(mapper({0,0}), mapper({1,1}), 2414);

// 			//set the fix upperbound
// 			h.h(mapper({0,0}), mapper({0,4}));
// 			aStar.set_fixed_upperbound(h.getCPDPathCostInMap(graphWithAlteringArcs));
// 			//run a star
// 			auto path = aStar.get_path_as_vector(mapper({0,0}), mapper({4,0}));


// 			//			debug("expanded nodes", aStar.getNodeExpandedList());
// 			//			debug("generated nodes", aStar.getNodeGeneratedList());
// 			//			debug("touched nodes", aStar.getNodeTouchedList());

// 			debug("path obtained is ", path);
// 			//the path is unchanged
// 			REQUIRE(((path == std::vector<dpf::nodeid_t>{
// 				(mapper({0,0})),
// 						(mapper({1,0})),
// 						(mapper({2,0})),
// 						(mapper({3,0})),
// 						(mapper({4,0}))
// 			})));

// 			//BUT we generate less nodes!
// 			REQUIRE(aStar.get_nodes_expanded() == 4);
// 			REQUIRE(((aStar.get_nodes_generated() == 4)));
// 			REQUIRE(aStar.get_nodes_touched() == 23);
// 		}

// 	}

// 	GIVEN("testing CPD with caching") {
// 		AdjGraph newMap = AdjGraph{h.getGraph()};
// 		CpdCacheHeuristic h2 = CpdCacheHeuristic{h, newMap};
// 		CPDAStarSearch<CpdHeuristic, GridMapExpander, QuickStatistics> aStar{&h2, &e, &stats, true, true, false};
// 		//TODO remove dynamic_pathfinding_astar<CpdCacheHeuristic, QuickStatistics> aStar{&h2, &e, &stats, true, true, false, false, 0, 1.0, false, 1.0};
// 		aStar.set_verbose(true);

// 		WHEN("searching a path with A*") {
// 			xyLoc start = xyLoc{0,0};
// 			xyLoc goal = xyLoc{4,4};
// 			uint32_t startid = mapper(start);
// 			uint32_t goalid = mapper(goal);

// 			printCPD(mapName.c_str(), cpdFilename.c_str(), goal, std::string{"to_44"});

// 			std::vector<uint32_t> path = aStar.get_path_as_vector(startid, goalid);

// 			h2.h(startid, goalid);

// 			debug("printing cost");
// 			//debug("cost is", h2.getOriginalCostOfGoingToGoalFrom(mapper(xyLoc{0,0})));
// 			REQUIRE(h2.isDataInCache(mapper(xyLoc{0,0})));
// 			REQUIRE(h2.isDataInCache(mapper(xyLoc{0,1})));
// 			REQUIRE(h2.isDataInCache(mapper(xyLoc{0,2})));
// 			REQUIRE(h2.isDataInCache(mapper(xyLoc{1,3})));
// 			REQUIRE(h2.isDataInCache(mapper(xyLoc{2,4})));
// 			REQUIRE(h2.isDataInCache(mapper(xyLoc{3,4})));
// 			REQUIRE(h2.isDataInCache(mapper(xyLoc{4,4})));
// 			REQUIRE(h2.getOriginalCostOfGoingToGoalFrom(mapper(xyLoc{0,0})) == 6828);
// 			REQUIRE(h2.getOriginalCostOfGoingToGoalFrom(mapper(xyLoc{0,1})) == 5828);
// 			REQUIRE(h2.getOriginalCostOfGoingToGoalFrom(mapper(xyLoc{0,2})) == 4828);
// 			REQUIRE(h2.getOriginalCostOfGoingToGoalFrom(mapper(xyLoc{1,3})) == 3414);
// 			REQUIRE(h2.getOriginalCostOfGoingToGoalFrom(mapper(xyLoc{2,4})) == 2000);
// 			REQUIRE(h2.getOriginalCostOfGoingToGoalFrom(mapper(xyLoc{3,4})) == 1000);
// 			REQUIRE(h2.getOriginalCostOfGoingToGoalFrom(mapper(xyLoc{4,4})) == 0000);
// 			//other cell not in the best path are not cached
// 			REQUIRE(!h2.isDataInCache(mapper(xyLoc{1,0})));
// 			REQUIRE(!h2.isDataInCache(mapper(xyLoc{2,0})));
// 			REQUIRE(!h2.isDataInCache(mapper(xyLoc{1,1})));
// 		}

// 	}

// 	GIVEN("testing CPD heuristic when overflow on costs is bound to happen") {

// 		CPDAStarSearch<CpdHeuristic, GridMapExpander, QuickStatistics> aStar{&h, &e, &stats, true, true, false};
// 		//TODO remove dynamic_pathfinding_astar<CpdHeuristic, QuickStatistics> aStar{&h, &e, &stats, true, true, false, false, 0, 1.0, false, 1.0};
// 		aStar.set_verbose(true);

// 		WHEN("no overflow, jjust big numbers") {
// 			ITERATE_OVER_EDGES(graphWithAlteringArcs, sourceId, i, arc) {
// 				info("arc weight is", arc.weight);
// 				info("number is", (INT_MAX/10));
// 				info("division will be", (arc.weight + (INT_MAX/10)));
// 				graphWithAlteringArcs.changeWeightOfDirectedArc(
// 						static_cast<int>(sourceId), arc.target,
// 						arc.weight + (INT_MAX/10));
// 			}

// 			REQUIRE(graphWithAlteringArcs.getWeightOfArc(mapper({0,0}), mapper({0,1})) == (1000 + INT_MAX/10));
// 			REQUIRE(graphWithAlteringArcs.getWeightOfArc(mapper({0,0}), mapper({1,1})) == (1414 + INT_MAX/10));

// 			std::vector<dpf::nodeid_t> path = aStar.get_path_as_vector(mapper({0,0}), mapper({4,0}), false);
// 			REQUIRE(path == std::vector<dpf::nodeid_t>{
// 				mapper({0,0}),
// 						mapper({1,0}),
// 						mapper({2,0}),
// 						mapper({3,0}),
// 						mapper({4,0})
// 			});

// 			THEN("") {

// 				critical("max is", INT_MAX);
// 				critical("/10 is", INT_MAX/10);
// 				critical("g 0 0 ", e.generate(mapper({0,0}))->get_g());
// 				critical("g 1 0 ", e.generate(mapper({1,0}))->get_g());
// 				critical("g 2 0 ", e.generate(mapper({2,0}))->get_g());
// 				critical("g 3 0 ", e.generate(mapper({3,0}))->get_g());
// 				critical("g 4 0 ", e.generate(mapper({4,0}))->get_g());

// 				critical("g 0 0 ", e.generate(mapper({0,0}))->get_f());
// 				critical("g 1 0 ", e.generate(mapper({1,0}))->get_f());
// 				critical("g 2 0 ", e.generate(mapper({2,0}))->get_f());
// 				critical("g 3 0 ", e.generate(mapper({3,0}))->get_f());
// 				critical("g 4 0 ", *(e.generate(mapper({4,0}))));
// 				REQUIRE(e.generate(mapper({0,0}))->get_g() == 0*(1000 + INT_MAX/10));
// 				REQUIRE(e.generate(mapper({1,0}))->get_g() == 1*(1000 + INT_MAX/10));
// 				REQUIRE(e.generate(mapper({2,0}))->get_g() == 2*(1000 + INT_MAX/10));
// 				REQUIRE(e.generate(mapper({3,0}))->get_g() == 3*(1000 + INT_MAX/10));
// 				REQUIRE(e.generate(mapper({4,0}))->get_g() == 4*(1000 + INT_MAX/10));

// 				REQUIRE(e.generate(mapper({0,0}))->get_f() == (4000 + 0*(1000 + INT_MAX/10)));
// 				REQUIRE(e.generate(mapper({1,0}))->get_f() == (3000 + 1*(1000 + INT_MAX/10)));
// 				REQUIRE(e.generate(mapper({2,0}))->get_f() == (2000 + 2*(1000 + INT_MAX/10)));
// 				REQUIRE(e.generate(mapper({3,0}))->get_f() == (1000 + 3*(1000 + INT_MAX/10)));
// 				REQUIRE(e.generate(mapper({4,0}))->get_f() == (0000 + 4*(1000 + INT_MAX/10)));
// 			}
// 		}

// 		WHEN("overflow on the goal") {
// 			ITERATE_OVER_EDGES(graphWithAlteringArcs, sourceId, i, arc) {
// 				info("arc weight is", arc.weight);
// 				info("number is", (INT_MAX/4));
// 				info("division will be", (arc.weight + (INT_MAX/4)));
// 				graphWithAlteringArcs.changeWeightOfDirectedArc(
// 						static_cast<int>(sourceId), arc.target,
// 						arc.weight + (INT_MAX/4));
// 			}

// 			REQUIRE(graphWithAlteringArcs.getWeightOfArc(mapper({0,0}), mapper({0,1})) == (1000 + INT_MAX/4));
// 			REQUIRE(graphWithAlteringArcs.getWeightOfArc(mapper({0,0}), mapper({1,1})) == (1414 + INT_MAX/4));

// 			std::vector<dpf::nodeid_t> path = aStar.get_path_as_vector(mapper({0,0}), mapper({4,0}), false);
// 			REQUIRE(path == std::vector<dpf::nodeid_t>{
// 				mapper({0,0}),
// 						mapper({1,0}),
// 						mapper({2,0}),
// 						mapper({3,0}),
// 						mapper({4,0})
// 			});

// 			THEN("") {

// 				critical("max is", INT_MAX);
// 				critical("/4 is", INT_MAX/4);
// 				critical("g 0 0 ", e.generate(mapper({0,0}))->get_g());
// 				critical("g 1 0 ", e.generate(mapper({1,0}))->get_g());
// 				critical("g 2 0 ", e.generate(mapper({2,0}))->get_g());
// 				critical("g 3 0 ", e.generate(mapper({3,0}))->get_g());
// 				critical("g 4 0 ", e.generate(mapper({4,0}))->get_g());

// 				critical("g 0 0 ", e.generate(mapper({0,0}))->get_f());
// 				critical("g 1 0 ", e.generate(mapper({1,0}))->get_f());
// 				critical("g 2 0 ", e.generate(mapper({2,0}))->get_f());
// 				critical("g 3 0 ", e.generate(mapper({3,0}))->get_f());
// 				critical("g 4 0 ", *(e.generate(mapper({4,0}))));
// 				REQUIRE(e.generate(mapper({0,0}))->get_g() == safe_int{0}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{4}));
// 				REQUIRE(e.generate(mapper({1,0}))->get_g() == safe_int{1}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{4}));
// 				REQUIRE(e.generate(mapper({2,0}))->get_g() == safe_int{2}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{4}));
// 				REQUIRE(e.generate(mapper({3,0}))->get_g() == safe_int{3}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{4}));
// 				REQUIRE(e.generate(mapper({4,0}))->get_g() == safe_int{4}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{4}));

// 				REQUIRE(e.generate(mapper({0,0}))->get_f() == (safe_int{4000} + safe_int{0}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{4})));
// 				REQUIRE(e.generate(mapper({1,0}))->get_f() == (safe_int{3000} + safe_int{1}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{4})));
// 				REQUIRE(e.generate(mapper({2,0}))->get_f() == (safe_int{2000} + safe_int{2}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{4})));
// 				REQUIRE(e.generate(mapper({3,0}))->get_f() == (safe_int{1000} + safe_int{3}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{4})));
// 				REQUIRE(e.generate(mapper({4,0}))->get_f() == (safe_int{0000} + safe_int{4}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{4})));
// 			}
// 		}

// 		WHEN("overflow before reaching the path") {
// 			ITERATE_OVER_EDGES(graphWithAlteringArcs, sourceId, i, arc) {
// 				info("arc weight is", arc.weight);
// 				info("number is", (INT_MAX/3));
// 				info("division will be", (arc.weight + (INT_MAX/3)));
// 				graphWithAlteringArcs.changeWeightOfDirectedArc(
// 						static_cast<int>(sourceId), arc.target,
// 						arc.weight + (INT_MAX/3));
// 			}

// 			REQUIRE(graphWithAlteringArcs.getWeightOfArc(mapper({0,0}), mapper({0,1})) == (1000 + INT_MAX/3));
// 			REQUIRE(graphWithAlteringArcs.getWeightOfArc(mapper({0,0}), mapper({1,1})) == (1414 + INT_MAX/3));

// 			//after 3 moves, g of a location reaches the upperlimit. Hence, from this point on, all the states have f=upperbound
// 			//so the first 3 moves are correctly handles by A*, after those 3 every state will have the same f, hence every path
// 			//has the same cost. It is a matter of how the open list is implemented that determine which state goes first
// 			std::vector<dpf::nodeid_t> path = aStar.get_path_as_vector(mapper({0,0}), mapper({4,0}), false);

// 			auto labelPrinter = DefaultLabelPrinter{};
// 			auto colorPrinter = DefaultColorPrinter{};
// 			printGridMap<DefaultLabelPrinter, DefaultColorPrinter>(mapData, width, height, mapper, xyLoc{4,0}, "overflowMap", labelPrinter, colorPrinter);

// 			REQUIRE(((path == std::vector<dpf::nodeid_t>{
// 				mapper({0,0}),
// 						mapper({1,0}),
// 						mapper({2,0}),
// 						mapper({3,0}),
// 						mapper({4,0})
// 			}) || (path == std::vector<dpf::nodeid_t>{
// 				mapper({0,0}),
// 						mapper({1,0}),
// 						mapper({2,0}),
// 						mapper({3,1}),
// 						mapper({4,0})
// 			})));

// 			THEN("") {

// 				critical("max is", INT_MAX);
// 				critical("/3 is", INT_MAX/3);

// 				critical("g 0 0 ", *(e.generate(mapper({0,0}))));
// 				critical("g 1 0 ", *(e.generate(mapper({1,0}))));
// 				critical("g 2 0 ", *(e.generate(mapper({2,0}))));
// 				critical("g 3 0 ", *(e.generate(mapper({3,0}))));
// 				critical("g 3 0 ", *(e.generate(mapper({3,1}))));
// 				critical("g 4 0 ", *(e.generate(mapper({4,0}))));
// 				REQUIRE(e.generate(mapper({0,0}))->get_g() == safe_int{0}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{3}));
// 				REQUIRE(e.generate(mapper({1,0}))->get_g() == safe_int{1}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{3}));
// 				REQUIRE(e.generate(mapper({2,0}))->get_g() == safe_int{2}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{3}));
// 				//all of them has the same value
// 				REQUIRE(e.generate(mapper({3,0}))->get_g() == safe_int{3}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{3}));
// 				REQUIRE(e.generate(mapper({3,1}))->get_g() == safe_int{1414} + safe_int{3}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{3}));
// 				REQUIRE(e.generate(mapper({4,0}))->get_g() == safe_int{4}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{3}));

// 				REQUIRE(e.generate(mapper({0,0}))->get_f() == (safe_int{4000} + safe_int{0}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{3})));
// 				REQUIRE(e.generate(mapper({1,0}))->get_f() == (safe_int{3000} + safe_int{1}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{3})));
// 				REQUIRE(e.generate(mapper({2,0}))->get_f() == (safe_int{2000} + safe_int{2}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{3})));
// 				//all of them has the same value
// 				REQUIRE(e.generate(mapper({3,0}))->get_f() == (safe_int{1000} + safe_int{3}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{3})));
// 				REQUIRE(e.generate(mapper({3,1}))->get_f() == (safe_int{1000} + safe_int{1414} + safe_int{3}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{3})));
// 				REQUIRE(e.generate(mapper({4,0}))->get_f() == (safe_int{0000} + safe_int{4}*(safe_int{1000} + safe_int{INT_MAX}/safe_int{3})));

// 				REQUIRE(e.generate(mapper({3,0}))->get_f() == e.generate(mapper({3,1}))->get_f());
// 				REQUIRE(e.generate(mapper({3,0}))->get_f() == e.generate(mapper({4,0}))->get_f());
// 			}
// 		}
// 	}

// }

// SCENARIO("testing landmarks", "[landmarks]") {

// 	const std::string arenaMapName{"arena.map"};
// 	const std::string arenaScenarioName{"arena.map.scen"};
// 	MapLoaderFactory factory{};

// 	AbstractMapLoader* loader = factory.get(arenaMapName.c_str());
// 	GridMap arenaGridMap = loader->LoadMap(arenaMapName.c_str());
// 	Mapper mapper{arenaGridMap};
// 	ListGraph listGraph = extract_graph(mapper);
// 	AdjGraph arenaGraph{listGraph};

// 	GIVEN("generating landmarks") {
// 		const std::string arenaLandmarkDBName{"arena.map.landmark.db"};

// //		WHEN("gerating one landmark") {
// //
// //			DifferentHeuristicAdvancePlacingLandmarkStrategy policy{1};
// //			LandMarkDatabase landmarkDB{arenaGraph, policy, arenaLandmarkDBName};
// //			auto h = DifferentialHeuristic{arenaGraph, mapper, landmarkDB};
// //			auto e = GridMapExpander{arenaGridMap, mapper, arenaGraph, true};
// //			auto stats = QuickStatistics{};
// //			auto aStar = SimplePathFindingAStar<DifferentialHeuristic, GridMapExpander, QuickStatistics>{&h, &e, &stats, true};
// //
// //			THEN("") {
// //
// //				xyLoc startLoc{10,10};
// //				xyLoc endLoc{11,10};
// //
// //				std::vector<dpf::nodeid_t> path = aStar.get_path_as_vector(mapper(startLoc), mapper(endLoc));
// //				REQUIRE((path == std::vector<dpf::nodeid_t>{mapper({10,10}), mapper({11,10})}));
// //
// //				PPMImage image{aStar.printExpandedNodesImage(mapper, arenaGridMap, mapper(startLoc), mapper(endLoc), nullptr)};
// //				image.saveAndConvertIntoPNG("landmark1_query1.png");
// //
// //			}
// //
// //			THEN("") {
// //
// //				xyLoc startLoc{10,10};
// //				xyLoc endLoc{21,10};
// //
// //				std::vector<dpf::nodeid_t> path = aStar.get_path_as_vector(mapper(startLoc), mapper(endLoc));
// //
// //				critical("landmarks are", mapper.convert(landmarkDB.getLandmarks()));
// //
// //				PPMImage image{aStar.printExpandedNodesImage(mapper, arenaGridMap, mapper(startLoc), mapper(endLoc), nullptr)};
// //
// //				for (auto landmark : landmarkDB.getLandmarks()) {
// //					xyLoc loc = mapper(landmark);
// //					image.setPixel(4*loc.x+1, 4*loc.y+1, PURPLE);
// //					image.setPixel(4*loc.x+2, 4*loc.y+1, PURPLE);
// //					image.setPixel(4*loc.x+3, 4*loc.y+1, PURPLE);
// //					image.setPixel(4*loc.x+1, 4*loc.y+2, PURPLE);
// //					image.setPixel(4*loc.x+2, 4*loc.y+2, PURPLE);
// //					image.setPixel(4*loc.x+3, 4*loc.y+2, PURPLE);
// //					image.setPixel(4*loc.x+1, 4*loc.y+3, PURPLE);
// //					image.setPixel(4*loc.x+2, 4*loc.y+3, PURPLE);
// //					image.setPixel(4*loc.x+3, 4*loc.y+3, PURPLE);
// //				}
// //
// //				image.saveAndConvertIntoPNG("landmark1_query2.png");
// //
// //				REQUIRE((path == std::vector<dpf::nodeid_t>{
// //					mapper({10,10}), mapper({11,10}), mapper({12,10}), mapper({13,10}), mapper({14,10}), mapper({15,10}),
// //					mapper({16,10}), mapper({17,10}), mapper({18,10}), mapper({19,10}), mapper({20,10}), mapper({21,10})}));
// //
// //
// //
// //			}
// //
// //			THEN("") {
// //
// //				xyLoc startLoc{10,10};
// //				xyLoc endLoc{25,20};
// //
// //				std::vector<dpf::nodeid_t> path = aStar.get_path_as_vector(mapper(startLoc), mapper(endLoc));
// //
// //				PPMImage image{aStar.printExpandedNodesImage(mapper, arenaGridMap, mapper(startLoc), mapper(endLoc), nullptr)};
// //				image.saveAndConvertIntoPNG("landmark1_query3.png");
// //
// //			}
// //
// //
// //			THEN("") {
// //
// //				xyLoc startLoc{10,10};
// //				xyLoc endLoc{45,36};
// //
// //				std::vector<dpf::nodeid_t> path = aStar.get_path_as_vector(mapper(startLoc), mapper(endLoc));
// //
// //				PPMImage image{aStar.printExpandedNodesImage(mapper, arenaGridMap, mapper(startLoc), mapper(endLoc), nullptr)};
// //				image.saveAndConvertIntoPNG("landmark1_query4.png");
// //
// //			}
// //		}
// //
// //		WHEN("gerating two landmark") {
// //
// //			DifferentHeuristicAdvancePlacingLandmarkStrategy policy{2};
// //			LandMarkDatabase landmarkDB{arenaGraph, policy, arenaLandmarkDBName};
// //			auto h = DifferentialHeuristic{arenaGraph, mapper, landmarkDB};
// //			auto e = GridMapExpander{arenaGridMap, mapper, arenaGraph, true};
// //			auto stats = QuickStatistics{};
// //			auto aStar = SimplePathFindingAStar<DifferentialHeuristic, GridMapExpander, QuickStatistics>{&h, &e, &stats, true};
// //
// //			THEN("") {
// //
// //				xyLoc startLoc{10,10};
// //				xyLoc endLoc{11,10};
// //
// //				std::vector<dpf::nodeid_t> path = aStar.get_path_as_vector(mapper(startLoc), mapper(endLoc));
// //				REQUIRE((path == std::vector<dpf::nodeid_t>{mapper({10,10}), mapper({11,10})}));
// //
// //				PPMImage image{aStar.printExpandedNodesImage(mapper, arenaGridMap, mapper(startLoc), mapper(endLoc), nullptr)};
// //				image.saveAndConvertIntoPNG("landmark2_query1.png");
// //
// //			}
// //
// //			THEN("") {
// //
// //				xyLoc startLoc{10,10};
// //				xyLoc endLoc{21,10};
// //
// //				std::vector<dpf::nodeid_t> path = aStar.get_path_as_vector(mapper(startLoc), mapper(endLoc));
// //
// //				critical("landmarks are", mapper.convert(landmarkDB.getLandmarks()));
// //
// //				PPMImage image{aStar.printExpandedNodesImage(mapper, arenaGridMap, mapper(startLoc), mapper(endLoc), nullptr)};
// //
// //				image.saveAndConvertIntoPNG("landmark2_query2.png");
// //
// //				REQUIRE((path == std::vector<dpf::nodeid_t>{
// //					mapper({10,10}), mapper({11,10}), mapper({12,10}), mapper({13,10}), mapper({14,10}), mapper({15,10}),
// //					mapper({16,10}), mapper({17,10}), mapper({18,10}), mapper({19,10}), mapper({20,10}), mapper({21,10})}));
// //
// //
// //
// //			}
// //
// //			THEN("") {
// //
// //				xyLoc startLoc{10,10};
// //				xyLoc endLoc{25,20};
// //
// //				std::vector<dpf::nodeid_t> path = aStar.get_path_as_vector(mapper(startLoc), mapper(endLoc));
// //
// //				PPMImage image{aStar.printExpandedNodesImage(mapper, arenaGridMap, mapper(startLoc), mapper(endLoc), nullptr)};
// //				image.saveAndConvertIntoPNG("landmark2_query3.png");
// //
// //			}
// //
// //
// //			THEN("") {
// //
// //				xyLoc startLoc{10,10};
// //				xyLoc endLoc{45,36};
// //
// //				std::vector<dpf::nodeid_t> path = aStar.get_path_as_vector(mapper(startLoc), mapper(endLoc));
// //
// //				PPMImage image{aStar.printExpandedNodesImage(mapper, arenaGridMap, mapper(startLoc), mapper(endLoc), nullptr)};
// //
// //				for (auto landmark : landmarkDB.getLandmarks()) {
// //					xyLoc loc = mapper(landmark);
// //					image.setPixel(4*loc.x+1, 4*loc.y+1, PURPLE);
// //					image.setPixel(4*loc.x+2, 4*loc.y+1, PURPLE);
// //					image.setPixel(4*loc.x+3, 4*loc.y+1, PURPLE);
// //					image.setPixel(4*loc.x+1, 4*loc.y+2, PURPLE);
// //					image.setPixel(4*loc.x+2, 4*loc.y+2, PURPLE);
// //					image.setPixel(4*loc.x+3, 4*loc.y+2, PURPLE);
// //					image.setPixel(4*loc.x+1, 4*loc.y+3, PURPLE);
// //					image.setPixel(4*loc.x+2, 4*loc.y+3, PURPLE);
// //					image.setPixel(4*loc.x+3, 4*loc.y+3, PURPLE);
// //				}
// //
// //				image.saveAndConvertIntoPNG("landmark2_query4.png");
// //
// //			}
// //		}

// 		WHEN("gerating five landmark") {

// 			DifferentHeuristicAdvancePlacingLandmarkStrategy policy{5};
// 			LandMarkDatabase landmarkDB{arenaGraph, policy, arenaLandmarkDBName};
// 			critical("landmark memory occupied: ", landmarkDB.getMemoryOccupied());
// 			REQUIRE(landmarkDB.getMemoryOccupied() > 0);
// 			auto h = DifferentialHeuristic{arenaGraph, mapper, landmarkDB};
// 			auto e = GridMapExpander{arenaGridMap, mapper, arenaGraph, true};
// 			auto stats = QuickStatistics{};
// 			SimplePathFindingAStar<DifferentialHeuristic, GridMapExpander, QuickStatistics> aStar{&h, &e, &stats, true};

// 			THEN("") {

// 				xyLoc startLoc{10,10};
// 				xyLoc endLoc{11,10};

// 				std::vector<dpf::nodeid_t> path = aStar.get_path_as_vector(mapper(startLoc), mapper(endLoc));
// 				REQUIRE((path == std::vector<dpf::nodeid_t>{mapper({10,10}), mapper({11,10})}));

// 				PPMImage image{aStar.printExpandedNodesImage(mapper, arenaGridMap, mapper(startLoc), mapper(endLoc), path, nullptr)};
// 				image.saveAndConvertIntoPNG("landmark5_query1.png");

// 			}

// 			THEN("") {

// 				xyLoc startLoc{10,10};
// 				xyLoc endLoc{21,10};

// 				std::vector<dpf::nodeid_t> path = aStar.get_path_as_vector(mapper(startLoc), mapper(endLoc));

// 				critical("landmarks are", mapper.convert(landmarkDB.getLandmarks()));

// 				PPMImage image{aStar.printExpandedNodesImage(mapper, arenaGridMap, mapper(startLoc), mapper(endLoc), path, nullptr)};

// 				image.saveAndConvertIntoPNG("landmark5_query2.png");

// 				REQUIRE((path == std::vector<dpf::nodeid_t>{
// 					mapper({10,10}), mapper({11,10}), mapper({12,10}), mapper({13,10}), mapper({14,10}), mapper({15,10}),
// 					mapper({16,10}), mapper({17,10}), mapper({18,10}), mapper({19,10}), mapper({20,10}), mapper({21,10})}));



// 			}

// 			THEN("") {

// 				xyLoc startLoc{10,10};
// 				xyLoc endLoc{25,20};

// 				std::vector<dpf::nodeid_t> path = aStar.get_path_as_vector(mapper(startLoc), mapper(endLoc));

// 				PPMImage image{aStar.printExpandedNodesImage(mapper, arenaGridMap, mapper(startLoc), mapper(endLoc), path, nullptr)};
// 				image.saveAndConvertIntoPNG("landmark5_query3.png");

// 			}


// 			THEN("") {

// 				xyLoc startLoc{10,10};
// 				xyLoc endLoc{45,36};

// 				std::vector<dpf::nodeid_t> path = aStar.get_path_as_vector(mapper(startLoc), mapper(endLoc));

// 				PPMImage image{aStar.printExpandedNodesImage(mapper, arenaGridMap, mapper(startLoc), mapper(endLoc), path, nullptr)};

// 				for (auto landmark : landmarkDB.getLandmarks()) {
// 					xyLoc loc = mapper(landmark);
// 					image.setPixel(4*loc.x+1, 4*loc.y+1, PURPLE);
// 					image.setPixel(4*loc.x+2, 4*loc.y+1, PURPLE);
// 					image.setPixel(4*loc.x+3, 4*loc.y+1, PURPLE);
// 					image.setPixel(4*loc.x+1, 4*loc.y+2, PURPLE);
// 					image.setPixel(4*loc.x+2, 4*loc.y+2, PURPLE);
// 					image.setPixel(4*loc.x+3, 4*loc.y+2, PURPLE);
// 					image.setPixel(4*loc.x+1, 4*loc.y+3, PURPLE);
// 					image.setPixel(4*loc.x+2, 4*loc.y+3, PURPLE);
// 					image.setPixel(4*loc.x+3, 4*loc.y+3, PURPLE);
// 				}

// 				image.saveAndConvertIntoPNG("landmark5_query4.png");

// 			}
// 		}

// 	}

// 	delete loader;

// }

// SCENARIO("testing bound technique", "[bound]") {

// 	GIVEN("square01 map") {
// 		int width, height;
// 		vector<bool> mapData;
// 		FILE* perturbatedMapFile;

// 		const char* mapName = "square01.map";
// 		const char* perturbatedMapName = "square01.perturbated.map";

// 		MapLoaderFactory mlf = MapLoaderFactory{};
// 		AbstractMapLoader* mapLoader = mlf.get(mapName);
// 		GridMap map = mapLoader->LoadMap(mapName);
// 		const string cpdFilename = getCPDFilename(mapName);
// 		auto h = CpdHeuristic{map, cpdFilename.c_str()};
// 		const Mapper& mapper = h.getMapper();
// 		AdjGraph graphWithAlteringArcs;
// 		auto stats = QuickStatistics{};
// 		//4*1000 +4*1000*x = 2*1000 + 2*1414
// 		//if you want to perform a diagonal slide it costs you 0.207 more that the optimal solution.
// 		// 
// 		// A -> 1000 -> B -> 1000 -> C
// 		// A -> 1414 -> D -> 1414 -> C

// 		WHEN("perturbating an edge which is not on the optimal path and the bound does not allow for diagonal pass") {
// 			double bound = 0.1;
// 			//create perturbation which does not affect the query
// 			graphWithAlteringArcs = AdjGraph{h.getGraph()};
// 			CpdCacheHeuristic h2 = CpdCacheHeuristic{h, graphWithAlteringArcs};
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{0,0}), mapper(xyLoc{1,0}), 3000);

// 			THEN("") {
// 				auto e = GridMapExpander{map, mapper, graphWithAlteringArcs, true};
// 				CPDBoundAStarSearch<CpdHeuristic, GridMapExpander, QuickStatistics> aStar{&h, &e, &stats, false, false, false, bound};
// 				//dynamic_pathfinding_astar<CpdCacheHeuristic, QuickStatistics> aStar{&h2, &e, &stats, false, true, true, true, bound, 1.0, false, 1.0};

// 				//perform search
// 				dpf::nodeid_t start = mapper(xyLoc{1,0});
// 				dpf::nodeid_t end = mapper(xyLoc{1,4});
// 				std::vector<xyLoc> path = mapper.convert(aStar.get_path_as_vector(start, end));

// 				PPMImage image{aStar.printExpandedNodesImage(mapper, map, start, end, mapper.convert(path), nullptr)};
// 				image.saveAndConvertIntoPNG("bound_not_invovled_01");

// 				REQUIRE(path.size() == 5);
// 				REQUIRE(path == std::vector<xyLoc>{{1,0}, {1,1}, {1,2}, {1,3}, {1,4}});
// 			}
// 		}

// 		WHEN("perturbating an edge which is not on the optimal path and the bound does  allow for diagonal pass") {
// 			double bound = 0.3;
// 			//create perturbation which does not affect the query
// 			graphWithAlteringArcs = AdjGraph{h.getGraph()};
// 			CpdCacheHeuristic h2 = CpdCacheHeuristic{h, graphWithAlteringArcs};
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{0,0}), mapper(xyLoc{1,0}), 1200);

// 			THEN("") {
// 				auto e = GridMapExpander{map, mapper, graphWithAlteringArcs, true};
// 				CPDBoundAStarSearch<CpdCacheHeuristic, GridMapExpander, QuickStatistics> aStar{&h2, &e, &stats, false, true, true, bound};

// 				//perform search
// 				dpf::nodeid_t start = mapper(xyLoc{1,0});
// 				dpf::nodeid_t end = mapper(xyLoc{1,4});
// 				std::vector<xyLoc> path = mapper.convert(aStar.get_path_as_vector(start, end));

// 				PPMImage image{aStar.printExpandedNodesImage(mapper, map, start, end, mapper.convert(path), nullptr)};
// 				image.saveAndConvertIntoPNG("bound_not_invovled_02");

// 				REQUIRE(path.size() == 5);
// 				REQUIRE(path == std::vector<xyLoc>{{1,0}, {1,1}, {1,2}, {1,3}, {1,4}});
// 			}
// 		}

// 		WHEN("perturbating an edge which is on the optimal path but the perturbation is within the bound") {
// 			double bound = 0.1;
// 			//create perturbation which does not affect the query
// 			graphWithAlteringArcs = AdjGraph{h.getGraph()};
// 			CpdCacheHeuristic h2 = CpdCacheHeuristic{h, graphWithAlteringArcs};
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{1,0}), mapper(xyLoc{1,1}), 1200);

// 			THEN("") {
// 				auto e = GridMapExpander{map, mapper, graphWithAlteringArcs, true};
// 				CPDBoundAStarSearch<CpdCacheHeuristic, GridMapExpander, QuickStatistics> aStar{&h2, &e, &stats, false, true, true, bound};

// 				//perform search
// 				dpf::nodeid_t start = mapper(xyLoc{1,0});
// 				dpf::nodeid_t end = mapper(xyLoc{1,4});
// 				std::vector<xyLoc> path = mapper.convert(aStar.get_path_as_vector(start, end));

// 				PPMImage image{aStar.printExpandedNodesImage(mapper, map, start, end, mapper.convert(path), nullptr)};
// 				image.saveAndConvertIntoPNG("bound_not_invovled_03");

// 				REQUIRE(path.size() == 5);
// 				REQUIRE(path == std::vector<xyLoc>{{1,0}, {1,1}, {1,2}, {1,3}, {1,4}});
// 			}
// 		}

// 		WHEN("perturbating an edge which is on the optimal path and the perturbation invalidate the path for the bound") {
// 			double bound = 0.3;
// 			//create perturbation which does not affect the query
// 			graphWithAlteringArcs = AdjGraph{h.getGraph()};
// 			CpdCacheHeuristic h2 = CpdCacheHeuristic{h, graphWithAlteringArcs};
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{1,0}), mapper(xyLoc{1,1}), 3000);

// 			THEN("") {
// 				auto e = GridMapExpander{map, mapper, graphWithAlteringArcs, true};
// 				CPDBoundAStarSearch<CpdCacheHeuristic, GridMapExpander, QuickStatistics> aStar{&h2, &e, &stats, false, true, true, bound};

// 				//perform search
// 				dpf::nodeid_t start = mapper(xyLoc{1,0});
// 				dpf::nodeid_t end = mapper(xyLoc{1,4});
// 				std::vector<xyLoc> path = mapper.convert(aStar.get_path_as_vector(start, end));

// 				PPMImage image{aStar.printExpandedNodesImage(mapper, map, start, end, mapper.convert(path), nullptr)};
// 				image.saveAndConvertIntoPNG("bound_not_invovled_04");
// 				critical("cost of generated path is ", graphWithAlteringArcs.getCostOfPath(path, mapper));

// 				REQUIRE(path.size() == 5);
// 				REQUIRE(path == std::vector<xyLoc>{{1,0}, {0,1}, {0,2}, {0,3}, {1,4}});
// 			}
// 		}

// 		WHEN("perturbating an edge which is in the middle of A* on the optimal path but the perturbation is within the bound") {
// 			double bound = 0.1;
// 			//create perturbation which does not affect the query
// 			graphWithAlteringArcs = AdjGraph{h.getGraph()};
// 			CpdCacheHeuristic h2 = CpdCacheHeuristic{h, graphWithAlteringArcs};
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{1,2}), mapper(xyLoc{1,3}), 1200);

// 			THEN("") {
// 				auto e = GridMapExpander{map, mapper, graphWithAlteringArcs, true};
// 				CPDBoundAStarSearch<CpdCacheHeuristic, GridMapExpander, QuickStatistics> aStar{&h2, &e, &stats, false, true, true, bound};

// 				//perform search
// 				dpf::nodeid_t start = mapper(xyLoc{1,0});
// 				dpf::nodeid_t end = mapper(xyLoc{1,4});
// 				std::vector<xyLoc> path = mapper.convert(aStar.get_path_as_vector(start, end));

// 				PPMImage image{aStar.printExpandedNodesImage(mapper, map, start, end, mapper.convert(path), nullptr)};
// 				image.saveAndConvertIntoPNG("bound_not_invovled_03");

// 				REQUIRE(path.size() == 5);
// 				REQUIRE(path == std::vector<xyLoc>{{1,0}, {1,1}, {1,2}, {1,3}, {1,4}});
// 			}
// 		}

// 		WHEN("perturbating an edge which is in the middle of A* on the optimal path and the perturbation invalidate the path for the bound") {
// 			double bound = 0.3;
// 			//create perturbation which does not affect the query
// 			graphWithAlteringArcs = AdjGraph{h.getGraph()};
// 			CpdCacheHeuristic h2 = CpdCacheHeuristic{h, graphWithAlteringArcs};
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{1,2}), mapper(xyLoc{1,3}), 3000);

// 			THEN("") {
// 				auto e = GridMapExpander{map, mapper, graphWithAlteringArcs, true};
// 				CPDBoundAStarSearch<CpdCacheHeuristic, GridMapExpander, QuickStatistics> aStar{&h2, &e, &stats, false, true, true, bound};

// 				//perform search
// 				dpf::nodeid_t start = mapper(xyLoc{1,0});
// 				dpf::nodeid_t end = mapper(xyLoc{1,4});
// 				std::vector<xyLoc> path = mapper.convert(aStar.get_path_as_vector(start, end));

// 				PPMImage image{aStar.printExpandedNodesImage(mapper, map, start, end, mapper.convert(path), nullptr)};
// 				image.saveAndConvertIntoPNG("bound_not_invovled_04");
// 				critical("cost of generated path is ", graphWithAlteringArcs.getCostOfPath(path, mapper));

// 				REQUIRE(path.size() == 5);
// 				REQUIRE(path == std::vector<xyLoc>{{1, 0}, {1,1}, {1,2}, {0,3}, {1,4}});
// 			}
// 		}


// 		delete mapLoader;
// 	}
// }

// SCENARIO("test anytime technique", "[anytime]") {
// 	GIVEN("square01 map") {
// 		int width, height;
// 		vector<bool> mapData;
// 		FILE* perturbatedMapFile;

// 		const char* mapName = "square01.map";
// 		const char* perturbatedMapName = "square01.perturbated.map";

// 		MapLoaderFactory mlf = MapLoaderFactory{};
// 		AbstractMapLoader* mapLoader = mlf.get(mapName);
// 		GridMap map = mapLoader->LoadMap(mapName);
// 		const string cpdFilename = getCPDFilename(mapName);
// 		auto h = CpdHeuristic{map, cpdFilename.c_str()};
// 		const Mapper& mapper = h.getMapper();
// 		AdjGraph graphWithAlteringArcs;
// 		auto stats = QuickStatistics{};
		

// 		WHEN("calling the anytime algorithm") {
// 			//create perturbation which does affect the query
// 			graphWithAlteringArcs = AdjGraph{h.getGraph()};
// 			CpdCacheHeuristic h2 = CpdCacheHeuristic{h, graphWithAlteringArcs};
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{0,0}), mapper(xyLoc{1,0}), 3000);
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{0,0}), mapper(xyLoc{1,1}), 4000);
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{0,0}), mapper(xyLoc{0,1}), 3000);
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{1,0}), mapper(xyLoc{1,1}), 3000);
			

// 			THEN("") {
// 				auto e = GridMapExpander{map, mapper, graphWithAlteringArcs, true};

// 				CPDAnytimeWAStarSearch<CpdCacheHeuristic, GridMapExpander, QuickStatistics> aStar{&h2, &e, &stats, false, true, true, 1.0};
// 				//TODO remove dynamic_pathfinding_astar<CpdCacheHeuristic, GridMapExpander, QuickStatistics> aStar{&h2, &e, &stats, false, true, true, false, 1.0, 1.0, true, 100.0};

// 				//perform search
// 				dpf::nodeid_t start = mapper(xyLoc{1,0});
// 				dpf::nodeid_t end = mapper(xyLoc{1,4});
// 				std::vector<xyLoc> path = mapper.convert(aStar.get_path_as_vector(start, end));

// 				PPMImage image{aStar.printExpandedNodesImage(mapper, map, start, end, mapper.convert(path), nullptr)};
// 				image.saveAndConvertIntoPNG("anytime_01");

// 				critical("anytime infos are ", aStar.getAnyTimeInfoVector());

// 				REQUIRE(path.size() == 5);
// 				REQUIRE(aStar.getAnyTimeInfoVector().size() == 2);
// 				REQUIRE(aStar.getAnyTimeInfoVector()[0].solutionCost > aStar.getAnyTimeInfoVector()[1].solutionCost);
// 				REQUIRE(aStar.getAnyTimeInfoVector()[0].microSeconds > 0);
// 				REQUIRE(aStar.getAnyTimeInfoVector()[1].microSeconds > 0);
// 			}
// 		}

// 		WHEN("calling the anytime algorithm") {
// 			//create perturbation which does affect the query
// 			graphWithAlteringArcs = AdjGraph{h.getGraph()};
// 			CpdCacheHeuristic h2 = CpdCacheHeuristic{h, graphWithAlteringArcs};
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{0,0}), mapper(xyLoc{1,0}), 3000);
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{0,0}), mapper(xyLoc{1,1}), 4000);
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{0,0}), mapper(xyLoc{0,1}), 3000);
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{0,1}), mapper(xyLoc{1,1}), 4000);
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{1,1}), mapper(xyLoc{1,2}), 3000);
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{1,1}), mapper(xyLoc{0,2}), 4000);

// 			THEN("") {
// 				auto e = GridMapExpander{map, mapper, graphWithAlteringArcs, true};
// 				CPDAnytimeWAStarSearch<CpdCacheHeuristic, GridMapExpander, QuickStatistics> aStar{&h2, &e, &stats, false, true, true, 1.0};
// 				//TODO remove dynamic_pathfinding_astar<CpdCacheHeuristic, QuickStatistics> aStar{&h2, &e, &stats, false, true, true, true, false, 1.0, true, 1.0};

// 				//perform search
// 				dpf::nodeid_t start = mapper(xyLoc{1,0});
// 				dpf::nodeid_t end = mapper(xyLoc{1,4});
// 				std::vector<xyLoc> path = mapper.convert(aStar.get_path_as_vector(start, end));

// 				PPMImage image{aStar.printExpandedNodesImage(mapper, map, start, end, mapper.convert(path), nullptr)};
// 				image.saveAndConvertIntoPNG("anytime_02");

// 				REQUIRE(path.size() == 5);
// 				REQUIRE(aStar.getAnyTimeInfoVector().size() == 2);
// 				REQUIRE(aStar.getAnyTimeInfoVector()[0].solutionCost > aStar.getAnyTimeInfoVector()[1].solutionCost);
// 			}
// 		}

// 		WHEN("calling the anytime algorithm with W=2") {
// 			//create perturbation which does affect the query
// 			graphWithAlteringArcs = AdjGraph{h.getGraph()};
// 			CpdCacheHeuristic h2 = CpdCacheHeuristic{h, graphWithAlteringArcs};
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{0,0}), mapper(xyLoc{1,0}), 3000);
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{0,0}), mapper(xyLoc{1,1}), 4000);
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{0,0}), mapper(xyLoc{0,1}), 3000);
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{1,0}), mapper(xyLoc{1,1}), 3000);
			

// 			THEN("") {
// 				auto e = GridMapExpander{map, mapper, graphWithAlteringArcs, true};

// 				CPDAnytimeWAStarSearch<CpdCacheHeuristic, GridMapExpander, QuickStatistics> aStar{&h2, &e, &stats, false, true, true, 2.0};
// 				//TODO remove dynamic_pathfinding_astar<CpdCacheHeuristic, GridMapExpander, QuickStatistics> aStar{&h2, &e, &stats, false, true, true, false, 1.0, 1.0, true, 100.0};

// 				//perform search
// 				dpf::nodeid_t start = mapper(xyLoc{1,0});
// 				dpf::nodeid_t end = mapper(xyLoc{1,4});
// 				std::vector<xyLoc> path = mapper.convert(aStar.get_path_as_vector(start, end));

// 				PPMImage image{aStar.printExpandedNodesImage(mapper, map, start, end, mapper.convert(path), nullptr)};
// 				image.saveAndConvertIntoPNG("anytime_01_WA");

// 				critical("anytime infos are ", aStar.getAnyTimeInfoVector());

// 				REQUIRE(path.size() == 5);
// 				REQUIRE(aStar.getAnyTimeInfoVector().size() == 2);
// 				REQUIRE(aStar.getAnyTimeInfoVector()[0].microSeconds > 0);
// 				REQUIRE(aStar.getAnyTimeInfoVector()[1].microSeconds > 0);
// 				REQUIRE(aStar.getAnyTimeInfoVector()[0].solutionCost > aStar.getAnyTimeInfoVector()[1].solutionCost);
// 			}
// 		}

// 		WHEN("calling the anytime algorithm with W=2") {
// 			//create perturbation which does affect the query
// 			graphWithAlteringArcs = AdjGraph{h.getGraph()};
// 			CpdCacheHeuristic h2 = CpdCacheHeuristic{h, graphWithAlteringArcs};
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{0,0}), mapper(xyLoc{1,0}), 3000);
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{0,0}), mapper(xyLoc{1,1}), 4000);
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{0,0}), mapper(xyLoc{0,1}), 3000);
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{0,1}), mapper(xyLoc{1,1}), 4000);
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{1,1}), mapper(xyLoc{1,2}), 3000);
// 			graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{1,1}), mapper(xyLoc{0,2}), 4000);

// 			THEN("") {
// 				auto e = GridMapExpander{map, mapper, graphWithAlteringArcs, true};
// 				CPDAnytimeWAStarSearch<CpdCacheHeuristic, GridMapExpander, QuickStatistics> aStar{&h2, &e, &stats, false, true, true, 2.0};
// 				//TODO remove dynamic_pathfinding_astar<CpdCacheHeuristic, QuickStatistics> aStar{&h2, &e, &stats, false, true, true, true, false, 1.0, true, 1.0};

// 				//perform search
// 				dpf::nodeid_t start = mapper(xyLoc{1,0});
// 				dpf::nodeid_t end = mapper(xyLoc{1,4});
// 				std::vector<xyLoc> path = mapper.convert(aStar.get_path_as_vector(start, end));

// 				PPMImage image{aStar.printExpandedNodesImage(mapper, map, start, end, mapper.convert(path), nullptr)};
// 				image.saveAndConvertIntoPNG("anytime_02_WA");

// 				critical("anytime infos are ", aStar.getAnyTimeInfoVector());

// 				REQUIRE(path.size() == 5);
// 				REQUIRE(path == std::vector<xyLoc>{xyLoc{1,0}, xyLoc{0,1}, xyLoc{0,2}, xyLoc{0,3}, xyLoc{1,4},});
// 				REQUIRE(aStar.getAnyTimeInfoVector().size() == 2);
// 				REQUIRE(aStar.getAnyTimeInfoVector()[0].solutionCost > aStar.getAnyTimeInfoVector()[1].solutionCost);
// 			}
// 		}
// 	}

// 	GIVEN("square01 map") {
// 		int width, height;
// 		vector<bool> mapData;
// 		FILE* perturbatedMapFile;

// 		const char* mapName = "square01.map";
// 		const char* perturbatedMapName = "square01.perturbated.map";

// 		MapLoaderFactory mlf = MapLoaderFactory{};
// 		AbstractMapLoader* mapLoader = mlf.get(mapName);
// 		GridMap map = mapLoader->LoadMap(mapName);
// 		const string cpdFilename = getCPDFilename(mapName);
// 		auto h = CpdHeuristic{map, cpdFilename.c_str()};
// 		const Mapper& mapper = h.getMapper();
// 		AdjGraph graphWithAlteringArcs;
// 		auto stats = QuickStatistics{};

// 		graphWithAlteringArcs = AdjGraph{h.getGraph()};
// 		graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{0,0}), mapper(xyLoc{1,0}), 3000);
// 		graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{0,0}), mapper(xyLoc{1,1}), 4000);
// 		graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{0,0}), mapper(xyLoc{0,1}), 3000);
// 		graphWithAlteringArcs.changeWeightOfArc(mapper(xyLoc{1,0}), mapper(xyLoc{1,1}), 3000);

// 		WHEN("calling the WA* anytime algorithm with CPD") {
// 			//create perturbation which does not affect the query
// 			CpdCacheHeuristic h2 = CpdCacheHeuristic{h, graphWithAlteringArcs};
			
// 			THEN("") {
// 				auto e = GridMapExpander{map, mapper, graphWithAlteringArcs, true};
// 				AnytimeWeightPathFindingAStar<CpdCacheHeuristic, GridMapExpander, QuickStatistics> waStar{&h2, &e, &stats, false, 1.1};

// 				//perform search
// 				dpf::nodeid_t start = mapper(xyLoc{1,0});
// 				dpf::nodeid_t end = mapper(xyLoc{1,4});
// 				std::vector<xyLoc> path = mapper.convert(waStar.get_path_as_vector(start, end));

// 				PPMImage image{waStar.printExpandedNodesImage(mapper, map, start, end, mapper.convert(path), nullptr)};
// 				image.saveAndConvertIntoPNG("anytime_wa_01");

// 				critical("anytime infos are ", waStar.getAnyTimeInfoVector());

// 				REQUIRE(path.size() == 5);
// 				REQUIRE(waStar.getAnyTimeInfoVector().size() >= 1);
// 				REQUIRE(waStar.getAnyTimeInfoVector()[0].microSeconds > 0);
// 			}
// 		}

// 		WHEN("calling the WA* anytime algorithm with LANDMARK") {
// 			//create perturbation which does not affect the query
// 			const std::string squareLandmarkDBName{"square01.map.landmark.db"};
// 			DifferentHeuristicAdvancePlacingLandmarkStrategy policy{2};

// 			AdjGraph squareGraph{map.getAdjGraph()};
// 			LandMarkDatabase landmarkDB{squareGraph, policy, squareLandmarkDBName};
// 			auto h2 = DifferentialHeuristic{squareGraph, mapper, landmarkDB};
			
// 			THEN("") {
// 				auto e = GridMapExpander{map, mapper, graphWithAlteringArcs, true};
// 				AnytimeWeightPathFindingAStar<DifferentialHeuristic, GridMapExpander, QuickStatistics> waStar{&h2, &e, &stats, false, 1.1};

// 				//perform search
// 				dpf::nodeid_t start = mapper(xyLoc{1,0});
// 				dpf::nodeid_t end = mapper(xyLoc{1,4});
// 				std::vector<xyLoc> path = mapper.convert(waStar.get_path_as_vector(start, end));

// 				PPMImage image{waStar.printExpandedNodesImage(mapper, map, start, end, mapper.convert(path), nullptr)};
// 				image.saveAndConvertIntoPNG("anytime_wa_01");

// 				critical("anytime infos are ", waStar.getAnyTimeInfoVector());

// 				REQUIRE(path.size() == 5);
// 				REQUIRE(waStar.getAnyTimeInfoVector().size() >= 1);
// 				REQUIRE(waStar.getAnyTimeInfoVector()[0].microSeconds > 0);
// 			}
// 		}

		
// 	}
// }

// SCENARIO("additional wrong scenarios happended during performance testing", "[newfeatures]") {
// 	MapLoader mapLoader;
// 	int width, height;
// 	vector<bool> mapData;
// 	FILE* perturbatedMapFile;

// 	GIVEN("hrt201n map") {

// 		//		const string mapName = "square03.map";
// 		//		mapLoader.LoadMap(mapName.c_str(), mapData, width, height);
// 		//		Mapper mapperTmp = Mapper{mapData, width, height};
// 		//		ListGraph listGraph1 = extract_graph(mapperTmp);
// 		//
// 		//		const string cpdFilename = getCPDFilename(mapName);
// 		//		auto h = CpdHeuristic{mapData, width, height, cpdFilename.c_str()};
// 		//		perturbatedMapFile = fopen("hrt201n_001", "rb");
// 		//		AdjGraph graphWithAlteringArcs = AdjGraph::load(perturbatedMapFile);
// 		//		const Mapper& mapper = h.getMapper();
// 		//		auto e = GridMapExpander{mapData, width, height, h.getMapper(), graphWithAlteringArcs};
// 		//
// 		//		AdjGraph newMap = AdjGraph{h.getGraph()};
// 		//		CpdCacheHeuristic h2 = CpdCacheHeuristic{h, newMap};
// 		//		dynamic_pathfinding_astar aStar = dynamic_pathfinding_astar{&h2, &e, true, true};
// 		//
// 		//
// 		//		WHEN("testing query #2") {
// 		//			118	155	120	154	2.41421356
// 		//			xyLoc start{118, 155};
// 		//			xyLoc goal{120, 154};
// 		//			std::vector<xyLoc> path = aStar.get_path_as_vector(mapper(start), mapper(goal));
// 		//		}

// 	}
// }
