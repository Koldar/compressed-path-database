/*
 * test.cpp
 *
 *  Created on: Oct 1, 2018
 *      Author: koldar
 */

#define CATCH_CONFIG_NO_POSIX_SIGNALS // https://github.com/catchorg/Catch2/issues/1295 avoid catch catching signals
#include "catch.hpp"

#include "CpdManager.hpp"

#include <string>
#include <sstream>

#include <pathfinding-utils/xyLoc.hpp>
#include <pathfinding-utils/GridMap.hpp>
#include <pathfinding-utils/GridMapGraphConverter.hpp>
#include <pathfinding-utils/GridBranching.hpp>
#include <pathfinding-utils/MovingAIGridMapReader.hpp>

using namespace compressed_path_database;
using namespace pathfinding;
using namespace cpp_utils;


SCENARIO("test cpd") {

	GIVEN("square03 map") {

		maps::MovingAIGridMapReader reader{
			'.', cost_t{100}, color_t::WHITE,
			'T', cost_t::INFTY, color_t::BLACK,
			'@', cost_t::INFTY, color_t::BLACK
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
		critical("fetching the map...");
		graphs::AdjacentGraph<std::string, xyLoc, cost_t> graph{*converter.toGraph(map)};
		critical("map fetched");
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
		cost_t moveCost;

		WHEN("target is equal to start") {
			xyLoc start{0,0};
			xyLoc target{0,0};

			nodeid_t startId = actualGraph.idOfVertex(start);
			nodeid_t targetId = actualGraph.idOfVertex(target);
			REQUIRE(manager.getFirstMove(startId, targetId, move, nextNode, moveCost) == false);

		}

		WHEN("target is under start") {
			xyLoc start{0,0};
			xyLoc target{1,0};

			nodeid_t startId = actualGraph.idOfVertex(start);
			nodeid_t targetId = actualGraph.idOfVertex(target);
			REQUIRE(manager.getFirstMove(startId, targetId, move, nextNode, moveCost) == true);

			REQUIRE(actualGraph.getVertex(nextNode) == target);
			REQUIRE(moveCost == 100);
		}

		WHEN("target is much under start") {
			xyLoc start{0,0};
			xyLoc target{2,0};

			nodeid_t startId = actualGraph.idOfVertex(start);
			nodeid_t targetId = actualGraph.idOfVertex(target);
			REQUIRE(manager.getFirstMove(startId, targetId, move, nextNode, moveCost) == true);

			REQUIRE(actualGraph.getVertex(nextNode) == xyLoc{1,0});
			REQUIRE(moveCost == 100);
		}

		WHEN("target is far from start") {
			xyLoc start{0,0};
			xyLoc target{4,4};

			nodeid_t startId = actualGraph.idOfVertex(start);
			nodeid_t targetId = actualGraph.idOfVertex(target);
			REQUIRE(manager.getFirstMove(startId, targetId, move, nextNode, moveCost) == true);

			REQUIRE(actualGraph.getVertex(nextNode) == xyLoc{0,1});
			REQUIRE(moveCost == 100);
		}

		WHEN("target is unreachable") {
			xyLoc start{4,2};
			xyLoc target{1,0};

			nodeid_t startId = actualGraph.idOfVertex(start);
			nodeid_t targetId = actualGraph.idOfVertex(target);
			REQUIRE(manager.getFirstMove(startId, targetId, move, nextNode, moveCost) == false);
		}

		WHEN("example of path") {
			xyLoc target{4,4};
			nodeid_t targetId = actualGraph.idOfVertex(target);

			REQUIRE(manager.getFirstMove(actualGraph.idOfVertex(xyLoc{0,0}), targetId, move, nextNode, moveCost) == true);
			REQUIRE(actualGraph.getVertex(nextNode) == xyLoc{0,1});
			REQUIRE(moveCost == 100);

			REQUIRE(manager.getFirstMove(actualGraph.idOfVertex(xyLoc{0,1}), targetId, move, nextNode, moveCost) == true);
			REQUIRE(actualGraph.getVertex(nextNode) == xyLoc{0,2});
			REQUIRE(moveCost == 100);
			
			REQUIRE(manager.getFirstMove(actualGraph.idOfVertex(xyLoc{0,2}), targetId, move, nextNode, moveCost) == true);
			REQUIRE(actualGraph.getVertex(nextNode) == xyLoc{1,3});
			REQUIRE(moveCost == 141);

			REQUIRE(manager.getFirstMove(actualGraph.idOfVertex(xyLoc{1,3}), targetId, move, nextNode, moveCost) == true);
			REQUIRE(actualGraph.getVertex(nextNode) == xyLoc{2,4});
			REQUIRE(moveCost == 141);

			REQUIRE(manager.getFirstMove(actualGraph.idOfVertex(xyLoc{2,4}), targetId, move, nextNode, moveCost) == true);
			REQUIRE(actualGraph.getVertex(nextNode) == xyLoc{3,4});
			REQUIRE(moveCost == 100);

			REQUIRE(manager.getFirstMove(actualGraph.idOfVertex(xyLoc{3,4}), targetId, move, nextNode, moveCost) == true);
			REQUIRE(actualGraph.getVertex(nextNode) == xyLoc{4,4});
			REQUIRE(moveCost == 100);
		}

		WHEN("testing whole path when target is source") {
			xyLoc start{0,0};
			xyLoc target{0,0};
			nodeid_t startId = actualGraph.idOfVertex(start);
			nodeid_t targetId = actualGraph.idOfVertex(target);

			REQUIRE(manager.generateOptimalPathOfNodes(startId, targetId) == std::vector<nodeid_t>{0});
			REQUIRE(manager.generateOptimalPathCost(startId, targetId) == 0);
		}

		WHEN("testing whole path when target is below from source") {
			xyLoc start{0,0};
			xyLoc target{0,1};
			nodeid_t startId = actualGraph.idOfVertex(start);
			nodeid_t targetId = actualGraph.idOfVertex(target);

			REQUIRE(manager.generateOptimalPathOfNodes(startId, targetId) == std::vector<nodeid_t>{0, 1});
			REQUIRE(manager.generateOptimalPathCost(startId, targetId) == 100);
		}

		WHEN("testing whole path when target is unreachable") {
			xyLoc start{0,0};
			xyLoc target{4,2};
			nodeid_t startId = actualGraph.idOfVertex(start);
			nodeid_t targetId = actualGraph.idOfVertex(target);

			REQUIRE(manager.generateOptimalPathOfNodes(startId, targetId) == std::vector<nodeid_t>{});
			REQUIRE(manager.generateOptimalPathCost(startId, targetId) == cost_t::INFTY);
		}

		WHEN("testing whole path when target is far from source") {
			xyLoc start{0,0};
			xyLoc target{4,4};
			nodeid_t startId = actualGraph.idOfVertex(start);
			nodeid_t targetId = actualGraph.idOfVertex(target);

			actualGraph.saveBMP("cpdgraph");
			REQUIRE(manager.generateOptimalPathOfNodes(startId, targetId) == std::vector<nodeid_t>{0, 1, 2, 6, 15, 16, 17});
			REQUIRE(manager.generateOptimalPathCost(startId, targetId) == (2* 141 + 4*100));
		}

	}
}