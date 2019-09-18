#ifndef ENTRY_H
#define ENTRY_H

#include <string>
#include "mapper.hpp"
//#include "adj_graph.h"
#include <warthog/constants.h>
#include "types.hpp"
#include <cpp-utils/listGraph.hpp>
#include <pathfinding-utils/types.hpp>
#include <boost/filesystem.hpp>
#include "list_graph.hpp"
#include "order.hpp"
#include <cpp-utils/adjacentGraph.hpp>
#include "cpd.hpp"
#include "dijkstra.hpp"
#include <cpp-utils/Timer.hpp>
#include <boost/filesystem.hpp>
#include <cpp-utils/exceptions.hpp>
#include <cpp-utils/imemory.hpp>

namespace cpd {

using namespace pathfinding;

#define DIJKSTRA_PRINTF_EVERY_VERTEX 1

/**
 * @brief class allowing to interface with a cpd
 * 
 */
template <typename G, typename V>
class CpdManager: public cpp_utils::IMemorable {
private:
    /**
     * A state used when searchinf for the optimal path in a static CPD
     */
    struct CpdContext {
        CPD cpd;
        //TOD remove Mapper mapper;
        cpp_utils::graphs::AdjacentGraph<G,V, cost_t> graph;
        // TODO REMOVE dpf::nodeid_t current_node;
        // dpf::nodeid_t target_node;
    };
public:
    friend std::ostream& operator <<(std::ostream& out, const CpdManager<G, V>& m) {
        out << "{cpd path=" << m.cpdPath << ", context=" << ((m.context != nullptr) ? "YES": "NO") << "}";
        return out;
    }
private:
    /**
     * @brief represents the path where the cpd dealt with this class will be put/read
     * 
     */
    boost::filesystem::path cpdPath;
    CpdContext* context;
public:
    /**
     * @brief construct a new cpd manager
     * 
     * This manager will immediately try to load the CPD from the given file.
     * If the file do not exist, it will create the CPD itself (so the developer do not need to call saveCpd).
     * 
     * @note
     * Note that, while this construct does everythign by itself, you won't be in control on when the cpd
     * is actually built. Since cpd construction is CPU-intensive, you need to be careful to use this constructor in performance watched function since maybe the constructor decides to build the CPD there!
     * 
     * @param cpdPath the path where the cpd will be saved
     * @param g the graph used to buld/load the cpd
     */
    CpdManager(const boost::filesystem::path& cpdPath, const IImmutableGraph<G, V, cost_t>& g): cpdPath{cpdPath}, context{nullptr} {
        this->saveCpd(g);
        this->loadCpd(g);
    }
    /**
     * @brief creates a new manager
     * 
     * The develop is required to manually call (after this construct) both:
     * @li saveCpd to create the cpd;
     * @li loadCpd to fetch the reordered graph
     * 
     * @param cpdPath the path where the cpd will be positioned in the file system
     */
    CpdManager(const boost::filesystem::path& cpdPath): cpdPath{cpdPath}, context{nullptr} {

    }
    //I don't want that the manager is copied
    CpdManager(const CpdManager& manager) = delete;
    //I don't want that the manager is copied
    CpdManager& operator =(const CpdManager& manager) = delete;
    ~CpdManager() {
        if (context != nullptr) {
            delete context;
        }
    }
    /**
     * @brief True if loadCpd has already been called
     * 
     * @return true 
     * @return false 
     */
    bool isCpdLoaded() const {
        return this->context != nullptr;
    }

    /**
     * @brief Get the Reordered Graph object
     * 
     * @pre
     *  @li loadCpD needs to have been called at least once (or in alternative you had to call the constructor with 2 arguments)
     * 
     * @return const IImmutableGraph<G, V, cost_t>& 
     */
    const IImmutableGraph<G, V, cost_t>& getReorderedGraph() const {
        return this->context->graph;
    }

    /**
     * @brief 
     * 
     * @param g 
     */
    void saveCpd(const cpp_utils::graphs::IImmutableGraph<G,V,cost_t>& g) const {
        this->saveCpd(cpp_utils::graphs::ListGraph<G,V,cost_t>{g});
    }
    /**
     * @brief generate the cpd of the given map and saves it in the given path
     * 
     * @post
     *  @li file @c cpdPath will be generated in the system
     * 
     * @note
     * this operation may take a lot of time
     * 
     * @param g the graph to compress in the cpd
     */
    void saveCpd(const cpp_utils::graphs::ListGraph<G,V,cost_t>& g) const {
        //check if ther file exists
        if (boost::filesystem::exists(this->cpdPath)) {
            return;
        }

        debug("Computing node order");

        //use CPD datastructure to compute the order... I don't want to deal with it for now
        cpd::datastructures::ListGraph cpdListGraph{cpd::datastructures::fromCppUtilsListGraphToCpdListGraph(g)};
#ifndef USE_CUT_ORDER
        //NodeOrdering order = compute_real_dfs_order(getListGraphFrom(map, mapper, MULTI_TERRAIN_STRATEGY));
        NodeOrdering order = compute_real_dfs_order(cpdListGraph);
#else
        NodeOrdering order = compute_cut_order(
                cpdListGraph,
                prefer_zero_cut(balanced_min_cut)
        );
#endif

        //convert to our datastructures
        //info("the order is ", order);
        cpp_utils::graphs::AdjacentGraph<G, V, cost_t> reorderedGraph{*(g.reorderVertices(order.getToNewArray<nodeid_t>(), order.getToOldArray<nodeid_t>()))};
        //mapper.reorder(order);


        debug("Computing first-move matrix");
        CPD cpd;
        {
            //AdjGraph g(getListGraphFrom(map, mapper, MULTI_TERRAIN_STRATEGY));
            {
                info("calling Dijkstra one time to generate an estimate of time...");
                Dijkstra<G, V> dij{reorderedGraph};
                cpp_utils::Timer t{};
                t.start();
                dij.run(0);
                t.stop();
                info("Estimated sequential running time:", (t.getElapsedMicroSeconds() * reorderedGraph.numberOfVertices()).toMinutes());
            }

#ifndef USE_PARALLELISM
            info("we're not going to use parallelism!");
            Dijkstra<G, V> dij{reorderedGraph};
            for(nodeid_t source_node=0; source_node < reorderedGraph.numberOfVertices(); ++source_node){
                //progress bar
                if((source_node % DIJKSTRA_PRINTF_EVERY_VERTEX) == 0) {
                    info(((source_node*100.0)/reorderedGraph.numberOfVertices()), "done");
                }

                debug("calling dijktstra on node", source_node, "!");
                const auto& allowed = dij.run(source_node);
                debug("appending on row");
                cpd.append_row(source_node, allowed);
            }
#else
            info("Using", omp_get_max_threads(), "threads");
            vector<CPD> thread_cpd(omp_get_max_threads());

            int progress = 0;

#pragma omp parallel
            {
                const int thread_count = omp_get_num_threads();
                const int thread_id = omp_get_thread_num();
                const int node_count = reorderedGraph.numberOfVertices();

                nodeid_t node_begin = (node_count*thread_id) / thread_count;
                nodeid_t node_end = (node_count*(thread_id+1)) / thread_count;

                //TODO remove AdjGraph thread_adj_g(g);
                AdjacentGraph<G,V,cost_t> threadAdjacentGraph{reorderedGraph};
                Dijkstra thread_dij{threadAdjacentGraph};

                for(nodeid_t source_node=node_begin; source_node < node_end; ++source_node){
                    thread_cpd[thread_id].append_row(source_node, thread_dij.run(source_node));
#pragma omp critical
                    {
                        ++progress;
                        if((progress % DIJKSTRA_PRINTF_EVERY_VERTEX) == 0){
                            info(((progress*100.0)/reorderedGraph.numberOfVertices()), "done");
                            fflush(stdout);
                        }
                    }
                }
            }

            for(auto& x : thread_cpd) {
                cpd.append_rows(x);
            }
#endif
        }

        info("Saving data to", cpdPath);
        FILE* f = fopen(cpdPath.native().c_str(), "wb");
        order.save(f);
        cpd.save(f);
        fclose(f);
        info("done");
    }

    /**
     * @brief load a cpd from the given file
     * 
     * @note
     * The function will concretely load the cpd if it has been called for the first time, otherwise
     * it will just yield the reordered graph
     * 
     * @param g the graph we want to load
     * @return IImmutableGraph<G, V, cost_t> a vertex permutation of @c g. all the other data remain unchanged
     */
    const IImmutableGraph<G, V, cost_t>& loadCpd(const IImmutableGraph<G, V, cost_t>& g) {
        if (this->context != nullptr) {
            return this->context->graph;
        }
        this->context = new CpdContext;

        //TODO remove state->mapper = Mapper(bits, w, h);

        info("Loading preprocessing data from", this->cpdPath.native());
        FILE* f = fopen(this->cpdPath.native().c_str(), "rb");
        if (f == nullptr) {
            throw cpp_utils::exceptions::FileOpeningException{this->cpdPath.native()};
        }
        NodeOrdering order;
        order.load(f);
        this->context->cpd.load(f);
        fclose(f);

        this->context->graph = cpp_utils::graphs::AdjacentGraph<G, V, cost_t>{*(g.reorderVertices(order.getToNewArray<nodeid_t>(), order.getToOldArray<nodeid_t>()))};
        //TODO remove state->mapper.reorder(order);

        //TODO remove state->graph = AdjGraph(extract_graph(state->mapper));
        //TODO remove state->current_node = -1;
        info("Loading done");
        return this->context->graph;
    }
    /**
     * @brief Get the First Move object
     * 
     * @param current the node in the graph you're in at the moment
     * @param target the target you're aiming to
     * @param firstMove the move the cpd tells you to perform. Set only if the return value is true
     * @param nextNode the node you arriving to by applying the move from the cpd. 
     * @return bool true if the cpd generated a move
     */
    bool getFirstMove(nodeid_t current, nodeid_t target, moveid_t& firstMove, nodeid_t& nextNode, cost_t& moveCost) const {
        if (this->context == nullptr) {
            error("cpd not loaded yet!");
            throw cpp_utils::exceptions::InvalidStateException<CpdManager>{*this};
        }
        unsigned char move = this->context->cpd.get_first_move(current, target);
        info("from ", current, "to ", target, " the cpd says the best move is", static_cast<unsigned int>(move));
        if(move == 0xF) {
            info("returning false");
            return false;
        } else if(current == target) {
            return false;
        } else {
            firstMove = move;
            const OutEdge<cost_t> outEdge = this->context->graph.getOutEdge(current, move); 
            nextNode = outEdge.getSinkId();
            moveCost = outEdge.getPayload();
            return true;
        }
    }
    std::vector<nodeid_t> generateOptimalPathOfNodes(nodeid_t current, nodeid_t target) const {
        std::vector<nodeid_t> result{};
        this->generateOptimalPath(current, target, &result, nullptr, nullptr);
        return result;
    }
    std::vector<moveid_t> generateOptimalPathOfMoves(nodeid_t current, nodeid_t target) const {
        std::vector<moveid_t> result{};
        this->generateOptimalPath(current, target, nullptr, &result, nullptr);
        return result;
    }
    cost_t generateOptimalPathCost(nodeid_t current, nodeid_t target) const {
        cost_t result;
        this->generateOptimalPath(current, target, nullptr, nullptr, &result);
        return result;
    }
    void generateOptimalPath(nodeid_t current, nodeid_t target, std::vector<nodeid_t>* nodes, std::vector<moveid_t>* moves, cost_t* cost) const {
        if (this->context == nullptr) {
            throw cpp_utils::exceptions::InvalidStateException<CpdManager>{*this};
        }

        if (current == target) {
            //we have already reached the target
            if (nodes != nullptr) {
                nodes->push_back(current);
            }
            if (cost != nullptr) {
                *cost = 0;
            }
            return;
        }

	    moveid_t firstMove = this->context->cpd.get_first_move(current, target);
        if(firstMove == 0xF) {
            //no move available
            if (cost != nullptr) {
                *cost = cost_t::INFTY;
            }    
            return;
        } 
        // current != target
        if (cost != nullptr) {
            *cost = 0;
        }
        for(;;) {
            if (nodes != nullptr) {
                nodes->push_back(current);
            }
            if (moves != nullptr) {
                moves->push_back(firstMove);
            }
            OutEdge<cost_t> outEdge = this->context->graph.getOutEdge(current, firstMove);
            if (cost != nullptr) {
                *cost += outEdge.getPayload();
            }
            current = outEdge.getSinkId();

            if(current == target) {
                //add target
                if (nodes != nullptr) {
                    nodes->push_back(current);
                }
                if (moves != nullptr) {
                    moves->push_back(firstMove);
                }
                return;
            }
            firstMove = this->context->cpd.get_first_move(current, target);

        }
    }
public:
    virtual cpp_utils::MemoryConsumption getByteMemoryOccupied() const {
        cpp_utils::MemoryConsumption result;
        result += sizeof(*this);
        result += this->cpdPath.size();
        if (this->context != nullptr) {
            result += sizeof(*this->context);
            result += this->context->cpd.getByteMemoryOccupied();
            result += this->context->graph.getByteMemoryOccupied();
        }
        return result;
    }

};


// /**
//  * preprocess a map
//  *
//  * @opst
//  *  @li a file named @c filename will appear in the CWD
//  *
//  * @param[in] bits the map you need to preprocess
//  * @param[in] width the width of the map
//  * @param[in] height the height of the map
//  * @param[in] filename the name of the file that will represent the CPD serialized version
//  */
// void PreprocessMap(std::vector<bool> &bits, int width, int height, const char *filename);
// //TODO remove void PreprocessMap(const GridMap& map, const char *filename);
// void *PrepareForSearch(std::vector<bool> &bits, int width, int height, const char *filename);
// //TODO remove void *PrepareForSearch(const GridMap& map, const char *filename);

// /**
//  * A strategy allowing you to encdode GridMap in ListGraph by adjusting costs of terrains
//  * depending on the neighbours
//  */
// //TODO remove extern std::function<int(const GridMap&, const Mapper&, xyLoc, xyLoc)> MULTI_TERRAIN_STRATEGY;

// //TODO removeconst Mapper& GetMapper(const void* helper);

// //TODO removeconst AdjGraph& GetGraph(const void* helper);

// /**
//  * Remove from memory the pointer previosuly created with ::PrepareForSearch
//  *
//  * @param[in] helper the helper to remove from memory
//  */
// void DeleteHelperOfSearch(const void* helper);

// /**
//  * get the first optimal move to perform if you want to reach @c t starting from @c s
//  *
//  * @attention
//  * The behaviour of this function is heavily influenced by the macro ::EXTRACT_ALL_AT_ONCE,
//  * See @ref buildingMacros for further information
//  *
//  *
//  * @param[in] data a pointer of type Start
//  * @param[in] s the starting location
//  * @param[in] t the goal we need to reach
//  * @param[inout] an initialized vector that, after this call, will contain the first move.
//  * 	The first move will be appended at the tail of the vector. The path is just a sequence of locations.
//  * 	At index `i` is present where are we in the timestamp `i`
//  * @return
//  *  @li true if we have reached the goal;
//  *  @li true if there is no path from @c s to @c t
//  *  @li false if we have to perform other step other than the first one in order to reach @c t
//  */
// //TODO remove bool GetPath(void *data, xyLoc s, xyLoc g, std::vector<xyLoc> &path);

// /**
//  * Generate an optimal path using the CPD.
//  *
//  * @note
//  * this function is a wrapper to the function GetPath which is compilation sensitive
//  *
//  * @param[inout] cpdHelper the helper created with ::PrepareForSearch
//  * @param[in] start the initial position of the agent
//  * @param[in] goal the final destination of the agent
//  * @param[inout] path the built path. At index `i` the vector contains the cell where we need to go in order to reach the location @c goal
//  * @return
//  *  @li true if we ere able to find the optional path (thus @c path is altered);
//  *  @li false if no path could be obtained (thus @c path is unchanged);
//  */
// //TODO remove bool GetPathWithCPD(void *cpdHelper, xyLoc start, xyLoc goal, std::vector<xyLoc> &path);

// /**
//  * like ::GetPathWithCPD but generates the cost of the path in the CPD as well
//  *
//  * @param[inout] cpdHelper the helper created with ::PrepareForSearch
//  * @param[in] start the initial position of the agent
//  * @param[in] goal the final destination of the agent
//  * @param[out] pathCost the cost of the path @c path.
//  * @param[out] moves the built path. Each index `i` represents the action the CPD told us to perform. Each action is the i-th OutArc we need to select
//  * 	in the map we're in. So it's the second parameter of AdjGraph::getIthOutArc;
//  * @param[inout] path the built path. At index `i` the vector contains the cell where we need to go in order to reach the location @c goal
//  * @return
//  *  @li true if we were able to find the optional path (thus @c path is altered);
//  *  @li false if no path could be obtained (thus @c path is unchanged);
//  */
// //TODO remove bool GetPathDataAllAtOnceWithCPD(void* cpdHelper, xyLoc start, xyLoc goal, dpf::cost_t& pathCost, std::vector<dpf::move_t>& moves, std::vector<xyLoc>& path);

// /**
//  * like ::GetPathDataAllAtOnceWithCPD but generates the cost of the path in the CPD as well
//  *
//  * @param[inout] cpdHelper the helper created with ::PrepareForSearch
//  * @param[in] start the initial position of the agent
//  * @param[in] goal the final destination of the agent
//  * @param[out] pathCost the cost of the path @c path.
//  * @param[out] moves the built path. Each index `i` represents the action the CPD told us to perform. Each action is the i-th OutArc we need to select
//  * 	in the map we're in. So it's the second parameter of AdjGraph::getIthOutArc;
//  * @return
//  *  @li true if we were able to find the optional path (thus @c path is altered);
//  *  @li false if no path could be obtained (thus @c path is unchanged);
//  */
// //TODO remove bool GetPathDataCostAndMovesWithCPD(void* cpdHelper, xyLoc start, xyLoc goal, dpf::cost_t& pathCost, std::vector<dpf::move_t>& moves);

// /**
//  * Retrieve all the information about the next optimal move through the CPD
//  *
//  * This function is sucpposed to be run in a loop by altering @c start each time with the last position @c cells has
//  *
//  * @param[in] cpdHelper a structure aiding you in the first move computation
//  * @param[in] start the position where the agent currently is
//  * @param[in] goal the position where the agent would like to be
//  * @param[inout] pathCost an accumulator variable used to store the cost fo the path we're generating
//  * @param[inout] moves an accumulator storing all the action we're doing in order to go from @c start till @c goal
//  * @param[out] nextCellId id of the next position where the CPD told us to go
//  * @param[out] isPathTerminated true if no more calls of this function should be performed, false otheriwsE;
//  * @return
//  *  @li true if we've reached the @c goal;
//  *  @li false otherwise
//  */
// //TODO remove bool GetPathInformationNextMoveWithCPD(void* cpdHelper, xyLoc start, xyLoc goal, dpf::cost_t& pathCost, std::vector<dpf::move_t>& moves, uint32_t& nextCellId, bool& isPathTerminated);

// /**
//  * retrieves the cost of the optimal path after determine the next move of the path itself
//  *
//  * The function computes what is the cost of the **first move** of the optimal path from going from @c start till @c goal.
//  *
//  * @note
//  * in general the cost fo the path and the length of the plan are different things.
//  *
//  * @param[inout] cpdHelper a pointer used to interact with the cpd
//  * @param[in] start the starting location where the agent is
//  * @param[in] goal the final destination of the agent
//  * @param[out] pathCost place where to put the cost of the first move of the path generated by the CPD
//  * @param[out] isPathTerminated true if no more actions can be perform in order to reach the goal; false otherwise
//  * @return
//  *  @li true if we have reached the goal
//  *  @li false if otherwise;
//  */
// //TODO remove bool GetPathLengthNextMoveWithCPD(void* cpdHelper, xyLoc start, xyLoc goal, dpf::cost_t& pathCost, bool& isPathTerminated);

// /**
//  * like ::GetPathLengthNextMoveWithCPD but generates the whole path in one sweep
//  *
//  * @param[inout] cpdHelper a pointer used to interact with the cpd
//  * @param[in] start the starting location where the agent is
//  * @param[in] goal the final destination of the agent
//  * @param[out] pathCost place where to put the cost of the first move of the path generated by the CPD
//  * @return
//  *  @li true if it's possible to reach @c goal from @c start
//  *  @li false otherwise
//  */
// //TODO remove bool getPathLengthAllAtOnceWithCPD(void* cpdHelper, xyLoc start, xyLoc goal, dpf::cost_t& pathCost);

const char *GetName();

/**
 * Draw a representation of a map with a path embedded into it
 *
 * @param[inout] str the stream where to write the data on
 * @param[in] map the map where to write the path on
 * @param[in] width the number of columns in the map
 * @param[in] height the number of rows in the map
 * @param[in] path the path to write
 * @param[in] pathSymbol the character to use to print the elements in the path (start and goal locations are excluded though)
 */
//TODO remove void drawMapWithPath(std::ostream& str, const std::vector<bool>& map, int width, int height, const std::vector<xyLoc>& path, char pathSymbol='*');

/**
 * generate an image representing the first best move of the cpd
 *
 * @post
 *  @li an image is present in CWD
 *
 * @param[in] mapName the filename containing the map to print
 * @param[in] cpdFilename a filename specifying which is the file containing the cpd associated to @c mapName
 * @param[in] goal the target the agent aims to reach
 * @param[in] basename the base name of the image to generate
 */
//TODO remove void printCPD(const char* mapName, const char* cpdFilename, xyLoc goal, const std::string& basename);

}

#endif
