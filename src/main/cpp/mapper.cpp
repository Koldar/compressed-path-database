// #include "mapper.hpp"
// #include <cstdio>
// using namespace std;

// namespace compressed_path_database::datastructures {

// Mapper::Mapper(const std::vector<bool>&field, int width, int height):
// 				width_(width),
// 				height_(height),
// 				node_count_(0),
// 				pos_to_node_(width*height, -1)
// {
// 	for(dpf::coo2d_t y=0; y<height_; ++y)
// 		for(dpf::coo2d_t x=0; x<width_; ++x)
// 			if(field[x+y*width_]){
// 				node_to_pos_.push_back({(x), (y)});
// 				pos_to_node_[x+y*width_] = node_count_++;
// 			}else{
// 				pos_to_node_[x+y*width_] = -1;
// 			}
// }

// Mapper::Mapper(const GridMap& map) : width_(map.getWidth()),
// 		height_(map.getHeight()),
// 		node_count_(0),
// 		pos_to_node_(map.getWidth()*map.getHeight(), -1)
// {
// 	for(dpf::coo2d_t y=0; y<height_; ++y)
// 		for(dpf::coo2d_t x=0; x<width_; ++x)
// 			if(map.isTraversable(x, y)){
// 				node_to_pos_.push_back({(x), (y)});
// 				pos_to_node_[x+y*width_] = node_count_++;
// 			}else{
// 				pos_to_node_[x+y*width_] = -1;
// 			}
// }

// int Mapper::width()const{
// 	return width_;
// }

// int Mapper::height()const{
// 	return height_;
// }

// int Mapper::node_count()const{
// 	return node_count_;
// }

// xyLoc Mapper::operator()(dpf::nodeid_t x)const{ return node_to_pos_[x]; }

// dpf::nodeid_t Mapper::operator()(const xyLoc& p)const{
// 	if(p.x < 0 || p.x >= width_ || p.y < 0 || p.y >= height_)
// 		return -1;
// 	else
// 		return pos_to_node_[p.x + p.y*width_];
// }

// std::vector<xyLoc> Mapper::convert(const std::vector<dpf::nodeid_t> ids) const {
// 	std::vector<xyLoc> result;

// 	for (dpf::nodeid_t d : ids) {
// 		result.push_back(this->operator()(d));
// 	}

// 	return result;
// }

// std::vector<dpf::nodeid_t> Mapper::convert(const std::vector<xyLoc> locs) const {
// 	std::vector<dpf::nodeid_t> result;

// 	for (xyLoc loc : locs) {
// 		result.push_back(this->operator()(loc));
// 	}

// 	return result;
// }

// void Mapper::reorder(const NodeOrdering&order){
// 	for(auto&x:pos_to_node_){
// 		if(x != -1){
// 			x = order.to_new(x);
// 		}
// 	}
// 	std::vector<xyLoc>new_node_to_pos_(node_count_);
// 	for(int new_node=0; new_node<node_count(); ++new_node){
// 		int old_node = order.to_old(new_node);
// 		new_node_to_pos_[new_node] = node_to_pos_[old_node];
// 	}
// 	new_node_to_pos_.swap(node_to_pos_);
// }

// void dump_map(const Mapper&map, const char*file){
// 	FILE* f = fopen(file, "w");

// 	for(dpf::coo2d_t y=0; y<map.height(); ++y){
// 		for(dpf::coo2d_t x=0; x<map.width(); ++x){
// 			fprintf(f, "%5d", map(xyLoc{(x), (y)}));
// 		}
// 		fprintf(f, "\n");
// 	}
// 	fclose(f);
// }

// }