/**
 * @file
 *
 * Module useful for debugging to understand how a slide of CPD (CPD map only for a specific goal) is laid down
 *
 * Together with the abstarct classes this module also provide some basic implementations
 *
 *  Created on: Oct 11, 2018
 *      Author: koldar
 */

#ifndef CPD_SLICE_PRINTER_H_
#define CPD_SLICE_PRINTER_H_

#include "map_loader.h"
#include "xyLoc.h"
#include <vector>
#include "Entry.h"
#include "file_utils.h"
#include <fstream>
#include <unordered_map>
#include <warthog/search_node.h>

/**
 * Print a slide of the CPD
 *
 * @param[in] rawMap a map loaded thaks to LoadMap
 * @param[in] width width of @c rawMap
 * @param[in] height height of @c rawMap
 * @param[in] mapper a mapper to use to convert nodeids and locations in @c rawMap
 * @param[in] goal the location we want to reach
 * @param[in] basename name of the file to generate (no extension!)
 * @param[in] labelPrinter a structure that tells us what text to put inside a cell of the map
 * @param[in] colorPrinter a structure that tells us what color to paint the background of a cell of the map
 * @param[in] topLeftLoc top left corner of the rectangle to draw. If nullptr there is no bound of the4 drawing
 * @param[in] bottomRightLoc bottom right corner of the rectangle to draw. If nullptr there is no bound of the drawing
 */
template<typename LABELPRINTER, typename COLORPRINTER>
void printGridMap(const std::vector<bool> rawMap, int width, int height, const Mapper& mapper, xyLoc goal, const string& basename, LABELPRINTER& labelPrinter, COLORPRINTER& colorPrinter, const xyLoc* topLeftLoc=nullptr, const xyLoc* bottomRightLoc=nullptr) {
	MapLoader mp;

	stringstream ss;
	ss << basename << ".dot";
	ofstream dotFile;
	dotFile.open(ss.str(), std::ios::trunc | std::ios::out);

	xyLoc topLeft;
	xyLoc bottomRight;
	topLeft = (topLeftLoc != nullptr) ? *topLeftLoc : xyLoc{0,0};
	bottomRight = (bottomRightLoc != nullptr) ? *bottomRightLoc : xyLoc{static_cast<dpf::coo2d_t>(width),static_cast<dpf::coo2d_t>(height)};


	//x [shape="none", label=<<TABLE><TR><TD>a</TD><TD>a</TD></TR><TR><TD>a</TD><TD>a</TD></TR></TABLE>>];
	dotFile << "graph {" << "\n"
			<< "map [shape=\"none\", label=<<TABLE>" << "\n";

	for (dpf::coo2d_t y=0; y<height; ++y) {
		bool shouldPrintRow = false;

		for (dpf::coo2d_t x=0; x<width; ++x) {
			const char* label;
			const char* color;
			xyLoc source = xyLoc{x,y};

			if (source.isInside(topLeft, bottomRight)){

				if (!shouldPrintRow) {
					dotFile << " <TR>" << "\n";
					shouldPrintRow = true;
				}

				label = labelPrinter.handleLabelCell(rawMap, width, height, mapper, source, goal).c_str();
				color = colorPrinter.handleColorCell(rawMap, width, height, mapper, source, goal).c_str();

				dotFile << "  <TD BGCOLOR=\"" << color << "\">" << label << "</TD>" << "\n";
			}
		}

		if (shouldPrintRow) {
			dotFile << " </TR>" << "\n";
		}
	}

	dotFile << "</TABLE>>];" << "\n"
			<< "}" << "\n";

	dotFile.close();

	ss.str(""); ss.clear();
	ss << "dot -Tsvg -o " << basename << ".svg " << basename << ".dot";
	int errorCode = system(ss.str().c_str());
	if (errorCode != 0) {
		warning(ss.str(), "resulted in an error code of", errorCode, ". Continuing as nothing happened");
	}

	ss.str(""); ss.clear();
	ss << basename << ".dot";
	//unlink(ss.str().c_str());
}


class AbstractLabelPrinter {
public:
	virtual ~AbstractLabelPrinter() {

	}
	virtual std::string handleLabelCell(const std::vector<bool>& rawMap, int width, int height, const Mapper& mapper, const xyLoc& source, const xyLoc& goal) = 0;
};

class AbstractColorPrinter {
public:
	virtual ~AbstractColorPrinter() {

	}
	virtual std::string handleColorCell(const std::vector<bool>& rawMap, int width, int height, const Mapper& mapper, const xyLoc& source, const xyLoc& goal) = 0;
};

/**
 * A print which print for every cell the id and the coordiantes of the cell according to a mapper
 */
class DefaultLabelPrinter : public AbstractLabelPrinter {
public:
	virtual std::string handleLabelCell(const std::vector<bool>& rawMap, int width, int height, const Mapper& mapper, const xyLoc& source, const xyLoc& goal) {
		std::string label;
		if (!rawMap[source.y*width+source.x]) {
			//untraversable cell
			label = std::string{""};
		} else if (source == goal) {
			label = std::string{"GOAL"};
		} else {
			stringstream ss;
			ss << source.x << "," << source.y << "\n" << "(" << mapper(source) << ")";
			label = ss.str();
		}
		return label;
	}
};

/**
 * A color printer which simply paint traversable, untraversable and goal locations
 */
class DefaultColorPrinter : public AbstractColorPrinter {
public:
	DefaultColorPrinter() {

	}

	virtual std::string handleColorCell(const std::vector<bool>& rawMap, int width, int height, const Mapper& mapper, const xyLoc& source, const xyLoc& goal) {
		const char* color;

		if (!rawMap[source.y*width+source.x]) {
			//untraversable cell
			return std::string{"black"};
		}

		if (source == goal) {
			return std::string{"green"};
		}

		return std::string{"white"};
	}
};

class ExpandedColorPrinter : public AbstractColorPrinter {
public:
	ExpandedColorPrinter(const std::unordered_map<uint32_t, warthog::search_node&>& nodeMap) : nodeMap{nodeMap} {

	}

	virtual ~ExpandedColorPrinter() {

	}

	virtual std::string handleColorCell(const std::vector<bool>& rawMap, int width, int height, const Mapper& mapper, const xyLoc& source, const xyLoc& goal) {
		const char* color;

		if (!rawMap[source.y*width+source.x]) {
			//untraversable cell
			return std::string{"black"};
		}

		if (source == goal) {
			return std::string{"green"};
		}

		auto it = nodeMap.find(mapper(source));
		if (it == nodeMap.end()) {
			return std::string{"grey"};
		}

		if (it->second.get_expanded()) {
			return std::string{"white"};
		} else {
			return std::string{"red"};
		}

	}
private:
	const std::unordered_map<uint32_t, warthog::search_node&>& nodeMap;
};



#endif /* CPD_SLICE_PRINTER_H_ */
