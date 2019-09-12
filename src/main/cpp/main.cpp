//#include <stdio.h>
//#include <stdint.h>
//#include <numeric>
//#include <algorithm>
//#include "ScenarioLoader.h"
//#include "Timer.h"
//#include "Entry.h"
//
//void LoadMap(const char *fname, std::vector<bool> &map, int &w, int &h);
//
//struct stats {
//	/**
//	 * for experiment i-th, the time the we needed to returnan answer to the query
//	 */
//	std::vector<double> times;
//	/**
//	 * for experiment i-th, the path computed to reach the goal from the start location
//	 */
//	std::vector<xyLoc> path;
//	/**
//		 * for experiment i-th, the path length computed to reach the goal from the start location
//		 */
//	std::vector<int> lengths;
//
//	double GetTotalTime()
//	{
//		return std::accumulate(times.begin(), times.end(), 0.0);
//	}
//	double GetMaxTimestep()
//	{
//		return *std::max_element(times.begin(), times.end());
//	}
//	double Get20MoveTime()
//	{
//		for (unsigned int x = 0; x < lengths.size(); x++)
//			if (lengths[x] >= 20)
//				return std::accumulate(times.begin(), times.begin()+1+x, 0.0);
//		return GetTotalTime();
//	}
//	double GetPathLength()
//	{
//		double len = 0;
//		for (int x = 0; x < (int)path.size()-1; x++)
//		{
//			if (path[x].x == path[x+1].x || path[x].y == path[x+1].y)
//			{
//				len++;
//			}
//			else {
//				len += 1.4142;
//			}
//		}
//		return len;
//	}
//	bool ValidatePath(int width, int height, const std::vector<bool> &mapData)
//	{
//		for (int x = 0; x < (int)path.size()-1; x++)
//		{
//			if (abs(path[x].x - path[x+1].x) > 1)
//				return false;
//			if (abs(path[x].y - path[x+1].y) > 1)
//				return false;
//			if (!mapData[path[x].y*width+path[x].x])
//				return false;
//			if (!mapData[path[x+1].y*width+path[x+1].x])
//				return false;
//			if (path[x].x != path[x+1].x && path[x].y != path[x+1].y)
//			{
//				if (!mapData[path[x+1].y*width+path[x].x])
//					return false;
//				if (!mapData[path[x].y*width+path[x+1].x])
//					return false;
//			}
//		}
//		return true;
//	}
//};
//
//int main(int argc, char **argv)
//{
//	char filename[255];
//	std::vector<xyLoc> thePath;
//	std::vector<bool> mapData;
//	int width, height;
//	//true if you want to run the preprocessing
//	bool pre = false;
//	//true if you want to run some queries (experimentally evaluated
//	bool run = false;
//
//	if (argc != 4)
//	{
//		printf("Invalid Arguments\nUsage %s <flag> <map> <scenario>\n", argv[0]);
//		printf("Flags:\n");
//		printf("\t-full : Preprocess map and run scenario\n");
//		printf("\t-pre : Preprocess map\n");
//		printf("\t-run : Run scenario without preprocessing\n");
//		exit(0);
//	}
//
//	//********************** argv[1] what should we do? **********************
//
//	if (strcmp(argv[1], "-full") == 0)
//	{
//		//enable both preprocessing and experiment running
//		pre = run = true;
//	}
//	else if (strcmp(argv[1], "-pre") == 0)
//	{
//		//enable only the preprocessing
//		pre = true;
//	}
//	else if (strcmp(argv[1], "-run") == 0)
//	{
//		//enable only the experiment running
//		run = true;
//	}
//	else {
//        printf("Invalid Arguments\nUsage %s <flag> <map> <scenario>\n", argv[0]);
//		printf("Flags:\n");
//        printf("\t-full : Preprocess map and run scenario\n");
//        printf("\t-pre : Preprocess map\n");
//        printf("\t-run : Run scenario without preprocessing\n");
//        exit(0);
//	}
//
//	// ***************************** argv[2] map filename **************************
//
//	LoadMap(argv[2], mapData, width, height);
//	//all maps have filename of pattern %s-%s
//	sprintf(filename, "%s-%s", GetName(), argv[2]);
//
//	// *************************** preprorcess the map *****************************
//
//	if (pre)
//	{
//		PreprocessMap(mapData, width, height, filename);
//	}
//
//	if (!run)
//	{
//		return 0;
//	}
//
//	// ********************** run queries ***********************
//
//	void *reference = PrepareForSearch(mapData, width, height, filename);
//
//
//	ScenarioLoader scen(argv[3]);
//
//	Timer t;
//	std::vector<stats> experimentStats;
//	//loop over the experiments in the scenario
//	for (int x = 0; x < scen.GetNumExperiments(); x++)
//    {
//		//printf("%d of %d\n", x+1, scen.GetNumExperiments());
//		thePath.resize(0);
//		experimentStats.resize(x+1);
//		bool done;
//		do {
//			xyLoc s, g;
//			s.x = scen.GetNthExperiment(x).GetStartX();
//			s.y = scen.GetNthExperiment(x).GetStartY();
//			g.x = scen.GetNthExperiment(x).GetGoalX();
//			g.y = scen.GetNthExperiment(x).GetGoalY();
//
//			t.StartTimer();
//			done = GetPath(reference, s, g, thePath);
//			t.EndTimer();
//
//			experimentStats[x].times.push_back(t.GetElapsedTime());
//			experimentStats[x].lengths.push_back(thePath.size());
//			for (unsigned int t = experimentStats[x].path.size(); t < thePath.size(); t++)
//				experimentStats[x].path.push_back(thePath[t]);
//		} while (done == false);
//
//    }
//
//	//write a file containing the timings
//	for (unsigned int x = 0; x < experimentStats.size(); x++)
//	{
//		printf("%s\ttotal-time\t%f\tmax-time-step\t%f\ttime-20-moves\t%f\ttotal-len\t%f\tsubopt\t%f\t", argv[3],
//			   experimentStats[x].GetTotalTime(), experimentStats[x].GetMaxTimestep(), experimentStats[x].Get20MoveTime(),
//			   experimentStats[x].GetPathLength(),
//			   experimentStats[x].GetPathLength() == scen.GetNthExperiment(x).GetDistance() ? 1.0 :
//			   experimentStats[x].GetPathLength() / scen.GetNthExperiment(x).GetDistance()
//		);
//		if (experimentStats[x].path.size() == 0 ||
//			(experimentStats[x].ValidatePath(width, height, mapData) &&
//			 scen.GetNthExperiment(x).GetStartX() == experimentStats[x].path[0].x &&
//			 scen.GetNthExperiment(x).GetStartY() == experimentStats[x].path[0].y &&
//			 scen.GetNthExperiment(x).GetGoalX() == experimentStats[x].path.back().x &&
//			 scen.GetNthExperiment(x).GetGoalY() == experimentStats[x].path.back().y))
//		{
//			printf("valid\n");
//		}
//		else {
//			printf("invalid\n");
//		}
//	}
//
//	return 0;
//}
//
///**
// * load a map from a file
// *
// * @post
// *  @li @c map contains true if the cell is traversabole, false otheriwse
// *
// * @includedoc mapLayout.dox
// *
// * @param[in] fname the name of the filename containing the map
// * @param[out] map outputs the map loaded
// * @param[out] width the width of the map read
// * @param[out] height the height the map read
// */
//void LoadMap(const char *fname, std::vector<bool> &map, int &width, int &height)
//{
//	FILE *f;
//	f = fopen(fname, "r");
//	if (f)
//    {
//		fscanf(f, "type octile\nheight %d\nwidth %d\nmap\n", &height, &width);
//		map.resize(height*width);
//		for (int y = 0; y < height; y++)
//		{
//			for (int x = 0; x < width; x++)
//			{
//				char c;
//				do {
//					fscanf(f, "%c", &c);
//				} while (isspace(c));
//				map[y*width+x] = (c == '.' || c == 'G' || c == 'S');
//				//printf("%c", c);
//			}
//			//printf("\n");
//		}
//		fclose(f);
//    }
//}
