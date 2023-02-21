
#include <iostream>
#include <chrono>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <array>
#include <list>
#include <unordered_set>
#include <set>
#include <utility>
#include <forward_list>
#include <iterator> 
#include <unordered_map>
#include <algorithm> 
#include <map>

using namespace std;
using namespace std::chrono;

using Graph = unordered_map <long long, unordered_map <long long, double>>;

    long long convertStringToLongLong(const string& input) {
        stringstream iss(input);
        long long retval;
        iss >> retval;
        return retval;
    }

    list<long long> splitString(const string& s, char delim) {
        stringstream ss(s);
        string itemString;
        list<long long> elemsList;
        while (getline(ss, itemString, delim)) {
            elemsList.push_back(convertStringToLongLong(itemString));
        }
        return elemsList;
    }

    void openFile(ifstream& myfile, set <long long>& nodeSet, set <pair <long long, long long>>& edgeSet, unordered_map <long long, long long> &nodeWComUMap) {
        string lineString;
        list<long long> firstSecElemList;

        if (myfile.is_open()) {
            while (getline(myfile, lineString)) {
                istringstream iss(lineString);
                firstSecElemList = splitString(lineString, ',');
                nodeSet.insert(firstSecElemList.front());
                nodeSet.insert(firstSecElemList.back());
                nodeWComUMap.insert(make_pair(firstSecElemList.front(), NULL));
                nodeWComUMap.insert(make_pair(firstSecElemList.back(), NULL));
                edgeSet.insert(make_pair(firstSecElemList.front(), firstSecElemList.back()));
                edgeSet.insert(make_pair(firstSecElemList.back(), firstSecElemList.front()));
            }
            myfile.close();
        }
        else {
            cout << "Unable to open file";

        }
    }


    void writeFile(ofstream& MyFile, unordered_map<long long, set<long long>>& assignedCommunitesUMap) {
        if (MyFile.is_open()) {
            for (auto it1 = assignedCommunitesUMap.begin(); it1 != assignedCommunitesUMap.end(); ++it1) {
                MyFile << (*it1).first << '\t';
                MyFile << (*it1).first << ',';
                for (auto it2 = ((*it1).second).begin(); it2 != ((*it1).second).end(); ++it2) {
                    if (it2 != --((*it1).second).end()) {
                        MyFile << *it2 << ',';
                    }
                    else {
                        MyFile << *it2;
                    }
                }
                MyFile << '\n';
            }
            MyFile.close();
        }
        else {
            cout << "Unable to open file";
        }

    }

    void nodeBaseRank(set <pair <long long, long long>>& edgeSet, unordered_map <long long, double>& nodeBaseRankUMap) {
        set <pair <long long, long long>>::iterator f_it = edgeSet.begin();
        set <pair <long long, long long>>::iterator s_it = ++edgeSet.begin();
        long long count = 0;
        while (f_it != edgeSet.end()) {
            count++;
            if ((*f_it).first != (*s_it).first) {
                nodeBaseRankUMap.insert(make_pair((*f_it).first, count / (double)edgeSet.size()));
                count = 0;
            }
            f_it++;
            if (s_it != --edgeSet.end()) {
                s_it++;
            }
            else {
                s_it = edgeSet.begin();
            }
        }
    }

    void calculateTotalRank(Graph& graph, set <pair <long long, long long>>& edgeSet, unordered_map <long long, double>& nodeBaseRankUMap, set <pair<double, long long>>& nodeTotalRankSet) {

        for (auto it = edgeSet.begin(); it != edgeSet.end(); ++it) {
            graph[(*it).first][(*it).second] = (nodeBaseRankUMap.find((*it).first))->second * (nodeBaseRankUMap.find((*it).second))->second;
        }

        double sumTotal;

        for (auto element : graph) {
            sumTotal = 0;
            //cout << "Node " << element.first << ": ";
            for (auto edge : element.second) {
                sumTotal += edge.second;
            }
            nodeTotalRankSet.insert(make_pair(sumTotal, element.first));
            //cout << "--->> " << sumTotal << '\n';
        }
    }


    void findComPhase1(Graph& graph, unsigned long long& minSizeOfCommunitySet, set <pair<double, long long>>& nodeTotalRankSet, unordered_map<long long, set<long long>>& assignedCommunitesUMap, unordered_map <long long, long long>& nodeWComUMap, set <long long>& nodeSet) {
        set <long long> addedNodesSet;
        set <long long> overlappedNodesSet;
        // assign nodeSet into communities
        for (set <pair<double, long long>>::reverse_iterator rit = nodeTotalRankSet.rbegin(); rit != nodeTotalRankSet.rend(); ++rit) {
            set<long long> tempCommunitySet;
            if (addedNodesSet.find((*rit).second) == addedNodesSet.end()) {
                // assign all edges into tempCommunitySet if not assigned any community 
                for (auto it = graph[(*rit).second].begin(); it != graph[(*rit).second].end(); ++it) {
                    if (addedNodesSet.find((*it).first) == addedNodesSet.end()) {
                        tempCommunitySet.insert((*it).first);
                    }
                    addedNodesSet.insert((*it).first);
                }
                addedNodesSet.insert((*rit).second);
                // if community size is equal to 0 do not add into assigned community
                if (tempCommunitySet.size() >= minSizeOfCommunitySet) {
                    assignedCommunitesUMap.insert(make_pair((*rit).second, tempCommunitySet));
                    for (auto it1 = tempCommunitySet.begin(); it1 != tempCommunitySet.end(); ++it1) {
                        nodeWComUMap[*it1] = (*rit).second;
                    }
                }
            }
            // if number of community is equal to aasigned community size 
            if (addedNodesSet.size() == graph.size()) {
                auto rit1 = nodeTotalRankSet.rbegin();
                // identifty overlapped nodeSet
                for (auto it1 = assignedCommunitesUMap[(*rit1).second].begin(); it1 != assignedCommunitesUMap[(*rit1).second].end(); ++it1) {
                    for (auto it2 = assignedCommunitesUMap.begin(); it2 != assignedCommunitesUMap.end(); ++it2) {
                        if ((*it2).first != (*rit1).second && graph[(*it2).first].find(*it1) != graph[(*it2).first].end()) {
                            overlappedNodesSet.insert(*it1);
                        }
                    }
                }
                // remove overlapped nodeSet from first community because we directly added overlapped nodeSet into first community
                for (auto it1 = overlappedNodesSet.begin(); it1 != overlappedNodesSet.end(); ++it1) {
                    assignedCommunitesUMap[(*rit1).second].erase(*it1);
                    nodeWComUMap[*it1] = NULL;
                }
                // remove used nodeSet
                for (auto it1 = assignedCommunitesUMap.begin(); it1 != assignedCommunitesUMap.end(); ++it1) {
                    nodeWComUMap[(*it1).first] = (*it1).first;
                    nodeSet.erase((*it1).first);
                    for (auto it2 = ((*it1).second).begin(); it2 != ((*it1).second).end(); ++it2) {
                        nodeSet.erase(*it2);
                    }
                }
                break;
            }
        }
    }


    void findComPhase2(Graph& graph, unordered_map<long long, set<long long>>& assignedCommunitesUMap, set <long long>& nodeSet, unordered_multimap <long long, pair <long long, long long>>& undeterminedNodesFreqUMMap, unordered_map <long long, long long>& nodeWComUMap, unordered_map <long long, unordered_map<long long, long long> >& counterTableUMap) {
        // determine each unused nodeSet with ranks
        for (auto it1 = nodeSet.begin(); it1 != nodeSet.end(); ++it1) {
            for (auto it2 = graph[*it1].begin(); it2 != graph[*it1].end(); ++it2) {
                if (nodeWComUMap[(*it2).first] != NULL) {
                    undeterminedNodesFreqUMMap.insert(make_pair(*it1, make_pair((*it2).first, nodeWComUMap[(*it2).first])));
                }
            }
        }
        // create Counter Table
        for (auto it1 = nodeSet.begin(); it1 != nodeSet.end(); ++it1) {
            // *it2 return elements with given keys (*it1)
            auto it2 = undeterminedNodesFreqUMMap.equal_range(*it1);
            // count edges with given nodeSet, identify those edges which will be assigned to which community depending on community frequency.
            unordered_map<long long, long long> counterComUMap;
            for (auto it3 = it2.first; it3 != it2.second; ++it3) {
                if (counterComUMap.find(((*it3).second).second) == counterComUMap.end()) {
                    counterComUMap.insert(make_pair(((*it3).second).second, 1));
                }
                else {
                    counterComUMap[((*it3).second).second]++;
                }
            }
            counterTableUMap[*it1] = counterComUMap;
        }
        for (auto it1 = nodeSet.begin(); it1 != nodeSet.end(); ) {
            if (counterTableUMap[*it1].size() != NULL) {
                auto it2 = counterTableUMap[*it1].begin();
                auto maxCnt = (*it2).second;
                auto nodeNum = (*it2).first;
                for (; it2 != counterTableUMap[*it1].end(); ++it2) {
                    if (maxCnt < (*it2).second) {
                        maxCnt = (*it2).second;
                        nodeNum = (*it2).first;
                    }
                }
                nodeWComUMap[*it1] = nodeNum;
                counterTableUMap.erase(*it1);
                undeterminedNodesFreqUMMap.erase(*it1);
                auto setNodes = assignedCommunitesUMap[nodeNum];
                setNodes.insert(*it1);
                assignedCommunitesUMap.erase(nodeNum);
                assignedCommunitesUMap.insert(make_pair(nodeNum, setNodes));
                it1 = nodeSet.erase(it1);
            }
            else {
                ++it1;
            }
        }

    }


    int main() {
        Graph graph;
        set <long long> nodeSet;
        unordered_map <long long, long long> nodeWComUMap;
        set <pair <long long, long long>> edgeSet;
        unordered_map <long long, double> nodeBaseRankUMap;
        set <pair<double, long long>> nodeTotalRankSet;
        unordered_map<long long, set<long long>> assignedCommunitesUMap;
        unordered_multimap <long long, pair <long long, long long>> undeterminedNodesFreqUMMap;
        unordered_map <long long, unordered_map<long long, long long>> counterTableUMap;

        ifstream myfile("dataset-karate.txt");
        ofstream MyFile("result-dataset-karate.txt");
        unsigned long long minSizeOfCommunitySet = 2;

        //ifstream myfile("dataset-football.txt");
        //ofstream MyFile("result-dataset-football.txt");
        //unsigned long long minSizeOfCommunitySet = 2;

        //ifstream myfile("dataset-polbooks.txt");
        //ofstream MyFile("result-dataset-polbooks.txt");
        //unsigned long long minSizeOfCommunitySet = 7;
    
        //ifstream myfile("dataset-dblp.txt");
        //ofstream MyFile("result-dataset-dblp.txt");
        //unsigned long long minSizeOfCommunitySet = 2; 

        //ifstream myfile("dataset-amazon.txt");
        //ofstream MyFile("result-dataset-amazon.txt");
        //unsigned long long minSizeOfCommunitySet = 2; 

        openFile(myfile, nodeSet, edgeSet, nodeWComUMap);

        // Get starting timepoint
        auto start = high_resolution_clock::now();

        nodeBaseRank(edgeSet, nodeBaseRankUMap);
        calculateTotalRank(graph, edgeSet, nodeBaseRankUMap, nodeTotalRankSet);
        findComPhase1(graph, minSizeOfCommunitySet, nodeTotalRankSet, assignedCommunitesUMap, nodeWComUMap, nodeSet);
        while (nodeSet.size() != 0) {
            findComPhase2(graph, assignedCommunitesUMap, nodeSet, undeterminedNodesFreqUMMap, nodeWComUMap, counterTableUMap);
        }

        // Get ending timepoint
        auto stop = high_resolution_clock::now();
        // Calculate duration
        auto duration = duration_cast<microseconds>(stop - start);
        cout << "Time taken by algorithm: " << duration.count() << " microseconds" << endl;
        writeFile(MyFile, assignedCommunitesUMap);
        return 0;
    }
