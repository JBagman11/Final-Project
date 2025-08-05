#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <queue>
#include <tuple>
#include <climits>
#include <cstdlib>
#include <ctime>
#include <chrono>
using namespace std;

const int CELL_SIZE = 2; //Each cell(square)= 2x2 pixels on the screen
int dirRow[4] = {-1, 1, 0, 0};
int dirCol[4] = {0, 0, -1, 1};

//Estimates the distance from the current cell to the goal. The distance from a given node to the goal.
int estimateDistanceToGoal(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

vector<vector<int>> generateRandomMaze(int rows, int cols, int openProbability = 60) {
    vector<vector<int>> maze(rows, vector<int>(cols)); //creates 2d vector(grid). with size rows x cols

    //Fill each cell randomly
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            maze[i][j] = (rand() % 100 < openProbability) ? 0 : 1;  //If the number randomly generated(between 0-99) is less than openProbability, the cell becomes 0 (open),else 1 (wall).
    return maze;
}

pair<pair<int, int>, pair<int, int>> randomizeStartEnd(int rows, int cols, vector<vector<int>>& maze) {
    vector<pair<int, int>> borderCells;
    for (int j = 0; j < cols; j++) {
        borderCells.emplace_back(0, j);
        borderCells.emplace_back(rows - 1, j);
    }
    for (int i = 1; i < rows - 1; i++) {
        borderCells.emplace_back(i, 0);
        borderCells.emplace_back(i, cols - 1);
    }

    int startIdx = rand() % borderCells.size();
    int endIdx;
    do {
        endIdx = rand() % borderCells.size();
    } while (endIdx == startIdx ||
             estimateDistanceToGoal(borderCells[startIdx].first, borderCells[startIdx].second,
                                    borderCells[endIdx].first, borderCells[endIdx].second) < (rows + cols) / 2);

    pair<int, int> start = borderCells[startIdx];
    pair<int, int> end = borderCells[endIdx];

    for (int i = 0; i < rows; i++) {
        maze[i][0] = 1;
        maze[i][cols - 1] = 1;
    }
    for (int j = 0; j < cols; j++) {
        maze[0][j] = 1;
        maze[rows - 1][j] = 1;
    }

    maze[start.first][start.second] = 0;
    maze[end.first][end.second] = 0;

    return {start, end};
}

int bfs(const vector<vector<int>> &mazeGrid, pair<int, int> startCell, pair<int, int> endCell,vector<vector<pair<int, int>>> &previousCell) {

    int numRows = mazeGrid.size(), numCols = mazeGrid[0].size();

    //Grid to track visited cells. initialized to false
    vector<vector<bool>> visitedCells(numRows, vector<bool>(numCols, false));

    //Grid to track the previous cell for each visited cell. For path reconstruction
    previousCell.assign(numRows, vector<pair<int, int>>(numCols, {-1, -1}));

    //Queue for the BFS traversal. It stores (row, col, distance from start)
    queue<tuple<int, int, int>> bfsQueue;
    bfsQueue.push({startCell.first, startCell.second, 0});
    //A 2D boolean grid, the entire grid. True if the cell was visited, else false.
    visitedCells[startCell.first][startCell.second] = true;

    while (!bfsQueue.empty()) {
        //Dequeue current cell information
        auto [currentRow, currentCol, distanceFromStart] = bfsQueue.front();
        bfsQueue.pop();

        //If end cell is reached, return the path length(number of steps/cells)
        if (make_pair(currentRow, currentCol) == endCell) return distanceFromStart;

        //Explores the four possible directions(up, down, left, right)
        for (int directionIndex = 0; directionIndex < 4; directionIndex++) {
            int nextRow = currentRow + dirRow[directionIndex];
            int nextCol = currentCol + dirCol[directionIndex];

            // Check bounds and whether the next cell is a valid path and not yet visited
            if (nextRow >= 0 && nextCol >= 0 && nextRow < numRows && nextCol < numCols && mazeGrid[nextRow][nextCol] == 0 && !visitedCells[nextRow][nextCol]) {
                //Marks the next cell as visited
                visitedCells[nextRow][nextCol] = true;
                //Records where we came from to reach this next cell
                previousCell[nextRow][nextCol] = {currentRow, currentCol};
                //Adds the next cell to the BFS queue with updated distance
                bfsQueue.push({nextRow, nextCol, distanceFromStart + 1});
            }
        }
    }
    //If no path was found then return -1.
    return -1;
}


int astar(const vector<vector<int>> &mazeGrid, pair<int, int> startCell, pair<int, int> endCell, vector<vector<pair<int, int>>> &previousCell) {
    int numRows = mazeGrid.size(), numCols = mazeGrid[0].size();

    //The cost from start cell to each cell (g-values), initialized to infinity
    vector<vector<int>> costFromStart(numRows, vector<int>(numCols, INT_MAX));

    //Tracks the visited cells
    vector<vector<bool>> visitedCells(numRows, vector<bool>(numCols, false));


    //Grid to store the previous step for reconstructing the path
    previousCell.assign(numRows, vector<pair<int, int>>(numCols, {-1, -1}));

    //Heap-based priority queue. Each element in the queue is a tuple holding 3 values(estimatedTotalCostToGoal, row, col).
    //greater<> Makes the queue a min-heap where the smallest estimatedTotalCostToGoal will be top. (default is max heap)
    priority_queue<tuple<int, int, int>, vector<tuple<int, int, int>>, greater<>> cellsToExplore;

    // Initialize the start cell
    costFromStart[startCell.first][startCell.second] = 0;
    int initialEstimate = estimateDistanceToGoal(startCell.first, startCell.second, endCell.first, endCell.second);
    cellsToExplore.push({initialEstimate, startCell.first, startCell.second});

    while (!cellsToExplore.empty()) {
        auto [estimatedTotalCostToGoal, currentRow, currentCol] = cellsToExplore.top();
        cellsToExplore.pop();

        //Skips if already processed
        if (visitedCells[currentRow][currentCol]) continue;
        visitedCells[currentRow][currentCol] = true;

        //If goal is reached, return the cost
        if (make_pair(currentRow, currentCol) == endCell)
            return costFromStart[currentRow][currentCol];

        //Explore neighbors
        for (int directionIndex = 0; directionIndex < 4; directionIndex++) {
            int nextRow = currentRow + dirRow[directionIndex];
            int nextCol = currentCol + dirCol[directionIndex];

            if (nextRow >= 0 && nextCol >= 0 && nextRow < numRows && nextCol < numCols &&
                mazeGrid[nextRow][nextCol] == 0) {

                int newCost = costFromStart[currentRow][currentCol] + 1;

                if (newCost < costFromStart[nextRow][nextCol]) {
                    costFromStart[nextRow][nextCol] = newCost;

                    int heuristic = estimateDistanceToGoal(nextRow, nextCol, endCell.first, endCell.second);
                    int totalEstimatedCost = newCost + heuristic;

                    previousCell[nextRow][nextCol] = {currentRow, currentCol};
                    cellsToExplore.push({totalEstimatedCost, nextRow, nextCol});
                }
            }
        }
    }

    // No path found
    return -1;
}

//Runs the chosen pathfinding algorithm (BFS or A*) and returns the performance info(path length, time taken in milliseconds, and memory used in bytes)
tuple<int, double, size_t> runPathfindingAlgorithm(int algorithmType,const vector<vector<int>>& mazeGrid,pair<int, int> startCell,pair<int, int> endCell,
                                                   vector<vector<pair<int, int>>>& outputPreviousCells,vector<vector<bool>>& outputPathMap){
    //algorithmType: 1 = BFS, 2 = A*
    //startCell: Starting cell coordinates
    //endCell:  Ending cell coordinates
    //outputPreviousCells: Tracks path for reconstruction
    //outputPathMap: Path mark that was taken


    int numRows = mazeGrid.size(), numCols = mazeGrid[0].size();

    //Clear and initialize the previous cell tracker with (-1, -1) for all cells
    outputPreviousCells.clear();
    for (int i = 0; i < numRows; i++) {
        vector<pair<int, int>> row(numCols, {-1, -1});
        outputPreviousCells.push_back(row);
    }

    //Start timing the algorithm. Starts the counter/timer
    auto timeStart = chrono::high_resolution_clock::now();

    int pathLength = -1;
    //Runs the selected algorithm
    if (algorithmType == 1)
        pathLength = bfs(mazeGrid, startCell, endCell, outputPreviousCells);
    else if (algorithmType == 2)
        pathLength = astar(mazeGrid, startCell, endCell, outputPreviousCells);

    //stops timer/counter.
    auto timeEnd = chrono::high_resolution_clock::now();

    chrono::duration<double, milli> elapsedTime = timeEnd - timeStart;

    //Backtracks from endCell to startCell using outputPreviousCells to mark the path
    pair<int, int> traceCell = endCell;
    int stepCount = 0;
    while (traceCell != startCell && outputPreviousCells[traceCell.first][traceCell.second] != make_pair(-1, -1)) {

        //Marks this cell as part of the path
        outputPathMap[traceCell.first][traceCell.second] = true;
        //Moves to the previous cell
        traceCell = outputPreviousCells[traceCell.first][traceCell.second];

        stepCount++;
    }

    //returns path length(cells), time taken(ms), and memory used(based on step count)
    return {pathLength, elapsedTime.count(), stepCount * sizeof(pair<int, int>)};
}

int main() {
    srand(static_cast<unsigned>(time(nullptr)));

    const int mazeRows = 317, mazeCols = 317;

    vector<vector<int>> mazeGrid;
    pair<int, int> startCell, endCell;
    vector<vector<pair<int, int>>> previousCells;

    // Generate first solvable maze
    do {
        mazeGrid = generateRandomMaze(mazeRows, mazeCols);
        tie(startCell, endCell) = randomizeStartEnd(mazeRows, mazeCols, mazeGrid);
    } while (bfs(mazeGrid, startCell, endCell, previousCells) == -1);

    while (true) {
        cout << "|========================================================|\n";
        cout << "|           MAZE PATHFINDING VISUALIZER:                 |\n";
        cout << "|                                                        |\n";
        cout << "|Select an option:                                       |\n";
        cout << "|1)Generate New Maze                                     |\n";
        cout << "|2)Run BFS Algorithm                                     |\n";
        cout << "|3)Run A* Algorithm                                      |\n";
        cout << "|4)Compare BFS vs A* on Maze                             |\n";
        cout << "|5)Exit                                                  |\n";
        cout << "|========================================================|\n";
        cout << "Enter your choice[1-5]: \n";

        int userChoice;
        cin >> userChoice;

        if (userChoice == 5) {
            cout << "Exiting Maze Visualizer. Goodbye!\n";
            break;
        }

        if (userChoice == 1) {
            do {
                mazeGrid = generateRandomMaze(mazeRows, mazeCols);
                tie(startCell, endCell) = randomizeStartEnd(mazeRows, mazeCols, mazeGrid);
            } while (bfs(mazeGrid, startCell, endCell, previousCells) == -1);
            cout << "New maze generated!\n\n";
        }

        else if (userChoice == 2 || userChoice == 3) {
            int algorithmType = (userChoice == 2) ? 1 : 2;
            vector<vector<bool>> visitedPathMap(mazeRows, vector<bool>(mazeCols, false));
            auto [pathLength, elapsedMs, memoryBytes] = runPathfindingAlgorithm(algorithmType, mazeGrid, startCell, endCell, previousCells, visitedPathMap);

            cout << "Path found\nTime: " << elapsedMs << " ms\nLength: " << pathLength << " Cells\nMemory: " << memoryBytes << " bytes\n";
            cout << "\nClose window to continue..." << endl;

            sf::RenderWindow window(sf::VideoMode(mazeCols * CELL_SIZE, mazeRows * CELL_SIZE), algorithmType == 1 ? "BFS Path" : "A* Path");
            while (window.isOpen()) {
                sf::Event event;
                while (window.pollEvent(event))
                    if (event.type == sf::Event::Closed) window.close();

                window.clear();
                for (int i = 0; i < mazeRows; ++i) {
                    for (int j = 0; j < mazeCols; ++j) {
                        sf::RectangleShape cell(sf::Vector2f(CELL_SIZE, CELL_SIZE));
                        cell.setPosition(j * CELL_SIZE, i * CELL_SIZE);

                        if (mazeGrid[i][j] == 1) cell.setFillColor(sf::Color::Black);
                        else if (make_pair(i, j) == startCell) cell.setFillColor(sf::Color::Green);
                        else if (make_pair(i, j) == endCell) cell.setFillColor(sf::Color::Red);
                        else if (visitedPathMap[i][j]) cell.setFillColor(algorithmType == 1 ? sf::Color::Yellow : sf::Color::Blue);
                        else cell.setFillColor(sf::Color::White);

                        window.draw(cell);
                    }
                }
                window.display();
            }
        }

        else if (userChoice == 4) {
            vector<vector<bool>> bfsPath(mazeRows, vector<bool>(mazeCols, false));
            vector<vector<bool>> astarPath(mazeRows, vector<bool>(mazeCols, false));
            vector<vector<pair<int, int>>> bfsPrev, astarPrev;

            auto [len1, time1, mem1] = runPathfindingAlgorithm(1, mazeGrid, startCell, endCell, bfsPrev, bfsPath);
            auto [len2, time2, mem2] = runPathfindingAlgorithm(2, mazeGrid, startCell, endCell, astarPrev, astarPath);

            cout << "\nYellow: BFS   Blue: A*   Pink: Overlap\n";
            cout << "BFS:\n  Time: " << time1 << " ms\n  Length: " << len1 << "\n  Memory: " << mem1 << " bytes\n";
            cout << "A*:\n  Time: " << time2 << " ms\n  Length: " << len2 << "\n  Memory: " << mem2 << " bytes\n";

            sf::RenderWindow window(sf::VideoMode(mazeCols * CELL_SIZE, mazeRows * CELL_SIZE), "BFS vs A* Comparison");

            while (window.isOpen()) {
                sf::Event event;
                while (window.pollEvent(event))
                    if (event.type == sf::Event::Closed) window.close();

                window.clear();
                for (int i = 0; i < mazeRows; ++i) {
                    for (int j = 0; j < mazeCols; ++j) {
                        sf::RectangleShape cell(sf::Vector2f(CELL_SIZE, CELL_SIZE));
                        cell.setPosition(j * CELL_SIZE, i * CELL_SIZE);

                        if (mazeGrid[i][j] == 1) cell.setFillColor(sf::Color::Black);
                        else if (make_pair(i, j) == startCell) cell.setFillColor(sf::Color::Green);
                        else if (make_pair(i, j) == endCell) cell.setFillColor(sf::Color::Red);
                        else if (bfsPath[i][j] && astarPath[i][j]) cell.setFillColor(sf::Color::Magenta);
                        else if (bfsPath[i][j]) cell.setFillColor(sf::Color::Yellow);
                        else if (astarPath[i][j]) cell.setFillColor(sf::Color::Blue);
                        else cell.setFillColor(sf::Color::White);

                        window.draw(cell);
                    }
                }
                window.display();
            }
        }

        else {
            cout << "Invalid option. Please select 1-5.\n";
        }
    }

    return 0;
}

