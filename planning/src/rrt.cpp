#include "rrt.h"

planning::RRT::RRT(int numberOfShots, float dragoonLength, int searchThresh) : dragoonConstraints(numberOfShots, dragoonLength), searchThresh(searchThresh)
{
    // to be set by ros parameters
    goalThresh = 0.2;
}

void
planning::RRT::setGraph(const std::vector<DragoonConfig>& newGraph)
{
    graph = newGraph;
}

planning::DragoonConfig
planning::RRT::getConfig(int index) const
{
    if (graph.empty())
    {
        return DragoonConfig();
    } else if (index >= 0 and index < graph.size())
    {
        return graph.at(index);
    } else
    {
        return graph[0];
    }
}

bool
planning::RRT::run(DragoonConfig start, DragoonConfig goal, std::vector<DragoonConfig> &outPath){
    // parameters for shooting algo
    DragoonAction sampAction;
    DragoonConfig sampState;
    // configurations for rrt
    DragoonConfig qRand;
    DragoonConfig qNear;
    bool goalFound = false;
    float distToGoal = 10;
    // initialize the lists
    graph.push_back(start);
    actions.push_back(sampAction);
    parents.push_back(0);
    while (graph.size() < searchThresh and not goalFound)
    {
        // sample a random configuration
        qRand.sampleConfig(3, 3, 3);
        // goal bias
        if (planning::random() < goalBias)
        {
            DragoonConfig goalGuess = goalSphere(goal);
            qRand.setConfig(goalGuess);
        }

        // find the nearest vertex
        int qNearIdx = findNearestVertex(qRand);
        qNear = graph[qNearIdx];
        bool valid = dragoonConstraints.handleConstraints(qNear, qRand, sampState, sampAction);
        // if valid sample state was found, add it to the graph
        if (valid){
            graph.push_back(sampState);
            actions.push_back(sampAction);
            parents.push_back(qNearIdx);
        }

        // check if we're near the goal
        const int goalNearIdx = findNearestVertex(goal);
        distToGoal = norm(goal - graph[goalNearIdx]);
        if (distToGoal < goalThresh)
        {
            goalFound = true;
            graph.push_back(goal);
            parents.push_back(goalNearIdx);
        }
    }
    std::cout << "Distance to goal: " << distToGoal << " Nodes: " << graph.size() << std::endl;

    if (goalFound){
        int parent = parents.back();
        DragoonConfig node = getConfig(parent);
        outPath.push_back(goal);

        while (not norm(start - node) < 0.0001)
        {
            parent = parents[parent];
            node = getConfig(parent);
            outPath.emplace_back(std::move(node));
        }
        // return the path from start to goal
        std::reverse(outPath.begin(), outPath.end());
    }
    return goalFound;
}

int
planning::RRT::findNearestVertex(const DragoonConfig &new_point) const
{
    int index = 0;
    std::vector<float> distances;
    for (auto config : graph)
    {
        // push the norm into the array
        distances.emplace_back(norm(config - new_point));
    }
    // get the index of the minimum element
    index =std::min_element(distances.begin(), distances.end()) - distances.begin();
    return index;
}

planning::DragoonConfig
planning::RRT::goalSphere(const DragoonConfig &goal) const
{
    // lows and highs for the config
    const float lowX      = goal.x - epsilon[0];
    const float lowY      = goal.y - epsilon[1];
    const float lowPsi    = goal.psi - epsilon[2];
    const float highX     = goal.x + epsilon[0];
    const float highY     = goal.y + epsilon[1];
    const float highPsi   = goal.psi + epsilon[2];
    // generate random sample in space epsilon around goal
    const float xSphere   = random(lowX, highX);
    const float ySphere   = random(lowY, highY);
    const float psiSphere = random(lowPsi, highPsi);
    return DragoonConfig {xSphere, ySphere, psiSphere};
}
