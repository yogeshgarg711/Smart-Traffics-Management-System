#include <bits/stdc++.h>
using namespace std;

// Forward declarations
class Intersection;
class Road;
class Vehicle;

// Represents an intersection in the city (node in the graph)
class Intersection {
public:
    int id;
    string name;
    bool hasTrafficLight;
    bool isGreenLight;
    int greenDuration;
    int redDuration;
    int lightTimer;
    vector<Road*> outgoingRoads;
    int waitingVehicleCount;

    Intersection(int id, string name, bool hasLight = false) {
        this->id = id;
        this->name = name;
        this->hasTrafficLight = hasLight;
        this->isGreenLight = true;
        this->greenDuration = 30;
        this->redDuration = 30;
        this->lightTimer = 0;
        this->waitingVehicleCount = 0;
    }

    void addOutgoingRoad(Road* road) {
        outgoingRoads.push_back(road);
    }

    void addWaitingVehicle() { waitingVehicleCount++; }
    void removeWaitingVehicle() { if (waitingVehicleCount > 0) waitingVehicleCount--; }

    void setLightDuration(int green, int red) {
        greenDuration = green;
        redDuration = red;
    }

    void updateLight() {
        if (!hasTrafficLight) return;

        lightTimer++;
        if (isGreenLight && lightTimer >= greenDuration) {
            isGreenLight = false;
            lightTimer = 0;
        }
        else if (!isGreenLight && lightTimer >= redDuration) {
            isGreenLight = true;
            lightTimer = 0;
        }
    }
};

// Represents a road between intersections (edge in the graph)
class Road {
public:
    int id;
    Intersection* source;
    Intersection* destination;

    Road(int id, Intersection* source, Intersection* destination) {
        this->id = id;
        this->source = source;
        this->destination = destination;
        source->addOutgoingRoad(this);
    }
};

// vehicle class
class Vehicle {
public:
    int id;
    Intersection* current;
    Intersection* destination;
    vector<Road*> route;
    int routeIndex;
    bool arrived;
    int arrivalTime;
    int sourceId;
    int destId;
    bool isCountedAsWaiting;

    Vehicle(int id, Intersection* start, Intersection* end) {
        this->id = id;
        this->current = start;
        this->destination = end;
        this->routeIndex = 0;
        this->arrived = false;
        this->arrivalTime = -1;
        this->sourceId = start->id;
        this->destId = end->id;
        this->isCountedAsWaiting = false;
    }
    
    void setRoute(vector<Road*>& newRoute) {
        route = newRoute;
        routeIndex = 0;
    }

    void move(int currentTimeStep) {
        if (arrived || route.empty() || routeIndex >= route.size()) return;

        // Get current road
        Road* currentRoad = route[routeIndex];
        Intersection* nextIntersection = currentRoad->destination;

        // Check if we need to wait at a traffic light
        if (current->hasTrafficLight && !current->isGreenLight) {
            // Only increment waiting count if not already counted
            if (!isCountedAsWaiting) {
                current->addWaitingVehicle();
                isCountedAsWaiting = true;
            }
            return; // Wait at red light
        }

        // Vehicle is moving, so if it was waiting, remove from waiting count
        if (isCountedAsWaiting) {
            current->removeWaitingVehicle();
            isCountedAsWaiting = false;
        }

        // Move from current intersection to the next one via the road
        current = nextIntersection;
        routeIndex++;

        // Check if reached destination
        if (current == destination) {
            arrived = true;
            arrivalTime = currentTimeStep + 1;
        }
    }

    string getRouteInfo() {
        // Get source and destination IDs
        int sourceId = route.empty() ? -1 : route[0]->source->id;
        int destId = destination->id;
        return "From " + to_string(sourceId) + " to " + to_string(destId) + " (path: " + to_string(route.size()) + " roads)";
    }
};

// The city graph representing road network
class CityGraph {
public:
    vector<Intersection*> intersections; 
    vector<Road*> roads;                 
    vector<Intersection*> trafficLights; 
    
    // Recursive backtracking function for findAllRoutes
    void findRoutesRecursive(Intersection* current, Intersection* end, 
                             vector<Road*>& path, vector<int>& visited,
                             vector<vector<Road*>>& results, int maxLength) {
        // Base case: reached destination
        if (current == end) {
            results.push_back(path);
            return;
        }
        
        // Base case: path too long
        if (path.size() >= maxLength) {
            return;
        }
        
        visited.push_back(current->id);
        
        auto it = current->outgoingRoads.begin();
        while (it != current->outgoingRoads.end()) {
            Road* road = *it;
            Intersection* next = road->destination;
            
            // Check if already visited
            bool isVisited = false;
            auto visitedIt = visited.begin();
            while (visitedIt != visited.end()) {
                if (*visitedIt == next->id) {
                    isVisited = true;
                    break;
                }
                ++visitedIt;
            }
            
            if (!isVisited) {
                path.push_back(road);
                findRoutesRecursive(next, end, path, visited, results, maxLength);
                path.pop_back();
            }
            
            ++it;
        }
        
        // Backtrack: remove the last element
        visited.pop_back();
    }

    void addIntersection(int id, const string& name, bool hasLight = false) {
        if (id >= intersections.size()) {
            intersections.resize(id + 1, nullptr);
        }
        intersections[id] = new Intersection(id, name, hasLight);
    }

    void addRoad(int id, int sourceId, int destId) {
        Intersection* source = getIntersection(sourceId);
        Intersection* dest = getIntersection(destId);
        if (source && dest) {
            if (id >= roads.size()) {
                roads.resize(id + 1, nullptr);
            }
            roads[id] = new Road(id, source, dest);
        }
    }

    Intersection* getIntersection(int id) {
        if (id < 0 || id >= intersections.size()) return nullptr;
        return intersections[id];
    }

    Road* getRoad(int id) {
        if (id < 0 || id >= roads.size()) return nullptr;
        return roads[id];
    }

    void findAllRoutes(int startId, int endId, int maxLength) {
        Intersection* start = getIntersection(startId);
        Intersection* end = getIntersection(endId);
        
        if (!start || !end) {
            cout << "Invalid start or end intersection" << endl;
            return;
        }
        
        vector<Road*> currentPath;
        vector<int> visited;
        vector<vector<Road*>> allRoutes;
        
        findRoutesRecursive(start, end, currentPath, visited, allRoutes, maxLength);
        
        if (allRoutes.empty()) {
            vector<Road*> anyPath = findShortestPath(startId, endId);
            
            if (anyPath.empty()) {
                cout << "No route exists between intersections " << startId << " and " << endId << "." << endl;
            } else if (anyPath.size() > maxLength) {
                cout << "Routes exist but exceed the maximum length of " << maxLength << " roads." << endl;
                cout << "The shortest path has length " << anyPath.size() << " roads." << endl;
            } else {
                cout << "No routes found. This might be due to search limitations." << endl;
            }
            return;
        }
        
        cout << "Found " << allRoutes.size() << " possible routes from " 
             << startId << " to " << endId << ":" << endl;
        
        for (int i = 0; i < allRoutes.size(); i++) {
            cout << "Route " << (i+1) << " (length " << allRoutes[i].size() << "): ";
            for (Road* road : allRoutes[i]) {
                cout << road->source->id << " -> ";
            }
            cout << endId << endl;
        }
    }

    vector<Road*> findShortestPath(int startId, int endId) {
        vector<Road*> path;
        vector<int> dis(intersections.size(), INT_MAX);
        vector<int> prev(intersections.size(), -1);
        vector<Road*> connectingRoads(intersections.size(), nullptr);

        // Min-heap priority queue to store the node and its distance
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        
        // Initialize distances with a large value and set the source distance as 0
        dis[startId] = 0;
        
        // Push the source node with distance 0 into the priority queue
        pq.push({0, startId});
        
        // Process the priority queue
        while (!pq.empty()) {
            // Get the node and its distance from the top of the priority queue
            int node = pq.top().second;
            int d = pq.top().first;
            pq.pop();
            
            // If we reached the destination, we can terminate early
            if (node == endId) break;
            
            // Skip if we've found a better path already
            if (d > dis[node]) continue;
            
            // Explore the neighbors of the current node
            Intersection* current = getIntersection(node);
            if (!current) continue;
            
            for (Road* road : current->outgoingRoads) {
                int nextNode = road->destination->id;
                int weight = 1;  // Weight of the edge (using 1 as default)
                
                // Relax the edge if a shorter path is found
                if (d + weight < dis[nextNode]) {
                    dis[nextNode] = d + weight;
                    prev[nextNode] = node;
                    connectingRoads[nextNode] = road;
                    pq.push({dis[nextNode], nextNode});  // Push the neighbor with updated distance
                }
            }
        }
        
        // No path found
        if (prev[endId] == -1 && startId != endId) {
            return path;
        }
        
        // Reconstruct the path
        int current = endId;
        while (current != startId) {
            Road* road = connectingRoads[current];
            if (!road) break;
            path.push_back(road);
            current = prev[current];
        }
        
        // Reverse the path to get correct order from start to end
        reverse(path.begin(), path.end());
        return path;
    }

    void optimizeTrafficLights() {
        trafficLights.clear();
        for (Intersection* intersection : intersections) {
            if (intersection && intersection->hasTrafficLight) {
                trafficLights.push_back(intersection);
            }
        }

        for (Intersection* intersection : trafficLights) {
            int waiting = intersection->waitingVehicleCount;
            
            if (waiting <= 3) {
                intersection->setLightDuration(20, 40);
                cout << "Intersection " << intersection->id 
                     << ": Short green (20s/40s) - Few vehicles: " << waiting << endl;
            } else {
                intersection->setLightDuration(40, 20);
                cout << "Intersection " << intersection->id 
                     << ": Long green (40s/20s) - Many vehicles: " << waiting << endl;
            }
        }

        cout << "Traffic lights optimized using simple threshold approach." << endl;
    }

    void updateAllLights() {
        for (Intersection* intersection : intersections) {
            if (intersection) {
                intersection->updateLight();
            }
        }
    }

    void printNetwork() {
        int intersectionCount = 0;
        for (Intersection* i : intersections) {
            if (i) intersectionCount++;
        }

        int roadCount = 0;
        for (Road* r : roads) {
            if (r) roadCount++;
        }

        cout << "City Network: " << intersectionCount << " intersections, " 
                  << roadCount << " roads" << endl;
        
        for (Intersection* i : intersections) {
            if (!i) continue;
            
            cout << "Intersection " << i->id << " (" << i->name << ")";
            if (i->hasTrafficLight) {
                cout << " Traffic Light: " << (i->isGreenLight ? "GREEN" : "RED");
            }
            cout << endl;
            
            vector<Road*> outRoads = i->outgoingRoads;
            sort(outRoads.begin(), outRoads.end(), 
                [](Road* a, Road* b) { return a->destination->id < b->destination->id; });
            
            cout << "  Connected to intersections: ";
            for (size_t j = 0; j < outRoads.size(); j++) {
                cout << outRoads[j]->destination->id;
                if (j < outRoads.size() - 1) cout << ", ";
            }
            cout << endl;
            
            for (Road* road : outRoads) {
                cout << "  → Road to " << road->destination->id << endl;
            }
        }
    }
    
    int getIntersectionCount() {
        int count = 0;
        for (Intersection* i : intersections) {
            if (i) count++;
        }
        return count;
    }
};

// Traffic Simulation
class TrafficSimulation {
public:
    CityGraph cityGraph;
    vector<Vehicle*> vehicles;
    int timeStep;
    int totalVehiclesArrived;

    TrafficSimulation() {
        timeStep = 0;
        totalVehiclesArrived = 0;
    }

    void setupCity(int gridSize) {
        createDefaultNetwork(gridSize);
    }
    
    int getRandomIntersectionId() {
        int count = cityGraph.getIntersectionCount();
        if (count == 0) return -1;
        return rand() % count;
    }

    void listAllVehicles() {
        cout << "\n=== All Vehicles ===" << endl;
        if (vehicles.empty()) {
            cout << "No vehicles have been added." << endl;
            return;
        }
        
        cout << "Total vehicles: " << vehicles.size() << endl;
        for (int i = 0; i < vehicles.size(); i++) {
            Vehicle* v = vehicles[i];
            cout << "Vehicle ID: " << v->id 
                 << " | From: " << v->sourceId 
                 << " | To: " << v->destId 
                 << " | " << (v->arrived ? "Arrived" : "In transit") << endl;
        }
    }

    bool addVehicle(int sourceId, int destId) {
        Intersection* source = cityGraph.getIntersection(sourceId);
        Intersection* dest = cityGraph.getIntersection(destId);
        
        if (source && dest && source != dest) {
            int id = vehicles.size();
            Vehicle* vehicle = new Vehicle(id, source, dest);
            
            vector<Road*> path = cityGraph.findShortestPath(sourceId, destId);
            
            if (!path.empty()) {
                vehicle->setRoute(path);
                vehicles.push_back(vehicle);
                
                cout << "Added vehicle " << id << " from " << sourceId << " to " << destId 
                     << " (path: " << path.size() << " roads)" << endl;
                cout << "Path: ";
                for (int i = 0; i < path.size(); i++) {
                    cout << path[i]->source->id;
                    if (i < path.size() - 1) cout << " -> ";
                }
                cout << " -> " << path.back()->destination->id << endl;
                return true;
            } else {
                delete vehicle;
                cout << "No path found from " << sourceId << " to " << destId << endl;
                return false;
            }
        }
        cout << "Invalid source or destination intersection" << endl;
        return false;
    }
    
    void addRandomVehicles(int count) {
        cout << "Adding " << count << " random vehicles..." << endl;
        
        int added = 0;
        while (added < count) {
            int source = getRandomIntersectionId();
            int dest = getRandomIntersectionId();
            
            if (source != dest && source >= 0 && dest >= 0) {
                if (addVehicle(source, dest)) {
                    added++;
                }
            }
        }
        
        cout << "Added " << added << " random vehicles." << endl;
    }

    void step() {
        cityGraph.updateAllLights();
        
        for (Vehicle* vehicle : vehicles) {
            if (!vehicle->arrived) {
                vehicle->move(timeStep);
                if (vehicle->arrived) {
                    totalVehiclesArrived++;
                }
            }
        }
        
        if (timeStep > 0 && timeStep % 10 == 0) {
            cityGraph.optimizeTrafficLights();
        }
        
        timeStep++;
    }

    void runSimulation(int steps) {
        if (vehicles.empty()) {
            cout << "No vehicles added. Please add vehicles first using Option 2 in the main menu." << endl;
            return;
        }

        cout << "=== Run Simulation ===" << endl;
        cout << "Current total time steps before simulation: " << timeStep << endl;
        cout << "Starting simulation for " << steps << " time steps..." << endl;
        
        for (int i = 0; i < steps; i++) {
            step();
            
            if (steps > 10 && i % (steps / 10) == 0) {
                cout << "Simulation " << (i * 100 / steps) << "% complete..." << endl;
            }
        }
        
        displayStatistics();
    }

    void printCityMap() {
        cityGraph.printNetwork();
        
        cout << "\n--- Vehicle Positions ---" << endl;
        
        unordered_map<int, vector<int>> vehiclesAtIntersection;
        
        for (int i = 0; i < vehicles.size(); i++) {
            Vehicle* v = vehicles[i];
            if (!v->arrived) {
                int intersectionId = v->current->id;
                vehiclesAtIntersection[intersectionId].push_back(v->id);
            }
        }
        
        if (vehiclesAtIntersection.empty()) {
            cout << "No active vehicles on the map." << endl;
        } else {
            cout << "Active vehicles at intersections:" << endl;
            for (const auto& entry : vehiclesAtIntersection) {
                cout << "Intersection " << entry.first << ": ";
                cout << "Vehicles [";
                for (size_t i = 0; i < entry.second.size(); i++) {
                    cout << entry.second[i];
                    if (i < entry.second.size() - 1) cout << ", ";
                }
                cout << "]" << endl;
            }
            
            int arrivedCount = 0;
            for (Vehicle* v : vehicles) {
                if (v->arrived) arrivedCount++;
            }
            
            if (arrivedCount > 0) {
                cout << arrivedCount << " vehicles have reached their destinations." << endl;
            }
        }
    }
    
    void findAllRoutes(int startId, int endId, int maxLength) {
        cityGraph.findAllRoutes(startId, endId, maxLength);
    }

    void displayStatistics() {
        cout << "\n=== Simulation Statistics ===" << endl;
        cout << "Total simulation time elapsed: " << timeStep << " time steps" << endl;
        cout << "Vehicles: " << vehicles.size() << " total, " 
              << totalVehiclesArrived << " arrived" << endl;
        
        if (!vehicles.empty()) {
            double arrivalRate = static_cast<double>(totalVehiclesArrived) / vehicles.size() * 100.0;
            cout << "Arrival rate: " << arrivalRate << "%" << endl;
        }
        
        cout << "\n=== Vehicle Arrivals ===" << endl;
        
        vector<Vehicle*> sortedVehicles = vehicles;
        sort(sortedVehicles.begin(), sortedVehicles.end(), 
            [](Vehicle* a, Vehicle* b) { return a->id < b->id; });
        
        int shownVehicles = 0;
        for (Vehicle* v : sortedVehicles) {
            if (v->arrived) {
                cout << "Vehicle " << v->id 
                     << " arrived at time step " << v->arrivalTime
                     << "! From " << v->sourceId 
                     << " to " << v->destId
                     << " (path: " << v->route.size() << " roads)" << endl;
                shownVehicles++;
            }
        }
        
        cout << "Total vehicles displayed: " << shownVehicles << " of " << totalVehiclesArrived << endl;
    }

private:
    void createDefaultNetwork(int gridSize = 3) {
        int totalIntersections = gridSize * gridSize;
        for (int i = 0; i < totalIntersections; i++) {
            bool hasLight = (i % 2 == 0);
            cityGraph.addIntersection(i, "Intersection_" + to_string(i), hasLight);
        }

        int roadId = 0;
        
        // Horizontal roads
        for (int row = 0; row < gridSize; row++) {
            for (int col = 0; col < gridSize - 1; col++) {
                int sourceId = row * gridSize + col;
                int destId = row * gridSize + col + 1;
                
                cityGraph.addRoad(roadId++, sourceId, destId);
                cityGraph.addRoad(roadId++, destId, sourceId);
            }
        }
        
        // Vertical roads
        for (int col = 0; col < gridSize; col++) {
            for (int row = 0; row < gridSize - 1; row++) {
                int sourceId = row * gridSize + col;
                int destId = (row + 1) * gridSize + col;
                
                cityGraph.addRoad(roadId++, sourceId, destId);
                cityGraph.addRoad(roadId++, destId, sourceId);
            }
        }
        
        cout << "Created a " << gridSize << "x" << gridSize << " grid city with " << roadId << " roads." << endl;
    }
};

// Function to clear console screen
void clearScreen() {
    system("cls");
}

// Function to get integer input with validation
int getIntInput(const string& prompt, int minValue = INT_MIN, int maxValue = INT_MAX) {
    int value;
    bool validInput = false;
    
    do {
        cout << prompt;
        string input;
        getline(cin, input);
        
        try {
            value = stoi(input);
            if (value >= minValue && value <= maxValue) {
                validInput = true;
            } else {
                cout << "Invalid input. Please enter a number between " << minValue << " and " << maxValue << "." << endl;
            }
        } catch (...) {
            cout << "Invalid input. Please enter a number." << endl;
        }
    } while (!validInput);
    
    return value;
}

// Function to display the main menu
void displayMainMenu(int currentTimeStep = 0) {
    cout << "\n=== SMART TRAFFIC MANAGEMENT SYSTEM ===" << endl;
    if (currentTimeStep > 0) {
        cout << "Current simulation time: Step " << currentTimeStep << endl;
    }
    cout << "1. Setup City Grid" << endl;
    cout << "2. Add Vehicles             (requires city setup first)" << endl;
    cout << "3. Run Simulation           (requires city setup first, vehicles)" << endl;
    cout << "4. Find All Routes          (requires city setup first)" << endl;
    cout << "5. Display City Map" << endl;
    cout << "6. Display Statistics       (requires simulation run first)" << endl;
    cout << "7. Show Current Time Step   (requires simulation run first)" << endl;
    cout << "8. Exit" << endl;
    cout << "\nNote: Traffic lights are automatically optimized every 10 time steps during simulation." << endl;
    cout << "Enter your choice: ";
}

// Main function with interactive menu
int main() {
    srand(static_cast<unsigned int>(time(nullptr)));
    
    TrafficSimulation simulation;
    bool citySetup = false;
    bool running = true;
    
    clearScreen();
    cout << "\n=== SMART TRAFFIC MANAGEMENT SYSTEM ===" << endl;
    cout << "\nFeatures:" << endl;
    cout << "- Graph representation for the road network" << endl;
    cout << "- Dijkstra's algorithm for shortest path calculation" << endl;
    cout << "- Adaptive traffic light scheduling" << endl;
    cout << "- Real-time traffic simulation" << endl;
    cout << "\nPress Enter to continue...";
    cin.get();
    clearScreen();
    
    while (running) {
        displayMainMenu(simulation.timeStep);
        
        int choice = getIntInput("", 1, 8);
        
        switch (choice) {
            case 1: { // Setup City Grid
                clearScreen();
                cout << "=== Setup City Grid ===" << endl;
                int gridSize = getIntInput("Enter grid size (2-10): ", 2, 10);
                simulation.setupCity(gridSize);
                citySetup = true;
                cout << "City grid setup complete!" << endl;
                break;
            }
            
            case 2: { // Add Vehicles
                clearScreen();
                if (!citySetup) {
                    cout << "Please set up the city first (Option 1)." << endl;
                    break;
                }
                
                cout << "=== Add Vehicles ===" << endl;
                cout << "1. Add a specific vehicle" << endl;
                cout << "2. Add random vehicles" << endl;
                cout << "3. Back to main menu" << endl;
                
                int vehicleChoice = getIntInput("Enter your choice: ", 1, 3);
                
                if (vehicleChoice == 1) {
                    int sourceId = getIntInput("Enter source intersection ID: ", 0, INT_MAX);
                    int destId = getIntInput("Enter destination intersection ID: ", 0, INT_MAX);
                    simulation.addVehicle(sourceId, destId);
                } else if (vehicleChoice == 2) {
                    int count = getIntInput("Enter number of random vehicles to add: ", 1, 100);
                    simulation.addRandomVehicles(count);
                }
                break;
            }
            
            case 3: { // Run Simulation
                clearScreen();
                if (!citySetup) {
                    cout << "Please set up the city first (Option 1)." << endl;
                    break;
                }
                
                if (simulation.vehicles.size() == 0) {
                    cout << "Please add some vehicles first (Option 2)." << endl;
                    break;
                }
                
                int steps = getIntInput("Enter number of time steps to simulate (1-1000): ", 1, 1000);
                simulation.runSimulation(steps);
                break;
            }
            
            case 4: { // Find All Routes
                clearScreen();
                if (!citySetup) {
                    cout << "Please set up the city first (Option 1)." << endl;
                    break;
                }
                
                cout << "=== Find All Routes ===" << endl;
                int startId = getIntInput("Enter start intersection ID: ", 0, INT_MAX);
                int endId = getIntInput("Enter destination intersection ID: ", 0, INT_MAX);
                
                if (startId == endId) {
                    cout << "Start and destination cannot be the same." << endl;
                    break;
                }
                
                int maxLength = getIntInput("Enter maximum route length: ", 1, INT_MAX);
                
                cout << "\nFinding all routes from " << startId << " to " << endId 
                     << " with maximum length " << maxLength << "..." << endl;
                simulation.findAllRoutes(startId, endId, maxLength);
                break;
            }
            
            case 5: { // Display City Map
                clearScreen();
                cout << "=== City Map ===" << endl;
                simulation.printCityMap();
                break;
            }
            
            case 6: { // Display Statistics
                clearScreen();
                if (!citySetup) {
                    cout << "Please set up the city first (Option 1)." << endl;
                    break;
                }
                
                cout << "=== Traffic Statistics ===" << endl;
                
                if (simulation.timeStep == 0) {
                    cout << "No simulation has been run yet. Please run a simulation first (Option 3)." << endl;
                    break;
                }
                
                simulation.displayStatistics();
                break;
            }
            
            case 7: { // Show Current Time Step
                clearScreen();
                cout << "=== Current Simulation Time ===" << endl;
                int currentStep = simulation.timeStep;
                if (currentStep == 0) {
                    cout << "No simulation has been run yet. Please run a simulation first (Option 3)." << endl;
                } else {
                    cout << "Current simulation time: Step " << currentStep << endl;
                    cout << "Total time elapsed: " << currentStep << " time steps" << endl;
                }
                break;
            }
            
            case 8: { // Exit
                cout << "Thank you for using the Smart Traffic Management System. Goodbye!" << endl;
                running = false;
                break;
            }
        }
        
        if (running) {
            cout << "\nPress Enter to continue...";
            cin.get();
            clearScreen();
        }
    }
    
    return 0;
} 