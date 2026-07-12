#include <memory>
#include <string>
#include <iostream>

#include "peridetic.h" // coordinate conversion lib
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geographic_msgs/msg/geo_point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "spring_planner_msgs/msg/hint_map.hpp" //have to write these
#include "spring_planner_msgs/msg/entity.hpp" //have to write these
#include "global_frame/msg/global_detection.hpp"
#include "global_frame/msg/global_detection_array.hpp"

/* the data sructure is a 2d array with 3 layers
 * the top two layers are the x and y position, 
 * the third layer is whether a node is fixed or not 
 * have it be either zero, or 1 for the boarder, or a pointer
 * also need a linked list with the buoys in there 
 * then you can publish the list of where you think the 
 * bouys are but lowkey prob just publish next waypoint
 * to mavros directly 
 * You also need to add the searching for stuff when 
 * you're near by, and also add the inter-run validation
 * Probably store a copy of the original map guess
 * We'll also have to do some stuff to make this inter-
 * operate with left right bouy and docking or wtv */

class SpringPlanner : public rclcpp::Node
{
public:
  SpringPlanner() : Node("spring-planner-node")
  {
    // this->frequency_ = this->get_parameter("frequency").as_int();
    waypoints.reserve(100);

    setpoint_position_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
    map_sub = this->create_subscription<spring_planner_msgs::msg::HintMap>("map_hint", rclcpp::SensorDataQoS(), std::bind(&SpringPlanner::load_map, this, std::placeholders::_1));
    detect_sub = this->create_subscription<global_frame::msg::GlobalDetectionArray>("global_detections", rclcpp::SensorDataQoS(), std::bind(&SpringPlanner::update_map, this, std::placeholders::_1));
    global_origin_sub = this->create_subscription<geographic_msgs::msg::GeoPointStamped>("mavros/global_position/gp_origin", rclcpp::SensorDataQoS(), std::bind(&SpringPlanner::gp_origin_callback, this, std::placeholders::_1) );
    local_position_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("mavros/local_position/pose", rclcpp::SensorDataQoS(), std::bind(&SpringPlanner::local_position_callback, this, std::placeholders::_1) );
//    delay_timer_ = this->create_wall_timer(1s, [this]() { map_update(); });
  }

private:
  void gp_origin_callback(const geographic_msgs::msg::GeoPointStamped::SharedPtr msg){
	origin_lat = msg->position.latitude;
	origin_long = msg->position.longitude;
	origin_alt = msg->position.altitude;
  }
  // load in the original map
  void load_map(const spring_planner_msgs::msg::HintMap::SharedPtr msg) {
	/* perform the frame conversion from the latitude top left to local frame https://github.com/mavlink/mavros/blob/ros2/mavros/src/lib/ftf_frame_conversions.cpp */
	  // we use the peridetic library to simplify the conversion from ellipsoid coordinates
	// we get coordinates in local frame called x0 (east) and y0 (north)
	
	if (origin_alt == 0) { // If you have no origin you can't load a map becuase you can't do coordinate conversion
		// TODO emit an error
		return;
	}
		
	// convert the origin and the fixed ne corner on the map to ECEF
	static constexpr double DEG_TO_RAD = (M_PI / 180.0);

	peri::LPA const origin_LPA{ origin_long * DEG_TO_RAD, origin_lat * DEG_TO_RAD, origin_alt * DEG_TO_RAD };
	peri::XYZ const origin_ECEF{ peri::xyzForLpa(origin_LPA) };

	peri::LPA const corner_LPA{ msg->ne_corner[1] * DEG_TO_RAD, msg->ne_corner[0] * DEG_TO_RAD, msg->ne_corner[2] * DEG_TO_RAD };
	peri::XYZ const corner_ECEF{ peri::xyzForLpa(corner_LPA) };

	// convert the fixed ne corner to ENU relative to origin 
	
	Eigen::Vector3d corner_vec = Eigen::Vector3d(corner_ECEF);
	const double sin_lat = std::sin(origin_long * DEG_TO_RAD);
	const double sin_lon = std::sin(origin_long * DEG_TO_RAD);
	const double cos_lat = std::cos(origin_lat * DEG_TO_RAD);
	const double cos_lon = std::cos(origin_lat * DEG_TO_RAD);
	Eigen::Matrix3d R;
	  R <<  -sin_lon, cos_lon, 0.0,
	        -cos_lon * sin_lat, -sin_lon * sin_lat, cos_lat,
		    cos_lon * cos_lat, sin_lon * cos_lat, sin_lat;
	corner_vec = R * corner_vec;

	float x0 = corner_vec.x();
	float y0 = corner_vec.y();

	// delete any old map, if this is the first time ew_dim will be 0 and grid will be NULL 
	for (int i=0; i < ew_dim; i++){
		for (int j=0; j < ns_dim; j++){
			delete [] grid[i][j];
		}
		delete [] grid[i];
	}
	delete [] grid; 

	ew_dim = msg->ew_dim;
	ns_dim = msg->ns_dim;
	grid = new float**[ew_dim];
	for (int i=0; i < ew_dim; i++){
		grid[i] = new float*[ns_dim];
		for (int j=0; j < ns_dim; j++){
			grid[i][j] = new float[4];
		}
	}
	for (int i=0; i < ns_dim; i++){
		for (int j=0; j < ew_dim; j++){
			grid[i][j][0] = x0 + i*msg->grid_spacing;
			grid[i][j][1] = y0 + j*msg->grid_spacing;
			grid[i][j][2] = -1; // entity class if any
			grid[i][j][3] = (i == 0 || j == 0) ? 	1 : 0; // 1 for pinned or 0 for free. The map edges are always pinned
		}
	}

	// add the entities
	for (i = 0; i < msg->entities->size(); i++){
		j = msg->entities[i]->position[0];
		k = msg->entities[i]->position[1];
		grid[j][k][2] = msg->entities[i]->type;
		int* ent = new int(2);
		ent[0] = msg->entities[i]->position[0];
		ent[1] = msg->entities[i]->position[1];
		entityMap.insert(ent);
		int id = msg->entities[i]->id;
		entityIds.insert(id)
		if (id > 9900){
			waypoints[9999-id] = ent
		}
	}
  }

  // Get the new detection info, match stuff up, balance the grid, and publish the new map
  void update_map(const global_frame::msg::GlobalDetectionArray::SharedPtr msg) {
	  for (const auto& detection : *msg){
		auto ent = entityIds.find(detection->id);
		if (ent == entityIds.end()){
			// TODO update our grid with the most recent position
			continue;
		} else {
			// attach the new entity to it's nearest neighbour in our grid
			int coords[2] = {detection->x, detection->y};
			int neighbour[2] = {0,0};
			nearest_free(coords, neighbour);
			if (neighbour[0] == 0 && neighbour[1] ==1) {
				// TODO emit an error message
				break;
			} else {
				map[neighbour[0]][neighbour[1]][0] = detection->x;
				map[neighbour[0]][neighbour[1]][] = detection->y;
				map[neighbour[0]][neighbour[1]][] = detection->type;
				map[neighbour[0]][neighbour[1]][] = 1;
				entityIds.insert(detection->id);
				arr = new int(6);
				arr[0] = neighbour[0];
				arr[1] = neighbour[1];
				entityMap.insert(arr);
			}
		}
		balance();
		align();
	  }
	  // this message is an array of global_detection messages
	// don't add anything more than 10 metres away, trust the map
	// update things when you move closer to them
	// de-dup by id -- need to collaboarte with mapping on this, on all of this lowkey
	// add anything new by nearest free neighbour
	// balence 
	// align
  }

  void algin(){
	for (const auto& x : entityMap){
		original_energy = energy(map);
		int neighbour[2];
		neighbour[0] = x[0];
		neighbour[1] = x[1];
		nearest(x, neighbour);
		if (x[0] == neighbour[0] && x[1] == neighbour[1]){ // no other entities of the same type
			continue;
		}
		int dupx = map[neighbour[0]][neighbour[1]][0];
		int dupy = map[neighbour[0]][neighbour[1]][1];
		map[neighbour[0]][neighbour[1]][0] = map[x[0]][x[1]][0];
		map[neighbour[0]][neighbour[1]][1] = map[x[0]][x[1]][1];
		map[x[0]][x[1]][0] = dupx;
		map[x[0]][x[1]][1] = dupy;
		balance();
		if (energy() < original_energy) {
			align();
			break;
		} else {
			int dupx = map[neighbour[0]][neighbour[1]][0];
			int dupy = map[neighbour[0]][neighbour[1]][1];
			map[neighbour[0]][neighbour[1]][0] = map[x[0]][x[1]][0];
			map[neighbour[0]][neighbour[1]][1] = map[x[0]][x[1]][1];
			map[x[0]][x[1]][0] = dupx;
			map[x[0]][x[1]][1] = dupy;
			balance();
		}
	}
}

  /* let a grid get to it's lowest energy state */
  void balance() {
	E = energy();
	for (int i = 0; i < 100; i++){
		for (j=1; j < ew_dim - 1; j++){
			for (k=1; k < ns_dim - 1; k++){
				if (map[i][j][3] == 1)
					continue;
				map[i][j][0] += 1/2 * ( map[i-1][j][0] - map[i][j][0] 
						+ map[i+1][j][0] - map[i][j][0] 
						+ map[i][j+1][0] - map[i][j][0]
						+ map[i][j-1][0] - map[i][j][0]);
				map[i][j][1] += 1/2 * ( map[i-1][j][1] - map[i][j][1] 
						+ map[i+1][j][1] - map[i][j][1] 
						+ map[i][j+1][1] - map[i][j][1]
						+ map[i][j-1][1] - map[i][j][1]);
			}
		}
		if (abs(E - energy()) < 0.0001)
			break;
		E = energy();
	}
  }
  /* get the energy in a grid */ 
int energy(){
	int E = 0;
	for (int i=0; i < xwidth-1; i++){
		for (int j=0; j < ywidth-1; j++){
			int a[2] = {i,j};
			int b[2] = {i+1,j};
			int c[2] = {i,j+1};
			E += dist(a,b)*dist(a,b);
			E += dist(a,c)*dist(a,c);
		}
	}
}

// You pass this function the coordinates of the point you want to get neighours of 
// and another array containing the same coordinates
// It will place the coordinates of the nearest neighbour in that array
// Or will leave the original coordinates in that array if there are no entities 
// of the same type that have been seen yet
void nearest(int coords[2], int nearest[2]){
	if (grid[coords[0]][coords[1]][3] == 0)
		throw runtime_error("Tried to find nearest neighbours of a location with no entity");
	int max_dist = 0;
	nearest[0] = coords[0];
	nearest[1] = coords[1];
	for (const auto& x : entityMap){
		if (x[0] == coords[0] && x[1] == coords[1]) // don't count the distance to yourself
			continue;
		else if (max_dist == 0)
			nearest[0] = x[0];
			nearest[1] = x[1];
			max_dist = dist(coords, x);
		else if (dist(coords, x) < max_dist){
			max_dist = dist(coords, x);
			nearest[0] = x[0];
			nearest[1] = x[1];
		}
    }
}

void nearest_free(int coords[2], int nearest[2]){
	int max_dist = 0;
	for (const auto& x : entityMap){
		if (x[0] == coords[0] && x[1] == coords[1]) // don't count the distance to yourself
			continue;
		int arr[2] = {map[x[0]][x[1]][0] , map[x[0]][x[1]][1]};
		d = sqrt( (arr[0]-coords[0])*(arr[0]-coords[0]) + (arr[1]-coords[1])*(arr[1]-coords[1]) );
		if (max_dist == 0 || d < max_dist){
			nearest[0] = arr[0];
			nearest[1] = arr[1];
			max_dist = d;
		}
    }
}

float dist(int x[2], int y[2]){
	a = grid[x[0]][x[1]][0];
	b = grid[x[0]][x[1]][1];
	c = grid[y[0]][y[1]][0];
	d = grid[y[0]][y[1]][1];
	return sqrt((a-c)*(a-c) + (b-d)*(b-d))
}

void local_position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
	if (waypoints.empty()) {
		// there are no waypoints so do nothing
		return;
	}	
	int way[2], pos[2];
	int* w = waypoints.back();
	way[0] = grid[w[0]][w[1]][0];
	way[1] = grid[w[0]][w[1]][1];
	pos[0] = msg->pose->position->x;
	pos[1] = msg->pose->position->y;
	if (pow(way[0]-pos[0], 2) + pow(way[1]-pos[1], 2) < 0.71) {
		// advance to the next waypoint
		waypoints.pop_back();
	}
	// publish the location of the next waypoint
	auto setpoint = geometry_msgs::msg::PoseStamped();
	w = waypoints.back();
	setpoint->pose->position->x = grid[w[0]][w[1]][0];
	setpoint->pose->position->y = grid[w[0]][w[1]][1];
	setpoint_position_pub->publish(setpoint);
}

  // Pubs and subs
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_position_pub;
  rclcpp::Subscription<spring_planner_msgs::msg::HintMap>::SharedPtr map_sub;
  rclcpp::Subscription<global_frame::msg::GlobalDetectionArray>::SharedPtr detect_sub;
  rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr global_origin_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_sub;
  //rclcpp::TimerBase::SharedPtr delay_timer_;
  // Variables
  float origin_lat = 0;
  float origin_long = 0;
  float origin_alt = 0;
  float*** grid = NULL;
  int ew_dim = 0;
  int ns_dim = 0;
  std::unordered_set<int*> entityMap;
  std::unordered_set<int> entityIds;
  std::vector<int*> waypoints; // ordered list of waypoints with the highest index the next waypoint to visit
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpringPlanner>());
  rclcpp::shutdown();
  return 0;
}

