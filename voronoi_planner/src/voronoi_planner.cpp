#include <voronoi_planner/voronoi_planner.hpp>

void VoronoiPlanner::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{ 
  occ_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("voronoi_planner/occ_map", 10, true);
  dist_occ_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("voronoi_planner/dist_map", 10, true);
  voro_occ_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("voronoi_planner/voronoi_map", 10, true);
  
  /* Publishers */

  front_end_publisher_map_["front_end/closed_list"] = nh.advertise<visualization_msgs::Marker>("closed_list_viz", 10, true);

  front_end_plan_viz_pub_ = nh.advertise<visualization_msgs::Marker>("plan_viz", 5, true);
  start_pt_pub_ = nh.advertise<visualization_msgs::Marker>("start_point", 5, true);
  goal_pt_pub_ = nh.advertise<visualization_msgs::Marker>("goal_point", 5, true);

  /* Subscribers */
  start_sub_ = nh.subscribe("/start_pt", 5, &VoronoiPlanner::startPointCB, this);
  goal_sub_ = nh.subscribe("/goal_pt", 5, &VoronoiPlanner::goalPointCB, this);

  initParams(pnh);

  pgmFileToBoolMap(&bool_map_, size_x_, size_y_, map_fname_);

  // DynamicVoronoi::DynamicVoronoiParams dyn_voro_params;
  // dyn_voro_params.height = 0.0;
  // dyn_voro_params.resolution = res_;
  // dyn_voro_params.origin_x = 0.0;
  // dyn_voro_params.origin_y = 0.0;

  tm_voronoi_map_init_.start();

  dyn_voro_ = std::make_shared<DynamicVoronoi>();

  dyn_voro_->initializeMap(size_x_, size_y_, bool_map_);
  dyn_voro_->update(); // update distance map and Voronoi diagram
  // dyn_voro_->visualize("/home/john/gestelt_ws/src/gestelt/voronoi_planner/maps/final.ppm");

  tm_voronoi_map_init_.stop(verbose_planning_);

  // if (doPrune){
  //   dyn_voro.prune();  // prune the Voronoi
  // }
  // else if (doPruneAlternative) { // prune the Voronoi
  //   dyn_voro.updateAlternativePrunedDiagram();  
  // }

  // Set start and goal
  DblPoint start_pos(7.0, 8.0);
  DblPoint goal_pos(7.0, 9.0);


  AStarPlanner::AStarParams astar_params_; 
  astar_params_.max_iterations = 99999;
  astar_params_.debug_viz = true;
  astar_params_.tie_breaker = 1.00001;
  astar_params_.cost_function_type  = 2;

  front_end_planner_ = std::make_unique<AStarPlanner>(dyn_voro_, astar_params_);

  front_end_planner_->addPublishers(front_end_publisher_map_);

  occmapToOccGrid(*dyn_voro_, size_x_, size_y_,  occ_grid_); // Occupancy map
  voronoimapToOccGrid(*dyn_voro_, size_x_, size_y_,  voro_occ_grid_); // Voronoi map

  voro_occ_grid_pub_.publish(voro_occ_grid_);
  occ_map_pub_.publish( occ_grid_);
}

void VoronoiPlanner::initParams(ros::NodeHandle &pnh)
{
  pnh.param("map_filename", map_fname_, std::string(""));
  pnh.param("resolution", res_, 0.1);
  pnh.param("negate", negate_, false);
  pnh.param("occ_threshold", occ_th_, 100.0);
  pnh.param("free_threshold", free_th_, 0.0);
  pnh.param("origin_x", origin_x_, 0.0);
  pnh.param("origin_y", origin_y_, 0.0);
  pnh.param("yaw", yaw_, 0.0);
}

void VoronoiPlanner::realignBoolMap(bool ***map, bool ***map_og, int& size_x, int& size_y)
{
  for (int x=0; x<size_x; x++) {
    (*map)[x] = new bool[size_y];
  }

  for(int j = 0; j < size_y; j++)
  {
    for (int i = 0; i < size_x; i++)
    {
      (*map)[i][j] = (*map_og)[i][size_y-j-1];
    }
  }
  
}

void VoronoiPlanner::pgmFileToBoolMap(bool ***map,
                                      int& size_x, int& size_y,
                                      const std::string& fname)
{
  SDL_Surface* img;

  unsigned char* pixels;
  unsigned char* p;
  int rowstride, n_channels;
  unsigned int i,j;
  int k;
  double occ;
  int color_sum;
  double color_avg;

  // Load the image using SDL.  If we get NULL back, the image load failed.
  if(!(img = IMG_Load(fname.c_str())))
  {
    std::string errmsg = std::string("failed to open image file \"") + 
            fname + std::string("\"");
    throw std::runtime_error(errmsg);
  }

  // Copy the image data into the map structure
  size_x = img->w;
  size_y = img->h;

  *map = new bool*[size_x];
  for (int x=0; x<size_x; x++) {
    (*map)[x] = new bool[size_y];
  }

  // Allocate space to hold the data
  occ_idx_.clear();
  free_idx_.clear();

  // Get values that we'll need to iterate through the pixels
  rowstride = img->pitch;
  n_channels = img->format->BytesPerPixel;

  // Copy pixel data into the map structure
  pixels = (unsigned char*)(img->pixels);
  for(int j = 0; j < size_y; j++)
  {
    for (int i = 0; i < size_x; i++)
    {
      // Compute mean of RGB for this pixel
      p = pixels + j*rowstride + i*n_channels;
      color_sum = 0;
      for(k=0;k<n_channels;k++)
        color_sum += *(p + (k));
      color_avg = color_sum / (double)n_channels;

      // If negate is true, we consider blacker pixels free, and whiter
      // pixels free.  Otherwise, it's vice versa.
      if(negate_)
        occ = color_avg / 255.0;
      else
        occ = (255 - color_avg) / 255.0;
      
      // Apply thresholds to RGB means to determine occupancy values for
      // map.  Note that we invert the graphics-ordering of the pixels to
      // produce a map with cell (0,0) in the lower-left corner.

      if(occ > occ_th_){  // Occupied
        (*map)[i][j] = true;
      }
      else if(occ < free_th_){ // Free
        (*map)[i][j] = false;
      }
      else{ // Unknown
        (*map)[i][j] = true;
      }


    }
  }

  SDL_FreeSurface(img);
}


void VoronoiPlanner::loadMapFromFile(nav_msgs::OccupancyGrid& map,
                                      const std::string& fname)
{
  SDL_Surface* img;

  unsigned char* pixels;
  unsigned char* p;
  int rowstride, n_channels;
  unsigned int i,j;
  int k;
  double occ;
  int color_sum;
  double color_avg;

  // Load the image using SDL.  If we get NULL back, the image load failed.
  if(!(img = IMG_Load(fname.c_str())))
  {
    std::string errmsg = std::string("failed to open image file \"") + 
            fname + std::string("\"");
    throw std::runtime_error(errmsg);
  }

  // Copy the image data into the map structure
  map.header.stamp = ros::Time::now();
  map.header.frame_id = "map";
  map.info.width = img->w;
  map.info.height = img->h;
  map.info.resolution = res_;
  map.info.origin.position.x = origin_x_;
  map.info.origin.position.y = origin_y_;
  map.info.origin.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_);
  map.info.origin.orientation.x = q.x();
  map.info.origin.orientation.y = q.y();
  map.info.origin.orientation.z = q.z();
  map.info.origin.orientation.w = q.w();

  // Allocate space to hold the data
  map.data.resize(map.info.width * map.info.height);
  occ_idx_.clear();
  free_idx_.clear();

  // Get values that we'll need to iterate through the pixels
  rowstride = img->pitch;
  n_channels = img->format->BytesPerPixel;

  // Copy pixel data into the map structure
  pixels = (unsigned char*)(img->pixels);
  for(int j = 0; j < map.info.height; j++)
  {
    for (int i = 0; i < map.info.width; i++)
    {
      // Compute mean of RGB for this pixel
      p = pixels + j*rowstride + i*n_channels;
      color_sum = 0;
      for(k=0;k<n_channels;k++)
        color_sum += *(p + (k));
      color_avg = color_sum / (double)n_channels;

      // If negate is true, we consider blacker pixels free, and whiter
      // pixels free.  Otherwise, it's vice versa.
      if(negate_)
        occ = color_avg / 255.0;
      else
        occ = (255 - color_avg) / 255.0;
      
      // Apply thresholds to RGB means to determine occupancy values for
      // map.  Note that we invert the graphics-ordering of the pixels to
      // produce a map with cell (0,0) in the lower-left corner.
      size_t idx = map2Dto1DIdx(map.info.width, i, map.info.height - j - 1);

      if(occ > occ_th_){  // Occupied
        map.data[idx] = cost_val::OCC;
        occ_idx_.push_back(idx);
      }
      else if(occ < free_th_){ // Free
        map.data[idx] = cost_val::FREE;
        free_idx_.push_back(idx);
      }
      else{ // Unknown
        map.data[idx] = cost_val::UNKNOWN;
        unknown_idx_.push_back(idx);
      }
    }
  }

  SDL_FreeSurface(img);
}

// void VoronoiPlanner::computeDistanceMap()
// {
//   dist_map_.resize(occ_grid_.info.width * occ_grid_.info.height, -1);
//   obst_map_.resize(occ_grid_.info.width * occ_grid_.info.height, -1); 

//   open_queue_.clear();

//   for(int j = 0; j < occ_grid_.info.height; j++)
//   {
//     for (int i = 0; i < occ_grid_.info.width; i++)
//     {
//       const size_t idx = map2Dto1DIdx(occ_grid_.info.width, i, occ_grid_.info.height - j - 1);
//       const int8_t s = occ_grid_.data[idx];

//       if (s == cost_val::OCC){ // Occupied
//         obst_map_[idx] = idx; 
//         dist_map_[idx] = 0; 

//         open_queue_.put(idx, 0); // add to open queue
//       }
//       else if (s == cost_val::UNKNOWN){ // Unknown
//          dist_map_[idx] = 0; 
//       }
//       else if (s == cost_val::FREE){ // Free
//          dist_map_[idx] = INF; 
//       }
//       else{
//          dist_map_[idx] = INF; 
//       }
//     }
//   }

//   while (!open_queue_.empty()){
//     const size_t idx = open_queue_.get();
//     lowerStatic(idx);
//   }
// }


void VoronoiPlanner::lowerStatic(const size_t& idx)
{
  for (const size_t& nb_idx: get8ConNeighbours(idx))
  { 
    const double d = getL2Norm(obst_map_[idx], nb_idx); //d: distance from nearest obstacle on idx to neighbor
    if (d < dist_map_[nb_idx]) {
      dist_map_[nb_idx] = d;
      obst_map_[nb_idx] = obst_map_[idx];
      open_queue_.put(nb_idx, d); // add to open queue
    }
  }
}

void VoronoiPlanner::computeDistanceMap()
{
  dist_map_.resize(occ_grid_.info.width * occ_grid_.info.height, -1);
  obst_map_.resize(occ_grid_.info.width * occ_grid_.info.height, -1); 
  voro_map_.resize(occ_grid_.info.width * occ_grid_.info.height, false); 
  to_raise_.resize(occ_grid_.info.width * occ_grid_.info.height, false); 

  open_queue_.clear();

  for(int j = 0; j < occ_grid_.info.height; j++)
  {
    for (int i = 0; i < occ_grid_.info.width; i++)
    {
      const size_t idx = map2Dto1DIdx(occ_grid_.info.width, i, occ_grid_.info.height - j - 1);
      const int8_t val = occ_grid_.data[idx];

      if (val == cost_val::OCC){ // Occupied
        obst_map_[idx] = idx; 
        dist_map_[idx] = 0; 

        open_queue_.put(idx, 0); // add to open queue
      }
      else if (val == cost_val::UNKNOWN){ // Unknown
         dist_map_[idx] = 0; 
      }
      else if (val == cost_val::FREE){ // Free
         dist_map_[idx] = INF; 
      }
      else{
         dist_map_[idx] = INF; 
      }
    }
  }

  while (!open_queue_.empty()){
    const size_t idx = open_queue_.get();
    if (to_raise_[idx]){
      raise(idx);
    }
    else if (isOcc(obst_map_[idx])){
      voro_map_[idx] = false;
      lower(idx);
    }
  }
  
}