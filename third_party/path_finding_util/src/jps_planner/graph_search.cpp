#include <cmath>
#include <path_finding_util/jps_planner/jps_planner/graph_search.h>

using namespace JPS;

GraphSearch::GraphSearch(std::shared_ptr<GridMap> map, int xDim, int yDim, int zDim,
                         double eps, bool verbose)
    : map_(map), xDim_(xDim), yDim_(yDim), zDim_(zDim), eps_(eps),
      verbose_(verbose) {
  hm_.resize(xDim_ * yDim_ * zDim_);
  seen_.resize(xDim_ * yDim_ * zDim_, false);

  // Set 3D neighbors
  for (int x = -1; x <= 1; x++) {
    for (int y = -1; y <= 1; y++) {
      for (int z = -1; z <= 1; z++) {
        if (x == 0 && y == 0 && z == 0)
          continue;
        ns_.push_back(std::vector<int>{x, y, z});
      }
    }
  }
  jn3d_ = std::make_shared<JPS3DNeib>();
}

inline int GraphSearch::coordToId(int x, int y, int z) const {
  Eigen::Vector3i idx = Eigen::Vector3i(x,y,z) - map_->getVoxelOrigin();
  return idx(0) + idx(1) * xDim_ + idx(2) * xDim_ * yDim_;
}

inline bool GraphSearch::isFree(int x, int y, int z) const {

  return map_->isFree(Eigen::Vector3i{x, y, z});
}


inline bool GraphSearch::isOccupied(int x, int y, int z) const {

  return map_->isOccupied(Eigen::Vector3i{x, y, z});
}


inline double GraphSearch::getHeur(int x, int y, int z) const {
  return eps_ *
         std::sqrt((x - xGoal_) * (x - xGoal_) + (y - yGoal_) * (y - yGoal_) +
                   (z - zGoal_) * (z - zGoal_));
}


bool GraphSearch::plan(int xStart, int yStart, int zStart, int xGoal, int yGoal,
                       int zGoal, bool useJps, int maxExpand) {

  auto a = std::chrono::high_resolution_clock::now();

  pq_.clear();
  path_.clear();
  hm_.resize(xDim_ * yDim_ * zDim_);
  seen_.resize(xDim_ * yDim_ * zDim_, false);

  // Set jps
  use_jps_ = useJps;

  // Set goal
  int goal_id = coordToId(xGoal, yGoal, zGoal);
  xGoal_ = xGoal;
  yGoal_ = yGoal;
  zGoal_ = zGoal;
  // Set start node
  int start_id = coordToId(xStart, yStart, zStart);
  StatePtr currNode_ptr =
      std::make_shared<State>(State(start_id, xStart, yStart, zStart, 0, 0, 0));
  currNode_ptr->g = 0;
  currNode_ptr->h = getHeur(xStart, yStart, zStart);

  auto b = std::chrono::high_resolution_clock::now();
  double plan_dur = std::chrono::duration_cast<std::chrono::duration<double>>(
            b - a).count() * 1000.0;
            
  std::cout << "GraphSearch::plan setup duration: "<< plan_dur << " s" << std::endl;

  return plan(currNode_ptr, maxExpand, start_id, goal_id);
}

bool GraphSearch::plan(StatePtr &currNode_ptr, int maxExpand, int start_id,
                       int goal_id) {
  // Insert start node
  currNode_ptr->heapkey = pq_.push(currNode_ptr);
  currNode_ptr->opened = true;
  hm_[currNode_ptr->id] = currNode_ptr;
  seen_[currNode_ptr->id] = true;

  std::cout << "Before main planning loop" << std::endl;;

  double getsucc_dur = 0;
  double procsucc_dur = 0;

  int expand_iteration = 0;
  while (true) {
    std::cout << "expand_iteration: " << expand_iteration << std::endl;;
    expand_iteration++;
    // get element with smallest cost
    currNode_ptr = pq_.top();
    pq_.pop();
    currNode_ptr->closed = true; // Add to closed list

    if (currNode_ptr->id == goal_id) {
      if (verbose_)
        printf("Goal Reached!!!!!!\n\n");
      break;
    }

    // printf("expand: %d, %d\n", currNode_ptr->x, currNode_ptr->y);
    std::vector<int> succ_ids;
    std::vector<double> succ_costs;
    // Get successors
    if (!use_jps_){
      // auto getsucc_a = std::chrono::high_resolution_clock::now();
      getSucc(currNode_ptr, succ_ids, succ_costs);
      // getsucc_dur += std::chrono::duration_cast<std::chrono::duration<double>>(
      //           std::chrono::high_resolution_clock::now() - getsucc_a).count() * 1000.0;
    }
    else{
      auto getsucc_a = std::chrono::high_resolution_clock::now();
      getJpsSucc(currNode_ptr, succ_ids, succ_costs);
      getsucc_dur += std::chrono::duration_cast<std::chrono::duration<double>>(
                std::chrono::high_resolution_clock::now() - getsucc_a).count() * 1000.0;
    }

    auto procsucc_a = std::chrono::high_resolution_clock::now();

    // if(verbose_)
    // printf("size of succs: %zu\n", succ_ids.size());
    // Process successors
    for (int s = 0; s < (int)succ_ids.size(); s++) {
      // see if we can improve the value of succstate
      StatePtr &child_ptr = hm_[succ_ids[s]];
      double tentative_gval = currNode_ptr->g + succ_costs[s];

      if (tentative_gval < child_ptr->g) {
        child_ptr->parentId = currNode_ptr->id; // Assign new parent
        child_ptr->g = tentative_gval;          // Update gval

        // double fval = child_ptr->g + child_ptr->h;

        // if currently in OPEN, update
        if (child_ptr->opened && !child_ptr->closed) {
          pq_.increase(child_ptr->heapkey); // update heap
          child_ptr->dx = (child_ptr->x - currNode_ptr->x);
          child_ptr->dy = (child_ptr->y - currNode_ptr->y);
          child_ptr->dz = (child_ptr->z - currNode_ptr->z);
          if (child_ptr->dx != 0)
            child_ptr->dx /= std::abs(child_ptr->dx);
          if (child_ptr->dy != 0)
            child_ptr->dy /= std::abs(child_ptr->dy);
          if (child_ptr->dz != 0)
            child_ptr->dz /= std::abs(child_ptr->dz);
        }
        // if currently in CLOSED
        else if (child_ptr->opened && child_ptr->closed) {
          printf("ASTAR ERROR!\n");
        } else // new node, add to heap
        {
          // printf("add to open set: %d, %d\n", child_ptr->x, child_ptr->y);
          child_ptr->heapkey = pq_.push(child_ptr);
          child_ptr->opened = true;
        }
      } //
    }   // Process successors

    procsucc_dur += std::chrono::duration_cast<std::chrono::duration<double>>(
              std::chrono::high_resolution_clock::now() - procsucc_a).count() * 1000.0;

    if (maxExpand > 0 && expand_iteration >= maxExpand) {
      if (verbose_)
        printf("MaxExpandStep [%d] Reached!!!!!!\n\n", maxExpand);
      return false;
    }

    if (pq_.empty()) {
      if (verbose_)
        printf("Priority queue is empty!!!!!!\n\n");
      return false;
    }
  } // end while(true)

  if (verbose_) {
    printf("goal g: %f, h: %f!\n", currNode_ptr->g, currNode_ptr->h);
    printf("Expand [%d] nodes!\n", expand_iteration);
  }

  std::cout << "getsucc_dur: " << getsucc_dur << " s" << std::endl; 
  std::cout << "procsucc_dur: " << procsucc_dur << " s" << std::endl; 

  path_ = recoverPath(currNode_ptr, start_id);

  return true;
}

std::vector<StatePtr> GraphSearch::recoverPath(StatePtr node, int start_id) {
  std::vector<StatePtr> path;
  path.push_back(node);
  while (node && node->id != start_id) {
    node = hm_[node->parentId];
    path.push_back(node);
  }

  return path;
}

void GraphSearch::getSucc(const StatePtr &curr, std::vector<int> &succ_ids,
                          std::vector<double> &succ_costs) {
    for (const auto &d : ns_) {
      int new_x = curr->x + d[0];
      int new_y = curr->y + d[1];
      int new_z = curr->z + d[2];
      if (!isFree(new_x, new_y, new_z))
        continue;

      int new_id = coordToId(new_x, new_y, new_z);
      if (!seen_[new_id]) {
        seen_[new_id] = true;
        hm_[new_id] = std::make_shared<State>(new_id, new_x, new_y, new_z, d[0],
                                              d[1], d[2]);
        hm_[new_id]->h = getHeur(new_x, new_y, new_z);
      }

      succ_ids.push_back(new_id);
      succ_costs.push_back(std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]));
    }
}

void GraphSearch::getJpsSucc(const StatePtr &curr, std::vector<int> &succ_ids,
                             std::vector<double> &succ_costs) {
    const int norm1 =
        std::abs(curr->dx) + std::abs(curr->dy) + std::abs(curr->dz);
    int num_neib = jn3d_->nsz[norm1][0];
    int num_fneib = jn3d_->nsz[norm1][1];
    int id = (curr->dx + 1) + 3 * (curr->dy + 1) + 9 * (curr->dz + 1);

    for (int dev = 0; dev < num_neib + num_fneib; ++dev) {
      int new_x, new_y, new_z;
      int dx, dy, dz;
      if (dev < num_neib) {
        dx = jn3d_->ns[id][0][dev];
        dy = jn3d_->ns[id][1][dev];
        dz = jn3d_->ns[id][2][dev];
        if (!jump(curr->x, curr->y, curr->z, dx, dy, dz, new_x, new_y, new_z))
          continue;
      } else {
        int nx = curr->x + jn3d_->f1[id][0][dev - num_neib];
        int ny = curr->y + jn3d_->f1[id][1][dev - num_neib];
        int nz = curr->z + jn3d_->f1[id][2][dev - num_neib];
        if (isOccupied(nx, ny, nz)) {
          dx = jn3d_->f2[id][0][dev - num_neib];
          dy = jn3d_->f2[id][1][dev - num_neib];
          dz = jn3d_->f2[id][2][dev - num_neib];
          if (!jump(curr->x, curr->y, curr->z, dx, dy, dz, new_x, new_y, new_z))
            continue;
        } else
          continue;
      }

      int new_id = coordToId(new_x, new_y, new_z);
      if (!seen_[new_id]) {
        seen_[new_id] = true;
        hm_[new_id] =
            std::make_shared<State>(new_id, new_x, new_y, new_z, dx, dy, dz);
        hm_[new_id]->h = getHeur(new_x, new_y, new_z);
      }

      succ_ids.push_back(new_id);
      succ_costs.push_back(std::sqrt((new_x - curr->x) * (new_x - curr->x) +
                                     (new_y - curr->y) * (new_y - curr->y) +
                                     (new_z - curr->z) * (new_z - curr->z)));
    }
}


bool GraphSearch::jump(int x, int y, int z, int dx, int dy, int dz, int &new_x,
                       int &new_y, int &new_z) {
  new_x = x + dx;
  new_y = y + dy;
  new_z = z + dz;
  if (!isFree(new_x, new_y, new_z))
    return false;

  // check if it is traversable for safe corridors
  if (!isTraversable(x, y, z, dx, dy, dz))
    return false;

  if (new_x == xGoal_ && new_y == yGoal_ && new_z == zGoal_)
    return true;

  if (hasForced(new_x, new_y, new_z, dx, dy, dz))
    return true;

  const int id = (dx + 1) + 3 * (dy + 1) + 9 * (dz + 1);
  const int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
  int num_neib = jn3d_->nsz[norm1][0];
  for (int k = 0; k < num_neib - 1; ++k) {
    int new_new_x, new_new_y, new_new_z;
    if (jump(new_x, new_y, new_z, jn3d_->ns[id][0][k], jn3d_->ns[id][1][k],
             jn3d_->ns[id][2][k], new_new_x, new_new_y, new_new_z))
      return true;
  }

  return jump(new_x, new_y, new_z, dx, dy, dz, new_x, new_y, new_z);
}


inline bool GraphSearch::isTraversable(int x, int y, int z, int dx, int dy,
                                       int dz) {
  // first check if the norm is bigger than one; if not return true
  const int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
  if (norm1 < 2) {
    return true;
  } else {
    // if the norm is 2, then check if we can get to it from the sides
    if (norm1 == 2) {
      int x_1, y_1, z_1;
      int x_2, y_2, z_2;
      if (dx == 0) {
        x_1 = x;
        y_1 = y + dy;
        z_1 = z;

        x_2 = x;
        y_2 = y;
        z_2 = z + dz;
      } else if (dy == 0) {
        x_1 = x + dx;
        y_1 = y;
        z_1 = z;

        x_2 = x;
        y_2 = y;
        z_2 = z + dz;
      } else {
        x_1 = x + dx;
        y_1 = y;
        z_1 = z;

        x_2 = x;
        y_2 = y + dy;
        z_2 = z;
      }
      if (isFree(x_1, y_1, z_1) || isFree(x_2, y_2, z_2)) {
        return true;
      } else {
        return false;
      }
    } else if (norm1 == 3) {
      int x_1, y_1, z_1;
      int x_2, y_2, z_2;
      int x_3, y_3, z_3;
      int x_4, y_4, z_4;
      int x_5, y_5, z_5;
      int x_6, y_6, z_6;

      x_1 = x + dx;
      y_1 = y;
      z_1 = z;

      x_2 = x + dx;
      y_2 = y + dy;
      z_2 = z;

      x_3 = x;
      y_3 = y + dy;
      z_3 = z;

      if (isFree(x_2, y_2, z_2) &&
          (isFree(x_1, y_1, z_1) || isFree(x_3, y_3, z_3)))
        return true;

      x_4 = x;
      y_4 = y;
      z_4 = z + dz;

      x_5 = x;
      y_5 = y + dy;
      z_5 = z + dz;

      x_6 = x + dx;
      y_6 = y;
      z_6 = z + dz;

      if (isFree(x_4, y_4, z_4) &&
          (isFree(x_5, y_5, z_5) || isFree(x_6, y_6, z_6)))
        return true;

      if ((isFree(x_1, y_1, z_1) && isFree(x_6, y_6, z_6)) ||
          (isFree(x_3, y_3, z_3) && isFree(x_5, y_5, z_5)))
        return true;

      return false;
    } else {
      return false;
    }
  }
}

inline bool GraphSearch::hasForced(int x, int y, int z, int dx, int dy,
                                   int dz) {
  int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
  int id = (dx + 1) + 3 * (dy + 1) + 9 * (dz + 1);
  switch (norm1) {
  case 1:
    // 1-d move, check 8 neighbors
    for (int fn = 0; fn < 8; ++fn) {
      int nx = x + jn3d_->f1[id][0][fn];
      int ny = y + jn3d_->f1[id][1][fn];
      int nz = z + jn3d_->f1[id][2][fn];
      if (isOccupied(nx, ny, nz))
        return true;
    }
    return false;
  case 2:
    // 2-d move, check 8 neighbors
    for (int fn = 0; fn < 8; ++fn) {
      int nx = x + jn3d_->f1[id][0][fn];
      int ny = y + jn3d_->f1[id][1][fn];
      int nz = z + jn3d_->f1[id][2][fn];
      if (isOccupied(nx, ny, nz))
        return true;
    }
    return false;
  case 3:
    // 3-d move, check 6 neighbors
    for (int fn = 0; fn < 6; ++fn) {
      int nx = x + jn3d_->f1[id][0][fn];
      int ny = y + jn3d_->f1[id][1][fn];
      int nz = z + jn3d_->f1[id][2][fn];
      if (isOccupied(nx, ny, nz))
        return true;
    }
    return false;
  default:
    return false;
  }
}

std::vector<StatePtr> GraphSearch::getPath() const { return path_; }

std::vector<StatePtr> GraphSearch::getOpenSet() const {
  std::vector<StatePtr> ss;
  for (const auto &it : hm_) {
    if (it && it->opened && !it->closed)
      ss.push_back(it);
  }
  return ss;
}

std::vector<StatePtr> GraphSearch::getCloseSet() const {
  std::vector<StatePtr> ss;
  for (const auto &it : hm_) {
    if (it && it->closed)
      ss.push_back(it);
  }
  return ss;
}

std::vector<StatePtr> GraphSearch::getAllSet() const {
  std::vector<StatePtr> ss;
  for (const auto &it : hm_) {
    if (it)
      ss.push_back(it);
  }
  return ss;
}

constexpr int JPS3DNeib::nsz[4][2];

JPS3DNeib::JPS3DNeib() {
  int id = 0;
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
        for (int dev = 0; dev < nsz[norm1][0]; ++dev)
          Neib(dx, dy, dz, norm1, dev, ns[id][0][dev], ns[id][1][dev],
               ns[id][2][dev]);
        for (int dev = 0; dev < nsz[norm1][1]; ++dev) {
          FNeib(dx, dy, dz, norm1, dev, f1[id][0][dev], f1[id][1][dev],
                f1[id][2][dev], f2[id][0][dev], f2[id][1][dev], f2[id][2][dev]);
        }
        id++;
      }
    }
  }
}

void JPS3DNeib::Neib(int dx, int dy, int dz, int norm1, int dev, int &tx,
                     int &ty, int &tz) {
  switch (norm1) {
  case 0:
    switch (dev) {
    case 0:
      tx = 1;
      ty = 0;
      tz = 0;
      return;
    case 1:
      tx = -1;
      ty = 0;
      tz = 0;
      return;
    case 2:
      tx = 0;
      ty = 1;
      tz = 0;
      return;
    case 3:
      tx = 1;
      ty = 1;
      tz = 0;
      return;
    case 4:
      tx = -1;
      ty = 1;
      tz = 0;
      return;
    case 5:
      tx = 0;
      ty = -1;
      tz = 0;
      return;
    case 6:
      tx = 1;
      ty = -1;
      tz = 0;
      return;
    case 7:
      tx = -1;
      ty = -1;
      tz = 0;
      return;
    case 8:
      tx = 0;
      ty = 0;
      tz = 1;
      return;
    case 9:
      tx = 1;
      ty = 0;
      tz = 1;
      return;
    case 10:
      tx = -1;
      ty = 0;
      tz = 1;
      return;
    case 11:
      tx = 0;
      ty = 1;
      tz = 1;
      return;
    case 12:
      tx = 1;
      ty = 1;
      tz = 1;
      return;
    case 13:
      tx = -1;
      ty = 1;
      tz = 1;
      return;
    case 14:
      tx = 0;
      ty = -1;
      tz = 1;
      return;
    case 15:
      tx = 1;
      ty = -1;
      tz = 1;
      return;
    case 16:
      tx = -1;
      ty = -1;
      tz = 1;
      return;
    case 17:
      tx = 0;
      ty = 0;
      tz = -1;
      return;
    case 18:
      tx = 1;
      ty = 0;
      tz = -1;
      return;
    case 19:
      tx = -1;
      ty = 0;
      tz = -1;
      return;
    case 20:
      tx = 0;
      ty = 1;
      tz = -1;
      return;
    case 21:
      tx = 1;
      ty = 1;
      tz = -1;
      return;
    case 22:
      tx = -1;
      ty = 1;
      tz = -1;
      return;
    case 23:
      tx = 0;
      ty = -1;
      tz = -1;
      return;
    case 24:
      tx = 1;
      ty = -1;
      tz = -1;
      return;
    case 25:
      tx = -1;
      ty = -1;
      tz = -1;
      return;
    }
  case 1:
    tx = dx;
    ty = dy;
    tz = dz;
    return;
  case 2:
    switch (dev) {
    case 0:
      if (dz == 0) {
        tx = 0;
        ty = dy;
        tz = 0;
        return;
      } else {
        tx = 0;
        ty = 0;
        tz = dz;
        return;
      }
    case 1:
      if (dx == 0) {
        tx = 0;
        ty = dy;
        tz = 0;
        return;
      } else {
        tx = dx;
        ty = 0;
        tz = 0;
        return;
      }
    case 2:
      tx = dx;
      ty = dy;
      tz = dz;
      return;
    }
  case 3:
    switch (dev) {
    case 0:
      tx = dx;
      ty = 0;
      tz = 0;
      return;
    case 1:
      tx = 0;
      ty = dy;
      tz = 0;
      return;
    case 2:
      tx = 0;
      ty = 0;
      tz = dz;
      return;
    case 3:
      tx = dx;
      ty = dy;
      tz = 0;
      return;
    case 4:
      tx = dx;
      ty = 0;
      tz = dz;
      return;
    case 5:
      tx = 0;
      ty = dy;
      tz = dz;
      return;
    case 6:
      tx = dx;
      ty = dy;
      tz = dz;
      return;
    }
  }
}

void JPS3DNeib::FNeib(int dx, int dy, int dz, int norm1, int dev, int &fx,
                      int &fy, int &fz, int &nx, int &ny, int &nz) {
  switch (norm1) {
  case 1:
    switch (dev) {
    case 0:
      fx = 0;
      fy = 1;
      fz = 0;
      break;
    case 1:
      fx = 0;
      fy = -1;
      fz = 0;
      break;
    case 2:
      fx = 1;
      fy = 0;
      fz = 0;
      break;
    case 3:
      fx = 1;
      fy = 1;
      fz = 0;
      break;
    case 4:
      fx = 1;
      fy = -1;
      fz = 0;
      break;
    case 5:
      fx = -1;
      fy = 0;
      fz = 0;
      break;
    case 6:
      fx = -1;
      fy = 1;
      fz = 0;
      break;
    case 7:
      fx = -1;
      fy = -1;
      fz = 0;
      break;
    }
    nx = fx;
    ny = fy;
    nz = dz;
    // switch order if different direction
    if (dx != 0) {
      fz = fx;
      fx = 0;
      nz = fz;
      nx = dx;
    }
    if (dy != 0) {
      fz = fy;
      fy = 0;
      nz = fz;
      ny = dy;
    }
    return;
  case 2:
    if (dx == 0) {
      switch (dev) {
      case 0:
        fx = 0;
        fy = 0;
        fz = -dz;
        nx = 0;
        ny = dy;
        nz = -dz;
        return;
      case 1:
        fx = 0;
        fy = -dy;
        fz = 0;
        nx = 0;
        ny = -dy;
        nz = dz;
        return;
      case 2:
        fx = 1;
        fy = 0;
        fz = 0;
        nx = 1;
        ny = dy;
        nz = dz;
        return;
      case 3:
        fx = -1;
        fy = 0;
        fz = 0;
        nx = -1;
        ny = dy;
        nz = dz;
        return;
      case 4:
        fx = 1;
        fy = 0;
        fz = -dz;
        nx = 1;
        ny = dy;
        nz = -dz;
        return;
      case 5:
        fx = 1;
        fy = -dy;
        fz = 0;
        nx = 1;
        ny = -dy;
        nz = dz;
        return;
      case 6:
        fx = -1;
        fy = 0;
        fz = -dz;
        nx = -1;
        ny = dy;
        nz = -dz;
        return;
      case 7:
        fx = -1;
        fy = -dy;
        fz = 0;
        nx = -1;
        ny = -dy;
        nz = dz;
        return;
      // Extras
      case 8:
        fx = 1;
        fy = 0;
        fz = 0;
        nx = 1;
        ny = dy;
        nz = 0;
        return;
      case 9:
        fx = 1;
        fy = 0;
        fz = 0;
        nx = 1;
        ny = 0;
        nz = dz;
        return;
      case 10:
        fx = -1;
        fy = 0;
        fz = 0;
        nx = -1;
        ny = dy;
        nz = 0;
        return;
      case 11:
        fx = -1;
        fy = 0;
        fz = 0;
        nx = -1;
        ny = 0;
        nz = dz;
        return;
      }
    } else if (dy == 0) {
      switch (dev) {
      case 0:
        fx = 0;
        fy = 0;
        fz = -dz;
        nx = dx;
        ny = 0;
        nz = -dz;
        return;
      case 1:
        fx = -dx;
        fy = 0;
        fz = 0;
        nx = -dx;
        ny = 0;
        nz = dz;
        return;
      case 2:
        fx = 0;
        fy = 1;
        fz = 0;
        nx = dx;
        ny = 1;
        nz = dz;
        return;
      case 3:
        fx = 0;
        fy = -1;
        fz = 0;
        nx = dx;
        ny = -1;
        nz = dz;
        return;
      case 4:
        fx = 0;
        fy = 1;
        fz = -dz;
        nx = dx;
        ny = 1;
        nz = -dz;
        return;
      case 5:
        fx = -dx;
        fy = 1;
        fz = 0;
        nx = -dx;
        ny = 1;
        nz = dz;
        return;
      case 6:
        fx = 0;
        fy = -1;
        fz = -dz;
        nx = dx;
        ny = -1;
        nz = -dz;
        return;
      case 7:
        fx = -dx;
        fy = -1;
        fz = 0;
        nx = -dx;
        ny = -1;
        nz = dz;
        return;
      // Extras
      case 8:
        fx = 0;
        fy = 1;
        fz = 0;
        nx = dx;
        ny = 1;
        nz = 0;
        return;
      case 9:
        fx = 0;
        fy = 1;
        fz = 0;
        nx = 0;
        ny = 1;
        nz = dz;
        return;
      case 10:
        fx = 0;
        fy = -1;
        fz = 0;
        nx = dx;
        ny = -1;
        nz = 0;
        return;
      case 11:
        fx = 0;
        fy = -1;
        fz = 0;
        nx = 0;
        ny = -1;
        nz = dz;
        return;
      }
    } else { // dz==0
      switch (dev) {
      case 0:
        fx = 0;
        fy = -dy;
        fz = 0;
        nx = dx;
        ny = -dy;
        nz = 0;
        return;
      case 1:
        fx = -dx;
        fy = 0;
        fz = 0;
        nx = -dx;
        ny = dy;
        nz = 0;
        return;
      case 2:
        fx = 0;
        fy = 0;
        fz = 1;
        nx = dx;
        ny = dy;
        nz = 1;
        return;
      case 3:
        fx = 0;
        fy = 0;
        fz = -1;
        nx = dx;
        ny = dy;
        nz = -1;
        return;
      case 4:
        fx = 0;
        fy = -dy;
        fz = 1;
        nx = dx;
        ny = -dy;
        nz = 1;
        return;
      case 5:
        fx = -dx;
        fy = 0;
        fz = 1;
        nx = -dx;
        ny = dy;
        nz = 1;
        return;
      case 6:
        fx = 0;
        fy = -dy;
        fz = -1;
        nx = dx;
        ny = -dy;
        nz = -1;
        return;
      case 7:
        fx = -dx;
        fy = 0;
        fz = -1;
        nx = -dx;
        ny = dy;
        nz = -1;
        return;
      // Extras
      case 8:
        fx = 0;
        fy = 0;
        fz = 1;
        nx = dx;
        ny = 0;
        nz = 1;
        return;
      case 9:
        fx = 0;
        fy = 0;
        fz = 1;
        nx = 0;
        ny = dy;
        nz = 1;
        return;
      case 10:
        fx = 0;
        fy = 0;
        fz = -1;
        nx = dx;
        ny = 0;
        nz = -1;
        return;
      case 11:
        fx = 0;
        fy = 0;
        fz = -1;
        nx = 0;
        ny = dy;
        nz = -1;
        return;
      }
    }
  case 3:
    switch (dev) {
    case 0:
      fx = -dx;
      fy = 0;
      fz = 0;
      nx = -dx;
      ny = dy;
      nz = dz;
      return;
    case 1:
      fx = 0;
      fy = -dy;
      fz = 0;
      nx = dx;
      ny = -dy;
      nz = dz;
      return;
    case 2:
      fx = 0;
      fy = 0;
      fz = -dz;
      nx = dx;
      ny = dy;
      nz = -dz;
      return;
    // Need to check up to here for forced!
    case 3:
      fx = 0;
      fy = -dy;
      fz = -dz;
      nx = dx;
      ny = -dy;
      nz = -dz;
      return;
    case 4:
      fx = -dx;
      fy = 0;
      fz = -dz;
      nx = -dx;
      ny = dy;
      nz = -dz;
      return;
    case 5:
      fx = -dx;
      fy = -dy;
      fz = 0;
      nx = -dx;
      ny = -dy;
      nz = dz;
      return;
    // Extras
    case 6:
      fx = -dx;
      fy = 0;
      fz = 0;
      nx = -dx;
      ny = 0;
      nz = dz;
      return;
    case 7:
      fx = -dx;
      fy = 0;
      fz = 0;
      nx = -dx;
      ny = dy;
      nz = 0;
      return;
    case 8:
      fx = 0;
      fy = -dy;
      fz = 0;
      nx = 0;
      ny = -dy;
      nz = dz;
      return;
    case 9:
      fx = 0;
      fy = -dy;
      fz = 0;
      nx = dx;
      ny = -dy;
      nz = 0;
      return;
    case 10:
      fx = 0;
      fy = 0;
      fz = -dz;
      nx = 0;
      ny = dy;
      nz = -dz;
      return;
    case 11:
      fx = 0;
      fy = 0;
      fz = -dz;
      nx = dx;
      ny = 0;
      nz = -dz;
      return;
    }
  }
}
