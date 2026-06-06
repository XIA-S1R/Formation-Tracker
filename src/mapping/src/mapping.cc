#include <mapping/mapping.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>

namespace mapping {
void OccGridMap::updateMap(const Eigen::Vector3d& sensor_p,
                           const std::vector<Eigen::Vector3d>& pc) {
  vis.fillData(0);

  Eigen::Vector3i sensor_idx = pos2idx(sensor_p);
  Eigen::Vector3i offset = sensor_idx - Eigen::Vector3i(size_x / 2, size_y / 2, size_z / 2);
  // NOTE clear the updated part
  if (init_finished) {
    Eigen::Vector3i move = offset - Eigen::Vector3i(offset_x, offset_y, offset_z);
    Eigen::Vector3i from, to, from_small, to_small;
    Eigen::Vector3i size_xyz(size_x, size_y, size_z);
    for (int i = 0; i < 3; ++i) {
      if (move[i] >= 0) {
        from[i] = 0;
        from_small[i] = inflate_size;
        to[i] = move[i];
        to_small[i] = move[i] + inflate_size;
      } else {
        from[i] = move[i] + size_xyz[i];
        from_small[i] = move[i] + size_x - inflate_size;
        to[i] = size_xyz[i];
        to_small[i] = size_xyz[i] - inflate_size;
      }
    }

    for (int x = from.x(); x < to.x(); ++x) {
      for (int y = 0; y < size_y; ++y) {
        for (int z = 0; z < size_z; ++z) {
          Eigen::Vector3i id = Eigen::Vector3i(offset_x + x, offset_y + y, offset_z + z);
          occ.atId(id) = 0;
          infocc.atId(id) = 0;
        }
      }
    }
    for (int x = from_small.x(); x < to_small.x(); ++x) {
      for (int y = inflate_size; y < size_y - inflate_size; ++y) {
        for (int z = inflate_size; z < size_z - inflate_size; ++z) {
          Eigen::Vector3i id = Eigen::Vector3i(offset_x + x, offset_y + y, offset_z + z);
          pro.atId(id) = p_def;
        }
      }
    }
    offset_x = offset.x();
    for (int y = from.y(); y < to.y(); ++y) {
      for (int x = 0; x < size_x; ++x) {
        for (int z = 0; z < size_z; ++z) {
          Eigen::Vector3i id = Eigen::Vector3i(offset_x + x, offset_y + y, offset_z + z);
          occ.atId(id) = 0;
          infocc.atId(id) = 0;
        }
      }
    }
    for (int y = from_small.y(); y < to_small.y(); ++y) {
      for (int x = inflate_size; x < size_x - inflate_size; ++x) {
        for (int z = inflate_size; z < size_z - inflate_size; ++z) {
          Eigen::Vector3i id = Eigen::Vector3i(offset_x + x, offset_y + y, offset_z + z);
          pro.atId(id) = p_def;
        }
      }
    }
    offset_y = offset.y();
    for (int z = from.z(); z < to.z(); ++z) {
      for (int x = 0; x < size_x; ++x) {
        for (int y = 0; y < size_y; ++y) {
          Eigen::Vector3i id = Eigen::Vector3i(offset_x + x, offset_y + y, offset_z + z);
          occ.atId(id) = 0;
          infocc.atId(id) = 0;
        }
      }
    }
    for (int z = from_small.z(); z < to_small.z(); ++z) {
      for (int x = inflate_size; x < size_x - inflate_size; ++x) {
        for (int y = inflate_size; y < size_y - inflate_size; ++y) {
          Eigen::Vector3i id = Eigen::Vector3i(offset_x + x, offset_y + y, offset_z + z);
          pro.atId(id) = p_def;
        }
      }
    }
    offset_z = offset.z();
  } else {
    offset_x = offset.x();
    offset_y = offset.y();
    offset_z = offset.z();
    init_finished = true;
  }
  // set occupied
  for (const auto& p : pc) {
    Eigen::Vector3d pt;
    bool inrange = filter(sensor_p, p, pt);
    Eigen::Vector3i idx = pos2idx(pt);
    if (vis.atId(idx) != 1) {
      if (inrange) {
        hit(idx);
      } else {
        mis(idx);
      }
    }
  }
}

void OccGridMap::occ2pc(sensor_msgs::PointCloud2& msg) {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> pcd;
  for (int x = 0; x < size_x; ++x) {
    for (int y = 0; y < size_y; ++y) {
      for (int z = 0; z < size_z; ++z) {
        Eigen::Vector3i idx(offset_x + x, offset_y + y, offset_z + z);
        if (infocc.atId(idx) == 1) {
          pt.x = (offset_x + x + 0.5) * resolution;
          pt.y = (offset_y + y + 0.5) * resolution;
          pt.z = (offset_z + z + 0.5) * resolution;
          pcd.push_back(pt);
        }
      }
    }
  }
  pcd.width = pcd.points.size();
  pcd.height = 1;
  pcd.is_dense = true;
  pcl::toROSMsg(pcd, msg);
  msg.header.frame_id = "world";
}

void OccGridMap::occ2pc(sensor_msgs::PointCloud2& msg, double floor, double ceil) {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> pcd;
  for (int x = 0; x < size_x; ++x) {
    for (int y = 0; y < size_y; ++y) {
      for (int z = 0; z < size_z; ++z) {
        Eigen::Vector3i idx(offset_x + x, offset_y + y, offset_z + z);
        if (infocc.atId(idx) == 1) {
          pt.x = (offset_x + x + 0.5) * resolution;
          pt.y = (offset_y + y + 0.5) * resolution;
          pt.z = (offset_z + z + 0.5) * resolution;
          if (pt.z > floor && pt.z < ceil) {
            pcd.push_back(pt);
          }
        }
      }
    }
  }
  pcd.width = pcd.points.size();
  pcd.height = 1;
  pcd.is_dense = true;
  pcl::toROSMsg(pcd, msg);
  msg.header.frame_id = "world";
}

void OccGridMap::inflate_once() {
  static Eigen::Vector3i p;
  for (const auto& id : v0) {
    int x0 = id.x() - 1 >= offset_x ? id.x() - 1 : offset_x;
    int y0 = id.y() - 1 >= offset_y ? id.y() - 1 : offset_y;
    int z0 = id.z() - 1 >= offset_z ? id.z() - 1 : offset_z;
    int x1 = id.x() + 1 <= offset_x + size_x - 1 ? id.x() + 1 : offset_x + size_x - 1;
    int y1 = id.y() + 1 <= offset_y + size_y - 1 ? id.y() + 1 : offset_y + size_y - 1;
    int z1 = id.z() + 1 <= offset_z + size_z - 1 ? id.z() + 1 : offset_z + size_z - 1;
    for (p.x() = x0; p.x() <= x1; p.x()++)
      for (p.y() = y0; p.y() <= y1; p.y()++)
        for (p.z() = z0; p.z() <= z1; p.z()++) {
          auto ptr = infocc.atIdPtr(p);
          if ((*ptr) != 1) {
            *ptr = 1;
            v1.push_back(p);
          }
        }
  }
}

void OccGridMap::inflate_xy() {
  static Eigen::Vector3i p;
  for (const auto& id : v0) {
    int x0 = id.x() - 1 >= offset_x ? id.x() - 1 : offset_x;
    int y0 = id.y() - 1 >= offset_y ? id.y() - 1 : offset_y;
    int x1 = id.x() + 1 <= offset_x + size_x - 1 ? id.x() + 1 : offset_x + size_x - 1;
    int y1 = id.y() + 1 <= offset_y + size_y - 1 ? id.y() + 1 : offset_y + size_y - 1;
    p.z() = id.z();
    for (p.x() = x0; p.x() <= x1; p.x()++)
      for (p.y() = y0; p.y() <= y1; p.y()++) {
        auto ptr = infocc.atIdPtr(p);
        if ((*ptr) != 1) {
          *ptr = 1;
          v1.push_back(p);
        }
      }
  }
}

void OccGridMap::inflate_last() {
  static Eigen::Vector3i p;
  for (const auto& id : v1) {
    int x0 = id.x() - 1 >= offset_x ? id.x() - 1 : offset_x;
    int y0 = id.y() - 1 >= offset_y ? id.y() - 1 : offset_y;
    int z0 = id.z() - 1 >= offset_z ? id.z() - 1 : offset_z;
    int x1 = id.x() + 1 <= offset_x + size_x - 1 ? id.x() + 1 : offset_x + size_x - 1;
    int y1 = id.y() + 1 <= offset_y + size_y - 1 ? id.y() + 1 : offset_y + size_y - 1;
    int z1 = id.z() + 1 <= offset_z + size_z - 1 ? id.z() + 1 : offset_z + size_z - 1;
    for (p.x() = x0; p.x() <= x1; p.x()++)
      for (p.y() = y0; p.y() <= y1; p.y()++)
        for (p.z() = z0; p.z() <= z1; p.z()++)
          infocc.atId(p) = 1;
  }
}

void OccGridMap::inflate(int inflate_size) {
  if (inflate_size < 1) {
    return;
  }
  Eigen::Vector3i idx;
  v1.clear();
  for (idx.x() = offset_x; idx.x() < offset_x + size_x; ++idx.x())
    for (idx.y() = offset_y; idx.y() < offset_y + size_y; ++idx.y())
      for (idx.z() = offset_z; idx.z() < offset_z + size_z; ++idx.z()) {
        if (infocc.atId(idx) == 1) {
          v1.push_back(idx);
        }
      }
  for (int i = 0; i < inflate_size - 1; ++i) {
    std::swap(v0, v1);
    v1.clear();
    inflate_once();
  }
  inflate_last();
}

// ---- ESDF implementation ----

template <typename F_get, typename F_set>
void OccGridMap::fillESDF(F_get f_get, F_set f_set, int start, int end) {
  int n = end - start + 1;
  if (n <= 0) return;
  // Felzenszwalb's 1D squared-distance transform
  std::vector<int> v_arr(n);
  std::vector<double> z_arr(n + 1);
  int k = 0;
  v_arr[0] = start;
  z_arr[0] = -1e18;
  z_arr[1] = 1e18;

  for (int q = start + 1; q <= end; q++) {
    double s;
    k++;
    do {
      k--;
      s = ((f_get(q) + (double)q * q) - (f_get(v_arr[k]) + (double)v_arr[k] * v_arr[k])) /
          (2.0 * q - 2.0 * v_arr[k]);
    } while (s <= z_arr[k]);
    k++;
    v_arr[k] = q;
    z_arr[k] = s;
    z_arr[k + 1] = 1e18;
  }

  k = 0;
  for (int q = start; q <= end; q++) {
    while (z_arr[k + 1] < q) k++;
    double val = (double)(q - v_arr[k]) * (q - v_arr[k]) + f_get(v_arr[k]);
    f_set(q, val);
  }
}

void OccGridMap::updateESDF() {
  int total = size_x * size_y * size_z;
  if ((int)esdf_buffer_.size() != total) esdf_buffer_.resize(total);
  if ((int)esdf_tmp1_.size() != total) esdf_tmp1_.resize(total);
  if ((int)esdf_tmp2_.size() != total) esdf_tmp2_.resize(total);

  // Pass 1: along Z
  for (int rx = 0; rx < size_x; rx++) {
    for (int ry = 0; ry < size_y; ry++) {
      fillESDF(
          [&](int rz) -> double {
            Eigen::Vector3i id(offset_x + rx, offset_y + ry, offset_z + rz);
            return (infocc.atId(id) == 1) ? 0.0 : 1e10;
          },
          [&](int rz, double val) { esdf_tmp1_[esdfAddr(rx, ry, rz)] = val; },
          0, size_z - 1);
    }
  }

  // Pass 2: along Y
  for (int rx = 0; rx < size_x; rx++) {
    for (int rz = 0; rz < size_z; rz++) {
      fillESDF(
          [&](int ry) -> double { return esdf_tmp1_[esdfAddr(rx, ry, rz)]; },
          [&](int ry, double val) { esdf_tmp2_[esdfAddr(rx, ry, rz)] = val; },
          0, size_y - 1);
    }
  }

  // Pass 3: along X → final distance
  for (int ry = 0; ry < size_y; ry++) {
    for (int rz = 0; rz < size_z; rz++) {
      fillESDF(
          [&](int rx) -> double { return esdf_tmp2_[esdfAddr(rx, ry, rz)]; },
          [&](int rx, double val) {
            esdf_buffer_[esdfAddr(rx, ry, rz)] = resolution * std::sqrt(val);
          },
          0, size_x - 1);
    }
  }

  esdf_valid_ = true;
}

void OccGridMap::evaluateEDT(const Eigen::Vector3d& pos, double& dist) const {
  if (!esdf_valid_) { dist = 0; return; }

  // Continuous position to relative float index
  double fx = (pos.x() / resolution - 0.5) - offset_x;
  double fy = (pos.y() / resolution - 0.5) - offset_y;
  double fz = (pos.z() / resolution - 0.5) - offset_z;

  int x0 = (int)std::floor(fx), y0 = (int)std::floor(fy), z0 = (int)std::floor(fz);
  double dx = fx - x0, dy = fy - y0, dz = fz - z0;

  // Clamp to valid range
  auto clamp = [](int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); };
  int x1 = clamp(x0 + 1, 0, size_x - 1); x0 = clamp(x0, 0, size_x - 1);
  int y1 = clamp(y0 + 1, 0, size_y - 1); y0 = clamp(y0, 0, size_y - 1);
  int z1 = clamp(z0 + 1, 0, size_z - 1); z0 = clamp(z0, 0, size_z - 1);

  // Trilinear interpolation
  double d000 = esdf_buffer_[esdfAddr(x0,y0,z0)], d100 = esdf_buffer_[esdfAddr(x1,y0,z0)];
  double d010 = esdf_buffer_[esdfAddr(x0,y1,z0)], d110 = esdf_buffer_[esdfAddr(x1,y1,z0)];
  double d001 = esdf_buffer_[esdfAddr(x0,y0,z1)], d101 = esdf_buffer_[esdfAddr(x1,y0,z1)];
  double d011 = esdf_buffer_[esdfAddr(x0,y1,z1)], d111 = esdf_buffer_[esdfAddr(x1,y1,z1)];

  double v00 = (1-dx)*d000 + dx*d100;
  double v10 = (1-dx)*d010 + dx*d110;
  double v01 = (1-dx)*d001 + dx*d101;
  double v11 = (1-dx)*d011 + dx*d111;
  double v0 = (1-dy)*v00 + dy*v10;
  double v1 = (1-dy)*v01 + dy*v11;
  dist = (1-dz)*v0 + dz*v1;
}

void OccGridMap::evaluateFirstGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad) const {
  if (!esdf_valid_) { grad.setZero(); return; }

  double fx = (pos.x() / resolution - 0.5) - offset_x;
  double fy = (pos.y() / resolution - 0.5) - offset_y;
  double fz = (pos.z() / resolution - 0.5) - offset_z;

  int x0 = (int)std::floor(fx), y0 = (int)std::floor(fy), z0 = (int)std::floor(fz);
  double dx = fx - x0, dy = fy - y0, dz = fz - z0;

  auto clamp = [](int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); };
  int x1 = clamp(x0 + 1, 0, size_x - 1); x0 = clamp(x0, 0, size_x - 1);
  int y1 = clamp(y0 + 1, 0, size_y - 1); y0 = clamp(y0, 0, size_y - 1);
  int z1 = clamp(z0 + 1, 0, size_z - 1); z0 = clamp(z0, 0, size_z - 1);

  double d000 = esdf_buffer_[esdfAddr(x0,y0,z0)], d100 = esdf_buffer_[esdfAddr(x1,y0,z0)];
  double d010 = esdf_buffer_[esdfAddr(x0,y1,z0)], d110 = esdf_buffer_[esdfAddr(x1,y1,z0)];
  double d001 = esdf_buffer_[esdfAddr(x0,y0,z1)], d101 = esdf_buffer_[esdfAddr(x1,y0,z1)];
  double d011 = esdf_buffer_[esdfAddr(x0,y1,z1)], d111 = esdf_buffer_[esdfAddr(x1,y1,z1)];

  double inv_res = 1.0 / resolution;

  // dF/dx
  double v00 = (1-dx)*d000 + dx*d100;
  double v10 = (1-dx)*d010 + dx*d110;
  double v01 = (1-dx)*d001 + dx*d101;
  double v11 = (1-dx)*d011 + dx*d111;

  // grad z
  double e0 = (1-dy)*v00 + dy*v10;
  double e1 = (1-dy)*v01 + dy*v11;
  grad[2] = (e1 - e0) * inv_res;

  // grad y
  grad[1] = ((1-dz)*(v10 - v00) + dz*(v11 - v01)) * inv_res;

  // grad x
  grad[0]  = (1-dz)*(1-dy)*(d100 - d000);
  grad[0] += (1-dz)*dy*(d110 - d010);
  grad[0] += dz*(1-dy)*(d101 - d001);
  grad[0] += dz*dy*(d111 - d011);
  grad[0] *= inv_res;
}

}  // namespace mapping
