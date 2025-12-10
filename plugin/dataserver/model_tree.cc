#include "model_tree.h"
#include <iostream>
#include <sstream>
#include <string>

namespace mujoco::plugin::dataserver {

JointInfo::JointInfo(int id, const std::string &name, int type)
    : id(id), name(name), type(type), dof_adr(-1), qpos_adr(-1), limited(0) {
  axis[0] = axis[1] = axis[2] = 0.0;
  pos[0] = pos[1] = pos[2] = 0.0;
  range[0] = range[1] = 0.0;
}

GeomInfo::GeomInfo(int id, const std::string &name, int type)
    : id(id), name(name), type(type) {
  size[0] = size[1] = size[2] = 0.0;
  pos[0] = pos[1] = pos[2] = 0.0;
  quat[0] = 1.0;
  quat[1] = quat[2] = quat[3] = 0.0;
  rgba[0] = rgba[1] = rgba[2] = rgba[3] = 1.0;
}
BodyNode::BodyNode(int id, const std::string &name, int parent_id)
    : id(id), name(name), parent_id(parent_id) {
  pos[0] = pos[1] = pos[2] = 0.0;
  quat[0] = 1.0;
  quat[1] = quat[2] = quat[3] = 0.0;
}

// 添加子节点
void BodyNode::addChild(std::shared_ptr<BodyNode> child) {
  children.push_back(child);
}

// 添加关节
void BodyNode::addJoint(const JointInfo &joint) { joints.push_back(joint); }

// 添加几何体
void BodyNode::addGeom(const GeomInfo &geom) { geoms.push_back(geom); }

// 打印树结构
void BodyNode::printTree(int depth) const {
  std::string indent(depth * 2, ' ');
  std::cout << indent << "Body: " << name << " (ID: " << id
            << ", Parent: " << parent_id << ")" << std::endl;

  // 打印关节
  for (const auto &joint : joints) {
    std::cout << indent << "  Joint: " << joint.name << " (Type: " << joint.type
              << ")" << std::endl;
  }

  // 打印几何体
  for (const auto &geom : geoms) {
    std::cout << indent << "  Geom: " << geom.name << " (Type: " << geom.type
              << ")" << std::endl;
  }

  // 递归打印子节点
  for (const auto &child : children) {
    child->printTree(depth + 1);
  }
}

// 转换为MJCF格式
std::string BodyNode::toMJCF(int depth) const {
  std::string indent(depth * 2, ' ');
  std::stringstream ss;

  // 开始body标签
  ss << indent << "<body";
  if (!name.empty() && name != "world") {
    ss << " name=\"" << name << "\"";
  }

  // 添加位置
  if (pos[0] != 0.0 || pos[1] != 0.0 || pos[2] != 0.0) {
    ss << " pos=\"" << pos[0] << " " << pos[1] << " " << pos[2] << "\"";
  }

  // 添加姿态 (如果不是单位四元数)
  if (quat[0] != 1.0 || quat[1] != 0.0 || quat[2] != 0.0 || quat[3] != 0.0) {
    ss << " quat=\"" << quat[0] << " " << quat[1] << " " << quat[2] << " "
       << quat[3] << "\"";
  }

  // 如果没有子节点和内容，可以自闭合
  if (joints.empty() && geoms.empty() && children.empty()) {
    ss << "/>\n";
  } else {
    ss << ">\n";
    // 打印mass and inertial
    if (mass != 0.0) {
      ss << indent << "  <inertial mass=\"" << mass << "\" ";
      ss << "pos=\"" << ipos[0] << " " << ipos[1] << " " << ipos[2] << "\" ";
      ss << "quat=\"" << iquat[0] << " " << iquat[1] << " " << iquat[2] << " "
         << iquat[3] << "\" ";
      ss << "diaginertia=\"" << inertia[0] << " " << inertia[1] << " "
         << inertia[2] << "\"/>\n";
    }
    // 添加关节
    for (const auto &joint : joints) {
      ss << indent << "  <joint";
      if (!joint.name.empty()) {
        ss << " name=\"" << joint.name << "\"";
      }

      // 关节类型
      const char *type_str = "";
      switch (joint.type) {
      case mjJNT_FREE:
        type_str = "free";
        break;
      case mjJNT_BALL:
        type_str = "ball";
        break;
      case mjJNT_SLIDE:
        type_str = "slide";
        break;
      case mjJNT_HINGE:
        type_str = "hinge";
        break;
      default:
        type_str = "hinge";
      }
      ss << " type=\"" << type_str << "\"";

      // 关节轴
      if (joint.type == mjJNT_HINGE || joint.type == mjJNT_SLIDE) {
        ss << " axis=\"" << joint.axis[0] << " " << joint.axis[1] << " "
           << joint.axis[2] << "\"";
      }

      // 关节位置
      if (joint.pos[0] != 0.0 || joint.pos[1] != 0.0 || joint.pos[2] != 0.0) {
        ss << " pos=\"" << joint.pos[0] << " " << joint.pos[1] << " "
           << joint.pos[2] << "\"";
      }

      // 限制
      if (joint.limited) {
        ss << " range=\"" << joint.range[0] << " " << joint.range[1] << "\"";
      }

      ss << "/>\n";
    }

    // 添加几何体
    for (const auto &geom : geoms) {
      ss << indent << "  <geom";
      if (!geom.name.empty()) {
        ss << " name=\"" << geom.name << "\"";
      }

      // 几何体类型
      const char *type_str = "";
      switch (geom.type) {
      case mjGEOM_PLANE:
        type_str = "plane";
        break;
      case mjGEOM_BOX:
        type_str = "box";
        break;
      case mjGEOM_SPHERE:
        type_str = "sphere";
        break;
      case mjGEOM_CAPSULE:
        type_str = "capsule";
        break;
      case mjGEOM_ELLIPSOID:
        type_str = "ellipsoid";
        break;
      case mjGEOM_CYLINDER:
        type_str = "cylinder";
        break;
      default:
        type_str = "box";
      }
      ss << " type=\"" << type_str << "\"";

      // 尺寸
      if (geom.type == mjGEOM_BOX) {
        ss << " size=\"" << geom.size[0] << " " << geom.size[1] << " "
           << geom.size[2] << "\"";
      } else if (geom.type == mjGEOM_SPHERE) {
        ss << " size=\"" << geom.size[0] << "\"";
      } else if (geom.type == mjGEOM_CAPSULE || geom.type == mjGEOM_CYLINDER) {
        ss << " size=\"" << geom.size[0] << " " << geom.size[1] << "\"";
      }

      // 位置
      if (geom.pos[0] != 0.0 || geom.pos[1] != 0.0 || geom.pos[2] != 0.0) {
        ss << " pos=\"" << geom.pos[0] << " " << geom.pos[1] << " "
           << geom.pos[2] << "\"";
      }

      // 颜色
      if (geom.rgba[0] != 1.0 || geom.rgba[1] != 1.0 || geom.rgba[2] != 1.0 ||
          geom.rgba[3] != 1.0) {
        ss << " rgba=\"" << geom.rgba[0] << " " << geom.rgba[1] << " "
           << geom.rgba[2] << " " << geom.rgba[3] << "\"";
      }

      ss << "/>\n";
    }

    // 递归添加子节点
    for (const auto &child : children) {
      ss << child->toMJCF(depth + 1);
    }

    // 结束body标签
    ss << indent << "</body>\n";
  }

  return ss.str();
}
std::shared_ptr<BodyNode>
ModelTreeBuilder::buildTreeFromModel(const mjModel *m) {
  // 创建所有 body 节点
  std::vector<std::shared_ptr<BodyNode>> nodes(m->nbody);

  // 创建根节点 (世界体)
  //   auto root = std::make_shared<BodyNode>(-1, "world", -2);

  // 创建所有 body 节点
  for (int i = 0; i < m->nbody; i++) {
    const char *name = mj_id2name(m, mjOBJ_BODY, i);
    std::string body_name = name ? name : ("body_" + std::to_string(i));

    auto node = std::make_shared<BodyNode>(i, body_name, m->body_parentid[i]);

    // 设置位置和姿态
    node->pos[0] = m->body_pos[3 * i];
    node->pos[1] = m->body_pos[3 * i + 1];
    node->pos[2] = m->body_pos[3 * i + 2];

    node->quat[0] = m->body_quat[4 * i];
    node->quat[1] = m->body_quat[4 * i + 1];
    node->quat[2] = m->body_quat[4 * i + 2];
    node->quat[3] = m->body_quat[4 * i + 3];

    nodes[i] = node;
  }

  // 构建父子关系
  int root_id = 0;
  for (int i = 0; i < m->nbody; i++) {
    int parent_id = m->body_parentid[i];
    // parent_id == i对应根节点，同时防止循环递归
    if (parent_id == -1 || parent_id == i) {
      // 直接连接到世界体
      root_id = i;
      //   root->addChild(nodes[i]);
    } else if (parent_id >= 0 && parent_id < m->nbody) {
      // 连接到父身体
      nodes[parent_id]->addChild(nodes[i]);
    }
  }

  // 添加关节到对应的身体
  for (int i = 0; i < m->njnt; i++) {
    int body_id = m->jnt_bodyid[i];
    if (body_id >= 0 && body_id < m->nbody) {
      const char *name = mj_id2name(m, mjOBJ_JOINT, i);
      std::string joint_name = name ? name : ("joint_" + std::to_string(i));

      JointInfo joint(i, joint_name, m->jnt_type[i]);

      // 设置关节属性
      joint.dof_adr = m->jnt_dofadr[i];
      joint.qpos_adr = m->jnt_qposadr[i];

      // 关节轴
      joint.axis[0] = m->jnt_axis[3 * i];
      joint.axis[1] = m->jnt_axis[3 * i + 1];
      joint.axis[2] = m->jnt_axis[3 * i + 2];

      // 关节位置
      if (m->jnt_pos) {
        joint.pos[0] = m->jnt_pos[3 * i];
        joint.pos[1] = m->jnt_pos[3 * i + 1];
        joint.pos[2] = m->jnt_pos[3 * i + 2];
      }

      // 关节限制
      if (m->jnt_limited) {
        joint.limited = m->jnt_limited[i];
        if (joint.limited) {
          joint.range[0] = m->jnt_range[2 * i];
          joint.range[1] = m->jnt_range[2 * i + 1];
        }
      }

      nodes[body_id]->addJoint(joint);
    }
  }
  // mass and inertial
  for (int i = 0; i < m->nbody; i++) {
    nodes[i]->mass = m->body_mass[i];
    nodes[i]->inertia[0] = m->body_inertia[3 * i];
    nodes[i]->inertia[1] = m->body_inertia[3 * i + 1];
    nodes[i]->inertia[2] = m->body_inertia[3 * i + 2];
    nodes[i]->ipos[0] = m->body_ipos[3 * i];
    nodes[i]->ipos[1] = m->body_ipos[3 * i + 1];
    nodes[i]->ipos[2] = m->body_ipos[3 * i + 2];
    nodes[i]->iquat[0] = m->body_iquat[4 * i];
    nodes[i]->iquat[1] = m->body_iquat[4 * i + 1];
    nodes[i]->iquat[2] = m->body_iquat[4 * i + 2];
    nodes[i]->iquat[3] = m->body_iquat[4 * i + 3];
  }
  // 添加几何体到对应的身体
  for (int i = 0; i < m->ngeom; i++) {
    int body_id = m->geom_bodyid[i];
    if (body_id >= 0 && body_id < m->nbody) {
      const char *name = mj_id2name(m, mjOBJ_GEOM, i);
      std::string geom_name = name ? name : ("geom_" + std::to_string(i));

      GeomInfo geom(i, geom_name, m->geom_type[i]);

      // 设置几何体属性
      geom.size[0] = m->geom_size[3 * i];
      geom.size[1] = m->geom_size[3 * i + 1];
      geom.size[2] = m->geom_size[3 * i + 2];

      geom.pos[0] = m->geom_pos[3 * i];
      geom.pos[1] = m->geom_pos[3 * i + 1];
      geom.pos[2] = m->geom_pos[3 * i + 2];

      geom.quat[0] = m->geom_quat[4 * i];
      geom.quat[1] = m->geom_quat[4 * i + 1];
      geom.quat[2] = m->geom_quat[4 * i + 2];
      geom.quat[3] = m->geom_quat[4 * i + 3];

      geom.rgba[0] = m->geom_rgba[4 * i];
      geom.rgba[1] = m->geom_rgba[4 * i + 1];
      geom.rgba[2] = m->geom_rgba[4 * i + 2];
      geom.rgba[3] = m->geom_rgba[4 * i + 3];

      nodes[body_id]->addGeom(geom);
    }
  }

  return nodes[root_id];
}

// 生成完整的MJCF
std::string ModelTreeBuilder::generateMJCF(const mjModel *m) {
  auto root = buildTreeFromModel(m);

  std::stringstream ss;
  ss << "<mujoco>\n";
  ss << "  <worldbody>\n";

  // 添加所有直接连接到世界体的body
  for (const auto &child : root->children) {
    ss << child->toMJCF(2);
  }

  ss << "  </worldbody>\n";

  // 添加执行器
  if (m->nu > 0) {
    ss << "  <actuator>\n";
    for (int i = 0; i < m->nu; i++) {
      const char *name = mj_id2name(m, mjOBJ_ACTUATOR, i);
      if (name) {
        int trnid = m->actuator_trnid[2 * i];
        if (trnid >= 0) {
          const char *joint_name = mj_id2name(m, mjOBJ_JOINT, trnid);
          if (joint_name) {
            ss << "    <motor name=\"" << name << "\" joint=\"" << joint_name
               << "\"/>\n";
          }
        }
      }
    }
    ss << "  </actuator>\n";
  }

  ss << "</mujoco>\n";

  return ss.str();
}

// 获取身体到关节的映射
std::unordered_map<int, std::vector<int>>
ModelTreeBuilder::getBodyToJointsMap(const mjModel *m) {
  std::unordered_map<int, std::vector<int>> body_joints_map;

  for (int i = 0; i < m->njnt; i++) {
    int body_id = m->jnt_bodyid[i];
    if (body_id >= 0) {
      body_joints_map[body_id].push_back(i);
    }
  }

  return body_joints_map;
}

// 获取身体树层次结构
std::vector<std::pair<int, int>>
ModelTreeBuilder::getBodyHierarchy(const mjModel *m) {
  std::vector<std::pair<int, int>> hierarchy;

  for (int i = 0; i < m->nbody; i++) {
    hierarchy.emplace_back(i, m->body_parentid[i]);
  }

  return hierarchy;
}

// 打印模型结构
void ModelTreeBuilder::printModelStructure(const mjModel *m) {
  std::cout << "=== Model Structure ===" << std::endl;
  std::cout << "Total bodies: " << m->nbody << std::endl;
  std::cout << "Total joints: " << m->njnt << std::endl;
  std::cout << "Total geoms: " << m->ngeom << std::endl;
  std::cout << "Total actuators: " << m->nu << std::endl;

  // 打印身体层次结构
  std::cout << "\n=== Body Hierarchy ===" << std::endl;
  for (int i = 0; i < m->nbody; i++) {
    const char *name = mj_id2name(m, mjOBJ_BODY, i);
    std::cout << "Body " << i << ": " << (name ? name : "unnamed")
              << ", Parent: " << m->body_parentid[i] << std::endl;
  }

  // 打印关节信息
  std::cout << "\n=== Joints ===" << std::endl;
  for (int i = 0; i < m->njnt; i++) {
    const char *name = mj_id2name(m, mjOBJ_JOINT, i);
    int body_id = m->jnt_bodyid[i];
    const char *body_name = mj_id2name(m, mjOBJ_BODY, body_id);

    std::cout << "Joint " << i << ": " << (name ? name : "unnamed")
              << ", Body: " << (body_name ? body_name : "unnamed") << " ("
              << body_id << ")"
              << ", Type: " << m->jnt_type[i] << std::endl;
  }
}
}; // namespace mujoco::plugin::dataserver