// tree_node.h
#ifndef TREE_NODE_H
#define TREE_NODE_H
#include <memory>
#include <mujoco/mujoco.h>
#include <unordered_map>
#include <vector>
namespace mujoco::plugin::dataserver {

// 关节信息结构
struct JointInfo {
  int id;           // 关节ID
  std::string name; // 关节名称
  int type;         // 关节类型 (mjJNT_*)
  int dof_adr;      // 在 qvel 中的地址
  int qpos_adr;     // 在 qpos 中的地址
  double axis[3];   // 关节轴
  double pos[3];    // 关节位置 (相对于身体)
  double limited;   // 是否有限制
  double range[2];  // 限制范围

  // 构造函数
  JointInfo(int id, const std::string &name, int type);
};

// 几何体信息结构
struct GeomInfo {
  int id;           // 几何体ID
  std::string name; // 几何体名称
  int type;         // 几何体类型 (mjGEOM_*)
  double size[3];   // 尺寸
  double pos[3];    // 位置 (相对于身体)
  double quat[4];   // 姿态四元数
  double rgba[4];   // 颜色

  GeomInfo(int id, const std::string &name, int type);
};

// 树节点类
class BodyNode {
public:
  int id;                                          // 身体ID
  std::string name;                                // 身体名称
  int parent_id;                                   // 父身体ID
  double pos[3];                                   // 相对位置
  double quat[4];                                  // 相对姿态
  std::vector<JointInfo> joints;                   // 该身体的关节
  std::vector<GeomInfo> geoms;                     // 该身体的几何体
  std::vector<std::shared_ptr<BodyNode>> children; // 子节点
  double mass{0};                                  // 质量
  double inertia[3]{0, 0, 0};                      // 惯性对角线
  double ipos[3]{0, 0, 0};                         // 惯性位置
  double iquat[4]{1, 0, 0, 0};                     // 惯性四元数
  // 构造函数
  BodyNode(int id, const std::string &name, int parent_id);

  // 添加子节点
  void addChild(std::shared_ptr<BodyNode> child);

  // 添加关节
  void addJoint(const JointInfo &joint);

  // 添加几何体
  void addGeom(const GeomInfo &geom);

  // 打印树结构
  void printTree(int depth = 0) const;

  // 转换为MJCF格式
  std::string toMJCF(int depth = 0) const;
};
class ModelTreeBuilder {
public:
  // 从 mjModel 构建树
  static std::shared_ptr<BodyNode> buildTreeFromModel(const mjModel *m);
  // 生成完整的MJCF
  static std::string generateMJCF(const mjModel *m);

  // 获取身体到关节的映射
  static std::unordered_map<int, std::vector<int>>
  getBodyToJointsMap(const mjModel *m);

  // 获取身体树层次结构
  static std::vector<std::pair<int, int>> getBodyHierarchy(const mjModel *m);

  // 打印模型结构
  static void printModelStructure(const mjModel *m);
};
} // namespace mujoco::plugin::dataserver

#endif // TREE_NODE_H