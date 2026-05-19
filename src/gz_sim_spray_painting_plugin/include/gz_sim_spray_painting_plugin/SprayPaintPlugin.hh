#pragma once

#include <atomic>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace gz::sim::systems
{

/// \brief Spray paint plugin for Gazebo Sim 8 (Harmonic).
///
/// Uses physics raycasting (gz::sim::components::RaycastData) to detect
/// spray hits on any geometry type, including MESH.  On each active spray
/// tick the plugin:
///   1. Reads raycast results (hit point + normal in nozzle-local frame).
///   2. Transforms them to world frame via the nozzle link pose.
///   3. Finds the nearest non-own link to the hit point and parents a thin
///      coloured disc patch visual to it.
///
/// SDF parameters (all optional, defaults shown):
///   <nozzle_link>          spray_gun_nozzle_link  </nozzle_link>
///   <cone_half_angle_deg>  15                     </cone_half_angle_deg>
///   <cone_max_range>       3.0                    </cone_max_range>
///   <spray_color>          1.0 0.2 0.1 1.0        </spray_color>
///   <spray_topic>          /spray_paint/trigger    </spray_topic>
class SprayPaintPlugin
    : public System
    , public ISystemConfigure
    , public ISystemPreUpdate
{
public:
  SprayPaintPlugin();
  ~SprayPaintPlugin() override = default;

  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &_eventMgr) override;

  void PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm) override;

private:
  void OnSprayMsg(const gz::msgs::Boolean &_msg);

  // ── Config ────────────────────────────────────────────────────────────────
  std::string     nozzleLink_{"nozzle_link"};
  double          coneHalfAngle_{0.2618};    // 15° in radians
  double          coneMaxRange_{3.0};
  gz::math::Color sprayColor_{1.0f, 0.2f, 0.1f, 1.0f};
  std::string     sprayTopic_{"/spray_paint/trigger"};

  double   particleRate_{100.0};      // particles/s  (SDF: <particle_rate>)
  double   patchSpacing_{0.02};       // min patch centre gap in m (SDF: <patch_spacing>)
  uint32_t paintIntervalSteps_{10};   // paint scan every N steps (SDF: <paint_interval_steps>)
  int      numRays_{16};              // cone rays per scan (SDF: <num_rays>)

  // ── Runtime ───────────────────────────────────────────────────────────────
  std::atomic<bool>      sprayActive_{false};
  gz::transport::Node    transportNode_;
  gz::sim::Entity        nozzleEntity_{gz::sim::kNullEntity};
  gz::sim::EventManager *eventMgr_{nullptr};

  gz::sim::Entity        robotModelEntity_{gz::sim::kNullEntity};

  gz::sim::Entity        emitterEntity_{gz::sim::kNullEntity};
  bool                   lastEmitterState_{false};

  // Set once after nozzle entity is found and rays are attached.
  bool raysAttached_{false};

  // Own-robot link entities — excluded from nearest-link search.
  std::unordered_set<gz::sim::Entity> ownLinks_;

  uint64_t paintStepCounter_{0};

  // Per-link list of applied patch centres in link-local frame.
  // Keyed by the nearest link entity at the hit point.
  std::unordered_map<gz::sim::Entity,
                     std::vector<gz::math::Vector3d>> patchCenters_;

  // ── Patch geometry ────────────────────────────────────────────────────────

  struct PaintPatch
  {
    bool valid{false};
    gz::math::Pose3d   worldPose;
    gz::math::Vector3d size;
  };

  /// Build a PaintPatch from a world-frame hit point + outward surface normal.
  PaintPatch MakePatch(const gz::math::Vector3d &_hitWorld,
                       const gz::math::Vector3d &_normalWorld,
                       double _dist) const;

  /// Generate N cone rays in nozzle-local frame: {start, end} pairs.
  /// Center ray is always first; remaining rays use Fibonacci disk sampling.
  std::vector<std::pair<gz::math::Vector3d, gz::math::Vector3d>>
  GenerateConeRays() const;

  /// Return the Link entity whose Collision is nearest to hitWorld,
  /// excluding own-robot links.  More accurate than link-origin proximity.
  gz::sim::Entity FindHitLink(
      const gz::math::Vector3d &hitWorld,
      EntityComponentManager &_ecm) const;

  // ── Debug logging ─────────────────────────────────────────────────────────
  std::ofstream logFile_;
  std::string   logPath_;
  bool          debugDumped_{false};

  std::string Timestamp() const;
  void Log(const std::string &level, const std::string &step,
           const std::string &msg);
  void Log(const std::string &step, const std::string &msg);
  void LogSection(const std::string &title);
};

}  // namespace gz::sim::systems
