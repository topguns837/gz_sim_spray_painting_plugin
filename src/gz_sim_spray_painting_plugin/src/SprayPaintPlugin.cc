#include "gz_sim_spray_painting_plugin/SprayPaintPlugin.hh"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

// gz-sim
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/ParticleEmitter.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/RaycastData.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/Util.hh>

// gz-math
#include <gz/math/Quaternion.hh>

// sdf
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Material.hh>
#include <sdf/ParticleEmitter.hh>
#include <sdf/Visual.hh>

// gz-common
#include <gz/common/Console.hh>

// gz-plugin
#include <gz/plugin/Register.hh>

// gz-msgs (particle emitter proto + color helper)
#include <gz/msgs/particle_emitter.pb.h>
#include <gz/msgs/convert/Color.hh>

namespace gz::sim::systems
{

SprayPaintPlugin::SprayPaintPlugin() = default;

// Logging helpers

/**
 * @brief Returns the current wall-clock time as a formatted string.
 *
 * Format: HH:MM:SS.mmm - used as a prefix in log lines produced by Log().
 *
 * @return Formatted timestamp string.
 */
std::string SprayPaintPlugin::Timestamp() const
{
  using namespace std::chrono;
  const auto now    = system_clock::now();
  const auto now_t  = system_clock::to_time_t(now);
  const auto ms     = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

  std::ostringstream ts;
  ts << std::put_time(std::localtime(&now_t), "%H:%M:%S");
  ts << '.' << std::setfill('0') << std::setw(3) << ms.count();
  return ts.str();
}

/**
 * @brief Emits a structured log line to the Gazebo message stream.
 *
 * Each line is formatted as:
 *   [HH:MM:SS.mmm] [LEVEL  ] [step          ] message
 *
 * @param level  Severity label (e.g. "INFO", "WARN").
 * @param step   Short tag identifying the plugin stage (e.g. "Configure").
 * @param msg    Human-readable message body.
 */
void SprayPaintPlugin::Log(const std::string &level,
                           const std::string &step,
                           const std::string &msg)
{
  std::ostringstream line;
  line << '[' << Timestamp() << ']'
       << " [" << std::left << std::setw(7) << level << ']'
       << " [" << std::left << std::setw(14) << step << "] "
       << msg << '\n';

  const std::string out = line.str();
  gzmsg << out;
}

/**
 * @brief Convenience overload that logs at INFO level.
 *
 * @param step  Short tag identifying the plugin stage.
 * @param msg   Human-readable message body.
 */
void SprayPaintPlugin::Log(const std::string &step, const std::string &msg)
{
  Log("INFO", step, msg);
}

/**
 * @brief Emits a visual section header to the Gazebo message stream.
 *
 * Prints a 60-character '=' bar above and below the title, making it easy
 * to find major lifecycle transitions in a scrolling log.
 *
 * @param title  Label to display between the separator bars.
 */
void SprayPaintPlugin::LogSection(const std::string &title)
{
  const std::string bar(60, '=');
  const std::string entry = "\n" + bar + "\n  " + title + "\n" + bar + "\n";
  gzmsg << entry;
}

// MakePatch

/**
 * @brief Constructs a PaintPatch descriptor for a single raycast hit.
 *
 * The patch is a thin disc whose radius matches the cone footprint at the
 * hit distance.  The disc is offset slightly along the surface normal so it
 * sits proud of the geometry and avoids z-fighting.
 *
 * @param _hitWorld    Hit point in world coordinates.
 * @param _normalWorld Outward surface normal at the hit point (world frame).
 * @param _dist        Distance from the nozzle origin to the hit point (m).
 * @return             A PaintPatch with worldPose, size, and valid=true set.
 */
SprayPaintPlugin::PaintPatch SprayPaintPlugin::MakePatch(
    const gz::math::Vector3d &_hitWorld,
    const gz::math::Vector3d &_normalWorld,
    double _dist) const
{
  PaintPatch result;

  // Patch radius = actual cone footprint at this distance.
  // Minimum 2 cm so very-close hits are still visible.
  constexpr double kThickness = 0.003;
  constexpr double kMinRadius = 0.02;
  const double coneRadius = std::max(_dist * std::tan(coneHalfAngle_), kMinRadius);
  result.size = gz::math::Vector3d(coneRadius * 2.0, coneRadius * 2.0, kThickness);

  const gz::math::Vector3d centre = _hitWorld + _normalWorld * (kThickness * 0.5);

  const gz::math::Vector3d zAxis(0.0, 0.0, 1.0);
  const double dot = std::clamp(zAxis.Dot(_normalWorld), -1.0, 1.0);

  gz::math::Quaterniond rot;
  if (dot > 1.0 - 1e-6)
    rot = gz::math::Quaterniond::Identity;
  else if (dot < -(1.0 - 1e-6))
    rot = gz::math::Quaterniond(M_PI, 0.0, 0.0);
  else
  {
    const gz::math::Vector3d axis = zAxis.Cross(_normalWorld).Normalized();
    rot = gz::math::Quaterniond(axis, std::acos(dot));
  }

  result.worldPose = gz::math::Pose3d(centre, rot);
  result.valid = true;
  return result;
}

// GenerateConeRays

/**
 * @brief Generates ray origin-endpoint pairs spanning the spray cone.
 *
 * Rays are expressed in nozzle-local frame (+X is the spray axis).  The
 * first ray is always the centre axis.  Remaining rays are distributed
 * across the cone solid angle using Fibonacci/sunflower disk sampling,
 * which produces uniform coverage with no grid bias.
 *
 * @return Vector of (origin, endpoint) pairs, each in nozzle-local frame.
 */
std::vector<std::pair<gz::math::Vector3d, gz::math::Vector3d>>
SprayPaintPlugin::GenerateConeRays() const
{
  using Vec3 = gz::math::Vector3d;
  std::vector<std::pair<Vec3, Vec3>> rays;
  const Vec3 origin(0.0, 0.0, 0.0);

  // Center ray is always first.
  rays.emplace_back(origin, Vec3(coneMaxRange_, 0.0, 0.0));

  if (numRays_ <= 1) return rays;

  // Remaining rays use Fibonacci / sunflower disk sampling so they cover the
  // cone footprint uniformly with no grid bias.  All end-points lie on a disk
  // of radius coneMaxRange_*tan(halfAngle) at x = coneMaxRange_, giving ray
  // directions that span exactly the full cone solid angle.
  const double diskRadius  = coneMaxRange_ * std::tan(coneHalfAngle_);
  const double goldenAngle = M_PI * (3.0 - std::sqrt(5.0));  // ≈ 2.3999 rad

  for (int i = 1; i < numRays_; ++i)
  {
    const double r     = std::sqrt(static_cast<double>(i) / (numRays_ - 1)) * diskRadius;
    const double theta = i * goldenAngle;
    rays.emplace_back(origin,
        Vec3(coneMaxRange_, r * std::cos(theta), r * std::sin(theta)));
  }
  return rays;
}

// FindHitLink

/**
 * @brief Finds the nearest non-robot link to a world-space hit point.
 *
 * Uses a two-pass search:
 *  1. Collision entity centres (accurate for multi-link / offset models).
 *  2. Link entity origins as a fallback for geometry types (e.g. PLANE)
 *     whose Collision components are absent from the ECM.
 *
 * Links belonging to the robot's own model are excluded via ownLinks_.
 *
 * @param hitWorld  Hit point in world coordinates.
 * @param _ecm      Reference to the Entity Component Manager.
 * @return          Entity ID of the nearest paintable link, or kNullEntity.
 */
gz::sim::Entity SprayPaintPlugin::FindHitLink(
    const gz::math::Vector3d &hitWorld,
    EntityComponentManager &_ecm) const
{
  gz::sim::Entity bestLink = gz::sim::kNullEntity;
  double minDist = std::numeric_limits<double>::max();

  // Primary pass: use Collision entity world-pose centres.
  // This is accurate for multi-link / offset-collision models (e.g. a car
  // where the chassis collision sits at the car body, not the model origin).
  _ecm.Each<gz::sim::components::Collision>(
      [&](const gz::sim::Entity &colEnt,
          const gz::sim::components::Collision *) -> bool
      {
        const auto *parentComp =
            _ecm.Component<gz::sim::components::ParentEntity>(colEnt);
        if (!parentComp) return true;
        const gz::sim::Entity linkEnt = parentComp->Data();
        if (ownLinks_.count(linkEnt)) return true;

        const double dist =
            gz::sim::worldPose(colEnt, _ecm).Pos().Distance(hitWorld);
        if (dist < minDist)
        {
          minDist  = dist;
          bestLink = linkEnt;
        }
        return true;
      });

  if (bestLink != gz::sim::kNullEntity)
    return bestLink;

  // Fallback pass: use Link entity origin proximity.
  // This covers geometry types (e.g. PLANE) whose Collision entities are
  // not registered in the ECM as gz::sim::components::Collision.
  _ecm.Each<gz::sim::components::Link>(
      [&](const gz::sim::Entity &linkEnt,
          const gz::sim::components::Link *) -> bool
      {
        if (ownLinks_.count(linkEnt)) return true;
        const double dist =
            gz::sim::worldPose(linkEnt, _ecm).Pos().Distance(hitWorld);
        if (dist < minDist)
        {
          minDist  = dist;
          bestLink = linkEnt;
        }
        return true;
      });

  return bestLink;
}

// Configure

/**
 * @brief ISystemConfigure callback - reads SDF parameters and subscribes to
 *        the spray trigger topic.
 *
 * Called once by Gazebo when the plugin is loaded.  All SDF parameters are
 * optional; missing elements keep their default-initialised values.
 *
 * @param _entity   Entity this plugin is attached to (unused).
 * @param _sdf      SDF element containing plugin parameters.
 * @param _ecm      Entity Component Manager (unused at configure time).
 * @param _eventMgr Event Manager - cached for use in PreUpdate.
 */
void SprayPaintPlugin::Configure(
    const Entity & /*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  // 1. Read SDF parameters
  if (_sdf->HasElement("nozzle_link"))
    nozzleLink_ = _sdf->Get<std::string>("nozzle_link");

  if (_sdf->HasElement("cone_half_angle_deg"))
    coneHalfAngle_ = _sdf->Get<double>("cone_half_angle_deg") * M_PI / 180.0;

  if (_sdf->HasElement("cone_max_range"))
    coneMaxRange_ = _sdf->Get<double>("cone_max_range");

  if (_sdf->HasElement("spray_color"))
    sprayColor_ = _sdf->Get<gz::math::Color>("spray_color");

  if (_sdf->HasElement("spray_topic"))
    sprayTopic_ = _sdf->Get<std::string>("spray_topic");

  if (_sdf->HasElement("particle_rate"))
    particleRate_ = _sdf->Get<double>("particle_rate");

  if (_sdf->HasElement("patch_spacing"))
    patchSpacing_ = _sdf->Get<double>("patch_spacing");

  if (_sdf->HasElement("paint_interval_steps"))
    paintIntervalSteps_ = static_cast<uint32_t>(_sdf->Get<int>("paint_interval_steps"));

  if (_sdf->HasElement("num_rays"))
    numRays_ = std::max(1, _sdf->Get<int>("num_rays"));

  if (_sdf->HasElement("enable_particle_emitter"))
    enableParticleEmitter_ = _sdf->Get<bool>("enable_particle_emitter");

  // 2. Cache EventManager pointer
  eventMgr_ = &_eventMgr;

  // 3. Subscribe to trigger topic
  transportNode_.Subscribe(sprayTopic_, &SprayPaintPlugin::OnSprayMsg, this);

  // 4. Log startup banner
  LogSection("SprayPaintPlugin  –  Configure");
  Log("Configure", "nozzle_link", nozzleLink_);
  Log("Configure", "half_angle",
      std::to_string(coneHalfAngle_ * 180.0 / M_PI) + " deg");
  Log("Configure", "max_range",   std::to_string(coneMaxRange_) + " m");
  Log("Configure", "spray_color",
      "R=" + std::to_string(sprayColor_.R()) +
      " G=" + std::to_string(sprayColor_.G()) +
      " B=" + std::to_string(sprayColor_.B()));
  Log("Configure", "topic",          sprayTopic_);
  Log("Configure", "particle_rate",     std::to_string(particleRate_) + " /s");
  Log("Configure", "patch_spacing",     std::to_string(patchSpacing_) + " m");
  Log("Configure", "paint_interval",    std::to_string(paintIntervalSteps_) + " steps");
  Log("Configure", "num_rays",          std::to_string(numRays_) + " cone rays per scan");
  Log("Configure", "particle_emitter",  enableParticleEmitter_ ? "enabled" : "disabled");
  Log("Configure", "status",            "Plugin ready – waiting for nozzle entity");
}

// OnSprayMsg

/**
 * @brief Transport callback fired when a message arrives on the spray topic.
 *
 * Atomically updates sprayActive_ and logs the state transition.  Resets
 * the one-shot diagnostic flag so the next spray-ON edge re-dumps nozzle
 * state.
 *
 * @param _msg  Boolean message: true = spray ON, false = spray OFF.
 */
void SprayPaintPlugin::OnSprayMsg(const gz::msgs::Boolean &_msg)
{
  const bool newState = _msg.data();
  sprayActive_.store(newState);

  LogSection(std::string("Spray ") + (newState ? "ON" : "OFF"));
  Log("OnSprayMsg", "trigger",     newState ? "ACTIVE" : "INACTIVE");
  Log("OnSprayMsg", "patch_count",
      std::to_string(patchCenters_.size()) + " links have patches so far");

  if (newState)
  {
    debugDumped_ = false;
    Log("OnSprayMsg", "debug_dump", "Will log nozzle state on next PreUpdate");
  }
}

// PreUpdate

/**
 * @brief ISystemPreUpdate callback - core per-step logic.
 *
 * Executed every simulation step before physics.  Responsibilities:
 *  - Resolve the nozzle link entity on first appearance (STEP 2).
 *  - Manage the particle emitter lifecycle: create, reposition, remove
 *    (STEP 3 / 3b / 3c) - gated by enableParticleEmitter_.
 *  - Rate-limit ray scans to every paintIntervalSteps_ steps (STEP 6).
 *  - Read RaycastData results and deposit PaintPatch visuals on hit
 *    surfaces, with per-link deduplication (STEP 7-9).
 *
 * @param _info  Simulation update info (timestep, paused state, etc.).
 * @param _ecm   Entity Component Manager for querying and creating entities.
 */
void SprayPaintPlugin::PreUpdate(
    const UpdateInfo & /*_info*/,
    EntityComponentManager &_ecm)
{
  // STEP 1: Nozzle validity check
  if (nozzleEntity_ != kNullEntity && !_ecm.HasEntity(nozzleEntity_))
  {
    LogSection("PreUpdate – Nozzle Lost");
    Log("WARN", "PreUpdate",
        "nozzle entity " + std::to_string(nozzleEntity_) +
        " gone from ECM – re-resolving");
    nozzleEntity_     = kNullEntity;
    robotModelEntity_ = kNullEntity;
    raysAttached_     = false;
    ownLinks_.clear();
    patchCenters_.clear();
  }

  // STEP 2: Nozzle resolution
  if (nozzleEntity_ == kNullEntity)
  {
    _ecm.Each<components::Link, components::Name>(
        [&](const Entity &entity,
            const components::Link *,
            const components::Name *name) -> bool
        {
          if (name->Data() == nozzleLink_)
          {
            nozzleEntity_ = entity;
            return false;
          }
          return true;
        });

    if (nozzleEntity_ == kNullEntity)
      return;

    // Walk up to find parent model.
    {
      Entity e = nozzleEntity_;
      while (e != kNullEntity)
      {
        if (_ecm.Component<components::Model>(e))
        {
          robotModelEntity_ = e;
          break;
        }
        const auto *p = _ecm.Component<components::ParentEntity>(e);
        e = p ? p->Data() : kNullEntity;
      }
    }

    // Collect own-robot links to exclude from nearest-link search.
    if (robotModelEntity_ != kNullEntity)
    {
      _ecm.Each<components::Link>(
          [&](const Entity &lkEnt, const components::Link *) -> bool
          {
            Entity e = lkEnt;
            while (e != kNullEntity)
            {
              if (e == robotModelEntity_)
              {
                ownLinks_.insert(lkEnt);
                break;
              }
              const auto *p = _ecm.Component<components::ParentEntity>(e);
              e = p ? p->Data() : kNullEntity;
            }
            return true;
          });
    }

    LogSection("PreUpdate – Nozzle Resolved");
    Log("PreUpdate", "nozzle_entity",
        "Link '" + nozzleLink_ + "' -> entity " + std::to_string(nozzleEntity_));
    Log("PreUpdate", "own_links",
        std::to_string(ownLinks_.size()) + " own-robot links excluded");

    const gz::math::Pose3d p = gz::sim::worldPose(nozzleEntity_, _ecm);
    std::ostringstream ps;
    ps << "pos=(" << p.Pos().X() << ", " << p.Pos().Y() << ", " << p.Pos().Z()
       << ")  spray_axis_+X=(" << p.Rot().XAxis().X()
       << ", " << p.Rot().XAxis().Y() << ", " << p.Rot().XAxis().Z() << ")";
    Log("PreUpdate", "nozzle_world_pose", ps.str());

    // Attach RaycastData component
    // numRays_ rays spanning the cone solid angle (Fibonacci disk sampling).
    // Rays are in nozzle-local frame; the physics system transforms them by
    // the nozzle world pose each step, so they follow the moving nozzle.
    gz::sim::components::RaycastDataInfo rayData;
    for (const auto &ray : GenerateConeRays())
      rayData.rays.push_back({ray.first, ray.second});

    _ecm.CreateComponent(nozzleEntity_,
        gz::sim::components::RaycastData(rayData));
    raysAttached_ = true;

    Log("PreUpdate", "raycast",
        std::to_string(numRays_) + " cone rays attached to nozzle entity "
        + std::to_string(nozzleEntity_)
        + "  half_angle=" + std::to_string(coneHalfAngle_ * 180.0 / M_PI) + " deg"
        + "  max_range=" + std::to_string(coneMaxRange_) + " m");

    Log("PreUpdate", "emitter",
        "nozzle ready – emitter will be created on first spray-ON trigger"
        "  range=" + std::to_string(coneMaxRange_) + " m" +
        "  half_angle=" + std::to_string(coneHalfAngle_ * 180.0 / M_PI) + " deg");
  }

  // STEP 3 / 3b / 3c: Particle emitter (skipped when disabled in SDF)
  if (enableParticleEmitter_)
  {
    // STEP 3: Particle emitter toggle
    if (emitterEntity_ != kNullEntity)
    {
      const bool active = sprayActive_.load();
      if (active != lastEmitterState_)
      {
        if (!active)
        {
          // Hard stop: remove the emitter entity entirely so the rendering
          // side immediately clears all particles.  It is recreated on the
          // next spray-ON edge (see STEP 3b below).
          _ecm.RequestRemoveEntity(emitterEntity_);
          emitterEntity_ = kNullEntity;
          Log("PreUpdate", "emitter", "OFF – entity removed");
        }
        else
        {
          // Spray turned ON but emitterEntity_ was just cleared; it will be
          // recreated in STEP 3b below this block.
          Log("PreUpdate", "emitter", "ON – recreating emitter");
        }
        lastEmitterState_ = active;
      }
    }

    // STEP 3b: Recreate emitter when spray is ON but entity was removed
    if (sprayActive_ && emitterEntity_ == kNullEntity &&
        nozzleEntity_ != kNullEntity)
    {
      constexpr double kSprayVelocity = 2.0;
      const double effectiveRange =
          coneMaxRange_ / (1.0 + std::tan(coneHalfAngle_));
      const double kLifetime      = effectiveRange / kSprayVelocity;
      const double coneRadiusAtMax = effectiveRange * std::tan(coneHalfAngle_);
      constexpr double kInitSize  = 0.001;
      const double scaleRate =
          std::max((2.0 * coneRadiusAtMax - kInitSize) / kLifetime, 0.01);

      sdf::ParticleEmitter emitterSdf;
      emitterSdf.SetName("spray_emitter_" + std::to_string(++emitterCounter_));
      emitterSdf.SetType(sdf::ParticleEmitterType::POINT);
      emitterSdf.SetEmitting(true);
      emitterSdf.SetRate(particleRate_);
      emitterSdf.SetDuration(0.0);
      emitterSdf.SetLifetime(kLifetime);
      emitterSdf.SetMinVelocity(kSprayVelocity * 0.9);
      emitterSdf.SetMaxVelocity(kSprayVelocity * 1.1);
      emitterSdf.SetColorStart(sprayColor_);
      emitterSdf.SetColorEnd(
          gz::math::Color(sprayColor_.R(), sprayColor_.G(),
                          sprayColor_.B(), 0.0f));
      emitterSdf.SetParticleSize(
          gz::math::Vector3d(kInitSize, kInitSize, kInitSize));
      emitterSdf.SetScaleRate(scaleRate);
      emitterSdf.SetSize(gz::math::Vector3d(0.005, 0.005, 0.005));

      // Local pose zero relative to nozzle - SetParent keeps it as-is (LOCAL).
      // STEP 3c re-asserts zero every PreUpdate so any SetParent-induced drift
      // on 2nd+ activations is corrected before PostUpdate renders it.
      emitterSdf.SetRawPose(gz::math::Pose3d(0, 0, 0, 0, 0, 0));

      sdf::Material emitterMat;
      emitterMat.SetAmbient(sprayColor_);
      emitterMat.SetDiffuse(sprayColor_);
      emitterMat.SetEmissive(sprayColor_);
      emitterSdf.SetMaterial(emitterMat);

      gz::sim::SdfEntityCreator creator(_ecm, *eventMgr_);
      emitterEntity_ = creator.CreateEntities(&emitterSdf);
      creator.SetParent(emitterEntity_, nozzleEntity_);

      Log("PreUpdate", "emitter",
          "recreated entity=" + std::to_string(emitterEntity_));
    }

    // STEP 3c: Force emitter local Pose to zero every frame
    // Uses CreateComponent (not raw pointer write) so the ECM marks the Pose
    // as Changed, triggering the renderer to reposition the Ogre2 scene node.
    // World pose = nozzle_world + local(0,0,0) = nozzle_world every cycle.
    if (emitterEntity_ != kNullEntity)
    {
      _ecm.CreateComponent(emitterEntity_,
          gz::sim::components::Pose(gz::math::Pose3d::Zero));
    }
  }  // enableParticleEmitter_

  // STEP 4: Guard : do nothing if spray not active
  if (!sprayActive_)
    return;

  // STEP 5: One-shot diagnostic on spray-ON edge
  if (!debugDumped_)
  {
    debugDumped_ = true;
    LogSection("PreUpdate – Spray ON Diagnostic");
    const gz::math::Pose3d np = gz::sim::worldPose(nozzleEntity_, _ecm);
    std::ostringstream h;
    h << "pos=(" << np.Pos().X() << ", " << np.Pos().Y() << ", " << np.Pos().Z()
      << ")  axis_+X=(" << np.Rot().XAxis().X()
      << ", " << np.Rot().XAxis().Y() << ", " << np.Rot().XAxis().Z()
      << ")  half_angle=" << (coneHalfAngle_ * 180.0 / M_PI) << " deg"
      << "  max_range=" << coneMaxRange_ << " m"
      << "  rays_attached=" << (raysAttached_ ? "yes" : "no");
    Log("Dump", "nozzle", h.str());
    Log("Dump", "patch_count",
        std::to_string(patchCenters_.size()) + " links painted so far");
  }

  // STEP 6: Rate-limit paint scan
  if ((++paintStepCounter_ % paintIntervalSteps_) != 0)
    return;

  // STEP 7: Read physics raycast results
  if (!raysAttached_) return;

  const auto *raycastComp =
      _ecm.Component<gz::sim::components::RaycastData>(nozzleEntity_);
  if (!raycastComp || raycastComp->Data().results.empty())
    return;

  const gz::math::Pose3d nozzlePose = gz::sim::worldPose(nozzleEntity_, _ecm);

  // STEP 8: Build spray material
  sdf::Material sdfMat;
  sdfMat.SetAmbient(sprayColor_);
  sdfMat.SetDiffuse(sprayColor_);
  sdfMat.SetSpecular(gz::math::Color(
      sprayColor_.R() * 0.3f,
      sprayColor_.G() * 0.3f,
      sprayColor_.B() * 0.3f, 1.0f));

  // STEP 9: Create paint patches from raycast hits
  for (const auto &res : raycastComp->Data().results)
  {
    // fraction == 0 → no hit; fraction == 1 → hit at max range (wall of world)
    if (res.fraction <= 0.0 || res.fraction >= 1.0) continue;

    // Require a valid outward normal
    if (res.normal.Length() < 0.5) continue;

    const double dist = res.fraction * coneMaxRange_;
    if (dist < 1e-3) continue;

    // Transform hit point and normal from nozzle-local to world frame.
    const gz::math::Vector3d hitWorld =
        nozzlePose.CoordPositionAdd(res.point);
    const gz::math::Vector3d normWorld =
        nozzlePose.Rot().RotateVector(res.normal);

    // Find the link whose collision shape is nearest to the hit point.
    const Entity patchParent = FindHitLink(hitWorld, _ecm);
    if (patchParent == kNullEntity)
      continue;  // no paintable surface found - skip silently

    const PaintPatch patch = MakePatch(hitWorld, normWorld, dist);
    if (!patch.valid) continue;

    // Dedup in parent-link-local frame.
    const gz::math::Pose3d parentPose  = gz::sim::worldPose(patchParent, _ecm);
    const gz::math::Pose3d localPatchPose = parentPose.Inverse() * patch.worldPose;
    const gz::math::Vector3d newCenter = localPatchPose.Pos();

    auto &centers = patchCenters_[patchParent];
    bool tooClose = false;
    for (const auto &c : centers)
    {
      if ((c - newCenter).Length() < patchSpacing_)
      { tooClose = true; break; }
    }
    if (tooClose) continue;

    // Create the thin disc visual.
    sdf::Cylinder patchCylinder;
    patchCylinder.SetRadius(patch.size.X() / 2.0);
    patchCylinder.SetLength(patch.size.Z());
    sdf::Geometry patchGeom;
    patchGeom.SetType(sdf::GeometryType::CYLINDER);
    patchGeom.SetCylinderShape(patchCylinder);

    const std::string patchName =
        "paint_patch_" + std::to_string(patchParent) +
        "_" + std::to_string(centers.size());

    sdf::Visual patchVisualSdf;
    patchVisualSdf.SetName(patchName);
    patchVisualSdf.SetRawPose(localPatchPose);
    patchVisualSdf.SetGeom(patchGeom);
    patchVisualSdf.SetMaterial(sdfMat);
    patchVisualSdf.SetCastShadows(false);

    gz::sim::SdfEntityCreator creator(_ecm, *eventMgr_);
    const Entity patchEntity = creator.CreateEntities(&patchVisualSdf);
    creator.SetParent(patchEntity, patchParent);

    centers.push_back(newCenter);

    {
      const gz::math::Vector3d euler = patch.worldPose.Rot().Euler();
      std::ostringstream pm;
      // pm << "patch=" << patchName
      //    << "  dist=" << dist << " m"
      //    << "  hit=(" << hitWorld.X() << ", " << hitWorld.Y()
      //    << ", " << hitWorld.Z() << ")"
      //    << "  normal=(" << normWorld.X() << ", " << normWorld.Y()
      //    << ", " << normWorld.Z() << ")"
      //    << "  parent_link=" << patchParent
      //    << "  total=" << centers.size();
      // Log("PreUpdate", "painted", pm.str());
    }
  }
}

}  // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::SprayPaintPlugin,
              gz::sim::System,
              gz::sim::systems::SprayPaintPlugin::ISystemConfigure,
              gz::sim::systems::SprayPaintPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::SprayPaintPlugin,
                    "gz::sim::systems::SprayPaintPlugin")
